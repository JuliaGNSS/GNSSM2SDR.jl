# ─────────────────────────────────────────────────────────────────────────────
# The LiteX-M2SDR as a `HardwareLoopCore.AbstractLoopDriver`.
#
# What the loop core asks of a device, answered with the gnss-m2sdr gateware's
# means: records are drained straight off DMA1 (`read_records!`, woken by
# `poll(2)` in `wait_records`), a word is the two immediate `carrier_freq` /
# `code_freq` CSR writes (`write_word!`), an arm is the code load plus a
# sample-exact scheduled handover whose commit is verified on the passes that
# follow (`arm!` / `assignment_start`), and the sample counter is the bank's
# own. Nothing here allocates per record or per word: the register names a hot
# path writes are built once per channel, the record buffers are sized once, and
# the code chips are filled into one scratch vector at arm time.
# ─────────────────────────────────────────────────────────────────────────────

"Antenna blocks read per record. The 2T2R second block is not driven yet."
const LOOP_ANTS = 1

"How many handover commits are attempted before the arm is reported failed."
const MAX_HANDOVER_ATTEMPTS = 3

# One hardware channel as the driver keeps it.
mutable struct LoopChannel
    const bank::GNSSBankChannel{LiteXCSR}
    # The two registers a word writes, resolved once.
    const carrier_name::String
    const code_name::String
    const control_name::String
    active::Bool
    failed::Bool
    signal_slot::Int                # index into the driver's signal tuple, 0 free
    prn::Int
    loaded_key::Tuple{Int,Int}      # what the code RAM holds: (signal slot, PRN)
    # The scheduled handover awaiting its commit.
    pending::Bool
    target::Int64
    attempt::Int
    carrier_hz::Float64
    code_doppler_hz::Float64
    code_phase::Float64
    valid_at_sample::Int64
    assignment_start::Int64
    # The words the channel runs, so an unchanged word costs no CSR writes.
    last_carrier_word::Int64
    last_code_word::Int64
end

LoopChannel(bank::GNSSBankChannel{LiteXCSR}, index::Integer) = LoopChannel(
    bank,
    "gnss_ch$(index)_carrier_freq",
    "gnss_ch$(index)_code_freq",
    "gnss_ch$(index)_control",
    false, false, 0, 0, (0, 0),
    false, typemax(Int64), 0, 0.0, 0.0, 0.0, Int64(0), typemax(Int64),
    Int64(-1), Int64(-1),
)

"""
    M2SDRDriver(csr, signals; fs, n_channels = :detect, handover_margin, dma_device,
                open_dma = true, caps = gateware_capabilities(csr))

The LiteX-M2SDR behind the loop core's driver API. `signals` is the tuple of
signal objects the loop may be asked to arm (fixed, so a trimmed binary knows
every code table it needs); `fs` the sample rate in Hz; `handover_margin` how
far ahead of the sample counter an arm is committed (default 10 ms, well past a
CSR round trip). `open_dma = false` leaves DMA1 closed — a driver over a shadow
[`LiteXCSR`](@ref) for the tests.
"""
mutable struct M2SDRDriver{S<:Tuple} <: AbstractLoopDriver
    const csr::LiteXCSR
    const bank::GNSSBank{LiteXCSR}
    const signals::S
    const channels::Vector{LoopChannel}
    const fs::Float64
    const handover_margin::Int
    # Build limits, off the capability CSRs.
    const build_taps::Int
    const tap_layouts::Vector{Int}
    const code_frac_bits::Int
    const max_code_length::Int
    const max_subchips::Int
    const modulations::Int
    const bands::Vector{BandEntry}
    # DMA1.
    const stream::Union{Nothing,DMAWriterStream}
    const fd::Cint
    const buf::Vector{UInt8}
    filled::Int
    const raw_records::Vector{M2SDRRecord{LOOP_ANTS}}
    const last_sidx::Vector{Int64}
    # Arm-time scratch.
    const code_chips::Vector{Int}
    const tap_scratch::Vector{Int}
    # Counters.
    reads::Int64
    records_seen::Int64
    duplicates::Int64
    words_written::Int64
end

function M2SDRDriver(
    csr::LiteXCSR,
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}};
    fs,
    n_channels::Union{Integer,Symbol} = :detect,
    handover_margin::Integer = 0,
    dma_device::AbstractString = "/dev/m2sdr1",
    open_dma::Bool = true,
    caps = gateware_capabilities(csr),
    read_buffer_bytes::Integer = 1 << 20,
)
    fs_hz = _hz(fs)
    resolved = n_channels === :detect ? detect_num_channels(csr) : Int(n_channels)
    bank = GNSSBank(
        csr;
        fs = fs_hz,
        n_channels = resolved,
        code_frac_bits = caps.code_frac_bits,
        carrier_phase_bits = caps.carrier_phase_bits,
        num_taps = caps.num_taps,
        max_subchips = max(1, caps.max_subchips),
        replica_bits = max(2, caps.replica_bits),
    )
    margin = handover_margin > 0 ? Int(handover_margin) : round(Int, fs_hz / 100)
    stream = open_dma ? DMAWriterStream(dma_device; buffers = 16) : nothing
    fd = isnothing(stream) ? Cint(-1) : stream.fd
    raw = M2SDRRecord{LOOP_ANTS}[]
    sizehint!(raw, read_buffer_bytes ÷ RECORD_BYTES + 1)
    code_chips = Int[]
    sizehint!(code_chips, max(1, caps.max_code_length))
    M2SDRDriver(
        csr,
        bank,
        signals,
        [LoopChannel(bank.channels[i], i - 1) for i = 1:resolved],
        fs_hz,
        margin,
        Int(caps.num_taps),
        collect(Int, caps.tap_layouts),
        Int(caps.code_frac_bits),
        Int(caps.max_code_length),
        max(1, Int(caps.max_subchips)),
        Int(caps.modulations),
        [BandEntry(get_band_id(get_band(first(signals))), fs_hz)],
        stream,
        fd,
        Vector{UInt8}(undef, read_buffer_bytes),
        0,
        raw,
        fill(typemin(Int64), resolved + 1),
        code_chips,
        zeros(Int, TAPS_VEPL),
        0, 0, 0, 0,
    )
end

# ── Session ──────────────────────────────────────────────────────────────────

"""
    start_device!(driver; epoch_period)

Program the epoch strobe period (samples), clear the overflow flags and enable
the bank. The raw stream (DMA0) must already be draining: the bank only counts
samples while it is.
"""
function start_device!(dev::M2SDRDriver; epoch_period::Integer)
    set_epoch_period!(dev.bank, epoch_period)
    clear_overflow!(dev.bank)
    enable!(dev.bank, true)
    dev
end

"Disable the bank, release every channel and close DMA1."
function stop_device!(dev::M2SDRDriver)
    for ch = 1:length(dev.channels)
        dev.channels[ch].active && release!(dev, ch)
    end
    enable!(dev.bank, false)
    isnothing(dev.stream) || close(dev.stream)
    dev
end

# ── The driver API ────────────────────────────────────────────────────────────

driver_capabilities(dev::M2SDRDriver) =
    DriverCapabilities(length(dev.channels), dev.build_taps, LOOP_ANTS, dev.bands)

sample_count(dev::M2SDRDriver, ::Integer) = sample_count(dev.bank)

function wait_records(dev::M2SDRDriver, timeout_ms::Integer)
    dev.fd < 0 && return nothing
    _poll_readable(dev.fd, timeout_ms)
    nothing
end

function overflowed_channels!(dev::M2SDRDriver)
    bits = overflow(dev.bank)
    bits == 0 && return 0
    clear_overflow!(dev.bank)
    count_ones(bits)
end

# Drain everything DMA1 has completed, parse it and hand the new records over.
function read_records!(dev::M2SDRDriver, records::Vector{DeviceRecord})
    dev.fd < 0 && return 0
    taken = 0
    while true
        n = _read_into!(dev.fd, dev.buf, dev.filled)
        n <= 0 && break
        dev.reads += 1
        dev.filled += n
        empty!(dev.raw_records)
        dev.filled = _take_records!(dev.raw_records, dev.buf, dev.filled, Val(LOOP_ANTS))
        @inbounds for i in eachindex(dev.raw_records)
            taken += _push_record!(dev, records, dev.raw_records[i])
        end
    end
    taken
end

# The record's taps in the core's order — latest first, `[late, prompt, early]`
# for three and `[very_late, late, prompt, early, very_early]` for five — as
# the fixed tuple a `DeviceRecord` carries.
@inline function _record_taps(r::M2SDRRecord{LOOP_ANTS}, num_taps::Int)
    z = complex(0.0, 0.0)
    if num_taps >= TAPS_VEPL
        (r.very_late[1], r.late[1], r.prompt[1], r.early[1], r.very_early[1], z, z, z, z, z)
    else
        (r.late[1], r.prompt[1], r.early[1], z, z, z, z, z, z, z)
    end
end

function _push_record!(dev::M2SDRDriver, records::Vector{DeviceRecord}, r::M2SDRRecord{LOOP_ANTS})
    dev.records_seen += 1
    strobe = is_strobe(r)
    slot = strobe ? length(dev.last_sidx) : Int(r.channel) + 1
    1 <= slot <= length(dev.last_sidx) || return 0
    # The ring occasionally re-delivers a whole buffer: genuine records are
    # strictly increasing in sample index per channel, so anything not newer
    # is a duplicate. Folding it twice would double-step the loop.
    @inbounds if r.sample_index <= dev.last_sidx[slot]
        dev.duplicates += 1
        return 0
    end
    @inbounds dev.last_sidx[slot] = r.sample_index
    if strobe
        push!(records, strobe_record(r.sample_index))
        return 1
    end
    versioned = r.version >= RECORD_FORMAT_VERSION
    num_taps = versioned ? Int(r.num_taps) : TAPS_EPL
    code_phase = versioned ? Int(r.code_phase_chip) + r.code_phase / (1 << dev.code_frac_bits) : NaN
    push!(
        records,
        DeviceRecord(
            slot,
            Int(r.prn),
            r.sample_index,
            Int(r.integrated_samples),
            _record_taps(r, num_taps),
            num_taps;
            band = 1,
            num_ants = LOOP_ANTS,
            code_phase,
        ),
    )
    1
end

# The code NCO step for a chip rate, or 0 when the NCO cannot represent it.
@inline function _code_step(dev::M2SDRDriver, chip_rate_hz::Float64)
    word = round(Int64, chip_rate_hz / dev.fs * (Int64(1) << dev.code_frac_bits))
    0 < word < (Int64(1) << dev.code_frac_bits) ? word : Int64(0)
end

function write_word!(dev::M2SDRDriver, channel::Integer, carrier_hz::Float64, code_hz::Float64)
    ch = dev.channels[channel]
    ch.active || return false
    bank_ch = ch.bank
    step = _code_step(dev, bank_ch.signal.code_frequency + code_hz)
    step == 0 && return false
    cw = carrier_word(carrier_hz, dev.fs, bank_ch.carrier_phase_bits)
    # An unchanged word is a no-op on the device; the CSR writes are what a
    # pass's time goes on.
    (cw == ch.last_carrier_word && step == ch.last_code_word) && return true
    write(dev.csr, ch.carrier_name, cw)
    write(dev.csr, ch.code_name, step)
    ch.last_carrier_word = cw
    ch.last_code_word = step
    dev.words_written += 1
    true
end

# Which slot of the signal tuple holds `signal`'s type, or 0.
_signal_slot(::Tuple{}, signal, k::Int) = 0
_signal_slot(signals::Tuple, signal, k::Int) =
    typeof(first(signals)) === typeof(signal) ? k : _signal_slot(Base.tail(signals), signal, k + 1)

# The gateware's capability bit for a modulation family, or 0.
function _modulation_bit(modulation::Symbol)
    for (bit, name) in MODULATION_BITS
        name === modulation && return bit
    end
    0
end

# The primary chips as the code RAM takes them, from the primary table (not
# `get_code`, which multiplies in overlay chip 0 and would invert half the
# BeiDou and Galileo pilot replicas).
function _fill_code!(chips::Vector{Int}, signal::AbstractGNSSSignal, prn::Int)
    n = Int(get_code_length(signal))
    resize!(chips, n)
    @inbounds for chip = 0:(n-1)
        chips[chip+1] = GNSSSignals.get_code_at_index(signal, chip, prn) > 0 ? 1 : 0
    end
    chips
end

function arm!(dev::M2SDRDriver, channel::Integer, spec::ArmSpec)
    1 <= channel <= length(dev.channels) ||
        return arm_rejected(HardwareLoopProtocol.REJECT_NO_SUCH_CHANNEL)
    slot = _signal_slot(dev.signals, spec.signal, 1)
    slot == 0 && return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    spec.band == 1 || return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    n = spec.num_taps
    (n in dev.tap_layouts && n <= dev.build_taps) ||
        return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    signal = spec.signal
    cs = ChannelSignal(
        signal,
        spec.prn;
        code_amplitude = spec.code_amplitude,
        replica_amplitude = spec.replica_amplitude,
    )
    cs.code_length <= dev.max_code_length ||
        return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    (dev.modulations & _modulation_bit(cs.modulation)) != 0 ||
        return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    cs.subchips <= dev.max_subchips ||
        return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    step = _code_step(dev, cs.code_frequency + spec.code_doppler_hz)
    step == 0 && return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    # The taps reach chip index ±1 only.
    for i = 1:n
        abs(Int64(spec.tap_sample_shifts[i]) * step) < (Int64(1) << dev.code_frac_bits) ||
            return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    end
    ch = dev.channels[channel]
    bank_ch = ch.bank
    # Nothing this channel reports is believed until the handover commits.
    ch.active = false
    ch.pending = false
    ch.failed = false
    ch.assignment_start = typemax(Int64)
    bank_ch.signal = cs
    # The code RAM is rewritten only when the satellite changes: one CSR write
    # per chip, and the ioctl storm has been seen to wedge the board's MSI.
    if ch.loaded_key != (slot, spec.prn)
        _fill_code!(dev.code_chips, signal, spec.prn)
        shape = replica_shape(signal)
        load_code!(
            bank_ch,
            spec.prn,
            dev.code_chips;
            select = subcarrier_select_bits(shape, length(dev.code_chips)),
        )
        load_replica_shape!(bank_ch, shape; num_taps = n)
        ch.loaded_key = (slot, spec.prn)
    end
    shifts = dev.tap_scratch
    resize!(shifts, n)
    for i = 1:n
        shifts[i] = Int(spec.tap_sample_shifts[i])
    end
    set_tap_offsets!(bank_ch, shifts, spec.code_doppler_hz)
    ch.carrier_hz = spec.carrier_doppler_hz
    ch.code_doppler_hz = spec.code_doppler_hz
    ch.code_phase = spec.code_phase_chips
    ch.valid_at_sample = spec.valid_at_sample
    ch.signal_slot = slot
    ch.prn = spec.prn
    ch.active = true
    ch.last_carrier_word = Int64(-1)
    ch.last_code_word = Int64(-1)
    _schedule_handover!(dev, ch, 1)
    ARM_ACCEPTED
end

# Stage the handover `handover_margin` samples ahead, with the code phase
# propagated from the sample it was valid at to the sample it commits on.
function _schedule_handover!(dev::M2SDRDriver, ch::LoopChannel, attempt::Int)
    s = ch.bank.signal
    code_freq = s.code_frequency + ch.code_doppler_hz
    target = sample_count(dev.bank) + dev.handover_margin
    elapsed = target - ch.valid_at_sample
    phase = mod(ch.code_phase + code_freq * elapsed / dev.fs, s.code_length)
    schedule!(
        ch.bank,
        target;
        carrier_hz = ch.carrier_hz,
        code_doppler_hz = ch.code_doppler_hz,
        carrier_phase_cycles = 0.0,
        code_phase_chips = phase,
    )
    ch.pending = true
    ch.target = target
    ch.attempt = attempt
    nothing
end

# The device sample the channel's assignment took effect at; `typemax` while the
# handover is still to be verified, `typemin` once it has failed for good.
# Verification is lazy — on the passes after the target plus half the margin —
# and a commit that landed late is re-scheduled from the same seed, up to
# `MAX_HANDOVER_ATTEMPTS` times.
function assignment_start(dev::M2SDRDriver, channel::Integer)
    ch = dev.channels[channel]
    ch.active || return typemax(Int64)
    ch.failed && return typemin(Int64)
    ch.pending || return ch.assignment_start
    now = sample_count(dev.bank)
    now >= ch.target + dev.handover_margin ÷ 2 || return typemax(Int64)
    status = apply_status(ch.bank)
    if !status.armed && !status.late
        ch.pending = false
        ch.assignment_start = ch.target
        return ch.target
    elseif ch.attempt >= MAX_HANDOVER_ATTEMPTS
        ch.pending = false
        ch.failed = true
        return typemin(Int64)
    end
    _schedule_handover!(dev, ch, ch.attempt + 1)
    typemax(Int64)
end

function release!(dev::M2SDRDriver, channel::Integer)
    ch = dev.channels[channel]
    write(dev.csr, ch.control_name, 0)
    ch.active = false
    ch.pending = false
    ch.failed = false
    ch.assignment_start = typemax(Int64)
    nothing
end
