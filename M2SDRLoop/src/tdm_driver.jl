# ─────────────────────────────────────────────────────────────────────────────
# The TDM correlator as a `HardwareLoopCore.AbstractLoopDriver`.
#
#   read_records!   DMA1 blocks -> DeviceRecords (and arm confirmations)
#   write_word!     three CSR writes: carrier word, code word, WORD command
#   arm!            code into the code memory (cached by PRN), the staged
#                   assignment with the signal's sub-chip replica, the ARM
#                   command with a start sample ahead of now
#   assignment_start  the first integrated sample, once the start has landed
#   sample_count    the engine's sample counter, extended to 64 bits
#
# Nothing on the per-record or per-word path allocates.
# ─────────────────────────────────────────────────────────────────────────────

"""
How many times an arm whose start sample passed before it landed is re-scheduled.
"""
const MAX_ARM_ATTEMPTS = 3

const CH_FREE = 0x00
const CH_PENDING = 0x01
const CH_RUNNING = 0x02
const CH_FAILED = 0x03

mutable struct TDMChannel
    state::UInt8
    signal_slot::Int
    prn::Int
    assign_id::UInt16
    code_base::Int
    code_length::Int
    nominal_code_hz::Float64
    lead::Int
    warmup::Int
    offsets::NTuple{MAX_TAPS,Int}
    num_taps::Int
    antenna::Int
    # The arm, kept for a re-schedule.
    carrier_hz::Float64
    code_doppler_hz::Float64
    code_phase::Float64
    valid_at::Int64
    start::Int64
    attempt::Int
    assignment_start::Int64
    # The words in force.
    carrier_word::UInt32
    code_word::Int64
    # The replica (`s_subcarrier`, `s_amplitude`).
    subcarrier::UInt32
    amplitude::UInt32
end

TDMChannel() = TDMChannel(
    CH_FREE,
    0,
    0,
    0x0000,
    0,
    0,
    0.0,
    0,
    0,
    ntuple(_ -> 0, Val(MAX_TAPS)),
    0,
    0,
    0.0,
    0.0,
    0.0,
    0,
    0,
    0,
    typemax(Int64),
    0x00000000,
    0,
    SUBCARRIER_BPSK,
    AMPLITUDE_UNIT,
)

"""
    TDMDriver(io, signals; fs, n_channels = 0, antenna = 1, arm_lead_samples,
              strobe_period, flush_us = 100, dma_buffer_size = 512)

The TDM correlator behind the loop core's driver API. `io` reaches the gateware
(`LitePCIeTDM` or `TDMEmulator`); `signals` is the tuple of signals the loop may
be asked to arm (fixed, so a trimmed binary knows every code table it needs);
`fs` is the sample rate; `n_channels = 0` drives every channel the gateware
has. `antenna` is the RF input (1 = RX1, 2 = RX2) the band
table declares and the noise reference uses; satellite arms pick theirs.
`arm_lead_samples` is how far ahead of the sample counter an arm's start is
scheduled (default 2 ms); `strobe_period` the epoch-strobe period (default
`fs / 4000`, 250 µs), which with the `flush_us` buffer flush bounds how long a
record waits before the loop sees it.

Each signal is replicated as GNSSSignals' Int8 code table has it
([`subchip_replica`](@ref)): ±1 for GPS L1 C/A, ±25/±13 for Galileo E1B's
CBOC. An arm of a signal the gateware cannot replicate (TMBOC, or anything but
BPSK on a revision-1 gateware) is rejected as unsupported.
"""
mutable struct TDMDriver{S<:Tuple,IO<:TDMIO} <: AbstractLoopDriver
    const io::IO
    const regs::TDMRegs
    const signals::S
    const caps::TDMCapabilities
    # Per signal of `signals`: how the gateware replicates it, and whether
    # this gateware has the sub-chip replica at all (interface revision 2).
    const replicas::Vector{SubchipReplica}
    const subchips::Bool
    const fs::Float64
    const channels::Vector{TDMChannel}
    const bands::Vector{BandEntry}
    const arm_lead_samples::Int
    const strobe_period::Int
    const flush_cycles::Int
    const dma_buffer_size::Int
    # Code memory: fixed-size slots, keyed by (signal slot, PRN).
    const slot_words::Int
    const slot_key::Vector{Tuple{Int,Int}}
    const slot_used::Vector{Int64}
    const code_scratch::Vector{UInt32}
    # Record bytes.
    const buf::Vector{UInt8}
    filled::Int
    # The engine's sample counter, extended past its 32-bit register.
    last_count::Int64
    next_assign_id::UInt16
    any_pending::Bool
    # Counters.
    records_seen::Int64
    strobes_seen::Int64
    stale_records::Int64
    pad_blocks::Int64
    words_written::Int64
    words_skipped::Int64
    arms::Int64
    arm_retries::Int64
    arm_failures::Int64
    code_loads::Int64
end

function TDMDriver(
    io::TDMIO,
    regs::TDMRegs,
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}};
    fs::Real,
    n_channels::Integer = 0,
    antenna::Integer = 1,
    arm_lead_samples::Integer = round(Int, fs / 500),
    strobe_period::Integer = round(Int, fs / 4000),
    flush_us::Real = 100,
    sys_clk_hz::Real = 125e6,
    dma_buffer_size::Integer = 512,
    read_buffer_bytes::Integer = 1 << 16,
)
    id = reg_read(io, regs.id)
    (id == TDM_GNSS_ID || id == TDM_GNSS_ID_V1) || throw(
        ArgumentError(
            "gateware id $(string(id; base = 16)) is not the TDM correlator ($(string(TDM_GNSS_ID; base = 16)) or $(string(TDM_GNSS_ID_V1; base = 16)))",
        ),
    )
    # A revision-2 gateware has the replica registers; a map without them is
    # another build's csr.csv, and would leave every BOC and CBOC arm rejected.
    subchips = id == TDM_GNSS_ID
    (!subchips || (regs.s_subcarrier != 0 && regs.s_amplitude != 0)) || throw(
        ArgumentError(
            "the gateware is interface revision 2, but the CSR map has no s_subcarrier or s_amplitude: a csr.csv from another build",
        ),
    )
    caps = decode_capabilities(reg_read(io, regs.caps))
    n = n_channels == 0 ? caps.num_channels : Int(n_channels)
    1 <= n <= caps.num_channels ||
        throw(ArgumentError("the gateware has $(caps.num_channels) channels, not $n"))
    1 <= antenna <= 2 || throw(ArgumentError("antenna must be 1 (RX1) or 2 (RX2)"))
    dma_buffer_size % RECORD_BYTES == 0 ||
        throw(ArgumentError("the DMA buffer size must be a whole number of records"))
    slot_words = maximum(map(s -> code_words_for(get_code_length(s)), signals))
    n_slots = caps.code_words ÷ slot_words
    n_slots >= n || throw(
        ArgumentError(
            "the code memory holds $n_slots codes of $slot_words words, fewer than the $n channels",
        ),
    )
    band_id = Symbol(get_band_id(get_band(first(signals))))
    TDMDriver(
        io,
        regs,
        signals,
        caps,
        SubchipReplica[map(subchip_replica, signals)...],
        subchips,
        Float64(fs),
        [TDMChannel() for _ = 1:n],
        [BandEntry(band_id, Float64(fs); rf_input = antenna)],
        Int(arm_lead_samples),
        Int(strobe_period),
        round(Int, flush_us * 1e-6 * sys_clk_hz),
        Int(dma_buffer_size),
        slot_words,
        fill((0, 0), n_slots),
        zeros(Int64, n_slots),
        zeros(UInt32, slot_words),
        zeros(UInt8, max(Int(read_buffer_bytes), Int(dma_buffer_size))),
        0,
        Int64(0),
        0x0001,
        false,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
end

TDMDriver(io::LitePCIeTDM, signals::Tuple; kwargs...) =
    TDMDriver(io, TDMRegs(io.csr), signals; dma_buffer_size = io.dma_buffer_size, kwargs...)

_command(ch::Integer, op::Integer) = UInt32(op) | (UInt32(ch - 1) << 8)

"""
    start_device!(driver)

Reset the engine, program the strobe, the flush and the DMA geometry, open the
record stream and enable the engine and the records. The RX stream (DMA0) must
be draining for samples to reach the engine.
"""
function start_device!(d::TDMDriver)
    r = d.regs
    # Keep the tag bit as the receiver set it: it describes the host's raw stream.
    tag = reg_read(d.io, r.control) & CONTROL_TAG
    reg_write(d.io, r.control, CONTROL_RESET | tag)
    reg_write(d.io, r.strobe_period, d.strobe_period)
    reg_write(d.io, r.flush_cycles, d.flush_cycles)
    reg_write(d.io, r.buffer_words, d.dma_buffer_size ÷ 8)
    dma_open!(d.io)
    r.dma1_sync_bypass != 0 && reg_write(d.io, r.dma1_sync_bypass, 1)
    reg_write(d.io, r.control, CONTROL_ENABLE | CONTROL_RECORDS | tag)
    for ch in d.channels
        ch.state = CH_FREE
        ch.assignment_start = typemax(Int64)
    end
    fill!(d.slot_key, (0, 0))
    d.filled = 0
    d.last_count = _read_full_count(d)
    d
end

"""
    stop_device!(driver)

Stop the records and close the stream. The channels keep correlating until
the next `start_device!` resets the engine; the tag bit is left alone.
"""
function stop_device!(d::TDMDriver)
    r = d.regs
    tag = reg_read(d.io, r.control) & CONTROL_TAG
    reg_write(d.io, r.control, CONTROL_ENABLE | tag)
    dma_close!(d.io)
    d
end

# ── Sample counter ───────────────────────────────────────────────────────────

function _read_full_count(d::TDMDriver)
    r = d.regs
    hi1 = reg_read(d.io, r.sample_count_hi)
    lo = reg_read(d.io, r.sample_count)
    hi2 = reg_read(d.io, r.sample_count_hi)
    hi = hi1 == hi2 ? hi1 : (lo < 0x80000000 ? hi2 : hi1)
    (Int64(hi) << 32) | Int64(lo)
end

# The 64-bit count whose low 32 bits are `lo`, nearest the last one seen.
@inline function _extend(d::TDMDriver, lo::UInt32)
    base = d.last_count & ~Int64(0xFFFFFFFF)
    full = base | Int64(lo)
    if full < d.last_count - Int64(0x80000000)
        full += Int64(1) << 32
    elseif full > d.last_count + Int64(0x80000000)
        full -= Int64(1) << 32
    end
    full
end

function sample_count(d::TDMDriver, ::Integer)
    full = _extend(d, reg_read(d.io, d.regs.sample_count))
    full > d.last_count && (d.last_count = full)
    full
end

# ── Capabilities ─────────────────────────────────────────────────────────────

driver_capabilities(d::TDMDriver) =
    DriverCapabilities(length(d.channels), MAX_TAPS, 1, d.bands)

wait_records(d::TDMDriver, timeout_ms::Integer) = (dma_wait(d.io, timeout_ms); nothing)

overflowed_channels!(d::TDMDriver) = 0

assignment_start(d::TDMDriver, channel::Integer) = d.channels[channel].assignment_start

# ── Records ──────────────────────────────────────────────────────────────────

function read_records!(d::TDMDriver, records::Vector{DeviceRecord})
    d.any_pending && _confirm_arms!(d)
    n = dma_read!(d.io, d.buf, d.filled)
    d.filled += n
    whole = d.filled - d.filled % RECORD_BYTES
    count = 0
    offset = 0
    while offset < whole
        rec = decode_record(d.buf, offset)
        offset += RECORD_BYTES
        if rec.kind == KIND_DUMP
            count += _push_dump!(d, records, rec)
        elseif rec.kind == KIND_STROBE
            push!(records, strobe_record(rec.sample_index))
            d.strobes_seen += 1
            rec.sample_index > d.last_count && (d.last_count = rec.sample_index)
            count += 1
        else
            d.pad_blocks += 1
        end
    end
    rest = d.filled - whole
    rest > 0 && copyto!(d.buf, 1, d.buf, whole + 1, rest)
    d.filled = rest
    count
end

function _push_dump!(d::TDMDriver, records::Vector{DeviceRecord}, rec::TDMRecord)
    ch = rec.channel + 1
    if !(1 <= ch <= length(d.channels))
        d.stale_records += 1
        return 0
    end
    c = d.channels[ch]
    if rec.assign_id != c.assign_id || !(c.state == CH_RUNNING || c.state == CH_PENDING)
        d.stale_records += 1
        return 0
    end
    # A dump of the pending assignment proves its start landed after this
    # pass's confirmation read.
    if c.state == CH_PENDING
        c.state = CH_RUNNING
        c.assignment_start = c.start + c.warmup
    end
    d.records_seen += 1
    # The record carries the earliest tap's phase; the prompt is `lead` samples behind.
    earliest = rec.code_chip + rec.code_frac20 / 1048576.0
    prompt = mod(earliest - c.lead * (c.code_word / 4294967296.0), Float64(c.code_length))
    taps = ntuple(Val(MAX_RECORD_TAPS)) do i
        i <= MAX_TAPS ? rec.taps[i] : complex(0.0, 0.0)
    end
    push!(
        records,
        DeviceRecord(
            ch,
            c.prn,
            rec.sample_index,
            rec.integrated,
            taps,
            rec.num_taps;
            code_phase = prompt,
        ),
    )
    1
end

# ── Words ────────────────────────────────────────────────────────────────────

function write_word!(d::TDMDriver, channel::Integer, carrier_hz::Float64, code_hz::Float64)
    c = d.channels[channel]
    (c.state == CH_RUNNING || c.state == CH_PENDING) || return false
    cw = carrier_word(carrier_hz, d.fs)
    kw = code_word(c.nominal_code_hz + code_hz, d.fs)
    0 < kw < Int64(4294967296) || return false
    if cw == c.carrier_word && kw == c.code_word
        d.words_skipped += 1
        return true
    end
    r = d.regs
    reg_write(d.io, r.s_carrier_freq, cw)
    reg_write(d.io, r.s_code_freq, UInt32(kw))
    reg_write(d.io, r.cmd, _command(channel, CMD_WORD))
    c.carrier_word = cw
    c.code_word = kw
    d.words_written += 1
    true
end

# ── Arms ─────────────────────────────────────────────────────────────────────

_signal_slot(::Tuple{}, signal, k::Int) = 0
_signal_slot(signals::Tuple, signal, k::Int) =
    typeof(first(signals)) === typeof(signal) ? k :
    _signal_slot(Base.tail(signals), signal, k + 1)

function arm!(d::TDMDriver, channel::Integer, spec::ArmSpec)
    k = _signal_slot(d.signals, spec.signal, 1)
    k == 0 && return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    replica = d.replicas[k]
    # A BOC or CBOC replica sampled as BPSK correlates to nothing at the prompt
    # (GNSSM2SDR.jl#16): refuse what this gateware cannot replicate.
    (replica.supported && (d.subchips || is_bpsk(replica))) ||
        return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    (spec.band == 1 && 1 <= spec.num_taps <= MAX_TAPS && 1 <= spec.rf_input <= 2) ||
        return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    layout = tap_layout(spec.tap_sample_shifts, spec.num_taps)
    isnothing(layout) && return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    lead, offsets, warmup = layout
    signal = spec.signal
    c = d.channels[channel]
    # A channel being re-armed lets go of its code slot first.
    c.state = CH_FREE
    base = _load_code!(d, k, signal, spec.prn)
    c.signal_slot = k
    c.prn = spec.prn
    c.code_base = base
    c.code_length = get_code_length(signal)
    c.nominal_code_hz = ustrip(Hz, uconvert(Hz, get_code_frequency(signal)))
    c.lead = lead
    c.warmup = warmup
    c.offsets = offsets
    c.num_taps = spec.num_taps
    c.subcarrier = replica.subcarrier
    c.amplitude = replica.amplitude
    c.antenna = spec.rf_input - 1
    c.carrier_hz = spec.carrier_doppler_hz
    c.code_doppler_hz = spec.code_doppler_hz
    c.code_phase = spec.code_phase_chips
    c.valid_at = spec.valid_at_sample
    c.attempt = 0
    d.arms += 1
    _schedule_arm!(d, channel)
    ARM_ACCEPTED
end

function _schedule_arm!(d::TDMDriver, channel::Integer)
    c = d.channels[channel]
    r = d.regs
    now = sample_count(d, 1)
    start = now + d.arm_lead_samples
    rate = c.nominal_code_hz + c.code_doppler_hz
    kw = code_word(rate, d.fs)
    cw = carrier_word(c.carrier_hz, d.fs)
    # The handover's prompt phase carried to the start sample, then `lead` samples on.
    step = kw / 4294967296.0
    earliest = mod(
        c.code_phase + (start - c.valid_at) * (rate / d.fs) + c.lead * step,
        Float64(c.code_length),
    )
    chip = floor(Int, earliest)
    frac = round(Int64, (earliest - chip) * 4294967296.0)
    if frac >= Int64(4294967296)
        frac -= Int64(4294967296)
        chip += 1
    end
    chip >= c.code_length && (chip -= c.code_length)
    c.assign_id = d.next_assign_id
    d.next_assign_id = d.next_assign_id == 0xFFFF ? 0x0001 : d.next_assign_id + 0x0001
    taps = UInt32(0)
    for t = 1:MAX_TAPS
        taps |= UInt32(c.offsets[t]) << (5 * (t - 1))
    end
    taps |= UInt32(c.num_taps) << 25
    misc =
        UInt32(c.lead) | UInt32(c.warmup) << 5 | UInt32(c.antenna) << 10 |
        UInt32(c.assign_id) << 16
    reg_write(d.io, r.s_carrier_freq, cw)
    reg_write(d.io, r.s_code_freq, UInt32(kw))
    reg_write(d.io, r.s_carrier_phase, 0)
    reg_write(d.io, r.s_code_frac, UInt32(frac))
    reg_write(d.io, r.s_code_chip, chip)
    reg_write(d.io, r.s_code_length, c.code_length)
    reg_write(d.io, r.s_code_base, c.code_base)
    reg_write(d.io, r.s_taps, taps)
    reg_write(d.io, r.s_misc, misc)
    if d.subchips
        reg_write(d.io, r.s_subcarrier, c.subcarrier)
        reg_write(d.io, r.s_amplitude, c.amplitude)
    end
    reg_write(d.io, r.s_start_lo, UInt32(start & 0xFFFFFFFF))
    reg_write(d.io, r.s_start_hi, UInt32((start >> 32) & 0xFFFF))
    reg_write(d.io, r.cmd, _command(channel, CMD_ARM))
    c.carrier_word = cw
    c.code_word = kw
    c.start = start
    c.state = CH_PENDING
    c.assignment_start = typemax(Int64)
    d.any_pending = true
    nothing
end

# Once per pass while an arm waits: has its start landed, or was it missed?
function _confirm_arms!(d::TDMDriver)
    r = d.regs
    pending = reg_read(d.io, r.pending)
    missed = reg_read(d.io, r.missed)
    enabled = reg_read(d.io, r.enabled)
    still = false
    for i in eachindex(d.channels)
        c = d.channels[i]
        c.state == CH_PENDING || continue
        bit = UInt32(1) << (i - 1)
        if missed & bit != 0
            if c.attempt + 1 < MAX_ARM_ATTEMPTS
                c.attempt += 1
                d.arm_retries += 1
                _schedule_arm!(d, i)
                still = true
            else
                c.state = CH_FAILED
                c.assignment_start = typemin(Int64)
                d.arm_failures += 1
            end
        elseif pending & bit == 0 && enabled & bit != 0
            c.state = CH_RUNNING
            c.assignment_start = c.start + c.warmup
        else
            still = true
        end
    end
    d.any_pending = still
    nothing
end

function release!(d::TDMDriver, channel::Integer)
    c = d.channels[channel]
    reg_write(d.io, d.regs.cmd, _command(channel, CMD_RELEASE))
    c.state = CH_FREE
    c.assignment_start = typemax(Int64)
    nothing
end

# ── Code memory ──────────────────────────────────────────────────────────────

# The slot holding `(signal slot, prn)`'s primary code, loading it into the
# least recently used slot no running channel reads if it is not there.
function _load_code!(d::TDMDriver, k::Int, signal::AbstractGNSSSignal, prn::Int)
    key = (k, prn)
    stamp = d.arms + 1
    for s in eachindex(d.slot_key)
        if d.slot_key[s] == key
            d.slot_used[s] = stamp
            return (s - 1) * d.slot_words
        end
    end
    victim = 0
    oldest = typemax(Int64)
    for s in eachindex(d.slot_key)
        base = (s - 1) * d.slot_words
        _slot_in_use(d, base) && continue
        if d.slot_used[s] < oldest
            oldest = d.slot_used[s]
            victim = s
        end
    end
    victim == 0 && throw(ErrorException("no free code slot"))
    base = (victim - 1) * d.slot_words
    L = get_code_length(signal)
    replica = d.replicas[k]
    words = d.code_scratch
    fill!(words, 0x00000000)
    for chip = 0:(L-1)
        if chip_negative(signal, replica, prn, chip)
            words[(chip>>5)+1] |= UInt32(1) << (chip & 31)
        end
    end
    r = d.regs
    reg_write(d.io, r.code_addr, base)
    for w = 1:code_words_for(L)
        reg_write(d.io, r.code_data, words[w])
    end
    d.slot_key[victim] = key
    d.slot_used[victim] = stamp
    d.code_loads += 1
    base
end

function _slot_in_use(d::TDMDriver, base::Int)
    for c in d.channels
        (c.state == CH_RUNNING || c.state == CH_PENDING) &&
            c.code_base == base &&
            return true
    end
    false
end
