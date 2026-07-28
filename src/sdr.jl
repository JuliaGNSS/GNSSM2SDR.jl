# The vendor half of GNSSReceiver.jl#107 for the LiteX-M2SDR running the
# gnss-m2sdr gateware.
#
# GNSSReceiver owns the records, the epoch clock and the loop; this file owns
# everything device-specific: draining DMA1 into `CorrelatorDump`s, turning
# `NCOUpdate`s into scheduled CSR commits, and programming a channel on
# acquisition handover.

# The correlator type a dump carries. `EarlyPromptLateCorrelator{M,T}` has
# M = antennas and T = accumulator element type, which is a bare `ComplexF64`
# for one antenna and an `SVector` for more — so it cannot be spelled as one
# parametric alias.
correlator_type(::Val{1}) = Tracking.EarlyPromptLateCorrelator{1,ComplexF64}
correlator_type(::Val{N}) where {N} =
    Tracking.EarlyPromptLateCorrelator{N,SVector{N,ComplexF64}}

mutable struct M2SDRCorrelator{N,C} <: GNSSReceiver.AbstractHardwareCorrelatorSDR
    const csr::LiteXCSR
    const bank::GNSSBank
    const raw::SignalChannel
    const dumps::PipeChannel{GNSSReceiver.CorrelatorDump{C}}
    const ncos::PipeChannel{GNSSReceiver.NCOUpdate}
    const fs::Float64
    const handover_margin::Int
    const dma_device::String
    const codes::Dict{Int,Vector{Int}}
    # The device counter reading that corresponds to host raw-sample count 0.
    # Both streams count the same samples, so the mapping is this one constant
    # (see `_device_sample`).
    device_origin::Int64
    reader::Union{Task,Nothing}
    writer::Union{Task,Nothing}
    running::Bool
end

"""
    M2SDRCorrelator(csr_csv, raw; fs, n_channels, n_ants = 1, kwargs...)

A LiteX-M2SDR with the gnss-m2sdr tracking gateware, as an
`AbstractHardwareCorrelatorSDR`.

`raw` is the device's raw sample stream as a `SignalChannel` — this package does
not own it, because how you get I/Q off the board (SoapySDR, `LiteXM2SDR.jl`'s
shared-memory streamer, …) is independent of the correlator. It must keep
running for the whole session: the tracking bank is a *non-intrusive observer*
on the RX datapath, so it only sees samples while DMA0 is draining. Stop the raw
stream and the correlators stop, silently.

Keywords:

  - `csr_device` / `dma_device` — the litepcie char devices (`/dev/m2sdr0` for
    CSRs, `/dev/m2sdr1` for the DMA1 record stream).
  - `n_ants` — antenna blocks to read per record (≤ 2, the AD9361's 2T2R limit).
  - `handover_margin` — how far ahead of the device's current sample counter an
    acquisition handover is scheduled. Must exceed the CSR write round trip, or
    the commit lands late (visible through `apply_status`).
"""
function M2SDRCorrelator(
    csr_csv::AbstractString,
    raw::SignalChannel;
    fs,
    n_channels::Integer,
    n_ants::Integer = 1,
    csr_device::AbstractString = "/dev/m2sdr0",
    dma_device::AbstractString = "/dev/m2sdr1",
    handover_margin::Integer = 0,
    dump_capacity::Integer = 1 << 16,
    nco_capacity::Integer = 1 << 12,
)
    1 <= n_ants <= N_ANTS_MAX ||
        throw(ArgumentError("n_ants must be 1..$N_ANTS_MAX (the AD9361 is 2T2R)"))
    fs_hz = Float64(ustrip(uconvert(Hz, fs)))
    csr = LiteXCSR(csr_csv; device = csr_device)
    bank = GNSSBank(csr; fs = fs_hz, n_channels)

    device_ants = num_ants(bank)
    device_ants == n_ants || @warn(
        "gateware reports $device_ants antenna block(s) per record but $n_ants " *
        "requested; the host will read $n_ants"
    )

    # Default the handover margin to 10 ms — comfortably more than a CSR write
    # round trip over PCIe, and short enough that the acquisition's code phase
    # has not aged much by the time it commits.
    margin = handover_margin > 0 ? Int(handover_margin) : round(Int, fs_hz / 100)
    C = correlator_type(Val(Int(n_ants)))

    M2SDRCorrelator{Int(n_ants),C}(
        csr,
        bank,
        raw,
        PipeChannel{GNSSReceiver.CorrelatorDump{C}}(dump_capacity),
        PipeChannel{GNSSReceiver.NCOUpdate}(nco_capacity),
        fs_hz,
        margin,
        String(dma_device),
        Dict{Int,Vector{Int}}(),
        Int64(0),
        nothing,
        nothing,
        false,
    )
end

# ── The GNSSReceiver interface ───────────────────────────────────────────────

GNSSReceiver.raw_sample_channel(sdr::M2SDRCorrelator) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::M2SDRCorrelator) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::M2SDRCorrelator) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::M2SDRCorrelator) = length(sdr.bank.channels)

# The gateware's overflow status is a sticky per-channel bitmap, not a count, so
# report the number of channels that overflowed since the last read and clear.
function GNSSReceiver.dropped_dump_count!(sdr::M2SDRCorrelator)
    bits = overflow(sdr.bank)
    bits == 0 && return 0
    clear_overflow!(sdr.bank)
    count_ones(bits)
end

function GNSSReceiver.release_channel!(sdr::M2SDRCorrelator, hw_channel)
    ch = sdr.bank.channels[hw_channel]
    write(sdr.csr, ch.prefix * "control", 0)
    nothing
end

function GNSSReceiver.assign_channel!(
    sdr::M2SDRCorrelator,
    hw_channel,
    prn,
    carrier_doppler,
    code_doppler,
    code_phase,
    valid_at_sample;
    el_sample_spacing,
    signal,
)
    ch = sdr.bank.channels[hw_channel]
    carrier_hz = Float64(ustrip(uconvert(Hz, carrier_doppler)))
    code_doppler_hz = Float64(ustrip(uconvert(Hz, code_doppler)))

    # The code RAM only has to be rewritten when the PRN changes; 1023 CSR
    # writes is far too slow to do on every handover.
    code = get!(sdr.codes, Int(prn)) do
        Int.(gen_code(CA_CODE_LENGTH, signal, prn))
    end
    load_code!(ch, prn, code)

    # `el_sample_spacing` is the Early-to-Late distance in whole input samples,
    # already quantised the way Tracking quantises it. The CSR wants the
    # prompt→Early half of that.
    sample_shift = max(1, round(Int, el_sample_spacing / 2))
    write(sdr.csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, code_doppler_hz))

    # Schedule the handover far enough ahead that the CSR writes land first, and
    # propagate the code phase from the sample it was valid at to the sample it
    # will be committed on.
    target = sample_count(sdr.bank) + sdr.handover_margin
    elapsed = target - _device_sample(sdr, valid_at_sample)
    code_freq = GPS_CA_CHIP_RATE * (1.0 + carrier_hz / GPS_L1_HZ)
    code_phase_at_target = mod(
        Float64(code_phase) + code_freq * elapsed / sdr.fs,
        CA_CODE_LENGTH,
    )

    schedule!(
        ch,
        target;
        carrier_hz,
        code_doppler_hz,
        carrier_phase_cycles = 0.0,
        code_phase_chips = code_phase_at_target,
    )
    status = apply_status(ch)
    status.late && @warn(
        "handover for PRN $prn on channel $hw_channel committed late — increase " *
        "handover_margin (currently $(sdr.handover_margin) samples)"
    )
    nothing
end

# Host raw-sample count → the bank's free-running counter. Both count the same
# samples off the same RX datapath, so they differ by one constant, latched when
# streaming started. A dropped raw buffer would break this; the driver reports
# that as an overrun.
_device_sample(sdr::M2SDRCorrelator, host_sample) = sdr.device_origin + Int64(host_sample)

# ── Session lifecycle ────────────────────────────────────────────────────────

"""
    start!(sdr)

Latch the host↔device sample-counter offset, enable the bank and the epoch
strobe, and spawn the DMA1 reader and the NCO writer.

Call once the raw stream is already flowing: the offset is latched against the
device counter *now*, so it has to be taken when the host's raw sample count is
still zero.
"""
function start!(sdr::M2SDRCorrelator; epoch_period::Integer = 0)
    sdr.running && return sdr
    sdr.device_origin = sample_count(sdr.bank)
    period = epoch_period > 0 ? epoch_period : round(Int, sdr.fs / 1000)
    set_epoch_period!(sdr.bank, period)
    clear_overflow!(sdr.bank)
    enable!(sdr.bank, true)
    sdr.running = true
    sdr.reader = Base.errormonitor(Threads.@spawn _read_dumps!(sdr))
    sdr.writer = Base.errormonitor(Threads.@spawn _write_ncos!(sdr))
    sdr
end

function stop!(sdr::M2SDRCorrelator)
    sdr.running || return sdr
    sdr.running = false
    enable!(sdr.bank, false)
    close(sdr.dumps)
    close(sdr.ncos)
    sdr
end

# Drain DMA1 and push `CorrelatorDump`s. Reads whole DMA buffers: the driver
# only completes a buffer when it is full, so anything smaller just blocks.
function _read_dumps!(sdr::M2SDRCorrelator{N,C}) where {N,C}
    io = open(sdr.dma_device, "r")
    buffer = Vector{UInt8}(undef, 8192)
    records = M2SDRRecord{N}[]
    batch = GNSSReceiver.CorrelatorDump{C}[]
    try
        while sdr.running
            n = readbytes!(io, buffer, length(buffer))
            n == 0 && break
            empty!(records)
            parse_records!(records, @view(buffer[1:n]), Val(N))
            isempty(records) && continue
            empty!(batch)
            for record in records
                push!(batch, _to_dump(record, Val(N)))
            end
            # Never block the device reader on a full ring: dropping here would
            # be silent, so the bank's own sticky overflow status is what the
            # receiver sees (`dropped_dump_count!`).
            Base.n_avail(sdr.dumps) + length(batch) <= sdr.dumps.capacity - 1 || continue
            put!(sdr.dumps, batch)
        end
    finally
        close(io)
    end
end

# Wire record → `CorrelatorDump`. Two conventions have to be honoured here and
# nowhere else: the accumulators go in as `[late, prompt, early]` (Tracking's
# order, since `get_prompt_index` is 2 — E/P/L order inverts the DLL), and the
# strobe's reserved channel id becomes GNSSReceiver's sentinel.
function _to_dump(record::M2SDRRecord{N}, ::Val{N}) where {N}
    accumulators = if N == 1
        SVector{3,ComplexF64}(record.late[1], record.prompt[1], record.early[1])
    else
        SVector{3,SVector{N,ComplexF64}}(
            SVector{N,ComplexF64}(record.late),
            SVector{N,ComplexF64}(record.prompt),
            SVector{N,ComplexF64}(record.early),
        )
    end
    # The spacing argument is placeholder metadata: GNSSReceiver replaces it
    # with the tracked satellite's before the estimator sees it, so a mismatch
    # here cannot mis-normalise `dll_disc`.
    correlator = Tracking.EarlyPromptLateCorrelator(accumulators, 1)
    output = Tracking.CorrelatorOutput(
        correlator,
        Int(record.integrated_samples),
        Int(record.sample_index),
    )
    channel = is_strobe(record) ? GNSSReceiver.EPOCH_STROBE_CHANNEL :
              Int32(record.channel + 1)   # gateware is 0-based, the host 1-based
    GNSSReceiver.CorrelatorDump(channel, Int32(record.prn), output)
end

# Turn NCO updates into scheduled CSR commits at their named sample.
function _write_ncos!(sdr::M2SDRCorrelator)
    while sdr.running
        update = try
            take!(sdr.ncos)
        catch e
            e isa InvalidStateException && break
            rethrow(e)
        end
        ch = sdr.bank.channels[update.channel]
        schedule!(
            ch,
            update.apply_at_sample;
            carrier_hz = update.carrier_doppler,
            code_doppler_hz = update.code_doppler,
        )
    end
end
