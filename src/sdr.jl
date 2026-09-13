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

# A scheduled handover awaiting verification: everything needed to re-schedule
# it if the commit lands late.
struct PendingHandover
    prn::Int32
    carrier_hz::Float64
    code_doppler_hz::Float64
    code_phase::Float64
    valid_at_sample::Int64
    target::Int64
    attempt::Int
end

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
    # The `m2sdr_record` draining DMA1 into the service task's pipe, when the
    # record stream is taken that way (see `start!`'s `dump_transport`).
    dump_recorder::Union{Nothing,Base.Process}
    # `dump_source = :csr` only: dumps the poller provably skipped (dump_count
    # advanced by more than one). A skipped dump is a missing bit-buffer prompt,
    # which scrambles that satellite's decoded bit stream — watch this when
    # decoding matters.
    missed_csr_dumps::Int
    # Which channels currently hold a satellite, and its PRN — maintained by
    # `assign_channel!` / `release_channel!`. The CSR poller only services
    # active channels (every channel dumps at ~1 kHz whether assigned or not,
    # and a full dump readout is ~11 CSR ioctls: polling all of them pushes the
    # pass time past the dump period and *guarantees* missed dumps), and it
    # tags dumps with the assigned PRN rather than paying one more ioctl.
    const active::Vector{Bool}
    const assigned_prns::Vector{Int32}
    # Handovers scheduled but not yet verified, per channel — see
    # `verify_handovers!`. `assign_channel!` never waits for its commit: it runs
    # on the receiver's chunk-processing task, and every millisecond it blocks
    # there is a millisecond every *other* channel holds a stale NCO word.
    const pending::Vector{Union{Nothing,PendingHandover}}
    # Published by the verifier and read by the Receiver on another thread.
    const assignment_start::Vector{Threads.Atomic{Int64}}
    const assignment_locks::Vector{ReentrantLock}
end

"""
    M2SDRCorrelator(csr_csv, raw; fs, n_channels = :detect, n_ants = 1, kwargs...)

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
  - `n_channels` — hardware tracking channels to drive. Defaults to counting
    the `gnss_ch<i>_` banks in the CSR map, i.e. whatever the flashed gateware
    actually has ([`detect_num_channels`](@ref)).
  - `n_ants` — antenna blocks to read per record (≤ 2, the AD9361's 2T2R limit).
  - `handover_margin` — how far ahead of the device's current sample counter an
    acquisition handover is scheduled. Must exceed the CSR write round trip, or
    the commit lands late (visible through `apply_status`).
"""
function M2SDRCorrelator(
    csr_csv::AbstractString,
    raw::SignalChannel;
    fs,
    n_channels::Union{Integer,Symbol} = :detect,
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
    resolved_channels = n_channels === :detect ? detect_num_channels(csr) : Int(n_channels)
    bank = GNSSBank(csr; fs = fs_hz, n_channels = resolved_channels)

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
        nothing,
        0,
        fill(false, resolved_channels),
        zeros(Int32, resolved_channels),
        Union{Nothing,PendingHandover}[nothing for _ = 1:resolved_channels],
        [Threads.Atomic{Int64}(typemax(Int64)) for _ = 1:resolved_channels],
        [ReentrantLock() for _ = 1:resolved_channels],
    )
end

# ── The GNSSReceiver interface ───────────────────────────────────────────────

GNSSReceiver.raw_sample_channel(sdr::M2SDRCorrelator) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::M2SDRCorrelator) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::M2SDRCorrelator) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::M2SDRCorrelator) = length(sdr.bank.channels)

# The gateware wipes the carrier off with a ±127 sin/cos ROM where a host
# correlator uses a unit-amplitude replica, so its accumulators are 127× the
# prompt the same samples would give on the CPU (measured on sky: 126.9–128.1
# across six satellites, issue #107). The ingest divides it out so the prompt
# and the noise density Tracking measures from the raw stream share one scale.
GNSSReceiver.correlator_gain(::M2SDRCorrelator) = 127

# The gateware's overflow status is a sticky per-channel bitmap, not a count, so
# report the number of channels that overflowed since the last read and clear.
function GNSSReceiver.dropped_dump_count!(sdr::M2SDRCorrelator)
    bits = overflow(sdr.bank)
    bits == 0 && return 0
    clear_overflow!(sdr.bank)
    count_ones(bits)
end

GNSSReceiver.assignment_start_sample(sdr::M2SDRCorrelator, hw_channel) =
    sdr.assignment_start[hw_channel][]

GNSSReceiver.release_channel!(sdr::M2SDRCorrelator, hw_channel) =
    lock(() -> _release_channel!(sdr, hw_channel), sdr.assignment_locks[hw_channel])

function _release_channel!(sdr::M2SDRCorrelator, hw_channel)
    sdr.assignment_start[hw_channel][] = typemax(Int64)
    sdr.active[hw_channel] = false
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
    lock(sdr.assignment_locks[hw_channel])
    try
        _assign_channel!(sdr, hw_channel, prn, carrier_doppler, code_doppler,
                         code_phase, valid_at_sample; el_sample_spacing, signal)
    finally
        unlock(sdr.assignment_locks[hw_channel])
    end
end

function _assign_channel!(
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
    # Invalidate queued dumps before changing PRN metadata or code RAM.
    sdr.assignment_start[hw_channel][] = typemax(Int64)
    ch = sdr.bank.channels[hw_channel]
    carrier_hz = Float64(ustrip(uconvert(Hz, carrier_doppler)))
    code_doppler_hz = Float64(ustrip(uconvert(Hz, code_doppler)))

    # The code RAM only has to be rewritten when the channel's PRN changes:
    # 1023 back-to-back CSR writes per handover is not just slow, the ioctl
    # storm has been observed to wedge the board's MSI delivery (DMA0 then
    # starves while the sample counter keeps counting). Cache per channel.
    if sdr.assigned_prns[hw_channel] != Int32(prn)
        code = get!(sdr.codes, Int(prn)) do
            Int[get_code(signal, chip, prn) > 0 ? 1 : 0 for chip = 0:(CA_CODE_LENGTH-1)]
        end
        load_code!(ch, prn, code)
    end

    # `el_sample_spacing` is the Early-to-Late distance in whole input samples,
    # already quantised the way Tracking quantises it. The CSR wants the
    # prompt→Early half of that.
    sample_shift = max(1, round(Int, el_sample_spacing / 2))
    write(sdr.csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, code_doppler_hz))

    # Schedule the handover far enough ahead that the CSR writes land first, and
    # propagate the code phase from the sample it was valid at to the sample it
    # will be committed on. The commit is verified — and a late one re-scheduled
    # — by `verify_handovers!` on the NCO writer task, not here: this runs on the
    # receiver's chunk-processing task, and waiting even the 10 ms margin holds
    # every other channel's NCO word for that long (issue #107).
    _schedule_handover!(sdr, hw_channel, Int32(prn), carrier_hz, code_doppler_hz,
                        Float64(code_phase), Int64(valid_at_sample), 1)
    sdr.assigned_prns[hw_channel] = Int32(prn)
    sdr.active[hw_channel] = true
    nothing
end

function _schedule_handover!(sdr, hw_channel, prn, carrier_hz, code_doppler_hz, code_phase,
                             valid_at_sample, attempt)
    ch = sdr.bank.channels[hw_channel]
    code_freq = GPS_CA_CHIP_RATE * (1.0 + carrier_hz / GPS_L1_HZ)
    target = sample_count(sdr.bank) + sdr.handover_margin
    elapsed = target - _device_sample(sdr, valid_at_sample)
    code_phase_at_target = mod(code_phase + code_freq * elapsed / sdr.fs, CA_CODE_LENGTH)
    schedule!(
        ch,
        target;
        carrier_hz,
        code_doppler_hz,
        carrier_phase_cycles = 0.0,
        code_phase_chips = code_phase_at_target,
    )
    sdr.pending[hw_channel] = PendingHandover(prn, carrier_hz, code_doppler_hz, code_phase,
                                              valid_at_sample, target, attempt)
    nothing
end

"""
    verify_handovers!(sdr) -> Int

Check every scheduled handover whose target sample has passed: a commit that
landed on time is cleared; a late one is re-scheduled (up to three attempts)
from the same acquisition estimate; a channel released or re-assigned in the
meantime is simply forgotten. Returns how many handovers were re-scheduled.
Called from the device service task (DMA) or the CSR poller every pass, so the receiver's
processing task never waits on a commit.
"""
function verify_handovers!(sdr::M2SDRCorrelator)
    rescheduled = 0
    for hw_channel in eachindex(sdr.pending)
        l = sdr.assignment_locks[hw_channel]
        # `trylock`, never `lock`: this runs on the device service task, which
        # must not yield. A contended lock means the processing task is inside
        # assign_channel!/release_channel! for this channel right now; waiting
        # for it would hand this thread to the scheduler, and whatever task it
        # picks up — an acquisition chunk, say — keeps it until it is done,
        # while the ring fills and nothing commits NCO words. The check simply
        # runs again on the next buffer, ~0.75 ms later.
        trylock(l) || continue
        try
            isnothing(sdr.pending[hw_channel]) && continue
            rescheduled += _verify_handover!(sdr, hw_channel, sample_count(sdr.bank))
        finally
            unlock(l)
        end
    end
    rescheduled
end

function _verify_handover!(sdr, hw_channel, now)
    h = sdr.pending[hw_channel]
    isnothing(h) && return 0
    now >= h.target + sdr.handover_margin ÷ 2 || return 0
    if !sdr.active[hw_channel] || sdr.assigned_prns[hw_channel] != h.prn
        sdr.pending[hw_channel] = nothing
        return 0
    end
    status = apply_status(sdr.bank.channels[hw_channel])
    if !status.armed && !status.late
        sdr.assignment_start[hw_channel][] = h.target
        sdr.pending[hw_channel] = nothing
    elseif h.attempt >= 3
        @warn "handover failed; channel remains unconfirmed" hw_channel prn=h.prn
        sdr.pending[hw_channel] = nothing
    else
        _schedule_handover!(sdr, hw_channel, h.prn, h.carrier_hz, h.code_doppler_hz,
                            h.code_phase, h.valid_at_sample, h.attempt + 1)
        return 1
    end
    0
end

# Host raw-sample count → the bank's free-running counter. Both count the same
# samples off the same RX datapath, so they differ by one constant, latched when
# streaming started. A dropped raw buffer would break this; the driver reports
# that as an overrun.
_device_sample(sdr::M2SDRCorrelator, host_sample) = sdr.device_origin + Int64(host_sample)

# ── Session lifecycle ────────────────────────────────────────────────────────

"""
    start!(sdr; epoch_period = 0, device_origin = nothing, dump_source = :dma)

Latch the host↔device sample-counter offset, enable the bank and the epoch
strobe, and spawn the device service task (dump reader and NCO writer in one).

Call once the raw stream is already flowing: the offset is latched against the
device counter *now*, so it has to be taken when the host's raw sample count is
still zero. That latch is only as exact as the raw path's in-flight buffering;
pass `device_origin` (e.g. from a code-phase sweep calibration against an
acquisition) to override it with a measured value — required for acquisition
handovers programmed straight from the raw stream's code phases.

`epoch_period` is the timebase-strobe period in samples (`0` ⇒ 1 kHz). Strobes
also pace the DMA1 stream: the litepcie driver only completes whole 8 KiB
buffers (64 records), so the strobe rate bounds the dump latency the tracking
loop sees — at the default 1 kHz an idle bank delivers records ~64 ms late,
far beyond what the loop filters tolerate. Raise it (e.g. `fs ÷ 16000`) when
the correlator drives a live feedback loop over DMA.

`dump_source` picks how dumps reach the receiver: `:dma` drains the DMA1
record ring; `:csr` polls every channel's dump CSRs instead (no DMA1 device
needed, latency of one polling pass, but it can miss dumps under load and
fabricates its own strobes from the sample counter).

`dump_transport` says how the DMA1 ring is drained. `:recorder` (the default
whenever `m2sdr_record` is on the PATH) runs `m2sdr_record` on the DMA1 device
as a separate process writing into a pipe of `dump_pipe_bytes` — about three
seconds of records at the strobe rate a live loop uses — which the service task
reads. Nothing in this process can then keep the driver's ring from being
drained: a GC pause or a compilation only delays the reader, and the pipe
absorbs it, exactly as the raw stream's recorder does for DMA0. `:device` reads
`/dev/m2sdr1` directly, with only the driver's 256-buffer ring (~190 ms at
80 kHz strobes, discarding after half of it) between the gateware and the task.
"""
function start!(
    sdr::M2SDRCorrelator;
    epoch_period::Integer = 0,
    device_origin::Union{Nothing,Integer} = nothing,
    dump_source::Symbol = :dma,
    dump_transport::Symbol = :auto,
    dump_pipe_bytes::Integer = 32 * 2^20,
)
    sdr.running && return sdr
    dump_source in (:dma, :csr) ||
        throw(ArgumentError("dump_source must be :dma or :csr, got $dump_source"))
    dump_transport in (:auto, :recorder, :device) || throw(
        ArgumentError("dump_transport must be :auto, :recorder or :device, got $dump_transport"),
    )
    transport =
        dump_transport === :auto ?
        (isnothing(Sys.which("m2sdr_record")) ? :device : :recorder) : dump_transport
    sdr.device_origin =
        isnothing(device_origin) ? sample_count(sdr.bank) : Int64(device_origin)
    period = epoch_period > 0 ? epoch_period : round(Int, sdr.fs / 1000)
    set_epoch_period!(sdr.bank, period)
    clear_overflow!(sdr.bank)
    enable!(sdr.bank, true)
    sdr.running = true
    # The service tasks live on the interactive pool and each keeps its
    # thread (sticky, parked in the kernel rather than in the scheduler), so
    # the pool needs a thread per task: start Julia with `-t N,M` and M at
    # least the number of real-time tasks in the process — reader, writer,
    # the raw-stream reader and GNSSReceiver's processing task make four. With
    # no interactive threads they fall back to the default pool, where an
    # acquisition scan's chunk tasks can hold every thread for seconds.
    if dump_source === :dma
        # One task services the whole device: it drains DMA1 and, between
        # buffers, commits NCO updates and verifies handovers. See
        # `_service_dma!` for why it is one task and one thread.
        service = Threads.@spawn :interactive _service_dma!(sdr, transport, Int(dump_pipe_bytes))
        sdr.reader = Base.errormonitor(service)
        sdr.writer = nothing
    else
        # One spin loop owns all CSR traffic: the NCO drain runs between dump
        # polls, so commits land within a poll pass of being pushed and never
        # contend with the poller for the ioctl lock.
        sdr.reader = Base.errormonitor(Threads.@spawn :interactive _poll_dumps!(sdr, period))
        sdr.writer = nothing
    end
    sdr
end

function stop!(sdr::M2SDRCorrelator)
    sdr.running || return sdr
    sdr.running = false
    enable!(sdr.bank, false)
    close(sdr.dumps)
    close(sdr.ncos)
    # Ending the recorder ends the service task's stream (EOF on the pipe).
    recorder = sdr.dump_recorder
    if !isnothing(recorder)
        process_running(recorder) && kill(recorder)
        wait(recorder)
        sdr.dump_recorder = nothing
    end
    sdr
end

# Service the device from one task that owns one thread: drain DMA1 into
# `CorrelatorDump`s and, between buffers, commit the NCO updates the fold
# pushed and verify pending handovers.
#
# Why one task. The interactive pool is small and, when interactive threads
# exist, Julia puts the *main* thread in it — so the user's main task, and
# every compilation it triggers, shares the pool with the receiver's real-time
# tasks. Each task that blocks in the kernel or never yields keeps a thread;
# the fewer of them, the fewer interactive threads a host needs before a busy
# main task starts starving the processing task. Reader and writer together
# make one such task, and the record stream's own cadence (a completed buffer
# every ~0.75 ms at the strobe rate the receiver uses) is a fine clock for
# committing NCO words.
#
# Why it waits in the kernel. Julia services its event loop — every `sleep`,
# `Timer` and libuv read — from thread 1 unless that thread is blocked, so any
# `sleep`-based poll here stopped for as long as thread 1 was busy: measured
# on the board, 550 ms per acquisition scan run on the main task, and the
# whole of any compilation the main task did (GNSSReceiver.jl#107). `poll(2)`
# is woken by the driver's interrupt directly and, as a `gc_safe` ccall, never
# holds up a collection. The task is sticky so the scheduler neither migrates
# it nor runs anything else on its thread while it is parked.
#
# Why a recorder process. Even a task that waits in the kernel has to stop
# for a collection once it is back in Julia, and a full collection on the
# post-startup heap takes ~130 ms here — more than the 96 ms the driver's ring
# allows before it discards. `m2sdr_record` draining DMA1 into a 32 MiB pipe
# (about three seconds of records) puts the ring behind a process nothing in
# this one can hold up, the same way the raw stream is taken off DMA0.
#
# In `:device` mode `DMAWriterStream` starts the channel's DMA writer over
# ioctl before the first read. Without that the driver's read path waits on a
# buffer counter the gateware is never told to advance, so the drain blocks
# forever and no dump ever reaches the receiver — see dma.jl.
function _service_dma!(sdr::M2SDRCorrelator{N,C}, transport::Symbol, pipe_bytes::Int) where {N,C}
    current_task().sticky = true
    # Where the bytes come from: the recorder's pipe (blocking fd) or the
    # device itself (non-blocking fd, whole 8 KiB buffers per read). Either way
    # the loop waits in `poll` and reads into the tail of one accumulation
    # buffer; `_take_records!` copes with records cut at a read boundary.
    stream = nothing
    if transport === :recorder
        device_num = parse(Int, match(r"(\d+)$", sdr.dma_device).captures[1])
        recorder, fd = _spawn_into_pipe(`m2sdr_record -c $device_num -q - 0`; pipe_bytes)
        sdr.dump_recorder = recorder
    else
        stream = DMAWriterStream(sdr.dma_device; buffers = 16)
        fd = stream.fd
    end
    # 1 MiB per read at most — 8192 records, a tenth of a second at the live
    # strobe rate — so a backlog after a stall is fetched in a few reads.
    buf = Vector{UInt8}(undef, 1 << 20)
    filled = 0
    records = M2SDRRecord{N}[]
    batch = GNSSReceiver.CorrelatorDump{C}[]
    # The ring occasionally re-delivers a whole buffer, so the same records
    # (identical channel/seq/sample_index) arrive twice. A duplicated dump
    # folded twice double-steps the tracking loops and doubles the prompts per
    # navigation bit — satellites drop within a minute and bit sync never
    # happens. Sample indices of genuine new dumps are strictly increasing per
    # channel (and strobes on their own slot), so drop anything not newer.
    last_sidx = fill(typemin(Int64), length(sdr.bank.channels) + 1)
    try
        while sdr.running
            # NCO commits first: they are deadline-bound (`apply_at` is only
            # `feedback_delay_epochs` ahead), dump readout is not. `poll` bounds
            # the wait so a quiet record stream cannot delay a commit by more
            # than a few milliseconds.
            verify_handovers!(sdr)
            _drain_ncos!(sdr)
            _poll_readable(fd, 5) || continue
            n = _read_into!(fd, buf, filled)
            n == 0 && break          # the recorder is gone
            n < 0 && continue
            filled += n
            empty!(records)
            filled = _take_records!(records, buf, filled, Val(N))
            isempty(records) && continue
            empty!(batch)
            for record in records
                slot = is_strobe(record) ? length(last_sidx) : Int(record.channel) + 1
                if 1 <= slot <= length(last_sidx)
                    record.sample_index <= last_sidx[slot] && continue
                    last_sidx[slot] = record.sample_index
                end
                push!(batch, _to_dump(record, Val(N)))
            end
            # Never block the device reader on a full ring: dropping here would
            # be silent, so the bank's own sticky overflow status is what the
            # receiver sees (`dropped_dump_count!`).
            Base.n_avail(sdr.dumps) + length(batch) <= sdr.dumps.capacity - 1 || continue
            put!(sdr.dumps, batch)
        end
    catch e
        # `stop!` closes the ring while a batch may be in flight; that is the
        # normal end of the stream, not a fault.
        e isa InvalidStateException || rethrow()
    finally
        if isnothing(stream)
            ccall(:close, Cint, (Cint,), fd)
        else
            close(stream)
        end
    end
end

# Wire record → `CorrelatorDump`. Two conventions have to be honoured here and
# nowhere else: the accumulators go in as `[late, prompt, early]` (Tracking's
# order, since `get_prompt_index` is 2 — E/P/L order inverts the DLL), and the
# strobe's reserved channel id becomes GNSSReceiver's sentinel.
#
# The record's `code_phase` is the code NCO's fractional register latched on the
# dump sample. A dump fires on the sample whose advance wraps the last chip, so
# on that sample the replica sits at chip `CA_CODE_LENGTH - 1` plus that
# fraction — the absolute anchor GNSSReceiver's pseudorange bookkeeping wants.
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
    code_phase = is_strobe(record) ? NaN :
                 (CA_CODE_LENGTH - 1) + record.code_phase / (1 << CODE_FRAC_BITS)
    GNSSReceiver.CorrelatorDump(channel, Int32(record.prn), output, code_phase)
end

# CSR-polling dump source: read every channel's dump CSRs whenever its
# `dump_count` moves, and fabricate the timebase strobes from the sample
# counter. No DMA1 device needed and a latency of one polling pass, but unlike
# the DMA ring it can *miss* dumps when the poller falls behind — a missed dump
# is a missing bit-buffer prompt, which scrambles the decoder's bit stream — so
# this is a bring-up/diagnostic source; use `:dma` for decoding and PVT.
# Pin the calling OS thread to one CPU core. The poller's pass must run every
# dump period; any preemption is a burst of missed dumps, and a missed dump is
# a broken subframe. Launch julia under `taskset -c 0-(core-1)` and set
# GNSS_POLLER_CORE=<core>: every other thread (Julia pools, GC, FFTW, Polyester)
# inherits the reduced process mask, and the poller alone re-pins itself onto
# the reserved core — CPU storms elsewhere can then never touch it.
function _pin_current_thread(core::Integer)
    tid = ccall(:gettid, Cint, ())
    mask = zeros(UInt8, 128)
    mask[core ÷ 8 + 1] = UInt8(1) << (core % 8)
    rc = ccall(
        (:sched_setaffinity, "libc"),
        Cint,
        (Cint, Csize_t, Ptr{UInt8}),
        tid,
        length(mask),
        mask,
    )
    rc == 0 || @warn "could not pin the dump poller to core $core (errno $(Libc.errno()))"
    rc == 0
end

function _poll_dumps!(sdr::M2SDRCorrelator{N,C}, strobe_period::Integer) where {N,C}
    poller_core = get(ENV, "GNSS_POLLER_CORE", "")
    if !isempty(poller_core)
        # The task must stop migrating between pool threads before the OS-level
        # pin means anything.
        current_task().sticky = true
        _pin_current_thread(parse(Int, poller_core))
    end
    channels = sdr.bank.channels
    prev_counts = fill(-1, length(channels))
    prototype = _prototype_correlator(Val(N))
    last_strobe = sample_count(sdr.bank)
    last_carrier = fill(NaN, length(channels))
    last_code = fill(NaN, length(channels))
    rotate = 0
    while sdr.running
        # NCO commits first: they are deadline-bound (apply_at is only
        # feedback_delay_epochs ahead), dump readout is not.
        try
            verify_handovers!(sdr)
            _drain_ncos!(sdr, last_carrier, last_code)
        catch e
            e isa InvalidStateException || rethrow(e)
        end
        # Rotate the scan origin every pass: when a pass overruns the dump
        # period, the channels scanned last are the ones that lose dumps, and
        # a fixed order starves the same satellites' bit streams every time.
        rotate = mod1(rotate + 1, length(channels))
        for k in eachindex(channels)
            i = mod1(rotate + k - 1, length(channels))
            ch = channels[i]
            if !sdr.active[i]
                prev_counts[i] = -1
                continue
            end
            count = Int(read(sdr.csr, ch.prefix * "dump_count"))
            count == prev_counts[i] && continue
            dump = _read_dump_csrs(sdr, ch, Val(N))
            isnothing(dump) && continue
            # dump_count is 32-bit and monotonic while the channel runs; a jump
            # of more than one means the poller was outrun and dumps are gone.
            prev_counts[i] >= 0 &&
                (sdr.missed_csr_dumps += mod(dump.count - prev_counts[i] - 1, 1 << 32))
            prev_counts[i] = dump.count
            Base.n_avail(sdr.dumps) < sdr.dumps.capacity - 1 || continue
            put!(
                sdr.dumps,
                GNSSReceiver.CorrelatorDump(
                    Int32(i),
                    sdr.assigned_prns[i],
                    Tracking.CorrelatorOutput(dump.correlator, dump.n, dump.sample_index),
                    dump.code_phase,
                ),
            )
        end
        now = sample_count(sdr.bank)
        if now - last_strobe >= strobe_period
            last_strobe = now
            Base.n_avail(sdr.dumps) < sdr.dumps.capacity - 1 &&
                put!(sdr.dumps, GNSSReceiver.epoch_strobe(prototype, now))
        end
        yield()
    end
end

_prototype_correlator(::Val{1}) =
    Tracking.EarlyPromptLateCorrelator(zero(SVector{3,ComplexF64}), 1)
_prototype_correlator(::Val{N}) where {N} =
    Tracking.EarlyPromptLateCorrelator(zero(SVector{3,SVector{N,ComplexF64}}), 1)

# One coherent CSR dump read: retried until `dump_count` is stable around the
# field reads, so a dump firing mid-read cannot mix two integrations.
function _read_dump_csrs(sdr::M2SDRCorrelator, ch, ::Val{N}; tries::Integer = 10) where {N}
    csr = sdr.csr
    p = ch.prefix
    for _ = 1:tries
        c0 = read(csr, p * "dump_count")
        accumulators = _read_accumulators(csr, p, Val(N))
        n = read(csr, p * "integrated_samples")
        sample_index = read(csr, p * "sample_index")
        frac = read(csr, p * "dump_code_phase")
        if read(csr, p * "dump_count") == c0
            n == 0 && return nothing
            return (
                count = Int(c0),
                n = Int(n),
                sample_index = Int(sample_index),
                code_phase = (CA_CODE_LENGTH - 1) + Int(frac) / (1 << CODE_FRAC_BITS),
                correlator = Tracking.EarlyPromptLateCorrelator(accumulators, 1),
            )
        end
    end
    nothing
end

_acc_suffix(a::Integer) = a == 0 ? "" : "_ant$(a)"

function _read_accumulators(csr, prefix, ::Val{1})
    late = ComplexF64(read_signed(csr, prefix * "il", 32), read_signed(csr, prefix * "ql", 32))
    prompt = ComplexF64(read_signed(csr, prefix * "ip", 32), read_signed(csr, prefix * "qp", 32))
    early = ComplexF64(read_signed(csr, prefix * "ie", 32), read_signed(csr, prefix * "qe", 32))
    SVector{3,ComplexF64}(late, prompt, early)
end

function _read_accumulators(csr, prefix, ::Val{N}) where {N}
    per_ant = ntuple(Val(N)) do ant
        s = _acc_suffix(ant - 1)
        (
            late = ComplexF64(
                read_signed(csr, prefix * "il" * s, 32),
                read_signed(csr, prefix * "ql" * s, 32),
            ),
            prompt = ComplexF64(
                read_signed(csr, prefix * "ip" * s, 32),
                read_signed(csr, prefix * "qp" * s, 32),
            ),
            early = ComplexF64(
                read_signed(csr, prefix * "ie" * s, 32),
                read_signed(csr, prefix * "qe" * s, 32),
            ),
        )
    end
    SVector{3,SVector{N,ComplexF64}}(
        SVector{N,ComplexF64}(map(a -> a.late, per_ant)),
        SVector{N,ComplexF64}(map(a -> a.prompt, per_ant)),
        SVector{N,ComplexF64}(map(a -> a.early, per_ant)),
    )
end

# Turn NCO updates into scheduled CSR commits at their named sample.
#
# Latency here is loop-critical: an update scheduled `feedback_delay_epochs`
# (a few ms) ahead must reach the CSRs before its `apply_at_sample` passes, or
# it commits late with stale values. `PipeChannel`'s blocking single-item
# `take!` parks in a ~10 ms sleep-poll, which batches updates into bursts that
# all land late — the carrier keeps frequency lock but the PLL's phase
# corrections apply at random delays and phase never locks (no data bits).
# Hence the non-blocking batch drain, called from the device service task
# between DMA buffers (`:dma`) or between dump polls (`:csr`).
function _drain_ncos!(
    sdr::M2SDRCorrelator,
    last_carrier::Vector{Float64} = Float64[],
    last_code::Vector{Float64} = Float64[],
)
    n = Base.n_avail(sdr.ncos)
    n == 0 && return 0
    # A correction computed for a sample far in the past describes a loop state
    # that no longer exists: committing it steers the NCO with stale data and
    # throws the loop (observed while the host catches up a start-up backlog —
    # every satellite lost within a minute). Let the channel free-run instead
    # (a handover-seeded NCO drifts ~0.5 Hz/s, harmless for tens of seconds)
    # and resume with the first fresh correction.
    now = sample_count(sdr.bank)
    max_stale = Int64(round(0.02 * sdr.fs))   # 20 ms
    for _ = 1:n
        update = take!(sdr.ncos)
        update.apply_at_sample < now - max_stale && continue
        start = sdr.assignment_start[update.channel][]
        start == typemax(Int64) && continue
        update.prn == sdr.assigned_prns[update.channel] || continue
        update.apply_at_sample < start && continue
        ch = sdr.bank.channels[update.channel]
        # An update that quantizes to the NCO words the channel already runs
        # is a no-op on the device; committing it anyway costs ~6 serialized
        # CSR writes that delay the dump scan (missed dumps = scrambled bits).
        cw = Float64(carrier_word(ch, update.carrier_doppler))
        kw = Float64(code_word_from_code_doppler(ch, update.code_doppler))
        if update.channel <= length(last_carrier) &&
           cw == last_carrier[update.channel] &&
           kw == last_code[update.channel]
            continue
        end
        # Do NOT use the staged commit for streaming frequency updates: the
        # gateware has a single staging register with cancel-on-re-arm
        # semantics, and Tracking's apply_at (fold boundary + 1 epoch) is
        # ~1-2 ms in the future — the next 1 kHz update re-arms and cancels
        # the pending commit before it matures, so NO update ever applied and
        # every channel silently free-ran on its handover words (the
        # walk-off-and-die disease). The immediate CSRs apply on the next
        # sample with no arming; +-2 ms of application jitter is irrelevant
        # at these loop bandwidths. The staged path remains for handovers,
        # which need the sample-exact phase load and are one-shot.
        write(
            sdr.csr,
            ch.prefix * "carrier_freq",
            carrier_word(ch, update.carrier_doppler),
        )
        write(
            sdr.csr,
            ch.prefix * "code_freq",
            code_word_from_code_doppler(ch, update.code_doppler),
        )
        if update.channel <= length(last_carrier)
            last_carrier[update.channel] = cw
            last_code[update.channel] = kw
        end
    end
    n
end

