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
#
# The second argument is how many accumulator *slots* the link's dumps carry,
# which is the **widest** layout the flashed build produces, not the layout of
# any one channel. One stream has one element type, and a five-tap build runs
# GPS L1 C/A on three taps next to Galileo E1 on five: a three-tap record then
# fills the leading three slots and says `num_taps = 3`, and GNSSReceiver reads
# exactly that many (`CorrelatorDump.num_taps`). Sizing the stream at the
# narrower layout instead would make the five-tap channel unrepresentable, and
# padding a three-tap record out to five would hand `dll_disc` two accumulators
# that never saw a replica.
correlator_type(::Val{1}, ::Val{TAPS_EPL}) =
    Tracking.EarlyPromptLateCorrelator{1,ComplexF64}
correlator_type(::Val{N}, ::Val{TAPS_EPL}) where {N} =
    Tracking.EarlyPromptLateCorrelator{N,SVector{N,ComplexF64}}
correlator_type(::Val{1}, ::Val{TAPS_VEPL}) =
    Tracking.VeryEarlyPromptLateCorrelator{1,ComplexF64}
correlator_type(::Val{N}, ::Val{TAPS_VEPL}) where {N} =
    Tracking.VeryEarlyPromptLateCorrelator{N,SVector{N,ComplexF64}}
correlator_type(n::Val) = correlator_type(n, Val(TAPS_EPL))

# How many accumulator slots a link's wire correlator has, recovered from the
# type so the record decoder can be specialised on it without a runtime branch.
wire_taps(::Type{<:Tracking.VeryEarlyPromptLateCorrelator}) = Val(TAPS_VEPL)
wire_taps(::Type{<:Tracking.EarlyPromptLateCorrelator}) = Val(TAPS_EPL)

# A scheduled handover awaiting verification: everything needed to re-schedule
# it if the commit lands late.
#
# `signal_id` alongside `prn` because a channel can be re-assigned from a PRN's
# data component to its pilot, or to the same PRN number in another
# constellation, without the PRN changing: matching on the number alone would
# let a stale handover confirm an assignment it was never scheduled for.
struct PendingHandover
    prn::Int32
    signal_id::Symbol
    carrier_hz::Float64
    code_doppler_hz::Float64
    code_phase::Float64
    valid_at_sample::Int64
    target::Int64
    attempt::Int
    # The length the load staged, so the confirmation can check the gateware
    # committed it (`code_length_active`) rather than assume the restart did.
    code_length::Int
end

mutable struct M2SDRCorrelator{N,C} <: GNSSReceiver.AbstractHardwareCorrelatorSDR
    const csr::LiteXCSR
    const bank::GNSSBank{LiteXCSR}
    const raw::SignalChannel
    const dumps::PipeChannel{GNSSReceiver.CorrelatorDump{C}}
    const ncos::PipeChannel{GNSSReceiver.NCOUpdate}
    const fs::Float64
    const handover_margin::Int
    const dma_device::String
    # Generated primary replicas, keyed by *signal identity and PRN*. GPS L1 C/A
    # PRN 7 and Galileo E1B PRN 7 are different codes of different lengths, and
    # so are a satellite's pilot and data components; a cache keyed on the PRN
    # alone hands the second one the first one's chips.
    const codes::Dict{Tuple{Symbol,Int},Vector{Int}}
    # What the flashed gateware says it can do, read once at construction and
    # handed to GNSSReceiver's pre-arm validation.
    const capabilities::GNSSReceiver.HardwareCorrelatorCapabilities
    const code_frac_bits::Int
    # The widest tap layout the build produces, i.e. how many accumulator slots
    # this link's records carry. Which of them a given channel fills is staged
    # per channel and reported per record.
    const num_taps::Int
    # The sub-chip table's depth. Not part of GNSSReceiver's vendor-neutral
    # profile because no modulation bit can carry it — `:BOCsin` says nothing
    # about whether `BOCsin(1,1)` (2 sub-chips) or `BOCsin(6,1)` (12) fits — so
    # it is checked against the specific signal at arm time instead.
    const max_subchips::Int
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
    # The signal identity each channel currently replicates, next to its PRN:
    # together they are what a code reload, a stale dump and a late handover are
    # all judged by. `:none` until the channel has been assigned anything.
    const assigned_signals::Vector{Symbol}
    # Handovers scheduled but not yet verified, per channel — see
    # `verify_handovers!`. `assign_channel!` never waits for its commit: it runs
    # on the receiver's chunk-processing task, and every millisecond it blocks
    # there is a millisecond every *other* channel holds a stale NCO word.
    const pending::Vector{Union{Nothing,PendingHandover}}
    # Published by the verifier and read by the Receiver on another thread.
    const assignment_start::Vector{Threads.Atomic{Int64}}
    const assignment_locks::Vector{ReentrantLock}
    # NCO words accepted from the receiver but not yet due, one slot per
    # channel (the newest supersedes). `_drain_ncos!` commits each at the first
    # service pass at or after its `apply_at_sample`, so the word lands where
    # the receiver's timeline says it does — up to one pass (~0.75 ms) late,
    # rather than on arrival, ~1 ms early and jittering with the host.
    const nco_pending::Vector{Union{Nothing,GNSSReceiver.NCOUpdate}}
    # Diagnostics: words committed, how late they landed (samples past
    # `apply_at_sample`; sum and max), and words dropped as stale. The loop
    # delay as measured, not inferred; logged by `stop!`.
    nco_commits::Int
    nco_commit_lag_sum::Int64
    nco_commit_lag_max::Int64
    nco_dropped_stale::Int
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
    # Ask the build what it is before driving it: the code NCO's width, the code
    # memory's depth, the tap count and the modulations it can synthesise are all
    # build options, and every one of them was a constant of this driver before
    # the gateware learned to report them. A build too old to answer is refused
    # here, with a message naming what it cannot do, rather than arming channels
    # whose dumps cannot be interpreted.
    caps = gateware_capabilities(csr)
    resolved_channels = n_channels === :detect ? detect_num_channels(csr) : Int(n_channels)
    bank = GNSSBank(
        csr;
        fs = fs_hz,
        n_channels = resolved_channels,
        code_frac_bits = caps.code_frac_bits,
        carrier_phase_bits = caps.carrier_phase_bits,
        num_taps = caps.num_taps,
        max_subchips = max(1, caps.max_subchips),
        replica_bits = max(2, caps.replica_bits),
    )

    device_ants = num_ants(bank)
    device_ants == n_ants || @warn(
        "gateware reports $device_ants antenna block(s) per record but $n_ants " *
        "requested; the host will read $n_ants"
    )

    # Default the handover margin to 10 ms — comfortably more than a CSR write
    # round trip over PCIe, and short enough that the acquisition's code phase
    # has not aged much by the time it commits.
    margin = handover_margin > 0 ? Int(handover_margin) : round(Int, fs_hz / 100)
    # The record stream carries the *widest* layout the build produces, so one
    # element type serves a bank mixing three- and five-tap channels.
    C = correlator_type(Val(Int(n_ants)), Val(Int(caps.num_taps)))

    M2SDRCorrelator{Int(n_ants),C}(
        csr,
        bank,
        raw,
        PipeChannel{GNSSReceiver.CorrelatorDump{C}}(dump_capacity),
        PipeChannel{GNSSReceiver.NCOUpdate}(nco_capacity),
        fs_hz,
        margin,
        String(dma_device),
        Dict{Tuple{Symbol,Int},Vector{Int}}(),
        correlator_capabilities(caps, fs_hz; num_antennas = Int(n_ants)),
        caps.code_frac_bits,
        caps.num_taps,
        max(1, caps.max_subchips),
        Int64(0),
        nothing,
        nothing,
        false,
        nothing,
        0,
        fill(false, resolved_channels),
        zeros(Int32, resolved_channels),
        fill(:none, resolved_channels),
        Union{Nothing,PendingHandover}[nothing for _ = 1:resolved_channels],
        [Threads.Atomic{Int64}(typemax(Int64)) for _ = 1:resolved_channels],
        [ReentrantLock() for _ = 1:resolved_channels],
        Union{Nothing,GNSSReceiver.NCOUpdate}[nothing for _ = 1:resolved_channels],
        0,
        Int64(0),
        Int64(0),
        0,
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

# What the flashed gateware can replicate and correlate, read off its own
# capability CSRs at construction. GNSSReceiver validates every configured
# signal against this before a channel is armed, so an unserviceable request is
# an actionable error instead of a channel that never locks.
GNSSReceiver.hardware_capabilities(sdr::M2SDRCorrelator) = sdr.capabilities

"""
    primary_code(signal, prn) -> Vector{Int}

The channel's replica: `get_code_length(signal)` chips of `signal`'s *primary*
code for `prn`, as the 0/1 the code RAM takes.

Read from the primary code table rather than from `GNSSSignals.get_code`, which
multiplies in secondary chip 0 of the overlay. For BeiDou B1I PRN 6 (and half
the BeiDou and Galileo pilot PRNs) that chip is `-1`, so `get_code` would load
an inverted replica: every accumulator's sign flips, the PLL locks 180° out and
every navigation bit decodes backwards. The overlay is the host's to remove from
the dumps once its phase is known (GNSSReceiver.jl#132), not something to bake
into a replica before it is.

These are the *primary chips* only. Everything a BOC-family signal adds happens
inside a chip, and the gateware evaluates that separately from a per-channel
sub-chip table ([`ReplicaShape`](@ref)) — so the amplitude-bearing part of
Galileo E1B's CBOC (RMS ≈ 19.9) lives in the table, not here, and the chips stay
±1 for every signal. Reducing the *table* to signs would be a different code,
which is why the modulation and its sub-chip factor are both checked against
what the device declares before anything is loaded.
"""
primary_code(signal::AbstractGNSSSignal, prn::Integer) = Int[
    GNSSSignals.get_code_at_index(signal, chip, prn) > 0 ? 1 : 0 for
    chip = 0:(get_code_length(signal)-1)
]

# The cached replica for one signal *and* PRN. Generating a 10230-chip code is
# cheap next to the 10230 CSR writes that load it, but the cache is what keeps a
# re-assignment of the same satellite from rewriting the code RAM at all.
_cached_code!(cache::AbstractDict, signal::AbstractGNSSSignal, prn::Integer) =
    get!(() -> primary_code(signal, prn), cache, (get_signal_id(signal), Int(prn)))

# The arming window itself, as a function of the channel alone so it can be
# checked against a recorded register map rather than a board.
#
# Order matters and is the gateware's, not a convention: the chips first (each
# with its subcarrier-table select bit beside it), then the table and the staged
# replica shape. Both raise `code_status.loading`, so the channel emits no
# records until the restart the caller schedules commits all of it at once.
# Dropping the select bits leaves a GPS L1C-P channel replicating BOC(1,1) at
# every chip position — a channel that correlates, and reads about 0.6 dB down
# with the wrong correlation shape, rather than one that errors.
function _arm_replica!(
    ch::GNSSBankChannel,
    prn::Integer,
    code::AbstractVector,
    shape::ReplicaShape,
    num_taps::Integer,
)
    load_code!(ch, prn, code; select = subcarrier_select_bits(shape, length(code)))
    load_replica_shape!(ch, shape; num_taps)
    ch
end

# The sub-chip replica this channel is about to be programmed with, and the one
# place the host's amplitude bookkeeping is checked against it.
#
# GNSSReceiver divides the channel's declared code amplitude out of every
# accumulator before the C/N₀ estimator sees it, so a table programmed at one
# scale and declared at another reads the ratio away from the same satellite
# tracked in software — 26 dB for Galileo E1B reduced to signs — while tracking
# perfectly well. The table here is GNSSSignals' own, so the declared default is
# right by construction; say so loudly if a caller overrode it to something the
# programmed table is not.
function _replica_shape!(sdr::M2SDRCorrelator, signal::AbstractGNSSSignal, s::ChannelSignal)
    shape = replica_shape(signal)
    if !isapprox(shape.code_amplitude, s.code_amplitude; rtol = 1e-3)
        @warn "the channel's declared code amplitude is not the amplitude of the " *
              "replica table being programmed; every C/N₀ on this channel will be " *
              "out by their ratio" signal = s.id prn = s.prn declared = s.code_amplitude programmed =
            shape.code_amplitude
    end
    shape
end

# The quantised tap offsets GNSSReceiver hands over, checked against what this
# build can place them with: `[-s, 0, +s]` for three taps and
# `[-s2, -s1, 0, s1, s2]` for five, latest first with the prompt at zero.
#
# Returned as-is rather than reduced to a spacing and rebuilt. `Tracking`'s
# discriminators recover the tap distances from the correlator they are handed,
# and a five-tap correlator has two of them (E/L and VE/VL) that enter the
# discriminator separately, so there is no single number the array could be
# re-derived from — which is why v3's gateware has one register per tap and the
# contract hands over the whole array.
#
# `tap_layouts` is what the *build* declares (`[3]` or `[3, 5]`); a layout
# outside it is one the bank cannot produce, and the missing taps cannot be
# invented on the host.
function _validated_tap_shifts(
    shifts::AbstractVector{<:Integer},
    tap_layouts::AbstractVector{<:Integer},
)
    n = length(shifts)
    n in tap_layouts || throw(
        ArgumentError(
            "this channel needs a $n-tap correlator ($(collect(shifts))) and the " *
            "gateware's bank produces $(join(tap_layouts, "/"))-tap layouts; the " *
            "missing taps cannot be invented on the host",
        ),
    )
    prompt = div(n, 2) + 1
    shifts[prompt] == 0 || throw(
        ArgumentError(
            "tap offsets $(collect(shifts)) must carry the prompt (0) at index " *
            "$prompt; they are ordered latest first",
        ),
    )
    # Latest first means strictly increasing. An early-first array has the
    # prompt in the same place and a zero in the same slot, so ordering is the
    # only thing that catches it — and programming it reversed inverts the DLL
    # discriminator, which reads as "tracking never converges".
    issorted(shifts; lt = <) && allunique(shifts) || throw(
        ArgumentError(
            "tap offsets $(collect(shifts)) are not ordered latest first (strictly " *
            "increasing, negative through the prompt to positive); programming them " *
            "reversed inverts the DLL discriminator",
        ),
    )
    collect(Int, shifts)
end

# Every reason this device cannot serve `signal`, checked against what the
# gateware reported about itself, as one message — or `nothing`.
#
# GNSSReceiver runs the same check before the receiver starts
# (`validate_hardware_configuration`) and again before each arm, so reaching
# this is either a hand-built link or a signal the receiver grew later. Either
# way the CSR writes must not happen: a channel armed on a code the bank cannot
# hold correlates whatever the code RAM still contained.
_unsupported_reason(sdr::M2SDRCorrelator, s::ChannelSignal) =
    _unsupported_reason(sdr.capabilities, sdr.max_subchips, sdr.fs, s)

function _unsupported_reason(
    caps::GNSSReceiver.HardwareCorrelatorCapabilities,
    max_subchips::Integer,
    fs::Real,
    s::ChannelSignal,
)
    reasons = String[]
    if !isnothing(caps.modulations) && !(s.modulation in caps.modulations)
        push!(
            reasons,
            "the gateware cannot synthesise $(s.modulation) modulation (it declares " *
            "$(join(caps.modulations, ", "))), and reducing an amplitude-bearing " *
            "replica to ±1 chips would correlate a different code",
        )
    end
    # The family bit is not enough. `:BOCsin` is one bit whether the build can
    # hold `BOCsin(1,1)`'s 2 sub-chips or `BOCsin(6,1)`'s 12, and a table too
    # shallow for the order asked for does not error in the gateware either — it
    # raises `code_status.replica_unsupported` and suppresses the channel's
    # dumps, which on the host reads as a satellite that never comes up.
    if s.subchips > max_subchips
        push!(
            reasons,
            "$(s.modulation) at this order needs a $(s.subchips)-entry sub-chip " *
            "table and the build holds $(max_subchips); the sub-chip factor is not " *
            "something the modulation bit can carry (BOCsin(1,1) needs 2 and " *
            "BOCsin(6,1) needs 12), so it is checked against the signal",
        )
    end
    if s.code_length > caps.max_primary_code_length
        push!(
            reasons,
            "the primary code is $(s.code_length) chips, past the build's " *
            "$(caps.max_primary_code_length)-chip code memory",
        )
    end
    lo, hi = caps.code_frequency_limits
    if s.code_frequency < lo || s.code_frequency > hi
        push!(
            reasons,
            "the chip rate $(s.code_frequency / 1e6) Mcps is outside the code NCO's " *
            "$(lo / 1e6)–$(hi / 1e6) Mcps range at fs = $(fs) Hz",
        )
    end
    isempty(reasons) && return nothing
    "cannot track $(s.id) PRN $(s.prn) on this LiteX-M2SDR:\n" *
    join(map(r -> "  - " * r, reasons), "\n")
end

# The modern handover: everything one channel must do, in one argument.
function GNSSReceiver.assign_channel!(
    sdr::M2SDRCorrelator,
    hw_channel,
    config::GNSSReceiver.HardwareChannelConfig,
)
    lock(sdr.assignment_locks[hw_channel])
    try
        _assign_channel!(
            sdr,
            hw_channel,
            ChannelSignal(
                config.signal,
                config.prn;
                signal_index = config.signal_index,
                code_amplitude = config.code_amplitude,
                replica_amplitude = config.replica_amplitude,
                band_id = config.band_id,
            ),
            config.signal,
            config.carrier_doppler,
            config.code_doppler,
            config.code_phase,
            config.valid_at_sample,
            _validated_tap_shifts(config.tap_sample_shifts, sdr.capabilities.tap_layouts),
        )
    finally
        unlock(sdr.assignment_locks[hw_channel])
    end
end

# The three-tap array the legacy interface implies, from the Early-to-Late
# distance it carries: symmetric about the prompt, latest first.
_symmetric_tap_shifts(el_sample_spacing) =
    (shift = max(1, round(Int, el_sample_spacing / 2)); [-shift, 0, shift])

# The legacy positional handover, for a link built by hand against the interface
# that predates `HardwareChannelConfig`. Everything it does not carry is what
# that interface implied: one three-tap E/P/L bank, symmetric about the prompt,
# the primary code at the modelled amplitude.
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
        _assign_channel!(
            sdr,
            hw_channel,
            ChannelSignal(signal, prn),
            signal,
            _hz(carrier_doppler),
            _hz(code_doppler),
            Float64(code_phase),
            Int64(valid_at_sample),
            _symmetric_tap_shifts(el_sample_spacing),
        )
    finally
        unlock(sdr.assignment_locks[hw_channel])
    end
end

function _assign_channel!(
    sdr::M2SDRCorrelator,
    hw_channel,
    channel_signal::ChannelSignal,
    signal::AbstractGNSSSignal,
    carrier_hz::Float64,
    code_doppler_hz::Float64,
    code_phase::Float64,
    valid_at_sample::Int64,
    tap_sample_shifts::Vector{Int},
)
    unsupported = _unsupported_reason(sdr, channel_signal)
    isnothing(unsupported) || throw(ArgumentError("M2SDRCorrelator $unsupported"))

    # Invalidate queued dumps before changing PRN metadata or code RAM.
    sdr.assignment_start[hw_channel][] = typemax(Int64)
    ch = sdr.bank.channels[hw_channel]
    # Publish the signal before any word is derived from it: every conversion
    # below — code step, code phase modulus, E/L spacing — reads it off the
    # channel rather than off a constant.
    ch.signal = channel_signal

    # The code RAM only has to be rewritten when the channel's *signal and PRN*
    # change: 1023 (or 10230) back-to-back CSR writes per handover is not just
    # slow, the ioctl storm has been observed to wedge the board's MSI delivery
    # (DMA0 then starves while the sample counter keeps counting). Cache per
    # channel, and key the test on both — the same PRN number reassigned from
    # GPS L1 C/A to Galileo E1B, or from a data component to its pilot, is a
    # different code of a different length.
    key = signal_key(channel_signal)
    if (sdr.assigned_signals[hw_channel], Int(sdr.assigned_prns[hw_channel])) != key
        # The arming window, in the order §4 of gnss-m2sdr's
        # `docs/subchip_modulation.md` sets out: the chips (with the TMBOC
        # select bit written beside each of them), then the subcarrier table and
        # the staged replica shape, then the code phase — all of it committed by
        # the one restart the scheduled handover below performs. Either write
        # raises `code_status.loading`, so the channel emits no records in
        # between and none can describe a half-written code, a replica whose
        # amplitude changed under the integration, or a tap layout that does not
        # match the accumulators it carries.
        _arm_replica!(
            ch,
            channel_signal.prn,
            _cached_code!(sdr.codes, signal, channel_signal.prn),
            _replica_shape!(sdr, signal, channel_signal),
            length(tap_sample_shifts),
        )
    end

    # Every tap placed from the array the contract handed over, not from a
    # spacing re-derived from it: for three taps a mis-derived spacing is a DLL
    # loop-gain error, and for five there is no single number to re-derive from
    # at all, because the VE/VL distance enters the discriminator separately.
    set_tap_offsets!(ch, tap_sample_shifts, code_doppler_hz)

    # Schedule the handover far enough ahead that the CSR writes land first, and
    # propagate the code phase from the sample it was valid at to the sample it
    # will be committed on. The commit is verified — and a late one re-scheduled
    # — by `verify_handovers!` on the NCO writer task, not here: this runs on the
    # receiver's chunk-processing task, and waiting even the 10 ms margin holds
    # every other channel's NCO word for that long (issue #107).
    _schedule_handover!(
        sdr,
        hw_channel,
        carrier_hz,
        code_doppler_hz,
        code_phase,
        valid_at_sample,
        1,
    )
    sdr.assigned_prns[hw_channel] = Int32(channel_signal.prn)
    sdr.assigned_signals[hw_channel] = channel_signal.id
    sdr.active[hw_channel] = true
    nothing
end

function _schedule_handover!(
    sdr,
    hw_channel,
    carrier_hz,
    code_doppler_hz,
    code_phase,
    valid_at_sample,
    attempt,
)
    ch = sdr.bank.channels[hw_channel]
    s = ch.signal
    # The rate the replica will actually run at between the handover's reference
    # sample and the sample it commits on — the signal's own chip rate scaled by
    # its own carrier, not GPS L1 C/A's. For GPS L5 the two differ by 34 %, i.e.
    # a code phase propagated tens of chips wrong over a 10 ms margin.
    code_freq = s.code_frequency * (1.0 + carrier_hz / s.center_frequency)
    target = sample_count(sdr.bank) + sdr.handover_margin
    elapsed = target - _device_sample(sdr, valid_at_sample)
    code_phase_at_target = mod(code_phase + code_freq * elapsed / sdr.fs, s.code_length)
    schedule!(
        ch,
        target;
        carrier_hz,
        code_doppler_hz,
        carrier_phase_cycles = 0.0,
        code_phase_chips = code_phase_at_target,
    )
    sdr.pending[hw_channel] = PendingHandover(
        Int32(s.prn),
        s.id,
        carrier_hz,
        code_doppler_hz,
        code_phase,
        valid_at_sample,
        target,
        attempt,
        s.code_length,
    )
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
    if !sdr.active[hw_channel] ||
       sdr.assigned_prns[hw_channel] != h.prn ||
       sdr.assigned_signals[hw_channel] != h.signal_id
        sdr.pending[hw_channel] = nothing
        return 0
    end
    status = apply_status(sdr.bank.channels[hw_channel])
    if !status.armed && !status.late
        _confirm_code_commit!(sdr, hw_channel, h)
        sdr.assignment_start[hw_channel][] = h.target
        sdr.pending[hw_channel] = nothing
    elseif h.attempt >= 3
        @warn "handover failed; channel remains unconfirmed" hw_channel prn = h.prn signal =
            h.signal_id
        sdr.pending[hw_channel] = nothing
    else
        _schedule_handover!(
            sdr,
            hw_channel,
            h.carrier_hz,
            h.code_doppler_hz,
            h.code_phase,
            h.valid_at_sample,
            h.attempt + 1,
        )
        return 1
    end
    0
end

# The restart the handover commits on is also the code/length commit point, so
# this is where the staged length and the replica's health become observable:
# `code_length_active` is what is in force and `code_status` is clear only if
# the load finished and the programmed rate is representable. Two CSR reads per
# confirmed handover — a warning here names the one failure mode that otherwise
# looks exactly like a satellite that never comes up.
function _confirm_code_commit!(sdr, hw_channel, h::PendingHandover)
    ch = sdr.bank.channels[hw_channel]
    active = code_length_active(ch)
    status = code_status(ch)
    (
        active == h.code_length &&
        !status.loading &&
        !status.rate_unsupported &&
        !status.replica_unsupported
    ) && return
    @warn "channel armed but its replica did not commit cleanly" hw_channel prn = h.prn signal =
        h.signal_id staged_code_length = h.code_length active_code_length = active loading =
        status.loading rate_unsupported = status.rate_unsupported replica_unsupported =
        status.replica_unsupported
    nothing
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
        ArgumentError(
            "dump_transport must be :auto, :recorder or :device, got $dump_transport",
        ),
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
        service =
            Threads.@spawn :interactive _service_dma!(sdr, transport, Int(dump_pipe_bytes))
        sdr.reader = Base.errormonitor(service)
        sdr.writer = nothing
    else
        # One spin loop owns all CSR traffic: the NCO drain runs between dump
        # polls, so commits land within a poll pass of being pushed and never
        # contend with the poller for the ioctl lock.
        sdr.reader =
            Base.errormonitor(Threads.@spawn :interactive _poll_dumps!(sdr, period))
        sdr.writer = nothing
    end
    sdr
end

function stop!(sdr::M2SDRCorrelator)
    sdr.running || return sdr
    sdr.running = false
    enable!(sdr.bank, false)
    if sdr.nco_commits > 0
        @info "NCO commits: $(sdr.nco_commits) at their scheduled sample, landing " *
              "$(round(1e3 * sdr.nco_commit_lag_sum / sdr.nco_commits / sdr.fs; digits = 3)) ms " *
              "late on average (max $(round(1e3 * sdr.nco_commit_lag_max / sdr.fs; digits = 3)) ms); " *
              "$(sdr.nco_dropped_stale) dropped as stale"
    end
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
function _service_dma!(
    sdr::M2SDRCorrelator{N,C},
    transport::Symbol,
    pipe_bytes::Int,
) where {N,C}
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
            # Wait for DMA data no longer than the next held NCO word allows,
            # so it lands within a millisecond of its scheduled sample.
            wait_ms = _drain_ncos!(sdr)
            _poll_readable(fd, wait_ms) || continue
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
                push!(batch, _to_dump(record, Val(N), sdr.code_frac_bits, wire_taps(C)))
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
# The code phase is read whole off the record — the integer chip index the
# replica sat at on the last integrated sample, plus the NCO's fractional
# register — which is the absolute anchor GNSSReceiver's pseudorange bookkeeping
# wants. It used to be *inferred*: a dump fires on the sample that wraps the last
# chip, so the chip "must be" `CA_CODE_LENGTH - 1`, i.e. 1022. That holds only
# for a 1023-chip code, and only while every dump spans a whole code period
# (GNSSReceiver.jl#133). A record too old to carry the chip reports `NaN`, the
# contract's "this device does not report a code phase": the host then dead
# reckons the pseudorange from the handover seed instead of being handed a
# confident wrong anchor.
function _to_dump(
    record::M2SDRRecord{N},
    ::Val{N},
    frac_bits::Integer = CODE_FRAC_BITS,
    wire::Val = Val(TAPS_EPL),
) where {N}
    accumulators = _wire_accumulators(record, Val(N), wire)
    # The spacing arguments are placeholder metadata: GNSSReceiver replaces the
    # whole correlator with the tracked satellite's before the estimator sees
    # it, so a mismatch here cannot mis-normalise `dll_disc`.
    correlator = _wire_correlator(accumulators, wire)
    output = Tracking.CorrelatorOutput(
        correlator,
        Int(record.integrated_samples),
        Int(record.sample_index),
    )
    channel =
        is_strobe(record) ? GNSSReceiver.EPOCH_STROBE_CHANNEL : Int32(record.channel + 1)   # gateware is 0-based, the host 1-based
    reports_phase = !is_strobe(record) && record.version >= RECORD_FORMAT_VERSION
    code_phase = reports_phase ? code_phase_chips(record, frac_bits) : NaN
    # How many of the wire's accumulator slots this record filled, counted from
    # the first — per channel, not per build. A version-1 record does not say,
    # and by construction filled all three.
    num_taps = record.version >= RECORD_FORMAT_VERSION ? Int(record.num_taps) : TAPS_EPL
    GNSSReceiver.CorrelatorDump(
        channel,
        Int32(record.prn),
        output,
        code_phase,
        is_strobe(record) ? TAPS_EPL : num_taps,
    )
end

# The wire's accumulator slots, latest first.
#
# A three-tap record fills the leading three — `[late, prompt, early]`, which is
# where `Tracking`'s own `div(n - 1, 2) + 1` prompt rule puts them for `n = 3` —
# and leaves the rest alone; GNSSReceiver reads exactly `num_taps` of them and
# never the trailing ones. Zeroing them is not "reporting zero accumulators": a
# zero accumulator is a value a correlator can legitimately produce, and the
# record's `num_taps` is what says these are not one.
_ant_accumulator(x::NTuple{1,ComplexF64}, ::Val{1}) = x[1]
_ant_accumulator(x::NTuple{N,ComplexF64}, ::Val{N}) where {N} = SVector{N,ComplexF64}(x)

_accumulator_eltype(::Val{1}) = ComplexF64
_accumulator_eltype(::Val{N}) where {N} = SVector{N,ComplexF64}

function _wire_accumulators(record::M2SDRRecord{N}, ants::Val{N}, ::Val{TAPS_EPL}) where {N}
    T = _accumulator_eltype(ants)
    SVector{TAPS_EPL,T}(
        _ant_accumulator(record.late, ants),
        _ant_accumulator(record.prompt, ants),
        _ant_accumulator(record.early, ants),
    )
end

function _wire_accumulators(
    record::M2SDRRecord{N},
    ants::Val{N},
    ::Val{TAPS_VEPL},
) where {N}
    T = _accumulator_eltype(ants)
    late = _ant_accumulator(record.late, ants)
    prompt = _ant_accumulator(record.prompt, ants)
    early = _ant_accumulator(record.early, ants)
    if record.num_taps >= TAPS_VEPL
        SVector{TAPS_VEPL,T}(
            _ant_accumulator(record.very_late, ants),
            late,
            prompt,
            early,
            _ant_accumulator(record.very_early, ants),
        )
    else
        SVector{TAPS_VEPL,T}(late, prompt, early, zero(T), zero(T))
    end
end

_wire_correlator(accumulators, ::Val{TAPS_EPL}) =
    Tracking.EarlyPromptLateCorrelator(accumulators, 1)
_wire_correlator(accumulators, ::Val{TAPS_VEPL}) =
    Tracking.VeryEarlyPromptLateCorrelator(accumulators, 1, 2)

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
    mask[core÷8+1] = UInt8(1) << (core % 8)
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
    prototype = _prototype_correlator(Val(N), wire_taps(C))
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
            dump = _read_dump_csrs(sdr, ch, Val(N), wire_taps(C))
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
                    dump.num_taps,
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

_prototype_correlator(ants::Val{N}, wire::Val = Val(TAPS_EPL)) where {N} =
    _wire_correlator(zero(SVector{_wire_slots(wire),_accumulator_eltype(ants)}), wire)

_wire_slots(::Val{W}) where {W} = W

# One coherent CSR dump read: retried until `dump_count` is stable around the
# field reads, so a dump firing mid-read cannot mix two integrations.
function _read_dump_csrs(
    sdr::M2SDRCorrelator,
    ch,
    ants::Val{N},
    wire::Val = Val(TAPS_EPL);
    tries::Integer = 10,
) where {N}
    csr = sdr.csr
    p = ch.prefix
    for _ = 1:tries
        c0 = read(csr, p * "dump_count")
        # How many taps the *latched dump* carries — per channel, not per build.
        # The VE/VL registers of a three-tap dump hold whatever the accumulators
        # happened to contain, so they are not read as correlator values.
        dump_taps = Int(read(csr, p * "dump_num_taps"))
        accumulators = _read_accumulators(csr, p, ants, wire, dump_taps)
        n = read(csr, p * "integrated_samples")
        sample_index = read(csr, p * "sample_index")
        frac = read(csr, p * "dump_code_phase")
        # The integer chip, read rather than inferred — the same correction the
        # DMA record carries. `dump_code_chip` is a v2 CSR, and the constructor
        # refuses a build without it.
        chip = read(csr, p * "dump_code_chip")
        if read(csr, p * "dump_count") == c0
            n == 0 && return nothing
            return (
                count = Int(c0),
                n = Int(n),
                sample_index = Int(sample_index),
                code_phase = Int(chip) + Int(frac) / (1 << sdr.code_frac_bits),
                num_taps = dump_taps,
                correlator = _wire_correlator(accumulators, wire),
            )
        end
    end
    nothing
end

_acc_suffix(a::Integer) = a == 0 ? "" : "_ant$(a)"

# One tap's accumulator across the antennas, by the gateware's short tap name.
function _read_tap(csr, prefix, tap::String, ::Val{1})
    ComplexF64(
        read_signed(csr, prefix * "i" * tap, 32),
        read_signed(csr, prefix * "q" * tap, 32),
    )
end

function _read_tap(csr, prefix, tap::String, ::Val{N}) where {N}
    SVector{N,ComplexF64}(
        ntuple(Val(N)) do ant
            s = _acc_suffix(ant - 1)
            ComplexF64(
                read_signed(csr, prefix * "i" * tap * s, 32),
                read_signed(csr, prefix * "q" * tap * s, 32),
            )
        end,
    )
end

function _read_accumulators(csr, prefix, ants::Val{N}, ::Val{TAPS_EPL}, dump_taps) where {N}
    T = _accumulator_eltype(ants)
    SVector{TAPS_EPL,T}(
        _read_tap(csr, prefix, "l", ants),
        _read_tap(csr, prefix, "p", ants),
        _read_tap(csr, prefix, "e", ants),
    )
end

function _read_accumulators(
    csr,
    prefix,
    ants::Val{N},
    ::Val{TAPS_VEPL},
    dump_taps,
) where {N}
    T = _accumulator_eltype(ants)
    late = _read_tap(csr, prefix, "l", ants)
    prompt = _read_tap(csr, prefix, "p", ants)
    early = _read_tap(csr, prefix, "e", ants)
    if dump_taps >= TAPS_VEPL
        SVector{TAPS_VEPL,T}(
            _read_tap(csr, prefix, "vl", ants),
            late,
            prompt,
            early,
            _read_tap(csr, prefix, "ve", ants),
        )
    else
        SVector{TAPS_VEPL,T}(late, prompt, early, zero(T), zero(T))
    end
end

# Turn NCO updates into CSR commits at their named sample.
#
# Latency here is loop-critical: an update scheduled `feedback_delay_epochs`
# (a few ms) ahead must reach the CSRs before its `apply_at_sample` passes, or
# it commits late with stale values. `PipeChannel`'s blocking single-item
# `take!` parks in a ~10 ms sleep-poll, which batches updates into bursts that
# all land late — the carrier keeps frequency lock but the PLL's phase
# corrections apply at random delays and phase never locks (no data bits).
# Hence the non-blocking batch drain, called from the device service task
# between DMA buffers (`:dma`) or between dump polls (`:csr`).
#
# Updates are *held* until their `apply_at_sample` and written at the first
# pass at or after it. The receiver's delay-aware loop
# (`GNSSReceiver.NCOReferencedPLLAndDLL`) attributes every record to the word
# it believes was running and sizes each correction for the sample the new word
# lands at; a word applied on arrival — up to a millisecond before that sample,
# by an amount that varies with host timing — is attributed to the wrong span.
# Holding costs nothing in loop delay (the receiver chose the sample) and makes
# the landing error one-sided and bounded by the service pass period.
#
# Not the gateware's staged commit (`schedule!`): that has a single staging
# register with cancel-on-re-arm semantics, and the next update arrives before
# the pending one matures under any host jitter, so no word would ever apply
# (the walk-off-and-die disease). The immediate CSRs apply on the next sample
# with no arming. The staged path remains for handovers, which need the
# sample-exact phase load and are one-shot.
function _drain_ncos!(
    sdr::M2SDRCorrelator,
    last_carrier::Vector{Float64} = Float64[],
    last_code::Vector{Float64} = Float64[],
)
    pending = sdr.nco_pending
    n = Base.n_avail(sdr.ncos)
    for _ = 1:n
        update = take!(sdr.ncos)
        checkbounds(Bool, pending, update.channel) || continue
        # The newest word for a channel supersedes whatever was waiting: the
        # receiver never schedules a later command for an earlier sample.
        pending[update.channel] = update
    end
    any(!isnothing, pending) || return 5
    now = sample_count(sdr.bank)
    # Whole milliseconds until the earliest word still held is due, so the
    # service loop can bound its wait for DMA data by it (see `_service_dma!`)
    # instead of letting a word wait for the next buffer, ~2 ms at the live
    # strobe rate. 0 makes the caller come straight back and spin on the sample
    # counter for the remainder of the millisecond.
    earliest = typemax(Int64)
    # A correction computed for a sample far in the past describes a loop state
    # that no longer exists: committing it steers the NCO with stale data and
    # throws the loop (observed while the host catches up a start-up backlog —
    # every satellite lost within a minute). Let the channel free-run instead
    # (a handover-seeded NCO drifts ~0.5 Hz/s, harmless for tens of seconds)
    # and resume with the first fresh correction.
    max_stale = Int64(round(0.02 * sdr.fs))   # 20 ms
    for i in eachindex(pending)
        update = pending[i]
        update === nothing && continue
        if update.apply_at_sample > now                # not due yet: hold it
            earliest = min(earliest, update.apply_at_sample)
            continue
        end
        pending[i] = nothing
        lag = now - update.apply_at_sample
        if lag > max_stale
            sdr.nco_dropped_stale += 1
            continue
        end
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
        write(sdr.csr, ch.prefix * "carrier_freq", carrier_word(ch, update.carrier_doppler))
        write(
            sdr.csr,
            ch.prefix * "code_freq",
            code_word_from_code_doppler(ch, update.code_doppler),
        )
        if update.channel <= length(last_carrier)
            last_carrier[update.channel] = cw
            last_code[update.channel] = kw
        end
        sdr.nco_commits += 1
        sdr.nco_commit_lag_sum += lag
        sdr.nco_commit_lag_max = max(sdr.nco_commit_lag_max, lag)
    end
    earliest == typemax(Int64) && return 5
    clamp(Int(fld((earliest - now) * 1000, Int64(round(sdr.fs)))), 0, 5)
end
