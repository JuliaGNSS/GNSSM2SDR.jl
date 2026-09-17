# Closed-loop tracking of every satellite the flashed gateware has a channel
# for, driven through GNSSReceiver's `HardwareCorrelatorLink`, with a PVT fix as
# the goal.
#
# The multi-channel counterpart of `closed_loop.jl` — read that one first: it
# carries the argument for why the loop has to go through the link rather than
# merely be named `NCOReferencedPLLAndDLL`, the measurements that decided it, and
# the precompilation that now happens on the board. What is specific here:
#
#   * the channel count comes from the CSR map, not from a constant — this is
#     the validation harness for the >2-channel gateware images. The link arms
#     one channel as its noise reference and assigns the rest to satellites as
#     the scans find them;
#   * one `TrackState` holds every satellite and the estimator folds them all on
#     one epoch grid, so a satellite's correction is sized against the words its
#     own channel ran, not against a bank-wide average;
#   * a fix is what more than one satellite is for. Four have to be locked,
#     ranging-ready and decoded at once, so this run reports when the first
#     solution appears and keeps going for a while afterwards.
#
# ── Needs ───────────────────────────────────────────────────────────────────
#
# As `closed_loop.jl`: the `m2sdr` litepcie driver and `m2sdr_record`, gateware
# with CSR layout v3 streaming DMA1 record format v2 (gnss-m2sdr ≥ #32) and its
# `csr.csv`, and Julia started with interactive threads (`-t 6,4`).
#
# Usage: julia -t 6,4 --project=. closed_loop_multi.jl CSR_CSV [SECONDS] [MAX_CHANNELS]
#
# MAX_CHANNELS caps how many of the gateware's channels are driven; 0 (the
# default) drives every one of them. It is worth capping on a wide build: a first
# scan can hand over several false alarms along with the real satellites, and on
# a 20-channel image those blocked every real satellite for a minute
# (GNSSReceiver.jl#107).

using Printf
using Unitful
using Unitful: Hz, ms, s, dBHz, ustrip
using GNSSSignals: GPSL1CA
using GNSSReceiver
using GNSSM2SDR

const FS = 4e6Hz
const CHUNK = 8000                  # 2 ms of samples per processing chunk
const CSR_CSV = ARGS[1]
const SECONDS = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 300.0
const MAX_CHANNELS = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 0
const SECONDS_AFTER_FIX = 60.0
const gpsl1 = GPSL1CA()

cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)

# Every satellite currently in lock, with the carrier word its own hardware
# channel's NCO is running — the link's timeline is per channel, which is the
# whole reason a bank of them can be folded on one grid.
function sat_summary(link, sat_data)
    parts = String[]
    for ((_, prn), sat) in pairs(sat_data)
        sat.is_in_lock || continue
        carrier = nothing
        for (assignment, hw_channel) in pairs(link.channel_of)
            assignment.prn == prn || continue
            carrier = link.nco_timelines[hw_channel].applied_carrier_doppler
            break
        end
        push!(
            parts,
            isnothing(carrier) ? @sprintf("%d:%.0f", prn, cn0_db(sat.cn0)) :
            @sprintf("%d:%.0f@%+.0f", prn, cn0_db(sat.cn0), carrier),
        )
    end
    join(parts, " ")
end

# `get_LLA` is PositionVelocityTime's, reached through GNSSReceiver's namespace
# so the example needs no Geodesy dependency of its own.
function position_summary(pvt)
    lla = GNSSReceiver.get_LLA(pvt)
    @sprintf("%.6f° %.6f° %.0f m (%d sats)", lla.lat, lla.lon, lla.alt, length(pvt.sats))
end

function main()
    Base.cumulative_compile_timing(true)
    # The device index of the raw stream's first sample: the bank's counter only
    # advances on samples DMA0 accepts, so its value before the stream starts is
    # the constant that maps the host's sample count onto the device's. Every
    # acquisition handover is timed with it.
    origin, csr_map_channels = let csr = LiteXCSR(CSR_CSV)
        counts = (sample_count(GNSSBank(csr; fs = ustrip(Hz, FS))), detect_num_channels(csr))
        close(csr)
        counts
    end

    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)
    stream = start_raw_stream(; chunk = CHUNK)
    # `n_channels = :detect` counts the `gnss_ch<i>_` banks in the CSR map, i.e.
    # whatever the flashed image actually has. That is the point of this example:
    # nothing below is written against a channel count.
    sdr = M2SDRCorrelator(
        CSR_CSV,
        stream.channel;
        fs = FS,
        n_channels = MAX_CHANNELS > 0 ? MAX_CHANNELS : :detect,
    )
    n_channels = num_hardware_channels(sdr)
    @info "driving $n_channels hardware channel(s); the CSR map has $csr_map_channels"
    n_channels >= 5 || @warn(
        "a PVT fix needs four satellites locked at once, and the link keeps one " *
        "channel as its noise reference: $n_channels channel(s) cannot produce one"
    )

    link = HardwareCorrelatorLink(sdr; sampling_freq = FS, reference_signal = gpsl1)
    data = receive(
        sdr,
        gpsl1,
        FS;
        link,
        doppler_estimator = NCOReferencedPLLAndDLL(),
        prns = 1:32,
        max_meas = 2^11,
        # ±50 kHz: the board's free-running TCXO puts satellites around −8.5 kHz
        # before the physical ±5 kHz even starts. 10 ms coherent × 5 rounds finds
        # the 25–32 dBHz satellites that decide whether there is a fourth.
        acq_min_doppler_coverage = 50_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = 60s,
        code_lock_cn0_threshold = 24.0dBHz,
    )
    # Enable the device only with the pipeline built and its processing task
    # running, and pace DMA1 at 80 kHz (`epoch_period` = fs ÷ 50): the litepcie
    # driver only completes whole 8 KiB buffers, so the strobe rate bounds the
    # dump latency the loops see.
    start!(sdr; dump_source = :dma, epoch_period = 50, device_origin = origin)

    t0 = time()
    first_fix_at = nothing
    last_fix_time = nothing
    fresh_fixes = 0
    max_locked = 0
    last_print = -Inf
    try
        for d in data
            t = time() - t0
            max_locked = max(max_locked, count(sat -> sat.is_in_lock, d.sat_data))
            if !isnothing(d.pvt.time) && !isequal(d.pvt.time, last_fix_time)
                last_fix_time = d.pvt.time
                fresh_fixes += 1
                if isnothing(first_fix_at)
                    first_fix_at = t
                    @info @sprintf("first fix after %.1f s: %s", t, position_summary(d.pvt))
                end
            end
            if t - last_print >= 5
                last_print = t
                # Cumulative GC and compilation time: a stall in the loops is one
                # or the other far more often than it is anything in the receiver.
                gc_s = Base.gc_time_ns() / 1e9
                jit_s = Base.cumulative_compile_time_ns()[1] / 1e9
                @info @sprintf(
                    "t=%6.1f s  runtime=%6.1f s  gc=%5.2f s  jit=%5.2f s  cn0@nco[%s]  %s",
                    t,
                    ustrip(s, d.runtime),
                    gc_s,
                    jit_s,
                    sat_summary(link, d.sat_data),
                    isnothing(d.pvt.time) ? "no fix" : position_summary(d.pvt)
                )
            end
            (!isnothing(first_fix_at) && t - first_fix_at > SECONDS_AFTER_FIX) && break
            t > SECONDS && break
        end
    finally
        # Closing the output stops `receive`; its processing task closes the
        # sample channel, which ends the raw reader. Only then take the device
        # down.
        close(data)
        close(stream)
        stop!(sdr)
        isnothing(sdr.reader) || wait(sdr.reader)
        isnothing(sdr.writer) || wait(sdr.writer)
    end

    @info "dump stream: lost-record gaps $(link.lost_record_gaps), re-arm gaps $(link.rearm_gaps), " *
          "device-reported drops $(link.dropped_dumps), skipped epochs $(link.skipped_epochs), " *
          "implausible indices $(link.implausible_dumps), dropped NCO updates $(link.dropped_nco_updates)"
    @info "most satellites locked at once: $max_locked (a fix needs four)"
    if isnothing(first_fix_at)
        @error "no position fix"
        exit(1)
    end
    @info @sprintf("first fix after %.1f s, %d fresh solutions afterwards", first_fix_at, fresh_fixes)
end

main()
