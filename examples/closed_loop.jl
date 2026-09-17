# Closed-loop tracking of one live satellite on the board's FPGA correlators,
# driven through GNSSReceiver's `HardwareCorrelatorLink`.
#
# The FPGA downconverts and correlates; the CPU runs acquisition, the tracking
# loop filters, decoding and PVT off the raw sample stream and pushes NCO
# corrections back over CSR. Nothing is correlated on the CPU.
#
# This is the single-satellite example: two hardware channels, one of them the
# link's noise reference and the other whichever satellite came out of the scan.
# `closed_loop_multi.jl` is the same receiver across every channel the flashed
# gateware has, and is where a PVT fix becomes possible.
#
# ── Why this goes through GNSSReceiver ──────────────────────────────────────
#
# The loop is `NCOReferencedPLLAndDLL`, and it is only delay-aware when it runs
# against a `HardwareCorrelatorLink`. The estimator's hardware method reads the
# link's per-channel `NCOTimeline` — the word each record was really integrated
# under — and `link.scheduled_apply_at_sample`, the device sample the correction
# it is computing will land at. Named as the estimator on a loop that drives the
# CSRs by hand, it dispatches to GNSSReceiver's software path instead, which
# builds a `FixedNCOWord` from the satellite's own Doppler and passes
# `NO_LANDING_SAMPLE` — the conventional loop to the bit, wearing a delay-aware
# name. Getting the benefit means holding the link, not just the estimator.
#
# The delay is what breaks the conventional loop here. A hardware NCO word lands
# milliseconds after the record that motivated it, and at GPS L1 C/A's 18 Hz
# reference bandwidth a correction acting 3–4 ms late instead of 1 ms overshoots
# and limit-cycles while C/N₀ and code lock look perfect. Measured on this board
# and this gateware (gnss-m2sdr#39), same sky, same bandwidth:
#
#   ConventionalAssistedPLLAndDLL, 40 s: PRN 20 climbed 26× → 46× the noise
#     floor and then faded back to 7.8×, still fading; PRN 24 diverged to
#     +48 700 Hz.
#   NCOReferencedPLLAndDLL, 180 s:       PRN 20 held 42–46 dBHz, no decay.
#
# ── What that costs ─────────────────────────────────────────────────────────
#
# Earlier revisions of this example drove the device CSRs directly and pulled
# `csr.jl`/`bank.jl` into a standalone `M2Bank` module, so the board never had to
# precompile GNSSReceiver's dependency tree. That property is gone, deliberately.
# GNSSReceiver, Acquisition, GNSSDecoder, PositionVelocityTime and everything
# under them are precompiled on the Orin the first time this example runs — a few
# minutes, once per Julia version and package set; every later run pays only the
# package load. What it buys is the loop above, plus acquisition, lock detection,
# decoding and PVT rather than a reimplementation of them here.
#
# `examples/staging_slot_semantics.jl` still drives the bank directly, for the
# bring-up questions that are about the gateware rather than about a receiver.
#
# ── Needs, on the host the board is plugged into ────────────────────────────
#
#   - the `m2sdr` litepcie driver (`/dev/m2sdr0` for CSRs and the raw DMA0
#     stream, `/dev/m2sdr1` for the DMA1 record stream) and `m2sdr_record` on
#     the PATH, with the RF front end already at 4 MS/s on L1;
#   - gateware with CSR layout v3 streaming DMA1 record format v2
#     (gnss-m2sdr ≥ #32) — `M2SDRCorrelator` refuses an older build by name
#     rather than driving a register set it does not have;
#   - the gateware's own `csr.csv`;
#   - Julia started with interactive threads, e.g. `julia -t 6,4`. Four tasks
#     live on the interactive pool: this script's main task (Julia puts the main
#     thread there once the pool exists), the raw-stream reader, the DMA1
#     service task, and GNSSReceiver's chunk pipeline. With fewer threads a busy
#     main task — a compilation, say — takes the pipeline's thread away.
#
# Usage: julia -t 6,4 --project=. closed_loop.jl CSR_CSV [PRN] [SECONDS]
#
# PRN 0 (the default) means "acquire them all and track whichever the receiver
# picks": satellites here rise and set within the hour, so hand-picking one goes
# stale fast.

using Printf
using Unitful
using Unitful: Hz, ms, s, dBHz, ustrip
using GNSSSignals: GPSL1CA
using GNSSReceiver
using GNSSM2SDR

const FS = 4e6Hz
const CHUNK = 8000                  # 2 ms of samples per processing chunk
# Two channels: the link arms one as its noise reference (an unassigned PRN at a
# dithered Doppler, measuring the floor through the same replica and the same
# accumulators as a satellite) and hands the other to the satellite.
const N_HW_CHANNELS = 2
const CSR_CSV = ARGS[1]
const PRN = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 0
const SECONDS = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 120.0
const gpsl1 = GPSL1CA()

cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)

# The carrier word the device NCO is actually running for `prn`, read off the
# link's own timeline — the same state the delay-aware estimator references
# every record against, and the one number that says whether the loop is holding
# a satellite or walking away from one.
function held_carrier_hz(link, prn)
    for (assignment, hw_channel) in pairs(link.channel_of)
        assignment.prn == prn || continue
        return link.nco_timelines[hw_channel].applied_carrier_doppler
    end
    nothing
end

function main()
    Base.cumulative_compile_timing(true)
    # The bank's sample counter only advances on samples DMA0 accepts, so its
    # value just before the stream starts is the device index of the stream's
    # first sample. That constant maps the host's sample count onto the device's
    # and every acquisition handover is timed with it, so it is taken here —
    # while the host count is still zero — rather than left to `start!` to latch
    # once the stream has been running for a while.
    origin = let csr = LiteXCSR(CSR_CSV)
        count = sample_count(GNSSBank(csr; fs = ustrip(Hz, FS)))
        close(csr)
        count
    end

    # A drain leaked by a crashed earlier run is fatal in the least visible way:
    # two DMA0 readers split the buffers, and the only symptom is several dB less
    # C/N₀ and a scan that finds nothing.
    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)
    # The correlator bank is a non-intrusive observer on the RX datapath: it only
    # sees samples while DMA0 is draining. The raw stream is therefore not
    # optional and must outlive every channel — stop it and the correlators stop,
    # silently.
    stream = start_raw_stream(; chunk = CHUNK)
    sdr = M2SDRCorrelator(CSR_CSV, stream.channel; fs = FS, n_channels = N_HW_CHANNELS)
    @info "gateware exposes $(num_hardware_channels(sdr)) channels; driving $N_HW_CHANNELS"

    # Built here rather than left to `receive` only so its counters can be read
    # at the end: they are the sole record of dump-stream gaps and dropped
    # feedback. `receive` builds the same link when none is passed.
    link = HardwareCorrelatorLink(sdr; sampling_freq = FS, reference_signal = gpsl1)
    data = receive(
        sdr,
        gpsl1,
        FS;
        link,
        # The point of the example. It is also `receive`'s default on hardware;
        # named here because an example that leaves the loop implicit is an
        # example nobody can copy the loop out of.
        doppler_estimator = NCOReferencedPLLAndDLL(),
        prns = PRN == 0 ? (1:32) : [PRN],
        max_meas = 2^11,
        # ±50 kHz of one-sided coverage. The board's free-running TCXO shows up
        # as an apparent Doppler on top of the satellite's own — satellites turn
        # up around −8.5 kHz here — and a Doppler clipped to a narrow grid is a
        # kHz out, which is the first null of a 1 ms coherent integration.
        acq_min_doppler_coverage = 50_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = 60s,
        # Hold a satellite through a fade: the 31–40 dBHz satellites here breathe
        # several dB either side of the default 30 dBHz, and every drop costs
        # symbol timing and TOW.
        code_lock_cn0_threshold = 24.0dBHz,
    )
    # Enable the device only now, with the pipeline built and its processing task
    # running: the first records then meet a consumer instead of piling up in the
    # dump ring while `receive` is still being compiled.
    #
    # 80 kHz epoch strobes (`epoch_period` in samples, fs ÷ 50): the litepcie
    # driver only completes whole 8 KiB DMA buffers, so the strobe rate is what
    # bounds the dump latency the loop sees — ~0.75 ms here, against ~64 ms at
    # the 1 kHz default, which no loop filter tolerates.
    start!(sdr; dump_source = :dma, epoch_period = 50, device_origin = origin)

    t0 = time()
    last_print = -Inf
    locked_seconds = 0.0
    try
        for d in data
            t = time() - t0
            if t - last_print >= 2
                last_print = t
                # Cumulative GC and compilation time: a stall in the loop is one
                # or the other far more often than it is anything in the receiver.
                gc_s = Base.gc_time_ns() / 1e9
                jit_s = Base.cumulative_compile_time_ns()[1] / 1e9
                @printf("t=%6.1f s  gc=%5.2f s  jit=%5.2f s", t, gc_s, jit_s)
                if isempty(d.sat_data)
                    @printf("  (no satellite assigned yet)\n")
                else
                    for ((_, prn), sat) in pairs(d.sat_data)
                        carrier = held_carrier_hz(link, prn)
                        @printf(
                            "  PRN %2d  %.1f dBHz  |P|=%7.0f  NCO %s  %s%s",
                            prn,
                            cn0_db(sat.cn0),
                            abs(sat.prompt),
                            isnothing(carrier) ? "  —      " :
                            @sprintf("%+8.1f Hz", carrier),
                            sat.is_in_lock ? "locked" : "acquiring",
                            sat.is_ranging_ready ? "/ranging" : "",
                        )
                        sat.is_in_lock && (locked_seconds = t)
                    end
                    println()
                end
            end
            t > SECONDS && break
        end
    finally
        # Closing the output stops `receive`; its processing task closes the
        # sample channel, which ends the raw reader. Only then take the device
        # down — the bank stops seeing samples the moment DMA0 stops draining.
        close(data)
        close(stream)
        stop!(sdr)
        isnothing(sdr.reader) || wait(sdr.reader)
        isnothing(sdr.writer) || wait(sdr.writer)
    end

    @info "dump stream: lost-record gaps $(link.lost_record_gaps), re-arm gaps $(link.rearm_gaps), " *
          "device-reported drops $(link.dropped_dumps), skipped epochs $(link.skipped_epochs), " *
          "implausible indices $(link.implausible_dumps), dropped NCO updates $(link.dropped_nco_updates)"
    if locked_seconds == 0.0
        @error "no satellite ever reached lock"
        exit(1)
    end
    @info @sprintf("last locked report at t = %.1f s of %.0f s", locked_seconds, SECONDS)
end

main()
