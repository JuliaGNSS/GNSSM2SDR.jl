# Closed-loop tracking of a live satellite on the on-FPGA correlators, with
# Tracking.jl doing the loop filters.
#
# STATUS: runs end to end -- acquisition, code-phase handover, and ~970 NCO
# updates/s folded by Tracking.jl -- but does NOT yet hold lock. See the notes at
# the bottom of this file for what is missing and why.
#
# This is the seam GNSSReceiver.jl#107 is about: the FPGA downconverts and
# correlates, its Early/Prompt/Late dumps are appended to a satellite's
# `correlator_outputs` buffer through Tracking.jl's external-producer path, the
# estimator folds them into one NCO update per epoch, and the update is written
# back over CSR. No correlation on the CPU, and no reimplementation of the
# discriminators or loop filters -- those are Tracking.jl's, already optimised and
# tested.
#
# Device control is GNSSM2SDR.jl's csr.jl/bank.jl, included as M2Bank so this can
# run on the board without GNSSReceiver's full dependency tree.
#
# Usage: julia --project=. closed_loop.jl CSR_CSV PRN DOPPLER_HZ [SECONDS]

using Printf
using StaticArrays: SVector
using Statistics: mean, median
using Unitful: Hz, ustrip

using Acquisition: acquire
using GNSSSignals: GPSL1CA, get_code, get_code_frequency, get_code_center_frequency_ratio
using Tracking:
    TrackState,
    TrackedSat,
    CorrelatorOutput,
    EarlyPromptLateCorrelator,
    append_correlator_output!,
    estimate_dopplers_and_filter_prompt!,
    get_sat_state,
    get_carrier_doppler,
    get_code_doppler,
    get_prompt,
    get_correlator_outputs

# Device control is GNSSM2SDR's own csr.jl / bank.jl. On the Orin this was run with
# those two files pulled into a standalone `M2Bank` module instead, to avoid
# precompiling GNSSReceiver's whole dependency tree on the board -- swap the
# `using GNSSM2SDR` line for `include("M2Bank.jl"); using .M2Bank` to do the same.
using GNSSM2SDR:
    LiteXCSR, GNSSBank, GNSSBankChannel, read_signed,
    carrier_word, code_word, load_code!, schedule!, apply_status,
    sample_count, overflow, spacing_word,
    GPS_L1_HZ, GPS_CA_CHIP_RATE

const FS_NOMINAL = 4e6Hz         # the rate every NCO word is referred to
const FS_NOMINAL_HZ = 4e6
const FRAC = 24
const MARGIN = 24_000
const SKIP = 2
const EPOCH_SAMPLES = 4000       # one GPS L1 C/A code period at 4 MHz
const FEEDBACK_EPOCHS = 2        # loop delay, in epochs, made deterministic

csv     = ARGS[1]
prn     = parse(Int, ARGS[2])
seconds = length(ARGS) > 2 ? parse(Float64, ARGS[3]) : 15.0
# Doppler coverage well past +-7 kHz: the reference offset shows up as an apparent
# Doppler on top of the satellite's own, and observed offsets past 9 kHz are normal
# on this board. A Doppler clipped to a narrow grid is ~1 kHz out, which is exactly
# the first null of a 1 ms coherent integration.
const DOPPLER_COVERAGE = 30_000.0Hz

gpsl1 = GPSL1CA()
csr  = LiteXCSR(csv)
bank = GNSSBank(csr; fs = FS_NOMINAL, n_channels = 2)
ch   = bank.channels[1]

# ---- acquire in THIS process, so the Doppler is fresh -----------------------
# A Doppler even a couple of minutes old is a few hundred Hz out, and 100 Hz is
# ~0.065 chips/s of code-rate error -- two or three chips across the phase sweep
# below, which smears the peak over as many bins and makes the search unreliable.
# There must also be exactly one DMA0 reader, and the bank only sees samples while
# DMA0 drains, so one long capture serves as both the acquisition source and the
# drain for the whole run.
const CAPTURE_FILE = "/tmp/hwloop_capture.bin"
const ACQ_INSTANTS = 800_000                     # 200 ms
recorder = run(pipeline(`m2sdr_record $CAPTURE_FILE $(Int(FS_NOMINAL_HZ * 30) * 8)`,
                        stdout = devnull, stderr = devnull), wait = false)
need = ACQ_INSTANTS * 8
while !(isfile(CAPTURE_FILE) && filesize(CAPTURE_FILE) >= need) && process_running(recorder)
    sleep(0.02)
end
raw = reinterpret(Int16, read(CAPTURE_FILE)[1:min(need, filesize(CAPTURE_FILE))])
nsmp = length(raw) ÷ 4
words = reshape(view(raw, 1:4nsmp), 4, nsmp)
signal = ComplexF32.(Float32.(view(words, 1, :)), Float32.(view(words, 2, :)))
acq = acquire(gpsl1, signal, FS_NOMINAL, prn;
              min_doppler_coverage = DOPPLER_COVERAGE,
              num_coherently_integrated_code_periods = 10,
              num_noncoherent_accumulations = 10)
doppler0 = Float64(ustrip(Hz, acq.carrier_doppler))
@printf("acquired PRN %d: CN0 %.1f dBHz, peak/noise %.1f, doppler %+.0f Hz, code phase %.2f chips (from %d instants)\n",
        prn, acq.CN0, acq.peak_to_noise_ratio, doppler0, acq.code_phase, nsmp)
if acq.CN0 < 38
    println("CN0 too low to bother tracking")
    close(csr); kill(recorder); exit(0)
end

code_step = code_word(ch, doppler0)
r = code_step / (1 << FRAC)                      # chips per sample
# Tracking.jl quantises the E/L shift to whole samples and `dll_disc` normalises
# with that quantised spacing, so the gateware has to be programmed with the same
# integer shift.
sample_shift = max(1, round(Int, 0.5 * (1 << FRAC) / code_step))

@printf("PRN %d at %+.0f Hz, code_step=%d, E/L shift %d samples\n",
        prn, doppler0, code_step, sample_shift)

write(csr, "gnss_control", 1)
# The loader wants the raw 0/1 chips; get_code returns +-1.
load_code!(ch, prn, [get_code(gpsl1, i, prn) > 0 ? 1 : 0 for i = 0:1022])
write(csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, doppler0))
write(csr, ch.prefix * "carrier_freq", carrier_word(ch, doppler0))
write(csr, ch.prefix * "code_freq", code_step)

# ---- read a dump the FPGA did not overwrite mid-read -------------------------
const ACC = ("ie", "qe", "ip", "qp", "il", "ql")
function coherent_dump(ch)
    for _ = 1:10
        c0 = read(ch.csr, ch.prefix * "dump_count")
        v = Dict(k => read_signed(ch.csr, ch.prefix * k, 32) for k in ACC)
        n = read(ch.csr, ch.prefix * "integrated_samples")
        si = read(ch.csr, ch.prefix * "sample_index")
        if read(ch.csr, ch.prefix * "dump_count") == c0
            return (count = c0, n = Int(n), sample_index = Int(si), acc = v)
        end
    end
    nothing
end

prompt_power(d) = float(d.acc["ip"])^2 + float(d.acc["qp"])^2

# ---- drift-compensated code-phase search ------------------------------------
# The satellite's code phase moves a few chips/s, and each trial phase is
# restarted at a different sample, so the trial is expressed in the satellite's
# frame: phi_j = phi_base + (S_j - S0) * r.
function measure(bank, ch, doppler, r, S0, phases, dumps_each)
    out = Tuple{Float64,Float64}[]
    for phi_base in phases
        target = sample_count(bank) + MARGIN
        phi = mod(phi_base + (target - S0) * r, 1023)
        schedule!(ch, target; carrier_hz = doppler, code_doppler_hz = doppler,
                  code_phase_chips = phi)
        while apply_status(ch).armed
        end
        powers = Float64[]
        seen = 0
        while length(powers) < dumps_each
            d = coherent_dump(ch)
            d === nothing && break
            seen += 1
            seen > SKIP && push!(powers, prompt_power(d))
        end
        !isempty(powers) && push!(out, (Float64(phi_base), mean(powers)))
    end
    out
end

S0 = sample_count(bank)
coarse = measure(bank, ch, doppler0, r, S0, 0:1022, 10)
best_coarse = coarse[argmax(last.(coarse))][1]
fine = measure(bank, ch, doppler0, r, S0,
               [mod(best_coarse - 3 + 0.25i, 1023) for i = 0:24], 40)
phi_peak, peak_power = fine[argmax(last.(fine))]
floor_power = median(last.(fine))
@printf("\ncode phase %.2f chips, peak %.2fx floor\n",
        phi_peak, peak_power / floor_power)
if peak_power / floor_power < 3
    println("no usable peak -- not closing the loop")
    close(csr)
kill(recorder)
    exit(0)
end

# ---- hand the acquisition to Tracking.jl ------------------------------------
# `TrackState` is seeded with the code phase and Doppler the search just
# confirmed; from here Tracking.jl owns the loop.
track_state = TrackState(gpsl1, [TrackedSat(gpsl1, prn, phi_peak, doppler0 * Hz)])
# Keyed by RF band (Tracking.jl looks up sampling_frequencies[band]), not by group.
sampling_frequencies = (L1 = FS_NOMINAL,)

target = sample_count(bank) + MARGIN
schedule!(ch, target; carrier_hz = doppler0, code_doppler_hz = doppler0,
          code_phase_chips = mod(phi_peak + (target - S0) * r, 1023))
while apply_status(ch).armed
end

# ---- the loop, in a function so the hot path is not in global scope ---------
function run_loop(csr, ch, bank, gpsl1, prn, track_state, sampling_frequencies,
                  sample_shift, floor_power, peak_power, seconds)
    @printf("\nclosing the loop for %.0f s -- Tracking.jl folds the dumps and the\n",
            seconds)
    @printf("NCO updates go back over CSR\n\n")
    println("   t(s)  epochs   |P|^2/floor   carrier(Hz)   code_dopp(Hz)   |prompt|")

    t0 = time()
    epochs = 0
    prev_count = -1
    powers = Float64[]
    last_print = 0.0
    carrier_hz = 0.0
    code_dopp_hz = 0.0
    while time() - t0 < seconds
        d = coherent_dump(ch)
        (d === nothing || d.count == prev_count) && continue
        prev_count = d.count
        d.n == 0 && continue

        # The dump *is* Tracking.jl's CorrelatorOutput: accumulators in its
        # [late, prompt, early] order, the integrated sample count, and the
        # free-running sample index.
        accs = SVector{3,ComplexF64}(
            ComplexF64(d.acc["il"], d.acc["ql"]),
            ComplexF64(d.acc["ip"], d.acc["qp"]),
            ComplexF64(d.acc["ie"], d.acc["qe"]),
        )
        correlator = EarlyPromptLateCorrelator(accs, sample_shift)
        # append_correlator_output!(track_state, output, prn): the output comes
        # second, then the same addressing forms as the per-signal accessors.
        append_correlator_output!(track_state,
                                  CorrelatorOutput(correlator, d.n, d.sample_index),
                                  prn)

        track_state = estimate_dopplers_and_filter_prompt!(track_state,
                                                           sampling_frequencies)
        sat = get_sat_state(track_state, prn)
        carrier_hz = Float64(ustrip(Hz, get_carrier_doppler(sat)))
        # Tracking.jl reports the code Doppler in chips/s; the CSR helper wants it
        # as the equivalent carrier Doppler.
        code_dopp_hz = Float64(ustrip(Hz, get_code_doppler(sat))) *
                       (GPS_L1_HZ / GPS_CA_CHIP_RATE)
        # Apply at a KNOWN future epoch, exactly as GNSSReceiver's
        # HardwareCorrelatorLink does (apply_at_sample = boundary +
        # feedback_delay_epochs * epoch_length). Writing the words immediately
        # instead leaves the loop delay unmodelled and variable -- a 1 ms loop with
        # default bandwidths does not tolerate that.
        apply_at = d.sample_index + FEEDBACK_EPOCHS * EPOCH_SAMPLES
        schedule!(ch, apply_at; carrier_hz = carrier_hz,
                  code_doppler_hz = code_dopp_hz)

        epochs += 1
        push!(powers, prompt_power(d))
        t = time() - t0
        if t - last_print >= 1.0
            last_print = t
            win = @view powers[max(1, end - 199):end]
            @printf("  %5.1f  %6d   %11.2f   %11.1f   %13.1f   %8.0f\n",
                    t, epochs, mean(win) / floor_power, carrier_hz, code_dopp_hz,
                    abs(get_prompt(correlator)))
        end
    end

    win = @view powers[max(1, end - 399):end]
    @printf("\n%d epochs in %.0f s (%.0f Hz update rate)\n",
            epochs, seconds, epochs / seconds)
    @printf("final mean |P|^2/floor = %.2fx (open-loop peak was %.2fx)\n",
            isempty(powers) ? 0.0 : mean(win) / floor_power,
            peak_power / floor_power)
    @printf("carrier ended at %+.1f Hz, code Doppler %+.1f Hz\n",
            carrier_hz, code_dopp_hz)
    @printf("saturation=0x%x overflow=0x%x\n",
            read(csr, "gnss_saturation"), overflow(bank))
end

run_loop(csr, ch, bank, gpsl1, prn, track_state, sampling_frequencies,
         sample_shift, floor_power, peak_power, seconds)
close(csr)
kill(recorder)

# ---------------------------------------------------------------------------
# What works, and what does not
#
# Works: Acquisition.jl finds the satellite in this same process (so the Doppler
# is fresh -- a Doppler even minutes old is a few hundred Hz out, which is
# ~0.065 chips/s per 100 Hz and smears the phase sweep over several chips); the
# drift-compensated sweep finds the code phase; the FPGA's Early/Prompt/Late dumps
# go into Tracking.jl through `append_correlator_output!`; the estimator folds them
# and the NCO update is scheduled at a known future epoch, the way
# GNSSReceiver's `HardwareCorrelatorLink` does it.
#
# Does not: it does not hold lock. Two reasons, neither of them the gateware
# (test/verilog_sim in gnss-m2sdr proves the correlator despreads an injected
# signal to 0.05% of the analytical value, and on-sky the same sweep gives a clean
# correlation triangle):
#
#  1. A single 1 ms coherent dump on a ~45 dBHz satellite is marginal. Measured
#     like for like on the same capture, a CPU 1 ms correlation peaks at 16.9x the
#     noise floor, and |P|^2 has ~100% per-dump scatter. Tracking.jl's default loop
#     bandwidths at a 1 ms update interval integrate a lot of that noise.
#  2. This script is ad-hoc glue. `HardwareCorrelatorLink` additionally maintains
#     the epoch grid, rejects dumps that are stale after a channel reassignment,
#     and keeps `samples_consumed` bookkeeping so Tracking.jl's internal code phase
#     stays consistent with the device. None of that is here.
#
# The way forward is not to tune this, but to run
# `GNSSReceiver.receive(::AbstractHardwareCorrelatorSDR, ...)` with
# `M2SDRCorrelator`, which owns all of the above. That needs the DMA1 record drain
# (`DMAWriterStream`) rather than the CSR polling used here -- CSR readback is also
# lossy, which is the other reason this loop sees fewer dumps than the FPGA
# produces.
