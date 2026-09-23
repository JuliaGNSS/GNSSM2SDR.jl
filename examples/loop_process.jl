# The receiver with its tracking loops in the `gnss_loop` process
# (GNSSReceiver.jl docs/plans/2026-09-22-loop-process.md).
#
#   julia -t 4,2 --project=. examples/loop_process.jl CSR_CSV [SECONDS] [OUT_PREFIX]
#
# Build the loop executable first: `M2SDRLoop/build.sh` on the target. Writes
# `OUT_PREFIX.csv` (one line per receiver output and satellite: time, prn, C/N₀,
# lock, prompt) and `OUT_PREFIX.loop.log` (the loop process's status reports).
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful
using Unitful: Hz, dBHz

cn0_dbhz(cn0) = 10 * log10(max(Unitful.linear(cn0) / Hz, 1e-12))

csr_csv = ARGS[1]
seconds = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 60.0
prefix = length(ARGS) >= 3 ? ARGS[3] : "loop_run"
fs = 4e6
chunk = 4000

# Compile the pipeline before the raw stream starts: the device counter is
# latched against host sample 0 when `M2SDRRemote` is built, and a stream that
# backs up while `receive` compiles would drop samples and break that mapping.
let warm = GNSSReceiver.SignalChannel{Complex{Int16},1}(chunk, 4)
    Threads.@spawn begin
        for _ = 1:8
            put!(warm, zeros(Complex{Int16}, chunk, 1))
        end
        close(warm)
    end
    collect_data(receive(warm, GPSL1CA(), fs * Hz; acquire_async = false, pvt_approximate_year = 2026))
end
println("warm; probing the raw stream")
# A few hundred KiB off DMA0 before the run: a front end that streams zeros
# or garbage (seen twice on 2026-09-22 after back-to-back runs) would only
# show up as a receiver that acquires noise peaks.
let proc = open(`m2sdr_record -c 0 -q - 0`, "r")
    buf = read(proc, 1 << 19)
    kill(proc)
    samples = reinterpret(Int16, buf)
    rms = sqrt(sum(abs2, Float64.(samples)) / length(samples))
    println("raw stream probe: ", length(samples), " words, rms ", round(rms; digits = 2), " LSB, mean ",
            round(sum(Float64.(samples)) / length(samples); digits = 2))
    rms > 3 || error("the raw stream carries no signal (rms $(rms) LSB); check the front end / m2sdr_record")
    sleep(1.0)
end
println("starting the raw stream")
# `LOOP_RECORD=path` tees the raw 2R2T sc16 stream to `path` while the run
# tracks, so the same samples can be replayed through the software receiver
# (`examples/replay_software.jl`). 32 MB/s at 4 MS/s: mind the space.
record = get(ENV, "LOOP_RECORD", "")
recorder = isempty(record) ? `m2sdr_record -c 0 -q - 0` : `sh -c "m2sdr_record -c 0 -q - 0 | tee $(record)"`
raw = start_raw_stream(; chunk, capacity_chunks = 20_000, pipe_bytes = 256 * 2^20, command = recorder)
sdr = M2SDRRemote(csr_csv, raw.channel; fs, report = 10, log = prefix * ".loop.log")
trace = open(prefix * ".epochs.csv", "w")
loop = remote_loop(sdr; trace, publish_taps = get(ENV, "LOOP_TAPS", "0") == "1")
# The acquisition settings of `closed_loop_multi.jl`: the board's free-running
# TCXO puts every satellite around -8.5 kHz, outside the default search.
data = receive(
    loop,
    GPSL1CA(),
    fs * Hz;
    pvt_approximate_year = 2026,
    prns = 1:32,
    max_meas = 2^11,
    acq_min_doppler_coverage = 50_000.0Hz,
    acq_coherent_integration_time = 10u"ms",
)
csv = open(prefix * ".csv", "w")
println(csv, "t_s,prn,cn0_dbhz,in_lock,healthy,ranging_ready,prompt_re,prompt_im,fix")
t0 = time()
n = 0
first_fix = nothing
try
    GNSSReceiver.consume_channel(data) do out
        global n += 1
        t = time() - t0
        has_fix = !isnothing(out.pvt.time)
        has_fix && isnothing(first_fix) && (global first_fix = t)
        for ((sig, prn), sat) in pairs(out.sat_data)
            println(csv, round(t; digits = 3), ",", prn, ",", round(cn0_dbhz(sat.cn0); digits = 2), ",",
                    Int(sat.is_in_lock), ",", Int(sat.is_healthy), ",", Int(sat.is_ranging_ready), ",",
                    real(sat.prompt), ",", imag(sat.prompt), ",", Int(has_fix))
        end
        if n % 50 == 0
            sats = join(
                ["$(prn): $(round(cn0_dbhz(sat.cn0); digits = 1))$(sat.is_in_lock ? "" : " (out)")" for ((sig, prn), sat) in sort(collect(pairs(out.sat_data)); by = first)],
                ", ",
            )
            println("t = ", round(t; digits = 1), " s  ", sats, has_fix ? "  pvt: $(out.pvt.position)" : "")
            flush(stdout)
        end
        t >= seconds && close(raw)
    end
finally
    close(csv)
    close(loop)
    close(trace)
    println("first fix after: ", isnothing(first_fix) ? "none" : "$(round(first_fix; digits = 1)) s")
    println("loop: events=", loop.events, " arms=", loop.arms_sent, " rejected=", loop.arms_rejected,
            " releases=", loop.releases_sent, " lost_events=", loop.lost_events, " restarts=", loop.restarts,
            " refused_commands=", loop.commands_refused)
end
