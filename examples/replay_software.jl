# Replay a recorded LiteX-M2SDR raw stream (2R2T sc16, `I₁ Q₁ I₂ Q₂` per
# sample, as `m2sdr_record` writes it) through the *software* receiver: every
# correlation on the CPU, the loops stepped every code period by the estimator
# named on the command line. Written for comparing the loop process against the
# conventional loop on the very same samples (`LOOP_RECORD` in
# `loop_process.jl` records them).
#
#   julia -t 6 --project=. examples/replay_software.jl FILE conventional|nco [SECONDS] [OUT_PREFIX]
#
# Writes `OUT_PREFIX.csv` with one line per receiver output and satellite
# (signal time, prn, C/N₀, lock flags, prompt).
using GNSSReceiver, GNSSSignals, Tracking, Unitful
using Unitful: Hz, dBHz

file = ARGS[1]
which = length(ARGS) >= 2 ? ARGS[2] : "conventional"
seconds = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : Inf
prefix = length(ARGS) >= 4 ? ARGS[4] : "replay_" * which
fs = 4e6
chunk = 4000
words_per_sample = 4
estimator = which == "nco" ? NCOReferencedPLLAndDLL() : ConventionalAssistedPLLAndDLL()
cn0_dbhz(cn0) = 10 * log10(max(Unitful.linear(cn0) / Hz, 1e-12))

# Antenna 1 of the 2R2T stream, chunk by chunk, until `seconds` or the end.
measurement_channel = GNSSReceiver.SignalChannel{Complex{Int16},1}(chunk, 64)
dropouts = 0
producer = Threads.@spawn begin
    io = open(file)
    raw8 = Vector{UInt8}(undef, 2 * words_per_sample * chunk)
    n = 0
    try
        while n * chunk / fs < seconds
            try
                read!(io, raw8)
            catch e
                e isa EOFError ? break : rethrow()
            end
            raw = reinterpret(Int16, raw8)
            buf = Matrix{Complex{Int16}}(undef, chunk, 1)
            @inbounds for k = 1:chunk
                buf[k, 1] = Complex(raw[words_per_sample*(k-1)+1], raw[words_per_sample*(k-1)+2])
            end
            # A recorder dropout leaves an all-zero millisecond in the file. A
            # zero chunk correlates to exactly zero energy, which the software
            # loop's normalised discriminators turn into NaN; ±1 LSB of dither
            # keeps the chunk a signal-free millisecond the loops coast through
            # instead (121 such chunks in 175 s on 2026-09-23).
            if all(iszero, buf)
                @inbounds for k = 1:chunk
                    buf[k, 1] = Complex(Int16(rand((-1, 1))), Int16(rand((-1, 1))))
                end
                global dropouts += 1
            end
            put!(measurement_channel, buf)
            n += 1
        end
    finally
        close(measurement_channel)
        close(io)
    end
end
Base.errormonitor(producer)

data = receive(
    measurement_channel,
    GPSL1CA(),
    fs * Hz;
    doppler_estimator = estimator,
    max_meas = 2^11,
    prns = 1:32,
    acq_min_doppler_coverage = 50_000.0Hz,
    acq_coherent_integration_time = 10u"ms",
    acquire_async = false,
    pvt_approximate_year = 2026,
)
csv = open(prefix * ".csv", "w")
println(csv, "t_s,prn,cn0_dbhz,in_lock,healthy,ranging_ready,prompt_re,prompt_im,fix")
n = 0
first_fix = nothing
t0 = time()
GNSSReceiver.consume_channel(data) do out
    global n += 1
    t = ustrip(u"s", out.runtime)
    has_fix = !isnothing(out.pvt.time)
    has_fix && isnothing(first_fix) && (global first_fix = t)
    for ((sig, prn), sat) in pairs(out.sat_data)
        println(csv, round(t; digits = 3), ",", prn, ",", round(cn0_dbhz(sat.cn0); digits = 2), ",",
                Int(sat.is_in_lock), ",", Int(sat.is_healthy), ",", Int(sat.is_ranging_ready), ",",
                real(sat.prompt), ",", imag(sat.prompt), ",", Int(has_fix))
    end
    if n % 50 == 0
        sats = join(["$(prn): $(round(cn0_dbhz(sat.cn0); digits = 1))" for ((sig, prn), sat) in sort(collect(pairs(out.sat_data)); by = first)], ", ")
        println("signal t = ", round(t; digits = 1), " s (wall ", round(time() - t0; digits = 0), " s)  ", sats, has_fix ? "  fix" : "")
        flush(stdout)
    end
end
close(csv)
println("dithered dropout chunks: ", dropouts)
println("estimator: ", which, "  first fix after: ", isnothing(first_fix) ? "none" : "$(round(first_fix; digits = 1)) s of signal")
