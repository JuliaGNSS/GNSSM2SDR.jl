# GPS L1 C/A through the FPGA correlator: acquisition, decoding and PVT here,
# downconversion and correlation on the board, the loops in `gnss_loop`.
#
#   julia -t 4,1 --project=examples examples/receiver.jl CSR_CSV [seconds] [antenna]
#
# The RF front end must be configured first, e.g.
#   m2sdr_rf --sample-rate 4e6 --bandwidth 4e6 --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful, Printf, Dates
using Geodesy: LLAfromECEF, wgs84

const CSR = ARGS[1]
const SECONDS = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 300.0
const ANTENNA = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 2
const RECORDER = get(ENV, "M2SDR_RECORD", "m2sdr_record")
const SEGMENT = "/dev/shm/gnss-loop-m2sdr0"
const LOG = get(ENV, "GNSS_LOOP_LOG", "/tmp/gnss_loop.log")

function main()
    raw = start_raw_stream(; csr = CSR, chunk = 4000, antenna = ANTENNA, recorder = RECORDER)
    loop = HardwareLoop(SEGMENT; spawn = gnss_loop_command(; csr = CSR, antenna = ANTENNA, log = LOG))
    data = receive(loop, raw.channel, GPSL1CA(), 4e6u"Hz";
                   device_sample_origin = () -> device_sample_origin(raw), acquire_every = 10u"s",
                   acquisition_doppler_resolution = 100u"Hz",
                   debug = get(ENV, "GNSS_DEBUG", "0") == "1")
    t0 = time()
    last_print = 0.0
    first_fix = nothing
    try
        for d in data
            elapsed = time() - t0
            if !isnothing(d.pvt.time) && isnothing(first_fix)
                first_fix = elapsed
                @printf("first fix after %.1f s\n", elapsed)
                flush(stdout)
            end
            if elapsed - last_print >= 1
                last_print = elapsed
                sats = join([@sprintf("%d:%.0f%s%s%s", prn, ustrip(s.cn0), s.is_in_lock ? "L" : "-", s.is_ranging_ready ? "R" : "", s.is_healthy ? "H" : "")
                              for ((_, prn), s) in pairs(d.sat_data)], " ")
                pos = if isnothing(d.pvt.time)
                    "no fix"
                else
                    lla = LLAfromECEF(wgs84)(d.pvt.position)
                    pdop = isnothing(d.pvt.dop) ? NaN : d.pvt.dop.PDOP
                    @sprintf("%.6f° %.6f° %.1f m, %d sats, PDOP %.1f, %s", lla.lat, lla.lon, lla.alt, length(d.pvt.sats), pdop, string(d.pvt.time))
                end
                @printf("t=%6.1f s (signal %6.1f s)  [%s]  %s\n", elapsed, ustrip(u"s", d.runtime), sats, pos)
                flush(stdout)
            end
            elapsed > SECONDS && break
        end
    finally
        close(data)
        close(loop)
        close(raw)
    end
    @printf("raw stream: %d samples lost upstream (zero-filled), %d resyncs\n", raw.gap_samples[], raw.resyncs[])
end

main()
