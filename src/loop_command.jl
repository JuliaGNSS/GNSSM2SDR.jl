# Starting the loop process.

"""
    default_loop_executable() -> String

Where `M2SDRLoop/build.sh` puts the trimmed `gnss_loop`.
"""
default_loop_executable() = joinpath(pkgdir(M2SDRLoop), "build", "gnss_loop")

"""
    gnss_loop_command(; csr, fs = 4e6, antenna = 1, segment = "/dev/shm/gnss-loop-m2sdr0",
                      executable = default_loop_executable(), fifo = 50, core = nothing,
                      channels = nothing, signals = "GPSL1CA", report = 10, log = nothing,
                      extra = String[])

The `Cmd` that starts the loop process on the board whose gateware's `csr.csv`
is `csr`, for the `spawn` of GNSSReceiver's `HardwareLoop`. `antenna` is the RF
input of the band (1 = RX1, 2 = RX2); `fifo` the `SCHED_FIFO` priority (`nothing`
for the default scheduler; it needs `CAP_SYS_NICE`, which `build.sh` grants the
binary); `core` pins the loop thread. The loop prints a status report every
`report` seconds (0 for none); with `log`, they go to that file.
"""
function gnss_loop_command(;
    csr::AbstractString,
    fs::Real = 4e6,
    antenna::Integer = 1,
    segment::AbstractString = "/dev/shm/gnss-loop-m2sdr0",
    executable::AbstractString = default_loop_executable(),
    fifo::Union{Nothing,Integer} = 50,
    core::Union{Nothing,Integer} = nothing,
    channels::Union{Nothing,Integer} = nothing,
    signals::AbstractString = "GPSL1CA",
    report::Real = 10,
    log::Union{Nothing,AbstractString} = nothing,
    extra::Vector{String} = String[],
)
    args = String[
        "--csr=$csr",
        "--fs=$(Float64(fs))",
        "--antenna=$antenna",
        "--segment=$segment",
        "--signals=$signals",
        "--report=$report",
    ]
    isnothing(fifo) || push!(args, "--fifo=$fifo")
    isnothing(core) || push!(args, "--core=$core")
    isnothing(channels) || push!(args, "--channels=$channels")
    append!(args, extra)
    cmd = Cmd([String(executable); args])
    isnothing(log) ? cmd : pipeline(cmd; stdout = log, stderr = log)
end
