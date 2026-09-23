# ─────────────────────────────────────────────────────────────────────────────
# The receiver's side of the LiteX-M2SDR loop process: the raw stream, the
# declared capabilities, the device-counter origin, and the command that starts
# `gnss_loop` (built from `M2SDRLoop/` with `build.sh`). GNSSReceiver's
# `RemoteHardwareLoop` does the rest.
# ─────────────────────────────────────────────────────────────────────────────

"""
    M2SDRRemote(csr_csv, raw; fs, executable = default_loop_executable(),
                segment = "/dev/shm/gnss-loop-m2sdr0", n_channels = :detect,
                epoch = fs ÷ 1000, strobe = fs ÷ 160000, csr_device, dma_device, core = nothing,
                fifo = nothing, n_ants = 1)

A LiteX-M2SDR whose tracking loops run in the `gnss_loop` process, as the
device a [`GNSSReceiver.RemoteHardwareLoop`](@ref) serves.

`raw` is the board's raw sample stream (a `SignalChannel`, e.g.
[`start_raw_stream`](@ref)`(…).channel`), which must already be flowing: the
device counter is latched against host sample 0 *now*, so build this right
after starting the stream. The CSR device is opened here only to read the
gateware's capabilities and that counter; the loop process owns every write.

`executable` is the trimmed `gnss_loop` binary; `core` pins the loop to a CPU
core and `fifo` runs it under `SCHED_FIFO` at that priority.

```julia
raw = start_raw_stream(; chunk = 4000)
sdr = M2SDRRemote("build/…/csr.csv", raw.channel; fs = 4e6)
loop = remote_loop(sdr)                       # spawns gnss_loop, or attaches
data = receive(loop, GPSL1CA(), 4e6u"Hz")
```
"""
mutable struct M2SDRRemote <: GNSSReceiver.AbstractHardwareCorrelatorSDR
    const raw::SignalChannel
    const csr::LiteXCSR
    const csr_csv::String
    const fs::Float64
    const capabilities::GNSSReceiver.HardwareCorrelatorCapabilities
    const n_channels::Int
    const n_ants::Int
    const segment_path::String
    const command::Base.AbstractCmd
    origin::Int64
end

"""
    default_loop_executable() -> String

The `gnss_loop` binary `M2SDRLoop/build.sh` builds, or an error naming it.
"""
function default_loop_executable()
    path = joinpath(pkgdir(GNSSM2SDR), "M2SDRLoop", "build", "gnss_loop")
    isfile(path) || throw(
        ArgumentError(
            "no gnss_loop executable at $path; build it on the target with " *
            "`M2SDRLoop/build.sh` (Julia 1.13 and the JuliaC app), or pass `executable`",
        ),
    )
    path
end

function M2SDRRemote(
    csr_csv::AbstractString,
    raw::SignalChannel;
    fs,
    executable::Union{Nothing,AbstractString} = nothing,
    segment::AbstractString = "/dev/shm/gnss-loop-m2sdr0",
    n_channels::Union{Integer,Symbol} = :detect,
    epoch::Union{Nothing,Integer} = nothing,
    strobe::Union{Nothing,Integer} = nothing,
    csr_device::AbstractString = "/dev/m2sdr0",
    dma_device::AbstractString = "/dev/m2sdr1",
    core::Union{Nothing,Integer} = nothing,
    fifo::Union{Nothing,Integer} = nothing,
    n_ants::Integer = 1,
    signals::AbstractString = "GPSL1CA,GalileoE1B",
    # The loop's status report period in seconds (0 off) and where its output goes.
    report::Real = 0,
    log::Union{Nothing,AbstractString} = nothing,
)
    fs_hz = _hz(fs)
    csr = LiteXCSR(csr_csv; device = csr_device)
    caps = gateware_capabilities(csr)
    channels = n_channels === :detect ? detect_num_channels(csr) : Int(n_channels)
    epoch_samples = something(epoch, round(Int, fs_hz / 1000))
    exe = something(executable, default_loop_executable())
    args = String[
        "--csr=$(csr_csv)",
        "--fs=$(fs_hz)",
        "--channels=$(channels)",
        "--epoch=$(epoch_samples)",
        "--strobe=$(something(strobe, max(1, round(Int, fs_hz / 160000))))",
        "--segment=$(segment)",
        "--csr-device=$(csr_device)",
        "--dma-device=$(dma_device)",
        "--signals=$(signals)",
        "--report=$(report)",
    ]
    isnothing(core) || push!(args, "--core=$(core)")
    isnothing(fifo) || push!(args, "--fifo=$(fifo)")
    command = Cmd(vcat([String(exe)], args))
    isnothing(log) || (command = pipeline(command; stdout = String(log), stderr = String(log)))
    bank = GNSSBank(csr; fs = fs_hz, n_channels = channels)
    M2SDRRemote(
        raw,
        csr,
        String(csr_csv),
        fs_hz,
        correlator_capabilities(caps, fs_hz; num_antennas = Int(n_ants)),
        channels,
        Int(n_ants),
        String(segment),
        command,
        sample_count(bank),
    )
end

GNSSReceiver.raw_sample_channel(sdr::M2SDRRemote) = sdr.raw
GNSSReceiver.num_hardware_channels(sdr::M2SDRRemote) = sdr.n_channels
GNSSReceiver.hardware_capabilities(sdr::M2SDRRemote) = sdr.capabilities
# The gateware wipes the carrier off with a ±127 sin/cos ROM, so every
# accumulator carries that factor and the loop divides it back out.
GNSSReceiver.correlator_gain(::M2SDRRemote) = 127
GNSSReceiver.device_sample_origin(sdr::M2SDRRemote, ::Symbol) = sdr.origin

"""
    latch_origin!(sdr::M2SDRRemote)

Re-read the device counter as the origin of host sample 0 — for a raw stream
(re)started after the device was built.
"""
function latch_origin!(sdr::M2SDRRemote)
    bank = GNSSBank(sdr.csr; fs = sdr.fs, n_channels = sdr.n_channels)
    sdr.origin = sample_count(bank)
    sdr
end

"""
    remote_loop(sdr::M2SDRRemote; kwargs...) -> RemoteHardwareLoop

The receiver's handle on the loop process: attaches to a live `gnss_loop` on
the device's segment or spawns one with the device's command. `kwargs` are
`RemoteHardwareLoop`'s (`heartbeat_timeout`, `attach_timeout`, …).
"""
remote_loop(sdr::M2SDRRemote; kwargs...) =
    GNSSReceiver.RemoteHardwareLoop(sdr; segment = sdr.segment_path, spawn = sdr.command, kwargs...)
