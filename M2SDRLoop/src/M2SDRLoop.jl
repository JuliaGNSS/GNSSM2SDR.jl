"""
    M2SDRLoop

The LiteX-M2SDR half of the hardware-correlator loop process: the device layer
of gnss-m2sdr's TDM correlator gateware — its CSRs, the DMA1 record stream and
the 64-byte record — and, on top of it, [`TDMDriver`](@ref), the
`HardwareLoopCore.AbstractLoopDriver` the loop core closes its loops through,
plus [`loop_main`](@ref), the entry point of the `gnss_loop` executable.
[`TDMEmulator`](@ref) is the gateware in software, behind the same calls.

Nothing here is needed only by a receiver (no GNSSReceiver, no Tracking, no
tasks), so `juliac --trim=safe` builds the executable from it (`build.sh`).
"""
module M2SDRLoop

using GNSSSignals
using StaticArrays: MVector
using Unitful
using Unitful: Hz, ustrip, uconvert
using HardwareLoopProtocol
using HardwareLoopCore
import HardwareLoopCore:
    read_records!,
    write_word!,
    arm!,
    release!,
    assignment_start,
    sample_count,
    driver_capabilities,
    wait_records,
    overflowed_channels!

export LiteXCSR,
    DMAWriterStream,
    TDMRegs,
    TDMIO,
    LitePCIeTDM,
    TDMEmulator,
    TDMDriver,
    TDMRecord,
    TDMCapabilities,
    decode_record,
    decode_capabilities,
    tap_layout,
    reg_read,
    reg_write,
    feed!,
    start_device!,
    stop_device!,
    loop_main

include("csr.jl")
include("dma.jl")
include("tdm_format.jl")
include("tdm_io.jl")
include("tdm_driver.jl")
include("emulator.jl")
include("main.jl")

end # module M2SDRLoop
