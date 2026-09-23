"""
    M2SDRLoop

The LiteX-M2SDR half of the hardware-correlator loop process
(GNSSReceiver.jl `docs/plans/2026-09-22-loop-process.md`, Milestone 4): the
device layer of the gnss-m2sdr gateware — CSRs, the DMA1 record stream, the
record format, the tracking bank and its replica tables — and, on top of it,
[`M2SDRDriver`](@ref), the `HardwareLoopCore.AbstractLoopDriver` the loop core
closes its loops through, and [`loop_main`](@ref), the entry point of the
`gnss_loop` executable.

This package depends on nothing that is not needed in the loop process —
no GNSSReceiver, no Tracking, no channels or tasks — so `juliac --trim=safe`
can build the executable from it (`bin/gnss_loop.jl`, `build.sh`). The
receiver-side vendor package GNSSM2SDR imports the device layer from here.
"""
module M2SDRLoop

using GNSSSignals
using StaticArrays: SVector
using Unitful
using Unitful: Hz
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
    is_shadow,
    has_register,
    read_signed,
    DMAWriterStream,
    GNSSBank,
    GNSSBankChannel,
    ChannelSignal,
    ReplicaShape,
    replica_shape,
    subchip_factor,
    detect_num_channels,
    gateware_version,
    gateware_capabilities,
    decode_capabilities,
    sample_count,
    apply_status,
    applied_at,
    overflow,
    clear_overflow!,
    enable!,
    set_epoch_period!,
    M2SDRRecord,
    parse_record,
    parse_records!,
    is_strobe,
    code_phase_chips,
    M2SDRDriver,
    start_device!,
    stop_device!,
    loop_main

include("csr.jl")
include("dma.jl")
include("subcarrier.jl")
include("bank.jl")
include("record.jl")
include("driver.jl")
include("main.jl")

end # module M2SDRLoop
