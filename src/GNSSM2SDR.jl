"""
    GNSSM2SDR

The LiteX-M2SDR vendor package for GNSSReceiver.jl's hardware-correlator
interface (GNSSReceiver.jl#107).

The FPGA — gateware from [gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) —
downconverts and correlates, and its tracking loops are closed by a separate,
allocation-free process (`gnss_loop`, built from the `M2SDRLoop` sub-package).
This package is the receiver's side of that arrangement: the raw sample stream,
the board's declared capabilities, the device-counter origin and the command
that starts the loop process. Acquisition, decoding and PVT run in the receiver
process off the raw stream, as in the software receiver.

```julia
raw = start_raw_stream(; chunk = 4000)                    # DMA0 into a SignalChannel
sdr = M2SDRRemote("build/…/csr.csv", raw.channel; fs = 4e6)
loop = remote_loop(sdr)                                   # spawn gnss_loop, or attach
data = receive(loop, GPSL1CA(), 4e6u"Hz")
```

The raw stream must be flowing before `M2SDRRemote` is built and must keep
flowing: the tracking bank observes the RX datapath non-intrusively, so it only
sees samples while DMA0 drains, and the device counter is latched against host
sample 0 when the device is built.

Which signals a board can serve is read off its own capability CSRs
([`gateware_capabilities`](@ref)) and declared to GNSSReceiver, which refuses an
unserviceable one before a channel is armed. Each armed channel then carries its
own [`ChannelSignal`](@ref) — code length, chip rate, carrier, modulation, band
and replica normalisation — and every NCO word, phase wrap and record anchor is
derived from it rather than from a compiled-in GPS L1 C/A constant
(GNSSM2SDR.jl#8). BOC-family signals are replicated sub-chip from the modulation
GNSSSignals models ([`ReplicaShape`](@ref)), and five-tap channels place each tap
from the arm command's own offsets (GNSSM2SDR.jl#10). This needs gateware with
CSR layout 3 streaming DMA1 record format 2 (gnss-m2sdr#32); an older or newer
build is refused by name.
"""
module GNSSM2SDR

using GNSSReceiver
using GNSSSignals
using SignalChannels: SignalChannel
using Unitful
using Unitful: Hz

# The device layer — CSRs, DMA1, the record format, the bank and the replica
# tables — and the loop driver built on it live in the trimmable loop package
# (`M2SDRLoop/`), which the allocation-free loop process is built from.
# Everything it defines is imported here so this package's own files, and a
# user reaching for a register, read exactly as before.
using M2SDRLoop
for name in names(M2SDRLoop; all = true)
    (name === :M2SDRLoop || name === :eval || name === :include) && continue
    startswith(String(name), "#") && continue
    @eval import M2SDRLoop: $name
end

export M2SDRRemote,
    remote_loop,
    latch_origin!,
    default_loop_executable,
    RawStream,
    start_raw_stream,
    LiteXCSR,
    GNSSBank,
    GNSSBankChannel,
    ChannelSignal,
    ReplicaShape,
    replica_shape,
    subchip_factor,
    detect_num_channels,
    gateware_version,
    gateware_capabilities,
    sample_count,
    apply_status,
    applied_at,
    overflow,
    clear_overflow!

include("capabilities.jl")
include("raw_stream.jl")
include("remote.jl")

end # module GNSSM2SDR
