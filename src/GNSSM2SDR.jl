"""
    GNSSM2SDR

The LiteX-M2SDR as a hardware correlator for GNSSReceiver.jl.

The FPGA — the TDM correlator gateware of
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) — downconverts and
correlates every channel; the tracking loops are closed by a separate,
allocation-free process (`gnss_loop`, built from the `M2SDRLoop` sub-package
with `juliac --trim=safe`); acquisition, decoding and PVT run in the receiver
process off the raw sample stream. This package is the receiver's side of the
board: the tagged raw stream, the device-sample origin, and the command that
starts the loop process.

```julia
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful
csr = "build/gnss_tdm_m2_x1_ch24_code4096/csr.csv"
raw = start_raw_stream(; csr, chunk = 4000, antenna = 2)
loop =
    HardwareLoop("/dev/shm/gnss-loop-m2sdr0"; spawn = gnss_loop_command(; csr, antenna = 2))
data = receive(
    loop,
    raw.channel,
    GPSL1CA(),
    4e6u"Hz";
    device_sample_origin = () -> device_sample_origin(raw),
)
```
"""
module GNSSM2SDR

using GNSSSignals
using SignalChannels: SignalChannel
using Unitful
using Unitful: Hz

# The device layer and the loop driver live in the trimmable loop package.
using M2SDRLoop
using M2SDRLoop: LiteXCSR, TDMRegs, LitePCIeTDM, reg_read, reg_write

export RawStream,
    start_raw_stream,
    device_sample_origin,
    enable_tagging!,
    gnss_loop_command,
    default_loop_executable,
    LiteXCSR

include("raw_stream.jl")
include("loop_command.jl")

end # module GNSSM2SDR
