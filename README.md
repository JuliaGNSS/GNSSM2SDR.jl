# GNSSM2SDR.jl

[![Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://juliagnss.github.io/GNSSM2SDR.jl/dev/)
[![Test](https://github.com/JuliaGNSS/GNSSM2SDR.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGNSS/GNSSM2SDR.jl/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/JuliaGNSS/GNSSM2SDR.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/JuliaGNSS/GNSSM2SDR.jl)

Use the [LiteX-M2SDR](https://github.com/enjoy-digital/litex_m2sdr) as a hardware
correlator for [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl).

The board's FPGA runs the TDM correlator of
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr): one pipelined
downconversion and correlation datapath, time-multiplexed over 24 channels. The
host never correlates a sample. Three processes share the rest of the work:

| process | what it does | built from |
|---|---|---|
| `m2sdr_record` | drains the raw I/Q stream (DMA0) into a pipe | litex_m2sdr's user tools |
| `gnss_loop` | reads the correlator records (DMA1), closes the carrier and code loops and writes the NCO words back. It does not allocate and runs under `SCHED_FIFO` | `M2SDRLoop/`, with `juliac --trim=safe` |
| the receiver | acquisition on the raw samples, lock detection, decoding, PVT | GNSSReceiver.jl and this package |

`gnss_loop` and the receiver talk through a
[HardwareLoopProtocol](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
segment under `/dev/shm`. The loop arithmetic comes from
[HardwareLoopCore](https://github.com/JuliaGNSS/HardwareLoopCore.jl).

## Usage

```julia
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful

csr = "build/gnss_tdm_m2_x1_ch24_code4096/csr.csv"
raw = start_raw_stream(; csr, chunk = 4000, antenna = 2)
loop = HardwareLoop("/dev/shm/gnss-loop-m2sdr0";
                    spawn = gnss_loop_command(; csr, antenna = 2))
data = receive(loop, raw.channel, GPSL1CA(), 4e6u"Hz";
               device_sample_origin = () -> device_sample_origin(raw),
               replica_amplitude = 127)     # the correlator's carrier table, ±127
```

`examples/receiver.jl` runs GPS L1 C/A end to end:

```sh
julia --project=examples -e 'using Pkg; Pkg.instantiate()'    # once
M2SDR_RECORD=/path/to/m2sdr_record julia -t 4,1 --project=examples examples/receiver.jl /path/to/csr.csv 600 2
```

The board needs some preparation first: the gateware, a kernel module with a
small DMA1 ring, the `gnss_loop` executable and the RF front end. The
[documentation](https://juliagnss.github.io/GNSSM2SDR.jl/dev/) covers each
step, the loop process's options and the software emulator of the gateware.

## Repository layout

- `src/`: **GNSSM2SDR**, the receiver's side. `start_raw_stream` delivers the
  raw samples, each tagged with its device sample index, so the acquisition
  handover is exact and a lost DMA buffer is filled with zeros.
  `gnss_loop_command` builds the command that starts the loop process.
- `M2SDRLoop/`: **M2SDRLoop**, the loop process's side. It contains the
  gateware's CSRs and DMA1 record stream, `TDMDriver` (the
  `HardwareLoopCore.AbstractLoopDriver` for the TDM correlator), `TDMEmulator`
  (the gateware in software) and `loop_main`, the entry point of `gnss_loop`.
  It depends only on what the loop process needs, so it can be trimmed.
- `examples/`: `receiver.jl`, with its own environment.
- `docs/`: the documentation.

## Tests

Neither test suite needs a board. The loop tests run against the emulator.

```sh
julia --project=M2SDRLoop -e 'using Pkg; Pkg.test()'   # driver and emulator, closed loop
julia --project -e 'using Pkg; Pkg.test()'             # tag decoding, the loop command
```

## License

MIT
