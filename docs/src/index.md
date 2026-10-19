# GNSSM2SDR.jl

Use the [LiteX-M2SDR](https://github.com/enjoy-digital/litex_m2sdr) as a hardware
correlator for [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl).

The board's FPGA runs the TDM correlator gateware of
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr). One pipelined datapath
does the downconversion and correlation, and it is time-multiplexed over 24
channels. The host never correlates a sample. It closes the carrier and code
loops on the correlator dumps and runs acquisition, bit and frame decoding, and
PVT as usual.

## How the work is split

Three processes share the work:

| process | what it does | built from |
|---|---|---|
| `m2sdr_record` | drains the raw I/Q stream (DMA0) into a pipe | litex_m2sdr's user tools |
| `gnss_loop` | reads the correlator records (DMA1), closes the carrier and code loops and writes the NCO words back. It does not allocate and runs under `SCHED_FIFO` | `M2SDRLoop/`, with `juliac --trim=safe` |
| the receiver | acquisition on the raw samples, lock detection, decoding, PVT | GNSSReceiver.jl and this package |

```
 LiteX-M2SDR (FPGA)                          host
 ──────────────────                          ────
 RX datapath ──► DMA0 raw I/Q ─────────────► m2sdr_record ──► pipe ──► receiver
      │                                                                  ▲
      └──► TDM correlator ──► DMA1 records ──► gnss_loop ◄── /dev/shm ───┘
                 ▲                                │
                 └─────── NCO words (CSRs) ◄──────┘
```

`gnss_loop` and the receiver talk through a
[HardwareLoopProtocol](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
segment under `/dev/shm`. The receiver sends handovers from acquisition and
releases. The loop sends one state per channel and epoch: Doppler, code phase,
C/N₀ and prompt. The loop arithmetic comes from
[HardwareLoopCore](https://github.com/JuliaGNSS/HardwareLoopCore.jl), the same
code that GNSSReceiver.jl uses for its software loops.

Keeping the loops in their own process keeps them fast. The receiver's garbage
collection, compilation and acquisition bursts never delay an NCO update. On a
Jetson Orin the loop process allocates no memory, has no GC pauses, and folds
records at most 0.7 ms after the gateware wrote them.

## The two packages

This repository holds two Julia packages:

- **`M2SDRLoop`** (in `M2SDRLoop/`) is the loop process's half. It contains the
  gateware's CSRs and DMA1 record stream, [`TDMDriver`](@ref) (the
  `HardwareLoopCore.AbstractLoopDriver` for the TDM correlator),
  [`TDMEmulator`](@ref) (the gateware in software) and [`loop_main`](@ref), the
  entry point of `gnss_loop`. It depends only on what the loop process needs, so
  `juliac --trim=safe` can build an executable from it.
- **`GNSSM2SDR`** is the receiver's half. [`start_raw_stream`](@ref) delivers
  the raw samples tagged with their device sample index, and
  [`gnss_loop_command`](@ref) builds the command that starts the loop process.

## Quick start

With a [prepared board](setup.md):

```julia
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful

csr = "build/gnss_tdm_m2_x1_ch24_code4096/csr.csv"
raw = start_raw_stream(; csr, chunk = 4000, antenna = 2)
loop = HardwareLoop("/dev/shm/gnss-loop-m2sdr0";
                    spawn = gnss_loop_command(; csr, antenna = 2))
data = receive(loop, raw.channel, GPSL1CA(), 4e6u"Hz";
               device_sample_origin = () -> device_sample_origin(raw))

for d in data
    # d.pvt, d.sat_data, … as with GNSSReceiver's software receiver
end
```

See [Running the receiver](receiver.md) for details.

## Signals

The trimmed `gnss_loop` knows its signals at compile time. It currently serves
GPS L1 C/A (`--signals=GPSL1CA`, the default) and GPS L1 C/A together with
Galileo E1B (`--signals=GPSL1CA,GalileoE1B`).

The driver programs each channel's replica from GNSSSignals' Int8 code table,
the one Tracking.jl correlates with: ±1 for GPS L1 C/A, and 12 sub-chips of
±25/±13 for Galileo E1B's CBOC. Replicating anything but GPS-style BPSK needs
gnss-m2sdr's interface revision 2 (`gnss_id` `0x7D4C0002`); the driver rejects
what the gateware cannot replicate, such as TMBOC.

