# GNSSM2SDR.jl

LiteX-M2SDR vendor package for [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl)'s
hardware-correlator interface ([GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107)).

The FPGA — running the gateware from
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) — downconverts and
correlates; this package moves its correlator dumps to the host and the host's
NCO updates back, so `GNSSReceiver.receive` can run the tracking loop filters
without correlating a single sample on the CPU.

Acquisition, decoding and PVT are unchanged and still run on the CPU from the
raw sample stream, exactly as GNSSReceiver.jl#107 specifies.

```julia
raw = # a SignalChannel of raw I/Q from the board (SoapySDR, LiteXM2SDR.jl, …)
sdr = M2SDRCorrelator("build/…/csr.csv", raw; fs = 4e6u"Hz", n_channels = 4)
start!(sdr)
data = receive(sdr, GPSL1CA(), 4e6u"Hz")
```

## Status

> [!WARNING]
> **The device-facing paths are not yet validated against real hardware.** The
> CSR ioctl, the DMA1 reader and the host↔device sample-counter mapping have
> never successfully driven a board end to end. Treat this as work in progress.

What is tested (52 tests, no board required) is the part that has to agree with
the gateware bit for bit: the 128-byte DMA1 record wire format — including
resynchronising on the magic after a torn or dropped buffer — and the
fixed-point NCO word conversions.

Known gaps:

- `_read_dumps!` opens `/dev/m2sdr1` and reads it directly, but a litepcie
  chardev yields nothing until the DMA writer is started via ioctl. This is
  confirmed broken on hardware and needs the `liblitepcie` DMA setup sequence.
- At the time of writing the gnss-m2sdr gateware's correlators do not produce
  genuine correlation on hardware, and bit 31 of the accumulator readback CSRs
  reads stuck at 1. Both are upstream of this package.

## Layout

| file | what |
|---|---|
| `src/csr.jl` | LiteX CSR access over the litepcie `LITEPCIE_IOCTL_REG` ioctl, addresses resolved from the gateware's own `csr.csv` |
| `src/bank.jl` | Tracking-bank and per-channel control, plus the fixed-point NCO word conversions |
| `src/record.jl` | The 128-byte DMA1 correlator-dump record wire format |
| `src/sdr.jl` | The `AbstractHardwareCorrelatorSDR` implementation, DMA1 reader, NCO writer and acquisition handover |

It is a Julia port of gnss-m2sdr's `m2sdr_csr.py`, `gnss_tracking.py` and
`record_format.py`, plus the vendor half of the #107 interface.

## Design notes

**The raw sample stream is deliberately not owned here.** How I/Q comes off the
board (SoapySDR, `LiteXM2SDR.jl`'s shared-memory streamer, …) is independent of
the correlator offload. It must keep running for the whole session: the tracking
bank observes the RX datapath non-intrusively, so it only sees samples while
DMA0 is draining. Stop the raw stream and the correlators stop, silently.

**Accumulator order.** Dumps are handed to Tracking.jl as
`[late, prompt, early]` — its `get_prompt_index` is 2. Building them in E/P/L
order inverts the sign of the DLL discriminator and the loop never converges.

**Spacing metadata is the host's.** GNSSReceiver replaces the dump correlator's
`preferred_early_late_to_prompt_code_shift` with the tracked satellite's before
the estimator sees it, so this package only has to get the accumulator values
and their order right.

## License

MIT
