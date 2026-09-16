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

**Single-satellite closed-loop tracking is verified on hardware** (Jetson Orin +
LiteX-M2SDR, see `examples/closed_loop.jl`): 60 s at 60–84× the noise floor, then
180 s without losing lock at a ~999 Hz update rate and ~90k NCO commits with zero
late, the carrier tracking the satellite's physical Doppler ramp throughout.

The unit tests (no board required) cover the parts that must agree with the
gateware bit for bit: the 128-byte DMA1 record wire format — including
resynchronising on the magic after a torn or dropped buffer — and the fixed-point
NCO word conversions.

**Signals other than GPS L1 C/A are configured per channel, not compiled in**
([#8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/8)). Each assigned
channel carries its own primary-code length, chip rate, carrier, modulation,
band and replica normalisation, and every NCO word, phase wrap and dump anchor
is derived from those. Which signals a given board can actually serve is read
off its own capability CSRs and declared to GNSSReceiver, which refuses an
unserviceable one before a channel is armed. Today's gateware synthesises plain
±1 (`:LOC`) replicas into a three-tap E/P/L bank, so the BPSK families — GPS
L1 C/A, GPS L5, GPS L2C, Galileo E5, BeiDou B1I/B2/B3 — are in scope and the
BOC/CBOC/TMBOC ones are refused by name until
[gnss-m2sdr#30](https://github.com/JuliaGNSS/gnss-m2sdr/issues/30) adds the
replicas and the five-tap bank.

This requires gateware streaming **DMA1 record format v2**
([gnss-m2sdr#31](https://github.com/JuliaGNSS/gnss-m2sdr/pull/31)); an older
build is refused at construction with a message naming what it cannot do,
rather than driven with a code length it was never told.

Work in progress:

- **Multi-satellite closed loop** (`examples/closed_loop_multi.jl`). A PVT fix
  needs ≥4 channels locked simultaneously.
- The DMA1 record path is implemented but the bring-up loop above closes over CSR
  dump readback.

### Two traps worth knowing

**Always check `apply_status().late` after a handover.** A phase commit that
applies late puts the code replica hundreds of chips off — indistinguishable from
"the correlators don't work". Retry until `!late && applied_at == target`.

**`EarlyPromptLateCorrelator`'s second constructor argument is in chips, not
samples.** Passing a sample shift makes `dll_disc`'s `(2 - d)/2` normalisation go
negative, inverting the DLL. GNSSReceiver guards this on ingest by substituting
the host's correlator as the template, but direct users of Tracking.jl must get it
right themselves.

## Layout

| file | what |
|---|---|
| `src/csr.jl` | LiteX CSR access over the litepcie `LITEPCIE_IOCTL_REG` ioctl, addresses resolved from the gateware's own `csr.csv` |
| `src/bank.jl` | Tracking-bank and per-channel control, plus the fixed-point NCO word conversions |
| `src/record.jl` | The 128-byte DMA1 correlator-dump record wire format, including the version-2 signal fields |
| `src/sdr.jl` | The `AbstractHardwareCorrelatorSDR` implementation, DMA1 reader, NCO writer and acquisition handover |
| `src/precompile.jl` | Precompile statements for the receiver-facing interface on `M2SDRCorrelator`, so no handover or release compiles live on the processing task |
| `src/raw_stream.jl` | `start_raw_stream`: `m2sdr_record` into a large pipe, read by a task that blocks in the kernel rather than on Julia's event loop |

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

**One channel, one signal, all of its own numbers.** A `ChannelSignal` holds the
identity, primary-code length, chip rate, carrier, modulation, band and replica
normalisation of whatever a channel currently replicates, and every conversion
reads it off the channel. Codes are cached and reused by *signal identity plus
PRN*: GPS L1 C/A PRN 7 and Galileo E1B PRN 7 are different codes of different
lengths, and so are a satellite's pilot and data components. The replica itself
comes from the primary code table rather than `GNSSSignals.get_code`, which
multiplies in secondary chip 0 — `-1` for BeiDou B1I PRN 6 and half the pilot
PRNs, i.e. an inverted replica.

**Spacing metadata is the host's.** GNSSReceiver replaces the dump correlator's
`preferred_early_late_to_prompt_code_shift` with the tracked satellite's before
the estimator sees it, so this package only has to get the accumulator values
and their order right.

## License

MIT
