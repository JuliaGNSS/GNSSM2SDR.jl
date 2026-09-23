# GNSSM2SDR.jl

LiteX-M2SDR vendor package for [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl)'s
hardware-correlator interface ([GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107)).

The FPGA — running the gateware from
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) — downconverts and
correlates, and its tracking loops are closed by a **separate, allocation-free
process**: `gnss_loop`, built from the `M2SDRLoop/` sub-package with
`juliac --trim=safe`. The receiver process acquires, decodes and computes PVT
from the raw sample stream and never sees a correlator record.

```julia
using GNSSM2SDR, GNSSReceiver, GNSSSignals, Unitful

raw = start_raw_stream(; chunk = 4000)                       # DMA0 → SignalChannel
sdr = M2SDRRemote("build/…/csr.csv", raw.channel; fs = 4e6)  # the device
loop = remote_loop(sdr)                                      # spawn gnss_loop, or attach
data = receive(loop, GPSL1CA(), 4e6u"Hz")
```

Build the executable on the target first with `M2SDRLoop/build.sh` (Julia 1.13
and the JuliaC app); `remote_loop` finds it at `M2SDRLoop/build/gnss_loop`.

## Why a separate process

The loops need a correction written a millisecond or two after the record that
motivated it, every millisecond, per channel. In the receiver process that
deadline is missed by whatever else Julia is doing — a GC pause, a compilation,
an acquisition scan — and a missed deadline is not a late correction but an open
loop: at GPS L1 C/A's 18 Hz reference bandwidth the satellite fades or the
carrier walks off while C/N₀ and code lock still look perfect.

`gnss_loop` has no GC to wait for and nothing else to do. It reads DMA1, steps
the loops and writes the NCO registers, and publishes epoch states, records,
bits and status into a shared-memory segment
([HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl))
that the receiver mirrors into its tracking state. The loop arithmetic is
[TrackingLoops.jl](https://github.com/JuliaGNSS/TrackingLoops.jl)'s — the same
code the software receiver runs — driven by
[HardwareLoopCore.jl](https://github.com/JuliaGNSS/HardwareLoopCore.jl).

## Status

**Validated on hardware** (Jetson Orin + LiteX-M2SDR, six-channel five-tap
build, 2026-09-22): a 300 s live-sky run held PRN 20 for the whole run at
45 dBHz with one record per epoch and no gap, PRN 23 for 296 s at 42 dBHz, and
four more satellites for 30–140 s each; no allocation in the loop's service
pass, no lost events, no commands refused and no restarts. Satellites that
dropped did so because their received power faded below the receiver's 30 dBHz
code-lock threshold, which the software receiver reproduces on the same recorded
samples to within 0.5 dB. The measurements are in GNSSReceiver.jl's
`docs/plans/2026-09-22-loop-process/milestone-6-board-validation.md`.

**Signals other than GPS L1 C/A are configured per channel, not compiled in**
([#8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/8)). Each armed channel
carries its own primary-code length, chip rate, carrier, modulation, band and
replica normalisation, and every NCO word, phase wrap and record anchor is
derived from those. Which signals a given board can serve is read off its own
capability CSRs and declared to GNSSReceiver, which refuses an unserviceable one
before a channel is armed.

**BOC-family signals are replicated sub-chip, on five taps**
([#10](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/10),
[gnss-m2sdr#32](https://github.com/JuliaGNSS/gnss-m2sdr/pull/32)). A channel's
subcarrier table is evaluated from the modulation GNSSSignals models — including
Galileo E1's amplitude-bearing CBOC table, whose RMS is exactly
`get_code_amplitude(GalileoE1B())`, so the replica normalisation is right rather
than 26 dB out — and written alongside the code, with GPS L1C-P's TMBOC select
bit stored beside each chip. Each tap is placed from the arm command's own
offsets rather than from a spacing re-derived from them. The tap count is per
channel, so one bank runs GPS L1 C/A on three taps next to Galileo E1 on five in
the same record stream.

This requires gateware with **CSR layout v3** streaming **DMA1 record format v2**
([gnss-m2sdr#32](https://github.com/JuliaGNSS/gnss-m2sdr/pull/32)). An older or
newer build is refused at construction with a message naming what it cannot do,
rather than driven through a register set it does not have.

The unit tests (no board required) cover the parts that must agree with the
gateware bit for bit: the 128-byte DMA1 record wire format — including
resynchronising on the magic after a torn or dropped buffer — the fixed-point NCO
word conversions, the sub-chip replica tables, and the driver itself against a
shadow register file recorded from a real build.

Work in progress:

- **A PVT fix on this board.** A fix needs four satellites locked, ranging-ready
  and decoded at once; the runs above held two to three above the lock
  threshold at any one time.
- **Adopting a running loop's satellites on attach.** A receiver attaching to a
  live `gnss_loop` releases every channel and re-acquires instead of seeding
  from the loop's published channel states.

## Layout

| file | what |
|---|---|
| `M2SDRLoop/src/csr.jl` | LiteX CSR access over the litepcie `LITEPCIE_IOCTL_REG` ioctl, addresses resolved from the gateware's own `csr.csv` |
| `M2SDRLoop/src/dma.jl` | The DMA1 reader: the litepcie ioctls, the writer enable, and `poll(2)` on the channel's fd |
| `M2SDRLoop/src/bank.jl` | Tracking-bank and per-channel control, the fixed-point NCO word conversions and the replica tables |
| `M2SDRLoop/src/subcarrier.jl` | Sub-chip replica shapes evaluated from the GNSSSignals modulation |
| `M2SDRLoop/src/record.jl` | The 128-byte DMA1 record wire format, including the version-2 signal fields |
| `M2SDRLoop/src/driver.jl` | `M2SDRDriver`: the `HardwareLoopCore.AbstractLoopDriver` the loop core closes its loops through |
| `M2SDRLoop/src/main.jl` | `loop_main`: the `gnss_loop` entry point — open the board, create the segment, pin and `SCHED_FIFO`, run the service loop |
| `src/capabilities.jl` | The gateware's capability CSRs mapped onto GNSSReceiver's vendor-neutral profile |
| `src/remote.jl` | `M2SDRRemote` and `remote_loop`: the device the receiver sees and the command that starts the loop process |
| `src/raw_stream.jl` | `start_raw_stream`: `m2sdr_record` into a large pipe, read by a task that blocks in the kernel rather than on Julia's event loop |

`M2SDRLoop` depends on nothing the loop process cannot carry — no GNSSReceiver,
no Tracking, no channels or tasks — which is what lets `juliac --trim=safe`
build the executable from it. It is a Julia port of gnss-m2sdr's `m2sdr_csr.py`,
`gnss_tracking.py` and `record_format.py`, plus the vendor half of the #107
interface.

## Design notes

**The raw sample stream is deliberately not owned here.** How I/Q comes off the
board (SoapySDR, `LiteXM2SDR.jl`'s shared-memory streamer, …) is independent of
the correlator offload. It must keep running for the whole session: the tracking
bank observes the RX datapath non-intrusively, so it only sees samples while
DMA0 is draining. Stop the raw stream and the correlators stop, silently.

**The device counter is latched against host sample 0.** `M2SDRRemote` reads the
board's sample counter when it is built, so an acquisition's code phase — found
at a host sample — can be armed at the right device sample. Build it right after
starting the raw stream; a stream that backs up while the receiver compiles
would break the mapping.

**Accumulator order.** Records reach the loop as `[late, prompt, early]` (and
`[very late, late, prompt, early, very early]` on five taps) — the order
Tracking's correlators want. Building them early-first inverts the sign of the
DLL discriminator and the loop never converges.

**One channel, one signal, all of its own numbers.** A `ChannelSignal` holds the
identity, primary-code length, chip rate, carrier, modulation, band and replica
normalisation of whatever a channel currently replicates, and every conversion
reads it off the channel. Codes are loaded from the primary code table rather
than `GNSSSignals.get_code`, which multiplies in secondary chip 0 — `-1` for
BeiDou B1I PRN 6 and half the pilot PRNs, i.e. an inverted replica. The code RAM
is rewritten only when the satellite changes.

**Always check `apply_status().late` after a handover.** A phase commit that
applies late puts the code replica hundreds of chips off — indistinguishable from
"the correlators don't work". The driver retries until `!late && applied_at ==
target`, three times, and then reports the arm as rejected; code driving the
bank directly (`examples/staging_slot_semantics.jl`) has to do it itself.

## License

MIT
