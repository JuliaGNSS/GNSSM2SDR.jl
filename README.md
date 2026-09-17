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

**Closed-loop tracking is verified on hardware** (Jetson Orin + LiteX-M2SDR).
The bring-up loop that drove the bank's CSRs by hand held one satellite for 180 s
at a ~999 Hz update rate and ~90k NCO commits with zero late, the carrier tracking
the satellite's physical Doppler ramp throughout.

**That loop is not what the examples are any more.** At GPS L1 C/A's 18 Hz
reference bandwidth a conventional PLL/DLL cannot hold the few milliseconds
between a record and the NCO write it motivates: on sky it faded (PRN 20, 26× →
46× → 7.8× the noise floor over 40 s, still fading) or diverged outright (PRN 24,
+48 700 Hz), while C/N₀ and code lock looked perfect. `examples/closed_loop.jl`
and `examples/closed_loop_multi.jl` now drive GNSSReceiver's
`HardwareCorrelatorLink` with the delay-aware `NCOReferencedPLLAndDLL`, which
held PRN 20 at 42–46 dBHz for 180 s with no decay on the same board and gateware
(measurements from [gnss-m2sdr#39](https://github.com/JuliaGNSS/gnss-m2sdr/pull/39)).
The estimator is only delay-aware *through the link* — named on a loop that
writes the CSRs directly it dispatches to GNSSReceiver's software path and is the
conventional loop to the bit. The price is that GNSSReceiver's dependency tree is
now precompiled on the board on first run; `examples/staging_slot_semantics.jl`
still drives the bank directly for questions that are about the gateware.

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
unserviceable one before a channel is armed.

**BOC-family signals are replicated sub-chip, on five taps**
([#10](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/10),
[gnss-m2sdr#32](https://github.com/JuliaGNSS/gnss-m2sdr/pull/32)). A channel's
subcarrier table is evaluated from the modulation GNSSSignals models — including
Galileo E1's amplitude-bearing CBOC table, whose RMS is exactly
`get_code_amplitude(GalileoE1B())`, so the host's replica normalisation is right
rather than 26 dB out — and written alongside the code, with GPS L1C-P's TMBOC
select bit stored beside each chip. Each tap is placed from the contract's own
`tap_sample_shifts` rather than from a spacing re-derived from it. The tap count
is per channel, so one bank runs GPS L1 C/A on three taps next to Galileo E1 on
five in the same record stream. Which of BPSK, BOC, CBOC and TMBOC a given build
can synthesise — and how deep its sub-chip table is — is read off its capability
CSRs; a signal whose sub-chip factor does not fit is refused by name, because no
modulation bit can tell `BOC(1,1)` from `BOC(6,1)`.

This requires gateware with **CSR layout v3** streaming **DMA1 record format v2**
([gnss-m2sdr#32](https://github.com/JuliaGNSS/gnss-m2sdr/pull/32)). An older or
newer build is refused at construction with a message naming what it cannot do,
rather than driven through a register set it does not have: v3 dropped the single
symmetric `spacing` register and added `tap_offset_{ve,e,l,vl}`, `replica`,
`subcarrier_load` and `dump_num_taps`.

Work in progress:

- **A PVT fix on this board.** `examples/closed_loop_multi.jl` drives every
  channel the CSR map has and reports the first solution, but a fix needs four
  satellites locked, ranging-ready and decoded at once, and the 180 s run above
  held two to three.

### Two traps worth knowing

**Always check `apply_status().late` after a handover.** A phase commit that
applies late puts the code replica hundreds of chips off — indistinguishable from
"the correlators don't work". Retry until `!late && applied_at == target`. The
link does this for you; it is code driving the bank directly that has to.

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
