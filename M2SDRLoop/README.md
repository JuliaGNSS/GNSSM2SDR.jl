# M2SDRLoop

The LiteX-M2SDR half of the hardware-correlator loop process: the device layer
of gnss-m2sdr's TDM correlator gateware — its CSRs (resolved from the build's
`csr.csv`), the DMA1 record stream and the 64-byte record — `TDMDriver`, the
`HardwareLoopCore.AbstractLoopDriver` the loop core closes its loops through,
`TDMEmulator`, the gateware in software behind the same calls, and `loop_main`,
the entry point of the `gnss_loop` executable.

It depends on nothing the loop process does not need (no GNSSReceiver, no
Tracking, no tasks), so `juliac --trim=safe` builds the executable:

```sh
./build.sh              # on the target: Julia 1.13 + the JuliaC app
build/gnss_loop --csr=path/to/csr.csv --antenna=2 [--fifo=50] [--core=K] [--seconds=S]
```

`build/gnss_loop --help` lists the options. `--fifo=PRIO` runs the loop under
`SCHED_FIFO`, which keeps its latency under a millisecond when the host is busy;
it needs `CAP_SYS_NICE`, which `build.sh` grants the binary with `setcap`. The
receiver attaches to the segment with `GNSSReceiver.HardwareLoop` (or spawns the
executable through it, with `GNSSM2SDR.gnss_loop_command`).

The driver:

- **arms** a channel by loading the PRN's code into a code-memory slot (cached
  by PRN, least recently used slot first), carrying the handover's code phase
  to a start sample `arm_lead_samples` ahead of the counter, and staging the
  whole assignment; the gateware starts it at exactly that sample, and the
  driver confirms the start (or re-schedules a missed one, up to three times)
  from the gateware's pending/missed bitmaps;
- writes a **word** as three CSR writes, applied from the channel's next sample,
  and skips an unchanged one;
- reads **records** off DMA1 — dumps, epoch strobes and padding — and drops a
  dump whose assignment id is not the channel's current one.
