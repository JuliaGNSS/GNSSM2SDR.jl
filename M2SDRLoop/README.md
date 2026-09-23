# M2SDRLoop

The LiteX-M2SDR half of the hardware-correlator loop process: the gnss-m2sdr
gateware's device layer (CSRs, DMA1 records, the tracking bank and its replica
tables), `M2SDRDriver` — the `HardwareLoopCore.AbstractLoopDriver` the loop core
closes its loops through — and `loop_main`, the entry point of the `gnss_loop`
executable.

It depends on nothing that is not needed in the loop process (no GNSSReceiver,
no Tracking, no tasks), so `juliac --trim=safe` builds the executable:

```sh
./build.sh                       # on the target: Julia 1.13 + the JuliaC app
build/gnss_loop --csr=path/to/csr.csv --fs=4e6 [--segment=/dev/shm/gnss-loop-m2sdr0]
                [--epoch=4000] [--channels=6] [--core=K] [--fifo=PRIO] [--seconds=S]
```

The raw stream (DMA0) must be draining, or the bank counts no samples.
`--fifo=PRIO` runs the loop under `SCHED_FIFO`, which is what keeps its
latency under 1 ms when the host is busy (pinning alone does not); it needs
`CAP_SYS_NICE`, which `build.sh` tries to grant the binary with `setcap`. The
receiver attaches to the segment with `GNSSReceiver.RemoteHardwareLoop`, or
lets `GNSSM2SDR.M2SDRRemote` / `remote_loop` spawn the executable for it.

`LiteXCSR(csr_csv; device = nothing)` is a device-less handle whose reads and
writes go to a shadow register file; the tests drive the driver against the
board's recorded `csr.csv` that way.
