# GNSSM2SDR.jl

The [LiteX-M2SDR](https://github.com/enjoy-digital/litex_m2sdr) as a hardware
correlator for [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl).

The FPGA runs the TDM correlator of
[gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) (`gnss_m2sdr/tdm/`): one
pipelined downconversion-and-correlation datapath time-multiplexed over 24
channels. Three processes share the work:

| process | does | built from |
|---|---|---|
| `m2sdr_record` | drains the raw I/Q stream (DMA0) into a pipe | litex_m2sdr's user tools |
| `gnss_loop` | reads the correlator records (DMA1), closes the carrier and code loops, writes the NCO words back — allocation-free, `SCHED_FIFO` | `M2SDRLoop/` with `juliac --trim=safe` |
| the receiver | acquisition on the raw samples, lock detection, decoding, PVT | GNSSReceiver.jl + this package |

`gnss_loop` and the receiver talk over a
[HardwareLoopProtocol](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
segment under `/dev/shm`; the loop arithmetic is
[HardwareLoopCore](https://github.com/JuliaGNSS/HardwareLoopCore.jl)'s.

## What is in here

- `M2SDRLoop/` — the loop process's half, trimmable: the gateware's CSRs and
  DMA1 record stream, `TDMDriver` (the `HardwareLoopCore.AbstractLoopDriver` of
  the TDM correlator), `TDMEmulator` (the gateware in software, a port of
  gnss-m2sdr's bit-exact model, for tests and for trying the loop without a
  board) and `loop_main`, the entry point of `gnss_loop`.
- `src/` — the receiver's half: `start_raw_stream` (the tagged raw stream:
  every sample is named by its device index, so the acquisition handover is
  exact and a lost DMA buffer is filled with zeros) and `gnss_loop_command`.
- `examples/receiver.jl` — GPS L1 C/A end to end.

## Setting up a board

1. Build and flash the gateware — gnss-m2sdr's `build_tdm.py` and
   `docs/tdm_correlator.md`.
2. Rebuild litex_m2sdr's kernel module and user tools with the build's headers
   and load the module with a small, MSI-per-buffer ring for DMA1:
   ```
   options m2sdr dma_buffer_size=8192,512 dma_buffer_per_irq=8,1
   ```
3. Build the loop executable on the target (Julia 1.13, JuliaC):
   ```
   M2SDRLoop/build.sh        # -> M2SDRLoop/build/gnss_loop, with cap_sys_nice
   ```
4. Configure the RF front end:
   ```
   m2sdr_rf --sample-rate 4e6 --bandwidth 4e6 --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r
   ```

## Running

```
julia --project=examples -e 'using Pkg; Pkg.instantiate()'    # once
M2SDR_RECORD=…/m2sdr_record julia -t 4,1 --project=examples examples/receiver.jl …/csr.csv 600 2
```

(`2` is the antenna input, RX2.) On orin1 (Jetson Orin, an antenna seeing a
quarter of the sky), 2026-10-16: first fix 103 s after start, five satellites
at 38–46 dB-Hz, the loop process at 0 bytes allocated and records at most
0.6 ms old when they are folded.

## Tests

```
julia --project=M2SDRLoop -e 'using Pkg; Pkg.test()'   # driver + emulator, closed loop
julia --project -e 'using Pkg; Pkg.test()'             # tag decoding, the loop command
```
