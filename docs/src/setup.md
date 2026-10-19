# Setting up a board

The board needs four things: the TDM correlator gateware, a matching kernel
module, the `gnss_loop` executable and a configured RF front end.

## 1. Gateware

Build and flash the TDM correlator gateware with gnss-m2sdr's `build_tdm.py`.
gnss-m2sdr's `docs/tdm_correlator.md` describes the build options and the
register map. Keep the build's `csr.csv`. The driver and the raw stream look up
every register address in it, so no address is hard-coded to one build.

## 2. Kernel module and user tools

Rebuild litex_m2sdr's kernel module and user tools (including `m2sdr_record`
and `m2sdr_rf`) against the headers of your gateware build. Load the module
with a small ring for DMA1 that interrupts once per buffer. The correlator
records are small, and the loop process needs each one as soon as it is
written:

```
options m2sdr dma_buffer_size=8192,512 dma_buffer_per_irq=8,1
```

The first value of each pair is for DMA0 (raw I/Q), the second for DMA1
(correlator records). If you change the DMA1 buffer size, pass the new size to
`gnss_loop` with `--dma-buffer-size`.

## 3. The loop executable

Build `gnss_loop` on the target. You need Julia 1.13 and the
[JuliaC](https://github.com/JuliaLang/JuliaC.jl) app:

```sh
M2SDRLoop/build.sh        # -> M2SDRLoop/build/gnss_loop
```

`build.sh` instantiates `M2SDRLoop`, compiles `M2SDRLoop/bin/gnss_loop.jl` with
`juliac --trim=safe` and reports any trim verifier errors. It then tries to
grant the binary `CAP_SYS_NICE` with `sudo setcap`, so that the receiver can
start the loop under `SCHED_FIFO` without root. If that fails, it prints the
command to run by hand:

```sh
sudo setcap cap_sys_nice+ep M2SDRLoop/build/gnss_loop
```

Set `JULIA` and `JULIAC` to use other binaries than `julia +1.13` and `juliac`.

[`gnss_loop_command`](@ref) uses `M2SDRLoop/build/gnss_loop` by default (see
[`default_loop_executable`](@ref)).

## 4. RF front end

Tune the AD9361 to the band before you start the receiver, for example GPS L1
at 4 MS/s:

```sh
m2sdr_rf --sample-rate 4e6 --bandwidth 4e6 --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r
```

The sample rate must match the `fs` you give [`gnss_loop_command`](@ref) and
`GNSSReceiver.receive`.
