# Running the receiver

A receiver session has three parts: start the raw stream, attach to (or start)
the loop process, and run `GNSSReceiver.receive` over both.

GNSSM2SDR does not depend on GNSSReceiver.jl. It only provides the raw sample
channel and the loop command, which you hand to GNSSReceiver yourself, so add
GNSSReceiver.jl to your own environment, as `examples/` does.

## The example

`examples/receiver.jl` runs GPS L1 C/A end to end and prints the satellites and
the position once per second. It has its own environment:

```sh
julia --project=examples -e 'using Pkg; Pkg.instantiate()'    # once
M2SDR_RECORD=/path/to/m2sdr_record \
    julia -t 4,1 --project=examples examples/receiver.jl /path/to/csr.csv 600 2
```

The arguments are the gateware's `csr.csv`, the run time in seconds (default
300) and the antenna input (1 = RX1, 2 = RX2; default 2). Environment variables
set the recorder binary (`M2SDR_RECORD`), the loop process's log file
(`GNSS_LOOP_LOG`, default `/tmp/gnss_loop.log`) and GNSSReceiver's debug output
(`GNSS_DEBUG=1`).

Start Julia with an interactive thread (`-t N,1`). The raw stream's reader
blocks in the kernel and needs an interactive thread of its own.

On a Jetson Orin whose antenna saw a quarter of the sky, the first fix came
52–103 s after start. With six satellites at a PDOP of 2.9, the median
horizontal error was 9 m.

## Step by step

### The raw stream

```julia
raw = start_raw_stream(; csr = "path/to/csr.csv", chunk = 4000, antenna = 2)
```

[`start_raw_stream`](@ref) turns on the gateware's sample tags, starts
`m2sdr_record` and returns a [`RawStream`](@ref). Its `channel` is a
`SignalChannel` of `Complex{Int16}` chunks from one antenna.

The gateware tags every RX word with its device sample index. The reader
decodes the tags, so it knows which device sample the first host sample is
([`device_sample_origin`](@ref)). Every acquisition handover is stated against
this origin, so the loop process starts each channel at exactly the sample that
acquisition measured. When a DMA buffer is lost, the reader fills the hole with
zeros, so host sample `k` stays device sample `origin + k` for the whole run.
`raw.gap_samples[]` counts the zero-filled samples, and `raw.resyncs[]` counts
how often the tags had to be locked onto again.

The recorder is a separate process, so the receiver's garbage collection,
compilation or a stalled event loop never reach the driver's DMA ring. The pipe
between the two is grown to 32 MiB where the kernel allows it. The recorder also
queues `host_buffers` DMA buffers itself (about a second at 4 MS/s), because an
unprivileged pipe cannot grow past `/proc/sys/fs/pipe-max-size`.

!!! warning "Keep the raw stream running"
    The correlator observes the RX datapath without interrupting it. It only
    sees samples while DMA0 is drained. If you stop the raw stream, the
    correlators stop too, and nothing reports an error.

### The loop process

```julia
loop = HardwareLoop("/dev/shm/gnss-loop-m2sdr0";
                    spawn = gnss_loop_command(; csr = "path/to/csr.csv", antenna = 2,
                                              log = "/tmp/gnss_loop.log"))
```

`GNSSReceiver.HardwareLoop` attaches to the shared-memory segment. With `spawn`,
it starts the loop process first. [`gnss_loop_command`](@ref) builds that
command. Its keywords map to the [command-line options](loop_process.md) of
`gnss_loop`. The most important are:

- `antenna`: the RF input of the band, which must be the same as the raw
  stream's;
- `fifo`: the `SCHED_FIFO` priority (default 50; `nothing` for the default
  scheduler);
- `core`: a CPU core to pin the loop thread to;
- `signals`: `"GPSL1CA"`, `"GPSL1CA,GalileoE1B"` or `"GPSL1CA,GalileoE1B_BOC11"`
  (the one the receiver tracks; see [Signals](index.md#Signals));
- `report` and `log`: how often the loop prints its status and where to.

You can also start `gnss_loop` yourself, for example under a service manager,
and attach to its segment without `spawn`.

### Receiving

```julia
data = receive(loop, raw.channel, GPSL1CA(), 4e6u"Hz";
               device_sample_origin = () -> device_sample_origin(raw),
               replica_amplitude = 127)     # the correlator's carrier table, ±127
for d in data
    # d.pvt, d.sat_data, d.runtime
end
```

GNSSReceiver runs acquisition on the raw chunks and hands each satellite it
finds to the loop process. It reads the per-epoch loop states back for lock
detection, bit and frame decoding and PVT. See GNSSReceiver.jl's documentation
for the keywords of `receive`.

### Shutting down

Close in reverse order:

```julia
close(data)
close(loop)
close(raw)     # stops m2sdr_record and waits for the reader
```
