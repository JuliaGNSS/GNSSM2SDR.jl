# The loop process

`gnss_loop` reads the correlator records, closes the carrier and code loops and
writes the NCO words back. It is built from the `M2SDRLoop` package with
`juliac --trim=safe` (see [Setting up a board](setup.md)). Its entry point is
[`loop_main`](@ref).

## Command line

```
gnss_loop --csr=CSR_CSV [options]
```

| option | meaning | default |
|---|---|---|
| `--csr=PATH` | the gateware's `csr.csv` (required) | |
| `--fs=HZ` | sample rate | `4e6` |
| `--channels=N` | hardware channels to drive | all the gateware has |
| `--antenna=K` | RF input of the band (1 = RX1, 2 = RX2). The noise reference correlates on it; satellites use the input the receiver arms them on | `1` |
| `--epoch=SAMPLES` | fold epoch | `fs / 1000` |
| `--strobe=SAMPLES` | epoch-strobe period | `fs / 4000` |
| `--flush-us=US` | pad a partly filled DMA buffer after this long | `100` |
| `--arm-lead=SAMPLES` | how far ahead of the sample counter an arm starts | `fs / 500` |
| `--segment=PATH` | shared-memory segment | `/dev/shm/gnss-loop-m2sdr0` |
| `--csr-device=PATH` | litepcie CSR device | `/dev/m2sdr0` |
| `--dma-device=PATH` | DMA1 record device | `/dev/m2sdr1` |
| `--dma-buffer-size=B` | DMA1 buffer size of the kernel module | `512` |
| `--signals=LIST` | `GPSL1CA`, `GPSL1CA,GalileoE1B` or `GPSL1CA,GalileoE1B_BOC11` | `GPSL1CA` |
| `--core=K` | pin the loop thread to CPU core K | |
| `--fifo=PRIO` | run under `SCHED_FIFO` at PRIO (needs `CAP_SYS_NICE`) | |
| `--seconds=S` | stop after S seconds | run until shut down |
| `--report=S` | print a status block every S seconds (0 turns it off) | `10` |
| `--wait-ms=MS` | longest wait for records per pass | `1` |

`gnss_loop --help` prints this list. [`gnss_loop_command`](@ref) builds the
same command from Julia keywords.

## Latency

A record waits in the gateware until an epoch strobe or a full DMA buffer sends
it, and a partly filled buffer is flushed after `--flush-us`. The strobe period
and the flush time therefore bound how long a record takes to reach the loop.

`--fifo` keeps the loop's latency below a millisecond when the host is busy.
On a Jetson Orin, `SCHED_FIFO` is what removed the scheduling tail under load;
pinning to a core alone did not. `build.sh` grants the binary `CAP_SYS_NICE`
so that this works without root.

## Status report

Every `--report` seconds, and once more when it stops, the loop prints a status
block. Each value is printed on the line after its label. The counters include:

| label | meaning |
|---|---|
| `sample_count` | the gateware's sample counter |
| `sample_overflows`, `record_drops` | samples and records the gateware lost |
| `epochs_folded`, `records_seen`, `strobes_seen` | progress |
| `stale_records` | dumps the driver dropped because their assignment id is not the channel's current one |
| `stale_dumps` | dumps the loop core dropped because they belong to no armed channel or predate its start |
| `words_written`, `words_late` | NCO words written, and words applied more than an epoch after they were due |
| `arms`, `arm_retries`, `arm_failures`, `code_loads` | channel starts and code-memory loads |
| `lost_record_gaps` | gaps in a channel's dumps from lost records |
| `dropped_records` | records the loop core had no room to queue |
| `events_published`, `commands_handled` | traffic on the shared-memory segment |
| `max_pass_us`, `max_record_age_us` | the longest service pass and the oldest record folded since the last report |
| `record_age_below_us` / `record_age_count` | a histogram of record ages |
| `allocated_bytes`, `gc_pauses` | should both stay 0 |

## What the driver does

[`TDMDriver`](@ref) is the loop core's view of the gateware:

- **Arming a channel.** The driver loads the PRN's code into a code-memory
  slot. Slots are cached by signal and PRN, and the least recently used slot is
  replaced first. It carries the handover's code phase forward to a start
  sample `arm_lead_samples` ahead of the sample counter and stages the whole
  assignment. The gateware starts the channel at exactly that sample. The
  driver confirms the start from the gateware's pending and missed bitmaps. If
  the start sample passed before the arm landed, it reschedules the start, up
  to three times.
- **Writing a word.** A carrier and code NCO word takes three CSR writes and
  applies from the channel's next sample. An unchanged word is not written.
- **Reading records.** The driver reads dumps, epoch strobes and padding off
  DMA1. It drops a dump whose assignment id is not the channel's current one.

Nothing on the per-record or per-word path allocates.

[`start_device!`](@ref) resets the engine and starts the records.
[`stop_device!`](@ref) stops the records. The channels keep correlating until
the next `start_device!`, and the sample-tag bit that the raw stream relies on
is left alone.
