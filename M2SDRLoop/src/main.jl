# ─────────────────────────────────────────────────────────────────────────────
# The `gnss_loop` executable: parse the command line, open the board, create
# the shared-memory segment, build the loop core and run it until a shutdown
# command (or the requested duration) ends it. Everything printed goes through
# `report`, one value per line, because that is what `--trim=safe` can prove
# resolvable.
# ─────────────────────────────────────────────────────────────────────────────

const USAGE = """
usage: gnss_loop --csr=CSR_CSV [options]
  --csr=PATH            the gateware's csr.csv (required)
  --fs=HZ               sample rate (default 4e6)
  --channels=N          hardware channels to drive (default: all in the CSR map)
  --epoch=SAMPLES       fold epoch (default: fs / 1000)
  --strobe=SAMPLES      epoch-strobe period (default: fs / 160000). The litepcie driver
                        completes whole 8 KiB buffers (64 records), so the strobe rate
                        bounds the record latency the loop sees: measured on the Orin,
                        40 kHz strobes give 1-3 ms, 80 kHz 0.5-2 ms, 160 kHz 0.25-1 ms,
                        at 7 %, 9 % and 11 % of one core. The fold ignores the strobes.
  --segment=PATH        shared-memory segment (default /dev/shm/gnss-loop-m2sdr0)
  --csr-device=PATH     litepcie CSR device (default /dev/m2sdr0)
  --dma-device=PATH     DMA1 record device (default /dev/m2sdr1)
  --margin=SAMPLES      handover lead (default fs / 100)
  --signals=LIST        GPSL1CA or GPSL1CA,GalileoE1B (default)
  --core=K              pin the loop to CPU core K
  --fifo=PRIO           run under SCHED_FIFO at PRIO (needs CAP_SYS_NICE)
  --seconds=S           stop after S seconds (default: run until shut down)
  --report=S            print a status line every S seconds (default 10, 0 off)
  --wait-ms=MS          longest wait for records per pass (default 1)
"""

report(label::String, value) = (Core.println(Core.stdout, label); Core.println(Core.stdout, value))
report(label::String) = Core.println(Core.stdout, label)

# `--key=value` pairs into a dictionary; anything else is an error.
function parse_options(args::Vector{String})
    opts = Dict{String,String}()
    for arg in args
        startswith(arg, "--") || return nothing
        body = arg[3:end]
        eq = findfirst('=', body)
        if isnothing(eq)
            opts[body] = ""
        else
            opts[body[1:eq-1]] = body[eq+1:end]
        end
    end
    opts
end

option(opts, key::String, default::String) = get(opts, key, default)
option_float(opts, key::String, default::Float64) =
    haskey(opts, key) ? parse(Float64, opts[key]) : default
option_int(opts, key::String, default::Int) = haskey(opts, key) ? parse(Int, opts[key]) : default

# Pin the calling thread to one core: every other thread of the process keeps
# its mask, so a `taskset` at launch time reserves the core for this one.
function pin_thread!(core::Integer)
    mask = zeros(UInt8, 128)
    mask[core÷8+1] = UInt8(1) << (core % 8)
    rc = ccall(:sched_setaffinity, Cint, (Cint, Csize_t, Ptr{UInt8}), 0, length(mask), mask)
    rc == 0
end

# `SCHED_FIFO` for the calling thread; `struct sched_param` is one `int`.
function set_fifo!(priority::Integer)
    param = Ref{Cint}(Cint(priority))
    rc = ccall(:sched_setscheduler, Cint, (Cint, Cint, Ptr{Cint}), 0, 1, param)
    rc == 0
end


# One status line's worth of counters.
function report_status(core::LoopCore, driver::M2SDRDriver, gc_before)
    gc = Base.gc_num()
    report("sample_count", sample_count(driver.bank))
    report("epochs_folded", core.epochs_folded)
    report("records_seen", driver.records_seen)
    report("duplicates", driver.duplicates)
    report("words_written", driver.words_written)
    report("words_late", core.words_late)
    report("stale_dumps", core.stale_dumps)
    report("implausible_dumps", core.implausible_dumps)
    report("dropped_records", core.dropped_records)
    report("events_published", core.events_published)
    report("commands_handled", core.commands_handled)
    report("max_pass_us", div(core.max_pass_ns, 1000))
    report("max_record_age_us", core.max_record_age_us)
    for (i, edge) in enumerate(LATENCY_EDGES_US)
        report("record_age_below_us", edge)
        report("record_age_count", core.latency_hist[i])
    end
    report("record_age_overflow_count", core.latency_hist[end])
    report("allocated_bytes", Base.gc_total_bytes(gc) - Base.gc_total_bytes(gc_before))
    report("gc_pauses", Int(gc.pause - gc_before.pause))
    core.max_pass_ns = 0
    core.max_record_age_us = 0
    nothing
end

# The service loop with a deadline and periodic status, in a function of its
# own so the core's type is concrete throughout.
function run_loop!(core::LoopCore, driver::M2SDRDriver, seconds::Float64, report_every::Float64, wait_ms::Int)
    set_loop_state!(core.segment, HardwareLoopProtocol.LOOP_STATE_RUNNING)
    loop_heartbeat!(core.segment)
    deadline = seconds > 0 ? time_ns() + round(UInt64, seconds * 1e9) : typemax(UInt64)
    next_report = report_every > 0 ? time_ns() + round(UInt64, report_every * 1e9) : typemax(UInt64)
    gc_before = Base.gc_num()
    while core.running && time_ns() < deadline
        service_pass!(core; wait_ms)
        if time_ns() >= next_report
            report_status(core, driver, gc_before)
            gc_before = Base.gc_num()
            next_report = time_ns() + round(UInt64, report_every * 1e9)
        end
    end
    set_loop_state!(core.segment, HardwareLoopProtocol.LOOP_STATE_STOPPED)
    report_status(core, driver, gc_before)
    nothing
end

# Build the core over `driver` and `signals` and run it. A function barrier so
# the tuple type — and with it every bank the core compiles — is concrete.
function serve(driver::M2SDRDriver, signals::Tuple, segment_path::String, epoch::Int, seconds::Float64, report_every::Float64, wait_ms::Int)
    caps = driver_capabilities(driver)
    segment = create_segment(segment_path, SegmentConfig(; channel_count = caps.num_channels, bands = caps.bands))
    core = LoopCore(driver, signals, segment; config = LoopConfig(; epoch_length = epoch))
    report("segment", segment_path)
    report("channels", caps.num_channels)
    report("epoch_length", epoch)
    run_loop!(core, driver, seconds, report_every, wait_ms)
    nothing
end

"""
    loop_main(args) -> Cint

The `gnss_loop` entry point. See `USAGE`.
"""
function loop_main(args::Vector{String})::Cint
    opts = parse_options(args)
    if isnothing(opts) || !haskey(opts, "csr") || haskey(opts, "help")
        Core.println(Core.stderr, USAGE)
        return Cint(2)
    end
    fs = option_float(opts, "fs", 4e6)
    epoch = option_int(opts, "epoch", round(Int, fs / 1000))
    strobe = option_int(opts, "strobe", max(1, round(Int, fs / 160000)))
    segment_path = option(opts, "segment", "/dev/shm/gnss-loop-m2sdr0")
    csr_device = option(opts, "csr-device", "/dev/m2sdr0")
    dma_device = option(opts, "dma-device", "/dev/m2sdr1")
    margin = option_int(opts, "margin", round(Int, fs / 100))
    seconds = option_float(opts, "seconds", 0.0)
    report_every = option_float(opts, "report", 10.0)
    wait_ms = option_int(opts, "wait-ms", 1)
    signal_choice = option(opts, "signals", "GPSL1CA,GalileoE1B")

    if haskey(opts, "core")
        pin_thread!(parse(Int, opts["core"])) || report("warning", "could not pin the loop thread")
    end
    if haskey(opts, "fifo")
        set_fifo!(parse(Int, opts["fifo"])) || report("warning", "could not switch to SCHED_FIFO")
    end

    csr = LiteXCSR(opts["csr"]; device = csr_device)
    version = gateware_version(csr)
    report("gateware_csr_version", version.csr)
    report("gateware_record_version", version.record)
    channels = haskey(opts, "channels") ? parse(Int, opts["channels"]) : detect_num_channels(csr)
    # The signal tuples a build knows, fixed at compile time; the command line
    # picks one, and each is its own code path so the core's type is concrete.
    if signal_choice == "GPSL1CA"
        run_with(csr, (GPSL1CA(),), fs, channels, margin, dma_device, epoch, strobe, segment_path, seconds, report_every, wait_ms)
    else
        run_with(csr, (GPSL1CA(), GalileoE1B()), fs, channels, margin, dma_device, epoch, strobe, segment_path, seconds, report_every, wait_ms)
    end
    report("stopped")
    return Cint(0)
end

function run_with(csr::LiteXCSR, signals::Tuple, fs::Float64, channels::Int, margin::Int, dma_device::String,
                  epoch::Int, strobe::Int, segment_path::String, seconds::Float64, report_every::Float64, wait_ms::Int)
    driver = M2SDRDriver(csr, signals; fs, n_channels = channels, handover_margin = margin, dma_device)
    start_device!(driver; epoch_period = strobe)
    report("strobe_period", strobe)
    report("sample_count_at_start", sample_count(driver.bank))
    try
        serve(driver, signals, segment_path, epoch, seconds, report_every, wait_ms)
    finally
        stop_device!(driver)
        close(csr)
    end
    nothing
end
