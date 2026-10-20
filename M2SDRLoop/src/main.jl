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
  --channels=N          hardware channels to drive (default: all the gateware has)
  --antenna=K           RF input of the band (1 = RX1, 2 = RX2; default 1); the noise
                        reference correlates on it, satellites on what the receiver arms
  --epoch=SAMPLES       fold epoch (default: fs / 1000)
  --strobe=SAMPLES      epoch-strobe period (default: fs / 4000)
  --flush-us=US         pad a part-filled DMA buffer after this long (default 100)
  --arm-lead=SAMPLES    how far ahead of now an arm starts (default fs / 500)
  --segment=PATH        shared-memory segment (default /dev/shm/gnss-loop-m2sdr0)
  --csr-device=PATH     litepcie CSR device (default /dev/m2sdr0)
  --dma-device=PATH     DMA1 record device (default /dev/m2sdr1)
  --dma-buffer-size=B   DMA1 buffer size the kernel module uses (default 512)
  --signals=LIST        GPSL1CA (default), GPSL1CA,GalileoE1B or GPSL1CA,GalileoE1B_BOC11
                        (Galileo E1B as CBOC needs fs >= 12.276 MHz in the receiver;
                        below that GNSSSignals models it as BOC(1,1))
  --core=K              pin the loop to CPU core K
  --fifo=PRIO           run under SCHED_FIFO at PRIO (needs CAP_SYS_NICE)
  --seconds=S           stop after S seconds (default: run until shut down)
  --report=S            print a status block every S seconds (default 10, 0 off)
  --wait-ms=MS          longest wait for records per pass (default 1)
"""

report(label::String, value) =
    (Core.println(Core.stdout, label); Core.println(Core.stdout, value))
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
            opts[body[1:(eq-1)]] = body[(eq+1):end]
        end
    end
    opts
end

option(opts, key::String, default::String) = get(opts, key, default)
option_float(opts, key::String, default::Float64) =
    haskey(opts, key) ? parse(Float64, opts[key]) : default
option_int(opts, key::String, default::Int) =
    haskey(opts, key) ? parse(Int, opts[key]) : default

# Pin the calling thread to one core: every other thread of the process keeps
# its mask, so a `taskset` at launch time reserves the core for this one.
function pin_thread!(core::Integer)
    mask = zeros(UInt8, 128)
    0 <= core < 8 * length(mask) || return false
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
function report_status(core::LoopCore, driver::TDMDriver, gc_before)
    gc = Base.gc_num()
    r = driver.regs
    report("sample_count", sample_count(driver, 1))
    report("sample_overflows", reg_read(driver.io, r.sample_overflows))
    report("record_drops", reg_read(driver.io, r.record_drops))
    report("epochs_folded", core.epochs_folded)
    report("records_seen", driver.records_seen)
    report("strobes_seen", driver.strobes_seen)
    report("stale_records", driver.stale_records)
    report("words_written", driver.words_written)
    report("words_late", core.words_late)
    report("arms", driver.arms)
    report("arm_retries", driver.arm_retries)
    report("arm_failures", driver.arm_failures)
    report("code_loads", driver.code_loads)
    report("stale_dumps", core.stale_dumps)
    report("lost_record_gaps", core.lost_record_gaps)
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
function run_loop!(
    core::LoopCore,
    driver::TDMDriver,
    seconds::Float64,
    report_every::Float64,
    wait_ms::Int,
)
    set_loop_state!(core.segment, HardwareLoopProtocol.LOOP_STATE_RUNNING)
    loop_heartbeat!(core.segment)
    deadline = seconds > 0 ? time_ns() + round(UInt64, seconds * 1e9) : typemax(UInt64)
    next_report =
        report_every > 0 ? time_ns() + round(UInt64, report_every * 1e9) : typemax(UInt64)
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
function serve(
    driver::TDMDriver,
    signals::Tuple,
    segment_path::String,
    epoch::Int,
    seconds::Float64,
    report_every::Float64,
    wait_ms::Int,
)
    caps = driver_capabilities(driver)
    # Room for a few seconds of events while the receiver is busy (an acquisition).
    segment = create_segment(
        segment_path,
        SegmentConfig(;
            channel_count = caps.num_channels,
            bands = caps.bands,
            event_capacity = 16384,
        ),
    )
    core = LoopCore(driver, signals, segment; config = LoopConfig(; epoch_length = epoch))
    report("segment", segment_path)
    report("channels", caps.num_channels)
    report("epoch_length", epoch)
    run_loop!(core, driver, seconds, report_every, wait_ms)
    nothing
end

# The lowest sampling rate GNSSSignals generates `signal`'s sub-chip table at.
cboc_min_fs(signal::AbstractGNSSSignal) =
    ustrip(Hz, uconvert(Hz, get_code_frequency(signal))) * signal.lut.subchip_factor

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
    strobe = option_int(opts, "strobe", max(1, round(Int, fs / 4000)))
    flush_us = option_float(opts, "flush-us", 100.0)
    arm_lead = option_int(opts, "arm-lead", round(Int, fs / 500))
    antenna = option_int(opts, "antenna", 1)
    segment_path = option(opts, "segment", "/dev/shm/gnss-loop-m2sdr0")
    csr_device = option(opts, "csr-device", "/dev/m2sdr0")
    dma_device = option(opts, "dma-device", "/dev/m2sdr1")
    dma_buffer_size = option_int(opts, "dma-buffer-size", 512)
    seconds = option_float(opts, "seconds", 0.0)
    report_every = option_float(opts, "report", 10.0)
    wait_ms = option_int(opts, "wait-ms", 1)
    signal_choice = option(opts, "signals", "GPSL1CA")
    if !(signal_choice in ("GPSL1CA", "GPSL1CA,GalileoE1B", "GPSL1CA,GalileoE1B_BOC11"))
        Core.println(Core.stderr, USAGE)
        return Cint(2)
    end
    # The receiver samples at the same rate and GNSSSignals generates E1B's
    # CBOC only from code frequency × sub-chips up: below that no receiver
    # arms it, and it has to track GalileoE1B_BOC11.
    if signal_choice == "GPSL1CA,GalileoE1B" && fs < cboc_min_fs(GalileoE1B())
        Core.println(
            Core.stderr,
            "--signals=GPSL1CA,GalileoE1B needs --fs >= $(cboc_min_fs(GalileoE1B())); use GPSL1CA,GalileoE1B_BOC11",
        )
        return Cint(2)
    end

    if haskey(opts, "core")
        pin_thread!(parse(Int, opts["core"])) ||
            report("warning", "could not pin the loop thread")
    end
    if haskey(opts, "fifo")
        set_fifo!(parse(Int, opts["fifo"])) ||
            report("warning", "could not switch to SCHED_FIFO")
    end

    csr = LiteXCSR(opts["csr"]; device = csr_device)
    io = LitePCIeTDM(csr; dma_device, dma_buffer_size)
    channels = haskey(opts, "channels") ? parse(Int, opts["channels"]) : 0
    # The signal tuples a build knows, fixed at compile time; the command line
    # picks one, and each is its own code path so the core's type is concrete.
    if signal_choice == "GPSL1CA,GalileoE1B"
        run_with(
            io,
            (GPSL1CA(), GalileoE1B()),
            fs,
            channels,
            antenna,
            arm_lead,
            strobe,
            flush_us,
            epoch,
            segment_path,
            seconds,
            report_every,
            wait_ms,
        )
    elseif signal_choice == "GPSL1CA,GalileoE1B_BOC11"
        run_with(
            io,
            (GPSL1CA(), GalileoE1B_BOC11()),
            fs,
            channels,
            antenna,
            arm_lead,
            strobe,
            flush_us,
            epoch,
            segment_path,
            seconds,
            report_every,
            wait_ms,
        )
    else    # "GPSL1CA", checked above
        run_with(
            io,
            (GPSL1CA(),),
            fs,
            channels,
            antenna,
            arm_lead,
            strobe,
            flush_us,
            epoch,
            segment_path,
            seconds,
            report_every,
            wait_ms,
        )
    end
    report("stopped")
    return Cint(0)
end

function run_with(
    io::LitePCIeTDM,
    signals::Tuple,
    fs::Float64,
    channels::Int,
    antenna::Int,
    arm_lead::Int,
    strobe::Int,
    flush_us::Float64,
    epoch::Int,
    segment_path::String,
    seconds::Float64,
    report_every::Float64,
    wait_ms::Int,
)
    driver = TDMDriver(
        io,
        signals;
        fs,
        n_channels = channels,
        antenna,
        arm_lead_samples = arm_lead,
        strobe_period = strobe,
        flush_us,
    )
    start_device!(driver)
    report("channels", length(driver.channels))
    report("strobe_period", strobe)
    report("sample_count_at_start", sample_count(driver, 1))
    try
        serve(driver, signals, segment_path, epoch, seconds, report_every, wait_ms)
    finally
        stop_device!(driver)
        close(io.csr)
    end
    nothing
end
