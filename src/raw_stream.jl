# The raw I/Q stream: `m2sdr_record` drains DMA0 into a pipe, and a task that
# owns its thread turns the pipe into chunks on a `SignalChannel`.
#
# The correlator bank only sees samples while DMA0 is being drained, so this
# stream is not optional for a hardware-correlator receiver, and it has to keep
# running whatever else the process is doing. Two design points follow:
#
#   * The recorder is a separate process. Julia's GC, compilation, a stalled
#     event loop — none of it reaches the driver's ring; the recorder keeps
#     writing into the pipe, which is grown to tens of MB (about a second of
#     samples at 4 MS/s) so a slow reader backs into the pipe, not the ring.
#   * The reader does not go through libuv. Julia services its event loop from
#     thread 1 unless that thread is blocked, so a libuv `read!` on the pipe
#     stops for as long as thread 1 is busy — an acquisition scan on the main
#     task, or any compilation the main task triggers, was enough to starve the
#     processing task of chunks and with it every NCO update (issue #107). The
#     reader here is a sticky interactive task that blocks in `read(2)` on the
#     pipe's own fd, in a `gc_safe` ccall: woken by the kernel, invisible to the
#     scheduler, and never in the way of a collection.

"""
    RawStream

A running raw sample stream, from [`start_raw_stream`](@ref). `channel` is the
`SignalChannel{Complex{Int16},1}` the chunks arrive on — hand it to
[`M2SDRRemote`](@ref) and to `GNSSReceiver.receive`. `close` stops the
recorder and waits for the reader; the channel closes with it.
"""
mutable struct RawStream{C<:SignalChannel{Complex{Int16},1}}
    const channel::C
    const recorder::Base.Process
    const reader::Task
end

"""
    start_raw_stream(; chunk, capacity_chunks = 4000, device_num = 0, antenna = 1,
                       pipe_bytes = 32 * 2^20, command = `m2sdr_record -c \$device_num -q - 0`)

Start `m2sdr_record` on `/dev/m2sdr<device_num>` and return a [`RawStream`](@ref)
whose channel delivers `chunk`-sample frames of one antenna as `Complex{Int16}`.

The recorder writes the board's 2R2T sc16 stream (four `Int16` per sample) into
a pipe grown to `pipe_bytes`; `antenna` (1 or 2) selects which receive chain the
chunks carry. `capacity_chunks` is the channel's depth. `command` is what to run;
the default is the stock recorder, and a test can substitute any producer of the
same byte layout.

The reader is a sticky task on the interactive pool that blocks in the kernel,
so it needs an interactive thread of its own: start Julia with `-t N,M`.
"""
function start_raw_stream(;
    chunk::Integer,
    capacity_chunks::Integer = 4000,
    device_num::Integer = 0,
    antenna::Integer = 1,
    pipe_bytes::Integer = 32 * 2^20,
    command::Cmd = `m2sdr_record -c $device_num -q - 0`,
)
    1 <= antenna <= N_ANTS_MAX ||
        throw(ArgumentError("antenna must be 1..$N_ANTS_MAX (the board is 2R2T)"))
    chunk >= 1 || throw(ArgumentError("chunk must be at least 1 sample"))
    recorder, rfd = _spawn_into_pipe(command; pipe_bytes)
    channel = SignalChannel{Complex{Int16},1}(Int(chunk), Int(capacity_chunks))
    reader = Threads.@spawn :interactive _read_raw!(
        channel,
        rfd,
        Int(chunk),
        Int(capacity_chunks),
        Int(antenna),
    )
    Base.errormonitor(reader)
    RawStream(channel, recorder, reader)
end

"""
    _spawn_into_pipe(command; pipe_bytes) -> (process, read_fd)

Run `command` with its stdout on a fresh pipe grown to `pipe_bytes` and return
the process and the pipe's read end. The pipe is the slack between a producer
that never stops (a recorder draining a DMA ring) and a reader that Julia may
hold up for a while — a GC pause, a compilation — so every byte of it is time
the driver's ring does not have to cover. The fd is a plain blocking descriptor,
not a libuv stream: read it with `read(2)` from a task that owns its thread.
"""
function _spawn_into_pipe(command::Cmd; pipe_bytes::Integer)
    fds = Vector{Cint}(undef, 2)
    rc = ccall(:pipe, Cint, (Ptr{Cint},), fds)
    rc == 0 || systemerror("pipe", Libc.errno())
    rfd, wfd = fds[1], fds[2]
    # F_SETPIPE_SZ (1031): ask for `pipe_bytes`, halve until the kernel agrees.
    let want = Int(pipe_bytes), got = -1
        while want >= 2^20
            got = ccall(:fcntl, Cint, (Cint, Cint, Cint), rfd, 1031, want)
            got > 0 && break
            want >>= 1
        end
        got > 0 || @warn "could not grow the pipe for $(command); a late reader will drop data"
    end
    process = run(pipeline(command; stdout = RawFD(wfd), stderr = devnull); wait = false)
    # The child holds the write end now; closing ours makes its exit an EOF for
    # the reader.
    ccall(:close, Cint, (Cint,), wfd)
    process, rfd
end

function _read_raw!(channel::SignalChannel, fd::Cint, chunk::Int, capacity_chunks::Int, antenna::Int)
    current_task().sticky = true
    # One more frame than the channel can hold, so the frame being filled is
    # never one the consumer may still be reading.
    nbuf = capacity_chunks + 2
    pool = [Matrix{Complex{Int16}}(undef, chunk, 1) for _ = 1:nbuf]
    raw = Vector{UInt8}(undef, chunk * 8)   # 4 × Int16 per sample (2R2T)
    offset = 2 * (antenna - 1)
    idx = 1
    try
        while isopen(channel)
            _read_exactly!(fd, raw) || break   # EOF: the recorder is gone
            words = reinterpret(Int16, raw)
            buf = pool[idx]
            @inbounds for k = 1:chunk
                buf[k, 1] = Complex(words[4k-3+offset], words[4k-2+offset])
            end
            put!(channel, buf)
            idx = mod1(idx + 1, nbuf)
        end
    catch e
        e isa InvalidStateException || rethrow()
    finally
        close(channel)
        ccall(:close, Cint, (Cint,), fd)
    end
end

# Fill `buf` from `fd` with blocking reads; `false` at end of file.
function _read_exactly!(fd::Cint, buf::Vector{UInt8})
    filled = 0
    GC.@preserve buf while filled < length(buf)
        n = @ccall gc_safe = true read(
            fd::Cint,
            (pointer(buf) + filled)::Ptr{UInt8},
            (length(buf) - filled)::Csize_t,
        )::Cssize_t
        if n < 0
            err = Libc.errno()
            err == Libc.EINTR && continue
            systemerror("read(raw stream)", err)
        end
        n == 0 && return false
        filled += Int(n)
    end
    true
end

function Base.close(stream::RawStream)
    process_running(stream.recorder) && kill(stream.recorder)
    wait(stream.recorder)
    wait(stream.reader)
    stream
end
