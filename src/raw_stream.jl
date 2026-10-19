# The raw I/Q stream: `m2sdr_record` drains DMA0 into a pipe, and a task that
# owns its thread turns the pipe into chunks on a `SignalChannel`, each sample
# named by its device index.
#
# Two design points:
#
#   * The recorder is a separate process. Julia's GC, compilation, a stalled
#     event loop — none of it reaches the driver's ring; the recorder keeps
#     writing into the pipe, which is grown to tens of MB (about a second of
#     samples at 4 MS/s) so a slow reader backs into the pipe, not the ring.
#   * The gateware tags every word with its device sample index (the TDM
#     correlator's RXTap, `CONTROL_TAG`). The reader decodes the tags, so it
#     knows which device sample the first host sample is — the origin every
#     acquisition handover is stated against — and it fills a lost DMA buffer
#     with zeros, so host sample `k` is device sample `origin + k` for the
#     whole run. It holds each sample back by one tag block (about 1 ms at
#     4 MS/s) until the tags after it have confirmed its index.

const TAG_BLOCK = 4096

"""
    RawStream

A running raw sample stream, from [`start_raw_stream`](@ref). `channel` delivers
the chunks of one antenna as `Complex{Int16}` (12-bit values, tags removed);
`origin[]` is the device sample index of host sample 0 (`-1` until the first
chunk has been delivered); `gap_samples[]` counts the samples lost upstream and
replaced by zeros, `resyncs[]` the times the tags had to be locked onto again. `close` stops the recorder and waits for the reader.
"""
mutable struct RawStream{C<:SignalChannel{Complex{Int16},1}}
    const channel::C
    const recorder::Base.Process
    const reader::Task
    const origin::Threads.Atomic{Int64}
    const gap_samples::Threads.Atomic{Int64}
    const resyncs::Threads.Atomic{Int64}
end

"""
    device_sample_origin(stream; timeout = 10) -> Int64

The device sample index of the stream's first host sample, waiting for the
reader to establish it.
"""
function device_sample_origin(stream::RawStream; timeout::Real = 10)
    t0 = time()
    while stream.origin[] < 0
        time() - t0 > timeout && error("the raw stream has not established its device-sample origin")
        sleep(0.01)
    end
    stream.origin[]
end

"""
    enable_tagging!(csr)

Make the gateware tag every RX word with its device sample index. The bit
lives in the correlator's control register and survives the loop process
starting and stopping.
"""
function enable_tagging!(csr::LiteXCSR)
    regs = TDMRegs(csr)
    reg_write(LitePCIeTDM(csr), regs.control, reg_read(LitePCIeTDM(csr), regs.control) | M2SDRLoop.CONTROL_TAG)
    csr
end

"""
    start_raw_stream(; csr, chunk, capacity_chunks = 4000, device_num = 0, antenna = 1,
                       pipe_bytes = 32 * 2^20, recorder = "m2sdr_record", host_buffers = 4096)

Enable the gateware's sample tags through `csr` (a [`LiteXCSR`](@ref) or the
path of the gateware's `csr.csv`), start `recorder` on `/dev/m2sdr<device_num>`
and return a [`RawStream`](@ref) whose channel delivers `chunk`-sample frames of
`antenna` (1 = RX1, 2 = RX2).

`host_buffers` DMA buffers (8 KiB each; the default is about a second at
4 MS/s 2T2R) queue inside the recorder between its DMA reader and its write to
the pipe: an unprivileged pipe cannot grow past `/proc/sys/fs/pipe-max-size`
(1 MiB by default, 31 ms of samples), so without it a garbage collection in
this process longer than that costs the recorder DMA buffers.

The reader is a sticky task on the interactive pool that blocks in the kernel,
so it needs an interactive thread of its own: start Julia with `-t N,M`.
"""
function start_raw_stream(;
    csr::Union{LiteXCSR,AbstractString},
    chunk::Integer,
    capacity_chunks::Integer = 4000,
    device_num::Integer = 0,
    antenna::Integer = 1,
    pipe_bytes::Integer = 32 * 2^20,
    recorder::AbstractString = "m2sdr_record",
    host_buffers::Integer = 4096,
    command::Cmd = `$recorder -c $device_num -q --host-buffers $host_buffers - 0`,
)
    1 <= antenna <= 2 || throw(ArgumentError("antenna must be 1 (RX1) or 2 (RX2)"))
    chunk >= 1 || throw(ArgumentError("chunk must be at least 1 sample"))
    handle = csr isa LiteXCSR ? csr : LiteXCSR(csr; device = "/dev/m2sdr$device_num")
    enable_tagging!(handle)
    csr isa LiteXCSR || close(handle)
    proc, rfd = _spawn_into_pipe(command; pipe_bytes)
    channel = SignalChannel{Complex{Int16},1}(Int(chunk), Int(capacity_chunks))
    origin = Threads.Atomic{Int64}(-1)
    gaps = Threads.Atomic{Int64}(0)
    resyncs = Threads.Atomic{Int64}(0)
    reader = Threads.@spawn :interactive _read_raw!(channel, rfd, Int(chunk), Int(capacity_chunks), Int(antenna),
                                                    origin, gaps, resyncs)
    Base.errormonitor(reader)
    RawStream(channel, proc, reader, origin, gaps, resyncs)
end

"""
    _spawn_into_pipe(command; pipe_bytes) -> (process, read_fd)

Run `command` with its stdout on a fresh pipe grown to `pipe_bytes` and return
the process and the pipe's read end, a plain blocking descriptor to be read
with `read(2)` from a task that owns its thread.
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
    ccall(:close, Cint, (Cint,), wfd)
    process, rfd
end

@inline _sext12(x::UInt16) = Int16(reinterpret(Int16, x << 4) >> 4)
@inline _low12(w0::UInt16, w1::UInt16, w2::UInt16) =
    Int((w0 >> 12) | ((w1 >> 12) << 4) | ((w2 >> 12) << 8))

"""
    TagDecoder

The device index of each tagged word: the low 12 bits are in every word, the
bits above arrive one nibble per 256 samples over each aligned 4096-sample
block. Locks after one whole aligned block and from then on follows the words,
naming every gap.
"""
mutable struct TagDecoder
    locked::Bool
    expected::Int64          # the index the next word should carry
    # Assembling `index >> 12` from an aligned block.
    collecting::Bool
    block_pos::Int
    high::UInt64
    seen::UInt16
end

TagDecoder() = TagDecoder(false, 0, false, 0, 0, 0)

"""
    step!(dec, w) -> Int64

Advance over one word (four `UInt16` slots); the index of the word if the
decoder is locked, `-1` if not yet. A word whose low bits do not follow is a
gap: the decoder reports the word's own index (the caller fills the hole).
"""
function step!(dec::TagDecoder, w0::UInt16, w1::UInt16, w2::UInt16, w3::UInt16)
    low = _low12(w0, w1, w2)
    nib = UInt64(w3 >> 12)
    if dec.locked
        exp_low = Int(dec.expected & 0xFFF)
        idx = if low == exp_low
            dec.expected
        else
            # Taken as a hole shorter than a block: the low bits say how long.
            # A longer one is misnamed here until the nibble check below
            # catches it, within a block; the reader holds words back that long.
            dec.expected + mod(low - exp_low, TAG_BLOCK)
        end
        # Check the high nibble against the index we believe in.
        m = (idx >> 8) & 0xF
        if UInt64((idx >> (12 + 4m)) & 0xF) != nib
            dec.locked = false
            dec.collecting = false
            return Int64(-1)
        end
        dec.expected = idx + 1
        return idx
    end
    # Not locked: wait for a block start, then collect sixteen nibbles.
    if low == 0
        dec.collecting = true
        dec.block_pos = 0
        dec.high = 0
        dec.seen = 0
    end
    if dec.collecting
        if low != dec.block_pos
            dec.collecting = false
            return Int64(-1)
        end
        m = (low >> 8) & 0xF
        dec.high |= nib << (4m)
        dec.seen |= UInt16(1) << m
        dec.block_pos += 1
        if dec.block_pos == TAG_BLOCK
            dec.collecting = false
            dec.seen == 0xFFFF || return Int64(-1)
            dec.locked = true
            dec.expected = Int64(dec.high << 12) + TAG_BLOCK
            return dec.expected - 1
        end
    end
    Int64(-1)
end

function _read_raw!(channel::SignalChannel, fd::Cint, chunk::Int, capacity_chunks::Int, antenna::Int,
                    origin::Threads.Atomic{Int64}, gaps::Threads.Atomic{Int64}, resyncs::Threads.Atomic{Int64})
    current_task().sticky = true
    nbuf = capacity_chunks + 2
    pool = [Matrix{Complex{Int16}}(undef, chunk, 1) for _ = 1:nbuf]
    block_words = 8192
    raw = Vector{UInt8}(undef, block_words * 8)
    dec = TagDecoder()
    slot = 2 * (antenna - 1)
    # The hold-back: a word is delivered only after `TAG_BLOCK` more words have
    # passed the decoder's nibble check. A hole of a whole number of blocks
    # leaves the low bits unchanged and the decoder names the words after it
    # too low, but within one block their high nibbles disagree with that name
    # and the decoder unlocks; the words still held are then dropped and come
    # out as zeros, never under a wrong index.
    held_val = Vector{Complex{Int16}}(undef, TAG_BLOCK + 1)
    held_idx = Vector{Int64}(undef, TAG_BLOCK + 1)
    held_head = 1
    held_n = 0
    idx = 1
    fill_pos = 0
    next_index = Int64(-1)    # the device index the next output sample must be
    blocks = 0
    resyncing = false
    holes = 0
    try
        while isopen(channel)
            _read_exactly!(fd, raw) || break
            blocks += 1
            blocks == 64 && next_index < 0 && held_n == 0 &&
                @warn "the raw stream carries no sample tags yet; is the gateware's tag bit set (enable_tagging!)?"
            words = reinterpret(UInt16, raw)
            for k = 1:block_words
                w0 = words[4k-3]; w1 = words[4k-2]; w2 = words[4k-1]; w3 = words[4k]
                i = step!(dec, w0, w1, w2, w3)
                if i < 0
                    # Lost lock after it was established: start over; the held
                    # words and the samples until the next lock are lost.
                    if (next_index >= 0 || held_n > 0) && !resyncing
                        resyncing = true
                        Threads.atomic_add!(resyncs, 1)
                    end
                    held_n = 0
                    continue
                end
                resyncing = false
                # Re-delivered history after a resync.
                last = held_n > 0 ? held_idx[mod1(held_head + held_n - 1, TAG_BLOCK + 1)] : next_index - 1
                i <= last && continue
                tail = mod1(held_head + held_n, TAG_BLOCK + 1)
                held_val[tail] = Complex{Int16}(_sext12(words[4k-3+slot]), _sext12(words[4k-2+slot]))
                held_idx[tail] = i
                held_n += 1
                held_n > TAG_BLOCK || continue
                # The oldest held word is confirmed: deliver it.
                j = held_idx[held_head]
                v = held_val[held_head]
                held_head = mod1(held_head + 1, TAG_BLOCK + 1)
                held_n -= 1
                if next_index < 0
                    next_index = j
                    origin[] = j
                end
                # A hole: zeros keep host sample k at device sample origin + k.
                if next_index < j && holes < 20
                    holes += 1
                    @warn "raw stream: $(j - next_index) samples lost upstream at device sample $next_index"
                end
                while next_index < j
                    pool[idx][fill_pos+1, 1] = zero(Complex{Int16})
                    fill_pos += 1
                    next_index += 1
                    Threads.atomic_add!(gaps, 1)
                    if fill_pos == chunk
                        put!(channel, pool[idx]); idx = mod1(idx + 1, nbuf); fill_pos = 0
                    end
                end
                pool[idx][fill_pos+1, 1] = v
                fill_pos += 1
                next_index += 1
                if fill_pos == chunk
                    put!(channel, pool[idx])
                    idx = mod1(idx + 1, nbuf)
                    fill_pos = 0
                end
            end
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
    # The reader may be parked in `put!` on a full channel: closing it first
    # ends that, and the recorder's exit ends a `read`.
    close(stream.channel)
    process_running(stream.recorder) && kill(stream.recorder)
    wait(stream.recorder)
    wait(stream.reader)
    stream
end
