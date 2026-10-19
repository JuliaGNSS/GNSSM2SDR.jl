using Test
using GNSSM2SDR
using GNSSM2SDR: TagDecoder, step!, _sext12, _read_raw!
using SignalChannels: SignalChannel

# The gateware's tagging (gnss_m2sdr/tdm/engine.py, RXTap) of one word.
function tag_word(index::Integer, i0, q0, i1, q1)
    nib(k) = UInt16((index >> (4k)) & 0xF)
    hi = UInt16((index >> (12 + 4 * ((index >> 8) & 0xF))) & 0xF)
    s(v) = reinterpret(UInt16, Int16(v)) & 0x0FFF
    (s(i0) | nib(0) << 12, s(q0) | nib(1) << 12, s(i1) | nib(2) << 12, s(q1) | hi << 12)
end

@testset "GNSSM2SDR" begin
    @testset "12-bit samples out of tagged slots" begin
        for v in (-2048, -1, 0, 1, 2047)
            w = tag_word(0x3456789, v, 0, 0, 0)
            @test _sext12(w[1]) == v
        end
    end

    @testset "the tag decoder names every word, through holes" begin
        first = 0x2_3456_7000 + 123
        idx = collect(first:(first + 30_000))
        deleteat!(idx, 9000:9500)       # a lost DMA buffer
        deleteat!(idx, 20000:21023)     # another
        dec = TagDecoder()
        out = Int64[]
        for i in idx
            w = tag_word(i, 1, 2, 3, 4)
            push!(out, step!(dec, w...))
        end
        locked = findfirst(>=(0), out)
        @test locked !== nothing && locked <= 8192
        @test out[locked:end] == idx[locked:end]
    end

    @testset "a corrupted high nibble breaks the lock instead of misnaming" begin
        first = 0x1_0000_0000
        dec = TagDecoder()
        for i in first:(first + 9000)
            step!(dec, tag_word(i, 0, 0, 0, 0)...)
        end
        @test dec.locked
        w = tag_word(first + 9001, 0, 0, 0, 0)
        bad = (w[1], w[2], w[3], w[4] ⊻ 0x8000)
        @test step!(dec, bad...) == -1
        @test !dec.locked
    end

    @testset "the reader through holes: exact again within a block of a long one" begin
        # Holes of whole DMA buffers (1024 words). From a whole tag block on
        # (4096, 5120, 8192), the low tag bits name them a block short.
        first = 0x2_3456_7000 + 123
        idx = collect(first:(first + 200_000))
        for (at, len) in ((150_000, 8192), (110_000, 5120), (70_000, 4096), (30_000, 1024))
            deleteat!(idx, at:(at + len - 1))
        end
        resize!(idx, length(idx) - length(idx) % 8192)
        value(i) = Int16(i % 2000 + 1)
        words = UInt16[]
        for i in idx
            append!(words, tag_word(i, value(i), -value(i), 0, 0))
        end
        path, io = mktemp()
        write(io, words)
        close(io)
        fd = ccall(:open, Cint, (Cstring, Cint), path, 0)
        chunk = 1000
        channel = SignalChannel{Complex{Int16},1}(chunk, 1000)
        origin, gaps, resyncs = Threads.Atomic{Int64}(-1), Threads.Atomic{Int64}(0), Threads.Atomic{Int64}(0)
        _read_raw!(channel, fd, chunk, 1000, 1, origin, gaps, resyncs)
        out = reduce(vcat, [vec(copy(m)) for m in channel])
        o = origin[]
        @test o >= first
        @test count(iszero, out) == gaps[]
        right(k) = iszero(out[k]) || out[k] == Complex{Int16}(value(o + k - 1), -value(o + k - 1))
        # Each of the three long holes misplaces at most a block of samples.
        @test count(!right, eachindex(out)) <= 3 * 4096
        # The 1024-sample hole is named exactly, and the stream is exact
        # again a block after the last long one.
        @test all(right, 1:(first + 69_999 - o))
        @test all(right, (first + 158_191 + 4096 - o):length(out))
        rm(path)
    end

    @testset "the loop command" begin
        cmd = gnss_loop_command(; csr = "/x/csr.csv", antenna = 2, executable = "/opt/gnss_loop", core = 3)
        @test cmd.exec[1] == "/opt/gnss_loop"
        @test "--antenna=2" in cmd.exec && "--fifo=50" in cmd.exec && "--core=3" in cmd.exec
    end
end
