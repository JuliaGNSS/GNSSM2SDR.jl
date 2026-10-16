using Test
using GNSSM2SDR
using GNSSM2SDR: TagDecoder, step!, _sext12

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

    @testset "the loop command" begin
        cmd = gnss_loop_command(; csr = "/x/csr.csv", antenna = 2, executable = "/opt/gnss_loop", core = 3)
        @test cmd.exec[1] == "/opt/gnss_loop"
        @test "--antenna=2" in cmd.exec && "--fifo=50" in cmd.exec && "--core=3" in cmd.exec
    end
end
