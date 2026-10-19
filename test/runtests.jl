using Test
using GNSSM2SDR
using GNSSM2SDR: TagDecoder, step!, _sext12, _read_raw!
using M2SDRLoop: M2SDRLoop, TDMRegs, LitePCIeTDM, reg_read
using SignalChannels: SignalChannel

# The gateware's tagging (gnss_m2sdr/tdm/engine.py, RXTap) of one word.
function tag_word(index::Integer, i0, q0, i1, q1)
    nib(k) = UInt16((index >> (4k)) & 0xF)
    hi = UInt16((index >> (12 + 4 * ((index >> 8) & 0xF))) & 0xF)
    s(v) = reinterpret(UInt16, Int16(v)) & 0x0FFF
    (s(i0) | nib(0) << 12, s(q0) | nib(1) << 12, s(i1) | nib(2) << 12, s(q1) | hi << 12)
end

# The tagged words of device samples `indices`, as the recorder writes them.
tagged_words(indices, value) =
    reduce(vcat, [collect(tag_word(i, value(i), -value(i), 0, 0)) for i in indices])

# A device-less CSR handle on the correlator's registers, laid out as the emulator's.
function shadow_csr()
    regs = M2SDRLoop.emulator_regs()
    path, io = mktemp()
    for (k, name) in enumerate(M2SDRLoop.TDM_REG_NAMES)
        println(io, "csr_register,gnss_$name,$(getfield(regs, k)),1,rw")
    end
    close(io)
    LiteXCSR(path; device = nothing)
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
        idx = collect(first:(first+30_000))
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
        for i = first:(first+9000)
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
        idx = collect(first:(first+200_000))
        for (at, len) in ((150_000, 8192), (110_000, 5120), (70_000, 4096), (30_000, 1024))
            deleteat!(idx, at:(at+len-1))
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
        origin, gaps, resyncs =
            Threads.Atomic{Int64}(-1), Threads.Atomic{Int64}(0), Threads.Atomic{Int64}(0)
        _read_raw!(channel, fd, chunk, 1000, 1, origin, gaps, resyncs)
        out = reduce(vcat, [vec(copy(m)) for m in channel])
        o = origin[]
        @test o >= first
        @test count(iszero, out) == gaps[]
        right(k) =
            iszero(out[k]) || out[k] == Complex{Int16}(value(o + k - 1), -value(o + k - 1))
        # Each of the three long holes misplaces at most a block of samples.
        @test count(!right, eachindex(out)) <= 3 * 4096
        # The 1024-sample hole is named exactly, and the stream is exact
        # again a block after the last long one.
        @test all(right, 1:(first+69_999-o))
        @test all(right, (first+158_191+4096-o):length(out))
        rm(path)
    end

    @testset "the loop command" begin
        cmd = gnss_loop_command(;
            csr = "/x/csr.csv",
            antenna = 2,
            executable = "/opt/gnss_loop",
            core = 3,
        )
        @test cmd.exec[1] == "/opt/gnss_loop"
        @test "--antenna=2" in cmd.exec && "--fifo=50" in cmd.exec && "--core=3" in cmd.exec
    end

    @testset "tagging survives in the control register" begin
        csr = shadow_csr()
        control = TDMRegs(csr).control
        M2SDRLoop.writel(csr, control, M2SDRLoop.CONTROL_ENABLE)
        @test enable_tagging!(csr) === csr
        @test M2SDRLoop.readl(csr, control) ==
              M2SDRLoop.CONTROL_ENABLE | M2SDRLoop.CONTROL_TAG
    end

    @testset "the raw stream from a recorder process" begin
        first = 0x5_0000_0000 + 77
        value(i) = Int16(i % 1500 + 1)
        path, io = mktemp()
        write(io, tagged_words(first:(first+8192*4-1), value))
        close(io)
        csr = shadow_csr()
        stream = start_raw_stream(; csr, chunk = 1000, antenna = 2, command = `cat $path`)
        @test enable_tagging!(csr) === csr
        @test M2SDRLoop.readl(csr, TDMRegs(csr).control) & M2SDRLoop.CONTROL_TAG != 0
        out = reduce(vcat, [vec(copy(m)) for m in stream.channel])
        o = device_sample_origin(stream)
        @test o >= first
        @test stream.gap_samples[] == 0 && stream.resyncs[] == 0
        # Antenna 2 is the second slot pair of each word: zeros here.
        @test all(iszero, out)
        @test close(stream) === stream
        # RX1 carries the samples, from the origin on.
        stream = start_raw_stream(; csr, chunk = 1000, antenna = 1, command = `cat $path`)
        out = reduce(vcat, [vec(copy(m)) for m in stream.channel])
        o = device_sample_origin(stream)
        @test out == [Complex{Int16}(value(o + k), -value(o + k)) for k = 0:(length(out)-1)]
        close(stream)
        rm(path)
    end

    @testset "a raw stream that never names its first sample" begin
        stream = start_raw_stream(; csr = shadow_csr(), chunk = 1000, command = `sleep 30`)
        @test_throws ErrorException device_sample_origin(stream; timeout = 0.1)
        # Closing stops the recorder, and the reader with it.
        close(stream)
        @test !process_running(stream.recorder) && istaskdone(stream.reader)
    end

    @testset "untagged samples are pointed out" begin
        path, io = mktemp()
        write(io, zeros(UInt8, 64 * 8192 * 8))
        close(io)
        fd = ccall(:open, Cint, (Cstring, Cint), path, 0)
        channel = SignalChannel{Complex{Int16},1}(1000, 10)
        atomics =
            (Threads.Atomic{Int64}(-1), Threads.Atomic{Int64}(0), Threads.Atomic{Int64}(0))
        @test_logs (:warn, r"no sample tags") _read_raw!(
            channel,
            fd,
            1000,
            10,
            1,
            atomics...,
        )
        @test atomics[1][] == -1 && !isopen(channel)
        rm(path)
    end

    @testset "raw stream arguments" begin
        csr = shadow_csr()
        @test_throws ArgumentError start_raw_stream(; csr, chunk = 1000, antenna = 3)
        @test_throws ArgumentError start_raw_stream(; csr, chunk = 0)
        # A csr.csv path opens the board's CSR device.
        @test_throws SystemError start_raw_stream(;
            csr = "/nonexistent/csr.csv",
            chunk = 1000,
            device_num = 99,
        )
    end

    @testset "the loop command, logged" begin
        @test endswith(default_loop_executable(), joinpath("build", "gnss_loop"))
        cmd = gnss_loop_command(;
            csr = "/x/csr.csv",
            executable = "/opt/gnss_loop",
            fifo = nothing,
            channels = 8,
            log = "/tmp/loop.log",
            extra = ["--seconds=5"],
        )
        @test cmd isa Base.CmdRedirect
        exec = cmd.cmd.cmd.exec
        @test "--channels=8" in exec && "--seconds=5" in exec
        @test !any(startswith("--fifo"), exec)
    end
end
