using Test, GNSSM2SDR, GNSSReceiver, GNSSSignals, Tracking, Unitful, StaticArrays
using Unitful: Hz

using GNSSM2SDR:
    RECORD_BYTES,
    RECORD_WORDS,
    RECORD_MAGIC,
    MAGIC_WORD,
    MAGIC_SHIFT,
    NANTS_WORD,
    ANT_PROMPT_WORD,
    STROBE_CHANNEL,
    FLAG_EPOCH_STROBE,
    FLAG_OVERFLOW,
    M2SDRRecord,
    parse_record,
    parse_records!,
    find_record_offset,
    is_record_start,
    is_strobe,
    has_overflow,
    GNSSBankChannel,
    carrier_word,
    code_word,
    code_phase_word,
    carrier_phase_word,
    spacing_word,
    detect_num_channels,
    CA_CODE_LENGTH,
    GPS_CA_CHIP_RATE,
    GPS_L1_HZ

# A `LiteXCSR` handle with no device behind it. The fixed-point conversions are
# plain functions of (value, fs, width) and a channel's signal configuration, so
# they never reach the file descriptor — which is what lets every word the
# gateware is programmed with be checked without a board.
undef_csr() = GNSSM2SDR.LiteXCSR(
    RawFD(-1),
    Dict{String,Tuple{UInt32,Int}}(),
    Dict{String,UInt32}(),
    32,
    zeros(UInt8, GNSSM2SDR.REG_STRUCT_SIZE),
    ReentrantLock(),
    false,
    Dict{UInt32,UInt32}(),
)

# A recorded register map standing in for a flashed gateware, so the host's
# discovery and its refusals can be checked against a build that is not here.
struct StubCSR
    values::Dict{String,UInt64}
end
GNSSM2SDR.has_register(csr::StubCSR, name::AbstractString) = haskey(csr.values, name)
Base.read(csr::StubCSR, name::AbstractString) = csr.values[name]

u32(x) = UInt64(UInt32(x % UInt32))

# The host-side mirror of gnss-m2sdr's `pack_record`, so the parser is tested
# against the gateware's documented wire layout rather than against itself.
function pack_record(;
    sample_index,
    integrated_samples,
    channel,
    prn,
    seq = 0,
    flags = 0,
    code_phase = 0,
    ants,             # vector of (prompt, early, late) ComplexF64 tuples
    num_ants = length(ants),
    # Version-2 fields. Leaving them at 0 with `version = 1` reproduces the
    # version-1 layout byte for byte, which is what the compatibility tests
    # compare against.
    version = GNSSM2SDR.RECORD_FORMAT_VERSION,
    num_taps = 3,
    code_phase_chip = 0,
    code_length = 0,
    code_step = 0,
    # The five-tap tail: one (very_early, very_late) pair per antenna, in the
    # four words version 2 reserved. Written only when `num_taps` says 5, so a
    # three-tap record is byte for byte what it always was.
    very = fill((0.0 + 0im, 0.0 + 0im), length(ants)),
)
    words = zeros(UInt64, RECORD_WORDS)
    words[1] = UInt64(sample_index)
    words[2] =
        (UInt64(integrated_samples & 0xFFFFFFFF) << 32) | (UInt64(channel & 0xFF) << 24) |
        (UInt64(prn & 0xFF) << 16) | (UInt64(flags & 0xFF) << 8) | UInt64(seq & 0xFF)
    words[MAGIC_WORD+1] = (UInt64(RECORD_MAGIC) << MAGIC_SHIFT) | u32(code_phase)
    words[NANTS_WORD+1] =
        UInt64(num_ants & 0xFF) | (UInt64(version & 0xFF) << GNSSM2SDR.VERSION_SHIFT) |
        (UInt64(num_taps & 0xFF) << GNSSM2SDR.NUM_TAPS_SHIFT)
    words[GNSSM2SDR.CODE_WORD+1] =
        (u32(code_length) << GNSSM2SDR.CODE_LENGTH_SHIFT) | u32(code_phase_chip)
    words[GNSSM2SDR.CODE_STEP_WORD+1] = u32(code_step)
    for (n, (prompt, early, late)) in enumerate(ants)
        base = ANT_PROMPT_WORD[n] + 1
        words[base+0] =
            (u32(round(Int32, imag(prompt))) << 32) | u32(round(Int32, real(prompt)))
        words[base+1] =
            (u32(round(Int32, imag(early))) << 32) | u32(round(Int32, real(early)))
        words[base+2] =
            (u32(round(Int32, imag(late))) << 32) | u32(round(Int32, real(late)))
        if num_taps >= GNSSM2SDR.TAPS_VEPL
            very_early, very_late = very[n]
            base = GNSSM2SDR.ANT_VERY_WORD[n] + 1
            words[base+0] =
                (u32(round(Int32, imag(very_early))) << 32) |
                u32(round(Int32, real(very_early)))
            words[base+1] =
                (u32(round(Int32, imag(very_late))) << 32) |
                u32(round(Int32, real(very_late)))
        end
    end
    reinterpret(UInt8, words)
end

@testset "Record layout" begin
    @test RECORD_BYTES == 128
    # The framing property the whole DMA1 path rests on: a record divides the
    # DMA buffer exactly, so a dropped buffer costs whole records instead of
    # shifting every later one.
    @test 8192 % RECORD_BYTES == 0
end

@testset "Record round-trip" begin
    bytes = collect(
        pack_record(
            sample_index = 123_456_789,
            integrated_samples = 4000,
            channel = 2,
            prn = 24,
            seq = 7,
            code_phase = 0x00ABCDEF,
            ants = [(12.0 - 5.0im, 3.0 + 1.0im, -4.0 - 2.0im)],
        ),
    )
    @test is_record_start(bytes, 0)
    @test find_record_offset(bytes) == 0

    r = parse_record(bytes, 0, Val(1))
    @test r.sample_index == 123_456_789
    @test r.integrated_samples == 4000
    @test r.channel == 2
    @test r.prn == 24
    @test r.seq == 7
    @test r.code_phase == 0x00ABCDEF
    @test r.num_ants == 1
    @test r.prompt[1] == 12.0 - 5.0im
    @test r.early[1] == 3.0 + 1.0im
    @test r.late[1] == -4.0 - 2.0im
    @test !is_strobe(r)
    @test !has_overflow(r)
end

@testset "Negative accumulators survive the 32-bit two's complement" begin
    bytes = collect(
        pack_record(
            sample_index = 1,
            integrated_samples = 4000,
            channel = 0,
            prn = 1,
            ants = [(-12345.0 - 67890.0im, -1.0 + 0.0im, 2147483647.0 + 0.0im)],
        ),
    )
    r = parse_record(bytes, 0, Val(1))
    @test r.prompt[1] == -12345.0 - 67890.0im
    @test r.early[1] == -1.0 + 0.0im
    @test r.late[1] == 2147483647.0 + 0.0im
end

@testset "Two-antenna records read both blocks" begin
    bytes = collect(
        pack_record(
            sample_index = 8000,
            integrated_samples = 4000,
            channel = 1,
            prn = 5,
            ants = [
                (10.0 + 0im, 4.0 + 0im, 4.0 + 0im),
                (0.0 + 10.0im, 0.0 + 4.0im, 0.0 + 4.0im),
            ],
        ),
    )
    r = parse_record(bytes, 0, Val(2))
    @test r.num_ants == 2
    @test r.prompt == (10.0 + 0im, 0.0 + 10.0im)
    @test r.early == (4.0 + 0im, 0.0 + 4.0im)
end

@testset "Epoch strobes are recognised and carry no payload" begin
    bytes = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 0,
            channel = STROBE_CHANNEL,
            prn = 0,
            flags = FLAG_EPOCH_STROBE,
            ants = [(0.0 + 0im, 0.0 + 0im, 0.0 + 0im)],
            num_ants = 0,
        ),
    )
    r = parse_record(bytes, 0, Val(1))
    @test is_strobe(r)
    @test r.num_ants == 0
end

@testset "A record carries the prompt, early and late of each antenna" begin
    # The wire order is prompt/early/late; the loop's `[late, prompt, early]`
    # ordering is applied when the driver turns a record into a `DeviceRecord`
    # (M2SDRLoop's own tests pin that). Getting either backwards inverts the DLL
    # discriminator and the loop never converges, so both are pinned.
    bytes = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 4000,
            channel = 0,
            prn = 3,
            ants = [(100.0 + 0im, 40.0 + 0im, 20.0 + 0im)],  # prompt, early, late
        ),
    )
    r = parse_record(bytes, 0, Val(1))
    @test r.prompt[1] == 100.0 + 0im
    @test r.early[1] == 40.0 + 0im
    @test r.late[1] == 20.0 + 0im
    # Channel ids are 0-based in the gateware; the driver adds the one.
    @test r.channel == 0
    @test r.prn == 3
end

@testset "Parsing resynchronises instead of misparsing" begin
    good = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 4000,
            channel = 0,
            prn = 9,
            ants = [(1.0 + 0im, 1.0 + 0im, 1.0 + 0im)],
        ),
    )
    # Attach mid-record: 40 bytes of a previous record, then two whole ones.
    stream = vcat(good[1:40], good, good)
    records = M2SDRRecord{1}[]
    parse_records!(records, stream, Val(1))
    @test length(records) == 2
    @test all(r -> r.prn == 9, records)

    # Garbage where a record should be: skip it, keep the one that follows.
    torn = vcat(good, zeros(UInt8, RECORD_BYTES), good)
    records = M2SDRRecord{1}[]
    parse_records!(records, torn, Val(1))
    @test length(records) == 2
end

# ── Fixed-point word conversions ────────────────────────────────────────────
#
# These are the part that has to agree with the gateware bit for bit, and they
# are plain functions of (value, fs, width) precisely so they can be checked
# without a board to open.

@testset "NCO word conversions match the gateware's fixed point" begin
    fs = 4e6

    # Carrier: phase increment is fd/fs of a full 2^32 turn.
    @test carrier_word(0.0, fs) == 0
    @test carrier_word(1000.0, fs) == round(Int64, 1000 / fs * 2^32)

    # Code: the chip rate scales with carrier Doppler, fc = R_c(1 + fd/L1).
    @test code_word(0.0, fs) == round(Int64, GPS_CA_CHIP_RATE / fs * 2^24)
    fd = 1500.0
    expected = round(Int64, GPS_CA_CHIP_RATE * (1 + fd / GPS_L1_HZ) / fs * 2^24)
    @test code_word(fd, fs) == expected
    @test code_word(fd, fs) > code_word(0.0, fs)

    # Code phase splits into whole chip and fraction, and a fraction that rounds
    # up to a whole chip must carry rather than overflow the field.
    @test code_phase_word(0.0) == 0
    @test code_phase_word(5.5) == (Int64(5) << 24) | (Int64(1) << 23)
    @test code_phase_word(CA_CODE_LENGTH + 3.25) == code_phase_word(3.25)

    @test carrier_phase_word(0.5) == Int64(1) << 31
    @test carrier_phase_word(1.25) == carrier_phase_word(0.25)
end

@testset "E/L spacing is programmed as whole NCO samples" begin
    fs = 4e6
    step = code_word(0.0, fs)

    # Tracking hands over the Early-to-Late distance in samples; the CSR takes
    # the prompt→Early half, as an exact multiple of the code step so the two
    # fixed-point words cannot drift apart.
    @test spacing_word(2, 0.0, fs) == 2 * step
    @test spacing_word(1, 0.0, fs) == step

    # The E/L taps only reach chip index ±1, so a half-spacing of a whole chip
    # or more must be rejected rather than silently wrapping.
    @test_throws ArgumentError spacing_word(5, 0.0, fs)
end

# The DMA1 drain used to open /dev/m2sdr1 and read straight away. In litepcie's
# naming the *writer* is the FPGA→host direction, and the driver's read path
# waits on `writer_hw_count - writer_sw_count > 0` — a counter that only advances
# while that channel's DMA writer is enabled. So the read blocked forever and no
# correlator dump ever reached the receiver, with nothing reporting an error.
# These pin the ioctl encodings, because a wrong request number or struct size
# fails as a bare ENOTTY at run time, on hardware, and nowhere else.
@testset "litepcie DMA ioctl encodings match the kernel headers" begin
    # Verified against the target's own <sys/ioctl.h> on aarch64:
    #   sizeof: reg = 12, dma_writer = 24, lock = 6
    #   _IOWR('S',  0, struct litepcie_ioctl_reg)        = 0xc00c5300
    #   _IOWR('S', 21, struct litepcie_ioctl_dma_writer) = 0xc0185315
    #   _IOWR('S', 25, struct litepcie_ioctl_lock)       = 0xc0065319
    @test GNSSM2SDR.REG_STRUCT_SIZE == 12
    @test GNSSM2SDR.DMA_WRITER_STRUCT_SIZE == 24
    @test GNSSM2SDR.LOCK_STRUCT_SIZE == 6
    @test GNSSM2SDR.LITEPCIE_IOCTL_REG == 0xc00c5300
    @test GNSSM2SDR.LITEPCIE_IOCTL_DMA_WRITER == 0xc0185315
    @test GNSSM2SDR.LITEPCIE_IOCTL_LOCK == 0xc0065319
end

@testset "DMA reads are a whole number of driver buffers" begin
    # The driver's read path copies only in DMA_BUFFER_SIZE units
    # (`while (len >= DMA_BUFFER_SIZE)`), so a smaller request returns nothing at
    # all rather than a short read.
    @test GNSSM2SDR.DMA_BUFFER_SIZE == 8192
    @test GNSSM2SDR.DMA_BUFFER_SIZE % RECORD_BYTES == 0
    # 8192 / 128 = 64 records per buffer, so buffers never straddle a record.
    @test GNSSM2SDR.DMA_BUFFER_SIZE ÷ RECORD_BYTES == 64
end

@testset "Channel count is detected from the CSR map" begin
    # The gateware is the authority on how many channels exist; the host
    # counts the gnss_ch<i>_ banks instead of being told. Channels are
    # numbered consecutively from 0, so a gap ends the count.
    regs(chs) = Dict("gnss_ch$(i)_control" => (UInt32(0x1000 + 4i), 1) for i in chs)
    @test detect_num_channels(regs(0:19)) == 20
    @test detect_num_channels(regs(0:1)) == 2
    @test detect_num_channels(regs(1:4)) == 0     # no ch0: not a gnss build
    @test detect_num_channels(Dict{String,Tuple{UInt32,Int}}()) == 0
end

@testset "The raw stream delivers one antenna of the 2R2T pipe, then closes at EOF" begin
    # A file standing in for the recorder: 2R2T sc16, sample k carrying
    # (k, -k) on antenna 1 and (2k, -2k) on antenna 2, five chunks of 100.
    chunk = 100
    nchunks = 5
    words = Int16[]
    for k = 0:(chunk*nchunks-1)
        append!(words, Int16[k, -k, 2k, -2k])
    end
    path = tempname()
    write(path, reinterpret(UInt8, words))
    for antenna = 1:2
        stream =
            start_raw_stream(; chunk, capacity_chunks = 8, antenna, command = `cat $path`)
        frames = Matrix{Complex{Int16}}[]
        # The reader closes the channel once the producer's EOF arrives.
        try
            while true
                push!(frames, copy(take!(stream.channel)))
            end
        catch e
            e isa InvalidStateException || rethrow()
        end
        @test length(frames) == nchunks
        scale = antenna == 1 ? 1 : 2
        for (i, frame) in enumerate(frames)
            @test size(frame) == (chunk, 1)
            k0 = (i - 1) * chunk
            @test frame[:, 1] ==
                  [Complex{Int16}(scale * (k0 + j), -scale * (k0 + j)) for j = 0:(chunk-1)]
        end
        @test !isopen(stream.channel)
        close(stream)
        @test istaskdone(stream.reader)
    end
    rm(path)
end

@testset "Records cut by a pipe read boundary are reassembled in order" begin
    ants = [(1.0 + 2.0im, 3.0 + 4.0im, 5.0 + 6.0im)]
    stream = UInt8[]
    for k = 1:7
        r = pack_record(;
            sample_index = 1000k,
            integrated_samples = 4000,
            channel = k % 3,
            prn = k,
            ants,
        )
        append!(stream, r isa Vector{UInt8} ? r : collect(reinterpret(UInt8, r)))
    end
    # Feed the byte stream in awkward slices: not multiples of the record size,
    # some smaller than a record, one that ends exactly on a boundary.
    slices = [100, 27, 128, 300, 1, 5, 64, 200]
    buf = Vector{UInt8}(undef, 4096)
    filled = 0
    got = GNSSM2SDR.M2SDRRecord{1}[]
    pos = 0
    for n in slices
        n = min(n, length(stream) - pos)
        n == 0 && break
        copyto!(buf, filled + 1, stream, pos + 1, n)
        filled += n
        pos += n
        filled = GNSSM2SDR._take_records!(got, buf, filled, Val(1))
    end
    # The last slice list falls short of the stream: append the rest at once.
    rest = length(stream) - pos
    copyto!(buf, filled + 1, stream, pos + 1, rest)
    filled = GNSSM2SDR._take_records!(got, buf, filled + rest, Val(1))
    @test length(got) == 7
    @test [r.sample_index for r in got] == 1000 .* (1:7)
    @test [Int(r.prn) for r in got] == 1:7
    @test filled == 0
    # Garbage longer than a record ahead of a record is discarded down to a
    # record's worth, never left to grow.
    junk = zeros(UInt8, 3 * GNSSM2SDR.RECORD_BYTES)
    copyto!(buf, 1, junk, 1, length(junk))
    @test GNSSM2SDR._take_records!(got, buf, length(junk), Val(1)) ==
          GNSSM2SDR.RECORD_BYTES - 1
end

include("signal_config.jl")
include("subchip_replica.jl")
