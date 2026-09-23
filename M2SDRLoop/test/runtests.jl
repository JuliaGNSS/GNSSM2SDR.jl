using Test
using M2SDRLoop
using HardwareLoopCore
using HardwareLoopProtocol
using GNSSSignals
using Unitful: Hz
const HLP = HardwareLoopProtocol

const FIXTURE = joinpath(@__DIR__, "fixtures", "csr_ch6_tap5_sub12.csv")
const FS = 4e6

# A shadow register file describing the six-channel five-tap build the fixture
# was recorded from: CSR layout 3, record format 2, 24 fractional code bits,
# 4092-chip code memory, LOC/CBOC/BOCsin replicas, 3- and 5-tap layouts, a
# 12-entry sub-chip table.
function shadow_board()
    csr = LiteXCSR(FIXTURE; device = nothing)
    write(csr, "gnss_version", 3 | (2 << 8))
    capabilities = UInt64(6) | UInt64(1) << 8 | UInt64(5) << 16 | UInt64(24) << 24 | UInt64(32) << 32 |
                   UInt64(32) << 40 | UInt64(4092) << 48
    signal_caps = UInt64(1 | 4 | 16) | UInt64(100) << 8 | UInt64(1) << 16 | UInt64(0b11) << 17 |
                  UInt64(12) << 21 | UInt64(8) << 29
    write(csr, "gnss_capabilities", capabilities)
    write(csr, "gnss_signal_caps", signal_caps)
    write(csr, "gnss_num_ants", 1)
    csr
end

set_sample_count!(csr, n) = write(csr, "gnss_sample_count", n)

# The host-side mirror of the gateware's record packer.
function pack_record(;
    sample_index, integrated_samples = 4000, channel = 0, prn = 7, flags = 0, seq = 0,
    code_phase_frac = 0, prompt = 0im, early = 0im, late = 0im, very_early = 0im, very_late = 0im,
    num_ants = 1, version = 2, num_taps = 3, code_phase_chip = 0, code_length = 1023, code_step = 0,
)
    words = zeros(UInt64, 16)
    iq(z) = (UInt64(reinterpret(UInt32, Int32(round(imag(z))))) << 32) |
            UInt64(reinterpret(UInt32, Int32(round(real(z)))))
    words[1] = UInt64(sample_index)
    words[2] = UInt64(integrated_samples) << 32 | UInt64(channel) << 24 | UInt64(prn) << 16 |
               UInt64(flags) << 8 | UInt64(seq)
    words[3] = iq(prompt)
    words[4] = iq(early)
    words[5] = iq(late)
    words[6] = UInt64(M2SDRLoop.RECORD_MAGIC) << 32 | UInt64(code_phase_frac)
    words[10] = UInt64(num_ants) | UInt64(version) << 8 | UInt64(num_taps) << 16
    words[11] = UInt64(code_phase_chip) | UInt64(code_length) << 32
    words[12] = UInt64(code_step)
    words[13] = iq(very_early)
    words[14] = iq(very_late)
    collect(reinterpret(UInt8, words))
end

# Feed raw bytes through the driver's parser as `read_records!` would.
function push_bytes!(dev, bytes)
    records = DeviceRecord[]
    copyto!(dev.buf, 1, bytes, 1, length(bytes))
    dev.filled = length(bytes)
    empty!(dev.raw_records)
    dev.filled = M2SDRLoop._take_records!(dev.raw_records, dev.buf, dev.filled, Val(1))
    for r in dev.raw_records
        M2SDRLoop._push_record!(dev, records, r)
    end
    records
end

shadow_driver(; kwargs...) = (csr = shadow_board(); (csr, M2SDRDriver(csr, (GPSL1CA(), GalileoE1B()); fs = FS, open_dma = false, handover_margin = 40_000, kwargs...)))

@testset "A shadow CSR round-trips named registers" begin
    csr = shadow_board()
    @test is_shadow(csr)
    @test read(csr, "gnss_version") == 3 | (2 << 8)
    write(csr, "gnss_ch0_apply_at", 0x1_2345_6789)
    @test read(csr, "gnss_ch0_apply_at") == 0x1_2345_6789
    @test read(csr, "gnss_ch3_carrier_freq") == 0
    caps = gateware_capabilities(csr)
    @test caps.n_channels == 6 && caps.num_taps == 5 && caps.code_frac_bits == 24
    @test caps.tap_layouts == [3, 5] && caps.max_subchips == 12 && caps.max_code_length == 4092
    @test detect_num_channels(csr) == 6
end

@testset "The driver describes the build" begin
    csr, dev = shadow_driver()
    caps = driver_capabilities(dev)
    @test caps.num_channels == 6 && caps.max_taps == 5 && caps.num_ants == 1
    @test length(caps.bands) == 1 && caps.bands[1].sampling_freq_hz == FS
    @test String(caps.bands[1].band_id) == String(get_band_id(get_band(GPSL1CA())))
    set_sample_count!(csr, 123_456)
    @test sample_count(dev, 1) == 123_456
end

@testset "Records come off the wire latest tap first, strobes as strobes" begin
    csr, dev = shadow_driver()
    bytes = vcat(
        pack_record(; sample_index = 4000, channel = 2, prn = 9, prompt = 100 + 10im, early = 50 - 5im,
                    late = 40 + 4im, code_phase_chip = 1022, code_phase_frac = 1 << 23),
        pack_record(; sample_index = 8000, channel = 0xFF, flags = M2SDRLoop.FLAG_EPOCH_STROBE, num_ants = 0),
        pack_record(; sample_index = 4000, channel = 2, prn = 9, prompt = 100 + 10im),   # a re-delivered buffer
        pack_record(; sample_index = 8000, channel = 1, prn = 3, num_taps = 5, prompt = 7im, early = 6im,
                    late = 5im, very_early = 8im, very_late = 4im),
    )
    records = push_bytes!(dev, bytes)
    @test length(records) == 3
    @test dev.duplicates == 1
    r = records[1]
    @test r.channel == 3 && r.prn == 9 && r.sample_index == 4000 && r.integrated_samples == 4000
    @test r.num_taps == 3
    @test r.taps[1:3] == (40 + 4im, 100 + 10im, 50 - 5im)     # late, prompt, early
    @test r.code_phase ≈ 1022.5
    @test HardwareLoopCore.is_strobe(records[2]) && records[2].sample_index == 8000
    five = records[3]
    @test five.num_taps == 5
    @test five.taps[1:5] == (4im, 5im, 7im, 6im, 8im)          # vl, l, p, e, ve
end

# The arm the receiver would send for GPS L1 C/A PRN 7 on channel 1.
function gps_spec(; prn = 7, doppler = 1000.0, code_phase = 100.5, valid_at = 900_000, num_taps = 3)
    ArmSpec(GPSL1CA(), prn, doppler, doppler / 1540, code_phase, Int64(valid_at),
            (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), num_taps, 1, 1, 1, FS, 1.0, 1.0)
end

@testset "An arm loads the code and schedules a verified handover" begin
    csr, dev = shadow_driver()
    set_sample_count!(csr, 1_000_000)
    spec = gps_spec()
    @test arm!(dev, 1, spec).accepted
    @test read(csr, "gnss_ch0_prn") == 7
    @test read(csr, "gnss_ch0_code_length") == 1023
    @test read(csr, "gnss_ch0_apply_at") == 1_040_000
    step = M2SDRLoop.code_word_from_code_doppler(spec.code_doppler_hz, FS, 24)
    @test read(csr, "gnss_ch0_code_freq_next") == step
    @test read(csr, "gnss_ch0_carrier_freq_next") == M2SDRLoop.carrier_word(1000.0, FS, 32)
    @test read(csr, "gnss_ch0_tap_offset_e") == M2SDRLoop.tap_offset_word(2, spec.code_doppler_hz, FS, 24)
    @test read(csr, "gnss_ch0_tap_offset_l") == M2SDRLoop.tap_offset_word(-2, spec.code_doppler_hz, FS, 24)
    expected_phase = mod(100.5 + (1.023e6 + spec.code_doppler_hz) * 140_000 / FS, 1023)
    @test read(csr, "gnss_ch0_code_phase") == M2SDRLoop.code_phase_word(expected_phase, 24; code_length = 1023)
    # The apply strobe was pulsed with every field flagged.
    @test read(csr, "gnss_ch0_apply") == 0b11111
    # Not confirmed before the target plus half the margin.
    @test assignment_start(dev, 1) == typemax(Int64)
    set_sample_count!(csr, 1_050_000)
    @test assignment_start(dev, 1) == typemax(Int64)
    set_sample_count!(csr, 1_060_000)
    @test assignment_start(dev, 1) == 1_040_000
    @test assignment_start(dev, 1) == 1_040_000
    # Words go to the immediate registers, and an unchanged word costs nothing.
    @test write_word!(dev, 1, 1234.5, 0.8)
    @test read(csr, "gnss_ch0_carrier_freq") == M2SDRLoop.carrier_word(1234.5, FS, 32)
    @test read(csr, "gnss_ch0_code_freq") == M2SDRLoop.code_word_from_code_doppler(0.8, FS, 24)
    @test dev.words_written == 1
    @test write_word!(dev, 1, 1234.5, 0.8)
    @test dev.words_written == 1
    # A re-arm of the same satellite does not reload the code RAM.
    loads_before = read(csr, "gnss_ch0_code_load")
    write(csr, "gnss_ch0_code_load", 0)
    @test arm!(dev, 1, gps_spec(; doppler = 1100.0)).accepted
    @test read(csr, "gnss_ch0_code_load") == 0
    # Another PRN does.
    @test arm!(dev, 1, gps_spec(; prn = 8)).accepted
    @test read(csr, "gnss_ch0_code_load") != 0
    release!(dev, 1)
    @test read(csr, "gnss_ch0_control") == 0
    @test !write_word!(dev, 1, 1.0, 0.0)
    @test assignment_start(dev, 1) == typemax(Int64)
end

@testset "A handover that never commits fails after three attempts" begin
    csr, dev = shadow_driver()
    set_sample_count!(csr, 1_000_000)
    @test arm!(dev, 1, gps_spec()).accepted
    write(csr, "gnss_ch0_apply_status", 0b01)     # still armed: the commit never landed
    for attempt = 1:3
        set_sample_count!(csr, read(csr, "gnss_ch0_apply_at") + 20_000)
        start = assignment_start(dev, 1)
        attempt < 3 ? (@test start == typemax(Int64)) : (@test start == typemin(Int64))
    end
    @test dev.channels[1].failed
end

@testset "Arms the build cannot serve are refused before any CSR write" begin
    csr, dev = shadow_driver()
    set_sample_count!(csr, 1_000_000)
    # Five taps on a three-tap-only request is fine; seven is not a layout.
    @test !arm!(dev, 1, gps_spec(; num_taps = 4)).accepted
    # A signal outside the build's tuple.
    spec = ArmSpec(GPSL5I(), 7, 0.0, 0.0, 0.0, Int64(0), (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
    @test arm!(dev, 1, spec).reason == HLP.REJECT_UNSUPPORTED_SIGNAL
    @test arm!(dev, 7, gps_spec()).reason == HLP.REJECT_NO_SUCH_CHANNEL
    @test read(csr, "gnss_ch0_prn") == 0
    # Galileo E1B is in the tuple and the build synthesises CBOC.
    e1b = ArmSpec(GalileoE1B(), 11, 0.0, 0.0, 0.0, Int64(0), (Int32(-2), Int32(-1), Int32(0), Int32(1), Int32(2)), 5, 1, 1, 1, FS, 1.0, Float64(get_code_amplitude(GalileoE1B())))
    @test arm!(dev, 2, e1b).accepted
    @test read(csr, "gnss_ch1_prn") == 11
    @test read(csr, "gnss_ch1_code_length") == 4092
end

@testset "The loop core arms and confirms through the driver" begin
    csr, dev = shadow_driver()
    set_sample_count!(csr, 1_000_000)
    seg = create_segment(nothing, SegmentConfig(; channel_count = 6, bands = dev.bands))
    core = LoopCore(dev, (GPSL1CA(), GalileoE1B()), seg)
    shifts = HardwareLoopCore._template_tap_shifts(core.banks[1].template, FS, GPSL1CA())
    cmd = ArmCommand(; signal = get_signal_id(GPSL1CA()), prn = 7, carrier_doppler_hz = 1000.0,
        code_doppler_hz = 1000.0 / 1540, code_phase_chips = 100.5, valid_at_sample = 900_000,
        tap_sample_shifts = shifts, num_taps = 3, sampling_freq_hz = FS)
    publish!(command_ring(seg), CommandTag(HLP.COMMAND_ARM, 1, 1), cmd)
    service_pass!(core; wait_ms = 0)
    @test read(csr, "gnss_ch0_prn") == 7
    @test core.channels.armed[1] && !core.channels.confirmed[1]
    set_sample_count!(csr, 1_070_000)
    service_pass!(core; wait_ms = 0)
    @test core.channels.confirmed[1]
    ring = event_ring(seg, 1)
    status, view, _ = peek!(ring, EventTag)
    @test status === :ok
    ev = payload(StatusEvent, ring, view)
    @test ev.code == HLP.STATUS_ARMED && ev.sample == 1_040_000 && ev.sequence == 1
    commit!(ring, view)
    # A handover the device cannot commit is reported as a rejected arm and
    # the channel freed.
    publish!(command_ring(seg), CommandTag(HLP.COMMAND_ARM, 2, 2), cmd)
    service_pass!(core; wait_ms = 0)
    write(csr, "gnss_ch1_apply_status", 0b01)
    for _ = 1:3
        set_sample_count!(csr, read(csr, "gnss_ch1_apply_at") + 20_000)
        service_pass!(core; wait_ms = 0)
    end
    @test !core.channels.armed[2]
    ring2 = event_ring(seg, 2)
    status, view, _ = peek!(ring2, EventTag)
    @test status === :ok
    ev = payload(StatusEvent, ring2, view)
    @test ev.code == HLP.STATUS_ARM_REJECTED && ev.reason == HLP.REJECT_DEVICE_ERROR && ev.sequence == 2
    @test read(csr, "gnss_ch1_control") == 0
    # A warm pass over the shadow board allocates nothing.
    service_pass!(core; wait_ms = 0)
    @test (@allocated service_pass!(core; wait_ms = 0)) == 0
end
