using Test
using Random
using M2SDRLoop
using M2SDRLoop: CH_RUNNING, CH_PENDING, CH_FAILED, KIND_DUMP, KIND_STROBE, KIND_PAD, emulator_regs
using HardwareLoopCore
using HardwareLoopProtocol
using GNSSSignals
const HLP = HardwareLoopProtocol

const FS = 4e6
const L1 = 1575.42e6

# A satellite on one antenna: GPS L1 C/A `prn` at `doppler` (Hz) and `code_phase`
# (chips at sample 0), at `cn0` dBHz in unit-variance-per-component noise
# scaled by `sigma`, quantised to the AD9361's 12 bits.
function synthetic(n, first_sample; prn, doppler, code_phase, cn0, sigma = 60.0, rng)
    s = GPSL1CA()
    code_rate = 1.023e6 * (1 + doppler / L1)
    amp = sqrt(10^(cn0 / 10) * 2 * sigma^2 / FS)
    out = Vector{Complex{Int16}}(undef, n)
    for k = 1:n
        t = (first_sample + k - 1) / FS
        chip = get_code(s, code_phase + code_rate * t, prn)
        z = amp * chip * cis(2π * doppler * t) + sigma * complex(randn(rng), randn(rng))
        out[k] = Complex{Int16}(clamp(round(Int, real(z)), -2048, 2047), clamp(round(Int, imag(z)), -2048, 2047))
    end
    out
end

new_driver(emu; kwargs...) = TDMDriver(emu, emulator_regs(), (GPSL1CA(),); fs = FS, kwargs...)

@testset "M2SDRLoop" begin
    @testset "capabilities and tap layouts" begin
        caps = decode_capabilities(UInt32(24) | UInt32(25) << 8 | UInt32(5) << 16 | UInt32(32) << 20 | UInt32(12) << 26)
        @test caps.num_channels == 24 && caps.num_slots == 25 && caps.max_taps == 5 && caps.code_words == 4096
        @test tap_layout((Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3) == (2, (4, 2, 0, 0, 0), 4)
        @test tap_layout((Int32(-4), Int32(-2), Int32(0), Int32(2), Int32(4)), 5) == (4, (8, 6, 4, 2, 0), 8)
        @test isnothing(tap_layout((Int32(-40), Int32(0), Int32(2), Int32(0), Int32(0)), 3))
    end

    @testset "the driver refuses a gateware that is not the TDM correlator" begin
        emu = TDMEmulator(; n_channels = 4)
        reg_write(emu, emulator_regs().id, 0x12345678)
        @test_throws ArgumentError new_driver(emu)
    end

    @testset "arm, confirm, records, word, release" begin
        rng = MersenneTwister(1)
        emu = TDMEmulator(; n_channels = 4)
        d = new_driver(emu)
        start_device!(d)
        prn, doppler, code_phase = 7, 1200.0, 100.25
        spec = ArmSpec(GPSL1CA(), prn, doppler, doppler / 1540, code_phase, 0,
                       (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
        @test arm!(d, 2, spec) == ARM_ACCEPTED
        @test assignment_start(d, 2) == typemax(Int64)
        records = DeviceRecord[]
        n = 0
        for block = 1:12
            feed!(emu, synthetic(4000, n; prn, doppler, code_phase, cn0 = 50.0, rng))
            n += 4000
            read_records!(d, records)
        end
        @test d.channels[2].state == CH_RUNNING
        start = assignment_start(d, 2)
        @test start == d.arm_lead_samples + 4      # start + the three-tap warm-up
        dumps = filter(r -> !is_strobe(r), records)
        strobes = filter(is_strobe, records)
        @test length(strobes) == n ÷ d.strobe_period
        @test all(r -> r.channel == 2 && r.prn == prn && r.num_taps == 3, dumps)
        # Records tile the axis from the assignment start, one code period each.
        @test first(dumps).sample_index - first(dumps).integrated_samples == start
        for (a, b) in zip(dumps, dumps[2:end])
            @test b.sample_index - b.integrated_samples == a.sample_index
        end
        full = dumps[2:end]
        @test all(r -> abs(r.integrated_samples - 4000) <= 1, full)
        # The replica is aligned: the prompt dominates, early and late balance,
        # and the reported prompt code phase sits on the period boundary.
        for r in full
            late, prompt, early = abs(r.taps[1]), abs(r.taps[2]), abs(r.taps[3])
            @test prompt > 1.3 * late && prompt > 1.3 * early
            @test abs(early - late) / prompt < 0.3
            @test min(r.code_phase, 1023 - r.code_phase) < 0.3
        end
        # A word changes the carrier: 1 kHz off, the prompt nulls over 1 ms.
        @test write_word!(d, 2, doppler + 1000.0, doppler / 1540)
        before = d.words_written
        @test write_word!(d, 2, doppler + 1000.0, doppler / 1540)  # unchanged: no CSR writes
        @test d.words_written == before
        empty!(records)
        feed!(emu, synthetic(12000, n; prn, doppler, code_phase, cn0 = 50.0, rng))
        n += 12000
        read_records!(d, records)
        off = filter(r -> !is_strobe(r), records)[2:end]
        @test all(r -> abs(r.taps[2]) < 0.3 * abs(full[end].taps[2]), off)
        release!(d, 2)
        empty!(records)
        feed!(emu, synthetic(8000, n; prn, doppler, code_phase, cn0 = 50.0, rng))
        read_records!(d, records)
        @test all(is_strobe, records)
    end

    @testset "a start that passes before the arm lands is re-scheduled" begin
        emu = TDMEmulator(; n_channels = 2)
        d = new_driver(emu; arm_lead_samples = 10)
        start_device!(d)
        spec = ArmSpec(GPSL1CA(), 3, 0.0, 0.0, 0.0, 0,
                       (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
        arm!(d, 1, spec)
        records = DeviceRecord[]
        # Samples pass before each arm lands: three times the start is missed.
        for attempt = 1:3
            emu.sample_index += 1000
            feed!(emu, zeros(Complex{Int16}, 100))
            read_records!(d, records)
        end
        @test d.channels[1].state == CH_FAILED
        @test assignment_start(d, 1) == typemin(Int64)
        @test d.arm_retries == 2
    end

    @testset "gnss_loop refuses a signal list it was not built for" begin
        for signals in ("GalileoE1B", "GPSL1CA, GalileoE1B")
            @test redirect_stderr(devnull) do
                M2SDRLoop.loop_main(["--csr=/nonexistent/csr.csv", "--signals=$signals"])
            end == 2
        end
    end

    @testset "a dump of a pending assignment confirms it" begin
        rng = MersenneTwister(2)
        emu = TDMEmulator(; n_channels = 2)
        d = new_driver(emu)
        start_device!(d)
        spec = ArmSpec(GPSL1CA(), 5, 0.0, 0.0, 0.0, 0,
                       (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
        arm!(d, 1, spec)
        records = DeviceRecord[]
        feed!(emu, synthetic(16000, 0; prn = 5, doppler = 0.0, code_phase = 0.0, cn0 = 50.0, rng))
        # The start lands after the pass's confirmation read: skip it.
        d.any_pending = false
        read_records!(d, records)
        dumps = filter(r -> !is_strobe(r), records)
        @test d.channels[1].state == CH_RUNNING
        @test assignment_start(d, 1) == d.arm_lead_samples + 4
        @test d.stale_records == 0
        @test first(dumps).sample_index - first(dumps).integrated_samples == assignment_start(d, 1)
    end

    @testset "strobes on the period's multiples; records over lost samples flagged" begin
        rng = MersenneTwister(4)
        emu = TDMEmulator(; n_channels = 4)
        @test decode_capabilities(reg_read(emu, emulator_regs().caps)).num_slots == 7
        d = new_driver(emu; arm_lead_samples = 100)   # strobe period fs / 4000 = 1000
        start_device!(d)
        # Start off the period's multiples, lose 3 333 samples mid-way.
        emu.sample_index = 123_456
        spec = ArmSpec(GPSL1CA(), 9, 0.0, 0.0, 0.0, 0,
                       (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
        arm!(d, 1, spec)
        function drain()
            buf = zeros(UInt8, 1 << 20)
            n = M2SDRLoop.dma_read!(emu, buf, 0)
            [decode_record(buf, o) for o = 0:64:(n-64)]
        end
        recs = M2SDRLoop.TDMRecord[]
        sig = synthetic(20_000, 0; prn = 9, doppler = 0.0, code_phase = 0.0, cn0 = 50.0, rng)
        feed!(emu, sig[1:9000]); append!(recs, drain())
        emu.sample_index += 3333
        feed!(emu, sig[9001:end]); append!(recs, drain())
        strobes = [r.sample_index for r in recs if r.kind == KIND_STROBE]
        # The same strobes as gnss-m2sdr's model (test_tdm_engine.TestStrobeModel).
        @test first(strobes) == 124_000
        @test filter(s -> s % 1000 != 0, strobes) == [132_456 + 3333 + 1]
        @test allunique(strobes) && last(strobes) == 146_000
        dumps = [r for r in recs if r.kind == KIND_DUMP]
        lost = [r for r in dumps if r.flags & M2SDRLoop.FLAG_SAMPLE_OVERFLOW != 0]
        @test length(lost) == 1
        prev = dumps[findfirst(==(only(lost)), dumps) - 1]
        @test prev.sample_index <= 132_456 < only(lost).sample_index
        # A period set late strobes within a period (and the sync samples).
        reg_write(emu, emulator_regs().strobe_period, 0)
        feed!(emu, zeros(Complex{Int16}, 5000)); drain()
        reg_write(emu, emulator_regs().strobe_period, 700)
        at = emu.sample_index
        feed!(emu, zeros(Complex{Int16}, 2000))
        strobes = [r.sample_index for r in drain() if r.kind == KIND_STROBE]
        @test first(strobes) - at <= 700 + M2SDRLoop.STROBE_SYNC_SAMPLES && all(s -> s % 700 == 0, strobes)
    end

    @testset "the code cache loads each PRN once" begin
        emu = TDMEmulator(; n_channels = 4)
        d = new_driver(emu)
        start_device!(d)
        spec(prn) = ArmSpec(GPSL1CA(), prn, 0.0, 0.0, 0.0, 0,
                            (Int32(-2), Int32(0), Int32(2), Int32(0), Int32(0)), 3, 1, 1, 1, FS, 1.0, 1.0)
        arm!(d, 1, spec(5))
        arm!(d, 2, spec(6))
        arm!(d, 1, spec(5))
        @test d.code_loads == 2
        # PRN 5's code is in the memory as GNSSSignals has it.
        base = d.channels[1].code_base
        word = emu.code[base+1]
        @test all(k -> (((word >> k) & 1) == 1) == (get_code(GPSL1CA(), Float64(k), 5) < 0), 0:31)
    end

    @testset "closed loop: HardwareLoopCore through the driver and the emulator" begin
        rng = MersenneTwister(3)
        emu = TDMEmulator(; n_channels = 4)
        d = new_driver(emu)
        start_device!(d)
        seg = create_segment(nothing, SegmentConfig(; channel_count = 4, bands = driver_capabilities(d).bands))
        core = LoopCore(d, (GPSL1CA(),), seg)
        prn, doppler, code_phase = 11, -2350.0, 500.0
        # A handover 20 Hz and a quarter chip off.
        try_publish!(command_ring(seg), CommandTag(HLP.COMMAND_ARM, 1, 1), ArmCommand(;
            signal = :GPSL1CA, prn, carrier_doppler_hz = doppler + 20, code_doppler_hz = doppler / 1540,
            code_phase_chips = code_phase + 0.25, valid_at_sample = 0, tap_sample_shifts = (-2, 0, 2),
            num_taps = 3, sampling_freq_hz = FS))
        try_publish!(command_ring(seg), CommandTag(HLP.COMMAND_ARM, 2, 2), ArmCommand(;
            signal = :GPSL1CA, prn = 30, signal_index = 0, carrier_doppler_hz = 3000.0, code_doppler_hz = 0.0,
            code_phase_chips = 0.0, valid_at_sample = 0, tap_sample_shifts = (-2, 0, 2), num_taps = 3,
            sampling_freq_hz = FS))
        n = 0
        states = HLP.EpochStateEvent[]
        for block = 1:1500
            feed!(emu, synthetic(4000, n; prn, doppler, code_phase, cn0 = 45.0, rng))
            n += 4000
            service_pass!(core; wait_ms = 0)
            ring = event_ring(seg, 1)
            while true
                status, view, _ = peek!(ring, EventTag)
                status === :empty && break
                if view.tag.kind == HLP.EVENT_EPOCH_STATE
                    push!(states, payload(HLP.EpochStateEvent, ring, view))
                end
                commit!(ring, view)
            end
            while true
                status, view, _ = peek!(event_ring(seg, 2), EventTag)
                status === :empty && break
                commit!(event_ring(seg, 2), view)
            end
        end
        @test d.channels[1].state == CH_RUNNING
        @test core.words_committed > 1000
        @test core.lost_record_gaps == 0
        last_states = states[end-200:end]
        @test all(s -> abs(s.carrier_doppler_hz - doppler) < 5, last_states)
        cn0 = 10log10(last_states[end].cn0_linear_hz)
        @test 41 < cn0 < 49
        # The code phase the loop keeps is the satellite's, at the epoch boundary.
        s = last_states[end]
        @test s.flags & HLP.STATE_CODE_PHASE_ANCHORED != 0
    end
end
