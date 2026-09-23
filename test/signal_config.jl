# Per-channel signal configuration (GNSSM2SDR.jl#8, step 3 of
# GNSSReceiver.jl#130).
#
# The adapter used to inline GPS L1 C/A: `GPS_CA_CHIP_RATE` in the code NCO,
# `GPS_L1_HZ` as the carrier every Doppler was scaled against, `CA_CODE_LENGTH`
# as the modulus of every code phase, and a code cache keyed on the PRN alone.
# Each of those fails *silently* on another signal — the channel arms,
# correlates a plausible-looking nothing and never locks — so each gets a test
# that pins the number, not just the shape.
#
# Everything here runs without a board: the fixed-point conversions, the replica
# generation, the capability decoding and the record parsing are all plain
# functions of their inputs precisely so they can be.

using GNSSM2SDR:
    ChannelSignal,
    UNASSIGNED_SIGNAL,
    signal_key,
    code_step_word,
    code_word_from_code_doppler,
    decode_capabilities,
    decode_modulations,
    correlator_capabilities,
    code_phase_chips,
    code_chip_rate,
    RECORD_FORMAT_VERSION,
    CSR_LAYOUT_VERSION,
    CODE_FRAC_BITS

@testset "The code NCO step is the signal's own chip rate, not GPS L1 C/A's" begin
    fs = 30e6
    l1, l5 = GPSL1CA(), GPSL5I()

    # The acceptance number from the issue. 10.23 Mcps at 30 MHz is 0.341
    # chips/sample; the L1 C/A word is a tenth of that, and a channel programmed
    # with it runs its replica ten times too slowly.
    @test code_word(
        0.0,
        fs;
        code_frequency = ustrip(Hz, get_code_frequency(l5)),
        center_frequency = ustrip(Hz, get_center_frequency(l5)),
    ) == 5_721_031
    @test code_word(0.0, fs) == 572_103                      # GPS L1 C/A, unchanged
    @test code_step_word(10.23e6, fs) == 5_721_031

    # Code Doppler scales with the *signal's own* carrier. Using L1's
    # 1575.42 MHz for L5 mis-sizes the rate offset by 34 %. A large Doppler
    # here so the ratio is not dominated by the 2^-24 quantisation.
    fd = 1e5
    l5_word = code_word(
        fd,
        fs;
        code_frequency = 10.23e6,
        center_frequency = ustrip(Hz, get_center_frequency(l5)),
    )
    wrong_carrier =
        code_word(fd, fs; code_frequency = 10.23e6, center_frequency = GNSSM2SDR.GPS_L1_HZ)
    @test l5_word == round(Int64, 10.23e6 * (1 + fd / 1176.45e6) / fs * 2^24)
    @test l5_word != wrong_carrier
    @test (l5_word - 5_721_031) / (wrong_carrier - 5_721_031) ≈ 1575.42 / 1176.45 rtol =
        1e-3

    # A rate the NCO cannot represent must arrive as an error rather than be
    # masked into a plausible one. GPS L5 at the bring-up sample rate is the
    # concrete case: 2.5 chips/sample masks down to 0.5.
    @test_throws ArgumentError code_step_word(10.23e6, 4.092e6)
    @test_throws ArgumentError code_word(
        0.0,
        4.092e6;
        code_frequency = 10.23e6,
        center_frequency = 1176.45e6,
    )
    @test code_step_word(10.23e6, 30.72e6) > 0               # representable: no throw
    # A rate that rounds to zero is just as unusable as one that overflows.
    @test_throws ArgumentError code_step_word(0.1, 30e6)
end

@testset "Code phase wraps on the channel's own code length" begin
    # The acceptance number from the issue: chip 1500.25 of a 10230-chip code is
    # chip 1500.25, not 477.25.
    word = code_phase_word(1500.25; code_length = 10230)
    @test (word >> CODE_FRAC_BITS) == 1500
    @test (word & ((1 << CODE_FRAC_BITS) - 1)) == 1 << (CODE_FRAC_BITS - 2)
    @test word != code_phase_word(1500.25)                   # the L1 C/A wrap
    @test code_phase_word(1500.25) == code_phase_word(477.25) # …which is this

    # Wrapping still happens, on the right modulus, and a fraction that rounds
    # up to a whole chip carries into the next chip of *that* code.
    @test code_phase_word(10230 + 3.25; code_length = 10230) ==
          code_phase_word(3.25; code_length = 10230)
    @test code_phase_word(10229 + 1 - 2.0^-30; code_length = 10230) == 0
    @test code_phase_word(4092 + 7.5; code_length = 4092) ==
          code_phase_word(7.5; code_length = 4092)
end

@testset "A channel's signal configuration comes off the signal" begin
    s = ChannelSignal(GPSL5Q(), 7; signal_index = 2, replica_amplitude = 127.0)
    @test s.id == :GPSL5Q
    @test s.signal_index == 2
    @test s.prn == 7
    @test s.code_length == 10230
    @test s.code_frequency == 10.23e6
    @test s.center_frequency == 1176.45e6
    @test s.modulation == :LOC
    @test s.band_id == :L5
    @test s.code_amplitude == 1.0
    @test s.replica_amplitude == 127.0
    @test s.secondary_code_length == get_secondary_code_length(GPSL5Q())

    # The identity a replica is cached and reused by is the signal *and* the
    # PRN. A fresh channel matches neither, so its first assignment always loads.
    @test signal_key(s) == (:GPSL5Q, 7)
    @test signal_key(ChannelSignal(GPSL5I(), 7)) == (:GPSL5I, 7)
    @test signal_key(ChannelSignal(GalileoE1B(), 7)) == (:GalileoE1B, 7)
    @test signal_key(UNASSIGNED_SIGNAL) == (:none, 0)
    @test allunique(
        signal_key.([
            ChannelSignal(GPSL1CA(), 7),
            ChannelSignal(GalileoE1B(), 7),
            ChannelSignal(GPSL5I(), 7),
            ChannelSignal(GPSL5Q(), 7),
        ]),
    )
end

@testset "Channel-flavoured conversions read the channel's signal" begin
    # A channel carries its configuration, so nothing downstream needs to be
    # told which signal it is programming. Built without a device: the
    # conversions never touch `csr`.
    ch = GNSSBankChannel(undef_csr(), 0; fs = 30e6, signal = ChannelSignal(GPSL5I(), 3))
    @test code_word(ch, 0.0) == 5_721_031
    @test code_phase_word(ch, 1500.25) == code_phase_word(1500.25; code_length = 10230)
    # The E/L half-spacing is a whole number of NCO samples of *this* channel's
    # step, so the two fixed-point words cannot drift apart.
    @test spacing_word(ch, 2, 0.0) == 2 * 5_721_031
    # Re-assigning the channel to another signal moves every derived word.
    ch.signal = ChannelSignal(GPSL1CA(), 3)
    @test code_word(ch, 0.0) == 572_103
    @test code_phase_word(ch, 1500.25) == code_phase_word(477.25)
end

# The driver's code fill, as a vector: `_fill_code!` is what loads the code RAM.
primary_code(signal, prn) = GNSSM2SDR._fill_code!(Int[], signal, Int(prn))

@testset "The replica is the primary code, without an overlay chip baked in" begin
    # `get_code(signal, chip, prn)` multiplies in secondary chip 0. For BeiDou
    # B1I PRN 6 that chip is -1, so a replica built from it is the primary code
    # *inverted*: every accumulator's sign flips, the PLL locks 180° out and
    # every navigation bit decodes backwards. The overlay is the host's to
    # remove from the dumps once its phase is known, not something to bake in
    # before it is.
    signal, prn = BeiDouB1I(), 6
    @test GNSSSignals.secondary_value(get_secondary_code(signal), prn, 0) == -1
    chips = primary_code(signal, prn)
    @test length(chips) == get_code_length(signal) == 2046
    @test all(c -> c in (0, 1), chips)
    @test chips ==
          Int[GNSSSignals.get_code_at_index(signal, k, prn) > 0 ? 1 : 0 for k = 0:2045]
    @test chips != Int[get_code(signal, k, prn) > 0 ? 1 : 0 for k = 0:2045]

    # Where the overlay's first chip is +1 the two agree, which is exactly why
    # the bug was invisible on GPS L1 C/A (no overlay at all) and GPS L5I.
    for (sig, p) in ((GPSL1CA(), 1), (GPSL5I(), 1))
        @test primary_code(sig, p) ==
              Int[get_code(sig, k, p) > 0 ? 1 : 0 for k = 0:(get_code_length(sig)-1)]
    end

    # Full primary period, and the right one per signal.
    @test length(primary_code(GPSL1CA(), 1)) == 1023
    @test length(primary_code(GalileoE1B(), 1)) == 4092
    @test length(primary_code(GPSL5I(), 1)) == 10230
end

@testset "Tap offsets are programmed, not re-derived" begin
    # The receiver hands over every quantised replica offset, latest first with
    # the prompt at zero, and the device programs exactly those — one register
    # per tap. See `subchip_replica.jl` for the five-tap half of this, and
    # M2SDRLoop's own tests for the arm that writes them.
    fs = 30e6
    correlator = Tracking.get_default_correlator(GPSL5I(), Tracking.NumAnts(1))
    shifts = collect(Tracking.get_correlator_sample_shifts(correlator, fs, 10.23e6))
    # Latest first, prompt at the middle: the order the tap registers take.
    @test issorted(shifts)
    @test shifts[div(length(shifts), 2)+1] == 0
    # What Tracking quantises for a GPS L5 channel has to stay inside the
    # ±1-chip reach of the taps, which is what the offset word encodes.
    shift = last(shifts)
    @test shift * 5_721_031 < 1 << CODE_FRAC_BITS
    @test spacing_word(shift, 0.0, fs; code_frequency = 10.23e6) == shift * 5_721_031
    @test GNSSM2SDR.tap_offset_word(shift, 0.0, fs; code_frequency = 10.23e6) ==
          shift * 5_721_031
    # A whole chip is out of reach and is refused rather than wrapped onto the
    # wrong chip.
    @test_throws ArgumentError GNSSM2SDR.tap_offset_word(
        1000,
        0.0,
        fs;
        code_frequency = 10.23e6,
    )
end

# ── The versioned record ────────────────────────────────────────────────────

@testset "A version-2 record carries the whole code phase" begin
    bytes = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 4000,
            channel = 0,
            prn = 11,
            code_phase = 1 << (CODE_FRAC_BITS - 2),    # 0.25 chips
            code_phase_chip = 329,
            code_length = 330,
            code_step = 5_721_031,
            ants = [(1.0 + 0im, 1.0 + 0im, 1.0 + 0im)],
        ),
    )
    r = parse_record(bytes, 0, Val(1))
    @test r.version == RECORD_FORMAT_VERSION
    @test r.num_taps == 3
    @test r.code_phase_chip == 329
    @test r.code_length == 330
    @test r.code_step == 5_721_031
    # Explicitly 329.25, not the 1022.25 a host that infers `1023 - 1` reports.
    @test code_phase_chips(r, CODE_FRAC_BITS) == 329.25
    @test code_chip_rate(r, CODE_FRAC_BITS, 30e6) ≈ 10.23e6 rtol = 1e-6
end

@testset "A version-1 record refuses to invent a chip index" begin
    # The magic is deliberately unchanged between v1 and v2 — it anchors the
    # framing, and a host that cannot frame the stream cannot read the version
    # byte — so a v1 record still parses. Its reserved zeros must not be read as
    # data: "chip 0" is the most plausible-looking wrong answer available.
    #
    # Both `1` and `0` are tested because version 1 *left the byte reserved*:
    # gateware predating the field reads 0 there, which is what a live v1 board
    # actually streams (measured on the orin2 LiteX-M2SDR).
    for version in (1, 0)
        bytes = collect(
            pack_record(
                sample_index = 4000,
                integrated_samples = 4000,
                channel = 0,
                prn = 11,
                code_phase = 1 << (CODE_FRAC_BITS - 2),
                version = version,
                num_taps = 0,
                ants = [(1.0 + 0im, 1.0 + 0im, 1.0 + 0im)],
            ),
        )
        r = parse_record(bytes, 0, Val(1))
        @test is_record_start(bytes, 0)          # the magic did not move
        @test r.version == version
        @test r.code_phase == 1 << (CODE_FRAC_BITS - 2)
        @test r.code_phase_chip == 0
        @test r.code_length == 0
        @test r.code_step == 0
        @test_throws ArgumentError code_phase_chips(r, CODE_FRAC_BITS)
        @test_throws ArgumentError code_chip_rate(r, CODE_FRAC_BITS, 30e6)
    end
end

@testset "Epoch strobes carry the version but no code phase" begin
    bytes = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 0,
            channel = STROBE_CHANNEL,
            prn = 0,
            flags = FLAG_EPOCH_STROBE,
            num_taps = 0,
            ants = [(0.0 + 0im, 0.0 + 0im, 0.0 + 0im)],
            num_ants = 0,
        ),
    )
    r = parse_record(bytes, 0, Val(1))
    @test r.version == RECORD_FORMAT_VERSION   # describes the wire, not the payload
    @test r.num_taps == 0
    @test is_strobe(r)
end

# ── Capability discovery ────────────────────────────────────────────────────

# `gnss_capabilities` / `gnss_signal_caps` as the gateware packs them, so the
# field order is checked against the CSR definition rather than against itself.
caps_word(;
    n_channels,
    num_ants_max,
    num_taps,
    code_frac_bits,
    carrier_phase_bits,
    accum_bits,
    max_code_length,
) =
    UInt64(n_channels) | (UInt64(num_ants_max) << 8) | (UInt64(num_taps) << 16) |
    (UInt64(code_frac_bits) << 24) | (UInt64(carrier_phase_bits) << 32) |
    (UInt64(accum_bits) << 40) | (UInt64(max_code_length) << 48)

signal_caps_word(;
    modulations,
    max_secondary_code_length,
    reports_code_phase,
    tap_layouts = 0b0001,
    max_subchips = 1,
    replica_bits = 2,
) =
    UInt64(modulations) | (UInt64(max_secondary_code_length) << 8) |
    (UInt64(reports_code_phase ? 1 : 0) << 16) | (UInt64(tap_layouts) << 17) |
    (UInt64(max_subchips) << 21) | (UInt64(replica_bits) << 29)

@testset "The gateware's capability CSRs map onto GNSSReceiver's profile" begin
    fields = decode_capabilities(
        caps_word(
            n_channels = 8,
            num_ants_max = 2,
            num_taps = 3,
            code_frac_bits = 24,
            carrier_phase_bits = 32,
            accum_bits = 32,
            max_code_length = 10230,
        ),
        signal_caps_word(
            modulations = 0b0001,
            max_secondary_code_length = 1,
            reports_code_phase = true,
        ),
    )
    @test fields.n_channels == 8
    @test fields.num_ants_max == 2
    @test fields.num_taps == 3
    @test fields.code_frac_bits == 24
    @test fields.carrier_phase_bits == 32
    @test fields.accum_bits == 32
    @test fields.max_code_length == 10230
    @test fields.max_secondary_code_length == 1
    @test fields.reports_code_phase

    # Only the bits the gateware actually sets. An over-declared modulation is a
    # channel that arms and never locks.
    @test decode_modulations(0b0001) == [:LOC]
    @test decode_modulations(0b0000) == Symbol[]
    @test decode_modulations(0b1111) == [:LOC, :BOCcos, :CBOC, :TMBOC]
    @test fields.tap_layouts == [3]
    @test fields.max_subchips == 1
    @test fields.replica_bits == 2

    fs = 30e6
    caps = correlator_capabilities(fields, fs; num_antennas = 1)
    @test caps isa GNSSReceiver.HardwareCorrelatorCapabilities
    @test caps.modulations == [:LOC]
    @test caps.max_primary_code_length == 10230
    @test caps.tap_layouts == [3]
    @test caps.max_tap_offset_chips == 1.0
    @test caps.num_antennas == 1
    @test caps.max_secondary_code_length == 1
    @test caps.reports_code_phase
    # The code NCO's limits only exist relative to fs: the hardware bound is
    # chips per *sample*, not hertz.
    @test caps.code_frequency_limits == (fs / 2^24, fs * (2^24 - 1) / 2^24)
    # The gateware holds arbitrary chips and this driver generates them, so the
    # signal list is open; which band the front end is tuned to is RF-side
    # (GNSSReceiver.jl#134) and not in these registers.
    @test isnothing(caps.signals)
    @test isnothing(caps.bands)
    # A two-antenna build read by a one-antenna host is a one-antenna device.
    @test correlator_capabilities(fields, fs; num_antennas = 2).num_antennas == 2
    @test correlator_capabilities(fields, fs; num_antennas = 4).num_antennas == 2
end

@testset "The declared profile admits the signals it must and refuses the rest" begin
    fields = decode_capabilities(
        caps_word(
            n_channels = 4,
            num_ants_max = 1,
            num_taps = 3,
            code_frac_bits = 24,
            carrier_phase_bits = 32,
            accum_bits = 32,
            max_code_length = 10230,
        ),
        signal_caps_word(
            modulations = 0b0001,
            max_secondary_code_length = 1,
            reports_code_phase = true,
        ),
    )
    caps = correlator_capabilities(fields, 30e6; num_antennas = 1)
    support(signal, fs = 30e6) = GNSSReceiver.hardware_support_error(
        caps,
        signal,
        Tracking.get_default_correlator(signal, Tracking.NumAnts(1)),
        fs;
        dump_tap_slots = 3,
    )
    # GPS L1 C/A is the regression baseline for the whole chain.
    @test isnothing(support(GPSL1CA()))
    # The BPSK signals this step is about.
    @test isnothing(support(GPSL5I()))
    @test isnothing(support(GPSL5Q()))
    @test isnothing(support(BeiDouB1I()))
    @test isnothing(support(GalileoE5aI()))
    # A CBOC replica the gateware cannot synthesise is refused, by name,
    # before anything is armed (the BOC/five-tap work is gnss-m2sdr#30).
    e1b = support(GalileoE1B())
    @test !isnothing(e1b)
    @test occursin("CBOC", e1b)
    # A code past the build's code memory is refused too.
    small = correlator_capabilities(
        merge(fields, (max_code_length = 1023,)),
        30e6;
        num_antennas = 1,
    )
    @test !isnothing(
        GNSSReceiver.hardware_support_error(
            small,
            GPSL5I(),
            Tracking.get_default_correlator(GPSL5I(), Tracking.NumAnts(1)),
            30e6,
        ),
    )
    # And so is a chip rate the NCO cannot reach at this sample rate: GPS L5 at
    # the 4.092 MHz bring-up rate is 2.5 chips per input sample.
    slow = correlator_capabilities(fields, 4.092e6; num_antennas = 1)
    @test !isnothing(
        GNSSReceiver.hardware_support_error(
            slow,
            GPSL5I(),
            Tracking.get_default_correlator(GPSL5I(), Tracking.NumAnts(1)),
            4.092e6,
        ),
    )
    @test isnothing(
        GNSSReceiver.hardware_support_error(
            slow,
            GPSL1CA(),
            Tracking.get_default_correlator(GPSL1CA(), Tracking.NumAnts(1)),
            4.092e6;
            dump_tap_slots = 3,
        ),
    )
end

@testset "A gateware with no code-length register says so instead of staging" begin
    # On a build predating gnss-m2sdr#31 the primary-code length is a *build-time*
    # parameter of the code replica: there is no register to stage, and the only
    # correct code to load is one of exactly that length. The loop driver
    # refuses such a build outright, but the low-level bank API still has to be
    # usable by the GPS L1 C/A bring-up scripts that drive one by hand — so the
    # length staging is skipped where there is nothing to stage, and asking for
    # it explicitly is an error naming why.
    ch = GNSSBankChannel(undef_csr(), 0; fs = 4e6)
    @test !GNSSM2SDR.has_code_length_csr(ch)
    @test_throws ArgumentError GNSSM2SDR.set_code_length!(ch, 1023)
    modern = undef_csr()
    modern.regs["gnss_ch0_code_length"] = (UInt32(0x1000), 1)
    @test GNSSM2SDR.has_code_length_csr(GNSSBankChannel(modern, 0; fs = 4e6))
end

@testset "Gateware older than the record format is refused by name" begin
    # A build that predates runtime signal configuration answers `gnss_version`
    # with revision 1 — or does not answer at all, which is the same thing. It
    # cannot be told a code length or a chip rate and its records carry no chip
    # index, so this driver must not drive it: every failure it would produce is
    # a channel that arms and never locks.
    @test_throws ArgumentError GNSSM2SDR.gateware_capabilities(
        StubCSR(Dict{String,UInt64}()),
    )
    @test_throws ArgumentError GNSSM2SDR.gateware_capabilities(
        StubCSR(Dict("gnss_version" => UInt64(1) | (UInt64(1) << 8))),
    )
    err = try
        GNSSM2SDR.gateware_capabilities(StubCSR(Dict{String,UInt64}()))
        nothing
    catch e
        e
    end
    @test occursin("record format v2", sprint(showerror, err))

    # A CSR layout newer than this driver is refused just as firmly: an unknown
    # register set addressed by name reports whatever the fields line up with,
    # and the receiver would then validate satellites against a profile that is
    # not the device's.
    @test_throws ArgumentError GNSSM2SDR.gateware_capabilities(
        StubCSR(
            Dict(
                "gnss_version" =>
                    UInt64(CSR_LAYOUT_VERSION + 1) | (UInt64(RECORD_FORMAT_VERSION) << 8),
            ),
        ),
    )
    # …and so is an *older* one, by name. v2 streams the record format this
    # driver parses, so the record check passes it; its register set is the one
    # with a single symmetric `spacing` and no `tap_offset_*`, `replica`,
    # `subcarrier_load` or `dump_num_taps`, so nothing here could place a tap on
    # it. See `subchip_replica.jl` for the message.
    @test_throws ArgumentError GNSSM2SDR.gateware_capabilities(
        StubCSR(
            Dict(
                "gnss_version" =>
                    UInt64(CSR_LAYOUT_VERSION - 1) | (UInt64(RECORD_FORMAT_VERSION) << 8),
            ),
        ),
    )

    # A current build reads clean, and its fields reach the profile.
    ok = StubCSR(
        Dict(
            "gnss_version" =>
                UInt64(CSR_LAYOUT_VERSION) | (UInt64(RECORD_FORMAT_VERSION) << 8),
            "gnss_capabilities" => caps_word(
                n_channels = 4,
                num_ants_max = 1,
                num_taps = 3,
                code_frac_bits = 24,
                carrier_phase_bits = 32,
                accum_bits = 32,
                max_code_length = 10230,
            ),
            "gnss_signal_caps" => signal_caps_word(
                modulations = 0b0001,
                max_secondary_code_length = 1,
                reports_code_phase = true,
            ),
        ),
    )
    caps = GNSSM2SDR.gateware_capabilities(ok)
    @test caps.record_version == RECORD_FORMAT_VERSION
    @test caps.csr_version == CSR_LAYOUT_VERSION
    @test caps.max_code_length == 10230
    @test caps.code_frac_bits == 24
end

# ── The software reference the issue asks for ───────────────────────────────

@testset "A non-L1-C/A BPSK channel matches a software correlation" begin
    # The issue asks for a non-L1-C/A BPSK hardware channel compared against
    # software. Without a board, this is the software half: the replica and the
    # NCO words this adapter would program, correlated against a clean simulated
    # signal, must peak where the code phase says and fall off either side — the
    # same property the hardware comparison would check, minus the hardware.
    fs = 30e6
    signal, prn = BeiDouB1I(), 6            # 2046 chips, 2.046 Mcps, overlaid
    s = ChannelSignal(signal, prn)
    step = code_word(
        0.0,
        fs;
        code_frequency = s.code_frequency,
        center_frequency = s.center_frequency,
    )
    chips_per_sample = step / 2^CODE_FRAC_BITS
    @test chips_per_sample ≈ 2.046e6 / fs rtol = 1e-6

    # The replica the adapter loads, as ±1.
    replica = 2 .* primary_code(signal, prn) .- 1
    n = round(Int, s.code_length / chips_per_sample)          # one code period
    true_phase = 137.25

    # The received signal: the same primary code, sampled at the true phase.
    received = [
        Float64(
            GNSSSignals.get_code_at_index(
                signal,
                mod(floor(Int, true_phase + k * chips_per_sample), s.code_length),
                prn,
            ),
        ) for k = 0:(n-1)
    ]
    # Correlate with the replica at a candidate phase, the way the channel does.
    function correlate(phase)
        acc = 0.0
        for k = 0:(n-1)
            idx = mod(floor(Int, phase + k * chips_per_sample), s.code_length)
            acc += received[k+1] * replica[idx+1]
        end
        acc / n
    end
    prompt = correlate(true_phase)
    @test prompt ≈ 1.0 rtol = 1e-6
    # Early and Late sit one NCO sample either side, which is what the spacing
    # register programs.
    early = correlate(true_phase + chips_per_sample)
    late = correlate(true_phase - chips_per_sample)
    @test prompt > early && prompt > late
    @test early ≈ late rtol = 0.2
    # Off the peak there is no correlation to speak of.
    @test abs(correlate(true_phase + 400)) < 0.1

    # The same replica taken through `get_code` — i.e. with overlay chip 0
    # multiplied in — correlates to -1: the inversion this replica avoids.
    inverted = [Float64(get_code(signal, k, prn)) for k = 0:(s.code_length-1)]
    acc = 0.0
    for k = 0:(n-1)
        idx = mod(floor(Int, true_phase + k * chips_per_sample), s.code_length)
        acc += received[k+1] * inverted[idx+1]
    end
    @test acc / n ≈ -1.0 rtol = 1e-6

    # GPS L1 C/A stays the regression baseline: the same machinery, unchanged
    # numbers.
    l1 = ChannelSignal(GPSL1CA(), 1)
    @test code_word(
        0.0,
        4e6;
        code_frequency = l1.code_frequency,
        center_frequency = l1.center_frequency,
    ) == code_word(0.0, 4e6)
    @test code_phase_word(511.5; code_length = l1.code_length) == code_phase_word(511.5)
end
