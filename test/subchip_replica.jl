# Sub-chip BOC/CBOC/TMBOC replicas and five-tap channels (GNSSM2SDR.jl#10,
# step 5 of GNSSReceiver.jl#130).
#
# The gateware half landed as gnss-m2sdr#32 and bumped `CSR_LAYOUT_VERSION` to
# 3. Everything here is the host half of `docs/subchip_modulation.md` §5, and
# every one of these failures is silent on the wire rather than loud: a table
# one sub-chip out, a tap array programmed early-first, a five-tap record read
# as three, `:BOCsin` declared on the bit v2 reserved for `:BOCcos`. Each of
# them arms a channel that correlates a plausible-looking nothing.
#
# None of it needs a board. The subcarrier tables are functions of the
# GNSSSignals modulation, the CSR words are functions of the build's own
# reported limits, and a recording CSR stands in for the register map.

using GNSSM2SDR:
    subchip_factor,
    replica_shape,
    lut_rms,
    subcarrier_select_bits,
    ReplicaShape,
    decode_tap_layouts,
    decode_modulations,
    decode_capabilities,
    correlator_capabilities,
    gateware_capabilities,
    tap_offset_word,
    set_tap_offsets!,
    set_spacing_chips!,
    write_subcarrier!,
    set_replica!,
    load_replica_shape!,
    load_code!,
    code_status,
    ChannelSignal,
    GNSSBankChannel,
    TAPS_EPL,
    TAPS_VEPL,
    TAP_LAYOUTS,
    CODE_FRAC_BITS

# ── A register map that records instead of driving a board ──────────────────
#
# `GNSSBankChannel` is parametric in its CSR handle precisely so this works:
# every word the gateware is programmed with can then be checked against the
# gateware's own field packing without a bitstream in the room.
struct RecordingCSR
    writes::Vector{Pair{String,UInt64}}
    values::Dict{String,UInt64}
    registers::Set{String}
end
RecordingCSR(; registers = String[]) =
    RecordingCSR(Pair{String,UInt64}[], Dict{String,UInt64}(), Set{String}(registers))

GNSSM2SDR.has_register(csr::RecordingCSR, name::AbstractString) =
    isempty(csr.registers) || name in csr.registers
Base.read(csr::RecordingCSR, name::AbstractString) = get(csr.values, name, UInt64(0))
function Base.write(csr::RecordingCSR, name::AbstractString, value::Integer)
    push!(csr.writes, String(name) => UInt64(value) & typemax(UInt64))
    csr.values[String(name)] = UInt64(value)
    nothing
end

written(csr::RecordingCSR, name::AbstractString) =
    [v for (n, v) in csr.writes if n == "gnss_ch0_" * name]
wrote(csr::RecordingCSR, name::AbstractString) = !isempty(written(csr, name))

# A device that is nothing but its declared capabilities — enough for
# `validate_hardware_configuration`, which is the pre-arm gate the receiver
# actually runs.
struct StubHardwareSDR <: GNSSReceiver.AbstractHardwareCorrelatorSDR
    capabilities::GNSSReceiver.HardwareCorrelatorCapabilities
    n_channels::Int
end
StubHardwareSDR(capabilities; n_channels::Integer = 4) =
    StubHardwareSDR(capabilities, Int(n_channels))
GNSSReceiver.hardware_capabilities(sdr::StubHardwareSDR) = sdr.capabilities
GNSSReceiver.num_hardware_channels(sdr::StubHardwareSDR) = sdr.n_channels
GNSSReceiver.raw_sample_channel(::StubHardwareSDR) =
    GNSSReceiver.SignalChannel{ComplexF64,1}(4000, 4)

# A five-tap, 12-sub-chip channel: the build gnss-m2sdr#32 describes.
boc_channel(; fs = 20e6, signal = ChannelSignal(GalileoE1B(), 1), max_subchips = 12) =
    GNSSBankChannel(
        RecordingCSR(),
        0;
        fs,
        num_taps = TAPS_VEPL,
        max_subchips,
        replica_bits = 8,
        signal,
    )

# ── The Python reference: gnss-m2sdr's own `gnss_m2sdr/subcarrier.py` ───────
#
# The tables are *compared against the gateware repository*, not against a
# second copy of its constants living here. Two independently written tables
# drift, and a drifted subcarrier is a channel that arms and never locks — so
# the reference is the file that also produced the gateware's golden data.
#
# The checkout is found through GNSS_M2SDR_DIR, or next to this one, and a bare
# repository at the wrong branch still works: the file is read out of
# `origin/main` with `git show`.
function _m2sdr_repo()
    candidates = String[]
    haskey(ENV, "GNSS_M2SDR_DIR") && push!(candidates, ENV["GNSS_M2SDR_DIR"])
    here = dirname(@__DIR__)
    append!(
        candidates,
        [
            joinpath(dirname(here), "gnss-m2sdr"),
            joinpath(homedir(), "Code", "gnss-m2sdr"),
            joinpath(homedir(), "gnss-m2sdr"),
        ],
    )
    for dir in candidates
        isdir(dir) || continue
        isdir(joinpath(dir, ".git")) ||
            isfile(joinpath(dir, "gnss_m2sdr", "subcarrier.py")) ||
            continue
        return dir
    end
    nothing
end

# `gnss_m2sdr/subcarrier.py` as a standalone file, from the worktree if it is
# checked out there and from `origin/main` otherwise. Returns a directory that
# can be put on PYTHONPATH as the `gnss_m2sdr` package, or `nothing`.
function _subcarrier_module_dir()
    repo = _m2sdr_repo()
    isnothing(repo) && return nothing
    insitu = joinpath(repo, "gnss_m2sdr", "subcarrier.py")
    isfile(insitu) && return repo
    staged = mktempdir()
    mkpath(joinpath(staged, "gnss_m2sdr"))
    try
        open(joinpath(staged, "gnss_m2sdr", "subcarrier.py"), "w") do io
            run(
                pipeline(
                    `git -C $repo show origin/main:gnss_m2sdr/subcarrier.py`;
                    stdout = io,
                ),
            )
        end
    catch
        return nothing
    end
    touch(joinpath(staged, "gnss_m2sdr", "__init__.py"))
    staged
end

# Run `script` with `gnss_m2sdr.subcarrier` importable; the script prints one
# JSON-ish line per query, which is parsed with `Meta.parse` (the reference
# emits Julia-compatible literals, so nothing has to be added as a dependency).
function _python_reference(script::AbstractString, moduledir::AbstractString)
    out = IOBuffer()
    env = copy(ENV)
    env["PYTHONPATH"] =
        haskey(env, "PYTHONPATH") ? moduledir * ":" * env["PYTHONPATH"] : moduledir
    run(pipeline(setenv(`python3 -c $script`, env); stdout = out))
    eval(Meta.parse(String(take!(out))))
end

const L1_SIGNALS = (
    ("GPSL1CA", GPSL1CA()),
    ("GalileoE1B", GalileoE1B()),
    ("GalileoE1C", GalileoE1C()),
    ("GalileoE1B_BOC11", GalileoE1B_BOC11()),
    ("GalileoE1C_BOC11", GalileoE1C_BOC11()),
    ("GPSL1C_D", GPSL1C_D()),
    ("GPSL1C_P", GPSL1C_P()),
    ("BeiDouB1C_D", BeiDouB1C_D()),
    ("BeiDouB1C_P", BeiDouB1C_P()),
)

@testset "The sub-chip factor is the signal's own, not the family's" begin
    # One modulation bit cannot say which order of a family a build can hold, so
    # this number is what `max_subchips` is checked against. Getting it wrong
    # does not error in the gateware either: it raises
    # `code_status.replica_unsupported` and stops that channel dumping.
    @test subchip_factor(GNSSSignals.LOC()) == 1
    @test subchip_factor(GNSSSignals.BOCsin(1, 1)) == 2
    @test subchip_factor(GNSSSignals.BOCsin(6, 1)) == 12
    @test subchip_factor(GNSSSignals.BOCcos(1, 1)) == 4     # the quarter-sub-chip grid
    @test subchip_factor(
        GNSSSignals.CBOC(GNSSSignals.BOCsin(1, 1), GNSSSignals.BOCsin(6, 1), 10 / 11),
    ) == 12
    @test subchip_factor(get_modulation(GPSL1C_P())) == 12  # TMBOC(6,1,4/33)

    @test subchip_factor(GPSL1CA()) == 1
    @test subchip_factor(GalileoE1B()) == 12
    @test subchip_factor(GalileoE1B_BOC11()) == 2
    @test subchip_factor(BeiDouB1C_P()) == 2

    # The whole point of carrying it per signal: `:BOCsin` covers both of these
    # and they need different tables.
    @test subchip_factor(GNSSSignals.BOCsin(1, 1)) !=
          subchip_factor(GNSSSignals.BOCsin(6, 1))
end

@testset "Every subcarrier table reproduces GNSSSignals at every phase" begin
    # The table is indexed by `floor(frac · P)`, so it is only correct if the
    # modulation really is constant across each sub-chip — and a `P = 12`
    # subcarrier changes at multiples of 1/12 chip, which is not a dyadic
    # rational. Sample the whole chip, not the sub-chip midpoints the table was
    # built from, so a table that is a sub-chip out fails here.
    for (_, signal) in L1_SIGNALS
        shape = replica_shape(signal)
        modulation = get_modulation(signal)
        amplitude = get_code_amplitude(signal)
        @test length(shape.lut_a) == shape.subchips == subchip_factor(signal)
        modulation isa GNSSSignals.LOC && continue
        # A chip position the majority table applies to, and (TMBOC) one the
        # minority table does.
        positions =
            isnothing(shape.pattern) ? [(0, shape.lut_a)] :
            [
                (findfirst(!, shape.pattern) - 1, shape.lut_a),
                (findfirst(identity, shape.pattern) - 1, shape.lut_b),
            ]
        for (position, lut) in positions, k = 0:(shape.subchips-1)
            for u in (0.02, 0.17, 0.5, 0.83, 0.99)
                phase = position + (k + u) / shape.subchips
                reference = amplitude * GNSSSignals.get_subcarrier_code(modulation, phase)
                @test lut[k+1] ≈ reference rtol = 1e-2
            end
        end
    end
end

@testset "The replica amplitude is asserted, not assumed" begin
    # GNSSReceiver divides the channel's code amplitude out of every accumulator
    # before the C/N0 estimator sees it, so this equality is the whole reason
    # `replica_code_amplitude` can stay at its default: the table programmed
    # *is* the modelled code.
    for (_, signal) in L1_SIGNALS
        shape = replica_shape(signal)
        @test shape.code_amplitude == lut_rms(shape.lut_a)
        @test shape.code_amplitude ≈ get_code_amplitude(signal) rtol = 1e-3
        isnothing(shape.lut_b) || @test lut_rms(shape.lut_b) ≈ shape.code_amplitude
    end

    # Galileo E1's CBOC is the one table that is not ±1, and the amplitude *is*
    # the signal: levels ±25 and ±13, RMS sqrt(397) = 19.9249, exactly
    # `get_code_amplitude(GalileoE1B())`. A sign-only stand-in reads about 26 dB
    # away from the same satellite tracked in software — and tracks perfectly
    # well while doing it.
    e1b = replica_shape(GalileoE1B())
    @test sort(unique(abs.(e1b.lut_a))) == [13, 25]
    @test e1b.code_amplitude ≈ sqrt(397) rtol = 1e-9
    @test 20 * log10(e1b.code_amplitude) > 25          # the dB a sign-only table loses
    # …and the anti-phase pilot is a different table, not the same one signed.
    e1c = replica_shape(GalileoE1C())
    @test e1c.lut_a != e1b.lut_a
    @test e1c.lut_a != -e1b.lut_a
    @test e1c.code_amplitude ≈ e1b.code_amplitude

    # `GalileoE1B_BOC11` is a different signal, not a cheaper `GalileoE1B`: its
    # own type, its own sub-chip factor, its own amplitude. Nothing substitutes
    # one for the other.
    boc11 = replica_shape(GalileoE1B_BOC11())
    @test boc11.subchips == 2
    @test boc11.code_amplitude == 1.0
    @test boc11.lut_a == [1, -1]
end

@testset "TMBOC switches tables from a bit beside the chip" begin
    shape = replica_shape(GPSL1C_P())
    @test shape.modulation == :TMBOC
    @test shape.subchips == 12
    @test !isnothing(shape.lut_b)
    # The majority table is BOC(1,1) on the 12-sub-chip grid and the minority
    # one BOC(6,1); both ±1, so either RMS is the replica's.
    @test shape.lut_a == vcat(fill(1, 6), fill(-1, 6))
    @test shape.lut_b == [iseven(k) ? 1 : -1 for k = 0:11]

    bits = subcarrier_select_bits(shape, get_code_length(GPSL1C_P()))
    @test length(bits) == 10230
    @test all(b -> b in (0, 1), bits)
    # IS-GPS-800 §3.3: BOC(6,1) at positions 0, 4, 6 and 29 of every 33 chips.
    # GPS L1C's 10230 = 33 x 310, so the pattern tiles the code exactly and chip
    # 0 is pattern position 0.
    @test findall(==(1), bits)[1:4] .- 1 == [0, 4, 6, 29]
    @test count(==(1), bits) == 4 * 310
    @test bits == [shape.pattern[mod(c, 33)+1] ? 1 : 0 for c = 0:10229]

    # Every other modulation has one table and no bits to write beside its
    # chips, which is what keeps `code_load.sub` at 0 for them.
    @test isnothing(subcarrier_select_bits(replica_shape(GPSL1CA()), 1023))
    @test isnothing(subcarrier_select_bits(replica_shape(GalileoE1B()), 4092))
end

@testset "The tables match gnss-m2sdr's own subcarrier.py" begin
    moduledir = _subcarrier_module_dir()
    if isnothing(moduledir)
        @info "no gnss-m2sdr checkout found (set GNSS_M2SDR_DIR); the comparison " *
              "against gnss_m2sdr/subcarrier.py is not running"
        @test_skip false
    else
        script = """
import json
from gnss_m2sdr.subcarrier import signal_replica_shape, subchip_factor, lut_rms
names = %NAMES%
rows = []
for name in names:
    s = signal_replica_shape(name)
    rows.append('("%s", %d, %s, %s, %.17g)' % (
        name, s.subchips, json.dumps(s.lut_a),
        'nothing' if s.lut_b is None else json.dumps(s.lut_b),
        s.code_amplitude))
print('[' + ', '.join(rows) + ']')
"""
        names = "[" * join(["\"$(n)\"" for (n, _) in L1_SIGNALS], ", ") * "]"
        reference = _python_reference(replace(script, "%NAMES%" => names), moduledir)
        @test length(reference) == length(L1_SIGNALS)
        for ((name, signal), row) in zip(L1_SIGNALS, reference)
            shape = replica_shape(signal)
            @test row[1] == name
            @test shape.subchips == row[2]
            @test shape.lut_a == row[3]
            @test (isnothing(shape.lut_b) ? nothing : shape.lut_b) == row[4]
            @test shape.code_amplitude ≈ row[5] rtol = 1e-12
        end

        # The select-bit array and the sub-chip factors too, so nothing in this
        # package is a second copy of a gateware constant.
        select = _python_reference(
            """
import json
from gnss_m2sdr.subcarrier import tmboc_select_bits
print(json.dumps(tmboc_select_bits(10230)))
""",
            moduledir,
        )
        @test subcarrier_select_bits(replica_shape(GPSL1C_P()), 10230) == select

        factors = _python_reference(
            """
from gnss_m2sdr.subcarrier import subchip_factor
print('[%d, %d, %d, %d, %d]' % (
    subchip_factor('LOC'), subchip_factor('BOCsin', 1), subchip_factor('BOCsin', 6),
    subchip_factor('BOCcos', 1), subchip_factor('CBOC', 1, 6)))
""",
            moduledir,
        )
        @test factors == [
            subchip_factor(GNSSSignals.LOC()),
            subchip_factor(GNSSSignals.BOCsin(1, 1)),
            subchip_factor(GNSSSignals.BOCsin(6, 1)),
            subchip_factor(GNSSSignals.BOCcos(1, 1)),
            subchip_factor(
                GNSSSignals.CBOC(
                    GNSSSignals.BOCsin(1, 1),
                    GNSSSignals.BOCsin(6, 1),
                    10 / 11,
                ),
            ),
        ]
    end
end

# ── Capability discovery ────────────────────────────────────────────────────

# `gnss_signal_caps` as the v3 gateware packs it (bank.py's CSRField order), so
# the field offsets are checked against the register definition rather than
# against themselves.
v3_signal_caps(;
    modulations,
    max_secondary_code_length = 1,
    reports_code_phase = true,
    tap_layouts,
    max_subchips,
    replica_bits,
) =
    UInt64(modulations) | (UInt64(max_secondary_code_length) << 8) |
    (UInt64(reports_code_phase ? 1 : 0) << 16) | (UInt64(tap_layouts) << 17) |
    (UInt64(max_subchips) << 21) | (UInt64(replica_bits) << 29)

# The two builds this step is about: the five-tap sub-chip one gnss-m2sdr#32
# adds, and the three-tap BPSK one that is still the GPS L1 C/A baseline.
v3_caps(; num_taps = TAPS_VEPL, max_subchips = 12, max_code_length = 10230) =
    decode_capabilities(
        caps_word(;
            n_channels = 4,
            num_ants_max = 1,
            num_taps,
            code_frac_bits = 24,
            carrier_phase_bits = 32,
            accum_bits = 32,
            max_code_length,
        ),
        v3_signal_caps(;
            # Derived from the sub-chip depth by the gateware, never declared
            # from a spare bit: LOC | BOCsin | BOCcos | CBOC | TMBOC at 12,
            # LOC alone at 1.
            modulations = max_subchips >= 12 ? 0b11111 :
                          max_subchips >= 4 ? 0b10011 :
                          max_subchips >= 2 ? 0b10001 : 0b00001,
            tap_layouts = num_taps >= TAPS_VEPL ? 0b11 : 0b01,
            max_subchips,
            replica_bits = max_subchips > 1 ? 8 : 2,
        ),
    )

@testset "BOCsin is bit 4, and bit 1 stays BOCcos" begin
    # Record format v2 reserved bit 1 under the name `:BOCcos`. Every L1 BOC
    # signal GNSSSignals exposes is *sine*-phased, so redefining bit 1 would
    # make a v2-aware host declare `:BOCcos` for a build that synthesises
    # `:BOCsin` — an over-declared capability, which is a channel that arms and
    # never locks. A host that does not know bit 4 refuses the signal instead.
    @test decode_modulations(1 << 1) == [:BOCcos]
    @test decode_modulations(1 << 4) == [:BOCsin]
    @test decode_modulations(0b11111) == [:LOC, :BOCcos, :CBOC, :TMBOC, :BOCsin]
    @test :BOCsin ∉ decode_modulations(0b01111)
    # A `--max-subchips 1` build still reads LOC alone, exactly as v2 did.
    @test decode_modulations(0b00001) == [:LOC]
end

@testset "tap_layouts is read off the build, not inferred from num_taps" begin
    # Bit i means 2i + 3 taps. A five-tap build declares *both*, because the tap
    # count is staged per channel — that is what lets one bank run GPS L1 C/A on
    # three taps next to Galileo E1 on five. Reporting `[num_taps]` would refuse
    # GPS L1 C/A on the very build that adds Galileo.
    @test decode_tap_layouts(0b01) == [3]
    @test decode_tap_layouts(0b11) == [3, 5]
    @test decode_tap_layouts(0b10) == [5]
    @test decode_tap_layouts(0b00) == Int[]

    five = v3_caps()
    @test five.num_taps == 5
    @test five.tap_layouts == [3, 5]
    @test five.max_subchips == 12
    @test five.replica_bits == 8
    @test correlator_capabilities(five, 20e6; num_antennas = 1).tap_layouts == [3, 5]

    three = v3_caps(; num_taps = TAPS_EPL, max_subchips = 1)
    @test three.tap_layouts == [3]
    @test three.max_subchips == 1
    @test correlator_capabilities(three, 20e6; num_antennas = 1).tap_layouts == [3]
    @test correlator_capabilities(three, 20e6; num_antennas = 1).modulations == [:LOC]
end

@testset "A v3 five-tap build serves the BOC signals; a three-tap one refuses them" begin
    fs = 20e6
    five = correlator_capabilities(v3_caps(), fs; num_antennas = 1)
    three = correlator_capabilities(
        v3_caps(; num_taps = TAPS_EPL, max_subchips = 1),
        fs;
        num_antennas = 1,
    )
    support(caps, signal) = GNSSReceiver.hardware_support_error(
        caps,
        signal,
        Tracking.get_default_correlator(signal, Tracking.NumAnts(1)),
        fs;
        dump_tap_slots = 5,
    )

    # The signals this step exists for. Each is tracked with a five-tap
    # VeryEarlyPromptLateCorrelator and needs an amplitude-bearing or sub-chip
    # replica, so each was refused by name before gnss-m2sdr#32.
    for signal in (
        GalileoE1B(),
        GalileoE1C(),
        GalileoE1B_BOC11(),
        GalileoE1C_BOC11(),
        GPSL1C_D(),
        GPSL1C_P(),
        BeiDouB1C_D(),
        BeiDouB1C_P(),
    )
        @test isnothing(support(five, signal))
        refusal = support(three, signal)
        @test !isnothing(refusal)
        # Refused for a reason that names the actual gap, not a generic one.
        @test occursin("modulation", refusal) || occursin("tap", refusal)
    end

    # GPS L1 C/A stays the regression baseline: it works on the five-tap build
    # (on three taps, in the same bank) and on the three-tap one.
    @test isnothing(support(five, GPSL1CA()))
    @test isnothing(
        GNSSReceiver.hardware_support_error(
            three,
            GPSL1CA(),
            Tracking.get_default_correlator(GPSL1CA(), Tracking.NumAnts(1)),
            fs;
            dump_tap_slots = 3,
        ),
    )
    @test Tracking.get_num_accumulators(
        Tracking.get_default_correlator(GPSL1CA(), Tracking.NumAnts(1)),
    ) == 3
    @test Tracking.get_num_accumulators(
        Tracking.get_default_correlator(GalileoE1B(), Tracking.NumAnts(1)),
    ) == 5

    # `validate_hardware_configuration` is the gate the receiver runs before it
    # starts, so run it the same way rather than only its per-signal half.
    wide = StubHardwareSDR(five)
    narrow = StubHardwareSDR(three)
    @test isnothing(
        GNSSReceiver.validate_hardware_configuration(
            wide,
            (
                GPSL1CA(),
                GalileoE1B(),
                GalileoE1C(),
                GPSL1C_D(),
                GPSL1C_P(),
                BeiDouB1C_D(),
                BeiDouB1C_P(),
            ),
            fs,
        ),
    )
    # …and the same list on a three-tap `:LOC`-only build is refused as a whole,
    # with GPS L1 C/A still accepted on its own.
    @test_throws ArgumentError GNSSReceiver.validate_hardware_configuration(
        narrow,
        (GPSL1CA(), GalileoE1B()),
        fs,
    )
    @test isnothing(GNSSReceiver.validate_hardware_configuration(narrow, GPSL1CA(), fs))
end

@testset "max_subchips is checked against the signal, not just the family bit" begin
    fs = 20e6
    # A build with a 2-entry table declares `:BOCsin` — truthfully, for
    # BOCsin(1,1). It cannot hold BOCsin(6,1), and no modulation bit can say so:
    # the sub-chip factor has to be checked against the specific signal, which
    # is what the loop driver's arm does (`cs.subchips <= dev.max_subchips`,
    # `REJECT_UNSUPPORTED_SIGNAL`).
    shallow = v3_caps(; max_subchips = 2)
    @test :BOCsin in correlator_capabilities(shallow, fs; num_antennas = 1).modulations
    @test shallow.max_subchips == 2

    boc11 = ChannelSignal(GPSL1C_D(), 1)
    @test boc11.modulation == :BOCsin
    @test boc11.subchips == 2
    @test boc11.subchips <= shallow.max_subchips

    # The same family at a higher order: declared by the bit, refused by the
    # table depth. (Built by hand because GNSSSignals exposes no BOCsin(6,1)
    # signal today — which is exactly why the bit cannot be trusted alone.)
    boc61 = ChannelSignal(
        :BOCsin61,
        1,
        1,
        10230,
        1.023e6,
        1575.42e6,
        :BOCsin,
        12,
        :L1,
        1.0,
        1.0,
        1,
    )
    @test boc61.modulation == :BOCsin            # the same bit as boc11
    @test boc61.subchips > shallow.max_subchips  # and out of reach all the same
    # …and admitted once the table is deep enough.
    @test boc61.subchips <= v3_caps().max_subchips

    # CBOC needs 12 too, and a LOC-only build refuses it on the family bit
    # before the depth is ever consulted.
    e1b = ChannelSignal(GalileoE1B(), 1)
    @test e1b.subchips == 12
    loc_only = correlator_capabilities(
        v3_caps(; num_taps = TAPS_EPL, max_subchips = 1),
        fs;
        num_antennas = 1,
    )
    @test !(e1b.modulation in loc_only.modulations)
    refusal = GNSSReceiver.hardware_support_error(
        loc_only,
        GalileoE1B(),
        Tracking.get_default_correlator(GalileoE1B(), Tracking.NumAnts(1)),
        fs,
    )
    @test occursin("CBOC", refusal)
end

# ── Programming the device ──────────────────────────────────────────────────

@testset "Every tap is programmed from the contract's array" begin
    # `Tracking`'s discriminators recover the tap distances from the correlator
    # they are handed, and a five-tap correlator's VE/VL distance enters the
    # discriminator separately from the E/L one — so there is no single number
    # the array could be re-derived from, and the contract hands over all of it.
    fs = 20e6
    ch = boc_channel(; fs)
    step = code_word(ch, 0.0)
    shifts = [-12, -3, 0, 3, 12]
    set_tap_offsets!(ch, shifts, 0.0)
    mask = (Int64(1) << (CODE_FRAC_BITS + 1)) - 1
    # Earliest first on the wire, and every word exactly `sample_shift * step`.
    @test only(written(ch.csr, "tap_offset_ve")) == UInt64(12 * step)
    @test only(written(ch.csr, "tap_offset_e")) == UInt64(3 * step)
    @test only(written(ch.csr, "tap_offset_l")) == UInt64((-3 * step) & mask)
    @test only(written(ch.csr, "tap_offset_vl")) == UInt64((-12 * step) & mask)
    # The prompt has no register, and v3 has no `spacing` one.
    @test !wrote(ch.csr, "tap_offset_p")
    @test !wrote(ch.csr, "spacing")

    # A three-tap channel on the same five-tap build writes only E and L.
    three = boc_channel(; fs)
    set_tap_offsets!(three, [-4, 0, 4], 0.0)
    @test only(written(three.csr, "tap_offset_e")) == UInt64(4 * step)
    @test only(written(three.csr, "tap_offset_l")) == UInt64((-4 * step) & mask)
    @test !wrote(three.csr, "tap_offset_ve")
    @test !wrote(three.csr, "tap_offset_vl")

    # The symmetric shortcut is the same thing, said shorter.
    sym = boc_channel(; fs)
    set_spacing_chips!(sym, 4, 0.0)
    @test written(sym.csr, "tap_offset_e") == written(three.csr, "tap_offset_e")
    @test written(sym.csr, "tap_offset_l") == written(three.csr, "tap_offset_l")

    # The taps reach chip index ±1 only. Exactly -1.0 chip is the one
    # out-of-range value the signed register can hold, and the gateware reports
    # it as `replica_unsupported`; refuse it here instead.
    @test_throws ArgumentError tap_offset_word(
        1 << CODE_FRAC_BITS,
        0.0,
        fs,
        CODE_FRAC_BITS;
        code_frequency = 1.023e6,
    )
    @test_throws ArgumentError set_tap_offsets!(ch, [-400, -3, 0, 3, 400], 0.0)

    # What `Tracking` actually asks for on a Galileo E1 channel has to fit:
    # 0.15 chips E/L and 0.6 chips VE/VL, quantised onto whole input samples.
    correlator = Tracking.get_default_correlator(GalileoE1B(), Tracking.NumAnts(1))
    preferred = collect(Tracking.get_correlator_sample_shifts(correlator, fs, 1.023e6))
    # Latest first with the prompt at zero, which is the order the registers are
    # written in and what the arm command carries.
    @test issorted(preferred) && allunique(preferred)
    @test preferred[div(length(preferred), 2)+1] == 0
    programmed = boc_channel(; fs, signal = ChannelSignal(GalileoE1B(), 1))
    set_tap_offsets!(programmed, preferred, 0.0)
    for (name, shift) in zip(("ve", "e", "l", "vl"), reverse(preferred)[[1, 2, 4, 5]])
        word = only(written(programmed.csr, "tap_offset_" * name))
        signed =
            word >= (UInt64(1) << CODE_FRAC_BITS) ?
            Int64(word) - (Int64(1) << (CODE_FRAC_BITS + 1)) : Int64(word)
        @test signed == shift * code_word(programmed, 0.0)
        @test abs(signed) < (1 << CODE_FRAC_BITS)      # inside the ±1-chip reach
    end
end

@testset "The subcarrier table and the replica shape are staged with the code" begin
    shape = replica_shape(GalileoE1B())
    ch = boc_channel(; signal = ChannelSignal(GalileoE1B(), 1))
    load_replica_shape!(ch, shape; num_taps = TAPS_VEPL)

    # `subcarrier_load`, low to high: dat | adr | lut | we. replica_bits = 8 and
    # max_subchips = 12, so adr is 4 bits, lut is bit 12 and we is bit 13.
    words = written(ch.csr, "subcarrier_load")
    @test length(words) == 12
    for (k, word) in enumerate(words)
        @test (word >> 13) & 1 == 1                    # we
        @test (word >> 12) & 1 == 0                    # table A
        @test Int((word >> 8) & 0xF) == k - 1          # adr
        dat = Int(word & 0xFF)
        @test (dat >= 128 ? dat - 256 : dat) == shape.lut_a[k]
    end
    # `replica`: subchips in the low bits, the tap-count bit above them.
    # bits_for(12) = 4, so the taps bit is bit 4.
    @test only(written(ch.csr, "replica")) == UInt64(12 | (1 << 4))

    # A three-tap channel on the same build leaves the tap bit clear.
    l1 = boc_channel(; signal = ChannelSignal(GPSL1CA(), 1))
    load_replica_shape!(l1, replica_shape(GPSL1CA()); num_taps = TAPS_EPL)
    @test only(written(l1.csr, "replica")) == UInt64(1)
    # LOC still writes its one-entry table: the gateware's table survives the
    # previous occupant of the channel, and `subchips = 1` reads entry 0.
    @test length(written(l1.csr, "subcarrier_load")) == 1

    # TMBOC writes both tables, table B second.
    tmboc = boc_channel(; signal = ChannelSignal(GPSL1C_P(), 1))
    load_replica_shape!(tmboc, replica_shape(GPSL1C_P()); num_taps = TAPS_VEPL)
    words = written(tmboc.csr, "subcarrier_load")
    @test length(words) == 24
    @test all(w -> (w >> 12) & 1 == 0, words[1:12])
    @test all(w -> (w >> 12) & 1 == 1, words[13:24])

    # A build that cannot do what is asked says so rather than writing a
    # register that is not there.
    three_tap = GNSSBankChannel(
        RecordingCSR(),
        0;
        fs = 20e6,
        num_taps = TAPS_EPL,
        max_subchips = 1,
        replica_bits = 2,
    )
    @test_throws ArgumentError set_replica!(three_tap; subchips = 1, num_taps = TAPS_VEPL)
    @test_throws ArgumentError set_replica!(three_tap; subchips = 12, num_taps = TAPS_EPL)
    @test_throws ArgumentError write_subcarrier!(boc_channel(), 0, 200)   # past 8 bits signed
    @test_throws ArgumentError write_subcarrier!(boc_channel(), 12, 1)    # past the table
end

@testset "TMBOC's select bit is loaded beside its chip" begin
    shape = replica_shape(GPSL1C_P())
    # A short stand-in code: the select bits are per chip, so the wiring is the
    # same at 33 chips as at 10230 and the test does not pay 10230 CSR writes.
    code = Int[isodd(c) ? 1 : 0 for c = 0:32]
    select = subcarrier_select_bits(shape, length(code))
    ch = boc_channel(; signal = ChannelSignal(GPSL1C_P(), 1))
    load_code!(ch, 7, code; select, stage_length = false)
    words = written(ch.csr, "code_load")
    @test first(words) == 0b100                        # reset_addr
    chips = words[2:end]
    @test length(chips) == 33
    for (c, word) in enumerate(chips)
        @test (word >> 1) & 1 == 1                     # we
        @test Int(word & 1) == code[c]                 # the chip
        @test Int((word >> 3) & 1) == select[c]        # the table select bit
    end
    # Exactly the pattern positions IS-GPS-800 names, and nothing else.
    @test findall(w -> (w >> 3) & 1 == 1, chips) .- 1 == [0, 4, 6, 29]

    # Every other modulation leaves the bit clear.
    plain = boc_channel(; signal = ChannelSignal(GPSL1CA(), 1))
    load_code!(plain, 7, code; stage_length = false)
    @test all(w -> (w >> 3) & 1 == 0, written(plain.csr, "code_load")[2:end])

    # One bit per chip, or the load is refused rather than silently truncated.
    @test_throws ArgumentError load_code!(
        boc_channel(),
        7,
        code;
        select = select[1:10],
        stage_length = false,
    )
end

# The arming window as the loop driver's `arm!` performs it: the chips with
# their select bits first, then the table and the staged shape. Both raise
# `code_status.loading`, so the channel emits no records until the restart the
# handover schedules commits all of it at once.
arm_replica!(ch, prn, code, shape, num_taps) = begin
    load_code!(ch, prn, code; select = subcarrier_select_bits(shape, length(code)))
    load_replica_shape!(ch, shape; num_taps)
    ch
end

@testset "Arming writes the chips, their select bits and the replica together" begin
    # The whole arming window, against a recorded register map. Dropping the
    # select bits here leaves a GPS L1C-P channel replicating BOC(1,1) at every
    # chip position: it still correlates, about 0.6 dB down and with the wrong
    # correlation shape, so nothing errors.
    code = Int[isodd(c) ? 1 : 0 for c = 0:32]

    tmboc = boc_channel(; signal = ChannelSignal(GPSL1C_P(), 7))
    arm_replica!(tmboc, 7, code, replica_shape(GPSL1C_P()), TAPS_VEPL)
    chips = written(tmboc.csr, "code_load")[2:end]
    @test findall(w -> (w >> 3) & 1 == 1, chips) .- 1 == [0, 4, 6, 29]
    @test length(written(tmboc.csr, "subcarrier_load")) == 24     # both tables
    @test only(written(tmboc.csr, "replica")) == UInt64(12 | (1 << 4))
    # The chips are written before the table and the shape: both raise
    # `loading`, and the restart the handover schedules is what commits them.
    names = [n for (n, _) in tmboc.csr.writes]
    @test findlast(==("gnss_ch0_code_load"), names) <
          findfirst(==("gnss_ch0_subcarrier_load"), names) <
          findfirst(==("gnss_ch0_replica"), names)

    # Galileo E1B is one table and no select bits, on five taps.
    e1b = boc_channel(; signal = ChannelSignal(GalileoE1B(), 7))
    arm_replica!(e1b, 7, code, replica_shape(GalileoE1B()), TAPS_VEPL)
    @test all(w -> (w >> 3) & 1 == 0, written(e1b.csr, "code_load")[2:end])
    @test length(written(e1b.csr, "subcarrier_load")) == 12
    @test only(written(e1b.csr, "prn")) == 7

    # GPS L1 C/A on the same five-tap build: three taps, a one-entry table and
    # no select bits — the regression baseline, sharing the bank.
    l1 = boc_channel(; signal = ChannelSignal(GPSL1CA(), 7))
    arm_replica!(l1, 7, code, replica_shape(GPSL1CA()), TAPS_EPL)
    @test all(w -> (w >> 3) & 1 == 0, written(l1.csr, "code_load")[2:end])
    @test only(written(l1.csr, "replica")) == UInt64(1)
    @test length(written(l1.csr, "subcarrier_load")) == 1
end

@testset "replica_unsupported joins the channel's status word" begin
    ch = boc_channel()
    ch.csr.values["gnss_ch0_code_status"] = UInt64(0b100)
    status = code_status(ch)
    @test status.replica_unsupported
    @test !status.loading && !status.rate_unsupported
    ch.csr.values["gnss_ch0_code_status"] = UInt64(0b011)
    @test code_status(ch).loading
    @test code_status(ch).rate_unsupported
    @test !code_status(ch).replica_unsupported
end

# ── Records ─────────────────────────────────────────────────────────────────

@testset "A five-tap record carries the very-early/very-late pair" begin
    # The loop driver reads these into `[very late, late, prompt, early, very
    # early]` — the order Tracking's correlators want, and the order M2SDRLoop's
    # own tests pin. Here it is the wire half: the pair has to survive parsing,
    # in the four words version 2 reserved for it.
    bytes = collect(
        pack_record(
            sample_index = 8000,
            integrated_samples = 8000,
            channel = 0,
            prn = 11,
            num_taps = 5,
            code_phase_chip = 4091,
            code_length = 4092,
            ants = [(100.0 + 0im, 40.0 + 0im, 20.0 + 0im)],   # prompt, early, late
            very = [(5.0 + 0im, 1.0 + 0im)],                  # very early, very late
        ),
    )
    record = parse_record(bytes, 0, Val(1))
    @test record.num_taps == 5
    @test record.prompt[1] == 100.0 + 0im
    @test record.early[1] == 40.0 + 0im
    @test record.late[1] == 20.0 + 0im
    @test record.very_early[1] == 5.0 + 0im
    @test record.very_late[1] == 1.0 + 0im
end

@testset "Three- and five-tap records share one stream" begin
    # `num_taps` is per channel, not per build: a five-tap bank runs GPS L1 C/A
    # on three taps next to Galileo E1 on five in one record stream, and a
    # three-tap record simply leaves the five-tap words at zero.
    three = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 4000,
            channel = 0,
            prn = 3,
            num_taps = 3,
            ants = [(100.0 + 0im, 40.0 + 0im, 20.0 + 0im)],
        ),
    )
    five = collect(
        pack_record(
            sample_index = 4000,
            integrated_samples = 16368,
            channel = 1,
            prn = 11,
            num_taps = 5,
            ants = [(-100.0 + 0im, -40.0 + 0im, -20.0 + 0im)],
            very = [(-5.0 + 0im, -1.0 + 0im)],
        ),
    )
    r3 = parse_record(three, 0, Val(1))
    r5 = parse_record(five, 0, Val(1))
    @test r3.num_taps == 3
    @test r5.num_taps == 5
    @test (r3.prompt[1], r3.early[1], r3.late[1]) == (100.0 + 0im, 40.0 + 0im, 20.0 + 0im)
    @test r5.very_early[1] == -5.0 + 0im && r5.very_late[1] == -1.0 + 0im
    # Both are the same record type, so one stream carries both.
    @test typeof(r3) == typeof(r5)
    # A three-tap record's tail words are zero on the wire, so a reader that did
    # read them would see zeros rather than a previous integration.
    tail = reinterpret(UInt64, three)[13:16]
    @test all(iszero, tail)
end

@testset "Two antennas carry the very-early/very-late pair too" begin
    bytes = collect(
        pack_record(
            sample_index = 8000,
            integrated_samples = 8000,
            channel = 0,
            prn = 11,
            num_taps = 5,
            ants = [
                (100.0 + 0im, 40.0 + 0im, 20.0 + 0im),
                (0.0 + 100.0im, 0.0 + 40.0im, 0.0 + 20.0im),
            ],
            very = [(5.0 + 0im, 1.0 + 0im), (0.0 + 5.0im, 0.0 + 1.0im)],
        ),
    )
    record = parse_record(bytes, 0, Val(2))
    @test record.very_early == (5.0 + 0im, 0.0 + 5.0im)
    @test record.very_late == (1.0 + 0im, 0.0 + 1.0im)
    @test record.prompt == (100.0 + 0im, 0.0 + 100.0im)
end

# ── Version handling, in both directions ────────────────────────────────────

@testset "Only CSR layout v3 is driven; v1, v2 and v4 are refused by name" begin
    version(csr, record) = UInt64(csr) | (UInt64(record) << 8)
    build(csr_version; record = 2, num_taps = TAPS_VEPL, max_subchips = 12) = StubCSR(
        Dict(
            "gnss_version" => version(csr_version, record),
            "gnss_capabilities" => caps_word(;
                n_channels = 4,
                num_ants_max = 1,
                num_taps,
                code_frac_bits = 24,
                carrier_phase_bits = 32,
                accum_bits = 32,
                max_code_length = 10230,
            ),
            "gnss_signal_caps" => v3_signal_caps(;
                modulations = 0b11111,
                tap_layouts = num_taps >= TAPS_VEPL ? 0b11 : 0b01,
                max_subchips,
                replica_bits = 8,
            ),
        ),
    )

    caps = gateware_capabilities(build(3))
    @test caps.csr_version == 3
    @test caps.record_version == 2
    @test caps.tap_layouts == [3, 5]
    @test caps.max_subchips == 12

    # v2 has `spacing` and none of v3's registers; v1 predates runtime signal
    # configuration entirely. Both are refused with a message that names what is
    # missing rather than a bare version number.
    for (bad, needle) in ((2, "spacing"), (1, "record format"))
        err = try
            gateware_capabilities(build(bad; record = bad >= 2 ? 2 : 1))
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test occursin(needle, sprint(showerror, err))
    end
    # A layout newer than this driver is refused just as firmly: an unknown
    # register set addressed by name reports whatever the fields line up with.
    @test_throws ArgumentError gateware_capabilities(build(4))

    # A capability word declaring no tap layout at all describes no usable
    # build; say so rather than hand GNSSReceiver an empty profile.
    @test_throws ArgumentError gateware_capabilities(
        StubCSR(
            Dict(
                "gnss_version" => version(3, 2),
                "gnss_capabilities" => caps_word(;
                    n_channels = 4,
                    num_ants_max = 1,
                    num_taps = 5,
                    code_frac_bits = 24,
                    carrier_phase_bits = 32,
                    accum_bits = 32,
                    max_code_length = 10230,
                ),
                "gnss_signal_caps" => v3_signal_caps(;
                    modulations = 0b11111,
                    tap_layouts = 0b00,
                    max_subchips = 12,
                    replica_bits = 8,
                ),
            ),
        ),
    )
end
