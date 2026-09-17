# Control of the gnss-m2sdr tracking bank over CSRs.
#
# A port of gnss-m2sdr's `software/gnss_tracking.py` (GNSSChannel / GNSSBank).
# Every fixed-point convention here has to match the gateware exactly, so the
# reasoning behind each word is kept with it.

# GPS L1 C/A, the signal this bank was originally written against and still the
# default a channel starts out configured for. These are *defaults*, not
# constants of the conversions below: every one of them is a property of the
# signal a channel replicates, and a channel replicating GPS L5 or Galileo E1B
# carries its own (see [`ChannelSignal`](@ref)).
const GPS_L1_HZ = 1_575_420_000.0
const GPS_CA_CHIP_RATE = 1_023_000.0
const CA_CODE_LENGTH = 1023
# Fractional bits of the code NCO (and thus of `dump_code_phase`); must match
# the gateware's `code_frac_bits`, which `gnss_capabilities` reports.
const CODE_FRAC_BITS = 24

"""
    ChannelSignal

What one hardware channel currently replicates: the identity of the signal
component, and every number the NCO words and the phase bookkeeping are derived
from.

The bank used to hold none of this — the chip rate, the carrier the Doppler
scales against and the code length were `GPS_CA_CHIP_RATE`, `GPS_L1_HZ` and
`CA_CODE_LENGTH` inlined in the conversions, so a GPS L5 channel was programmed
a GPS L1 C/A code rate and its code phase wrapped at chip 1023 of a 10230-chip
code. Both failures are silent: the channel arms, correlates a plausible-looking
nothing and never locks.

  - `id` / `signal_index` / `prn` — *which component of which satellite*. `id`
    is `GNSSSignals.get_signal_id`, so a pilot and its data component
    (`:GPSL5Q` / `:GPSL5I`) are distinct identities on the same PRN, and so are
    two constellations that share a PRN number. Code caching and the
    "does this channel need a new code?" test key on `(id, prn)`, never on the
    PRN alone.
  - `code_length` — primary-code chips. The modulus of every code phase this
    channel reports or is given, and the length staged into the gateware's
    `code_length` CSR.
  - `code_frequency` — nominal chip rate in Hz.
  - `center_frequency` — the signal's own carrier in Hz. Code Doppler is
    `f_chip · fd / f_carrier`, so using L1's 1575.42 MHz for an L5 signal
    mis-scales the code rate by 34 %.
  - `modulation` / `subchips` — the replica shape: the modulation family name
    (`nameof(typeof(get_modulation(signal)))`), checked against what the
    gateware declares it can synthesise, and the sub-chip factor
    ([`subchip_factor`](@ref)), checked against the depth of its subcarrier
    table. Both are needed: one modulation bit cannot distinguish
    `BOCsin(1,1)`'s 2 sub-chips from `BOCsin(6,1)`'s 12.
  - `band_id` — the RF band the channel lives on. Band *routing* is
    GNSSReceiver.jl#134; this is carried so a dump can be attributed to the
    band whose gain and sample rate it was taken at.
  - `code_amplitude` / `replica_amplitude` — the replica normalisation: the RMS
    amplitude of the code table the device correlates with, and the amplitude of
    the carrier replica it wipes off with. The host ingest divides both out, so
    the same satellite lands on one scale whichever correlator produced it.
  - `secondary_code_length` — the overlay the *signal* has. The gateware
    replicates the primary code only, so this is what the host has to take off
    the dumps (GNSSReceiver.jl#132) — and the reason the replica is loaded from
    the primary code table rather than from `get_code`, which multiplies in
    overlay chip 0 and would invert the whole replica for, say, BeiDou B1I
    PRN 6.
"""
struct ChannelSignal
    id::Symbol
    signal_index::Int
    prn::Int
    code_length::Int
    code_frequency::Float64
    center_frequency::Float64
    modulation::Symbol
    subchips::Int
    band_id::Symbol
    code_amplitude::Float64
    replica_amplitude::Float64
    secondary_code_length::Int
end

# Accept a plain number (Hz) as well as a Unitful frequency.
_hz(f::Real) = Float64(f)
_hz(f) = Float64(ustrip(uconvert(Hz, f)))

"""
    ChannelSignal(signal, prn; signal_index = 1, replica_amplitude = 1.0, kwargs...)

Read a channel's configuration off the `AbstractGNSSSignal` it must replicate.
`code_amplitude` defaults to the modelled table's own amplitude, i.e. "the
device reproduces the code exactly", which is true of every ±1 code.
"""
ChannelSignal(
    signal::AbstractGNSSSignal,
    prn::Integer;
    signal_index::Integer = 1,
    code_amplitude::Real = get_code_amplitude(signal),
    replica_amplitude::Real = 1.0,
    band_id::Symbol = get_band_id(get_band(signal)),
) = ChannelSignal(
    get_signal_id(signal),
    Int(signal_index),
    Int(prn),
    Int(get_code_length(signal)),
    _hz(get_code_frequency(signal)),
    _hz(get_center_frequency(signal)),
    nameof(typeof(get_modulation(signal))),
    subchip_factor(get_modulation(signal)),
    band_id,
    Float64(code_amplitude),
    Float64(replica_amplitude),
    Int(get_secondary_code_length(signal)),
)

# What a channel reads as before anything has been assigned to it: GPS L1 C/A
# with an impossible PRN, so the `(id, prn)` reuse test can never mistake a
# fresh channel for one already carrying the satellite being handed over.
const UNASSIGNED_SIGNAL = ChannelSignal(
    :none,
    0,
    0,
    CA_CODE_LENGTH,
    GPS_CA_CHIP_RATE,
    GPS_L1_HZ,
    :LOC,
    1,
    :L1,
    1.0,
    1.0,
    1,
)

"""
    signal_key(s::ChannelSignal) -> Tuple{Symbol,Int}

The identity a replica is cached and reused by: signal *and* PRN. Keying on the
PRN alone reuses GPS L1 C/A PRN 7's chips for Galileo E1B PRN 7, or a data
component's for its pilot — a channel that arms on the wrong code and never
locks, with nothing to point at.
"""
signal_key(s::ChannelSignal) = (s.id, s.prn)

"""
    GNSSBankChannel(csr, index; fs, carrier_phase_bits = 32, code_frac_bits = 24)

One hardware tracking channel. `index` is 0-based, matching the gateware's
`gnss_ch<N>_` CSR prefix.

`signal` is the [`ChannelSignal`](@ref) currently configured — mutable because a
channel is re-assigned across signals and constellations over a run, and every
NCO word it is programmed with is derived from it.

The CSR handle is a type parameter rather than a hard-coded [`LiteXCSR`](@ref):
every word this file programs has to agree with the gateware bit for bit, and a
recording stand-in is how that is checked without a board to flash.
"""
mutable struct GNSSBankChannel{C}
    const csr::C
    const fs::Float64
    const index::Int
    const carrier_phase_bits::Int
    const code_frac_bits::Int
    # What the *build* can do, so the write ports below are sized the way the
    # gateware packs them rather than from a default that happens to be right
    # for the bring-up bitstream. `GNSSBank` fills them from the capability
    # CSRs; the defaults are the lean three-tap BPSK build.
    const num_taps::Int
    const max_subchips::Int
    const replica_bits::Int
    const prefix::String
    signal::ChannelSignal
end

GNSSBankChannel(
    csr::C,
    index::Integer;
    fs,
    carrier_phase_bits::Integer = 32,
    code_frac_bits::Integer = 24,
    num_taps::Integer = TAPS_EPL,
    max_subchips::Integer = 1,
    replica_bits::Integer = 2,
    signal::ChannelSignal = UNASSIGNED_SIGNAL,
) where {C} = GNSSBankChannel{C}(
    csr,
    _hz(fs),
    Int(index),
    Int(carrier_phase_bits),
    Int(code_frac_bits),
    Int(num_taps),
    Int(max_subchips),
    Int(replica_bits),
    "gnss_ch$(index)_",
    signal,
)

# Width of a CSR field holding 0..n, as migen's `bits_for` sizes it — the same
# arithmetic the gateware used to pack `subcarrier_load` and `replica`, so the
# shifts below land on the fields rather than next to them.
_bits_for(n::Integer) = n <= 0 ? 1 : (8 * sizeof(Int) - leading_zeros(Int(n)))

# Sub-chip index and sub-chip-count field widths of this build's write ports.
_sub_adr_bits(ch::GNSSBankChannel) = _bits_for(max(1, ch.max_subchips - 1))
_subchips_bits(ch::GNSSBankChannel) = _bits_for(ch.max_subchips)

# ── Fixed-point conversions ──────────────────────────────────────────────────
#
# Kept as plain functions of (value, fs, width) rather than methods on the
# channel: they are the part that has to agree with the gateware bit for bit,
# and this way they can be tested without a device to open.

# Carrier NCO phase increment for `hz`, as a `bits`-wide word.
carrier_word(hz, fs, bits::Integer = 32) =
    round(Int64, hz / fs * (Int64(1) << bits)) & ((Int64(1) << bits) - 1)

"""
    code_step_word(chip_rate, fs, frac_bits) -> Int64

The code NCO's per-sample phase increment for `chip_rate`, as a `frac_bits`
fixed-point word.

The NCO crosses at most one chip boundary per input sample, so the representable
rates are `fs / 2^frac_bits` up to just under `fs`, and a rate outside that has
*no* representation. Masking it off — which is what this did while the rate was
a constant that could never reach the limit — is how GPS L5 at the bring-up
sample rate becomes a channel that correlates a plausible-looking nothing:

    10.23 Mcps at fs = 4.092 MHz  →  step word 8388608 = 0.5 chips/sample (wanted 2.5)

Raise instead. The gateware refuses the same word on its side (the CSR is one bit
wider than the fraction, and an out-of-range word raises `code_status.rate_unsupported`
and the sticky `gnss_rate_error` and stops that channel's dumps), so neither end
can truncate silently.
"""
function code_step_word(chip_rate, fs, frac_bits::Integer = CODE_FRAC_BITS)
    word = round(Int64, chip_rate / fs * (Int64(1) << frac_bits))
    0 < word < (Int64(1) << frac_bits) || throw(
        ArgumentError(
            "code rate $(chip_rate) chips/s is not representable at fs = $(fs) Hz: " *
            "$(chip_rate / fs) chips/sample, and the code NCO covers " *
            "2^-$(frac_bits) … just under 1 chip/sample (i.e. 0 < f_chip < fs). " *
            "Sample faster, or track a slower code.",
        ),
    )
    word
end

# Code NCO step for a *carrier* Doppler: the chip rate scales with it as
# fc = f_chip(1 + fd/f_carrier). Both `code_frequency` and `center_frequency` are
# the replicated signal's own — scaling GPS L5's 10.23 Mcps by L1's 1575.42 MHz
# instead of L5's 1176.45 MHz mis-sizes the Doppler by 34 %.
# Use [`code_word_from_code_doppler`](@ref) when you have the code Doppler.
function code_word(
    doppler_hz,
    fs,
    frac_bits::Integer = CODE_FRAC_BITS;
    code_frequency = GPS_CA_CHIP_RATE,
    center_frequency = GPS_L1_HZ,
)
    code_step_word(code_frequency * (1.0 + doppler_hz / center_frequency), fs, frac_bits)
end

# Code NCO step from the *code* Doppler (Hz of chip rate): fc = f_chip + fd_code.
# This is the unit `Tracking` reports and `NCOUpdate.code_doppler` carries —
# 1/1540 of the carrier Doppler for GPS L1 C/A. Feeding that value into
# `code_word` (which expects the carrier Doppler) silently programs a ~zero
# code-rate offset: a 1540× loop-gain error on the code NCO.
function code_word_from_code_doppler(
    code_doppler_hz,
    fs,
    frac_bits::Integer = CODE_FRAC_BITS;
    code_frequency = GPS_CA_CHIP_RATE,
)
    code_step_word(code_frequency + code_doppler_hz, fs, frac_bits)
end

carrier_phase_word(cycles, bits::Integer = 32) =
    round(Int64, mod(cycles, 1.0) * (Int64(1) << bits)) & ((Int64(1) << bits) - 1)

# Code phase (fractional chips) → the chip|frac word the gateware wants.
# `code_length` is the channel's own primary-code length: wrapping a 10230-chip
# L5 phase at 1023 turns chip 1500.25 into 477.25 — a valid-looking phase 1023
# chips from the satellite.
function code_phase_word(
    chips,
    frac_bits::Integer = CODE_FRAC_BITS;
    code_length::Integer = CA_CODE_LENGTH,
)
    phase = mod(chips, code_length)
    chip = floor(Int, phase)
    frac = round(Int64, (phase - chip) * (Int64(1) << frac_bits))
    if frac == (Int64(1) << frac_bits)   # rounding carried a whole chip
        chip = mod(chip + 1, code_length)
        frac = 0
    end
    (Int64(chip) << frac_bits) | frac
end

"""
    spacing_word(sample_shift, code_doppler_hz, fs, frac_bits; code_frequency)

The E/L half-spacing in fixed-point chips for `sample_shift` whole NCO samples.

`sample_shift * code_step` places the Early tap exactly that many samples ahead
of the prompt (and Late the same behind) with no rounding drift between the two
fixed-point words. Programming the raw preferred chip shift instead would leave
the accumulators at a spacing `dll_disc` does not assume — a ~2.3 % DLL
loop-gain error at fs = 4 MHz and 0.5 chips. The taps only reach chip index ±1,
so the word must stay below one chip.

`code_doppler_hz` is the *code* Doppler, so the step here is the one the channel
is actually programmed with rather than a nominal-rate approximation of it.

This is a host-side convenience for the symmetric three-tap case only. The
gateware has one register per tap and no notion of "the spacing" — see
[`tap_offset_word`](@ref).
"""
function spacing_word(
    sample_shift::Integer,
    code_doppler_hz,
    fs,
    frac_bits::Integer = CODE_FRAC_BITS;
    code_frequency = GPS_CA_CHIP_RATE,
)
    step = code_word_from_code_doppler(code_doppler_hz, fs, frac_bits; code_frequency)
    word = Int64(sample_shift) * step
    if word >= (Int64(1) << frac_bits)
        throw(
            ArgumentError(
                "E/L half-spacing $(word / (1 << frac_bits)) chips ≥ 1 chip: the E/L " *
                "taps only reach chip index ±1 (sample_shift=$sample_shift at fs=$fs Hz)",
            ),
        )
    end
    word
end

"""
    tap_offset_word(sample_shift, code_doppler_hz, fs, frac_bits; code_frequency)

The `tap_offset_*` CSR word for a tap sitting `sample_shift` whole input samples
early (negative = late), as the `frac_bits + 1`-bit two's-complement value the
register holds.

Every tap gets its own word because there is no single number to re-derive the
array from: `Tracking`'s discriminators recover the spacing from the correlator
they are handed, and a five-tap correlator's VE/VL distance enters the
discriminator separately from the E/L one. That is why GNSSReceiver hands over
the whole `tap_sample_shifts` array and says "program exactly these", and why
v3's gateware dropped the single symmetric `spacing` register.

Whole samples times the code step is the grid `Tracking` quantises its preferred
shifts onto (`calc_preferred_code_shift_to_sample_shift`), so the accumulators
sit exactly where the loop filter assumes. The taps reach chip index ±1 only, so
a whole chip is refused here rather than wrapped onto the wrong chip — which the
gateware also refuses, as `code_status.replica_unsupported`.
"""
function tap_offset_word(
    sample_shift::Integer,
    code_doppler_hz,
    fs,
    frac_bits::Integer = CODE_FRAC_BITS;
    code_frequency = GPS_CA_CHIP_RATE,
)
    step = code_word_from_code_doppler(code_doppler_hz, fs, frac_bits; code_frequency)
    word = Int64(sample_shift) * step
    abs(word) < (Int64(1) << frac_bits) || throw(
        ArgumentError(
            "tap offset $(word / (1 << frac_bits)) chips is a whole chip or more: the " *
            "taps reach chip index ±1 only (max_tap_offset_chips = " *
            "$MAX_TAP_OFFSET_CHIPS; sample_shift=$sample_shift at fs=$fs Hz)",
        ),
    )
    word & ((Int64(1) << (frac_bits + 1)) - 1)
end

# The gateware's tap registers, earliest replica first — the order
# `tap_offset_*` is written in. The prompt has no register: the contract fixes
# it at zero, and a register would only be a way to get it wrong.
_tap_register_names(num_taps::Integer) =
    num_taps == TAPS_VEPL ? ("ve", "e", "p", "l", "vl") :
    num_taps == TAPS_EPL ? ("e", "p", "l") :
    throw(
        ArgumentError(
            "a $(num_taps)-tap layout is not one of the record format's $TAP_LAYOUTS",
        ),
    )

# Channel-flavoured forwarders. Each reads the channel's own signal
# configuration, so nothing below this line has to know what GPS L1 C/A is.
carrier_word(ch::GNSSBankChannel, hz) = carrier_word(hz, ch.fs, ch.carrier_phase_bits)
code_word(ch::GNSSBankChannel, doppler_hz = 0.0) = code_word(
    doppler_hz,
    ch.fs,
    ch.code_frac_bits;
    code_frequency = ch.signal.code_frequency,
    center_frequency = ch.signal.center_frequency,
)
code_word_from_code_doppler(ch::GNSSBankChannel, code_doppler_hz = 0.0) =
    code_word_from_code_doppler(
        code_doppler_hz,
        ch.fs,
        ch.code_frac_bits;
        code_frequency = ch.signal.code_frequency,
    )
carrier_phase_word(ch::GNSSBankChannel, cycles) =
    carrier_phase_word(cycles, ch.carrier_phase_bits)
code_phase_word(ch::GNSSBankChannel, chips) =
    code_phase_word(chips, ch.code_frac_bits; code_length = ch.signal.code_length)
spacing_word(ch::GNSSBankChannel, sample_shift::Integer, code_doppler_hz = 0.0) =
    spacing_word(
        sample_shift,
        code_doppler_hz,
        ch.fs,
        ch.code_frac_bits;
        code_frequency = ch.signal.code_frequency,
    )
tap_offset_word(ch::GNSSBankChannel, sample_shift::Integer, code_doppler_hz = 0.0) =
    tap_offset_word(
        sample_shift,
        code_doppler_hz,
        ch.fs,
        ch.code_frac_bits;
        code_frequency = ch.signal.code_frequency,
    )

"""
    set_tap_offsets!(ch, sample_shifts, code_doppler_hz = 0.0)

Program every one of the channel's taps from GNSSReceiver's
`HardwareChannelConfig.tap_sample_shifts`: whole input samples, **latest first**,
prompt at zero — `[-s, 0, s]` for three taps and `[-s2, -s1, 0, s1, s2]` for
five.

The array is programmed as given rather than reduced to a spacing and rebuilt.
`Tracking` recovers the tap distances from the correlator it is handed, so a
five-tap layout has two of them and no single number describes it; for three
taps getting it wrong is a DLL loop-gain error, for five there is nothing to
re-derive the array from at all.

The gateware's registers run earliest first, so the list is reversed here, and
the prompt entry has no register — the contract fixes it at zero and this
refuses anything else rather than programming half a layout.
"""
function set_tap_offsets!(
    ch::GNSSBankChannel,
    sample_shifts::AbstractVector{<:Integer},
    code_doppler_hz = 0.0,
)
    names = _tap_register_names(length(sample_shifts))
    prompt = div(length(sample_shifts), 2) + 1
    sample_shifts[prompt] == 0 || throw(
        ArgumentError(
            "tap shifts $(collect(sample_shifts)) must carry the prompt (0) at index " *
            "$prompt; they are ordered latest first",
        ),
    )
    for (name, shift) in zip(names, Iterators.reverse(sample_shifts))
        name == "p" && continue
        write(
            ch.csr,
            ch.prefix * "tap_offset_" * name,
            tap_offset_word(ch, shift, code_doppler_hz),
        )
    end
    ch
end

"""
    set_spacing_chips!(ch, sample_shift, code_doppler_hz = 0.0)

Place Early and Late `sample_shift` samples either side of the prompt — the
symmetric three-tap shortcut, kept on the host where a convenience belongs. The
gateware has one register per tap and no notion of "the spacing".
"""
function set_spacing_chips!(
    ch::GNSSBankChannel,
    sample_shift::Integer,
    code_doppler_hz = 0.0,
)
    set_tap_offsets!(ch, [-Int(sample_shift), 0, Int(sample_shift)], code_doppler_hz)
end

"""
    write_subcarrier!(ch, address, value; table = 0)

One entry of the channel's sub-chip subcarrier table (`subcarrier_load`).

The write raises `code_status.loading`, so the channel emits no records until
the next restart commits the shape — a replica whose amplitude changed under an
integration would be one record of two different codes.
"""
function write_subcarrier!(
    ch::GNSSBankChannel,
    address::Integer,
    value::Integer;
    table::Integer = 0,
)
    bits = ch.replica_bits
    -(1 << (bits - 1)) <= value < (1 << (bits - 1)) || throw(
        ArgumentError(
            "subcarrier amplitude $value does not fit this build's $(bits)-bit signed " *
            "table entry; program a table that fits, and declare its RMS as the " *
            "channel's code amplitude",
        ),
    )
    0 <= address < ch.max_subchips || throw(
        ArgumentError(
            "sub-chip index $address is outside this build's $(ch.max_subchips)-entry " *
            "subcarrier table",
        ),
    )
    adr_bits = _sub_adr_bits(ch)
    # `subcarrier_load` storage, low to high: dat | adr | lut | we.
    word =
        (Int(value) & ((1 << bits) - 1)) | (Int(address) << bits) |
        (Int(table) << (bits + adr_bits)) | (1 << (bits + adr_bits + 1))
    write(ch.csr, ch.prefix * "subcarrier_load", word)
    ch
end

"""
    set_replica!(ch; subchips = 1, num_taps = TAPS_EPL)

Stage the channel's replica shape — sub-chips per chip, and how many taps it
reports. Staged, not applied: the next restart commits it together with the
code, the code length and the code phase, because all of them change what a
record means.

`num_taps` is per *channel*: a five-tap build runs GPS L1 C/A on three taps next
to Galileo E1 on five in the same bank.
"""
function set_replica!(
    ch::GNSSBankChannel;
    subchips::Integer = 1,
    num_taps::Integer = TAPS_EPL,
)
    num_taps in TAP_LAYOUTS ||
        throw(ArgumentError("num_taps must be one of $TAP_LAYOUTS, got $num_taps"))
    num_taps <= ch.num_taps || throw(
        ArgumentError(
            "this gateware build produces $(ch.num_taps) taps; a $(num_taps)-tap " *
            "layout cannot be configured on it (rebuild with --taps $num_taps)",
        ),
    )
    1 <= subchips <= ch.max_subchips || throw(
        ArgumentError(
            "a $(subchips)-sub-chip replica does not fit this build's " *
            "$(ch.max_subchips)-entry subcarrier table",
        ),
    )
    word = Int(subchips)
    # `replica` storage: [sub_bits-1:0] = subchips, [sub_bits] = taps (a
    # five-tap build only).
    if ch.num_taps >= TAPS_VEPL && num_taps == TAPS_VEPL
        word |= 1 << _subchips_bits(ch)
    end
    write(ch.csr, ch.prefix * "replica", word)
    ch
end

"""
    load_replica_shape!(ch, shape; num_taps)

Write `shape`'s subcarrier table(s) and stage the replica shape. `shape` is a
[`ReplicaShape`](@ref), i.e. the replica GNSSSignals models for the signal —
this never substitutes a sign-only stand-in for an amplitude-bearing CBOC table,
because that is a different code, not a cheaper one.

Table B is written only where the modulation has one (TMBOC); every other
modulation leaves the chips' select bits at 0 and never reads it.
"""
function load_replica_shape!(
    ch::GNSSBankChannel,
    shape::ReplicaShape;
    num_taps::Integer = TAPS_EPL,
)
    # Written on every build that *has* a table, including for `:LOC`. The
    # gateware's entries survive the channel's previous occupant, and a
    # `subchips = 1` channel reads entry 0 of whatever is there — so a GPS L1
    # C/A channel reusing a slot that last held Galileo E1B would correlate at
    # 25x amplitude if its one-entry table were left unwritten. A build with no
    # table at all (`max_subchips = 1`) has no register to write.
    if ch.max_subchips > 1
        for (table, lut) in ((0, shape.lut_a), (1, shape.lut_b))
            isnothing(lut) && continue
            for (address, value) in enumerate(lut)
                write_subcarrier!(ch, address - 1, value; table)
            end
        end
    elseif !isnothing(shape.lut_b) || shape.subchips > 1
        throw(
            ArgumentError(
                "this build has no subcarrier table (max_subchips = 1) and the " *
                "replica needs $(shape.subchips) sub-chips; a sign-only stand-in " *
                "would be a different code, not a cheaper one",
            ),
        )
    end
    set_replica!(ch; subchips = shape.subchips, num_taps)
    ch
end

"""
    load_code!(ch, prn, code; select = nothing)

Write `code`'s chips into the channel's code RAM, stage its length and set the
channel's PRN field. One CSR write per chip — 1023 for GPS L1 C/A, 10230 for
GPS L5 — so this is a configuration-time operation, not a hot path.

`select` is the per-chip subcarrier-table bit a TMBOC replica needs
([`subcarrier_select_bits`](@ref)), one per chip. It is stored *beside* the chip
rather than derived from a counter, which is what frees the subcarrier from
having to stay in step with the code wrap and with every acquisition handover:
any pattern, any code length and any start chip then work by construction.
`nothing` leaves every chip on table A, which is what every modulation but TMBOC
wants.

The load is *armed*, not applied: `reset_addr` raises the gateware's
`code_status.loading`, which stops the channel emitting records, and the staged
`code_length` is not in force until the next restart (immediate, or the
scheduled one a handover commits on). That is what makes a re-assignment atomic:
no record can describe a half-written code, or a code read at the previous
satellite's length. Confirm with [`code_length_active`](@ref) and
[`code_status`](@ref) once the restart has landed.
"""
function load_code!(
    ch::GNSSBankChannel,
    prn::Integer,
    code::AbstractVector;
    select::Union{Nothing,AbstractVector} = nothing,
    stage_length::Bool = has_code_length_csr(ch),
)
    isnothing(select) ||
        length(select) == length(code) ||
        throw(
            ArgumentError(
                "$(length(select)) subcarrier-select bits for $(length(code)) chips: " *
                "the select bit is stored beside its chip, so there is exactly one " *
                "per chip",
            ),
        )
    write(ch.csr, ch.prefix * "code_load", 0b100)              # reset address
    for (index, chip) in enumerate(code)
        # bit0 = dat, bit1 = we, bit3 = subcarrier-table select.
        sub = isnothing(select) ? 0 : (Int(select[index]) & 1) << 3
        write(ch.csr, ch.prefix * "code_load", 0b010 | (Int(chip) & 1) | sub)
    end
    stage_length && set_code_length!(ch, length(code))
    write(ch.csr, ch.prefix * "prn", prn)
    ch
end

# Whether this gateware's code length is a runtime input at all. On a build
# predating gnss-m2sdr#31 it is a *build-time* parameter of the code replica —
# there is no register to stage, and the only correct code to load is one of
# exactly that length. `M2SDRCorrelator` refuses such a build outright; this
# keeps the low-level bank API usable for the GPS L1 C/A bring-up scripts that
# drive one by hand.
has_code_length_csr(ch::GNSSBankChannel) = has_register(ch.csr, ch.prefix * "code_length")

"""
    set_code_length!(ch, chips)

Stage the channel's primary-code length; the next restart commits it, together
with the code and the code phase.
"""
function set_code_length!(ch::GNSSBankChannel, chips::Integer)
    chips >= 1 || throw(ArgumentError("code length $chips must be at least 1 chip"))
    has_code_length_csr(ch) || throw(
        ArgumentError(
            "this gateware has no gnss_ch$(ch.index)_code_length register: its " *
            "primary-code length is fixed at build time and cannot be staged. " *
            "Flash a build streaming record format v$RECORD_FORMAT_VERSION " *
            "(gnss-m2sdr ≥ #31) to track anything but the length it was built for.",
        ),
    )
    write(ch.csr, ch.prefix * "code_length", chips)
    ch
end

"""
    code_length_active(ch) -> Int

The primary-code length actually in force, as of the channel's last restart —
the readback that confirms a staged length was committed.
"""
code_length_active(ch::GNSSBankChannel) =
    Int(read(ch.csr, ch.prefix * "code_length_active"))

"""
    code_status(ch) -> (loading, rate_unsupported, replica_unsupported)

`loading`: a code load or a subcarrier-table write is armed and the channel is
emitting no records.
`rate_unsupported`: the channel was programmed a code rate of one chip per input
sample or more, which the NCO cannot represent, so its dumps are suppressed
rather than produced at a truncated rate.
`replica_unsupported`: the replica shape in force cannot be evaluated — a
sub-chip count of zero or past the build's table, or a tap offset of a whole
chip, either of which would otherwise produce something that looks like a
replica and is not. All three are cleared by that channel's restart.
"""
function code_status(ch::GNSSBankChannel)
    v = read(ch.csr, ch.prefix * "code_status")
    (
        loading = (v & 0b001) != 0,
        rate_unsupported = (v & 0b010) != 0,
        replica_unsupported = (v & 0b100) != 0,
    )
end

"""
    restart!(ch)

Pulse the channel's restart + carrier-set strobe (edge-triggered 0 → 1 → 0).
"""
function restart!(ch::GNSSBankChannel)
    write(ch.csr, ch.prefix * "control", 0)
    write(ch.csr, ch.prefix * "control", 0b11)
    write(ch.csr, ch.prefix * "control", 0)
    ch
end

"""
    schedule!(ch, sample_index; carrier_hz, code_doppler_hz, carrier_phase_cycles,
              code_phase_chips)

Commit the supplied values atomically on global sample `sample_index`.

`sample_index` is on the bank's free-running counter — the same axis records are
timestamped on — and names the first input sample processed with the new values.
This is the hardware meaning of `GNSSReceiver.NCOUpdate.apply_at_sample`, and it
is what buys a fixed feedback delay instead of PCIe jitter. Only the values
passed are committed; supplying `code_phase_chips` (an acquisition handover)
also restarts the integration on that sample.

Check [`apply_status`](@ref) afterwards: `late` means the CSR writes did not
reach the board in time and the commit slipped to a later sample.

!!! warning "One outstanding commit per channel"
    The gateware has a single staging slot and a single `armed` bit, so this is
    **not** a queue: calling `schedule!` again while `apply_status(ch).armed` is
    still set does not enqueue a second commit, it *replaces* the pending one —
    the values from the first call are then never applied at the sample it
    picked. Wait for `armed` to clear (as [`assign_channel!`](@ref) does) before
    scheduling the next commit on the same channel.

    That makes the scheduled path unsuitable for *streaming* NCO corrections: at
    a 1 kHz loop rate the next correction arrives ~1 ms after the last, while
    `sample_index` is one or two epochs ahead, so each commit would be cancelled
    ~1 ms before it was due and the channel would keep free-running on its
    handover words while the host believed it was steering it. Rate-only updates
    therefore go through the immediate `carrier_freq` / `code_freq` CSRs
    (`_drain_ncos!`); `schedule!` is for sample-exact handovers.
"""
function schedule!(
    ch::GNSSBankChannel,
    sample_index::Integer;
    carrier_hz = nothing,
    code_doppler_hz = nothing,
    carrier_phase_cycles = nothing,
    code_phase_chips = nothing,
)
    flags = 0b1                                       # arm
    if carrier_hz !== nothing
        write(ch.csr, ch.prefix * "carrier_freq_next", carrier_word(ch, carrier_hz))
        flags |= 1 << 3
    end
    if code_doppler_hz !== nothing
        # `code_doppler_hz` is the *code* Doppler (chip-rate offset in Hz), the
        # unit Tracking and `NCOUpdate` carry — not the carrier Doppler.
        write(
            ch.csr,
            ch.prefix * "code_freq_next",
            code_word_from_code_doppler(ch, code_doppler_hz),
        )
        flags |= 1 << 4
    end
    if carrier_phase_cycles !== nothing
        write(
            ch.csr,
            ch.prefix * "carrier_phase",
            carrier_phase_word(ch, carrier_phase_cycles),
        )
        flags |= 1 << 2
    end
    if code_phase_chips !== nothing
        write(ch.csr, ch.prefix * "code_phase", code_phase_word(ch, code_phase_chips))
        flags |= 1 << 1
    end
    write(ch.csr, ch.prefix * "apply_at", sample_index)
    write(ch.csr, ch.prefix * "apply", 0)             # arm is 0 → 1 edge-triggered
    write(ch.csr, ch.prefix * "apply", flags)
    ch
end

"""
    apply_status(ch) -> (armed, late)
"""
function apply_status(ch::GNSSBankChannel)
    s = read(ch.csr, ch.prefix * "apply_status")
    (armed = (s & 0b01) != 0, late = (s & 0b10) != 0)
end

applied_at(ch::GNSSBankChannel) = read(ch.csr, ch.prefix * "applied_at")

"""
    detect_num_channels(csr) -> Int

How many `gnss_ch<i>_` tracking channels the flashed gateware exposes, counted
off the CSR map (channels are numbered consecutively from 0). This is the
authoritative channel count — passing a larger `n_channels` to [`GNSSBank`](@ref)
would build channels whose CSR reads throw `KeyError`.
"""
detect_num_channels(csr::LiteXCSR) = detect_num_channels(csr.regs)

# Core on the register map itself, so it can be tested without a device.
function detect_num_channels(regs::AbstractDict)
    n = 0
    while haskey(regs, "gnss_ch$(n)_control")
        n += 1
    end
    n
end

"""
    GNSSBank(csr; fs, n_channels = detect_num_channels(csr))

The tracking bank: the channels plus the bank-wide controls (enable, epoch
strobe period, overflow status, the free-running sample counter).
"""
struct GNSSBank{C}
    csr::C
    channels::Vector{GNSSBankChannel{C}}
end

function GNSSBank(
    csr::C;
    fs,
    n_channels::Integer = detect_num_channels(csr),
    code_frac_bits::Integer = CODE_FRAC_BITS,
    carrier_phase_bits::Integer = 32,
    num_taps::Integer = TAPS_EPL,
    max_subchips::Integer = 1,
    replica_bits::Integer = 2,
) where {C}
    GNSSBank{C}(
        csr,
        [
            GNSSBankChannel(
                csr,
                i - 1;
                fs,
                code_frac_bits,
                carrier_phase_bits,
                num_taps,
                max_subchips,
                replica_bits,
            ) for i = 1:n_channels
        ],
    )
end

enable!(bank::GNSSBank, on::Bool = true) =
    (write(bank.csr, "gnss_control", on ? 1 : 0); bank)

"""
    set_epoch_period!(bank, samples)

Emit a timebase-marker record every `samples` input samples. Without it the
host's epoch clock stalls whenever no channel is dumping.
"""
set_epoch_period!(bank::GNSSBank, samples::Integer) =
    (write(bank.csr, "gnss_epoch_period", samples); bank)

num_ants(bank::GNSSBank) = Int(read(bank.csr, "gnss_num_ants"))

"""
    overflow(bank) -> UInt64

The sticky per-channel overflow bitmap. Clear it with [`clear_overflow!`](@ref);
it is write-1-to-clear in the gateware, so reading alone does not reset it.
"""
overflow(bank::GNSSBank) = read(bank.csr, "gnss_overflow")
clear_overflow!(bank::GNSSBank, mask::Integer = 0xFFFFFFFF) =
    (write(bank.csr, "gnss_overflow_clear", mask); bank)

"""
    sample_count(bank; tries = 3) -> Int64

The bank's free-running sample counter. Read twice and retried because the
64-bit value is assembled from two 32-bit CSR reads that the counter can tick
between; a pair whose high word is stable is consistent.
"""
function sample_count(bank::GNSSBank; tries::Integer = 3)
    local value
    for _ = 1:tries
        first_read = read(bank.csr, "gnss_sample_count")
        second_read = read(bank.csr, "gnss_sample_count")
        value = second_read
        (second_read >> 32) == (first_read >> 32) && return Int64(second_read)
    end
    Int64(value)
end

"""
    rate_error(bank) -> UInt64

The sticky per-channel "an unrepresentable code rate was programmed" bitmap. A
set bit means that channel was asked for one chip per input sample or more, so
the gateware suppressed its dumps instead of tracking at a truncated rate.
Cleared by that channel's restart.
"""
rate_error(bank::GNSSBank) = read(bank.csr, "gnss_rate_error")

# ── Host discovery: what the flashed gateware actually is ────────────────────
#
# None of this is a constant of this driver. A bitstream rebuilt with deeper
# code memory, more channels or a wider code NCO describes itself, and the
# capabilities the receiver validates against are read off the board rather than
# assumed — because an over-declared capability is a channel that arms and never
# locks, which is the hardest failure in this whole path to attribute.

"""
    gateware_version(csr) -> (csr = Int, record = Int)

The CSR-layout and DMA1-record-format revisions the flashed gateware
implements, or `nothing` for a build that predates the `gnss_version` register
(which is, by construction, revision 1 of both).
"""
function gateware_version(csr)
    has_register(csr, "gnss_version") || return (csr = 1, record = 1)
    v = read(csr, "gnss_version")
    (csr = Int(v & 0xFF), record = Int((v >> 8) & 0xFF))
end

gateware_version(bank::GNSSBank) = gateware_version(bank.csr)

# Bit field of a packed capability CSR word.
_csr_field(value::Integer, shift::Integer, width::Integer) =
    Int((UInt64(value) >> shift) & ((UInt64(1) << width) - 1))

"""
    decode_capabilities(capabilities_word, signal_caps_word) -> NamedTuple

The `gnss_capabilities` / `gnss_signal_caps` bit fields, unpacked. Split from
the CSR read so the mapping — which has to agree with the gateware's field
order exactly — can be checked without a board to open.
"""
decode_capabilities(capabilities_word::Integer, signal_caps_word::Integer) = (
    n_channels = _csr_field(capabilities_word, 0, 8),
    num_ants_max = _csr_field(capabilities_word, 8, 8),
    num_taps = _csr_field(capabilities_word, 16, 8),
    code_frac_bits = _csr_field(capabilities_word, 24, 8),
    carrier_phase_bits = _csr_field(capabilities_word, 32, 8),
    accum_bits = _csr_field(capabilities_word, 40, 8),
    max_code_length = _csr_field(capabilities_word, 48, 16),
    modulations = _csr_field(signal_caps_word, 0, 8),
    max_secondary_code_length = _csr_field(signal_caps_word, 8, 8),
    reports_code_phase = _csr_field(signal_caps_word, 16, 1) != 0,
    tap_layouts = decode_tap_layouts(_csr_field(signal_caps_word, 17, 4)),
    max_subchips = _csr_field(signal_caps_word, 21, 8),
    replica_bits = _csr_field(signal_caps_word, 29, 8),
)

"""
    decode_tap_layouts(mask) -> Vector{Int}

The tap layouts a build declares, from `gnss_signal_caps.tap_layouts`: bit *i*
means 2*i* + 3 taps, so bit 0 is three and bit 1 is five.

A five-tap build declares **both**, because the tap count is staged per channel
(`replica.taps`): that is what lets one bank run GPS L1 C/A on three taps next
to Galileo E1 on five in the same record stream. Reporting `[num_taps]` instead
would refuse GPS L1 C/A on the very build that adds Galileo.
"""
decode_tap_layouts(mask::Integer) = Int[2i + 3 for i = 0:3 if (mask & (1 << i)) != 0]

# Replica modulations the gateware advertises, as the bitmask of
# `gnss_signal_caps.modulations`. The names are GNSSReceiver's
# `HardwareCorrelatorCapabilities.modulations` symbols, so a set bit maps
# straight onto one.
#
# `:BOCsin` is **bit 4, not bit 1**. Record format v2 allocated bits 0..3 and
# reserved bit 1 under the name `:BOCcos`; every L1 BOC signal GNSSSignals
# exposes is *sine*-phased, so redefining bit 1 would make a host that knows the
# v2 mapping declare `:BOCcos` for a build that synthesises `:BOCsin` — an
# over-declared capability, i.e. a channel that arms and never locks. A host
# that does not know bit 4 refuses the signal instead, which is the safe
# direction.
#
# One bit cannot say which *order* of a family a build can hold —
# `BOCsin(1,1)` needs 2 sub-chips and `BOCsin(6,1)` needs 12 — so the specific
# signal is checked against `max_subchips` as well (see
# [`subchip_factor`](@ref)).
const MODULATION_BITS = (
    (1 << 0) => :LOC,
    (1 << 1) => :BOCcos,
    (1 << 2) => :CBOC,
    (1 << 3) => :TMBOC,
    (1 << 4) => :BOCsin,
)

decode_modulations(mask::Integer) =
    Symbol[name for (bit, name) in MODULATION_BITS if (mask & bit) != 0]

"""
    gateware_capabilities(csr) -> NamedTuple

Read the build's fixed limits off its own capability CSRs, refusing a gateware
whose interface revision this driver does not know.

`csr` is anything answering [`has_register`](@ref) and `read(csr, name)` — a
[`LiteXCSR`](@ref) in practice, and a recorded register map in the tests, which
is how the refusals below are checked without a board to flash.

Refusing rather than guessing is the point: a newer CSR layout read as if it
were this one reports whatever the fields happen to line up with, and the
receiver then validates satellites against a capability profile that is not the
device's.
"""
function gateware_capabilities(csr)
    version = gateware_version(csr)
    version.record >= RECORD_FORMAT_VERSION || throw(
        ArgumentError(
            "this gnss-m2sdr gateware predates runtime signal configuration " *
            "(CSR layout v$(version.csr), record format v$(version.record)): it " *
            "correlates a single build-time code length at a single chip rate, and " *
            "its records carry no integer code-phase chip, no code length and no " *
            "code step. GPS L1 C/A is the only signal it can be trusted with, and " *
            "this driver will not infer the rest. Flash a build streaming record " *
            "format v$RECORD_FORMAT_VERSION (gnss-m2sdr ≥ #31).",
        ),
    )
    # The CSR layout is matched exactly, in both directions, because the driver
    # addresses the bank's registers *by name*: an unknown set reports whatever
    # the fields happen to line up with rather than failing, and the receiver
    # then validates satellites against a profile that is not the device's.
    version.csr < CSR_LAYOUT_VERSION && throw(
        ArgumentError(
            "this gnss-m2sdr gateware uses CSR layout v$(version.csr) and this driver " *
            "speaks v$CSR_LAYOUT_VERSION. v3 replaced the single symmetric `spacing` " *
            "register with per-tap `tap_offset_{ve,e,l,vl}`, and added `replica`, " *
            "`subcarrier_load` and `dump_num_taps`; a v$(version.csr) build has none " *
            "of them, so its taps cannot be placed and no BOC/CBOC/TMBOC replica can " *
            "be programmed on it. Flash a build with CSR layout " *
            "v$CSR_LAYOUT_VERSION (gnss-m2sdr ≥ #32).",
        ),
    )
    version.csr > CSR_LAYOUT_VERSION && throw(
        ArgumentError(
            "gateware CSR layout v$(version.csr) is newer than this driver " *
            "(v$CSR_LAYOUT_VERSION); update GNSSM2SDR.jl rather than addressing an " *
            "unknown register set by name",
        ),
    )
    version.record <= RECORD_FORMAT_VERSION || throw(
        ArgumentError(
            "gateware streams DMA1 record format v$(version.record), newer than " *
            "this driver's v$RECORD_FORMAT_VERSION",
        ),
    )
    fields =
        decode_capabilities(read(csr, "gnss_capabilities"), read(csr, "gnss_signal_caps"))
    isempty(fields.tap_layouts) && throw(
        ArgumentError(
            "this gateware declares no correlator tap layout " *
            "(gnss_signal_caps.tap_layouts = 0), so there is no layout a channel " *
            "could be armed with; the capability CSR is not describing a usable build",
        ),
    )
    merge(fields, (csr_version = version.csr, record_version = version.record))
end

gateware_capabilities(bank::GNSSBank) = gateware_capabilities(bank.csr)

# Furthest an E/P/L tap can sit from the prompt replica: the taps address chip
# index ±1, so a whole chip is the hard limit. Not a CSR — it is a property of
# the replica addressing, which has no build-time knob.
const MAX_TAP_OFFSET_CHIPS = 1.0

"""
    correlator_capabilities(caps, fs; num_antennas) -> HardwareCorrelatorCapabilities

Map the gateware's own capability fields onto GNSSReceiver's vendor-neutral
profile, the one [`GNSSReceiver.validate_hardware_configuration`](@ref) refuses
unserviceable signals against before a channel is armed.

`signals` is left unrestricted: the gateware holds arbitrary chips and this
driver generates them from `GNSSSignals`, so the limits that actually bind are
the code memory, the code NCO's range at `fs` and the modulations the replica
can synthesise — all of which the device reports. `bands` is likewise left open;
which band the front end is tuned to is RF-side and belongs to
GNSSReceiver.jl#134, not to these registers.

`tap_layouts` comes from `gnss_signal_caps`, not from `[caps.num_taps]`: the tap
count is staged per channel, so a five-tap build serves three-tap channels too
and declares `[3, 5]`. Declaring only the widest layout would refuse GPS L1 C/A
on the very build that adds Galileo E1.

One limit is *not* expressible here, and is enforced at arm time instead
([`GNSSM2SDR.subchip_factor`](@ref)): the sub-chip table depth. A modulation bit
cannot distinguish `BOCsin(1,1)`'s 2 sub-chips from `BOCsin(6,1)`'s 12, so
`max_subchips` is checked against the specific signal rather than folded into
this profile.
"""
function correlator_capabilities(caps::NamedTuple, fs::Real; num_antennas::Integer)
    scale = Float64(fs) / (1 << caps.code_frac_bits)
    GNSSReceiver.HardwareCorrelatorCapabilities(;
        signals = nothing,
        modulations = decode_modulations(caps.modulations),
        max_primary_code_length = caps.max_code_length,
        # The code NCO steps 1 … 2^code_frac_bits - 1 in 2^-code_frac_bits chips
        # per input sample, so the representable chip rates are a property of fs
        # and not of the gateware alone.
        code_frequency_limits = (scale, scale * ((1 << caps.code_frac_bits) - 1)),
        tap_layouts = caps.tap_layouts,
        max_tap_offset_chips = MAX_TAP_OFFSET_CHIPS,
        num_antennas = min(Int(num_antennas), caps.num_ants_max),
        bands = nothing,
        num_rf_inputs = 1,
        max_secondary_code_length = caps.max_secondary_code_length,
        reports_code_phase = caps.reports_code_phase,
    )
end
