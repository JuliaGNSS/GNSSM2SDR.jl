# Sub-chip replica shapes: what the gateware's subcarrier table has to contain.
#
# The Julia counterpart of gnss-m2sdr's `gnss_m2sdr/subcarrier.py`, and
# deliberately *not* a copy of its tables. The code RAM yields one chip sign per
# chip; everything a BOC-family signal adds happens inside a chip, and
# GNSSSignals models it as `get_subcarrier_code(modulation, phase)` — a function
# of the code phase alone, constant across each of `P` equal slices of a chip.
# So the replica factorises exactly:
#
#     replica(phase) = primary_chip(floor(phase)) · lut[floor(frac(phase) · P)]
#
# and the table below is that second factor, *evaluated from GNSSSignals* at
# each sub-chip's midpoint. Restating gnss-m2sdr's constants here would give two
# tables that can drift apart, and a drifted replica is a channel that arms,
# correlates a plausible-looking nothing and never locks — which is why
# `test/subchip_replica.jl` compares what this produces against
# `gnss_m2sdr/subcarrier.py` itself rather than against a second copy of it.
#
# See gnss-m2sdr's `docs/subchip_modulation.md` §2 for the gateware half.

# Sub-chips per chip of a modulation: how many equal slices of a chip its
# subcarrier is constant over, and therefore how deep a table the gateware needs
# (`gnss_signal_caps.max_subchips`).
#
# This is the number one capability bit *cannot* carry: `:BOCsin` says nothing
# about whether the build can hold `BOCsin(1,1)`'s 2 sub-chips or
# `BOCsin(6,1)`'s 12, so a host has to check the specific order it wants against
# the reported depth as well as the family bit.
_boc_order(m::Integer) = Int(m)
_boc_order(m) =
    isinteger(m) ? Int(m) :
    throw(
        ArgumentError(
            "a BOC order of $m is not a whole number of half-chips per chip, so it " *
            "has no exact sub-chip table; the gateware replicates only integer orders",
        ),
    )

subchip_factor(::GNSSSignals.LOC) = 1
subchip_factor(m::GNSSSignals.BOCsin) = 2 * _boc_order(m.m)
# The quarter-cycle shift straddles the sine grid, which is why cosine BOC needs
# quarter sub-chips; on that grid it is exact rather than rounded.
subchip_factor(m::GNSSSignals.BOCcos) = 4 * _boc_order(m.m)
subchip_factor(m::GNSSSignals.CBOC) = 2 * lcm(_boc_order(m.boc1.m), _boc_order(m.boc2.m))
subchip_factor(m::TMBOC) = 2 * _boc_order(m.boc2.m)
subchip_factor(m::GNSSSignals.Modulation) = throw(
    ArgumentError(
        "$(nameof(typeof(m))) has no sub-chip factor this driver knows; the " *
        "gateware synthesises LOC, BOCsin, BOCcos, CBOC and TMBOC",
    ),
)
subchip_factor(signal::AbstractGNSSSignal) = subchip_factor(get_modulation(signal))

"""
    lut_rms(lut) -> Float64

RMS amplitude of a subcarrier table — the amplitude of the code the device
actually correlates with, on the scale `GNSSSignals.get_code_amplitude` reports
(the primary chips are ±1, so the replica's RMS is the table's).

This is what `HardwareChannelConfig.code_amplitude` has to say for the channel:
GNSSReceiver divides it out of every accumulator before the C/N₀ estimator sees
it, so a table programmed at another scale and not declared reads the ratio
away from the same satellite tracked in software.
"""
lut_rms(lut::AbstractVector{<:Real}) = sqrt(sum(abs2, lut) / length(lut))

"""
    ReplicaShape

One channel's sub-chip replica: the table(s), the grid they are on, and the
amplitude they carry.

  - `subchips` — `P`, the sub-chip factor staged into `replica.subchips`.
  - `lut_a` / `lut_b` — the signed table entries. `lut_b` is `nothing` for every
    modulation but TMBOC, which alternates between two tables per *chip
    position*; the gateware picks between them from a bit stored beside the chip
    in the code RAM (`code_load.sub`) rather than from a counter that would have
    to stay in step with the code wrap and with every acquisition handover.
  - `pattern` — TMBOC's per-chip-position selector, the source of those bits
    ([`subcarrier_select_bits`](@ref)); `nothing` otherwise.
  - `code_amplitude` — `lut_rms(lut_a)`, the amplitude of the code as
    programmed. For GPS L1 C/A and every BOC/TMBOC subcarrier it is 1; for
    Galileo E1's CBOC it is √397 = 19.9249, which is exactly what
    `get_code_amplitude(GalileoE1B())` reports — so the host's default
    `code_amplitude` is right rather than a placeholder, and a sign-only
    stand-in would read ~26 dB away.
"""
struct ReplicaShape
    modulation::Symbol
    subchips::Int
    lut_a::Vector{Int}
    lut_b::Union{Nothing,Vector{Int}}
    pattern::Union{Nothing,Vector{Bool}}
    code_amplitude::Float64
end

# One table, evaluated off GNSSSignals at each sub-chip's midpoint and scaled to
# the modelled code's amplitude.
#
# The midpoint is not an approximation: the subcarrier is constant across a
# sub-chip by construction (that is what `P` means), so any phase inside it
# gives the same value, and the midpoint is the one that cannot land on a
# boundary through floating-point rounding.
#
# Scaling by `get_code_amplitude` is what makes CBOC come out as the *integer*
# table GNSSSignals' own resampler uses: its float subcarrier has unit RMS, so
# ±(√(10/11) ± √(1/11)) × √397 rounds to ±25 and ±13, the (a1, a2) = (19, 6)
# pair. Rounding a unit-RMS table for every other modulation is the identity.
_subcarrier_table(modulation, subchips::Int, amplitude::Float64, chip_position::Int) = [
    round(
        Int,
        amplitude * GNSSSignals.get_subcarrier_code(
            modulation,
            chip_position + (k + 0.5) / subchips,
        ),
    ) for k = 0:(subchips-1)
]

_subcarrier_tables(::GNSSSignals.LOC, subchips::Int, amplitude::Float64) =
    ([round(Int, amplitude)], nothing, nothing)

function _subcarrier_tables(m::TMBOC, subchips::Int, amplitude::Float64)
    pattern = collect(Bool, m.pattern)
    majority = findfirst(!, pattern)
    minority = findfirst(identity, pattern)
    (isnothing(majority) || isnothing(minority)) && throw(
        ArgumentError(
            "a TMBOC pattern that selects one component at every chip position is " *
            "that component, not a time multiplex; the gateware has no table to " *
            "switch to",
        ),
    )
    (
        _subcarrier_table(m, subchips, amplitude, majority - 1),
        _subcarrier_table(m, subchips, amplitude, minority - 1),
        pattern,
    )
end

_subcarrier_tables(m::GNSSSignals.Modulation, subchips::Int, amplitude::Float64) =
    (_subcarrier_table(m, subchips, amplitude, 0), nothing, nothing)

"""
    replica_shape(signal) -> ReplicaShape

The sub-chip replica `signal` needs, read off the modulation GNSSSignals gives
it. `GalileoE1B_BOC11` is a different signal from `GalileoE1B`, not a cheaper
one, and this returns what each of them actually models — nothing here
substitutes a BOC(1,1) stand-in for a CBOC channel behind the caller's back.
"""
function replica_shape(signal::AbstractGNSSSignal)
    modulation = get_modulation(signal)
    subchips = subchip_factor(modulation)
    amplitude = Float64(get_code_amplitude(signal))
    lut_a, lut_b, pattern = _subcarrier_tables(modulation, subchips, amplitude)
    ReplicaShape(
        nameof(typeof(modulation)),
        subchips,
        lut_a,
        lut_b,
        pattern,
        lut_rms(lut_a),
    )
end

"""
    subcarrier_select_bits(shape, code_length; pattern_phase = 0) -> Union{Nothing,Vector{Int}}

The per-chip "use the other subcarrier table" bits a TMBOC code is loaded with,
one per chip, or `nothing` for a modulation that has only one table.

`pattern_phase` is the pattern position of chip 0 — zero for a code whose length
is a whole number of pattern periods, which GPS L1C's 10230 = 33 × 310 is.
"""
function subcarrier_select_bits(
    shape::ReplicaShape,
    code_length::Integer;
    pattern_phase::Integer = 0,
)
    isnothing(shape.pattern) && return nothing
    n = length(shape.pattern)
    Int[shape.pattern[mod(pattern_phase+c, n)+1] ? 1 : 0 for c = 0:(code_length-1)]
end
