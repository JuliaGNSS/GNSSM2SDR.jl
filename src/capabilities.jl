# ─────────────────────────────────────────────────────────────────────────────
# What this board can serve, in GNSSReceiver's vendor-neutral terms.
#
# The same facts the loop process's driver declares to the loop core
# (`M2SDRLoop.driver_capabilities`), read off the same gateware registers and
# translated for the receiver's pre-arm validation.
# ─────────────────────────────────────────────────────────────────────────────

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
