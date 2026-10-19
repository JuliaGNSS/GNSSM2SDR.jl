# Without a board: the emulator

[`TDMEmulator`](@ref) is the TDM correlator in software. It has the register
file, the commands, the channel engine and the record stream, all in the
gateware's integer arithmetic. It is a port of gnss-m2sdr's
`gnss_m2sdr/tdm/model.py`, which the gateware is checked against record for
record. It implements the same [`TDMIO`](@ref) calls as the board
([`LitePCIeTDM`](@ref)), so the driver cannot tell the two apart.

The `M2SDRLoop` tests use it to run the driver and a closed loop without
hardware. You can use it the same way to try loop changes.

## A closed loop

Give the emulator samples with [`feed!`](@ref), then let the loop core take a
service pass. The emulator's registers are laid out by
`M2SDRLoop.emulator_regs()`:

```julia
using M2SDRLoop, HardwareLoopCore, HardwareLoopProtocol, GNSSSignals
using M2SDRLoop: emulator_regs
const HLP = HardwareLoopProtocol

fs = 4e6
emu = TDMEmulator(; n_channels = 4)
driver = TDMDriver(emu, emulator_regs(), (GPSL1CA(),); fs)
start_device!(driver)

segment = create_segment(nothing, SegmentConfig(; channel_count = 4,
                                                 bands = driver_capabilities(driver).bands))
core = LoopCore(driver, (GPSL1CA(),), segment)

# Hand PRN 11 to channel 1, as the receiver would after acquisition.
try_publish!(command_ring(segment), CommandTag(HLP.COMMAND_ARM, 1, 1), ArmCommand(;
    signal = :GPSL1CA, prn = 11, carrier_doppler_hz = -2330.0, code_doppler_hz = -2350 / 1540,
    code_phase_chips = 500.25, valid_at_sample = 0, tap_sample_shifts = (-2, 0, 2),
    num_taps = 3, sampling_freq_hz = fs))

for block = 1:1500
    samples = ...            # 4000 Complex{Int16} samples, 12-bit range
    feed!(emu, samples)      # one antenna; pass a second vector for RX2
    service_pass!(core; wait_ms = 0)
    # Read the loop's per-epoch states from event_ring(segment, 1) here.
end
```

`M2SDRLoop/test/runtests.jl` has a complete version, with a synthetic GPS L1 C/A
signal at 45 dB-Hz. It checks that the loop pulls a handover that is 20 Hz and a
quarter chip off onto the satellite.

## Running the tests

Neither package's tests need a board:

```sh
julia --project=M2SDRLoop -e 'using Pkg; Pkg.test()'   # driver and emulator, closed loop
julia --project -e 'using Pkg; Pkg.test()'             # tag decoding, the loop command
```
