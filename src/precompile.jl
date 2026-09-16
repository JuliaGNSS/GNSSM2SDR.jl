# Precompile the interface the receiver drives at run time.
#
# Every method GNSSReceiver reaches through `invokelatest` — the handover, the
# release, the per-chunk device queries — and every task this package spawns is
# specialised on `M2SDRCorrelator`, a type only this package can name, so
# GNSSReceiver's own workload cannot compile them. Left to first use they
# compile *live*, on the receiver's processing task: measured on the board with
# `--trace-compile-timing`, 259 ms for the first `assign_channel!`, 291 ms for
# `start!`, 162 ms for the link constructor, 33 ms for the first
# `release_channel!` — and a compilation the GC has to wait for stalls every
# thread in the process, the device service task included, for its whole
# duration (GNSSReceiver.jl#107). None of these can be *executed* here (they
# touch the device), so they are precompiled by signature.
#
# The list is a constant so the test suite can assert that each statement
# resolves to a method: a signature that drifts from what the receiver calls
# silently compiles nothing.

const _PRECOMPILE_SDR = M2SDRCorrelator{1,correlator_type(Val(1))}
const _PRECOMPILE_FREQ = typeof(1.0Hz)
const _PRECOMPILE_SIGNAL = typeof(GPSL1CA())
# `SignalChannel{T,N}` leaves its storage type free; a one-element channel names
# the concrete type the constructor actually produces.
const _PRECOMPILE_RAW = typeof(SignalChannel{Complex{Int16},1}(1, 1))

const _PRECOMPILE_SIGNATURES = Tuple{Any,Tuple}[
    # Construction and the session
    (
        Core.kwcall,
        (
            NamedTuple{(:fs, :n_channels),Tuple{_PRECOMPILE_FREQ,Int}},
            Type{M2SDRCorrelator},
            String,
            _PRECOMPILE_RAW,
        ),
    ),
    (
        Core.kwcall,
        (
            NamedTuple{(:dump_source, :epoch_period),Tuple{Symbol,Int}},
            typeof(start!),
            _PRECOMPILE_SDR,
        ),
    ),
    (stop!, (_PRECOMPILE_SDR,)),
    # The receiver's device interface, with the argument types `_assign!` and
    # `_arm_noise_channel!` in GNSSReceiver actually pass. The configuration
    # form is what the link calls; the positional one is the compatibility path
    # a hand-built link may still take.
    (
        GNSSReceiver.assign_channel!,
        (_PRECOMPILE_SDR, Int, GNSSReceiver.HardwareChannelConfig{_PRECOMPILE_SIGNAL}),
    ),
    (
        Core.kwcall,
        (
            NamedTuple{(:el_sample_spacing, :signal),Tuple{Int,_PRECOMPILE_SIGNAL}},
            typeof(GNSSReceiver.assign_channel!),
            _PRECOMPILE_SDR,
            Int,
            Int,
            _PRECOMPILE_FREQ,
            _PRECOMPILE_FREQ,
            Float64,
            Int,
        ),
    ),
    (GNSSReceiver.hardware_capabilities, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.release_channel!, (_PRECOMPILE_SDR, Int)),
    (GNSSReceiver.dropped_dump_count!, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.assignment_start_sample, (_PRECOMPILE_SDR, Int)),
    (GNSSReceiver.correlator_gain, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.num_hardware_channels, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.raw_sample_channel, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.correlator_dump_channel, (_PRECOMPILE_SDR,)),
    (GNSSReceiver.nco_update_channel, (_PRECOMPILE_SDR,)),
    # The link over this device, as the example builds it.
    (
        Core.kwcall,
        (
            NamedTuple{
                (:sampling_freq, :reference_signal, :feedback_delay_epochs),
                Tuple{_PRECOMPILE_FREQ,_PRECOMPILE_SIGNAL,Int},
            },
            Type{GNSSReceiver.HardwareCorrelatorLink},
            _PRECOMPILE_SDR,
        ),
    ),
    # The service tasks and what they call.
    (_service_dma!, (_PRECOMPILE_SDR, Symbol, Int)),
    (_poll_dumps!, (_PRECOMPILE_SDR, Int)),
    (verify_handovers!, (_PRECOMPILE_SDR,)),
    (_drain_ncos!, (_PRECOMPILE_SDR,)),
    (_drain_ncos!, (_PRECOMPILE_SDR, Vector{Float64}, Vector{Float64})),
    # The raw stream.
    (Core.kwcall, (NamedTuple{(:chunk,),Tuple{Int}}, typeof(start_raw_stream))),
    (_read_raw!, (_PRECOMPILE_RAW, Cint, Int, Int, Int)),
    (Base.close, (RawStream{_PRECOMPILE_RAW},)),
]

for (f, argtypes) in _PRECOMPILE_SIGNATURES
    precompile(f, argtypes)
end
