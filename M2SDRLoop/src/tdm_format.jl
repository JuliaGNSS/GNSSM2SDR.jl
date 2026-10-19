# ─────────────────────────────────────────────────────────────────────────────
# The fixed formats of gnss-m2sdr's TDM correlator (`gnss_m2sdr/tdm/params.py`):
# number formats, the 64-byte record, the command register, the capability
# word. Keep in step with params.py; `test/runtests.jl` checks the record
# layout against bytes the Python model wrote.
# ─────────────────────────────────────────────────────────────────────────────

# Interface revision 2 added the sub-chip replica (`s_subcarrier`, `s_amplitude`);
# a revision-1 gateware replicates BPSK only.
const TDM_GNSS_ID = 0x7D4C0002
const TDM_GNSS_ID_V1 = 0x7D4C0001

const CARRIER_LUT_BITS = 6
const CARRIER_AMPLITUDE = 127
const CHIP_BITS = 20
const HISTORY = 32
const MAX_TAPS = 5
const CODE_WORD_BITS = 32
const MAX_SUBCHIPS = 12
const AMP_BITS = 7

const RECORD_WORDS = 8
const RECORD_BYTES = 64
const RECORD_MAGIC = 0x4E47
const RECORD_VERSION = 1

const KIND_PAD = 0
const KIND_DUMP = 1
const KIND_STROBE = 2

const FLAG_SATURATED = 0x01
const FLAG_SAMPLE_OVERFLOW = 0x02  # samples were lost (the sample index jumped) during this record

# A slot's state is written back 7 cycles after it is read, so a round has at
# least 7 slots (`params.num_slots`).
const WRITEBACK_STAGE = 7

# Epoch strobes sit on the multiples of the strobe period in the sample index.
# After a reset or a write of the period, the first sample processed anchors the
# period's phase: it and the STROBE_SYNC_SAMPLES - 1 after it give no strobe
# (`params.py`, "Epoch strobes").
const STROBE_SYNC_SAMPLES = 8

const CMD_WORD = 1
const CMD_ARM = 2
const CMD_RELEASE = 3

const CONTROL_ENABLE = 0x1
const CONTROL_RECORDS = 0x2
const CONTROL_RESET = 0x4
const CONTROL_TAG = 0x8

"""
The gateware's build limits, decoded from `gnss_caps`.
"""
struct TDMCapabilities
    num_channels::Int
    num_slots::Int
    max_taps::Int
    history::Int
    code_words::Int
end

"""
    decode_capabilities(word) -> TDMCapabilities

The [`TDMCapabilities`](@ref) packed in the `gnss_caps` register.
"""
decode_capabilities(word::Integer) = TDMCapabilities(
    Int(word & 0xFF),
    Int((word >> 8) & 0xFF),
    Int((word >> 16) & 0xF),
    Int((word >> 20) & 0x3F),
    1 << Int((word >> 26) & 0x1F),
)

"""
    tap_layout(shifts) -> (lead, offsets, warmup)

The gateware's view of a tap layout given as sample shifts (positive = early,
in record order): the code NCO runs `lead` samples ahead of the prompt, tap `k`
reads replica-history entry `offsets[k] = lead - shifts[k]`, and `warmup`
samples fill the history before the first one is integrated. `nothing` if the
shifts do not fit the history.
"""
function tap_layout(shifts::NTuple{N,<:Integer}, num_taps::Integer) where {N}
    # Bound once, never reassigned: the closure below must not box it (a boxed
    # capture is a dynamic call `--trim` refuses).
    lead = _lead(shifts, num_taps)
    offsets = ntuple(k -> k <= num_taps ? lead - Int(shifts[k]) : 0, Val(MAX_TAPS))
    warmup = lead
    for k = 1:num_taps
        d = offsets[k]
        (0 <= d < HISTORY) || return nothing
        warmup = max(warmup, d)
    end
    (lead, offsets, warmup)
end

function _lead(shifts::Tuple, num_taps::Integer)
    lead = 0
    for k = 1:num_taps
        lead = max(lead, Int(shifts[k]))
    end
    lead
end

# Signed and unsigned fixed-point words.
carrier_word(freq_hz::Float64, fs::Float64) =
    UInt32(mod(round(Int64, freq_hz / fs * 4294967296.0), Int64(4294967296)))
code_word(rate_hz::Float64, fs::Float64) = round(Int64, rate_hz / fs * 4294967296.0)

# Two's complement of a 32-bit field.
@inline signed32(x::UInt64) = Int64(reinterpret(Int32, UInt32(x & 0xFFFFFFFF)))

"""
    TDMRecord

One 64-byte block off DMA1, decoded: a correlator dump, an epoch strobe or
padding (`kind`). Channels are the gateware's, counted from 0.
"""
struct TDMRecord
    kind::Int
    channel::Int
    num_taps::Int
    antenna::Int
    flags::Int
    assign_id::UInt16
    sample_index::Int64
    integrated::Int64
    code_chip::Int64
    code_frac20::Int64
    taps::NTuple{MAX_TAPS,ComplexF64}
end

const EMPTY_TAPS = ntuple(_ -> complex(0.0, 0.0), Val(MAX_TAPS))

@inline function _word(buf::Vector{UInt8}, offset::Int)
    GC.@preserve buf unsafe_load(Ptr{UInt64}(pointer(buf) + offset))
end

"""
    decode_record(buf, offset) -> TDMRecord

Decode the 64-byte block at byte `offset` (0-based) of `buf`. A block without
the magic is padding.
"""
function decode_record(buf::Vector{UInt8}, offset::Int)
    w0 = _word(buf, offset)
    if (w0 & 0xFFFF) != RECORD_MAGIC
        return TDMRecord(KIND_PAD, 0, 0, 0, 0, 0x0000, 0, 0, 0, 0, EMPTY_TAPS)
    end
    w1 = _word(buf, offset + 8)
    w2 = _word(buf, offset + 16)
    taps = ntuple(Val(MAX_TAPS)) do k
        w = _word(buf, offset + 16 + 8k)
        complex(Float64(signed32(w)), Float64(signed32(w >> 32)))
    end
    TDMRecord(
        Int((w0 >> 20) & 0xF),
        Int((w0 >> 24) & 0xFF),
        Int((w0 >> 32) & 0x7),
        Int((w0 >> 35) & 0x1),
        Int((w0 >> 36) & 0x3F),
        UInt16((w0 >> 48) & 0xFFFF),
        Int64(w1 & 0xFFFFFFFFFFFF),
        Int64(w2 & 0xFFFFFF),
        Int64((w2 >> 24) & 0xFFFFF),
        Int64((w2 >> 44) & 0xFFFFF),
        taps,
    )
end

code_words_for(chips::Integer) = (Int(chips) + 31) >> 5

# ── The replica ──────────────────────────────────────────────────────────────

# `s_subcarrier`: [11:0] sign (1 = negative), [23:12] level (1 = the high
# amplitude), [27:24] sub-chips per chip; `s_amplitude`: [6:0] high, [14:8] low.
# The reset values are a BPSK replica of amplitude 1.
const SUBCARRIER_BPSK = UInt32(1) << (2 * MAX_SUBCHIPS)
const AMPLITUDE_UNIT = UInt32(0x0101)

"""
    SubchipReplica

How the gateware replicates a signal: chip `j`, sub-chip `k` (of `subchips`
equal parts of the chip) is `code[j] * sign[k] * (level[k] ? amp_hi : amp_lo)`,
packed into the `s_subcarrier` and `s_amplitude` words. `supported` is false
for a signal whose replica does not factor that way (TMBOC, a baked secondary
code) or does not fit the table.
"""
struct SubchipReplica
    supported::Bool
    subchips::Int
    subcarrier::UInt32
    amplitude::UInt32
end

const UNSUPPORTED_REPLICA = SubchipReplica(false, 1, SUBCARRIER_BPSK, AMPLITUDE_UNIT)

is_bpsk(r::SubchipReplica) =
    r.subcarrier == SUBCARRIER_BPSK && r.amplitude == AMPLITUDE_UNIT

"""
    subchip_replica(signal) -> SubchipReplica

The gateware's replica of `signal`: GNSSSignals' own Int8 code table — the one
Tracking.jl correlates with, ±1 for BPSK and BOC, ±25/±13 for Galileo E1B's
CBOC (`get_code_amplitude` is its RMS) — factored into each chip's sign and a
per-sub-chip sign and amplitude, checked to factor for every PRN.
"""
function subchip_replica(signal::AbstractGNSSSignal)
    lut = signal.lut
    P = lut.subchip_factor
    L = get_code_length(signal)
    (1 <= P <= MAX_SUBCHIPS && lut.table_length == L * P) || return UNSUPPORTED_REPLICA
    table = lut.padded
    # The sub-chip pattern s[k], signed so that s[1] > 0: the first chip of the first PRN.
    flip = table[1, 1] < 0 ? -1 : 1
    hi = 0
    lo = typemax(Int)
    for k = 1:P
        v = abs(Int(table[k, 1]))
        v == 0 && return UNSUPPORTED_REPLICA
        hi = max(hi, v)
        lo = min(lo, v)
    end
    hi < (1 << AMP_BITS) || return UNSUPPORTED_REPLICA
    sign = UInt32(0)
    level = UInt32(0)
    for k = 1:P
        v = flip * Int(table[k, 1])
        abs(v) == hi || abs(v) == lo || return UNSUPPORTED_REPLICA
        v < 0 && (sign |= UInt32(1) << (k - 1))
        abs(v) == hi && hi != lo && (level |= UInt32(1) << (k - 1))
    end
    # Every chip of every PRN is its first sub-chip's sign times that pattern.
    for prn = 1:size(table, 2), j = 0:(L-1)
        c = table[j*P+1, prn] < 0 ? -1 : 1
        for k = 1:P
            Int(table[j*P+k, prn]) == c * flip * Int(table[k, 1]) ||
                return UNSUPPORTED_REPLICA
        end
    end
    SubchipReplica(
        true,
        P,
        sign | level << MAX_SUBCHIPS | UInt32(P) << (2 * MAX_SUBCHIPS),
        UInt32(hi) | UInt32(lo) << 8,
    )
end

"""
Chip `j` (from 0) of `prn`'s code is negative: the sign of its first sub-chip in GNSSSignals' table.
"""
@inline chip_negative(
    signal::AbstractGNSSSignal,
    r::SubchipReplica,
    prn::Integer,
    j::Integer,
) = signal.lut.padded[j*r.subchips+1, prn] < 0
