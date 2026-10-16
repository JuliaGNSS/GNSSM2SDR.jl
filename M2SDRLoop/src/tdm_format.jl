# ─────────────────────────────────────────────────────────────────────────────
# The fixed formats of gnss-m2sdr's TDM correlator (`gnss_m2sdr/tdm/params.py`):
# number formats, the 64-byte record, the command register, the capability
# word. Keep in step with params.py; `test/runtests.jl` checks the record
# layout against bytes the Python model wrote.
# ─────────────────────────────────────────────────────────────────────────────

const TDM_GNSS_ID = 0x7D4C0001

const CARRIER_LUT_BITS = 6
const CARRIER_AMPLITUDE = 127
const CHIP_BITS = 20
const HISTORY = 32
const MAX_TAPS = 5
const CODE_WORD_BITS = 32

const RECORD_WORDS = 8
const RECORD_BYTES = 64
const RECORD_MAGIC = 0x4E47
const RECORD_VERSION = 1

const KIND_PAD = 0
const KIND_DUMP = 1
const KIND_STROBE = 2

const FLAG_SATURATED = 0x01

const CMD_WORD = 1
const CMD_ARM = 2
const CMD_RELEASE = 3

const CONTROL_ENABLE = 0x1
const CONTROL_RECORDS = 0x2
const CONTROL_RESET = 0x4
const CONTROL_TAG = 0x8

"The gateware's build limits, decoded from `gnss_caps`."
struct TDMCapabilities
    num_channels::Int
    num_slots::Int
    max_taps::Int
    history::Int
    code_words::Int
end

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

"""
    pack_code_words!(words, chips) -> words

±1 chips into 32-chip code-memory words (bit set = -1, chip `k` at bit `k % 32`).
"""
function pack_code_words!(words::AbstractVector{UInt32}, chips::AbstractVector{<:Real})
    fill!(words, 0x00000000)
    @inbounds for k in eachindex(chips)
        if chips[k] < 0
            i = (k - 1) >> 5
            words[i+1] |= UInt32(1) << ((k - 1) & 31)
        end
    end
    words
end

code_words_for(chips::Integer) = (Int(chips) + 31) >> 5
