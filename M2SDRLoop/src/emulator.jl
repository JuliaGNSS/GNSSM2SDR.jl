# ─────────────────────────────────────────────────────────────────────────────
# A software copy of the TDM correlator behind the driver's `TDMIO` calls: the
# register file, the commands, the channel engine and the record stream, in the
# gateware's integer arithmetic (a port of gnss-m2sdr's `gnss_m2sdr/tdm/model.py`,
# which the gateware is checked against record for record). Feed it samples with
# `feed!`; the driver cannot tell it from the board.
# ─────────────────────────────────────────────────────────────────────────────

const EMU_REG_COUNT = 32

"""
The register layout of the emulator: consecutive words, in `TDM_REG_NAMES` order.
"""
emulator_regs() = TDMRegs(
    (UInt32(4 * (k - 1)) for k = 1:length(TDM_REG_NAMES))...,
    UInt32(4 * 28),
    UInt32(4 * 29),
    UInt32(4 * 30),
    UInt32(4 * 31),
)

mutable struct EmuChannel
    # Configuration.
    carrier_freq::Int64          # signed word
    code_freq::Int64
    code_length::Int
    code_base::Int
    lead::Int
    offsets::NTuple{MAX_TAPS,Int}
    num_taps::Int
    warmup::Int
    antenna::Int
    assign_id::UInt16
    subcarrier::UInt32
    amplitude::UInt32
    # Pending start.
    start::Int64
    p_carrier_phase::UInt32
    p_code_chip::Int
    p_code_frac::UInt32
    # Flags.
    enabled::Bool
    pending::Bool
    missed::Bool
    # State.
    carrier_phase::UInt32
    code_chip::Int
    code_frac::UInt32
    hist_rep::UInt32
    hist_wrap::UInt32
    hist_lvl::UInt32
    integ::Int
    warm::Int
    sat::Bool
    lost::Bool                   # the sample index jumped since the last dump
    acc::MVector{10,Int64}
end

EmuChannel() = EmuChannel(
    0,
    0,
    1023,
    0,
    0,
    ntuple(_ -> 0, Val(MAX_TAPS)),
    3,
    0,
    0,
    0x0000,
    SUBCARRIER_BPSK,
    AMPLITUDE_UNIT,
    0,
    0x00000000,
    0,
    0x00000000,
    false,
    false,
    false,
    0x00000000,
    0,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0,
    0,
    false,
    false,
    zero(MVector{10,Int64}),
)

"""
    TDMEmulator(; n_channels = 8, code_words = 4096, revision = 2)

The TDM correlator in software (`revision = 1`: one that reports the
interface revision without the sub-chip replica, to try the driver against). `feed!(emu, samples0, samples1)` correlates one
block of complex samples per antenna (12-bit integers, as the AD9361 delivers)
and queues the records for the driver to read.
"""
mutable struct TDMEmulator <: TDMIO
    const regs::Vector{UInt32}
    const channels::Vector{EmuChannel}
    const code::Vector{UInt32}
    const cos_table::NTuple{64,Int}
    const sin_table::NTuple{64,Int}
    const out::Vector{UInt8}
    out_head::Int
    sample_index::Int64
    last_n::Int64                # the previous sample processed, -1 after a reset
    strobe_sync::Bool            # anchor the strobe period's phase at the next sample
    strobe_hold::Int             # samples still to pass before the first strobe check
    strobe_next::Int64           # the next multiple of the period to strobe
    code_ptr::Int
    dma_open::Bool
end

function TDMEmulator(;
    n_channels::Integer = 8,
    code_words::Integer = 4096,
    revision::Integer = 2,
)
    regs = zeros(UInt32, EMU_REG_COUNT)
    regs[1] = revision == 1 ? TDM_GNSS_ID_V1 : TDM_GNSS_ID
    regs[31] = SUBCARRIER_BPSK
    regs[32] = AMPLITUDE_UNIT
    regs[2] =
        UInt32(n_channels) | UInt32(max(n_channels + 1, WRITEBACK_STAGE)) << 8 |
        UInt32(MAX_TAPS) << 16 | UInt32(HISTORY) << 20 |
        UInt32(trailing_zeros(code_words)) << 26
    cos_t = ntuple(k -> round(Int, CARRIER_AMPLITUDE * cos(2π * (k - 1) / 64)), Val(64))
    sin_t = ntuple(k -> round(Int, CARRIER_AMPLITUDE * sin(2π * (k - 1) / 64)), Val(64))
    TDMEmulator(
        regs,
        [EmuChannel() for _ = 1:n_channels],
        zeros(UInt32, code_words),
        cos_t,
        sin_t,
        UInt8[],
        0,
        0,
        -1,
        true,
        0,
        0,
        0,
        false,
    )
end

const _EMU = emulator_regs()

function reg_read(emu::TDMEmulator, addr::UInt32)
    a = addr
    a == _EMU.sample_count && return UInt32(emu.sample_index & 0xFFFFFFFF)
    a == _EMU.sample_count_hi && return UInt32((emu.sample_index >> 32) & 0xFFFF)
    a == _EMU.enabled && return _bitmap(c -> c.enabled, emu)
    a == _EMU.pending && return _bitmap(c -> c.pending, emu)
    a == _EMU.missed && return _bitmap(c -> c.missed, emu)
    emu.regs[Int(a÷4)+1]
end

function _bitmap(f, emu::TDMEmulator)
    m = UInt32(0)
    for (i, c) in enumerate(emu.channels)
        f(c) && (m |= UInt32(1) << (i - 1))
    end
    m
end

reg(emu::TDMEmulator, addr::UInt32) = emu.regs[Int(addr÷4)+1]

function reg_write(emu::TDMEmulator, addr::UInt32, value::Integer)
    v = UInt32(value & 0xFFFFFFFF)
    emu.regs[Int(addr÷4)+1] = v
    if addr == _EMU.cmd
        _command!(emu, Int(v & 0x3), Int((v >> 8) & 0xFF) + 1)
    elseif addr == _EMU.code_addr
        emu.code_ptr = Int(v & 0xFFFF)
    elseif addr == _EMU.code_data
        emu.code[mod(emu.code_ptr, length(emu.code))+1] = v
        emu.code_ptr += 1
    elseif addr == _EMU.strobe_period
        _strobe_resync!(emu)
    elseif addr == _EMU.control && (v & CONTROL_RESET) != 0
        for c in emu.channels
            c.enabled = c.pending = c.missed = false
        end
        empty!(emu.out)
        emu.out_head = 0
        emu.last_n = -1
        _strobe_resync!(emu)
    end
    nothing
end

_strobe_resync!(emu::TDMEmulator) = (emu.strobe_sync = true; emu.strobe_hold = 0; emu)

function _command!(emu::TDMEmulator, op::Int, ch::Int)
    1 <= ch <= length(emu.channels) || return nothing
    c = emu.channels[ch]
    if op == CMD_WORD || op == CMD_ARM
        c.carrier_freq = signed32(UInt64(reg(emu, _EMU.s_carrier_freq)))
        c.code_freq = Int64(reg(emu, _EMU.s_code_freq))
    end
    if op == CMD_ARM
        taps = reg(emu, _EMU.s_taps)
        misc = reg(emu, _EMU.s_misc)
        c.code_length = Int(reg(emu, _EMU.s_code_length) & 0xFFFFF)
        c.code_base = Int(reg(emu, _EMU.s_code_base) & 0xFFFF)
        c.offsets = ntuple(k -> Int((taps >> (5 * (k - 1))) & 0x1F), Val(MAX_TAPS))
        c.num_taps = Int((taps >> 25) & 0x7)
        c.lead = Int(misc & 0x1F)
        c.warmup = Int((misc >> 5) & 0x1F)
        c.antenna = Int((misc >> 10) & 0x1)
        c.assign_id = UInt16(misc >> 16)
        c.subcarrier = reg(emu, _EMU.s_subcarrier)
        c.amplitude = reg(emu, _EMU.s_amplitude)
        c.start =
            Int64(reg(emu, _EMU.s_start_lo)) |
            Int64(reg(emu, _EMU.s_start_hi) & 0xFFFF) << 32
        c.p_carrier_phase = reg(emu, _EMU.s_carrier_phase)
        c.p_code_chip = Int(reg(emu, _EMU.s_code_chip) & 0xFFFFF)
        c.p_code_frac = reg(emu, _EMU.s_code_frac)
        c.pending = true
        c.enabled = false
        c.missed = false
    elseif op == CMD_RELEASE
        c.enabled = false
        c.pending = false
    end
    nothing
end

dma_open!(emu::TDMEmulator) = (emu.dma_open = true; emu)
dma_close!(emu::TDMEmulator) = (emu.dma_open = false; emu)
dma_wait(emu::TDMEmulator, ::Integer) = emu.out_head < length(emu.out)

function dma_read!(emu::TDMEmulator, buf::Vector{UInt8}, filled::Int)
    avail = length(emu.out) - emu.out_head
    n = min(avail, length(buf) - filled)
    n <= 0 && return 0
    copyto!(buf, filled + 1, emu.out, emu.out_head + 1, n)
    emu.out_head += n
    if emu.out_head == length(emu.out)
        empty!(emu.out)
        emu.out_head = 0
    end
    n
end

# ── The engine ───────────────────────────────────────────────────────────────

"""
    feed!(emu, antenna0, antenna1 = antenna0) -> emu

Correlate one block of samples (`Complex{<:Integer}`, 12-bit range) per antenna.
"""
function feed!(emu::TDMEmulator, a0::AbstractVector, a1::AbstractVector = a0)
    control = reg(emu, _EMU.control)
    (control & CONTROL_ENABLE) == 0 && return emu
    period = Int64(reg(emu, _EMU.strobe_period))
    records_on = (control & CONTROL_RECORDS) != 0 && emu.dma_open
    @inbounds for k in eachindex(a0)
        n = emu.sample_index
        s0 = a0[k]
        s1 = a1[k]
        gap = emu.last_n >= 0 && n != emu.last_n + 1
        emu.last_n = n
        for (i, c) in enumerate(emu.channels)
            rec = _step_channel!(emu, c, n, c.antenna == 0 ? s0 : s1, gap)
            rec && records_on && _emit_dump!(emu, i - 1, c, n)
            rec && _clear_after_dump!(c)
        end
        if period != 0
            if emu.strobe_sync
                x = n + 1 + STROBE_SYNC_SAMPLES
                emu.strobe_next = cld(x, period) * period
                emu.strobe_hold = STROBE_SYNC_SAMPLES - 1
                emu.strobe_sync = false
            elseif emu.strobe_hold > 0
                emu.strobe_hold -= 1
            elseif n + 1 >= emu.strobe_next
                records_on && _emit_strobe!(emu, n)
                emu.strobe_next += period
                # A gap longer than a period: re-anchor at the next sample.
                emu.strobe_next <= n + 1 && (emu.strobe_sync = true)
            end
        end
        emu.sample_index = n + 1
    end
    emu
end

@inline function _saturate32(v::Int64)
    v > typemax(Int32) && return (Int64(typemax(Int32)), true)
    v < typemin(Int32) && return (Int64(typemin(Int32)), true)
    (v, false)
end

# One channel, one sample; `true` when the prompt's code period ended (the
# caller emits the record, then clears the accumulators).
function _step_channel!(emu::TDMEmulator, c::EmuChannel, n::Int64, sample, gap::Bool)
    if c.pending
        if n == c.start
            c.carrier_phase = c.p_carrier_phase
            c.code_chip = c.p_code_chip
            c.code_frac = c.p_code_frac
            c.hist_rep = 0x00000000
            c.hist_wrap = 0x00000000
            c.hist_lvl = 0x00000000
            c.integ = 0
            c.warm = c.warmup
            c.sat = false
            c.lost = false
            fill!(c.acc, 0)
            gap = false              # samples lost before the start do not concern it
            c.enabled = true
            c.pending = false
        elseif n > c.start
            c.missed = true
            c.pending = false
        end
    end
    c.enabled || return false
    c.lost |= gap
    idx = Int(c.carrier_phase >> (32 - CARRIER_LUT_BITS)) + 1
    co = emu.cos_table[idx]
    si = emu.sin_table[idx]
    i = Int(real(sample))
    q = Int(imag(sample))
    bb_i = i * co + q * si
    bb_q = q * co - i * si
    c.carrier_phase += reinterpret(UInt32, Int32(c.carrier_freq))
    # The chip's sign times the sub-chip's, and the sub-chip's level.
    word = emu.code[mod(c.code_base+(c.code_chip>>5), length(emu.code))+1]
    subchips = (c.subcarrier >> (2 * MAX_SUBCHIPS)) & 0xF
    sub = Int((UInt64(c.code_frac) * subchips) >> 32)
    bit = ((word >> (c.code_chip & 31)) ⊻ (c.subcarrier >> sub)) & 0x1
    level = (c.subcarrier >> (MAX_SUBCHIPS + sub)) & 0x1
    total = UInt64(c.code_frac) + UInt64(c.code_freq)
    c.code_frac = UInt32(total & 0xFFFFFFFF)
    chip = c.code_chip + Int(total >> 32)
    wrap = chip == c.code_length
    c.code_chip = wrap ? 0 : chip
    c.hist_rep = (c.hist_rep << 1) | bit
    c.hist_wrap = (c.hist_wrap << 1) | UInt32(wrap)
    c.hist_lvl = (c.hist_lvl << 1) | level
    prompt_wrap = ((c.hist_wrap >> c.lead) & 0x1) != 0
    if c.warm > 0
        c.warm -= 1
        return false
    end
    for t = 1:c.num_taps
        negative = ((c.hist_rep >> c.offsets[t]) & 0x1) != 0
        high = ((c.hist_lvl >> c.offsets[t]) & 0x1) != 0
        amp = Int((c.amplitude >> (high ? 0 : 8)) & 0x7F)
        vi, si1 = _saturate32(c.acc[2t-1] + (negative ? -amp * bb_i : amp * bb_i))
        vq, sq1 = _saturate32(c.acc[2t] + (negative ? -amp * bb_q : amp * bb_q))
        c.acc[2t-1] = vi
        c.acc[2t] = vq
        c.sat |= si1 | sq1
    end
    c.integ += 1
    prompt_wrap
end

function _clear_after_dump!(c::EmuChannel)
    fill!(c.acc, 0)
    c.integ = 0
    c.sat = false
    c.lost = false
    nothing
end

function _push_word!(out::Vector{UInt8}, w::UInt64)
    for b = 0:7
        push!(out, UInt8((w >> (8b)) & 0xFF))
    end
end

function _emit_dump!(emu::TDMEmulator, channel::Int, c::EmuChannel, n::Int64)
    w0 =
        UInt64(RECORD_MAGIC) | UInt64(RECORD_VERSION) << 16 | UInt64(KIND_DUMP) << 20 |
        UInt64(channel) << 24 | UInt64(c.num_taps) << 32 | UInt64(c.antenna) << 35 |
        UInt64((c.sat ? FLAG_SATURATED : 0x00) | (c.lost ? FLAG_SAMPLE_OVERFLOW : 0x00)) <<
        36 | UInt64(c.assign_id) << 48
    w1 = UInt64(n + 1) & 0xFFFFFFFFFFFF
    w2 =
        UInt64(c.integ & 0xFFFFFF) | UInt64(c.code_chip & 0xFFFFF) << 24 |
        UInt64(c.code_frac >> 12) << 44
    _push_word!(emu.out, w0)
    _push_word!(emu.out, w1)
    _push_word!(emu.out, w2)
    for t = 1:MAX_TAPS
        i = UInt64(reinterpret(UInt32, Int32(c.acc[2t-1])))
        q = UInt64(reinterpret(UInt32, Int32(c.acc[2t])))
        _push_word!(emu.out, i | q << 32)
    end
end

function _emit_strobe!(emu::TDMEmulator, n::Int64)
    _push_word!(
        emu.out,
        UInt64(RECORD_MAGIC) | UInt64(RECORD_VERSION) << 16 | UInt64(KIND_STROBE) << 20,
    )
    _push_word!(emu.out, UInt64(n + 1) & 0xFFFFFFFFFFFF)
    for _ = 3:RECORD_WORDS
        _push_word!(emu.out, UInt64(0))
    end
end
