# ─────────────────────────────────────────────────────────────────────────────
# How the TDM driver reaches the gateware: 32-bit register reads and writes by
# address, and the DMA1 record stream. `LitePCIeTDM` is the board (the litepcie
# REG ioctl and `/dev/m2sdrN`); `TDMEmulator` (emulator.jl) is a software copy of
# the gateware behind the same four calls, for the tests and for trying the loop
# without a board.
# ─────────────────────────────────────────────────────────────────────────────

"""
    TDMIO

How a [`TDMDriver`](@ref) reaches the gateware: [`reg_read`](@ref),
[`reg_write`](@ref) and the DMA1 record stream. [`LitePCIeTDM`](@ref) is the
board, [`TDMEmulator`](@ref) the gateware in software.
"""
abstract type TDMIO end

"""
    TDMRegs

The addresses of every register the driver touches, resolved once from the
gateware's `csr.csv` (or laid out by the emulator), so the hot path never
looks a name up.
"""
struct TDMRegs
    id::UInt32
    caps::UInt32
    control::UInt32
    strobe_period::UInt32
    flush_cycles::UInt32
    buffer_words::UInt32
    sample_count::UInt32
    sample_count_hi::UInt32
    sample_overflows::UInt32
    record_drops::UInt32
    records::UInt32
    enabled::UInt32
    pending::UInt32
    missed::UInt32
    s_carrier_freq::UInt32
    s_code_freq::UInt32
    s_carrier_phase::UInt32
    s_code_frac::UInt32
    s_code_chip::UInt32
    s_code_length::UInt32
    s_code_base::UInt32
    s_taps::UInt32
    s_misc::UInt32
    s_start_lo::UInt32
    s_start_hi::UInt32
    cmd::UInt32
    code_addr::UInt32
    code_data::UInt32
    dma1_sync_bypass::UInt32     # 0 when the map has no DMA1 synchronizer
    phy_control::UInt32          # AD9361 PHY control (bit 0: 1R1T), 0 if absent
    s_subcarrier::UInt32         # the sub-chip replica; 0 on a revision-1 gateware
    s_amplitude::UInt32
end

const TDM_REG_NAMES = (
    "id",
    "caps",
    "control",
    "strobe_period",
    "flush_cycles",
    "buffer_words",
    "sample_count",
    "sample_count_hi",
    "sample_overflows",
    "record_drops",
    "records",
    "enabled",
    "pending",
    "missed",
    "s_carrier_freq",
    "s_code_freq",
    "s_carrier_phase",
    "s_code_frac",
    "s_code_chip",
    "s_code_length",
    "s_code_base",
    "s_taps",
    "s_misc",
    "s_start_lo",
    "s_start_hi",
    "cmd",
    "code_addr",
    "code_data",
)

function TDMRegs(csr::LiteXCSR; prefix::AbstractString = "gnss_")
    addr(name) = begin
        key = string(prefix, name)
        haskey(csr.regs, key) || throw(
            ArgumentError("the CSR map has no $key: not a TDM-correlator gateware"),
        )
        csr.regs[key][1]
    end
    optional(key) = haskey(csr.regs, key) ? csr.regs[key][1] : UInt32(0)
    TDMRegs(
        map(addr, TDM_REG_NAMES)...,
        optional("pcie_dma1_synchronizer_bypass"),
        optional("ad9361_phy_control"),
        optional(string(prefix, "s_subcarrier")),
        optional(string(prefix, "s_amplitude")),
    )
end

"""
    LitePCIeTDM(csr; dma_device = "/dev/m2sdr1", dma_buffer_size = 512)

The board: registers through `csr` (a [`LiteXCSR`](@ref) on `/dev/m2sdrN`), records
through the DMA1 character device, read in whole buffers of the kernel module's
`dma_buffer_size` for channel 1.
"""
mutable struct LitePCIeTDM <: TDMIO
    const csr::LiteXCSR
    const dma_device::String
    const dma_buffer_size::Int
    stream::Union{Nothing,DMAWriterStream}
end

LitePCIeTDM(
    csr::LiteXCSR;
    dma_device::AbstractString = "/dev/m2sdr1",
    dma_buffer_size::Integer = 512,
) = LitePCIeTDM(csr, String(dma_device), Int(dma_buffer_size), nothing)

"""
    reg_read(io, addr) -> UInt32

Read the 32-bit register at byte address `addr` (a field of [`TDMRegs`](@ref)).
"""
reg_read(io::LitePCIeTDM, addr::UInt32) = readl(io.csr, addr)

"""
    reg_write(io, addr, value)

Write `value` to the 32-bit register at byte address `addr` (a field of
[`TDMRegs`](@ref)).
"""
reg_write(io::LitePCIeTDM, addr::UInt32, value::Integer) = writel(io.csr, addr, value)

function dma_open!(io::LitePCIeTDM)
    isnothing(io.stream) || return io
    io.stream = DMAWriterStream(io.dma_device; buffer_size = io.dma_buffer_size)
    io
end

function dma_close!(io::LitePCIeTDM)
    s = io.stream
    isnothing(s) || close(s)
    io.stream = nothing
    io
end

# Whole DMA buffers into `buf` after its first `filled` bytes; the number of
# bytes read (0 when none are complete).
function dma_read!(io::LitePCIeTDM, buf::Vector{UInt8}, filled::Int)
    s = io.stream
    isnothing(s) && return 0
    room = length(buf) - filled
    room < io.dma_buffer_size && return 0
    n = _read_into!(s.fd, buf, filled)
    max(n, 0)
end

function dma_wait(io::LitePCIeTDM, timeout_ms::Integer)
    s = io.stream
    isnothing(s) && return false
    _poll_readable(s.fd, timeout_ms)
end
