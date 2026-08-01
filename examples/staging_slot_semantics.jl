# What the gateware's scheduled-commit path actually does when the host arms a
# second commit while the first is still pending — measured on the board, not
# assumed.
#
# The staged path (`carrier_freq_next`/`code_freq_next` + `apply_at` + `apply.arm`)
# is what gives an NCO update a deterministic apply point. It has exactly ONE
# staging slot per channel and a single `armed` bit, so it is not a queue: this
# script demonstrates the two consequences that matter to a tracking loop, and
# they are the reason streaming corrections go through the *immediate*
# `carrier_freq`/`code_freq` CSRs instead (see `schedule!`'s docstring and
# gnss-m2sdr's `ChannelWithCSR`):
#
#   1. Re-arming replaces the pending commit. The first commit never fires at
#      the sample the host picked; only the second one fires, at its own target.
#   2. At a tracking-loop update rate (~1 kHz, each commit scheduled 1-2 epochs
#      ahead) EVERY commit is replaced ~1 ms before it was due, so across a
#      whole second of updates the replicas never see a single new NCO word —
#      the channel free-runs while the host believes it is steering.
#
# The commit compare is gated on the channel's sample strobe, so samples have to
# be flowing: this needs the RX datapath streaming (an `m2sdr_record` into
# /dev/null is enough) and the bank enabled.
#
# Usage: julia --project=. staging_slot_semantics.jl CSR_CSV [CHANNEL]

using Printf
using Unitful: Hz

using GNSSM2SDR:
    LiteXCSR,
    GNSSBank,
    schedule!,
    apply_status,
    applied_at,
    sample_count,
    enable!,
    carrier_word

const FS = 4e6Hz
const FS_HZ = 4e6

csr_csv = length(ARGS) >= 1 ? ARGS[1] :
          error("usage: staging_slot_semantics.jl CSR_CSV [CHANNEL]")
hw_channel = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 1

csr = LiteXCSR(csr_csv)
bank = GNSSBank(csr; fs = FS)
ch = bank.channels[hw_channel]
enable!(bank, true)

# The counter must be advancing, or nothing can ever reach an apply point.
let a = sample_count(bank)
    sleep(0.2)
    b = sample_count(bank)
    b > a || error(
        "the bank's sample counter is not advancing ($a → $b): start the RX " *
        "datapath (e.g. `m2sdr_record /dev/null` in the background) first",
    )
    @printf("sample counter advancing: %+d samples in 0.2 s (≈%.2f MS/s)\n",
            b - a, (b - a) / 0.2 / 1e6)
end

# ── 1. A re-arm replaces the pending commit ──────────────────────────────────
@info "1. arming a commit, then re-arming before it is due"
before = applied_at(ch)
now = sample_count(bank)
target_a = now + round(Int, 0.4 * FS_HZ)     # 400 ms out
target_b = now + round(Int, 0.8 * FS_HZ)     # 800 ms out

schedule!(ch, target_a; carrier_hz = 1000.0)
armed_after_first = apply_status(ch).armed
@printf("   armed after the first arm: %s (a commit is pending)\n", armed_after_first)

schedule!(ch, target_b; carrier_hz = 2000.0)

# Wait past the *first* target and check nothing fired there.
while sample_count(bank) < target_a + round(Int, 0.05 * FS_HZ)
    sleep(0.01)
end
armed_past_a = apply_status(ch).armed
applied_past_a = applied_at(ch)
@printf("   past target A: armed=%s applied_at unchanged=%s\n",
        armed_past_a, applied_past_a == before)

# Wait past the second target; that is the one that fires.
deadline = time() + 2.0
while apply_status(ch).armed && time() < deadline
    sleep(0.01)
end
fired_at = applied_at(ch)
@printf("   after target B: armed=%s applied_at=%d (target A %d, target B %d)\n",
        apply_status(ch).armed, fired_at, target_a, target_b)
@printf("   → the commit fired at target %s\n",
        abs(fired_at - target_b) <= 8 ? "B: the first commit was REPLACED, not queued" :
        abs(fired_at - target_a) <= 8 ? "A (?!)" : "neither")

# ── 2. At a loop update rate, no commit ever matures ─────────────────────────
@info "2. re-arming at ~1 kHz with a 2 ms lead, the way a tracking loop would"
base = applied_at(ch)
epoch = round(Int, 1e-3 * FS_HZ)
n_updates = 500
for k = 1:n_updates
    schedule!(ch, sample_count(bank) + 2 * epoch; carrier_hz = 1000.0 + k)
    sleep(1e-3)
end
during = applied_at(ch)
@printf("   %d updates over ~%.1f s: applied_at moved %d times worth of samples (%s)\n",
        n_updates, n_updates * 1e-3, during - base,
        during == base ? "NOT ONE commit matured — the free-run bug" :
        "some commits matured")

# Let the last one land, so the channel is not left with a pending commit.
let deadline = time() + 1.0
    while apply_status(ch).armed && time() < deadline
        sleep(0.01)
    end
end
@printf("   final: armed=%s applied_at=%d (moved %d samples in total)\n",
        apply_status(ch).armed, applied_at(ch), applied_at(ch) - base)

enable!(bank, false)
close(csr)
