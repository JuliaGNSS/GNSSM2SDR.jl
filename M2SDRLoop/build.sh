#!/usr/bin/env bash
# Build the gnss_loop executable with JuliaC (`--trim=safe`) on the target.
#   ./build.sh            build into build/gnss_loop
#   JULIA="julia +1.13" JULIAC=juliac ./build.sh
set -uo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.juliaup/bin:$HOME/.julia/bin:$PATH"
JULIA="${JULIA:-julia +1.13}"
JULIAC="${JULIAC:-juliac}"
mkdir -p build
echo "julia:  $($JULIA --version)"
echo "juliac: $($JULIAC --version 2>&1 | head -1)"
EXPERIMENTAL=""
$JULIAC --help 2>&1 | grep -q -- "--experimental" && EXPERIMENTAL="--experimental"
$JULIA --project=. -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()' > build/instantiate.log 2>&1 || { tail -30 build/instantiate.log; exit 1; }
t0=$(date +%s)
rm -f build/gnss_loop
( cd build && $JULIAC --output-exe gnss_loop --project .. --trim=safe $EXPERIMENTAL ../bin/gnss_loop.jl ) > build/juliac.log 2>&1
t1=$(date +%s)
errors=$(grep -c "^Verifier error" build/juliac.log | tr -dc '0-9')
if [ -x build/gnss_loop ]; then
    echo "gnss_loop: built in $((t1-t0)) s, $(stat -c%s build/gnss_loop) bytes, verifier errors: ${errors:-0}"
    # SCHED_FIFO (--fifo, remote_loop(...; fifo)) needs CAP_SYS_NICE; give it to
    # the binary so the receiver can spawn it unprivileged. Measured on the
    # Orin: it is what removes the scheduling tail under load (pinning alone
    # does not).
    if command -v setcap > /dev/null; then
        sudo -n setcap cap_sys_nice+ep build/gnss_loop 2> /dev/null && echo "gnss_loop: cap_sys_nice granted (SCHED_FIFO available without root)" || echo "gnss_loop: run 'sudo setcap cap_sys_nice+ep build/gnss_loop' to allow --fifo without root"
    fi
else
    echo "gnss_loop: FAILED in $((t1-t0)) s, verifier errors: ${errors:-0}"
    grep -m 20 -A 4 "^Verifier error" build/juliac.log | head -120
    tail -5 build/juliac.log
    exit 1
fi
