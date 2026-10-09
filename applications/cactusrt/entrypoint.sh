#!/bin/bash
set -e

# cactus-rt locks all memory (mlockall) and runs its loop with SCHED_FIFO.
# Without the container privileges for that it aborts; say why up front.
if [ "$(ulimit -l)" != "unlimited" ]; then
    echo "entrypoint: memlock limit is $(ulimit -l) KiB, mlockall may fail;" \
         "run with --ulimit memlock=-1 (compose: ulimits.memlock: -1)" >&2
fi
cap_eff="$(awk '/^CapEff:/ {print $2}' /proc/self/status)"
# CAP_SYS_NICE is capability bit 23
if [ "$(ulimit -r)" = "0" ] && (( ((16#${cap_eff:-0}) >> 23 & 1) == 0 )); then
    echo "entrypoint: real-time priorities are not allowed;" \
         "run with --cap-add SYS_NICE --ulimit rtprio=99" >&2
fi

exec "$@"
