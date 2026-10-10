#!/bin/bash
# Run a command (a docker build) and, if it failed on Docker Hub's pull rate
# limit (429 Too Many Requests / toomanyrequests), run it once more after a
# minute. Any other failure is returned as it is.
set -uo pipefail

log=$(mktemp)
trap 'rm -f "$log"' EXIT

"$@" 2>&1 | tee "$log"
status=$?
if [ "$status" -ne 0 ] && grep -qiE 'toomanyrequests|429 Too Many Requests' "$log"; then
    echo "::warning title=Docker Hub rate limit::Hit the pull rate limit; retrying once in a minute"
    sleep "${RATE_LIMIT_RETRY_DELAY:-60}"
    "$@"
    status=$?
fi
exit "$status"
