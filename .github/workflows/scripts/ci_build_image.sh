#!/bin/bash
# Build one application image for .github/workflows/images.yml with the same
# script the Makefile targets run (applications/_build_it.sh), so CI builds
# exactly what developers build.
#
#   ci_build_image.sh <app>          derived image: deleted afterwards, together
#                                    with the build cache, to give the disk back
#   ci_build_image.sh <app> --keep   family base: kept for the images built
#                                    FROM it until the job ends
#
# Needs REPOSITORY and PLATFORM in the environment (set by the workflow). Exits
# with the build's status after cleaning up, and adds a line to the job summary.
set -uo pipefail

app=${1:?usage: ci_build_image.sh <app> [--keep]}
keep=${2:-}
scripts_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
apps_dir="$(cd "$scripts_dir/../../../applications" && pwd)"
summary=${GITHUB_STEP_SUMMARY:-/dev/null}

# Same tag _build_it.sh gives the image
tag=latest
[ "${PLATFORM:-amd64}" = arm64 ] && tag=arm64
image="${REPOSITORY:?REPOSITORY is not set}:${app}-${tag}"

start=$SECONDS
status=0
(cd "$apps_dir" && APP="$app" "$scripts_dir/retry_on_rate_limit.sh" ./_build_it.sh) || status=$?
elapsed=$((SECONDS - start))
took="$((elapsed / 60))m$((elapsed % 60))s"

if [ "$status" -eq 0 ]; then
    size=$(docker image inspect -f '{{.Size}}' "$image" | numfmt --to=iec --suffix=B)
    echo "- \`${app}\`: built in ${took}, ${size}" >> "$summary"
else
    echo "::error title=Image build failed::${app} (applications/${app}/Dockerfile) exited with ${status}"
    echo "- \`${app}\`: **FAILED** after ${took}" >> "$summary"
fi

if [ "$keep" != --keep ]; then
    docker image rm "$image" > /dev/null 2>&1 || true
    docker builder prune --all --force > /dev/null || true
fi
df -h /

exit "$status"
