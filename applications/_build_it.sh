#!/bin/bash

# Input args
[ -z "$REPOSITORY" ] && echo "REPOSITORY is not defined, exitting!" && exit 1
[ -z "$APP" ] && echo "APP is not defined, exitting!" && exit 1

set -e  # exit on error
# set -u  # treat refs to unassigned vars as errors

# Directories
app_dir="$(pwd)/${APP}"

platform="${PLATFORM:-amd64}"

# Every app is a tag of one repository: <app>-latest for amd64 and
# <app>-arm64 for arm64. The variant is also passed to the Dockerfile as TAG
# so it builds FROM the matching base image (e.g. ros2-${TAG}).
tag="latest"
if [ "$platform" = "arm64" ]; then
    tag=arm64
fi

# Full image target (TARGET overrides the app name, not the variant)
target="${REPOSITORY}:${TARGET:-$APP}-${tag}"

# Build command
build_cmd=(docker build --force-rm)
if [ -n "$DOCKERFILE" ]; then
    echo "Using Dockerfile ${DOCKERFILE}"
    build_cmd+=(-f "$DOCKERFILE")
fi

build_cmd+=(--platform "linux/${platform}" --build-arg "TAG=${tag}")

# Optional multi-stage build target (docker build --target)
if [ -n "$BUILD_STAGE" ]; then
    build_cmd+=(--target "$BUILD_STAGE")
fi


while [ $# -gt 0 ]; do
  case $1 in
    --git)
      GIT_COMMIT=$(git rev-parse HEAD)
      GIT_BRANCH=$(git branch --show-current)
      GIT_TAG=$(git describe --tags --always)
      build_cmd+=(--build-arg "GIT_COMMIT=$GIT_COMMIT" --build-arg "GIT_BRANCH=$GIT_BRANCH" --build-arg "GIT_TAG=$GIT_TAG")
      ;;
  esac
  shift
done


if [[ $NOCACHE -eq 1 ]]; then
    echo "Turning off docker cache!"
    build_cmd+=(--no-cache)
fi


build_cmd+=(-t "$target")


# Build
echo ">>> Building ${target}"
cd "$app_dir"
"${build_cmd[@]}" .
echo ">>> Done building ${target}"
