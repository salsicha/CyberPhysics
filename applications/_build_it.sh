#!/bin/bash

# Input args
[ -z "$NAMESPACE" ] && echo "NAMESPACE is not defined, exitting!" && exit 1
[ -z "$APP" ] && echo "APP is not defined, exitting!" && exit 1

set -e  # exit on error
# set -u  # treat refs to unassigned vars as errors

# Directories
app_dir="$(pwd)/${APP}"

tag="latest"

# Full image target
if [ ! -z "$TARGET" ]; then
    # User specified target
    target="${NAMESPACE}/${TARGET}:${tag}"
else
    if [ ${PLATFORM} == "arm64" ]; then
        tag=arm64
    fi

    target="${NAMESPACE}/${APP}:${tag}"
fi

echo $DOCKERFILE
# Build command
if [ -z "$DOCKERFILE" ]; then
    build_cmd="docker build --force-rm"
else
    build_cmd="docker build -f $DOCKERFILE --force-rm"
fi

build_cmd="${build_cmd} --platform linux/${PLATFORM:-amd64} --build-arg TAG=${tag}"


while [ $# -gt 0 ]; do
  case $1 in
    --git)
      GIT_COMMIT=$(git rev-parse HEAD)
      GIT_BRANCH=$(git branch --show-current)
      GIT_TAG=$(git describe --tag)
      build_cmd="${build_cmd} --build-arg GIT_COMMIT=$GIT_COMMIT --build-arg GIT_BRANCH=$GIT_BRANCH --build-arg GIT_TAG=$GIT_TAG"
      ;;
  esac
  shift
done


if [[ $NOCACHE -eq 1 ]]; then
    echo "Turning off docker cache!"
    build_cmd="${build_cmd} --no-cache"
fi


build_cmd="${build_cmd} -t ${target}"


# Build
echo ">>> Building ${target}"
cd $app_dir
$build_cmd .
echo ">>> Done building ${target}"
