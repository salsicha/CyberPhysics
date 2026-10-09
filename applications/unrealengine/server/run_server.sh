#!/bin/bash
# Run a packaged Unreal Engine Linux build headless, rendering off-screen on
# the GPU. The build is the user's own project (packaged for Linux with a ROS
# plugin) and is bind-mounted at run time; see ../README.md.
#
# Environment (all optional):
#   UE_BUILD_PATH        build folder inside the container (default /server/build)
#   UE_PROJECT_SCRIPT    project launch script, absolute or relative to
#                        UE_BUILD_PATH (default: the single <Project>.sh that has
#                        a <Project>/Binaries/Linux folder next to it)
#   UE_MAP               map to open, e.g. /Game/Maps/Warehouse
#   UE_RES_X, UE_RES_Y   render resolution (default 1280x720)
#   UE_PORT              UE network (game) port, -port=; unset = UE default
#   UE_GRAPHICS_ADAPTER  GPU index for -graphicsadapter=
#   UE_RENDER_OFFSCREEN  1 (default): -RenderOffScreen; 0: on-screen window
#                        (needs DISPLAY and the X11 socket)
#   UE_NOSOUND           1 (default): -nosound
#   UE_EXTRA_ARGS        extra UE arguments, split on whitespace
# Arguments given to this script are appended to the UE command line.
set -euo pipefail

UE_BUILD_PATH="${UE_BUILD_PATH:-/server/build}"
UE_USER="${UE_USER:-unrealuser}"

log() { echo "[ue_server] $*"; }
die() {
    echo "[ue_server] ERROR: $*" >&2
    exit 1
}

# Unreal Engine refuses to run as root: drop to the image's non-root user
# (e.g. under `docker run --user root` or a Kubernetes runAsUser: 0)
if [ "$(id -u)" -eq 0 ]; then
    getent passwd "$UE_USER" > /dev/null || die "running as root and there is no '${UE_USER}' user to run Unreal Engine as"
    log "running as root: switching to ${UE_USER}"
    HOME="$(getent passwd "$UE_USER" | cut -d: -f6)"
    export HOME USER="$UE_USER" LOGNAME="$UE_USER"
    exec setpriv --reuid="$UE_USER" --regid="$UE_USER" --init-groups "$(readlink -f "$0")" "$@"
fi

no_build_help="Package your Unreal Engine project for Linux (with the rclUE or ROSIntegration
plugin) and copy the packaged folder - the one containing <Project>.sh - into
applications/unrealengine/server/build/, or point UE_BUILD_DIR at it on the host:
    UE_BUILD_DIR=/abs/path/to/Linux ./start.sh unrealengine
With docker run, mount it yourself: -v /abs/path/to/Linux:${UE_BUILD_PATH}
See applications/unrealengine/README.md."

if [ ! -d "$UE_BUILD_PATH" ]; then
    die "no Unreal Engine build mounted: ${UE_BUILD_PATH} does not exist.
${no_build_help}"
fi
if [ ! -r "$UE_BUILD_PATH" ] || [ ! -x "$UE_BUILD_PATH" ]; then
    die "${UE_BUILD_PATH} is not readable by $(id -un) (uid $(id -u)); make the build folder readable by that uid on the host"
fi
if [ -z "$(find -L "$UE_BUILD_PATH" -mindepth 1 -maxdepth 1 ! -name .gitkeep -print -quit)" ]; then
    die "no Unreal Engine build mounted: ${UE_BUILD_PATH} is empty.
${no_build_help}"
fi

# A packaged UE project is <Project>.sh next to <Project>/Binaries/Linux/
# (inside a Linux/ or LinuxNoEditor/ folder if the whole output was copied)
if [ -n "${UE_PROJECT_SCRIPT:-}" ]; then
    script="$UE_PROJECT_SCRIPT"
    [ "${script#/}" = "$script" ] && script="${UE_BUILD_PATH}/${script}"
    [ -f "$script" ] || die "UE_PROJECT_SCRIPT=${UE_PROJECT_SCRIPT}: ${script} does not exist"
else
    candidates=()
    while IFS= read -r -d '' sh; do
        if [ -d "$(dirname "$sh")/$(basename "$sh" .sh)/Binaries/Linux" ]; then
            candidates+=("$sh")
        fi
    done < <(find -L "$UE_BUILD_PATH" -mindepth 1 -maxdepth 3 -type f -name '*.sh' -print0 | sort -z)
    if [ "${#candidates[@]}" -eq 0 ]; then
        die "no packaged UE project found in ${UE_BUILD_PATH}: expected <Project>.sh next to a <Project>/Binaries/Linux/ folder (up to two levels down). Found:
$(find -L "$UE_BUILD_PATH" -mindepth 1 -maxdepth 2 | head -n 20)
Set UE_PROJECT_SCRIPT to the project's launch script if the layout differs."
    fi
    if [ "${#candidates[@]}" -gt 1 ]; then
        die "several packaged UE projects in ${UE_BUILD_PATH}; set UE_PROJECT_SCRIPT to one of:
$(printf '    %s\n' "${candidates[@]#"${UE_BUILD_PATH}"/}")"
    fi
    script="${candidates[0]}"
fi
script="$(readlink -f "$script")"

args=()
if [ -n "${UE_MAP:-}" ]; then
    args+=("$UE_MAP")
fi
if [ "${UE_RENDER_OFFSCREEN:-1}" = 1 ]; then
    args+=(-RenderOffScreen)
else
    args+=(-windowed)
fi
if [ "${UE_NOSOUND:-1}" = 1 ]; then
    args+=(-nosound)
fi
args+=("-ResX=${UE_RES_X:-1280}" "-ResY=${UE_RES_Y:-720}")
if [ -n "${UE_PORT:-}" ]; then
    args+=("-port=${UE_PORT}")
fi
if [ -n "${UE_GRAPHICS_ADAPTER:-}" ]; then
    args+=("-graphicsadapter=${UE_GRAPHICS_ADAPTER}")
fi
# No dialogs, and the full log on stdout for `docker logs`
args+=(-unattended -stdout -FullStdOutLogOutput)
read -r -a extra_args <<< "${UE_EXTRA_ARGS:-}"
args+=("${extra_args[@]}" "$@")

# GPU check: UE renders through Vulkan, which needs the NVIDIA driver's
# graphics libraries in the container (NVIDIA_DRIVER_CAPABILITIES graphics)
if command -v nvidia-smi > /dev/null && gpu="$(nvidia-smi -L 2> /dev/null)"; then
    log "GPU: ${gpu%%$'\n'*}"
    if ! ldconfig -p 2> /dev/null | grep 'libGLX_nvidia\.so\.0' > /dev/null; then
        log "WARNING: libGLX_nvidia.so.0 is not in the container, so Vulkan has no NVIDIA driver; include 'graphics' in NVIDIA_DRIVER_CAPABILITIES."
    fi
else
    log "WARNING: no NVIDIA GPU visible (nvidia-smi is missing or failed). Start the server with a GPU: the compose GPU reservation (./start.sh unrealengine) or docker run --gpus all, with the NVIDIA Container Toolkit installed on the host."
fi

# The image's ROS 2 environment must not leak into UE: rclUE ships its own
# ROS 2 libraries, and /opt/ros on LD_LIBRARY_PATH would override them.
# ROS_DOMAIN_ID, RMW_IMPLEMENTATION and DDS settings are kept.
ld_path=()
IFS=: read -r -a ld_parts <<< "${LD_LIBRARY_PATH:-}"
for p in "${ld_parts[@]}"; do
    case "$p" in
        "" | /opt/ros/*) ;;
        *) ld_path+=("$p") ;;
    esac
done
if [ "${#ld_path[@]}" -gt 0 ]; then
    LD_LIBRARY_PATH="$(IFS=:; echo "${ld_path[*]}")"
    export LD_LIBRARY_PATH
else
    unset LD_LIBRARY_PATH
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH

if [ ! -w "$(dirname "$script")" ]; then
    log "WARNING: $(dirname "$script") is not writable by uid $(id -u); UE writes its logs and config under <Project>/Saved there"
fi

cmd=("$script")
# Builds unpacked from an archive can lose the executable bit
[ -x "$script" ] || cmd=(/bin/sh "$script")

log "user: $(id -un) (uid $(id -u)), ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
log "starting: ${cmd[*]} ${args[*]}"
cd "$(dirname "$script")"

# UE's generated <Project>.sh does not exec the engine binary, so run it in
# its own process group and pass a stop request on to the whole group as
# SIGTERM (background jobs ignore SIGINT): `docker stop` and Ctrl-C then shut
# the engine down cleanly.
ue_pid=
stop_requested=
stop_engine() {
    # The group, or the process itself if it has not called setsid yet
    kill -s TERM -- "-${ue_pid}" 2> /dev/null || kill -s TERM "$ue_pid" 2> /dev/null || true
}
forward() {
    log "stopping Unreal Engine (SIG$1)"
    stop_requested=1
    if [ -n "$ue_pid" ]; then
        stop_engine
    fi
}
trap 'forward TERM' TERM
trap 'forward INT' INT
setsid "${cmd[@]}" "${args[@]}" &
ue_pid=$!
# A stop that arrived while the engine was being started
if [ -n "$stop_requested" ]; then
    stop_engine
fi
status=0
wait "$ue_pid" || status=$?
# A trapped signal interrupts wait; keep waiting for the engine to exit
while kill -0 "$ue_pid" 2> /dev/null; do
    status=0
    wait "$ue_pid" || status=$?
done
log "Unreal Engine exited with status ${status}"
exit "$status"
