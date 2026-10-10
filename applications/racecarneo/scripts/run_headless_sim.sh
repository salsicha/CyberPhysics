#!/bin/bash
# Run RacecarSim v2.8.3 unattended on a private Xvfb display.
#
# v2.8.3 has no command-line, PlayerPrefs or network way to start a level: it
# opens on its main menu, the Python ports (UDP 5064/5065) only exist once
# "Begin Simulation" loads a level, and a connected program then waits for the
# user to press START (Enter) to enter user-program mode. Its async UDP port
# only accepts connect, exit, camera and lidar requests, so the library's
# racecar_go packet is ignored. This script plays the user with xdotool:
#
#   1. Click "Begin Simulation" (the menu defaults: Sandbox Environments /
#      Demo World / Exploration) until the simulator binds its Python ports.
#   2. Press Enter every RACECAR_SIM_START_PERIOD_S seconds. Before a program
#      connects the level only shows a "connect a Python program" hint; once
#      the bridge has connected it enters user-program mode; after that it is
#      a no-op (LevelManager.HandleStart returns early). If the bridge
#      disconnects the level drops back to default drive and the next press
#      resumes once the bridge reconnects.
#
# -batchmode is not used: a batch-mode player has no window and reads no input.
# Rendering uses Vulkan on the NVIDIA GPU when the container has its Vulkan
# driver (NVIDIA_DRIVER_CAPABILITIES includes graphics), else Mesa's software
# OpenGL, which runs the simulator at only a few frames per second.
#
# Environment:
#   RACECAR_XVFB_SERVER_ARGS     Xvfb screen (default "-screen 0 1280x720x24";
#                                keep it 16:9, the menu click is relative)
#   RACECAR_SIM_GRAPHICS_API     auto (default) | vulkan | opengl
#   RACECAR_SIM_LOG_FILE         Unity player log (default /dev/stdout)
#   RACECAR_SIM_START_PERIOD_S   seconds between START presses (default 1)
#   RACECAR_SIM_LEVEL_TIMEOUT_S  give up if no level loads in time (default 120)
# Extra arguments are passed to the Unity player.
set -euo pipefail

RACECAR_SIM_DIR="${RACECAR_SIM_DIR:-/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_v2.8.3}"
XVFB_ARGS="${RACECAR_XVFB_SERVER_ARGS:--screen 0 1280x720x24}"
GRAPHICS_API="${RACECAR_SIM_GRAPHICS_API:-auto}"
SIM_LOG_FILE="${RACECAR_SIM_LOG_FILE:-/dev/stdout}"
START_PERIOD_S="${RACECAR_SIM_START_PERIOD_S:-1}"
LEVEL_TIMEOUT_S="${RACECAR_SIM_LEVEL_TIMEOUT_S:-120}"
# For `docker exec racecarneo_sim bash -c '. /tmp/racecarsim.env; xwd -root'`.
ENV_FILE=/tmp/racecarsim.env

# Python ports of a loaded level: 5064 (async) and 5065 (sync), in hex as
# /proc/net/udp prints them.
PYTHON_PORTS_HEX=(13C8 13C9)
# "Begin Simulation" button centre as a fraction of the 16:9 window
# (290, 331 on the 1280x720 menu).
BEGIN_X_NUM=290 BEGIN_X_DEN=1280
BEGIN_Y_NUM=331 BEGIN_Y_DEN=720

log() { echo "[run_headless_sim] $*"; }
die() { log "ERROR: $*"; exit 1; }

xvfb_pid="" sim_pid="" starter_pid="" runtime_dir=""

# shellcheck disable=SC2329  # invoked by the traps below
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${starter_pid}" ]]; then
    kill "${starter_pid}" 2>/dev/null || true
  fi
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    kill -TERM "${sim_pid}" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "${sim_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "${sim_pid}" 2>/dev/null || true
  fi
  if [[ -n "${xvfb_pid}" ]]; then
    kill "${xvfb_pid}" 2>/dev/null || true
  fi
  rm -rf "${runtime_dir}" "${ENV_FILE}"
}
trap cleanup EXIT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 130' INT

# Inodes of the UDP sockets bound to local port $1 (hex) in this network
# namespace (the host's, under network_mode: host).
udp_port_inodes() {
  awk -v port=":$1" 'FNR > 1 && substr($2, length($2) - 4) == port { print $10 }' \
    /proc/net/udp /proc/net/udp6 2>/dev/null
}

# True once the simulator process itself has bound a Python port, i.e. a
# level's PythonInterface exists.
sim_has_python_port() {
  local port inode fds
  fds="$(ls -l "/proc/${sim_pid}/fd" 2>/dev/null)" || return 1
  for port in "${PYTHON_PORTS_HEX[@]}"; do
    for inode in $(udp_port_inodes "${port}"); do
      if grep -qF "socket:[${inode}]" <<<"${fds}"; then
        return 0
      fi
    done
  done
  return 1
}

# First display >= 99 with neither a lock file nor a listening socket. The
# abstract X sockets live in the network namespace, which is the host's under
# network_mode: host, so the host's own displays are visible here.
pick_display() {
  local n
  for n in $(seq 99 199); do
    [[ -e "/tmp/.X${n}-lock" ]] && continue
    grep -q "/tmp/\.X11-unix/X${n}\$" /proc/net/unix 2>/dev/null && continue
    echo "${n}"
    return 0
  done
  return 1
}

sim_window() {
  xdotool search --onlyvisible --name '^RacecarSim$' 2>/dev/null | head -n 1 || true
}

click_begin_simulation() {
  local wid geometry WIDTH="" HEIGHT=""
  wid="$(sim_window)"
  [[ -n "${wid}" ]] || return 0
  geometry="$(xdotool getwindowgeometry --shell "${wid}" 2>/dev/null)" || return 0
  WIDTH="$(sed -n 's/^WIDTH=\([0-9]*\)$/\1/p' <<<"${geometry}")"
  HEIGHT="$(sed -n 's/^HEIGHT=\([0-9]*\)$/\1/p' <<<"${geometry}")"
  [[ -n "${WIDTH}" && -n "${HEIGHT}" ]] || return 0
  # Unity only registers a click after the pointer moved onto the button.
  xdotool mousemove --window "${wid}" 1 1 \
    mousemove --window "${wid}" \
      $((WIDTH * BEGIN_X_NUM / BEGIN_X_DEN)) $((HEIGHT * BEGIN_Y_NUM / BEGIN_Y_DEN)) \
    click 1 2>/dev/null || true
}

press_start() {
  local wid
  wid="$(sim_window)"
  [[ -n "${wid}" ]] || return 0
  xdotool windowfocus "${wid}" 2>/dev/null || true
  xdotool key Return 2>/dev/null || true
}

# Runs in the background for the simulator's lifetime; a failed xdotool call
# (no window yet, window closing) must not end it.
autostart() {
  set +e
  local started now
  started="$(date +%s)"
  log "waiting for the main menu; clicking \"Begin Simulation\""
  until sim_has_python_port; do
    now="$(date +%s)"
    if (( now - started > LEVEL_TIMEOUT_S )); then
      log "ERROR: no level loaded after ${LEVEL_TIMEOUT_S} s; stopping the simulator"
      kill -TERM "${sim_pid}" 2>/dev/null || true
      return 1
    fi
    click_begin_simulation
    sleep 1
  done
  log "level loaded after $(( $(date +%s) - started )) s; pressing START every" \
    "${START_PERIOD_S} s to enter user-program mode once the bridge connects"
  while kill -0 "${sim_pid}" 2>/dev/null; do
    press_start
    sleep "${START_PERIOD_S}"
  done
}

for port in "${PYTHON_PORTS_HEX[@]}"; do
  if [[ -n "$(udp_port_inodes "${port}")" ]]; then
    die "UDP port $((16#${port})) is already in use; is another RacecarSim running on this host?"
  fi
done

case "${GRAPHICS_API}" in
  auto)
    if [[ -e /etc/vulkan/icd.d/nvidia_icd.json || -e /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
      GRAPHICS_API=vulkan
    else
      GRAPHICS_API=opengl
    fi
    ;;
  vulkan | opengl) ;;
  *) die "RACECAR_SIM_GRAPHICS_API must be auto, vulkan or opengl, not '${GRAPHICS_API}'" ;;
esac
if [[ "${GRAPHICS_API}" == vulkan ]]; then
  graphics_args=(-force-vulkan)
else
  graphics_args=(-force-glcore)
  log "WARNING: no NVIDIA Vulkan driver in the container; rendering with software" \
    "OpenGL (llvmpipe), expect only a few simulator frames per second"
fi

runtime_dir="$(mktemp -d -t racecarsim.XXXXXX)"
display_num="$(pick_display)" || die "no free X display number in 99..199"
export DISPLAY=":${display_num}"
export XAUTHORITY="${runtime_dir}/Xauthority"
export XDG_RUNTIME_DIR="${runtime_dir}"
xauth -q -f "${XAUTHORITY}" add "${DISPLAY}" . "$(mcookie)"
# shellcheck disable=SC2086  # XVFB_ARGS is a list of server arguments
Xvfb "${DISPLAY}" -nolisten tcp -auth "${XAUTHORITY}" ${XVFB_ARGS} &
xvfb_pid=$!
for _ in $(seq 1 100); do
  xdotool getdisplaygeometry >/dev/null 2>&1 && break
  kill -0 "${xvfb_pid}" 2>/dev/null || die "Xvfb ${DISPLAY} exited"
  sleep 0.1
done
read -r screen_w screen_h < <(xdotool getdisplaygeometry) || die "Xvfb ${DISPLAY} did not start"
printf 'export DISPLAY=%q XAUTHORITY=%q\n' "${DISPLAY}" "${XAUTHORITY}" > "${ENV_FILE}"
log "Xvfb ${DISPLAY} ${screen_w}x${screen_h}, ${GRAPHICS_API} rendering"

cd "${RACECAR_SIM_DIR}"
# The Unity player is named after its build directory (RacecarSim_Linux_<version>).
# The screen size overrides the window size Unity keeps in PlayerPrefs.
"./$(basename "${RACECAR_SIM_DIR}").x86_64" \
  -logFile "${SIM_LOG_FILE}" \
  -screen-width "${screen_w}" -screen-height "${screen_h}" \
  "${graphics_args[@]}" \
  "$@" &
sim_pid=$!

autostart &
starter_pid=$!

status=0
wait "${sim_pid}" || status=$?
sim_pid=""
log "RacecarSim exited with status ${status}"
exit "${status}"
