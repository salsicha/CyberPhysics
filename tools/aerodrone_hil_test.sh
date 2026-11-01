#!/usr/bin/env bash
# Aerodrone hardware-in-the-loop test (tools/aerodrone_hil_test.md): the
# companion stack on the BlueOS Raspberry Pi 5, as on the aircraft
# (compositions/aerodrone_hardware.yaml + tools/aerodrone_hil_pi.yaml,
# brought up over SSH), flying the Gazebo simulation and ArduPilot SITL on
# this workstation (compositions/aerostack2_sim.yaml +
# tools/aerodrone_hil_sim.yaml + tools/tracking_sim_test.yaml +
# tools/aerodrone_hil_test.yaml) over Wi-Fi. Runs one scenario, samples the
# Pi's resources and the topics crossing the link, collects both sides' logs,
# tears both sides down (also on failure) and scores the run:
#
#   mission     the wilderness mission end to end (mission tree, flight_ready
#               and Aerostack2 on the Pi), recorded; scored by
#               tools/aerodrone_hil_mission_score.py; the navigation
#               validator runs alongside
#   gps_denied  the same with GPS 1 denied during the cruise
#               (tools/gps_denied_sim_test.sh's denial profile), scored by
#               tools/gps_denied_sim_scorer.py
#   tracking    tools/tracking_sim_test.sh's tracking scenario with the
#               tracker on the Pi; the page is loaded from the Pi in a
#               headless browser here (tools/aerodrone_hil_page_check.py)
#   fire        its launcher fire scenario (TRACKING_TEST_SCENARIO=fire)
#
# Usage: tools/aerodrone_hil_test.sh <mission|gps_denied|tracking|fire>
#        tools/aerodrone_hil_test.sh control <mission|gps_denied|tracking|fire>
#   control: the same scenario all on this workstation, with the HIL
#   middleware (tools/aerodrone_hil_control.yaml: loopback Zenoh peers of a
#   local router, ROS_DOMAIN_ID and Zenoh settings of aerodrone_hil.env),
#   through tools/gps_denied_sim_test.sh or tools/tracking_sim_test.sh
#   (their variables apply), or for the mission the Gazebo stack with the
#   recorder and the validator, scored as a HIL mission; output
#   generated/aerodrone_hil/control_<scenario>.
# Hold the host's simulator lock around it if the host is shared.
# Environment:
#   HIL_PI             ssh destination of the Pi (default pi@<HIL_PI_HOST>,
#                      applications/aerostack2/config/aerodrone_hil.env)
#   HIL_PI_REPO        the repository copy on the Pi (default /home/pi/cyberphysics)
#   HIL_SYNC           true (default): rsync the files the Pi's stack mounts
#                      (compositions, systems, tools, applications/aerostack2's
#                      config, scripts and web page) to the Pi's copy first.
#                      The Pi's images are built there (tools/aerodrone_hil_test.md).
#   HIL_OUTPUT         run directory (default generated/aerodrone_hil/<scenario>)
#   HIL_TIMEOUT_S      overall wall-clock limit (default 2400)
#   HIL_PROBE_S        topic probe window on each host (default 60)
#   HIL_PROBE_DELAY_S  seconds after the flight starts (mission, tracking,
#                      fire) to probe (default 240, 150, 120); gps_denied
#                      probes when GPS 1 is denied
#   HIL_PAGE_S         page check window, tracking (default 60)
#   HIL_IDLE_S         the Pi's idle baseline before anything starts (default 60)
#   HIL_KEEP_VOLUMES   true keeps the workstation's DEM/tile caches and
#                      terrain assets for the next run (default false)
#   PLAYWRIGHT_VERSION Playwright image and package (default 1.63.0)
#   and the GPS-denied test's GPS_DENIAL_* and the tracking test's
#   TRACKING_TEST_* variables.
# Writes, in the run directory: stack.log (workstation), pi/ (pi_stack.log,
# resources.jsonl, probe_pi.json, the mission tree's log), sitl/ (ArduPilot
# SITL's dataflash log, gzipped, and console), probe_sim.json,
# clock.json, sockets_*.txt, the scenario's recording and score, and
# hil_summary.json.
# Exit status: 0 pass, 1 a check failed, 2 no usable run, 3 (mission,
# gps_denied) the score passed but the navigation validation did not.
set -euo pipefail
cd "$(dirname "$0")/.."

scenario="${1:-}"
hil_env=applications/aerostack2/config/aerodrone_hil.env
# ArduPilot SITL's dataflash log (EKF, IMU, GPS, baro and the simulator's
# truth, SIM2) and console, from the running container to <run>/sitl before
# teardown: what an autopilot anomaly is diagnosed from.
keep_sitl_logs() {  # <container id> <run directory>
    [ -n "$1" ] || return 0
    rm -rf "${2:?}/sitl"
    mkdir -p "$2/sitl"
    docker cp "$1:/ardupilot/logs" "$2/sitl/logs" > /dev/null 2>&1
    docker cp "$1:/tmp/ArduCopter.log" "$2/sitl/ArduCopter.log" > /dev/null 2>&1
    gzip -f "$2"/sitl/logs/*.BIN 2> /dev/null
    return 0
}
env_value() { sed -n "s/^$1=//p" "$hil_env" | tail -1; }
usage="usage: $0 <mission|gps_denied|tracking|fire> | control <mission|gps_denied|tracking|fire>"
if [ "$scenario" = control ]; then
    control="${2:-}"
    ROS_DOMAIN_ID="$(env_value ROS_DOMAIN_ID)"
    HIL_ZENOH_PORT="$(env_value HIL_ZENOH_PORT)"
    ZENOH_ROUTER_CHECK_ATTEMPTS="$(env_value ZENOH_ROUTER_CHECK_ATTEMPTS)"
    ZENOH_CONFIG_OVERRIDE="$(env_value ZENOH_CONFIG_OVERRIDE)"
    ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE//\$\{HIL_ZENOH_PORT\}/$HIL_ZENOH_PORT}"
    export ROS_DOMAIN_ID HIL_ZENOH_PORT ZENOH_ROUTER_CHECK_ATTEMPTS ZENOH_CONFIG_OVERRIDE
    output="${HIL_OUTPUT:-generated/aerodrone_hil/control_$control}"
    case "$control" in
        gps_denied)
            GPS_DENIED_COMPOSE_OVERRIDE=tools/aerodrone_hil_control.yaml \
            GPS_DENIED_PROJECT=hil_control_gps_denied GPS_DENIED_OUTPUT="$output" \
                exec tools/gps_denied_sim_test.sh gazebo ;;
        mission)
            mkdir -p "$output"
            GPS_DENIED_OUTPUT="$(cd "$output" && pwd)"
            NAVSIM_REQUIRE_VALID="${NAVSIM_REQUIRE_VALID-demnav wildnav}"
            NAVSIM_REQUIRE_CORRECTION="${NAVSIM_REQUIRE_CORRECTION:-true}"
            export GPS_DENIED_OUTPUT NAVSIM_REQUIRE_VALID NAVSIM_REQUIRE_CORRECTION
            for name in recording.jsonl mission_score.json stack.log tree.log validation.log up.log; do
                rm -f "${GPS_DENIED_OUTPUT:?}/$name"
            done
            CONTROL=(docker compose -p hil_control_mission -f compositions/aerostack2_sim.yaml
                     -f tools/aerodrone_hil_control.yaml --profile validation)
            control_down() {
                keep_sitl_logs "$("${CONTROL[@]}" ps -q navigator_sitl 2> /dev/null)" \
                    "$GPS_DENIED_OUTPUT"
                "${CONTROL[@]}" --profile denial logs --no-color --timestamps \
                    > "$GPS_DENIED_OUTPUT/stack.log" 2>&1
                "${CONTROL[@]}" --profile denial down -v --remove-orphans > /dev/null 2>&1
            }
            trap control_down EXIT
            deadline=$(( $(date +%s) + ${HIL_TIMEOUT_S:-2400} ))
            # The recorder (and the simulator it depends on) first, then the
            # rest: up -d returns once flight_ready has passed (the mission
            # tree waits for it), and the recorder has seen the vehicle at rest.
            timeout "${HIL_TIMEOUT_S:-2400}" "${CONTROL[@]}" up -d gps_denial_recorder \
                > "$GPS_DENIED_OUTPUT/up.log" 2>&1 || { tail -5 "$GPS_DENIED_OUTPUT/up.log"; exit 2; }
            timeout "${HIL_TIMEOUT_S:-2400}" "${CONTROL[@]}" up -d >> "$GPS_DENIED_OUTPUT/up.log" 2>&1 ||
                { tail -5 "$GPS_DENIED_OUTPUT/up.log"; exit 2; }
            tree="$("${CONTROL[@]}" ps -a -q wilderness_mission)"
            while [ "$(docker inspect -f '{{.State.Status}}' "$tree")" != exited ] &&
                  [ "$(date +%s)" -lt "$deadline" ]; do
                sleep 15
            done
            docker logs "$tree" > "$GPS_DENIED_OUTPUT/tree.log" 2>&1
            tree_exit="$(docker inspect -f '{{.State.ExitCode}}' "$tree")"
            [ "$(docker inspect -f '{{.State.Status}}' "$tree")" = exited ] || tree_exit=255
            "${CONTROL[@]}" logs --no-color navsim_validator > "$GPS_DENIED_OUTPUT/validation.log" 2>&1
            control_down
            trap - EXIT
            status=0
            python3 tools/aerodrone_hil_mission_score.py "$GPS_DENIED_OUTPUT" \
                --tree-log "$GPS_DENIED_OUTPUT/tree.log" --tree-exit "$tree_exit" \
                > "$GPS_DENIED_OUTPUT/mission_score.json" || status=$?
            cat "$GPS_DENIED_OUTPUT/mission_score.json"
            exit "$status" ;;
        tracking|fire)
            TRACKING_TEST_COMPOSE_OVERRIDE=tools/aerodrone_hil_control.yaml \
            TRACKING_TEST_PROJECT="hil_control_$control" TRACKING_TEST_OUTPUT="$output" \
            TRACKING_TEST_SCENARIO="$([ "$control" = fire ] && echo fire || echo tracking)" \
                exec tools/tracking_sim_test.sh gazebo ;;
        *) echo "$usage" >&2; exit 2 ;;
    esac
fi
case "$scenario" in
    mission|gps_denied|tracking|fire) ;;
    *) echo "$usage" >&2; exit 2 ;;
esac
pi_host="$(env_value HIL_PI_HOST)"
sim_host="$(env_value HIL_SIM_HOST)"
port="$(env_value HIL_ZENOH_PORT)"
pi="${HIL_PI:-pi@$pi_host}"
pi_repo="${HIL_PI_REPO:-/home/pi/cyberphysics}"
timeout_s="${HIL_TIMEOUT_S:-2400}"
probe_s="${HIL_PROBE_S:-60}"
out="${HIL_OUTPUT:-generated/aerodrone_hil/$scenario}"
mkdir -p "$out/pi"
out="$(cd "$out" && pwd)"
pi_run="generated/aerodrone_hil/run_$scenario"
playwright_version="${PLAYWRIGHT_VERSION:-1.63.0}"
pi_image=salsicha/cyberphysics:aerostack2-arm64
sim_image=salsicha/cyberphysics:aerostack2-latest

env_files=(--env-file systems/aerodrone/config/topics.env
           --env-file applications/aerostack2/config/mount_tamalpais.env
           --env-file "$hil_env")
SIM=(docker compose -p hil_sim "${env_files[@]}"
     -f compositions/aerostack2_sim.yaml -f tools/aerodrone_hil_sim.yaml
     -f tools/tracking_sim_test.yaml -f tools/aerodrone_hil_test.yaml)
pi_compose="HIL_RUN_DIR=../$pi_run docker compose -p hil_pi ${env_files[*]} \
-f compositions/aerodrone_hardware.yaml -f tools/aerodrone_hil_pi.yaml"
# The Pi's bash warns about this host's locale: not forwarded.
SSH=(env -u LC_ALL -u LANG ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 "$pi")
pi_sh() { "${SSH[@]}" "cd '$pi_repo' && $*"; }

case "$scenario" in
    mission) sim_profiles=(--profile validation); pi_profiles="--profile mission"
             probe_delay="${HIL_PROBE_DELAY_S:-240}" ;;
    gps_denied) sim_profiles=(--profile validation --profile denial)
                pi_profiles="--profile mission"; probe_delay="" ;;
    tracking) sim_profiles=(--profile tracking); pi_profiles="--profile tracking"
              probe_delay="${HIL_PROBE_DELAY_S:-150}" ;;
    fire) sim_profiles=(--profile tracking); pi_profiles="--profile tracking"
          probe_delay="${HIL_PROBE_DELAY_S:-120}" ;;
esac
# Topics crossing to the Pi (from the OAK-1 stand-in, the detector and the
# launcher stand-in) and back (what the harness here consumes): probed only
# where a node takes them anyway.
pi_topics="/oak1/relative_depth /oak1/camera_info /oak1/image_highres \
/oak1/image_highres/camera_info /aerodrone/rangefinder"
case "$scenario" in
    mission|gps_denied)
        sim_topics="/navigation/fix /mavros/global_position/global /navigation/correction_fix \
/demnav/fix /wildnav/fix /mavros/state /mavros/setpoint_position/local" ;;
    tracking|fire)
        pi_topics="$pi_topics /tracking/detections /tracking/camera_info"
        [ "$scenario" = tracking ] && pi_topics="$pi_topics /tracking/image/compressed"
        [ "$scenario" = fire ] && pi_topics="$pi_topics /aerodrone/launcher/joint_states"
        sim_topics="/navigation/odometry /tracking/status /tracking/target /demnav/valid" ;;
esac

export GPS_DENIED_OUTPUT="$out" TRACKING_TEST_OUTPUT="$out" HIL_RUN_DIR="$out"
export NAVSIM_REQUIRE_VALID="${NAVSIM_REQUIRE_VALID-demnav wildnav}"
export NAVSIM_REQUIRE_CORRECTION="${NAVSIM_REQUIRE_CORRECTION:-true}"
# The tracking test driver's settings, as tools/tracking_sim_test.sh sets
# them for Gazebo (applications/aerostack2/config/tracking_gazebo.env).
export SIM_AUTO_FIRE=false TRACKING_TEST_STACK=gazebo TRACKING_TEST_DETECTOR=real
export TRACKING_TEST_MOUNT_YAW_DEG=0.0 TRACKING_TEST_CAMERA_STABILIZED=false
TRACKING_TEST_SCENARIO=tracking
[ "$scenario" != fire ] || TRACKING_TEST_SCENARIO=fire
export TRACKING_TEST_SCENARIO

log() { echo "[$(date +%H:%M:%S)] $*"; }
now() { date +%s.%N; }
for name in recording.jsonl recording.jsonl.gz denial_events.jsonl score.json mission_score.json \
            validation.log stack.log stack.log.gz up.log cpu.jsonl probe_sim.json page_check.json \
            clock_before.json clock_after.json pi_up.log probe_sim.log sockets_sim.txt \
            sockets_pi.txt hil_summary.json; do
    rm -f "${out:?}/$name"
done
rm -rf "${out:?}/pi" "${out:?}/page" "${out:?}/sitl"
mkdir -p "$out/pi" "$out/page"

# Only the outcome of the runs in progress (no hil_ project may be up).
leftovers() {
    { docker ps -a --format '{{.Names}}' | grep -a '^hil_' || true
      "${SSH[@]}" "docker ps -a --format '{{.Names}}' | grep -a '^hil_' || true"; } 2>/dev/null
}
if [ -n "$(leftovers)" ]; then
    echo "HIL ERROR: hil_ containers are already running: $(leftovers | tr '\n' ' ')" >&2
    exit 2
fi
for image in "$pi_image" salsicha/cyberphysics:demnav-arm64 salsicha/cyberphysics:wildnav-arm64; do
    "${SSH[@]}" "docker image inspect $image > /dev/null" ||
        { echo "HIL ERROR: no $image on the Pi (tools/aerodrone_hil_test.md, Images)" >&2; exit 2; }
done
for image in "$sim_image" salsicha/cyberphysics:demnav-latest salsicha/cyberphysics:wildnav-latest \
             salsicha/cyberphysics:oak1-latest salsicha/cyberphysics:navigator-latest; do
    docker image inspect "$image" > /dev/null ||
        { echo "HIL ERROR: no $image here" >&2; exit 2; }
done

if [ "${HIL_SYNC:-true}" = true ]; then
    log "Syncing the Pi's copy ($pi:$pi_repo)"
    for dir in compositions systems tools applications/aerostack2/config \
               applications/aerostack2/scripts applications/aerostack2/src/as2_platform_blueos/web; do
        rsync -a --delete --exclude __pycache__ -e "${SSH[*]:0:${#SSH[@]}-1}" \
            "$dir/" "$pi:$pi_repo/$dir/"
    done
fi

# The Pi's run directory (root-owned files from earlier containers: removed
# in a container). generated/aerodrone_hil is made first, as the SSH user:
# a bind mount of a missing directory would create it owned by root.
pi_sh "mkdir -p generated/aerodrone_hil && docker run --rm -v '$pi_repo/generated:/g' \
--entrypoint rm $pi_image -rf /g/aerodrone_hil/run_$scenario && mkdir -p '$pi_run'"

# The Pi's clock minus this host's: the smallest-round-trip exchange of 40
# over one SSH session (the Pi and this host both keep NTP time).
clock_offset() {
    python3 - "${SSH[@]}" <<'PY'
import json, subprocess, sys, time
proc = subprocess.Popen(sys.argv[1:] + ['python3 -u -c "import sys, time\nfor _ in sys.stdin: print(repr(time.time()), flush=True)"'],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
best = None
for _ in range(40):
    t0 = time.time()
    proc.stdin.write('x\n'); proc.stdin.flush()
    remote = float(proc.stdout.readline())
    t1 = time.time()
    if best is None or t1 - t0 < best[0]:
        best = (t1 - t0, remote - (t0 + t1) / 2)
    time.sleep(0.05)
proc.stdin.close(); proc.wait(timeout=10)
print(json.dumps({'t': round(time.time(), 3), 'pi_minus_sim_s': round(best[1], 4),
                  'rtt_s': round(best[0], 4)}))
PY
}
clock_offset > "$out/clock_before.json"
offset="$(python3 -c "import json; print(json.load(open('$out/clock_before.json'))['pi_minus_sim_s'])")"
log "Pi clock minus this host's: $offset s ($(cat "$out/clock_before.json"))"

# Teardown takes every profile: a service started by name (the mission's
# recorder) is down only if its profile is in the model.
all_sim_profiles=(--profile validation --profile denial --profile tracking --profile tracking_truth
                  --profile hil_probe)
all_pi_profiles="--profile mission --profile tracking --profile launcher --profile hil_probe"
down_args=(--remove-orphans)
[ "${HIL_KEEP_VOLUMES:-false}" = true ] || down_args+=(-v)
bg_pids=()
cpu_pid=""
torn_down=false
teardown() {
    "$torn_down" && return 0
    torn_down=true
    set +e
    for pid in "${bg_pids[@]}" ${cpu_pid:+"$cpu_pid"}; do kill "$pid" 2> /dev/null; done
    log "Tearing down both sides"
    pi_sh "$pi_compose $all_pi_profiles logs --no-color --timestamps > '$pi_run/pi_stack.log' 2>&1; \
$pi_compose $all_pi_profiles down -v --remove-orphans > /dev/null 2>&1; \
[ -f '$pi_run/sampler.pid' ] && kill \$(cat '$pi_run/sampler.pid'); true"
    "${SSH[@]}" "tar -C '$pi_repo/$pi_run' -cf - ." | tar -C "$out/pi" -xf -
    keep_sitl_logs "$("${SIM[@]}" ps -q navigator_sitl 2> /dev/null)" "$out"
    "${SIM[@]}" "${all_sim_profiles[@]}" logs --no-color --timestamps > "$out/stack.log" 2>&1
    "${SIM[@]}" "${all_sim_profiles[@]}" down "${down_args[@]}" > /dev/null 2>&1
    left="$(leftovers | tr '\n' ' ')"
    [ -z "$left" ] || log "HIL WARNING: containers left: $left"
}
trap teardown EXIT

# The Pi's resources from before the stack starts (BlueOS and its ArduPilot
# alone) to the end.
# (Grouped so that only the sampler goes to the background; no stdin, or the
# SSH session would wait for it.)
pi_sh "{ nohup python3 tools/aerodrone_hil_resources.py --output '$pi_run/resources.jsonl' \
--period-s 5 --tcp-peer $sim_host:$port $sim_host:5760 \
< /dev/null > '$pi_run/resources.log' 2>&1 & echo \$! > '$pi_run/sampler.pid'; }"
t_start="$(now)"
deadline=$(( $(date +%s) + timeout_s ))
# BlueOS and its ArduPilot alone first: the idle baseline.
log "Sampling the Pi idle for ${HIL_IDLE_S:-60} s"
sleep "${HIL_IDLE_S:-60}"
t_idle_end="$(now)"
log "HIL $scenario: workstation $sim_host (hil_sim), Pi $pi (hil_pi), output $out"

log "Workstation side up (seeds the DEM, tiles and terrain on a first run)"
sim_up=("${SIM[@]}" "${sim_profiles[@]}" up -d)
if ! timeout "$timeout_s" "${sim_up[@]}" > "$out/up.log" 2>&1; then
    tail -20 "$out/up.log"
    echo "HIL ERROR: the workstation side did not come up (see up.log)"
    exit 2
fi
if [ "$scenario" = mission ]; then
    "${SIM[@]}" "${sim_profiles[@]}" up -d gps_denial_recorder >> "$out/up.log" 2>&1
fi
t_sim_up="$(now)"

log "Copying the mission tree and the DEM and tile caches to the Pi"
pi_sh "docker run --rm -v '$pi_repo/generated:/g' --entrypoint rm $pi_image -rf \
/g/aerodrone_hil/pi && mkdir -p generated/aerodrone_hil/pi/assets \
generated/aerodrone_hil/pi/demnav_cache generated/aerodrone_hil/pi/wildnav_cache"
for pair in navsim_gazebo_assets:assets demnav_gazebo_cache:demnav_cache \
            wildnav_gazebo_cache:wildnav_cache; do
    volume="hil_sim_${pair%%:*}" dest="generated/aerodrone_hil/pi/${pair#*:}"
    members=(.)
    [ "${pair#*:}" = assets ] && members=(./wilderness_mission.xml ./tracking_mission.xml)
    docker run --rm -v "$volume:/v:ro" --entrypoint tar "$sim_image" -C /v -cf - "${members[@]}" |
        "${SSH[@]}" "tar -C '$pi_repo/$dest' -xf -"
done

log "Pi side up ($pi_profiles)"
pi_up="$pi_compose $pi_profiles up -d"
if ! timeout "$(( deadline - $(date +%s) ))" "${SSH[@]}" "cd '$pi_repo' && $pi_up" \
        > "$out/pi_up.log" 2>&1; then
    tail -20 "$out/pi_up.log"
    echo "HIL ERROR: the Pi side did not come up (see pi_up.log)"
    exit 2
fi
pi_state() { "${SSH[@]}" "docker inspect -f '{{.State.Status}} {{.State.ExitCode}}' $1 2> /dev/null || echo missing"; }
until [ "$(pi_state hil_flight_ready | cut -d' ' -f1)" = exited ]; do
    [ "$(date +%s)" -lt "$deadline" ] || { echo "HIL ERROR: flight_ready timed out"; exit 2; }
    sleep 5
done
ready="$(pi_state hil_flight_ready)"
t_ready="$(now)"
log "Pi flight_ready: $ready, $(python3 -c "print(round($t_ready - $t_start))") s after the start"
[ "$ready" = "exited 0" ] || { echo "HIL ERROR: flight_ready failed ($ready)"; exit 2; }

probe() {  # both hosts, in the background
    log "Probing the topics for $probe_s s"
    "${SIM[@]}" run --rm -T hil_probe python3 /workspace/tools/aerodrone_hil_probe.py \
        --duration-s "$probe_s" --label sim --output /out/probe_sim.json \
        --clock-offset-s "$(python3 -c "print(-($offset))")" $sim_topics \
        > "$out/probe_sim.log" 2>&1 &
    bg_pids+=($!)
    pi_sh "$pi_compose run --rm -T hil_probe python3 /workspace/tools/aerodrone_hil_probe.py \
--duration-s $probe_s --label pi --timesync --output /out/probe_pi.json \
--clock-offset-s $offset $pi_topics > '$pi_run/probe_pi.log' 2>&1" &
    bg_pids+=($!)
    # The sessions carrying HIL traffic, on both hosts: none may use
    # 10.0.0.0/24 (the wired LAN) or any address but the Wi-Fi ones.
    ss -tn > "$out/sockets_sim.txt"
    "${SSH[@]}" "ss -tn" > "$out/sockets_pi.txt"
}
page_check() {
    log "Loading the page from the Pi in a headless browser for ${HIL_PAGE_S:-60} s"
    docker run --rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "$PWD/tools:/tools:ro" -v "$out/page:/out" \
        "mcr.microsoft.com/playwright/python:v$playwright_version-noble" bash -c \
        "pip install -q --disable-pip-version-check playwright==$playwright_version &&
         python3 /tools/aerodrone_hil_page_check.py --url http://$pi_host:8080/ \
           --seconds ${HIL_PAGE_S:-60} --out /out" > "$out/page/page_check.log" 2>&1 &
    bg_pids+=($!)
}

status=0
problem=""
probed=false
case "$scenario" in
    mission|gps_denied)
        events="$out/denial_events.jsonl"
        while :; do
            [ "$(date +%s)" -lt "$deadline" ] || { problem="timed out after $timeout_s s"; break; }
            elapsed=$(python3 -c "print(int($(now) - $t_ready))")
            if ! "$probed"; then
                if { [ "$scenario" = mission ] && [ "$elapsed" -ge "$probe_delay" ]; } ||
                   grep -q '"event": "gps_disabled"' "$events" 2> /dev/null; then
                    probe; probed=true
                fi
            fi
            tree="$(pi_state hil_behavior_tree)"
            if [ "$scenario" = gps_denied ]; then
                denier="$("${SIM[@]}" "${sim_profiles[@]}" ps -a -q gps_denier)"
                [ "$(docker inspect -f '{{.State.Status}}' "$denier")" = exited ] && break
                if [ "${tree%% *}" = exited ] && ! grep -q '"event": "cruise"' "$events" 2> /dev/null; then
                    problem="the mission ended before reaching cruise"; break
                fi
            elif [ "${tree%% *}" = exited ]; then
                break
            fi
            sleep 15
        done
        tree="$(pi_state hil_behavior_tree)"
        "${SSH[@]}" "docker logs hil_behavior_tree" > "$out/pi/behavior_tree.log" 2>&1 || true
        log "Mission tree: $tree"
        validator="$("${SIM[@]}" "${sim_profiles[@]}" ps -a -q navsim_validator)"
        "${SIM[@]}" "${sim_profiles[@]}" logs --no-color navsim_validator > "$out/validation.log" 2>&1 || true
        validation="unfinished ($(docker inspect -f '{{.State.Status}}' "$validator" 2> /dev/null))"
        if [ "$(docker inspect -f '{{.State.Status}}' "$validator" 2> /dev/null)" = exited ]; then
            [ "$(docker inspect -f '{{.State.ExitCode}}' "$validator")" = 0 ] &&
                validation=passed || validation=failed
        fi
        log "Navigation validation $validation"
        ;;
    tracking|fire)
        # The workstation's CPU while the test flies (runs until stopped).
        python3 tools/tracking_sim_test.py cpu --project hil_sim --output "$out/cpu.jsonl" &
        cpu_pid=$!
        "${SIM[@]}" "${sim_profiles[@]}" up -d --scale tracking_sim_test=1 tracking_sim_test \
            >> "$out/up.log" 2>&1
        driver="$("${SIM[@]}" "${sim_profiles[@]}" ps -a -q tracking_sim_test)"
        t_driver="$(now)"
        log "Tracking driver started (docker logs -f $driver)"
        while :; do
            [ "$(date +%s)" -lt "$deadline" ] || { problem="timed out after $timeout_s s"; break; }
            [ "$(docker inspect -f '{{.State.Status}}' "$driver")" = exited ] && break
            if ! "$probed" && [ "$(python3 -c "print(int($(now) - $t_driver))")" -ge "$probe_delay" ]; then
                probe; probed=true
                [ "$scenario" = tracking ] && page_check
            fi
            sleep 10
        done
        driver_status="$(docker inspect -f '{{.State.Status}} {{.State.ExitCode}}' "$driver")"
        kill "$cpu_pid" 2> /dev/null || true
        log "Tracking driver: $driver_status"
        ;;
esac
t_end="$(now)"
for pid in "${bg_pids[@]}"; do wait "$pid" 2> /dev/null || true; done
bg_pids=()
clock_offset > "$out/clock_after.json"
teardown

[ -z "$problem" ] || { log "HIL ERROR: $problem"; status=2; }
log "Scoring"
case "$scenario" in
    mission)
        tree_exit="$(echo "$tree" | cut -d' ' -f2)"
        [[ "$tree_exit" =~ ^[0-9]+$ ]] || tree_exit=255
        python3 tools/aerodrone_hil_mission_score.py "$out" --tree-log "$out/pi/behavior_tree.log" \
            --tree-exit "$tree_exit" > "$out/mission_score.json" || { [ "$status" != 0 ] || status=1; }
        cat "$out/mission_score.json"
        [ "$validation" = passed ] || { [ "$status" != 0 ] || status=3; } ;;
    gps_denied)
        python3 tools/gps_denied_sim_scorer.py "$out" --stack gazebo || { [ "$status" != 0 ] || status=1; }
        [ "$validation" = passed ] || { [ "$status" != 0 ] || status=3; } ;;
    tracking|fire)
        if [ -s "$out/recording.jsonl" ]; then
            python3 tools/tracking_sim_test.py score "$out/recording.jsonl" --cpu "$out/cpu.jsonl" \
                --stack-log "$out/stack.log" > "$out/score.json" || { [ "$status" != 0 ] || status=1; }
            cat "$out/score.json"
        else
            echo "HIL ERROR: no tracking recording"; status=2
        fi
        [ "$driver_status" = "exited 0" ] || { [ "$status" != 0 ] || status=1; }
        if [ "$scenario" = tracking ] && ! python3 -c \
                "import json, sys; sys.exit(not json.load(open('$out/page/page_check.json'))['pass'])" \
                2> /dev/null; then
            echo "HIL: the page check failed (page/page_check.log)"; [ "$status" != 0 ] || status=1
        fi ;;
esac

python3 - "$out" "$scenario" "$t_start" "$t_sim_up" "$t_ready" "$t_end" "$sim_host" "$pi_host" \
    "$port" "$status" "$t_idle_end" <<'PY'
import json, os, sys
sys.path.insert(0, 'tools')
from aerodrone_hil_resources import summarize
out, scenario = sys.argv[1], sys.argv[2]
t_start, t_sim_up, t_ready, t_end = (float(v) for v in sys.argv[3:7])
sim_host, pi_host, port, status = sys.argv[7], sys.argv[8], sys.argv[9], int(sys.argv[10])
t_idle_end = float(sys.argv[11])

def load(name):
    try:
        with open(os.path.join(out, name)) as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return None

rows = []
try:
    with open(os.path.join(out, 'pi', 'resources.jsonl')) as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
except OSError:
    pass

def audit(name, local):
    """HIL sessions (Zenoh router, MAVLink, rosbridge, page) on this host's
    socket list: their addresses, and whether any is off the Wi-Fi ones."""
    hil_ports = {port, '5760', '9092', '8080'}
    found, bad = [], []
    try:
        lines = open(os.path.join(out, name)).read().splitlines()
    except OSError:
        return None
    for line in lines[1:]:
        fields = line.split()
        if len(fields) < 5:
            continue
        a, b = fields[3], fields[4]
        if a.rsplit(':', 1)[-1] in hil_ports or b.rsplit(':', 1)[-1] in hil_ports:
            if a.startswith('127.') or '[::ffff:127.' in a:
                continue
            found.append(f'{a} -> {b}')
            if not a.startswith(local) or not (b.startswith(sim_host) or b.startswith(pi_host)):
                bad.append(f'{a} -> {b}')
    return {'sessions': found, 'off_wifi': bad}

summary = {
    'scenario': scenario, 'status': status,
    'seconds': {'workstation_up': round(t_sim_up - t_start, 1),
                'flight_ready': round(t_ready - t_start, 1), 'total': round(t_end - t_start, 1)},
    'clock': {'before': load('clock_before.json'), 'after': load('clock_after.json')},
    'pi_resources': {'idle_before_stack': summarize(rows, None, t_idle_end),
                     'flying': summarize(rows, t_ready, t_end)},
    'probe_pi': load(os.path.join('pi', 'probe_pi.json')),
    'probe_sim': load('probe_sim.json'),
    'sockets': {'workstation': audit('sockets_sim.txt', sim_host),
                'pi': audit('sockets_pi.txt', pi_host)},
    'page_check': load(os.path.join('page', 'page_check.json')),
    'score': load('mission_score.json') if scenario == 'mission' else load('score.json'),
}
with open(os.path.join(out, 'hil_summary.json'), 'w') as stream:
    json.dump(summary, stream, indent=1)
print(json.dumps({k: summary[k] for k in ('scenario', 'status', 'seconds', 'sockets')}, indent=1))
PY
log "HIL $scenario: exit $status (summary in $out/hil_summary.json)"
exit "$status"
