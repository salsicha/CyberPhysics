# Native Zenoh validation — 12 October 2026

ROS 2 topics, services, actions and application APIs remain in place. The
default middleware is now `rmw_zenoh_cpp`, using Ubuntu 24.04 / ROS 2 Jazzy.
See the [deployment guide](../systems/ros2/README.md) before starting a mixed
set of old and rebuilt images.

## Runtime and build changes

The ROS base, SO101, UNO Q and VINS images install Jazzy's native
Zenoh package. Descendant images inherit it. Aerostack2 and its descendants
move from Humble to a pinned Jazzy-compatible source build at
`97ade9f1d43ac760da8f69312e8ef9d2b4bf2328`; Jazzy binaries were not available.
Isaac Sim 6.0.1 uses external Python 3.12 Jazzy libraries, loaded before Kit.
The desktop Aerostack2 image contains:

- `ros-jazzy-rmw-zenoh-cpp` 0.2.11-1noble.20260915.171026.
- `ros-jazzy-zenoh-cpp-vendor` 0.2.11-1noble.20260914.172423.
- `ros-jazzy-mavros` 2.16.0-1noble.20260927.165620.

Real builds and runs exposed three migration bugs that are fixed: the
BlueOS motion-controller launch needed its position-reference plugin name;
Gazebo's new AS2 camera bridge expected the `hd_camera` sensor name and
matching optical frames; and duplicated Gazebo camera-info sources needed
separate bridge processes to reach both RGB and depth destinations. The
Isaac ROS wrapper is executable both in the image and through a bind mount.

Peer multicast scouting is the default and does not require a router.
An optional native router uses TCP 7448, since BlueOS uses 7447. Routed
clients explicitly use client mode and disable multicast. Native Zenoh
Humble/Jazzy graphs do not interoperate, nor does native Zenoh directly
exchange ROS messages with DDS or `zenoh-bridge-ros2dds`.

The original micro-ROS XRCE agent and its paired subscriber explicitly retain
Fast DDS in `arduino_xrce.yaml`. A [subsequent firmware migration](vla_embedded_validation_20261012.md)
replaces the default Pico/ESP32 demo with Pico-ROS/zenoh-pico, tested over native
TCP and serial framing; physical-board acceptance remains pending.
Space ROS and its lunar rover graph now use native Zenoh after the
follow-up source rebuild below. MAVLink, USB, ZeroMQ policy RPC, Gazebo Transport and Foxglove WebSocket
transport are unchanged. Persistent 4 MiB socket limits and Fast DDS profiles
remain for the explicit DDS fallback. The Zenoh TCP path skips the DDS UDP
preflight.

The source-build job limit now also sets `MAKEFLAGS`: the installed colcon
otherwise adds its own Make job count, overriding `CMAKE_BUILD_PARALLEL_LEVEL`.
This was observed during the Pi build and checked against colcon's actual
argument-generation helper after correction.

### Space ROS source rebuild and validation

The first migration retained Cyclone because the pinned base links its core
directly to that implementation. The historical measurement remains in
[`zenoh_spaceros_exception.json`](validation/20261012/zenoh_spaceros_exception.json).
The follow-up audit completed the source migration: the exact pinned
`rmw_implementation`, `rcl`, `rclcpp` and `rclpy` sources now support runtime
selection. Rebuilding only those libraries exposed a second issue: installed
message dispatchers returned introspection support only. Rebuilding all installed
interfaces with the available Fast CDR serializers resolved node creation and
message publication. Source revisions are read from the pinned image's manifest.

The rebuilt image reported `rmw_zenoh_cpp` and ran the full Gazebo Curiosity
demo. All six controllers activated. The probe received 3,100 clock messages,
297 joint states, 62 images, 31 scans and 31 odometry messages; wheel positions
changed and movement/stop services responded. This was a smoke test with the
upstream demonstration active, not a complete driving course. The rebuilt
Python clients and C++ Gazebo/controller processes communicated successfully.
See [`zenoh_spaceros_gazebo.json`](validation/20261012/zenoh_spaceros_gazebo.json).

The smaller lunar-rover composition delivered 241 odometry messages and 240
nonzero velocity commands in 12 seconds, with changing position under native
Zenoh. See [`zenoh_spaceros_lunar.json`](validation/20261012/zenoh_spaceros_lunar.json).
The final repository image also passed its router-free native message/service
creation check and an actual Fast DDS fallback node/publisher check.
Both compositions now default to native Zenoh. Rebuild their image before
upgrading; installing an RMW package into the old image alone is insufficient.

## Transport recovery

Real Jazzy processes ran under different UIDs in host-networked containers.
The probe exercised 1280×720 RGB frames (2,764,800 bytes), a retained message
for late subscribers, a service and an action. Test actions used a dedicated
`/validation/zenoh/action` server and did not command a robot.

| Mode | Frames / interval | Recovery gap | Result |
| --- | --- | --- | --- |
| Peer discovery, server restart | 302 / 25 s | 1.53 s maximum | Both server generations delivered valid image payloads, retained state, services and actions. |
| Explicit router, multicast off, server/router restart | 421 / 35 s | 7.06 s maximum, including the deliberate outage | Both generations recovered all tested interfaces with zero payload errors. |

These rates include startup and intentional outages; they are functional
recovery tests, not sustained-throughput or zero-frame-loss claims. The
initial router test identified the missing client-mode setting; the guide
and successful rerun use the corrected configuration.

## Full flight runs

Each run includes the sensor/topic/TF validator, 150 simulated seconds of GPS
denial and 60 seconds of recovery. Thresholds were unchanged. The shared CAD
airframe and default Gazebo velocity-frame correction remain enabled.

The Gazebo run passed all checks, retained GUIDED mode with no failsafe,
and had complete correction coverage. Its fused median/p95/maximum error
during denial was **6.56 / 10.85 / 12.73 m**; maximum route error was 7.44 m.
The denial took 155.7 wall seconds (RTF 0.96). Data:
[`zenoh_gps_gazebo.json`](validation/20261012/zenoh_gps_gazebo.json).

Isaac also passed its validator and all flight checks, with complete correction
coverage, GUIDED mode throughout denial and no failsafe. Fused median/p95/
maximum error was **3.58 / 5.95 / 7.53 m**, and maximum route error was 6.68 m.
The denial took 778.3 wall seconds (RTF 0.19); recovery median error was 0.15 m.
Data: [`zenoh_gps_isaac.json`](validation/20261012/zenoh_gps_isaac.json).

The earlier [DDS baseline](gps_denied_sim_test.md#cad-airframe-baseline--12-october-2026)
is retained separately. Aerostack2 and ROS versions also changed, so these
runs do not isolate middleware as the cause of an accuracy or timing change.

## Raspberry Pi and OAK-1

Both Aerostack2 and OAK images built natively on the Raspberry Pi 5 as
`aerostack2-zenoh-arm64` and `oak1-zenoh-arm64`. The running ARM64 process
reported `rmw_zenoh_cpp`; image IDs and installed versions are retained in
[`zenoh_pi_runtime.json`](validation/20261012/zenoh_pi_runtime.json).
The final Dockerfile adds the compiler concurrency correction described above;
the tested binaries were built before that build-only correction.

With the real autopilot connected and OAK tracking running, the configured
19-plugin MAVROS process averaged **4.81% of one CPU** over 20 Docker samples.
A separate 30-second recording delivered IMU, raw IMU and pressure at 10 Hz,
and GPS at 5 Hz. Every observed autopilot state was connected, disarmed and
MANUAL. No test armed it, changed mode or commanded an actuator.

The 60-second active-camera run delivered 171 relative-depth frames (2.85 Hz),
30 high-resolution RGB frames (0.50 Hz), and 299 tracking triplets (4.98 Hz).
All detection/image/calibration timestamps paired, with zero unmatched
stamps. Depth was entirely finite, calibration was valid, and high-resolution
maximum delivery gap was 2.017 seconds. Tracking capture age p95 was about
0.308 seconds. These are processed pipeline rates, not the sensor frame rate;
RGB/depth publication timestamps do not measure sensor-to-result latency.
The camera was not aimed at a controlled scene, so this is a hardware and
transport test, not a perception-accuracy measurement.

With tracking unsubscribed, depth and aligned RGB rose to 269 frames /
4.48 Hz, and all 30 high-resolution frames arrived (maximum gap 2.013 s).
Both local camera runs passed their unchanged delivery checks.

A separate desktop subscriber discovered the Pi directly using the default
Zenoh peer configuration, with no router or explicit remote endpoint. Over
60 seconds across the LAN it received 171 depth/RGB frames, all 30 expected
1280×720 frames, and 300 matched tracking triplets with zero unmatched stamps.
The high-resolution maximum gap was 2.181 seconds. Tracking capture age p95
was about 0.394 seconds; cross-host age includes any clock offset and is not
an isolated network-latency measurement. All existing camera checks passed.
Data: [`zenoh_pi_lan.json`](validation/20261012/zenoh_pi_lan.json).

The temporary hardware and desktop probe containers were removed after the
successful runs; the four existing BlueOS containers stayed running.
Host receive/send limits remained 4 MiB, `~/.docker` was owned by `pi:pi`,
and the BlueOS log-archive timer remained active.

## Validation limits

The complete non-CAD regression suite passed **2,135 tests**, with six skips,
in 73.27 seconds after the policy and middleware fixes. Edited Python and
shell checks passed, as did Compose environment fallback validation. The
earlier CAD suite remains applicable because printable geometry did not
change during this migration.

Not every fleet image or GPU subsystem was rebuilt and exercised. The tested
desktop paths include ROS base, Aerostack2, DemNav, WildNav, Isaac and SO101;
other image recipes and connected Compose graphs were checked statically.
The follow-up audit added the full Space ROS Gazebo test, PanTiltROS GPU
image/runtime tests and successful ARM64 image CI. There was no physical
Jetson GPU acceptance or multi-robot field test. Image builds here are local; no container registry
publication is implied by pushing the repository.

References: [native RMW](https://github.com/ros2/rmw_zenoh),
[Isaac external ROS support](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html),
[Aerostack2 Jazzy support](https://github.com/aerostack2/aerostack2/issues/852).
