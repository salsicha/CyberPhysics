# UNO Q

Small ROS 2 Jazzy image for the Linux side of an Arduino UNO Q (Qualcomm
QRB2210, Debian, arm64, 2 or 4 GB RAM, 16 or 32 GB eMMC): the official
`ros:jazzy-ros-core` image (rclpy, `std_msgs`, `sensor_msgs`,
`diagnostic_msgs`, Fast DDS) plus `python3-msgpack` for the `arduino-router`
socket and `python3-serial`. About 0.5 GB, against about 4 GB for the
repository's `ros2` image.

On the UNO Q, Linux programs reach the STM32U585 MCU through `arduino-router`,
a msgpack-RPC router listening on `/var/run/arduino-router.sock`
(`srw-rw-rw- root root` with arduino-router 0.5.3) that owns the internal
serial link to the MCU. A sketch exposes functions with Arduino_RouterBridge
(`Bridge.provide("name", fn)`), and a client calls them by sending
`[0, msgid, "name", [params]]` to the socket and reading
`[1, msgid, error, result]`; the MCU's `Bridge.notify("name", ...)` reaches
the client that registered `name` with `$/register`. The containers bind-mount
the socket and run as root.

Scripts are not copied into the image; the compositions mount them from the
repository checkout on the board:

| Composition | Script | What |
|---|---|---|
| `compositions/turret_uno_q.yaml` (`--profile board`) | `applications/pantiltros/scripts/turret_arduino_bridge.py` | PanTiltROS turret bridge, `transport: uno_q_bridge` (`hardware/turret.md`) |
| `compositions/arduino_uno_q.yaml` | `applications/arduino/scripts/unoq_sub/unoq_sub_node.py` | UNO Q version of the micro-ROS `Sub` LED demo (`applications/arduino/README.md`) |

Both compositions cap the containers' open-file limit (`ulimits: nofile`).
Docker on the board gives containers a limit of about 2^30, and LTTng-UST, which
every ROS 2 Jazzy process loads, allocates and zeroes a descriptor bitmap
sized by it: 128 MiB per process. With the cap the turret bridge and the
`unoq_sub` node use about 66 MB each instead of about 200 MB.

Restarting `arduino-router` (`sudo systemctl restart arduino-router`, which
asks for the `arduino` user's password) recreates its socket file, and a
container's bind mount still points at the old one. The turret bridge and the
`unoq_sub` demo exit once the socket has been unreachable for
`router_lost_exit_s` (30 s), and their `restart: unless-stopped` policy starts
them again on the new socket. The compositions do not mount `/var/run`
itself, which would also expose `docker.sock`.

## Build

The compositions build the image on the board on first `up` (`pull_policy:
build`, tag `unoq-arm64`), which needs only the ros-core download and two apt
packages. With `pull_policy: build`, `docker compose run` builds again every
time (from the build cache when the image was built on the board). To build
elsewhere:

```bash
make -C applications build_unoq                   # amd64, unoq-latest
cd applications && make target_arm64 build_unoq   # arm64, needs QEMU binfmt on an amd64 host
```

### Board without internet

Load the base image over ADB from a PC with internet (no arm64 emulation
needed), give the board the two packages as arm64 .debs, and build there:

```bash
# On the PC, with the board on USB and adb working (about 1 min, 0.6 GB on the board)
docker pull --platform linux/arm64 ros:jazzy-ros-core
docker save ros:jazzy-ros-core | gzip -1 | adb shell docker load
mkdir -p unoq-debs && docker run --rm -v "$PWD/unoq-debs:/out" ubuntu:24.04 bash -c '
  sed -i "s/^Types: deb$/Types: deb\nArchitectures: amd64/" /etc/apt/sources.list.d/ubuntu.sources
  printf "Types: deb\nURIs: http://ports.ubuntu.com/ubuntu-ports\nSuites: noble noble-updates noble-security\nComponents: main universe\nArchitectures: arm64\nSigned-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg\n" \
    > /etc/apt/sources.list.d/ports.sources
  dpkg --add-architecture arm64 && apt-get update -qq &&
  cd /out && apt-get download python3-msgpack:arm64 python3-serial && chown -R '"$(id -u):$(id -g)"' /out'
adb shell mkdir -p /tmp/unoq
adb push unoq-debs applications/unoq/entrypoint.sh /tmp/unoq/
```

On the board (`adb shell`), the same image with the packages installed from
those files:

```bash
cat > /tmp/unoq/Dockerfile <<'EOF'
FROM ros:jazzy-ros-core
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive ROS_DISTRO=jazzy ROS2_DISTRO=jazzy
SHELL ["/bin/bash", "-c"]
COPY unoq-debs/ /tmp/debs/
RUN apt-get install -q -y --no-install-recommends /tmp/debs/*.deb && rm -rf /tmp/debs
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /scripts
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
EOF
docker build --network none -t salsicha/cyberphysics:unoq-arm64 /tmp/unoq
```

(`adb shell` sets `TMPDIR=/data/local/tmp`, which does not exist on the
board, so `docker build -f -` fails there; a Dockerfile in the context works.)
Then start the compositions with `up --no-build`, which uses this image;
`docker compose run` would try to build from the Dockerfile and fail without
apt access, so run the `unoq_led_on` publisher with
`docker compose -f compositions/arduino_uno_q.yaml --profile pub up --no-build unoq_led_on`.

## Verified on a real UNO Q

On an UNO Q without a network (Debian 13, Docker 26.1.5, Compose 2.26.1,
arduino-router 0.5.3), with the image built offline as above (520 MB on the
board): both compositions start with `up -d --no-build`; the turret bridge
connects, registers its notifications and sends its RPCs
(`hardware/turret.md`), and `unoq_sub` switches the LED with `set_led` from
`/micro_ros_arduino_subscriber` (the sketch answers `true` for on and `false`
for off, in about 7 ms). Recreating the socket file (a logging proxy in front
of the router, since restarting the router itself needs a password) made both
exit with status 1, 30 s (the bridge) and 32 s (`unoq_sub`, which probes
every 5 s) after the old socket went away, and Docker restarted them on the
new socket. Idle, the
bridge takes 35 % of one Cortex-A53 core (its 50 Hz poll timer) and
`unoq_sub` almost none.
