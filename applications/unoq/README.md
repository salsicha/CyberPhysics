# UNO Q

Small ROS 2 Jazzy image for the Linux side of an Arduino UNO Q (Qualcomm
QRB2210, Debian, arm64, 2 or 4 GB RAM, 16 or 32 GB eMMC): the official
`ros:jazzy-ros-core` image (rclpy, `std_msgs`, `sensor_msgs`,
`diagnostic_msgs`, Fast DDS) plus `python3-msgpack` for the `arduino-router`
socket and `python3-serial`. About 0.5 GB, against about 4 GB for the
repository's `ros2` image.

On the UNO Q, Linux programs reach the STM32U585 MCU through `arduino-router`,
a msgpack-RPC router listening on `/var/run/arduino-router.sock`
(root:arduino-router, mode 0660) that owns the internal serial link to the
MCU. A sketch exposes functions with Arduino_RouterBridge
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

## Build

The compositions build the image on the board on first `up` (`pull_policy:
build`, tag `unoq-arm64`), which needs only the ros-core download and two apt
packages. To build elsewhere:

```bash
make -C applications build_unoq                   # amd64, unoq-latest
cd applications && make target_arm64 build_unoq   # arm64, needs QEMU binfmt on an amd64 host
```

Restarting `arduino-router` (`sudo systemctl restart arduino-router`)
recreates its socket file, and a container's bind mount still points at the
old one. The turret bridge exits once the socket has been unreachable for
`router_lost_exit_s` (30 s) and its `restart: unless-stopped` policy starts
it again on the new socket; restart the `unoq_sub` demo by hand. The
compositions do not mount `/var/run` itself, which would also expose
`docker.sock`.
