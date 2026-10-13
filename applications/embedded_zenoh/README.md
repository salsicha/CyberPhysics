# Native Zenoh microcontroller demos

This replaces the Pico 2 W and ESP32/LAN8720 micro-ROS examples with **Pico-ROS
on zenoh-pico**. The MCU participates directly in the ROS 2 Jazzy Zenoh graph;
there is no XRCE agent or DDS hop. This is a firmware migration, not an RMW
environment variable applied to existing micro-ROS firmware. The original
rclc/micro-ROS sources remain available with
[`arduino_xrce.yaml`](../../compositions/arduino_xrce.yaml).

| ROS topic | Type | Behavior |
|---|---|---|
| `/micro_ros_arduino_subscriber` | `std_msgs/msg/Int32` | MCU input: zero turns the LED off, any other value turns it on |
| `/micro_ros_arduino_status` | `std_msgs/msg/Int32` | MCU output: acknowledges the received value |
| `/micro_ros_name` | `std_msgs/msg/String` | MCU input: name, at most 120 UTF-8 bytes |
| `/micro_ros_response` | `std_msgs/msg/String` | MCU output: `Hello <name>!` |

The ROS node retains the name `micro_ros_platformio_node`. Firmware and host
must share `ROS_DOMAIN_ID` (default 0). This implements the example's topics;
it is not a drop-in implementation of the entire micro-ROS service, action,
parameter, executor or QoS API. Existing applications using those APIs need
their own port. UNO Q uses its existing Linux/RouterBridge implementation and
is not a Pico 2 W firmware target.

## Build and test

On the development computer, from the repository root:

```bash
make -C applications build_embedded_zenoh_esp32
docker run --rm --network none --entrypoint bash \
  salsicha/cyberphysics:embedded_zenoh-latest -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/embedded_zenoh/tests/runtime_smoke.py'
docker run --rm --network none --entrypoint bash \
  salsicha/cyberphysics:embedded_zenoh-latest -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/embedded_zenoh/tests/runtime_smoke.py --transport serial --routed'
```

The first image compiles the actual MCU application's C sources for Linux and
RP2350/Pico 2 W. The second compiles the same application for ESP32 using
PlatformIO 6.2.0, espressif32 6.12.0 and Arduino-ESP32 2.0.17. The complete
Arduino IDE image is not needed. Firmware cross-builds were tested on amd64.
The ROS router can run on the Pi/Jetson with the normal `ros2-arm64` image.

The runtime tests use real `rclpy`/`rmw_zenoh_cpp`, native C pub/sub, ROS graph
discovery, signed Int32 extremes, both CDR byte orders, UTF-8/max-length strings,
malformed input, and router/client restarts. Serial is actual Zenoh framing
over paired pseudo terminals. **These tests do not exercise USB electronics,
ESP32 Ethernet, an LED or MCU scheduling.** Pico/ESP32 hardware was unavailable;
firmware was compiled but no board was flashed. CI builds both firmware targets
and runs both protocol tests, retaining firmware and logs as artifacts.

## Pico 2 W USB

Extract the UF2 without running or flashing a board:

```bash
mkdir -p generated/embedded_zenoh/firmware
container=$(docker create salsicha/cyberphysics:embedded_zenoh-latest)
docker cp "$container:/opt/embedded-pico/embedded_zenoh.uf2" generated/embedded_zenoh/firmware/
docker rm "$container"
```

Put the **Pico 2 W** into BOOTSEL mode and copy this UF2 to its mounted volume.
After reboot it exposes a USB CDC serial device. Identify that board's device
path (`/dev/serial/by-id/...` is preferable); do not flash a similarly named
UNO Q or another attached board. The firmware uses USB for protocol data and
UART for debug output. It does not require Wi-Fi credentials.

```bash
ZENOH_SERIAL_DEVICE=/dev/serial/by-id/YOUR_PICO_DEVICE \
  docker compose -f compositions/arduino.yaml --profile usb up -d
docker compose -f compositions/arduino.yaml --profile tools run --rm ros_sub
docker compose -f compositions/arduino.yaml --profile tools run --rm ros_sub \
  ros2 topic echo /micro_ros_arduino_status std_msgs/msg/Int32
```

The TCP router and serial router are separate processes so the ROS graph can
start with the MCU absent. A missing device makes only the USB service fail.
After physically unplugging/replugging the USB device, Docker may retain the
old device inode: recreate `serial_router` with `up -d --force-recreate
serial_router`. No broad `--privileged` or host `/dev` mount is needed.

The router uses TCP 7448 and a 10-second transport lease. It reconnects after
link/router loss and re-declares endpoints. Use an existing LAN router instead
of starting a second listener on the same port, or change `ZENOH_PORT` on this
stack and the MCU's locator together. Router transport is domain-independent;
the MCU and ROS client choose the domain.

For a different Pico locator/domain, rebuild in the image:

```bash
cmake -S /workspace/embedded_zenoh -B /opt/embedded-pico \
  -DCP_PICO=ON -DCP_ROS_DOMAIN_ID=7
cmake --build /opt/embedded-pico -j4
```

## ESP32 + LAN8720 Ethernet

The wiring matches the earlier Ethernet example: PHY address 0, power GPIO5,
MDC GPIO23, MDIO GPIO18, clock GPIO17 output, LED GPIO2. DHCP replaces the
old fixed `10.4.4.x` addresses. The router locator defaults to
`tcp/192.168.1.193:7448`. Start the default composition on that router host:

```bash
docker compose -f compositions/arduino.yaml up -d
```

For another host, add this line to `esp32/platformio.ini`'s `build_flags`, then
rebuild the ESP32 image (change the existing domain flag if needed):

```ini
    -DCP_ZENOH_LOCATOR=\"tcp/192.168.1.10:7448\"
```

After selecting the ESP32's actual programming port, explicitly upload:

```bash
docker run --rm --device /dev/ttyUSB0 \
  salsicha/cyberphysics:embedded_zenoh_esp32-latest \
  pio run -d /opt/embedded-esp32 -t upload --upload-port /dev/ttyUSB0
```

This is a serial firmware upload, not an over-the-network flash. Host ROS
nodes should connect to the same router using the
[shared Zenoh configuration](../../systems/ros2/README.md). A router image
or host RMW change alone does not change an already flashed microcontroller.

## Pinned sources and transport repairs

| Dependency | Revision | Upstream license |
|---|---|---|
| [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software) | `b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2` | BSD-3-Clause |
| [zenoh-pico fork](https://github.com/g4sp3r/zenoh-pico) | `0eb81272682a526b4bcf5a850c16ac74bbc6c73e` | Apache-2.0 / EPL-2.0 |
| Micro-CDR | `5a198fca92fba1819a04a7a6b9da5b2cbab008fd` | Apache-2.0 |
| Pico SDK 2.2.0 | `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779` | BSD-3-Clause |
| FreeRTOS V11.2.0 | `88971102bf8aa34b4fcc24f176db620dd9c854f7` | MIT |

Submodules are pinned by the Pico-ROS commit. Licenses remain in their cloned
source directories in the image. The small local patches address failures
observed during integration: TCP EOF otherwise spins until lease expiry;
serial startup can lose the first handshake and block indefinitely; a serial
RESET must initiate a new handshake; and disconnected USB writes need a
deadline. POSIX and MCU serial reads now time out, while initial USB enumeration
still waits for a host. The reconnect tests exercise the host transport paths;
the corresponding TinyUSB timeout paths require physical-board validation.

The ROS type-name component `dds_` is part of Jazzy's interface naming contract,
not evidence of a DDS transport. Native Zenoh keys include the RIHS01 hashes
of the two generated ROS types, and the runtime tests verify actual delivery.
