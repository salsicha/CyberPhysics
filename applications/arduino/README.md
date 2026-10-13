# Arduino

For the Pico 2 W and ESP32 ROS topic demos, the default is now
[native Zenoh firmware](../embedded_zenoh/README.md) and
`compositions/arduino.yaml`. Build/flash that firmware before migrating an
existing board. The micro-ROS sources documented below remain a legacy
XRCE/DDS fallback via `compositions/arduino_xrce.yaml`; they do not communicate
with a native Zenoh router without reflashing. UNO Q and turret firmware keep
their existing board-specific protocols.

Arduino IDE 2.3.10, PlatformIO and arduino-cli in one image
(`salsicha/cyberphysics:arduino-latest`, `make -C applications build_arduino`),
with the projects in `scripts/` built at image build time:

| Folder | Board | Built with |
|---|---|---|
| `scripts/Sub` | Raspberry Pi Pico 2 W: micro-ROS Int32 subscriber driving an LED (serial) | PlatformIO |
| `scripts/UDP` | Raspberry Pi Pico 2 W: Wi-Fi UDP | PlatformIO |
| `scripts/ethernet_pubsub` | ESP32 + LAN8720: micro-ROS over Ethernet | PlatformIO |
| `scripts/turret_firmware` | UNO R4 WiFi and UNO Q MCU: PanTiltROS turret (ST3215/STS3215 servos) | arduino-cli |
| `scripts/unoq_sub` | UNO Q: the `Sub` demo through the UNO Q Bridge instead of micro-ROS | arduino-cli |

The micro-ROS projects pin micro_ros_platformio cfee17f (ROS 2 Jazzy). The
image's arduino-cli 1.5.1 has the board packages `arduino:renesas_uno` 1.6.0
(UNO R4) and `arduino:zephyr` 1.0.0 ("Arduino Q Boards", FQBN
`arduino:zephyr:unoq`) and the UNO Q libraries Arduino_RouterBridge 0.4.3,
Arduino_RPClite 0.3.1, MsgPack 0.4.2, DebugLog 0.8.4, ArxContainer 0.7.0 and
ArxTypeTraits 0.3.2 (Dockerfile `ARG`s); the IDE in the image uses the same
installation. The two board packages add about 1.5 GB to the image (the Zephyr
toolchain alone is about 0.8 GB). PlatformIO cannot build for the UNO Q MCU
(platformio/platform-ststm32#869), so the micro-ROS PlatformIO projects cannot
target it; on the UNO Q, ROS 2 runs on the Linux side instead (below).

Compile a sketch by hand in the container (sizes are printed at the end):

```bash
docker run --rm salsicha/cyberphysics:arduino-latest \
  arduino-cli compile --fqbn arduino:zephyr:unoq turret_firmware
```

## Turret firmware

`scripts/turret_firmware` drives the PanTiltROS turret's serial bus servos
from an UNO R4 WiFi (ASCII line protocol on USB, for
`turret_arduino_bridge.py` on the host) or from the UNO Q MCU (RouterBridge
RPC, for the bridge on the UNO Q's Linux side). The servo protocol, limits,
watchdog and error reporting live in `src/` and are shared by both front ends;
`test/host_test.cpp` tests them on the host (`tests/test_turret_firmware.py`).
Settings are in `src/turret_config.h`. Wiring, flashing and deployment:
`hardware/turret.md`.

Besides pan and tilt it drives the foam dart launcher
(`systems/racecar_turret/interfaces.md`, "Foam dart launcher") when
`TURRET_ACTUATOR_FITTED` is 1 (the default in `src/turret_config.h`; 0 for a
turret without one; the design is `hardware/parts/turret/_launcher.py`): a
third ST3215, ID 14, turning the pinion (13.5 mm pitch radius) of the
priming rack, `actuator_joint` in metres (0 forward, 0.038 primed; the sear
catches from 0.03425), and a fourth ST3215, ID 15, on the trigger,
`trigger_joint` in radians (0 safe, 0.6 fires: the sear lifts 5.4 mm and
releases at 4.45 mm). The steps at 0 and the directions are placeholders,
set at bring-up (`hardware/parts/turret/README.md`).

- Serial: `SET pan_joint <rad> tilt_joint <rad> [actuator_joint <m>]
  [trigger_joint <rad>]`, and `POS ...` carries `actuator_joint <m>
  trigger_joint <rad>` when both launcher servos answered.
- UNO Q: `turret/set [pan, tilt]` as before, `turret/set_all [pan, tilt,
  actuator, trigger]`, and `turret/pos` / `turret/get_position` with four
  elements when both launcher servos answered.
- Goals are clamped to their limits and travel in the same sync write as
  pan and tilt. A `SET` that does not name the trigger returns it to 0; one
  that does not name the rack stops the rack at its present position, read
  from the servo just before the write (so the rack never moves toward a
  stale goal). `STOP` and the 500 ms watchdog do both, while pan and tilt
  keep their hold behaviour (`stop_disables_torque` applies to them only).
  The firmware never primes or fires by itself: only a `SET` naming the
  rack draws it, and only one naming the trigger lifts it.
- The launcher servos' torque limit register (48, 0.1 % units) is written
  after every torque enable (`torque_limit` in their `JointConfig`; the
  register is RAM and resets at power-up): the rack's 550 (55 %: priming
  needs about 0.9-1.05 N m of the servo's 2.9 N m) and the trigger's 300
  (30 %: with the safety catch on it stalls against it). The limits also
  cap the stall current.
- A missing or faulting launcher servo is reported like the others
  (`servo 14 (actuator_joint) timeout`), and pan and tilt positions keep
  flowing without it.
- Firmware built without the launcher ignores its joints with an error at
  most once a second and keeps driving pan and tilt.

Both builds compile (`arduino-cli compile` for `arduino:renesas_uno:unor4wifi`
and `arduino:zephyr:unoq` in the image), and the host tests cover the
launcher; the firmware has not run with servos.

## Arduino UNO Q Sub demo

The UNO Q counterpart of `Sub`. The MCU sketch `scripts/unoq_sub/unoq_sub.ino`
provides the RPC `set_led(int32)` (0 off, anything else on; the red channel of
RGB LED 3) through Arduino_RouterBridge. On the board's Linux side,
`scripts/unoq_sub/unoq_sub_node.py` subscribes to the same
`/micro_ros_arduino_subscriber` (`std_msgs/msg/Int32`) topic and calls
`set_led` through the `arduino-router` socket (`/var/run/arduino-router.sock`,
msgpack-RPC). No micro-ROS is involved.

1. Flash the sketch to the MCU, from a PC with the UNO Q on USB-C (the board's
   USB port belongs to its Linux side; the core uploads through it):

   ```bash
   arduino-cli board list      # the UNO Q and its port
   arduino-cli compile --upload -b arduino:zephyr:unoq -p <port> scripts/unoq_sub
   ```

   or, from this image, with the board's USB device (ADB) and serial port:

   ```bash
   docker run --rm --device "/dev/bus/usb/$(lsusb -d 2341:0078 | awk '{print $2 "/" substr($4, 1, 3)}')" \
     --device /dev/ttyACM0 salsicha/cyberphysics:arduino-latest \
     arduino-cli compile --upload -b arduino:zephyr:unoq -p /dev/ttyACM0 unoq_sub
   ```

   The upload runs OpenOCD on the board over ADB (`hardware/turret.md`, "Arduino
   UNO Q"). The board's serial port, `/dev/ttyACM0` on the PC, carries the
   sketch's `Monitor` text, not the MCU's UART. You can also open the sketch
   in the Arduino IDE 2, or copy it into an Arduino App Lab app on the board.
2. On the UNO Q, from a checkout of this repository (Docker comes with the
   board's Debian image; the small `unoq` image, `applications/unoq`, is built
   on the board the first time, or offline as in `applications/unoq/README.md`,
   in which case use `up --no-build`):

   ```bash
   docker compose -f compositions/arduino_uno_q.yaml up -d
   docker compose -f compositions/arduino_uno_q.yaml run --rm unoq_led_on
   ```

   or publish from any machine on the same `ROS_DOMAIN_ID`:
   `ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32 "data: 1"`.

Run on a real UNO Q (offline image, `up --no-build`): `set_led(1)` and
`set_led(0)` through the router socket return `true` and `false`, and the node
logs "LED on" and "LED off" for messages 1, 5 and 0. The LED itself was not
watched.

The UNO Q can host a native Zenoh router for the new firmware, or a legacy
micro-ROS agent for the Pico/ESP32 demos below, for
example the agent of the repository's `ros2` image built for arm64
(`make -C applications target_arm64 build_ros2`, about 4 GB) or the upstream
`microros/micro-ros-agent:jazzy` image, with the Pico on the board's USB-C
through a powered hub or the ESP32 on the same network. This has not been tried
here.

## TODO:
- get WiFi UDP micro-ros example working for Pico 2 W  
- get ESP32 PoE board


## Legacy micro-ROS walkthrough

(This uses Raspberry Pi Pico 2 W and Fast DDS. For native Zenoh use the linked
firmware workflow above.)

1. 
cd Sub/ && pio run -e pico --target upload --upload-port /dev/ttyUSB0  

2. 
Unplug and reconnect pico  

3. 
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200

4. 
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic pub /micro_ros_arduino_subscriber std_msgs/msg/Int32 "data: 1" -1



## Note on accessing devices

(The install.sh script does this)  
sudo usermod -a -G dialout $USERNAME  

OR  

(If I switch to the non-daemon/root version of docker)  
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules  
sudo service udev restart  
