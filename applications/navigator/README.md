
## BlueOS Navigator controller boundary

The `salsicha/cyberphysics:navigator-latest` image inherits the pinned ArduPilot SITL firmware
image and is the controller used by the drone, boat, and submarine simulation
compositions. It builds ArduCopter, ArduRover, and ArduSub at the same pinned
revision so each simulated vehicle crosses the same MAVLink/MAVROS boundary as
a production BlueOS Navigator installation.

Simulation cannot emulate the physical Navigator HAT. Production deployments
replace SITL with BlueOS/ArduPilot on that board without changing the MAVROS or
Aerostack2 command boundary.

The production and simulation compositions do not run the legacy
`blueos_ros2_bridge.py`. BlueOS and ArduPilot own the mounted Navigator HAT,
including its IMU, barometer, ADC, PWM outputs, motor mixing, EKF, RC input,
and vehicle failsafes. MAVROS exposes the resulting autopilot state and command
interface to `as2_platform_blueos`.

Running this bridge at the same time as MAVROS would create two independent
MAVLink command paths. Keep it only as a standalone diagnostic tool.

## Raspberry Pi 5: keep BlueOS's kernel

ArduPilot's Linux HAL finds the Raspberry Pi model from the peripheral base
in the `ranges` of the device-tree node named exactly `soc`
(`AP_HAL_Linux/Util_RPI.cpp`: 4.5.3 opens `/proc/device-tree/soc/ranges`,
4.6.3, 4.7.1 and master in October 2026 also look for a node named `soc`). Raspberry Pi OS's 6.12 kernel package installs
device trees that name that node `soc@107c000000`, so on a BlueOS Pi 5 whose
OS was upgraded (`apt full-upgrade`) every ArduPilot build exits at start:

```text
"ranges" file not found
Unknown rpi_version, cannot locate peripheral base address
```

BlueOS then restarts it every 11 s and no autopilot heartbeat ever appears,
whatever firmware is installed. Seen on a Pi 5 with BlueOS 1.4.2 after an
upgrade to `linux-image-6.12.87+rpt-rpi-2712`: the image's original 6.6.31
device tree, still installed there, names the node `soc` (base 0x10, which
ArduPilot reads as a Pi 5), the 6.12.87 one `soc@107c000000`. Do not upgrade the Pi's kernel and
firmware packages under BlueOS; on a Pi that was upgraded, boot the 6.6
kernel and its device trees again, or wait for an ArduPilot release that
accepts the `soc@` node name.

See:
https://bluerobotics.com/learn/connecting-your-device-with-navigator-and-blueos/
