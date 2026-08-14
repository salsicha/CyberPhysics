
## BlueOS Navigator controller boundary

The `cyberphysics/navigator` image inherits the pinned ArduPilot SITL firmware
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

See:
https://bluerobotics.com/learn/connecting-your-device-with-navigator-and-blueos/
