
## Navigator with Aerostack2

The production drone composition does not run the legacy
`blueos_ros2_bridge.py`. BlueOS and ArduPilot own the mounted Navigator HAT,
including its IMU, barometer, ADC, PWM outputs, motor mixing, EKF, RC input,
and vehicle failsafes. MAVROS exposes the resulting autopilot state and command
interface to `as2_platform_blueos`.

Running this bridge at the same time as MAVROS would create two independent
MAVLink command paths. Keep it only as a standalone diagnostic tool.

See:
https://bluerobotics.com/learn/connecting-your-device-with-navigator-and-blueos/
