# Boat System

This system describes a BlueBoat-class uncrewed surface vessel using Raspberry
Pi 5, BlueOS, Navigator, ArduRover, and the reusable `applications/boat` bridge.

The application does not replace BlueOS, ArduRover, QGroundControl, or Cockpit.
Those remain responsible for vehicle setup, calibration, arming, manual
operation, failsafes, geofencing, and mission control. CyberPhysics consumes the
MAVROS stream and republishes a stable ROS 2 topic contract.

## Expected BlueOS/MAVROS Setup

- BlueOS running on the onboard Raspberry Pi.
- Navigator running ArduRover firmware.
- MAVLink endpoint available to MAVROS, usually
  `udp://0.0.0.0:14550@`.
- GPS and compass calibrated in BlueOS/QGroundControl.
- RC/gamepad/manual override validated before enabling ROS velocity setpoints.

## Run

```bash
docker compose --env-file systems/boat/config/blueos.env \
  -f compositions/boat_hardware.yaml up
```

Bridge parameters live in `config/bridge.yaml`; topic names live in
`config/topics.yaml`.
