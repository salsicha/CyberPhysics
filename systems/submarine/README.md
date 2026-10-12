# Submarine System

This system describes a BlueROV2-class submarine using Raspberry Pi 5, BlueOS,
Navigator, ArduSub, and the reusable `applications/submarine` bridge.

Hardware (bill of materials, printed parts, printing and assembly):
[hardware/submarine.md](../../hardware/submarine.md).

The application does not replace BlueOS, ArduSub, QGroundControl, or Cockpit.
Those remain responsible for vehicle setup, calibration, arming, leak handling,
depth failsafes, manual operation, and mission control. CyberPhysics consumes
the MAVROS stream and republishes a stable ROS 2 topic contract.

## Expected BlueOS/MAVROS Setup

- BlueOS running on the onboard Raspberry Pi.
- Navigator running ArduSub firmware.
- MAVROS reaches the autopilot through BlueOS's MAVLink router, TCP
  `tcp://127.0.0.1:5777` (`BLUEOS_FCU_URL` in `config/blueos.env`): no
  BlueOS endpoint to add (`applications/aerostack2/README.md`, "BlueOS
  platform"). rosbridge listens on 9092: BlueOS holds 9090.
- Pressure/depth sensor, leak sensor, camera stream, lights, and thruster outputs
  configured in BlueOS/QGroundControl.
- Tether and topside network validated before enabling ROS velocity setpoints.

## Run

```bash
docker compose --env-file systems/submarine/config/blueos.env \
  -f compositions/submarine_hardware.yaml up
```

Bridge parameters live in `config/bridge.yaml`; topic names live in
`config/topics.yaml`.

## Simulation Scenario

```bash
docker compose -f compositions/submarine_sim.yaml up
```

The simulated scenario in `config/pipeline_inspection.yaml` drives an autonomous
submerged pipeline inspection with horizontal waypoints, commanded depth changes,
current drift, pressure telemetry, and battery discharge. The simulator
publishes MAVROS-like telemetry and consumes the same velocity setpoint topic
used by the hardware bridge.
