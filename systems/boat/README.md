# Boat System

This system describes a BlueBoat-class uncrewed surface vessel using Raspberry
Pi 5, BlueOS, Navigator, ArduRover, and the reusable `applications/boat` bridge.

Hardware (bill of materials, printed parts, printing and assembly):
[hardware/boat.md](../../hardware/boat.md).

The application does not replace BlueOS, ArduRover, QGroundControl, or Cockpit.
Those remain responsible for vehicle setup, calibration, arming, manual
operation, failsafes, geofencing, and mission control. CyberPhysics consumes the
MAVROS stream and republishes a stable ROS 2 topic contract.

## Expected BlueOS/MAVROS Setup

- BlueOS running on the onboard Raspberry Pi.
- Navigator running ArduRover firmware.
- MAVROS reaches the autopilot through BlueOS's MAVLink router, TCP
  `tcp://127.0.0.1:5777` (`BLUEOS_FCU_URL` in `config/blueos.env`): no
  BlueOS endpoint to add (`applications/aerostack2/README.md`, "BlueOS
  platform"). rosbridge listens on 9092: BlueOS holds 9090.
- GPS and compass calibrated in BlueOS/QGroundControl.
- RC/gamepad/manual override validated before enabling ROS velocity setpoints.

## Run

```bash
docker compose --env-file systems/boat/config/blueos.env \
  -f compositions/boat_hardware.yaml up
```

Bridge parameters live in `config/bridge.yaml`; topic names live in
`config/topics.yaml`.

## Telemetry rates

ArduRover streams telemetry at its `SR0_*` stream rates, 1 Hz on a fresh
vehicle, and MAVROS asks for nothing more: `/mavros/imu/data` and every
`/boat0/sensor_measurements/*` topic ran at 1 Hz. A waypoint follower such as
`applications/boat/scripts/harbor_waypoint_nav.py` steers once per odometry
message. The `boat_message_rates` service asks ArduRover for the rates in
`config/message_rates.yaml` (edit it to change them; each rate's reason is in
its comments):

| MAVLink message | Hz | For |
| --- | --- | --- |
| ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU | 10 | `/mavros/imu/data`, the odometry's heading |
| LOCAL_POSITION_NED | 10 | `/mavros/local_position/odom` |
| GLOBAL_POSITION_INT, GPS_RAW_INT | 5 | `/mavros/global_position/global` |
| VFR_HUD | 5 | `/mavros/vfr_hud` |
| BATTERY_STATUS | 2 | `/mavros/battery` |

It sends MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS
(`applications/aerostack2/scripts/mavros_message_rates.py`, using the request
function of the Aerodrone's flight_ready) when MAVROS connects, after every
reconnect, and whenever `/mavros/imu/data` stays below half its rate while
MAVROS is connected (an autopilot restart shorter than MAVROS's heartbeat
timeout, or a ground station resetting the streams), backing off to once a
minute while it stays low. ArduPilot keeps message intervals in RAM, for the
link a request came in on: here BlueOS's router, so every router client
(Cockpit, a GCS through BlueOS) gets them. A reboot forgets them, which is
why the service keeps running. It writes no parameter.

Measured on the Pi 5 Navigator (ArduRover 4.6.3, BlueOS 1.4.2, on the bench
without GPS), over 20 s:

| Topic | Before | After |
| --- | --- | --- |
| `/mavros/imu/data`, `/boat0/sensor_measurements/imu` | 1.0 Hz | 10.0 Hz |
| `/mavros/imu/data_raw` | 1.0 Hz | 10.0 Hz |
| `/mavros/global_position/global`, `/boat0/sensor_measurements/gps` | 1.0 Hz | 5.0 Hz |
| `/mavros/vfr_hud` | 1.0 Hz | 5.0 Hz |

After an autopilot restart through BlueOS (MAVROS declared the link lost
and was back 11 s later) the service asked again and measured imu/data at
10 Hz 9 s after the reconnect. On the bench ArduRover sent no BATTERY_STATUS
(no battery monitor set up), so `/mavros/battery` stayed at the SYS_STATUS
rate (1 Hz); LOCAL_POSITION_NED, and with it the odometry, comes only once
the EKF has an origin (a GPS fix). The service logs what it asked for and
the rate it then sees (`docker logs boat_message_rates`).

## Simulation Scenario

```bash
docker compose -f compositions/boat_sim.yaml up
```

The simulated scenario in `config/harbor_survey.yaml` drives an autonomous
harbor survey through a rectangular marina basin with wind/current drift. The
simulator publishes MAVROS-like telemetry and consumes the same velocity
setpoint topic used by the hardware bridge.
