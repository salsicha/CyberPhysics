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

## Telemetry rates

ArduSub streams telemetry at its `SR0_*` stream rates. Its defaults give the
attitude and VFR_HUD 10 Hz but RAW_IMU, the barometer and GPS_RAW_INT 2 Hz,
GLOBAL_POSITION_INT 3 Hz, and a vehicle's stored values may be lower; MAVROS
asks for nothing more. The `submarine_message_rates` service asks ArduSub for
the rates in `config/message_rates.yaml` (edit it to change them; each rate's
reason is in its comments):

| MAVLink message | Hz | For |
| --- | --- | --- |
| ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU | 10 | `/mavros/imu/data`, the odometry's heading |
| LOCAL_POSITION_NED | 10 | `/mavros/local_position/odom` |
| SCALED_PRESSURE | 10 | `/mavros/imu/static_pressure` (the pressure mirror) |
| GLOBAL_POSITION_INT, GPS_RAW_INT | 5 | `/mavros/global_position/global` |
| VFR_HUD | 10 | `/mavros/vfr_hud` (ArduSub's default) |
| BATTERY_STATUS | 3 | `/mavros/battery` (ArduSub's default) |

It sends MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS
(`applications/aerostack2/scripts/mavros_message_rates.py`, using the request
function of the Aerodrone's flight_ready) when MAVROS connects, after every
reconnect, and whenever `/mavros/imu/data` stays below half its rate while
MAVROS is connected (an autopilot restart shorter than MAVROS's heartbeat
timeout, or a ground station resetting the streams), backing off to once a
minute while it stays low. ArduPilot keeps message intervals in RAM, for the
link a request came in on: here BlueOS's router, so every router client
(Cockpit, QGroundControl through BlueOS) gets them. A reboot forgets them,
which is why the service keeps running. It writes no parameter.

Measured on the Pi 5 Navigator (ArduSub 4.7.1, BlueOS 1.4.2, on the bench
without a DVL, GPS or Bar30), over 20 s:

| Topic | Before | After |
| --- | --- | --- |
| `/mavros/imu/data`, `/sub0/sensor_measurements/imu` | 10.1 Hz | 10.1 Hz |
| `/mavros/imu/data_raw` | 2.0 Hz | 10.1 Hz |
| `/mavros/imu/static_pressure`, `/sub0/sensor_measurements/pressure` | 2.0 Hz | 10.1 Hz |
| `/mavros/global_position/global`, `/sub0/sensor_measurements/gps` | 3.0 Hz | 5.1 Hz |
| `/mavros/vfr_hud` | 10.0 Hz | 10.0 Hz |
| `/mavros/battery`, `/sub0/sensor_measurements/battery` | 3.0 Hz | 3.0 Hz |

After an autopilot restart through BlueOS (MAVROS declared the link lost
and was back 16 s later) the service asked again and measured imu/data at
10 Hz 9 s after the reconnect, every rate as in the table. With the attitude
set back to 1 Hz by hand and MAVROS still connected, it noticed at its next
5 s check, asked again, and measured 10 Hz at the check after.
LOCAL_POSITION_NED, and with it the odometry, comes only once the EKF has an
origin (a DVL or GPS position).

The pressure mirror is MAVROS's `imu/static_pressure`, which MAVROS 2.14 makes
from SCALED_PRESSURE: ArduSub's barometer 0, on this bench Navigator the
board's own barometer (1003.6 hPa at 36 C). ArduSub reports an external depth
sensor such as a Bar30 as a further barometer (SCALED_PRESSURE2 when it is
the second), which MAVROS does not publish: check which barometer the depth
sensor is before reading `/sub0/sensor_measurements/pressure` as depth. The
EKF's depth is in the odometry's z and VFR_HUD's altitude either way.

## Simulation Scenario

```bash
docker compose -f compositions/submarine_sim.yaml up
```

The simulated scenario in `config/pipeline_inspection.yaml` drives an autonomous
submerged pipeline inspection with horizontal waypoints, commanded depth changes,
current drift, pressure telemetry, and battery discharge. The simulator
publishes MAVROS-like telemetry and consumes the same velocity setpoint topic
used by the hardware bridge.
