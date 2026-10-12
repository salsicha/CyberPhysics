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

ArduSub streams telemetry at its stream rates (`MAV1_*` on ArduSub 4.7,
`SR0_*` before it). Its defaults give the attitude and VFR_HUD 10 Hz but
RAW_IMU, the barometers and GPS_RAW_INT 2 Hz,
GLOBAL_POSITION_INT 3 Hz, and a vehicle's stored values may be lower; MAVROS
asks for nothing more. The `submarine_message_rates` service asks ArduSub for
the rates in `config/message_rates.yaml` (edit it to change them; each rate's
reason is in its comments):

| MAVLink message | Hz | For |
| --- | --- | --- |
| ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU | 10 | `/mavros/imu/data`, the odometry's heading |
| LOCAL_POSITION_NED | 10 | `/mavros/local_position/odom` |
| SCALED_PRESSURE2, SCALED_PRESSURE | 10 | `/sub0/sensor_measurements/pressure`: the depth sensor, or barometer 0 (`/mavros/imu/static_pressure`) without one ([Depth pressure](#depth-pressure)) |
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

With the depth sensor's SCALED_PRESSURE2 added to the requests (the bench
has no Bar30: ArduSub accepted the request and sent none), the bench
measured `/sub0/sensor_measurements/pressure` at 10.0 Hz, from barometer 0.

## Depth pressure

`/sub0/sensor_measurements/pressure` (`sensor_msgs/FluidPressure`, Pa) is
the water pressure of the external depth sensor, a Bar30 or Bar100. MAVROS
2.14 cannot give it: its only pressure topic, `/mavros/imu/static_pressure`,
is made from SCALED_PRESSURE, ArduSub's barometer 0, which on a Navigator is
the board's own barometer inside the enclosure (on the bench Navigator a
BMP390 on I2C bus 1, `BARO1_DEVID` 1340937: 1004.3 hPa at 34.8 C), and no
MAVROS plugin publishes SCALED_PRESSURE2 or SCALED_PRESSURE3. From
ArduSub 4.7.1's source:

- The Navigator's board barometer (BMP280 or BMP388/390 at I2C1 0x76) is
  probed first. With `BARO_EXT_BUS` at the Navigator's default of 6 (the
  I2C6 header; read back on the bench, with `BARO_PROBE_EXT` 0) ArduSub then
  probes a Bar30 (MS5837) and a Bar100 (Keller LD) on that bus, so the depth
  sensor is barometer 1 and arrives as SCALED_PRESSURE2.
- At boot ArduSub makes the first water-pressure barometer (a Bar30 or
  Bar100; the board's is an air barometer) its depth sensor and primary
  barometer (`BARO_PRIMARY`), which the EKF's height comes from
  (`EK3_SRC1_POSZ` 1), and it reports an absolute-pressure sensor in
  SYS_STATUS only when it has such a sensor. Without one it uses
  barometer 0 with a large measurement noise and refuses depth hold
  ("Depth sensor is not connected").
- Its depth reaches MAVLink as the EKF's: GLOBAL_POSITION_INT's relative
  altitude (`/mavros/global_position/rel_alt`) and, once the EKF has an
  origin, LOCAL_POSITION_NED's z and VFR_HUD's altitude above sea level.
  With no depth sensor ArduSub sends 0 as GLOBAL_POSITION_INT's and
  VFR_HUD's altitudes.
- SCALED_PRESSURE3 is barometer 2, unless ArduSub has a temperature
  sensor (such as Blue Robotics' Celsius): it then sends that temperature
  there.

So the bridge (`pressure_source: ardusub_depth_sensor` in
`config/bridge.yaml`; `applications/aerostack2/mavros_mirror`) subscribes to
the raw MAVLink stream MAVROS 2's router hands its UAS,
`/uas1/mavlink_source` (MAVROS 2 has no `/mavros/mavlink/from`), decodes the
autopilot's (1:1) SCALED_PRESSURE2 and publishes its absolute pressure, in
MAVROS's `base_link` frame, stamped when MAVROS received it. It uses
barometer 0 instead, and logs why:

- with a warning, when ArduSub's SYS_STATUS reports no depth sensor, or no
  SCALED_PRESSURE2 has arrived for `depth_sensor_timeout_s` (5 s). The value
  is then most likely the enclosure's air pressure, not depth.
- as information instead, when ArduSub reports a depth sensor but has sent
  no SCALED_PRESSURE2 since the bridge started: its only barometer is then
  the depth sensor.

A depth sensor that starts reporting later takes over without a restart.
The bridge comes from the aerostack2 image (`mavros_mirror` is installed in
it): rebuild that image and the submarine image to deploy this. An older
image ignores these parameters and mirrors barometer 0 without a warning.
`depth_sensor_message: SCALED_PRESSURE3` selects barometer 2 for a vehicle
whose depth sensor is the third barometer. The simulation
(`config/bridge_sim.yaml`) keeps `pressure_source: pressure_topic`: ArduSub
SITL's barometers are all water sensors and barometer 0 is its depth sensor.

On the bench (no Bar30), the bridge logged after 5 s
`no SCALED_PRESSURE2 (barometer 1) in 5 s; ArduSub's SYS_STATUS reports no
depth sensor: publishing barometer 0 (/mavros/imu/static_pressure), ...` and
published barometer 0 at 10 Hz. Its decoder read the raw SCALED_PRESSURE
messages as MAVROS published them (24 of 24 equal, 100431.98 Pa). Fed the
bench's barometer as a SCALED_PRESSURE2 200 hPa higher, a second bridge on
private topics published that value at 10 Hz while the SYS_STATUS it was
given reported a depth sensor, went back to barometer 0 with the warning
when it did not, and took the external sensor back without a restart.
Reading every raw MAVLink message (about 110 a second) costs the bridge
about 6.5 % of one Pi 5 core (9.0 % against 2.5 %). No real Bar30 has been
tested: on the vehicle, check that the bridge logs `Pressure from the
external depth sensor` and that `/sub0/sensor_measurements/pressure` follows
the depth Cockpit shows before relying on it.

## Simulation Scenario

```bash
docker compose -f compositions/submarine_sim.yaml up
```

The simulated scenario in `config/pipeline_inspection.yaml` drives an autonomous
submerged pipeline inspection with horizontal waypoints, commanded depth changes,
current drift, pressure telemetry, and battery discharge. The simulator
publishes MAVROS-like telemetry and consumes the same velocity setpoint topic
used by the hardware bridge.
