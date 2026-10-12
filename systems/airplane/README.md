# Airplane System

This system describes a fixed-wing mapping airplane using Raspberry Pi 5,
BlueOS, Navigator, ArduPlane, GPS, and a downward camera for satellite-map and
DEM-assisted navigation.

Hardware (bill of materials, printed parts, printing and assembly):
[hardware/airplane.md](../../hardware/airplane.md).

The recommended physical stack is ArduPlane on BlueOS/Navigator for flight
control, MAVROS for ROS telemetry, OAK-1 MegaDepth for nadir RGB/depth, DemNav
for terrain correlation, and WildNav for satellite image matching. ArduPlane and
QGroundControl remain responsible for calibration, arming, mission upload,
manual control, and failsafes.

## Hardware Run

```bash
docker compose --env-file systems/airplane/config/blueos.env \
  -f compositions/airplane_hardware.yaml up
```

## Telemetry rates

ArduPlane streams telemetry at its `SR0_*` stream rates, all 1 Hz on a fresh
vehicle, and MAVROS asks for nothing more: `/mavros/imu/data` and every topic
the stack mirrors ran at 1 Hz. That is too slow for DemNav and WildNav, which
take the attitude at each camera frame from the odometry, and for the
navigation fusion, which publishes `/airplane/navigation/odometry` at the raw
odometry rate. The `airplane_message_rates` service asks ArduPlane for the
rates in `config/message_rates.yaml` (edit it to change them; each rate's
reason is in its comments):

| MAVLink message | Hz | For |
| --- | --- | --- |
| ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU | 20 | `/mavros/imu/data`, the odometry's orientation |
| LOCAL_POSITION_NED | 20 | `/mavros/local_position/odom` |
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
(Cockpit, a GCS through BlueOS) gets them, but not the SiK radio's serial
port, which keeps its own `SRn_*` rates. A reboot forgets them, which is why
the service keeps running. It writes no parameter.

Measured on the Pi 5 Navigator (ArduPlane 4.6.3, BlueOS 1.4.2, on the bench
without GPS), over 20 s:

| Topic | Before | After |
| --- | --- | --- |
| `/mavros/imu/data`, `/plane0/sensor_measurements/imu` | 1.0 Hz | 20.0 Hz |
| `/mavros/imu/data_raw` | 1.0 Hz | 20.0 Hz |
| `/mavros/global_position/global`, `/plane0/sensor_measurements/gps` | 1.0 Hz | 5.0 Hz |
| `/mavros/vfr_hud` | 1.0 Hz | 5.0 Hz |

Without a battery monitor (`BATT_MONITOR` 0, as there) ArduPlane sends no
BATTERY_STATUS and `/mavros/battery` stays at the SYS_STATUS rate (1 Hz).
LOCAL_POSITION_NED, and with it `/mavros/local_position/odom`, comes only once
the EKF has an origin (a GPS fix). The service logs what it asked for and the
rate it then sees (`docker logs airplane_message_rates`: "imu/data at 20.0 Hz
(requested 20 Hz)").

## Simulation Run

```bash
docker compose --env-file systems/airplane/config/sitl.env \
  -f compositions/airplane_sim.yaml up
```

The simulation stack uses ArduPilot Plane SITL for fixed-wing autopilot
behavior. JSBSim can be selected with `AIRPLANE_SITL_MODEL=jsbsim` when a local
JSBSim build is desired, but the default uses ArduPilot's built-in plane model
because it is stable under Docker Desktop emulation on Mac. The stack also
starts a synthetic georeferenced nadir camera, DEM, and satellite-tile source so
WildNav and DemNav can be fully exercised without a full Unreal, FlightGear, or
Gazebo terrain scene.

On first run, the SITL service uses the ArduPilot dev-base container and clones
ArduPilot into `cache/ardupilot` so `Tools/autotest/sim_vehicle.py` is available
without committing simulator source into this repository.

## Scenario

The default scenario is a coastal mapping sortie near the Golden Gate/Mount
Tamalpais region. The aircraft climbs to a mapping altitude, flies a lawnmower
survey, tolerates GPS degradation by leaning on WildNav/DemNav corrections, and
returns toward the launch corridor.

Full photorealistic validation should replace the synthetic nadir camera and
map backends with a terrain simulator that can render georeferenced
orthophoto/DEM tiles under the same `INITIAL_LAT`, `INITIAL_LON`, and
`ORIGIN_ALT` values.

## GPS_INPUT corrections (off)

The Aerodrone stacks feed accepted DemNav/WildNav fixes back to ArduCopter as
GPS 2 (MAVLink GPS_INPUT; `tools/gps_denied_sitl_test.md`). The airplane
stacks keep that off (`GPS_INPUT_CORRECTIONS=false` in `config/blueos.env`,
`gps_input_enabled:=false` in `compositions/airplane_sim.yaml`): it was
designed and SITL-validated on ArduCopter only. Enabling it on ArduPlane
needs:

- an ArduPlane parameter file layered after `config/ardupilot_plane.params`
  (in SITL too), with the EKF3 source set 2 / `EK3_SRC_OPTIONS=0` / GPS 1
  primary values the navigation bridge checks (its `required_fc_params`,
  mirroring `systems/aerodrone/config/arducopter_gps_input.params`), reviewed
  for fixed-wing flight: ArduPlane can dead-reckon on airspeed and has no
  EKF-failsafe landing, so its loss-of-position behaviour differs;
- a GPS-denied ArduPlane SITL run (`tools/sitl_smoke_test.sh` with
  `SMOKE_GPS_DENY_AFTER_S`) with and without corrections, showing that fixes
  arrive less than about 7 s apart at cruise speed and that the EKF accepts
  them;
- then the supervised flight test described for the Aerodrone.
