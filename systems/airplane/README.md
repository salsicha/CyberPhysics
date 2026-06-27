# Airplane System

This system describes a fixed-wing mapping airplane using Raspberry Pi 5,
BlueOS, Navigator, ArduPlane, GPS, and a downward camera for satellite-map and
DEM-assisted navigation.

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
