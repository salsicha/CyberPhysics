# Aerostack2 Gazebo simulation

[`aerostack2_sim.yaml`](../../compositions/aerostack2_sim.yaml) runs a real
Gazebo Fortress rigid-body simulation. ArduCopter SITL is the flight controller
behind the production-style MAVROS/`as2_platform_blueos` boundary; PWM output
drives Gazebo's native rotor model and Gazebo state is returned over ArduPilot's
JSON external-physics protocol.

The backend rotates [Gazebo OdometryPublisher's body-frame FLU linear velocity](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc#L347-L365)
into world ENU using the ground-truth attitude, then converts ENU to ArduPilot's
NED frame. This is the default for every Gazebo mission, including tracking
and the [GPS-denied baseline test](../../tools/gps_denied_sim_test.md).
`GAZEBO_TWIST_IN_BODY_FRAME=false` is only for a replacement twist source
already expressed in world ENU; set it in the backend container's environment.

The asset stage builds an 8 km Mount Tamalpais world from cached real SRTM
elevations and ArcGIS World Imagery. The textured render mesh and conservative
native collision cells use the same DEM. Native Gazebo RGB, RGB-D, IMU, pose,
and velocity outputs feed ArduPilot, DemNav, and WildNav—no synthetic sensor
publisher is started.

```bash
git lfs pull --include="applications/aerostack2/models/aerodrone/*.stl"
docker compose -f compositions/aerostack2_sim.yaml up
```

On first launch the elevation and imagery seed services require network access.
The readiness gate can take several minutes on software rendering because it
waits for ArduPilot GPS/EKF initialization before starting the generated
20-target behavior-tree mission.

Gazebo models and launch configuration live here. The shared scenario, mission, robustness contracts, and scoring tools live under `applications/aerostack2/`. `config/world_gazebo.yaml` is a small development fixture; the
normal composition uses the generated `/data/navsim_assets/world_gazebo.yaml`.

The `gazebo`, `gazebo_bridges` and `gazebo_ardupilot_backend` services run the
aerostack2 image, which installs Gazebo Fortress (`ros_gz`) and Aerostack2's
`as2_gazebo_assets` 1.1.3, pinned from the ROS snapshot like MAVROS
(`applications/aerostack2/Dockerfile`). The composition runs its
`launch_simulation.py` and bind-mounts `models/*.sdf.jinja` and
`scripts/drone_bridges_isolated.py` over the package's `quadrotor_base`,
`hd_camera` and `rgbd_camera` templates and `launch/drone_bridges.py`; the image
build fails if any of those files is missing.

The full CAD airframe, including landing gear and launcher, is rendered from
the shared mesh. Both downward cameras use the nose-mounted OAK lens position
from its manifest. Camera placement and self-occlusion are part of the depth
stream validation when changing the assembly.

Rendering is software (Mesa llvmpipe, EGL surfaceless), so no GPU is needed.
On a 20-core workstation, with the whole stack running, Gazebo keeps a real-time
factor of about 0.98 and renders both cameras at about 4.9 of their 5 Hz, more than
DemNav (every depth frame) and WildNav (`WILDNAV_RATE_HZ`, 0.2 Hz) use.

SITL loads the shared ArduCopter parameters (`systems/aerodrone/config`).
Rotor layout, maximum speed and thrust constant now come from the loaded
airframe model; the earlier generic quadrotor's measured hover throttle of
0.67 is not a calibration of this 3.165 kg assembly.

The tracking targets (a car and two people, `applications/aerostack2/README.md`,
"Tracking targets and scenario") are visual-only, gravity-free models the
terrain builder puts in the world. The `gazebo_targets` service
(`scripts/gazebo_tracking_targets.py`) moves them along their tracks every
0.02 s of simulation time through the world's `set_pose` service (it starts
the ros_gz_bridge for it) and publishes `/simulation/targets` at the same
times. A camera frame can show a pose up to one update old (0.1 m for the
car).
Targets named on `/simulation/targets/hide` are drawn 50 m under the
terrain, out of the camera's sight (the tracking test's occlusion), their
truth still on their route and marked `"hidden": true`.
