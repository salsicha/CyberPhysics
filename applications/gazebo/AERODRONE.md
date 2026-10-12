# Aerostack2 Gazebo simulation

[`aerostack2_sim.yaml`](../../compositions/aerostack2_sim.yaml) runs a real
Gazebo Fortress rigid-body simulation. ArduCopter SITL is the flight controller
behind the production-style MAVROS/`as2_platform_blueos` boundary; PWM output
drives Gazebo's native rotor model and Gazebo state is returned over ArduPilot's
JSON external-physics protocol.

The asset stage builds an 8 km Mount Tamalpais world from cached real SRTM
elevations and ArcGIS World Imagery. The textured render mesh and conservative
native collision cells use the same DEM. Native Gazebo RGB, RGB-D, IMU, pose,
and velocity outputs feed ArduPilot, DemNav, and WildNav—no synthetic sensor
publisher is started.

```bash
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

Nothing rendered may hang below the downward cameras: Gazebo's depth camera
reports geometry inside its near clip as `-inf`, so a visual under the lenses
blanks every depth frame (the landing gear is collision only).

Rendering is software (Mesa llvmpipe, EGL surfaceless), so no GPU is needed.
On a 20-core workstation, with the whole stack running, Gazebo keeps a real-time
factor of about 0.98 and renders both cameras at about 4.9 of their 5 Hz, more than
DemNav (every depth frame) and WildNav (`WILDNAV_RATE_HZ`, 0.2 Hz) use.

SITL loads only the shared ArduCopter parameters (`systems/aerodrone/config`):
in flight Gazebo's quadrotor learns a hover throttle of 0.67, the hardware
`MOT_THST_HOVER`, so unlike Isaac it needs no backend compensation.

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
