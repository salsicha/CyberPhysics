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
