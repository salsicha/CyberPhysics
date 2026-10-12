# Isaac Sim

The `salsicha/cyberphysics:isaac-latest` image is built from NVIDIA Isaac Sim 6.0.1 and is the single Isaac runtime used by compositions in this repository. It bakes the shared configuration and simulator integration scripts from this directory into `/workspace/applications/isaac`.

Build it after authenticating Docker with NVIDIA NGC:

```bash
make -C applications build_isaac
```

## Aerodrone simulation

[`aerostack2_isaac.yaml`](../../compositions/aerostack2_isaac.yaml) runs the
owned image with PhysX rigid-body dynamics and ROS 2 camera
helpers. It loads the same real SRTM/orthophoto terrain and detailed behavior
tree as the Gazebo backend. ArduCopter SITL supplies motor commands through the
JSON external-physics protocol; MAVROS and `as2_platform_blueos` provide the
Navigator-compatible control boundary. The custom Isaac vehicle model includes
the same CAD-derived STL airframe, 3.1647 kg loaded-launcher mass budget, full
inertia tensor, rotor positions and camera extrinsics as Gazebo, with passive
rigid-body damping and body-frame IMU conversion. The shared assets and their
calibration limits are documented in
[simulator geometry](../../hardware/aerodrone.md#simulator-geometry).

```bash
git lfs pull --include="applications/aerostack2/models/aerodrone/*.stl"
docker compose -f compositions/aerostack2_isaac.yaml up
```

An NVIDIA GPU, NVIDIA Container Toolkit, NGC access while building, and acceptance of
the NVIDIA EULA are required. Persistent Isaac caches are initialized by the
composition. RGB/depth data comes from the Isaac render product rather than
procedural arrays. The supplied UDP-only Fast DDS profile is required for
interoperability between Isaac's bundled Humble middleware and the other ROS 2
containers.

`scripts/aerodrone_navsim.py` steps the physics exactly once per ArduPilot
servo frame (lock-step; `World.step(render=True)` would run a whole camera
period of physics inside one frame). Every `ISAAC_PHYSICS_HZ / ISAAC_CAMERA_HZ`
frames it renders that physics state, reads the RGB and depth annotators once
they hold the frame of the current simulation time (RTX delivers each frame
two renders later), and publishes the frames, their camera_info, the ground
truth and `/clock` with one stamp. The terrain is lit by a uniform sky dome,
so the ground shows the orthophoto's own colours.

The tracking targets (a car and two people, `applications/aerostack2/README.md`,
"Tracking targets and scenario") are visual-only meshes under `/World/Targets`,
built from the baked `model_mesh.npz` and texture atlas; each camera frame
moves them to their pose at the frame's simulation time before rendering, and
`/simulation/targets` is published with the frame.
Targets named on `/simulation/targets/hide` are drawn 50 m under the
terrain, out of the camera's sight (the tracking test's occlusion), their
truth still on their route and marked `"hidden": true`.
