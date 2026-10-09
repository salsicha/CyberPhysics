# SO-101 System

This directory contains configuration for the SO-101 hardware setup used by the
SO-101 Gazebo, Genesis, and Isaac Sim adapters.

- `urdf/so101.urdf.xacro`: canonical robot description.
- `urdf/so101.urdf`: generated plain URDF for simulators that do not read Xacro.
- `config/controllers.yaml`: ROS 2 control configuration for Gazebo.
- `config/groot_demo.yaml`: SO-101 + GR00T bridge parameters for Isaac Sim.
- `worlds/empty.sdf`: minimal Gazebo world for the arm.
- `worlds/picking_table.sdf`: tabletop picking scene with bins, target objects, clutter, an occluder, calibration board, lighting, and collision geometry.
- `scenarios/picking_table.json`: simulator-neutral task and scene metadata for Isaac Sim, Genesis, Gazebo, and GR00T scoring.
- `validation/groot_demo_sync_manifest.example.json`: example topic synchronization manifest for GR00T regression bags.
- `validation/acceptance_thresholds.json`: default SO-101 picking acceptance thresholds.

Regenerate the plain URDF after editing the Xacro:

```bash
applications/so101/scripts/regenerate_urdf.sh
```

## GR00T Demo Camera

The SO-101 URDF includes a fixed `groot_camera_link` and
`groot_camera_rgb_optical_frame`. The scenario's `camera` block
(`scenarios/picking_table.json`) is the single source of the camera model: Isaac
renders from it, `/so101/camera/camera_info` publishes its intrinsics (69° HFOV,
640x480) and the URDF joint `base_to_groot_camera` must equal its pose
(`validate_calibration_and_dataset.py` checks this).

The same pose is the nominal mount for the physical rig: the camera sits
0.17 m forward, 0.52 m to the right of and 0.47 m above `base_link`, pitched 45°
down and yawed 83° to look across the table at the arm (xyz `0.17 -0.52 0.4665`,
rpy `0 0.785 1.449`). It is a nominal value, not a measurement of a particular
rig: after mounting a RealSense, measure its pose (e.g. with an AprilTag on the
base) and update both the scenario and the URDF.
