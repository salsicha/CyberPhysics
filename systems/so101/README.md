# SO-101 System

This directory contains configuration for the SO-101 hardware setup used by the
SO-101 Gazebo, Genesis, and Isaac Sim adapters.

- `urdf/so101.urdf.xacro`: canonical robot description.
- `urdf/so101.urdf`: generated plain URDF for simulators that do not read Xacro.
- `config/joint_limits.json`: the joint position limits (single source of truth; see below).
- `config/lerobot_joint_mapping.json`: nominal LeRobot-degrees to URDF joint mapping.
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

## Joint Limits

`config/joint_limits.json` is the only place the joint position limits are
written down. The Xacro reads it (`xacro.load_yaml`), so the generated URDF
carries the same values, and every runtime consumer loads it:
`so101_common.py` (Isaac, Genesis, the mock simulator, the Gazebo relay, the
CBF controller, the GR00T bridge and the HIL guard), the GR00T demo/mock policy
and dataset normaliser (`applications/groot/scripts/so101_joint_limits.py`), and
the SmolVLA server's checkpoint range check. Containers find it under
`/workspace/systems` (or `$SO101_JOINT_LIMITS_FILE`).
`validate_calibration_and_dataset.py` and the tests check that both URDF files
match it.

This URDF's zero pose is the arm pointing straight up. The physical range comes
from TheRobotStudio's `Simulation/SO101/so101_new_calib.urdf`, whose zero is
LeRobot's calibrated mid-range pose, mapped through the nominal calibration in
`config/lerobot_joint_mapping.json` (`urdf = sign * new_calib + offset`; when the
sign is −1 the lower and upper limits swap). The values are rounded outward to
1e-5 rad:

| Joint | so101_new_calib (rad) | sign, offset | This URDF (rad) | This URDF (deg) |
| --- | --- | --- | --- | --- |
| shoulder_pan | ±1.91986 | −1, 0° | −1.91986 .. 1.91986 | −110 .. 110 |
| shoulder_lift | ±1.74533 | +1, 14.0° | −1.50099 .. 1.98968 | −86 .. 114 |
| elbow_flex | ±1.69 | +1, 73.9° | −0.40021 .. 2.9798 | −22.9 .. 170.7 |
| wrist_flex | ±1.65806 | +1, 2.1° | −1.62141 .. 1.69472 | −92.9 .. 97.1 |
| wrist_roll | −2.74385 .. 2.84121 | −1, 0° | −2.84121 .. 2.74385 | −162.8 .. 157.2 |
| gripper | revolute −0.174533 .. 1.74533 | not mapped | 0 .. 0.04 m (prismatic jaw) | |

So LeRobot data at the ends of its calibrated range maps inside the URDF
limits, and the simulators cannot reach poses the real arm cannot (e.g. elbow
−100°). If you recalibrate `lerobot_joint_mapping.json` for your arm, re-derive
the limits with the same rule (`tests/test_so101_joint_limits.py` recomputes
them) and regenerate the URDF.

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
