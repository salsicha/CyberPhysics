# SO-101 System

This directory contains configuration for the SO-101 hardware setup used by the
SO-101 Gazebo and Isaac Sim adapters.

Hardware (bill of materials, printed parts, printing and assembly):
[hardware/so101.md](../../hardware/so101.md).

- `urdf/so101.urdf.xacro`: canonical robot description.
- `urdf/so101.urdf`: generated plain URDF for simulators that do not read Xacro.
- `config/joint_limits.json`: the joint position limits (single source of truth; see below).
- `config/lerobot_joint_mapping.json`: nominal LeRobot-degrees to URDF joint mapping.
- `config/controllers.yaml`: ROS 2 control configuration for Gazebo.
- `config/groot_demo.yaml`: SO-101 + GR00T bridge parameters for Isaac Sim.
- `worlds/empty.sdf`: minimal Gazebo world for the arm.
- `scenarios/picking_table.json`: the tabletop picking scene and its tasks, the single source of truth for Isaac Sim, Gazebo, GR00T scoring and the printed station (see "Scene" below).
- `worlds/picking_table.sdf`: the scene as a Gazebo world (bins, target objects, clutter, an occluder, calibration board, the camera sensor, lighting, and collision geometry), generated from the scenario by `applications/so101/scripts/scenario_world.py`; do not edit it by hand.
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
`so101_common.py` (Isaac, the mock simulator, the Gazebo relay, the
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

## Scene

`scenarios/picking_table.json` is the only place the scene's poses and
heights are written. Each asset and object names the surface it stands on
(`rests_on`, plus `standoff_m` for the AprilGrid board, which stands in its
printed feet 6 mm above the table), and the robot spawn rests on
`arm_base_mount`: the table top is at 0.400 m, the 31 mm riser under the arm
spans 0.400-0.431 m, the arm's base stands on it (spawn z 0.431 m, the
underside of the URDF's base box), and the bins, objects and occluder stand on
the table. `so101_common.scene_support_errors` checks every such contact and
that nothing interpenetrates; the tests and
`validate_calibration_and_dataset.py` run it, Isaac reports its findings at
start-up, and `scenario_world.py` refuses to write a Gazebo world from an
inconsistent scenario. Isaac (kinematic objects) and Gazebo (free objects)
therefore start the task from the same state, which is also the printed
station's (`hardware/parts/so101/`, derived from this file).

After editing the scenario, regenerate the Gazebo world:

```bash
python3 applications/so101/scripts/scenario_world.py   # --check to verify
```

## GR00T Demo Camera

The SO-101 URDF includes a fixed `groot_camera_link` and
`groot_camera_rgb_optical_frame`. The scenario's `camera` block
(`scenarios/picking_table.json`) is the single source of the camera model:
Isaac and Gazebo render from it, `/so101/camera/camera_info`
publishes its intrinsics and the URDF joint `base_to_groot_camera` must equal
its pose (`validate_calibration_and_dataset.py` checks this).

It models the station's RealSense D435i colour stream: 16:9, 69.4 deg
horizontal and 42.5 deg vertical view, 640x360 at 30 fps (the hardware
compositions select the same mode, `rgb_camera.color_profile:=640x360x30`),
square pixels, fx = fy = 462.1 px.

The same pose is the nominal mount for the physical rig: the camera's RGB
centre sits 0.170 m forward, 0.522 m to the right of and 0.465 m above
`base_link`, pitched 48.5 deg down and yawed 83 deg to look across the table
at the arm (xyz `0.1697563 -0.5219908 0.4646126`, rpy `0 0.8464847 1.449`).
That is where the printed camera head puts it when trimmed to 48.5 deg, the
pitch the station's assembly check recommends: the bins, objects, occluder,
riser and arm base then all lie within 88 % of the half-image (at the old
45 deg the D435i cut off the near edge of the right bin). It is a nominal
value, not a measurement of a particular rig: after mounting a RealSense,
measure its pose (e.g. with an AprilTag on the base) and update both the
scenario and the URDF. The change from the earlier 640x480, 69 deg, 45 deg
camera and what it means for trained policies is described in
`applications/so101/README.md` ("Camera").
