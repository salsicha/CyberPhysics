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
`groot_camera_rgb_optical_frame` mounted above and in front of the arm base. The
Isaac demo publishes a synthetic RGB stream on `/so101/camera/image_raw` with
`frame_id=groot_camera_rgb_optical_frame`, plus `/so101/camera/camera_info`, so
the GR00T bridge can exercise the same ROS image path that a rendered or physical
camera would use.

For production visual policy evaluation, replace the synthetic image publisher
in the Isaac adapter with an Isaac camera render product using the same optical
frame and topics.
