# SO-101 System

This directory contains configuration for the SO-101 hardware setup used by the
SO-101 Gazebo, Genesis, and Isaac Sim adapters.

- `urdf/so101.urdf.xacro`: canonical robot description.
- `urdf/so101.urdf`: generated plain URDF for simulators that do not read Xacro.
- `config/controllers.yaml`: ROS 2 control configuration for Gazebo.
- `worlds/empty.sdf`: minimal Gazebo world for the arm.

Regenerate the plain URDF after editing the Xacro:

```bash
applications/so101/scripts/regenerate_urdf.sh
```
