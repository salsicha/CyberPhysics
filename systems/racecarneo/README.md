# RACECAR Neo System

This system folder is the hardware and simulation configuration home for MIT RACECAR Neo.

Current contents:

- `config/bridge.yaml`: ROS parameters for the official simulator bridge.
- `config/topics.yaml`: topic and frame names used by the bridge.
- `calibration/camera.yaml`: placeholder camera calibration for the simulator.
- `urdf/racecarneo.urdf.xacro`: lightweight frame model for ROS visualization and future Gazebo/Isaac work.

The official MIT Unity simulator assets are downloaded into the `cyberphysics/racecarneo` Docker image at build time. They are not committed here.

A full ROS-native Gazebo model can be added next by extending the URDF with wheel joints, Ackermann steering, gazebo sensor tags, and controller parameters. The official simulator bridge is the initial working model because it matches the current MIT RACECAR Neo student tooling.
