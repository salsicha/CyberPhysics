# Third-Party Notices

The MIT license in `LICENSE` covers this repository's own code. The components
below are third-party code or binaries committed to the repository; each stays
under its own license, and using or redistributing them is subject to those
terms. Software that Dockerfiles download at build time is not listed here;
its license is the one published by its upstream.

| Path | Component | License |
|---|---|---|
| `applications/vinsmono/vins_mono_ros2/` | ROS 2 port of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) (HKUST Aerial Robotics Group) | GPL-3.0. `vins_estimator` and `pose_graph` declare `GPL-3.0-only`; `feature_tracker`, `camera_model` and `config_pkg` leave the declaration as `TODO` and are distributed with the rest of VINS-Mono. |
| `applications/whereami/whereami/` | [whereami](https://github.com/kootenpv/whereami) by Pascal van Kooten | AGPL-3.0 per its `LICENSE` file (its `setup.py` says MIT; the `LICENSE` file is kept as shipped). |
| `applications/blackfly/src/spinnaker_camera_driver/`, `applications/blackfly/src/flir_camera_description/` | [flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver) | Apache-2.0 (see the package `LICENSE`). |
| `applications/ros2/custom_msgs/flir_camera_msgs/` | flir_camera_msgs from flir_camera_driver | Apache-2.0 (see the package `LICENSE`). |
| `applications/blackfly/files/spinnaker-*.tar.gz` | FLIR Spinnaker SDK | FLIR/Teledyne Spinnaker SDK EULA, which restricts redistribution; check it before sharing this repository or images built from it. |
| `applications/matlab/ftb_sim/` | MATLAB/Simulink simulation models | No license or provenance information is included. |
| `applications/arduino/mp_firmware_unofficial_latest.uf2` | Prebuilt microcontroller firmware | No license or provenance information is included. |

Because GPL-3.0 and AGPL-3.0 code is included, distributing a combined work
(for example a Docker image containing VINS-Mono or whereami) must follow those
licenses for that part.
