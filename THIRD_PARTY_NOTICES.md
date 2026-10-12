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
| `applications/oak1/models/visdrone_yolov8n_640x352.onnx`, `applications/oak1/models/visdrone_yolov8n_640x352_openvino_2021.4_6shave.blob` | The aerodrone's tracking detector: [Ultralytics YOLOv8n](https://github.com/ultralytics/ultralytics) fine-tuned on VisDrone2019-DET by [DetectionBench](https://github.com/dronefreak/DetectionBench) ([dronefreak/visdrone-yolov8n](https://huggingface.co/dronefreak/visdrone-yolov8n), revision `008d286`, `best.pt` SHA-256 `060866322c84…`), exported by `applications/oak1/models/export_yolo.py` | AGPL-3.0 (the weights, as Ultralytics'). The training data, [VisDrone2019-DET](https://github.com/VisDrone/VisDrone-Dataset), is CC BY-NC-SA 3.0, which its model card applies to the model: non-commercial use only. |
| `applications/oak1/scripts/models/yolov8n_coco_640x352_*.blob` | [Ultralytics YOLOv8n](https://github.com/ultralytics/ultralytics) trained on COCO, compiled for the Myriad X | AGPL-3.0 (Ultralytics' weights). |

Because GPL-3.0 and AGPL-3.0 code is included, distributing a combined work
(for example a Docker image containing VINS-Mono or whereami) must follow those
licenses for that part. The oak1 image carries the tracking detector above:
AGPL-3.0, and non-commercial through its VisDrone training data.

## Assets downloaded at run time

The Aerodrone simulations' tracking targets (`applications/aerostack2/missions/tracking_targets.json`)
are 3D models from [Gazebo Fuel](https://app.gazebosim.org/fuel/models). They are not
committed: the compositions' `target_assets` service downloads each pinned version,
checks its SHA-256 and that Fuel still lists the licence below
(`applications/demnav/scripts/target_models.py`), and the terrain builder converts it
into the simulators' asset volume, with an `ATTRIBUTION.txt` beside each model.

| Model | Version | Author | License |
|---|---|---|---|
| [Hatchback blue](https://app.gazebosim.org/OpenRobotics/fuel/models/Hatchback%20blue) | 5 | Open Robotics | CC0-1.0 (Creative Commons Zero v1.0 Universal) |
| [Walking person](https://app.gazebosim.org/OpenRobotics/fuel/models/Walking%20person) | 3 | Open Robotics (made with MakeHuman) | CC0-1.0 (Creative Commons Zero v1.0 Universal) |
| [Casual female](https://app.gazebosim.org/OpenRobotics/fuel/models/Casual%20female) | 4 | Open Robotics (made with MakeHuman) | CC0-1.0 (Creative Commons Zero v1.0 Universal) |

## Vendored 3D-printable designs

These published hardware designs are committed byte for byte under
`hardware/parts/<platform>/upstream/`, with their licence beside them, and
pinned by commit and SHA-256 in the part modules
(`hardware/parts/README.md`, "Vendored parts"). The build turns each mesh
into a print orientation and writes it to `hardware/parts/<platform>/stl/`
(a modified copy: the STL header says so); the geometry is unchanged.

| Path | Design | License |
|---|---|---|
| `hardware/parts/so101/upstream/*.stl` (14 SO-101 follower and leader parts, two STS3215 fit gauges) and the rotated copies in `hardware/parts/so101/stl/*_so101.stl`, `gauge_*.stl` | [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) Standard Open SO-100 & SO-101 Arms by TheRobotStudio with Hugging Face (R. Knight, P. Kooijmans, R. Cadene, S. Alibert et al., per its `CITATION.cff`), commit `a758567c3978dfeefe282ede0500085a48fe8f78`, `STL/SO101/Individual/` and `STL/Gauges/` | Apache-2.0 (`hardware/parts/so101/upstream/LICENSE`, the repository's licence file at that commit) |
