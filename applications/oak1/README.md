
https://docs.luxonis.com/software/depthai/examples/feature_tracker/


# Manual quickstart:
source ~/venv/bin/activate
./scripts/install_deps.sh
python3 scripts/featuretracker.py
or:
depthai_viewer


# Oak-1 Camera ROS2 Driver

Publish Oak-1 camera images via ros2 msgs and utilize compute from camera 

## Camera config

The camera's default IP address is `169.254.1.222`

Run the config script to configure camera if needed
```
python3 config.py
```

## Pre-steps:

Ensure static IP is set to the correct subnet and specify the device IP in `deployment/oak1_driver.yml` using the device_ip arguement as shown below:
```
python3 driver.py 169.254.1.222
```

** If going to run a custom script on device use the following arguement as shown below:
```
python3 driver.py 169.254.1.222 custom_script.py   # make sure to edit the 'custom_script.py' file with your custom script
```

Build the oak1_driver and humble_base image using the command: 
```
make build_oak1_driver
```

## How to run:

1. Connect the Oak-1 camera to Linux machine via POE

2. Allow access to host machine display
```
xhost +
```

3. Run the compose file with the command: 
```
docker compose -f deployment/oak1_driver.yml up --remove-orphans
```

## Expected output:

Published image topic:
```
/oak1_driver/image_raw
```

## MegaDepth

The repository includes an OAK-1-compatible MegaDepth blob:

```
models/megadepth_192x256_openvino_2021.4_6shave.blob
```

The container also includes `scripts/models/megadepth_6.blob`. Run inference
on the camera's Myriad X and publish the unscaled network output:

```bash
python3 /scripts/megadepth_ros.py --ros-args \
  -p depth_topic:=/oak1/relative_depth
```

The output is deliberately published as relative `32FC1` depth. DemNav is
responsible for estimating its metric affine scale from terrain.

The same node triggers a separate 1280x720 still capture at 0.5 Hz for visual
geolocation:

```text
/oak1/image_highres
/oak1/image_highres/camera_info
```

Using triggered stills avoids continuously transporting high-resolution video.
The capture dimensions and rate are ROS parameters.

MegaDepth's input queue holds one frame (`megadepth_queue_size`, 1). With
DepthAI's default queue the camera stalled on MegaDepth: every stream ran at
MegaDepth's rate, the stills at 0.26 Hz instead of 0.5 Hz, and a depth frame
left the camera 1.3 s after its capture. With one queued frame the stills
keep their rate and depth frames are 0.5 s old (measured on an OAK-1, below);
MegaDepth's own rate is unchanged or better (4.0-4.6 Hz before, 4.6 Hz
after). Its topics are still stamped when the host receives them.

The model is tracked with Git LFS. Run `git lfs pull` before building; the
Docker build rejects the pointer file if the binary has not been fetched.

## Object tracking

The aerodrone's object-tracking detector (`systems/aerodrone/tracking.md`)
runs on the same OAK-1 and the same Myriad X as MegaDepth, in the same node
(`megadepth_ros.py`, which owns the device) with `tracking_enabled:=true`;
`compositions/blueos_nav_core.yaml` turns it on unless `OAK1_TRACKING=false`
(the airplane's `blueos.env`). It publishes, under `tracking_topic_prefix`
(`TRACKING_TOPIC_PREFIX`, `/tracking`):

| Topic | Content |
|---|---|
| `detections` | `vision_msgs/Detection2DArray`: the ObjectTracker's NEW and TRACKED tracklets, `id` the track ID, box in pixels of the 640x352 image, class name and detection score |
| `image/compressed` | the frame they come from, JPEG-encoded on the camera |
| `camera_info` | its calibrated intrinsics (`readCalibration`, cropped and scaled like the image) |

all three with the frame's capture time (the device clock converted to ROS
time), so the tracker geolocates with the attitude of that instant.

The detector runs on demand (`tracking_on_demand`, true): only while
something subscribes to those topics (the tracker of the `tracking` profile,
the web page through rosbridge). Otherwise the node pauses it on the camera
and MegaDepth gets its whole rate back, within a second either way.

### Pipeline (`scripts/oak1_pipeline.py`)

```text
ColorCamera THE_1080_P, 20 fps
├─ preview 256x192 ─ rate limit (3 Hz with tracking) ─ MegaDepth, 2 threads ─ camera, depth
├─ still 1280x720, triggered at 0.5 Hz ───────────────────────────────────── highres
└─ video 1920x1080 ─ rate limit (5 Hz) ─ ImageManip: centre 1920x1056 → 640x352 NV12
      ├─ VideoEncoder MJPEG ───────────────────────────────────────────────── tracking_jpeg
      └─ ImageManip → BGR planar ─ YoloDetectionNetwork, 1 thread ─ ObjectTracker ─ tracklets
```

The rate limits are Script nodes on the camera's Leon CPU passing every
n-th frame; the host sets their rates over XLink (`tracking_rate`,
`megadepth_rate`) when the demand changes. The ObjectTracker tracks on the
frames the detector saw (SHORT_TERM_IMAGELESS, unique IDs, one track across
class changes, a lost tracklet keeps its ID for `tracking_max_lost_s`, 2 s);
the host pairs each tracklets message with the JPEG of the same camera
frame by sequence number.

### Rates on the Myriad X (measured)

MegaDepth and YOLO share the Myriad X's two neural compute engines; an
inference holds one engine. Measured on an OAK-1 (IMX214, MxId
18443010317F680F00, USB 2 high speed, depthai 2.33.0.0) feeding each blob
over XLink (one thread per network, 6 SHAVEs):

| Network | Input | One inference | One thread | Two threads |
|---|---|---|---|---|
| MegaDepth | 256x192 | 396 ms | 2.6 Hz | 4.6 Hz |
| YOLOv8n (VisDrone or COCO) | 640x352 | 96-115 ms | 14 Hz | 25 Hz |
| YOLOv8n-seg (COCO) | 640x352 | 165 ms | 11 Hz | 15 Hz |

MegaDepth alone fills both engines, so any detector rate comes out of
MegaDepth's. The node's topics, measured through ROS on the same camera:

| Configuration | `/oak1/relative_depth` | `/oak1/image_highres` | `/tracking/*` | depth frame capture to host |
|---|---|---|---|---|
| before (DepthAI default queue, no tracking) | 4.0 Hz | 0.26 Hz | - | 1.3 s |
| tracking off | 4.6 Hz | 0.50 Hz | - | 0.5 s |
| tracking on, nothing subscribed (paused) | 4.5 Hz | 0.50 Hz | - | 0.5 s |
| tracking on and subscribed (defaults: detector 5 Hz, MegaDepth capped at 3 Hz) | 2.9 Hz | 0.50 Hz | 5.0 Hz, 0.16 s after capture | 0.5 s |

Other splits (bench, same camera): MegaDepth uncapped with the detector at
5 Hz gives MegaDepth 4.0 Hz but the detector only 1.4 Hz (MegaDepth's two
threads keep both engines busy); MegaDepth capped at 3.5 Hz gives 3.3 Hz and
the detector 4.3 Hz; MegaDepth on one thread gives 2.2 Hz with the detector
at 5 Hz. `tracking_rate_hz` (`OAK1_TRACKING_RATE_HZ`) and
`megadepth_rate_hz` move the split. With tracking the device uses 157 of
its 333 MiB DDR (137 without), its Leon CPUs 18 % (OS) and 30 % (RT), and
CMX is fully allocated as with MegaDepth alone; the MJPEG frames are about
21 kB.

### The network

YOLOv8n fine-tuned on VisDrone2019-DET (drone imagery) by DetectionBench
([dronefreak/visdrone-yolov8n](https://huggingface.co/dronefreak/visdrone-yolov8n)),
exported for DepthAI's anchor-free YOLO decoder at 640x352 and compiled for
the Myriad X with OpenVINO 2021.4 and 6 SHAVEs: `models/visdrone_yolov8n_640x352*`
(`TRACKING_DETECTOR_MODEL` in `systems/aerodrone/config/topics.env`). Its
licences: AGPL-3.0 (Ultralytics) and, through VisDrone, non-commercial use
only (`THIRD_PARTY_NOTICES.md`). `models/export_yolo.py` exports any
Ultralytics YOLOv8 detector the same way (ONNX, blob and a json with the
classes and provenance).

COCO-trained YOLOv8n is blind to cars seen from straight above. Vehicle
detection on 302 nadir orthophotos with 993 annotated vehicles (VEDAI
1024, 12.5 cm/px, research use), rescaled to the OAK-1 tracking image's
ground sampling at each height above ground and cut into 640x352 windows,
IoU 0.5, confidence 0.3:

| Network | 30 m (vehicles ~69 px) | 45 m (~46 px) | 60 m (~34 px) |
|---|---|---|---|
| YOLOv8n COCO (Ultralytics) | recall 0.02, precision 0.67, AP50 0.04 | 0.01, 0.43, 0.02 | 0.01, 1.00, 0.02 |
| YOLOv8n VisDrone (dronefreak) | 0.66, 0.69, 0.66 | 0.63, 0.76, 0.65 | 0.61, 0.78, 0.62 |
| **the same at confidence 0.2 (chosen)** | **0.71, 0.62** | **0.69, 0.71** | **0.66, 0.73** |
| YOLOv8n VisDrone (mshamrai, OpenRAIL) | 0.60, 0.44, 0.49 | 0.69, 0.63, 0.63 | 0.67, 0.68, 0.63 |

(VEDAI's 12.5 cm/px is what the OAK-1 sees from 61 m; nearer heights are
upsampled, so they understate what the camera resolves there. VisDrone's
own validation set could not be fetched: its Google Drive copy is over its
download quota.)

On Gazebo's nadir renders of the tracking targets (a car and two people,
100 frames hovering at each height; ground truth projected from
/simulation/targets; a detection counts when its centre is within the truth
box grown by half its size, at least 8 px), recall and precision of single
frames:

| Network | 30 m: people (~11 px) / car (~85 px) | 45 m: ~7 px / ~56 px | 60 m: ~6 px / ~43 px |
|---|---|---|---|
| YOLOv8n COCO, 0.3 | 0.02, 1.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 |
| YOLOv8n VisDrone (dronefreak), 0.3 | 0.36, 0.91 / 1.00, 0.98 | 0.73, 0.99 / 1.00, 1.00 | 0.69, 1.00 / 0.68, 0.83 |
| **the same at 0.2 (chosen)** | **0.70, 0.90 / 1.00, 0.98** | **0.88, 0.92 / 1.00, 1.00** | **0.80, 1.00 / 0.77, 0.69** |
| YOLOv8n VisDrone (mshamrai), 0.3 | 0.84, 0.77 / 0.99, 0.99 | 0.84, 0.87 / 1.00, 1.00 | 0.79, 1.00 / 1.00, 1.00 |

The VisDrone models see what COCO's cannot. dronefreak's is chosen for the
real imagery, where it is the more precise at the same recall, and for its
documented training (1280 px, 100 epochs, VisDrone test mAP50 0.40); the
renders, a uniform terrain texture and three targets, favour neither
clearly. A confidence of 0.2 (`tracking_confidence`, the simulator
detector's `confidence`) keeps people from 30 m.

YOLOv8n-seg fits on the Myriad X but costs 1.3-1.7 times the detector's
engine time (throughput, latency), its 32x88x160 mask prototypes (0.9 MB a
frame) would cross USB to the Pi for decoding, DepthAI's ObjectTracker
takes only boxes, and no aerial segmentation weights exist: the boxes the
UI draws need none of it.

### Simulators

`scripts/sim_tracking_detector.py` (service `tracking_detector`,
`compositions/aerodrone_tracking_sim.yaml`, profile `tracking`) runs the ONNX
the blob is compiled from on the simulated downward camera with ONNX
Runtime on the CPU, decoding it as the camera does (`scripts/tracking_yolo.py`;
measured against the blob on the OAK-1: same boxes to IoU 0.99 and scores to
0.01), and gives IDs with a host-side ByteTrack-style tracker
(`scripts/host_tracker.py`: Kalman-predicted IoU matching, weak detections
carrying confirmed tracks, centre-distance recovery of small fast objects
and of objects the camera's motion threw off their prediction, only
between boxes of comparable size, and, like the ObjectTracker's occlusion
ratio, no second track on one object). A 640x480 frame is covered by two overlapping 640x352 windows, so
the simulators see as many pixels per metre as the aircraft. It publishes
the same three topics with the camera image's stamp; its image is the
simulator's full 640x480 frame.

Live in the Gazebo stack (`--profile tracking`, the `tracking_detector`
service; 40 s hovering at each height over the car lapping its circuit, a
walking and a standing person): `/tracking/detections` at 5.0 Hz, the
camera's rate, 50-55 ms of CPU per frame; published tracks against the
truth, recall / precision and the track IDs each target had:

| Height | People | Car | IDs (car; walking person; standing person) |
|---|---|---|---|
| 30 m | 0.84 / 0.82 | 1.00 / 0.99 | 3 (it left the view twice); 3; 5 |
| 45 m | 0.98 / 0.93 | 1.00 / 1.00 | 1; 2; 1 |
| 60 m | 0.91 / 1.00 | 0.82 / 1.00 | 1; 2; 1 |

From 30 m the detector sometimes boxes a person twice, a few pixels apart
(body and shadow), and the two boxes took turns on the person's ID; the
tracker now starts no track on top of another (its occlusion ratio, as
DepthAI's). Replayed on the recorded frames, that leaves the standing
person one ID at every height and the walking person one or two.

In flight (`tools/tracking_sim_test.sh gazebo`, the aircraft following the
car), the body-fixed camera moves the image by up to about 100 px a frame
when the attitude changes, and the car's box often missed its prediction.
Replayed on 90 s of those frames (418 with the car in view), the car had 37
IDs with the tracker above, 22 once a box could only be recovered by a
track of comparable size (the car's box had taken over a person's track
beside it, and the reverse), 9 with recovery from the track's last box as
well as its prediction, within 3 box sizes for a track seen within the last
0.5 s (1.5 for one gone longer: a hidden person's track had reached a
person standing 6 m away), and 4 with every association needing the
detection's longer side within 2 times the track's last box's (a car's
prediction, shrinking at the image edge, had taken people's boxes); recall
0.92 to 0.98. The aircraft's tracker hands its target over across the
changes left (`systems/aerodrone/tracking.md`, "Track handover").

ONNX Runtime runs without busy-waiting between frames: 0.9 instead of 3.2
cores at 5 Hz, for 59 instead of 47 ms a frame (4 threads, outside the
stack). In the test stacks the detector took 2.3-2.4 cores in Gazebo (5 Hz,
71-79 ms a frame), 1.9 in Genesis (10 Hz of simulation time, 4.6 Hz of
wall time, 54 ms) and 1.0 in Isaac (10 Hz, 2.0 Hz of wall time, 75 ms),
with the JPEG encoding, scaling and publishing.

### To verify on the aircraft's OAK-1

The measurements above are from a bench OAK-1 on USB 2 pointed at a room;
on the aircraft check:

- the rates of `/oak1/relative_depth`, `/tracking/detections` and
  `/oak1/image_highres` with the Pi 5 as host over USB 3 (`ros2 topic hz`),
  with and without a subscriber to `/tracking/detections`, and that DemNav
  still matches at the lower MegaDepth rate while tracking. DemNav matches
  only where the depth footprint spans 5 DEM cells (`min_dem_footprint_pixels`):
  with MegaDepth's 256x192 at 69 deg and the aircraft's 30 m DEM that is
  about 145 m above ground, above tracking's 15-120 m, so the lower rate
  matters when the detector runs (a page or tracker subscribed) during a
  GPS-denied cruise. In Gazebo's GPS-denied test (`tools/gps_denied_sim_test.sh
  gazebo`, cruise 250 m) with the detector and tracker running and DemNav's
  depth relayed at 2.86 Hz instead of 5, DemNav made 293 fixes while GPS 1
  was denied (401-468 at 5 Hz in `tools/gps_denied_sim_test.md`'s latest
  Gazebo runs), 7.2 / 18.9 / 21.5 m from the truth (median / p95 / max;
  there 4.4-10.8 / 10.6-19.0 / 13.2-28.8), and the fused fix was 9.4 /
  23.3 / 25.7 m (7.6-13.0 / 13.2-40.5 / 13.9-49.7), 6.7 % beyond 3 sigma
  (0-7.9 %): within those runs' spread, from one run; the test passed. On
  the aircraft: fly the cruise with the page open and closed (a
  `/tracking/detections` subscriber pauses or resumes the detector) and
  compare `/demnav/fix` and `/demnav/valid` rates and the fixes against GPS.
- the sensor (`IMX378` or `IMX214`) and calibration: `camera_info` is
  computed from `readCalibration()` cropped like each stream, right when the
  calibration is at a 16:9 resolution as on the bench unit (3840x2160);
- detections from 30-60 m over people and cars, the IDs through a pass, and
  the frame stamps against the flight controller's attitude.

## Reference:

https://github.com/luxonis/depthai-python


## Dependencies:
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash


## DepthAI Viewer:
python3 -m pip install depthai-viewer
python3 -m depthai_viewer


## TODO:

This script should do optical flow for motion tracking on the MDK drone
scripts/featuretracker.py

## Running in container:
-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'
