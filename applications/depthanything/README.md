# Depth Anything

This application builds a ROS 2 Jazzy node for Depth Anything V2 metric monocular depth. It is used by the RACECAR Neo stack to turn the simulator RGB camera stream into a metric depth image and optional point cloud.

The Docker image downloads the upstream Depth-Anything-V2 repository and two metric small (`vits`, Apache-2.0) checkpoints during image build, each pinned to a model repo revision and checked by SHA-256: VKITTI (outdoor driving, trained with `max_depth` 80) and Hypersim (indoor, `max_depth` 20). Model weights are not stored in this repository.

## Build

```bash
make -C applications build_depthanything
```

## Topics

Default subscriptions:

- `/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `scale_reference_topic` when set (`sensor_msgs/Image`, `32FC1` metres or `16UC1` millimetres)

Default publications:

- `/depth_anything/depth/image` (`sensor_msgs/Image`, `32FC1`, meters)
- `/depth_anything/points` (`sensor_msgs/PointCloud2`)

## Launch

```bash
docker run --rm --network=host salsicha/cyberphysics:depthanything-latest
```

Or as part of the RACECAR Neo stack:

```bash
docker compose -f compositions/racecarneo.yaml up racecarneo_depthanything
```

Useful launch parameters:

- `image_topic`
- `camera_info_topic`
- `depth_topic`
- `pointcloud_topic`
- `encoder` (`vits` by default; the image has only `vits` checkpoints)
- `metric_dataset` (`vkitti` by default, or `hypersim`)
- `max_depth`: must be the checkpoint's training value, 80 for `vkitti` and
  20 for `hypersim`. The model's output is its sigmoid head times
  `max_depth`, so any other value scales every depth by the ratio (the node
  warns); it is not a range limit.
- `min_depth`
- `publish_pointcloud`
- `pointcloud_stride`
- `scale_reference_*`, `scale_hold_s`: scale alignment, below.

## Scale alignment

With `scale_reference_topic` set to a metric depth image registered to the
colour camera (a RealSense's `/camera/aligned_depth_to_color/image_raw`, a
simulator's `/camera/depth/image_rect_raw`), every published depth is the
model's estimate times the median of reference / estimate over the
reference's valid pixels between `scale_reference_min_m` and
`scale_reference_max_m` (0.3-4 m), sampled every `scale_reference_stride`
(8) pixels; at least `scale_reference_min_samples` (100) are needed. The
reference must carry the colour frame's stamp within
`scale_reference_max_age_s` (0.05 s). Without one the last scale holds for
`scale_hold_s` (1 s); after that the node publishes nothing rather than an
unscaled estimate. Off by default (empty topic).

## Accuracy

`depth_accuracy` (in the image) measures the node's depth against a
camera's measured depth:

```bash
# Record colour, the camera's depth and the running node's estimate, one
# frame per second once the car moves (ROS_DOMAIN_ID of the stack):
docker run --rm --network host -e ROS_DOMAIN_ID -v $PWD/frames:/out \
  salsicha/cyberphysics:depthanything-latest \
  ros2 run depthanything_pkg depth_accuracy record --output-dir /out \
    --depth-topic /camera/depth/image_rect_raw \
    --estimate-topic /depth_anything/depth/image --odom-topic /odom

# Score models on the frames, as depth_node runs them, raw, with one fixed
# scale and with per-frame scale alignment, and time them on this GPU:
docker run --rm --gpus all -v $PWD:/data salsicha/cyberphysics:depthanything-latest \
  ros2 run depthanything_pkg depth_accuracy evaluate --frames /data/frames \
    --model vkitti:vits:80 --model hypersim:vits:20 --output /data/accuracy.json
```

Errors are over the pixels whose reference depth is within `--band`
(0.5-5 m), every second row and column; per-frame alignment uses the same
median ratio as the node (`--align-range` 0.3-4 m, every 8th pixel, an
offset grid not scored). The "fixed" row applies the median of the
per-frame scales to every frame, which is what a recalibrated
`max_depth` would do at best (it is fitted on the same frames).

### Results: the RACECAR's low camera (2026-10-11)

Frames from the two simulated RACECARs, scored against the simulator's own
depth (its rendered depth with the RealSense noise model of
`systems/racecarneo/config/sensor_models.yaml`: 15 mm, 1 % dropout, nothing
beyond 10 m), over 0.5-5 m, the range that matters to Nav2 here:

- **Isaac** (`compositions/racecar_turret_isaac.yaml`, the hardware car's
  geometry: camera 0.22 m above the floor, level, 640x480, fx 617 px): 150
  frames, one per 0.2 m or 10 degrees of a lap of the arena course
  (`applications/isaac/RACECAR_TURRET.md`).
- **RacecarSim** (`compositions/racecarneo.yaml`, Demo World; its floor puts
  the camera 0.12 m above it): 195 frames while circling both ways.

The last two columns, Isaac only (a level camera over a flat floor), say
what each depth would do to nvblox's obstacle slice
(`systems/racecarneo/config/nvblox.yaml`: points 0.02-0.55 m above the floor
are obstacles): the share of floor pixels whose point lands in that band
(phantom obstacles) and of obstacle pixels whose point stays in it within
0.15 m of the true point (obstacles where they are).

| Depth (Isaac, 0.22 m camera) | Estimate / true, median (0.5-1, 3.5-5 m) | Abs. error median / p90 / p95 | Rel. error median / p90 / p95 | Within 25 % | Floor lifted into the slice | Obstacles in place | GPU per frame |
|---|---|---|---|---|---|---|---|
| a) vkitti, max_depth 80 (the setting) | 9.07 (9.7, 5.6) | 9.03 / 18.6 / 21.2 m | 807 / 894 / 909 % | 0.1 % | 0 % | 0.04 % | 48 ms |
| b) hypersim, max_depth 20 | 1.31 (1.42, 0.98) | 0.39 / 0.95 / 1.15 m | 31 / 71 / 81 % | 37 % | 2 % | 3 % | 47 ms |
| vkitti, one fixed scale (x0.107) | 0.97 (1.04, 0.61) | 0.06 / 1.46 / 1.94 m | 7 / 40 / 51 % | 75 % | 26 % | 2 % | 48 ms |
| hypersim, one fixed scale (x0.733) | 0.96 (1.04, 0.71) | 0.16 / 0.87 / 1.47 m | 13 / 34 / 40 % | 72 % | 34 % | 28 % | 47 ms |
| c) vkitti, aligned per frame | 0.99 (1.05, 0.63) | 0.06 / 1.37 / 1.85 m | 7 / 38 / 48 % | 76 % | 25 % | 3 % | 48 ms + 0.4 ms |
| c) hypersim, aligned per frame | 0.99 (1.04, 0.75) | 0.05 / 0.82 / 1.29 m | 6 / 23 / 30 % | 88 % | 18 % | 15 % | 47 ms + 0.4 ms |
| d) the camera's depth | 1 | the sensor model: 15 mm noise | | | 0 % | 100 % | none |

| Depth (RacecarSim, 0.12 m camera) | Estimate / true, median (0.5-1, 3.5-5 m) | Abs. error median / p90 / p95 | Rel. error median / p90 / p95 | Within 25 % |
|---|---|---|---|---|
| a) vkitti, max_depth 80 | 13.0 (14.9, 7.2) | 12.4 / 23.1 / 25.7 m | 1198 / 1473 / 1518 % | 0 % |
| b) hypersim, max_depth 20 | 2.86 (3.28, 1.93) | 2.08 / 4.12 / 4.86 m | 186 / 281 / 304 % | 0.1 % |
| c) vkitti, aligned per frame | 0.89 (1.02, 0.49) | 0.14 / 1.34 / 1.96 m | 14 / 44 / 51 % | 62 % |
| c) hypersim, aligned per frame | 0.94 (1.02, 0.58) | 0.09 / 1.03 / 1.53 m | 9 / 34 / 41 % | 76 % |

- Both metric models are trained on cameras at a person's or a car's height
  and take the low camera's view of the floor for a larger scene: vkitti
  reads 9 times the true depth at 0.22 m and 13 times at 0.12 m, hypersim
  1.3 and 2.9 times. The running node's output matched the offline
  inference exactly (the live row equals vkitti's).
- Scale alignment fixes the scale but not the shape: aligned to the
  camera's depth every frame, near pixels come out right (median error
  3-4 cm under 1 m) and far ones too close (0.63-0.75 of the true depth at
  3.5-5 m in Isaac, 0.49-0.58 in RacecarSim). From a camera this low that
  is fatal for nvblox: a floor point needs its depth within about 9 % for
  its height to stay under the slice's 0.02 m (0.02 / 0.22), so 18-25 % of
  the floor pixels became obstacles, and only 3-15 % of the obstacle
  pixels stayed within 0.15 m of where they are. One fixed scale (a
  recalibrated max_depth) is no better.
- The per-frame scale itself is steady for vkitti (0.104-0.115, p10-p90)
  and wanders for hypersim (0.60-0.87).
- GPU time: `infer_image` per 640x480 frame on an RTX 4000 Ada Laptop GPU
  (the network alone 42 ms), 0.29 GB of PyTorch memory; the same for both
  checkpoints. Alignment adds 0.1 ms (the median) plus 0.3 ms (scaling and
  masking) on the CPU. On the car's Jetson the image's arm64 PyTorch is
  CPU-only (`compositions/racecarneo_hardware.yaml`); not measured.

So the simulator compositions map the camera's own depth with nvblox
(`racecarneo.yaml`, `racecar_turret_isaac.yaml`; Depth Anything still runs
and publishes); with it nvblox maps the arena's obstacles where Depth
Anything's depth left them out (`applications/isaac/RACECAR_TURRET.md`,
"Depth for nvblox").

Reproduce: `depth_accuracy record` alongside `tools/racecar_turret_isaac_test.sh
compensated` (`--odom-topic /odom`) or the RacecarSim stack, then
`depth_accuracy evaluate --camera-height-m 0.22` (Isaac).

### On the car

`compositions/racecarneo_hardware.yaml` and `racecar_turret_hardware.yaml`
still feed nvblox Depth Anything's vkitti depth. The RealSense on the car
sits at the same 0.22 m as the Isaac camera, so a similar error is likely
there (check it with `depth_accuracy record --depth-topic
/camera/aligned_depth_to_color/image_raw` on the car). Options, with the
evidence above:

1. **nvblox from the RealSense depth** (`depth_image_topic:=
   /camera/aligned_depth_to_color/image_raw`, registered to the colour
   camera, so the colour camera_info stays), Depth Anything optional. The
   D435i's datasheet error is under 2 % at 2 m (the D455's at 4 m), inside
   the 9 % a floor point can afford at this height. Without Depth Anything
   the GPU time it takes is free (48 ms per frame on the laptop GPU above;
   not measured on the Orin Nano, where the image's PyTorch is CPU-only
   today). nvblox reads 16-bit millimetre depth as Isaac ROS's RealSense
   examples feed it; not tried on this composition.
2. **Depth Anything hypersim with scale alignment to the RealSense**
   (`metric_dataset:=hypersim max_depth:=20.0
   scale_reference_topic:=/camera/aligned_depth_to_color/image_raw`): the
   best Depth Anything setting measured here, but 18 % of floor pixels
   still land in the obstacle slice and obstacles 3.5-5 m away come out
   25 % too close, and it needs the RealSense anyway.
