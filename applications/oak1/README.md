
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

The model is tracked with Git LFS. Run `git lfs pull` before building; the
Docker build rejects the pointer file if the binary has not been fetched.

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

This script should do object tracking
scripts/objecttracking.py


## Running in container:
-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'
