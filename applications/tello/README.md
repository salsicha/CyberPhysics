
# Tello

This app serves as a demonstration of a project for controlling a DJI Tello with ROS2.

It builds [salsicha/tello-ros2](https://github.com/salsicha/tello-ros2), a
fork of [tentone/tello-ros2](https://github.com/tentone/tello-ros2), at a
pinned commit (`TELLO_ROS2_COMMIT` in the Dockerfile). The fork has three
packages:

- `tello`: the driver node, built on DJITelloPy (`djitellopy2==2.3`, the version
  the fork's install script names).
- `tello_msg`: the `TelloStatus`, `TelloID` and `TelloWifiConfig` messages.
- `tello_control`: keyboard teleop through an OpenCV window.

The image runs `tello.launch.py`. It starts the driver and a static
`map -> drone` transform, without the rqt/rviz GUIs that the fork's own
`launch.py` opens.

## Connect

1. Power on the Tello. Join this computer to the drone's WiFi access point,
   `TELLO-XXXXXX` (no password). The drone is then at 192.168.10.1. The driver
   always uses that address: in the pinned fork the `tello_ip` parameter has
   no effect.
2. Build the image (`cd applications && make build_tello`), then from the
   repository root run:

   ```bash
   ./start.sh tello
   ```

The driver talks UDP with the drone: commands go to 192.168.10.1:8889, and it
receives state on port 8890 and H.264 video on port 11111. The stack therefore
uses host networking, and those ports must be free on the host. When the
connection works the log shows `Tello: Driver node ready`.

If no drone answers, the driver tries the SDK handshake three times, 7 s each.
It then logs `Tello: no connection to the drone (...)` and exits. The launch
file restarts it 5 s later, so the stack keeps retrying until you join the
WiFi. Without any route to 192.168.10.1 the error is `Network is unreachable`
and the retries come faster.

## Keyboard teleop

`tello_control` opens a small blank "Tello" window. Keys typed into that
window are sent to the driver. It needs your X display:

```bash
xhost +SI:localuser:root
./start.sh tello --profile teleop
```

| Key | Action |
| --- | --- |
| T / L | take off / land |
| E | emergency stop (motors off at once) |
| F | flip forward |
| arrows | move left/right/forward/back |
| W / S | up / down |
| A / D | yaw left / right |
| any other key, e.g. space | zero velocity (stop moving) |

## Topics

Most data topics are only published while something subscribes to them.

| Topic | Type |
| --- | --- |
| `/image_raw` | sensor_msgs/Image (bgr8, 960x720) |
| `/imu`, `/odom` | sensor_msgs/Imu, nav_msgs/Odometry (orientation and speed only) |
| `/battery`, `/temperature`, `/status`, `/id` | BatteryState, Temperature, tello_msg/TelloStatus, tello_msg/TelloID |
| `/takeoff`, `/land`, `/emergency` | std_msgs/Empty (subscribed) |
| `/control` | geometry_msgs/Twist, values -100..100 (subscribed) |
| `/flip` | std_msgs/String `l`/`r`/`f`/`b` (subscribed) |

For example, `ros2 topic pub --once /takeoff std_msgs/msg/Empty`. To view
the camera and topics, run `./start.sh foxglove` alongside.

Known issues in the pinned fork:

- `/camera_info` fails: the driver reads the calibration YAML as attributes
  and uses ROS 1 field names. Subscribing to it stops the status thread, so
  do not subscribe to it.
- The driver ignores the `tello_ip` parameter.
