# VINS-Mono example system

`calibration/vins_mono.yaml` is an example pinhole camera and IMU calibration
for the VINS-Mono composition. The values make the container runnable, but they
are not a substitute for calibrating the camera, the IMU noise model, and the
camera-to-IMU transform on the target sensor.

The default ROS inputs are:

- camera images: `/camera/image_raw` (`sensor_msgs/msg/Image`)
- IMU samples: `/camera/imu` (`sensor_msgs/msg/Imu`)

Edit the topic names and calibrated values in the system YAML before using the
odometry output for navigation. VINS-Mono publishes under `/vins_estimator`;
the primary odometry topic is `/vins_estimator/odometry`.
