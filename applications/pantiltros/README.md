
# Pan Tilt ROS

This app serves as a demonstration for a turret.

See original project:
https://github.com/adityakamath/pan_tilt_ros


## Turret Simulation

The headless turret tracking simulation lives in `systems/turret` and can be
run through `compositions/turret_sim.yaml`. It preserves the PanTiltROS
`pan_joint` and `tilt_joint` command contract while validating camera, YOLO
segmentation, target lock, joint-state, diagnostics, and emergency-stop
telemetry.

## Arduino Hardware Bridge

`compositions/turret_hardware.yaml` runs the PanTiltROS-compatible Arduino
bridge from this image. The bridge subscribes to the turret joint command topic,
forwards compact serial commands to an Arduino UNO R4 WiFi, and republishes
joint-state/diagnostic telemetry for the rest of the ROS graph.
