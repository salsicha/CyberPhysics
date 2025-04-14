
# RPi Drone


TODO:
calculate homographies between frames 
use drone_stabilization.py's method, don't needs to keep tracks
but either optical_flow_mono.py or featuretracker.py have something similar but with hardware acceleration
publish transformation and subscribe in flight controller
add transformation to PID control in flight controller
add keyboard command subscriber to ros_flight_controller.py
get aerostack2 genesis sim working
enable mode switch, stabilize with INS, use Oak for MegaDepth or people tracking



Types:

Input to flight controller
- input_throttle
- input_roll
- input_pitch
- input_yaw
- adj_throttle

Output from AS2 PID speed controller
- x velocity
- y velocity
- z velocity
- yaw speed

Ozzmaker IMU filter

Keyboard output
- ControlValues.SPEED_VALUE
- ControlValues.VERTICAL_VALUE
- ControlValues.TURN_SPEED_VALUE
- ControlValues.POSITION_VALUE
- ControlValues.ALTITUDE_VALUE
- ControlValues.TURN_ANGLE_VALUE

