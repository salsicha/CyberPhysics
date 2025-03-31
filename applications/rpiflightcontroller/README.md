
# RPi Drone


TODO:
use velocity error in ros_flight_controller.py
add PID in ros_flight_controller.py with flow and height to estimate forward velocity
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

