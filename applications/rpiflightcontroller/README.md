
# RPi Drone


TODO:
get aerostack2 gazebo sim working
flight controller needs to be able to hover mode, this requires a PID
convert drone stabilization script to use Oak1 hardware
use velocity error in python script
drone will only move forward
pitch will control forward velocity
PID in python script will adjust pitch to get desired velocity
get aerostack2 genesis sim working




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

