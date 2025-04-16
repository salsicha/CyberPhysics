
# RPi Drone


TODO:
subscribe to "flow_pub" topic in rpiflightcontroller/scout/ros_flight_controller.py
rangefider drive
add translation to PID control in flight controller
add keyboard command subscriber to ros_flight_controller.py
enable mode switch
stabilize with INS
MegaDepth or people tracking
GUI


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

