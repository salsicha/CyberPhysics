
# RPi Drone


TODO:
aerostack2 keyboard interface  
- as2 compiles the platform with the framework
- and uses OOP to move data around
- I can just subscribe directly to the topics from the keyboard publisher
- as2_core/include/as2_core/names/
- takeoff service: "platform_takeoff", takeoff action: "TakeoffBehavior" ...
- I can just reproduce the services in my flight controller
Use the as2 PID and motion control nodes?
rangefider drive  
add translation to PID control in flight controller  
add keyboard command subscriber to ros_flight_controller.py  
enable mode switch  
stabilize with INS  
MegaDepth or people tracking  
GUI  
Make full c++ version in as2?


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

