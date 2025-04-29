

Maybe only PWM solution:  
https://abyz.me.uk/rpi/pigpio/ex_motor_shield.html  


Turns out that the RPi 5b can't emit 4 simultaneous PWM signals, even though it has the hardware for it. Maybe it will be supported one day...


I've ordered a PWM hat for the RPi5b  


"as2_platform_rpi" and "project_rpi" are copies of the mavlink platform files, they will be converted to the RPi5b platform  


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



## Edit Raspberry Pi system file:

The Ozzmaker pins conflict with the default PWM pins, new pins must be selected for the 4 PWMS

See ./pwm/README.md for instructions setting up PWM




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

