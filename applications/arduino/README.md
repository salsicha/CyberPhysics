
# Arduino

This app provides all the necessary things for building Arduino projects and deploying ROS2 enabled sketches.


## Instructions:

Connecting uros to the device:
serial --dev [YOUR BOARD PORT] -v6

(Deprecated, should be handled by the dockerfile now)
Loading the uros library into the Arduino IDE:
Sketch -> Include library -> Add .ZIP Library...
/microrosarduino.zip

(Deprecated, should be handled by the dockerfile now)
Teensy board support:
Copy "https://www.pjrc.com/teensy/package_teensy_index.json" 
Into "File -> Preferences -> Additional boards manager URLs", and click "OK"
Then "Boards Manager (Icon in left menu) -> teensy -> Install"

