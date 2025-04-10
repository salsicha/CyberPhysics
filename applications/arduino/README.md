

Run platformio
- pio run --target upload
Run uros bridge
- ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
Run ROS subscriber
- ros2 topic echo /micro_ros_arduino_node_publisher
UDEV Rules
- wget https://www.pjrc.com/teensy/00-teensy.rules
- sudo cp 00-teensy.rules /etc/udev/rules.d/




# Arduino

This app provides all the necessary things for building Arduino projects and deploying ROS2 enabled sketches.


## TODO:



## Instructions:

Run the arduino.yaml in the compositions folder to see full working example.

Debugging, connecting uros to the device:
serial --dev [YOUR BOARD PORT] -v6

Or if using the arduino CLI:
arduino-cli monitor -p /dev/ttyACM0

(Deprecated, should be handled by the dockerfile now)
Loading the uros library into the Arduino IDE:
Sketch -> Include library -> Add .ZIP Library...
/microrosarduino.zip

(Deprecated, should be handled by the dockerfile now)
Teensy board support:
Copy "https://www.pjrc.com/teensy/package_teensy_index.json" 
Into "File -> Preferences -> Additional boards manager URLs", and click "OK"
Then "Boards Manager (Icon in left menu) -> teensy -> Install"

