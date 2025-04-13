



## TODO:


Please install `99-platformio-udev.rules`. 
https://docs.platformio.org/en/latest/core/installation/udev-rules.html




## Example

Raspberry Pi Pico 2 W:
(Pico appears as a storage device, drag and drop this file)
https://downloads.raspberrypi.com/micropython/mp_firmware_unofficial_latest.uf2

Teensy:
Must hit program button before running "pio run upload"

docker run -ti --privileged cyberphysics/arduino bash -c "cd Sub && pio run -e pico --target upload"

docker run -ti -v /dev/shm:/dev/shm --privileged cyberphysics/ros2 bash -c "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v 6"

docker run -ti -v /dev/shm:/dev/shm --privileged cyberphysics/ros2 bash -c "ros2 topic pub /micro_ros_arduino_subscriber std_msgs/msg/Int32 'data: 1' -1"



## Example:

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

