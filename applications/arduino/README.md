

## Walkthrough:

(This uses Raspberry Pi Pico 2 W)  

1. 
cd Sub/ && pio run -e pico --target upload --upload-port /dev/ttyUSB0  

2. 
Unplug and reconnect pico  

3. 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200  

4. 
ros2 topic pub /micro_ros_arduino_subscriber std_msgs/msg/Int32 "data: 1" -1  


## Note on accessing devices

(The install.sh script does this)  
sudo usermod -a -G dialout $USERNAME  

OR  

(If I switch to the non-daemon/root version of docker)  
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules  
sudo service udev restart  

