# Oak-1 Camera ROS2 Driver

Publish Oak-1 camera images via ros2 msgs and utilize compute from camera 

## Camera config

The camera's default IP address is `169.254.1.222`

Run the config script to configure camera if needed
```
python3 config.py
```

## Pre-steps:

Ensure static IP is set to the correct subnet and specify the device IP in `deployment/oak1_driver.yml` using the device_ip arguement as shown below:
```
python3 driver.py 169.254.1.222
```

** If going to run a custom script on device use the following arguement as shown below:
```
python3 driver.py 169.254.1.222 custom_script.py   # make sure to edit the 'custom_script.py' file with your custom script
```

Build the oak1_driver and humble_base image using the command: 
```
make build_oak1_driver
```

## How to run:

1. Connect the Oak-1 camera to Linux machine via POE

2. Allow access to host machine display
```
xhost +
```

3. Run the compose file with the command: 
```
docker compose -f deployment/oak1_driver.yml up --remove-orphans
```

## Expected output:

Published image topic:
```
/oak1_driver/image_raw
```
## Reference:

https://github.com/luxonis/depthai-python