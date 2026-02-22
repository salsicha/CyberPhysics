# Drone behavior tree demo

This is a demonstration of using Behavior Trees to make an Ardupilot quadcopter autonomous.  

The application supports automatic map loading and GPS configuration based on your current location.


INSTALL:  
docker build -t drone_demo .  

Note: set ROS_IP to the publisher's IP  

START:  
1. Configure the mission based on your location:
   python3 scripts/configure_mission.py

2. Run the container:
   docker run -it --rm --net=host --privileged --ipc host -v $(pwd)/mission_config.json:/app/mission_config.json -v $(pwd)/map.png:/app/map.png drone_demo  

Which should run ./scripts/behaviortree.sh  

Load the web interface here:  
http://localhost:8080  
