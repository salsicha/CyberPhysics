# Drone behavior tree demo

This is a demonstration of using Behavior Trees to make an Ardupilot quadcopter autonomous.  

Currently the GPS coordinates are hard coded, and the map image has been pre-downloaded.  


INSTALL:  
docker build -t drone_demo .  

Note: set ROS_IP to the publisher's IP  

START:  
docker run -it --rm --net=host --privileged --ipc host drone  

Which should run ./scripts/behaviortree.sh  

Load the web interface here:  
http://localhost:8080  
