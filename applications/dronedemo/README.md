# lamp_autonomy

INSTALL:
docker build -t lamp_autonomy .

Note: set ROS_IP to the publisher's IP

START:
docker run -it --rm --net=host --privileged --ipc host lamp_autonomy

Which should run ./scripts/behaviortree.sh

Load the web interface here:
http://localhost:8080
or possibly:
http://lamp:8080
