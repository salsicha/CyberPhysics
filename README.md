<a href="">
  <img src="https://media.githubusercontent.com/media/salsicha/CyberPhysics/main/icon.png"
    height="70" align="right" alt="" />
</a>

# CyberPhysics

CyberPhysics is a repo for managing my personal robotics and ml projects.

It is great starting place for projects that need to manage large-data and complex cyber-physical systems. 

## Features

- browser-based graphical user interface
- access to latest ROS tools
- deployable to kubernetes
- and more...

## Installation

(This project is designed for Ubuntu 24.04)

```bash
./install.sh
```

## Building images

From the applications folder run "make build_<app_name>". For example:

```bash
cd applications && make build_ros2
```

## Usage

Write new applications and put them in the applications folder.

Compose your applications together using docker compose in the compositions folder.

Launch it with docker compose. For example:

```bash
docker compose -f compositions/jupyter.yaml up
```

For viewing ROS data, run the Foxglove compose file, and launch Foxglove on your host machine.
```bash
foxglove-studio
```

A web based GUI can be built with NiceGUI, and then accessed through your host machine's browser:
http://localhost:8080/


## Examples

ARDUINO:  
 - Connect Arduino Teensy 4.1 to USB port on host computer.  
 - Launch "foxglove-studio" from command line.  
```bash
cd compositions
docker compose -f arduino.yaml up
```
 - Load micro-ROS library: "Sketch -> Include library -> Add .ZIP Library... /microrosarduino.zip"
 - Load "pub.ino" sketch
 - Messages from the Arduino board should now be visible  

SPATIAL RECONSTRUCTION:  
 - Connect Realsense 435i to USB port on host computer.  
 - Launch "foxglove-studio" from command line.  
```bash  
cd compositions  
docker compose -f reconstruction.yaml up  
```  
 - Messages from the Nvblox board should now be visible  


## Documentation

Have a look at the README in each application's folder for explanations of what they do.


## Why?

Many cyber-physical systems quickly become unamangeable as complexity and dependency conflicts scale exponentially with the number of components. This project should help with that.

## Note on real time performance

Real time performance can be achieved by pinning a process to a core:

```bash
docker run --cpuset-cpus="0,1" -it your_image your_command
```

```yaml
services:
  your_service:
    image: your_image
    cpuset: "0,1"
```

Then reserve the cores needed before running the containers:

```bash
cset shield --cpu 0,1 --kthread on # cpu 5 and 11, move kernel processes if possible
```

Then set cgroup-parent in dockerd config "/etc/docker/daemon.json":
```json 
{
  "cgroup-parent": "/system"
}
```

Then to restore the host system:
```bash
cset shield --reset
```
