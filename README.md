<a href="">
  <img src="https://media.githubusercontent.com/media/salsicha/CyberPhysics/main/icon.png"
    height="70" align="right" alt="" />
</a>

# CyberPhysics

CyberPhysics is a platform for deploying robotic and ML applications.  



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


## Nvblox extension for Foxglove (on host)

Reference:  
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/tree/main/nvblox_foxglove  

1. git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git  
2. cd nvblox_foxglove  
3. (install npm==20.10.0, see below)  
4. npm install  
5. npm run local-install  

To install npm==20.10.0:
```bash
curl https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash
nvm install 20.10.0
nvm use 20.10.0
```


## Examples

ARDUINO:  
 - Connect Arduino Teensy 4.1 to USB port on host computer.  
 - Launch "foxglove-studio" from command line.  
```bash
cd compositions
docker compose -f arduino.yaml up
```
 - Messages from the Arduino board should now be visible  

SPATIAL RECONSTRUCTION:  
 - Connect Realsense 435i to USB port on host computer.  
```bash  
cd compositions  
docker compose -f reconstruction.yaml up  
```  
 - Launch "foxglove-studio" from command line.  
 - Select "Open connection" -> "Rosbridge" -> "Open"  
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
cset shield --cpu 0,1 --kthread on 
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

## Raspberry Pi 5 fix

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```


## Migrating to LFS
```bash
git lfs migrate import --include="*.exe, *.iso" --everything
```


## Gen AI notes:
Code gen

I need to implement [specific functionality] in [programming language].
Key requirements:
1. [Requirement 1]
2. [Requirement 2]
3. [Requirement 3]
Please consider:
- Error handling
- Edge cases
- Performance optimization
- Best practices for [language/framework]
Please do not unnecessarily remove any comments or code.
Generate the code with clear comments explaining the logic.

—

Review code

Can you explain the following part of the code in detail:
[paste code section]
Specifically:
1. What is the purpose of this section?
2. How does it work step-by-step?
3. Are there any potential issues or limitations with this approach?

—

Improvements 

Please review the following code:
[paste your code]
Consider:
1. Code quality and adherence to best practices
2. Potential bugs or edge cases
3. Performance optimizations
4. Readability and maintainability
5. Any security concerns
Suggest improvements and explain your reasoning for each suggestion.

—

Algorithm 

Implement a [name of algorithm] in [programming language]. Please include:
1. The main function with clear parameter and return types
2. Helper functions if necessary
3. Time and space complexity analysis
4. Example usage

—

Class

Create a [class/module] for [specific functionality] in [programming language].
Include:
1. Constructor/initialization
2. Main methods with clear docstrings
3. Any necessary private helper methods
4. Proper encapsulation and adherence to OOP principles

—

Optimizing

Here's a piece of code that needs optimization:
[paste code]
Please suggest optimizations to improve its performance. For each suggestion, explain the expected improvement and any trade-offs.

—

Tests

Generate unit tests for the following function:
[paste function]
Include tests for:
1. Normal expected inputs
2. Edge cases
3. Invalid inputs
Use [preferred testing framework] syntax.

