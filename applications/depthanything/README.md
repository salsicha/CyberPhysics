
# Depth Anything v2

This app builds a Depth Anything v2 ROS2 node for three targets:
1. Arduino App Lab brick 
2. BlueOS extension

## Features

- **Monocular Depth Estimation:** Generates high-quality relative depth maps from standard RGB camera feeds using the state-of-the-art Depth Anything v2 model.
- **Multi-Platform Support:** Specifically containerized and optimized for Arduino App Lab bricks, BlueOS extensions, and Oak-1 smart cameras.
- **ROS2 Integration:** Fully integrated with ROS2, subscribing to standard `sensor_msgs/Image` RGB topics and publishing depth maps as `sensor_msgs/Image`.
- **Optimized for Edge:** Leverages efficient inference execution to ensure smooth and reliable performance on constrained edge computing devices.

## How to build

To build the Docker image for this application, run the following from the root `applications` folder:

```bash
make build_depthanything
```

## Example usage

To launch the application using Docker Compose, navigate to the `deployments` folder and run:

```bash
docker compose -f depthanything.yml up
```
