# Lunar Rover System

This system describes a Space ROS lunar rover development platform. It uses the
existing `applications/spaceros` image for both the Space ROS Curiosity baseline
and a CyberPhysics south-pole traverse simulation.

The platform is a terrestrial prototype workflow. It validates rover autonomy,
topic contracts, and operations logic against lunar-relevant constraints, but it
does not claim flight qualification.

## Expected Space ROS Setup

- `cyberphysics/spaceros` built from `applications/spaceros`.
- ROS 2 Humble and Space ROS sourced by the container entrypoint.
- Optional use of the upstream Space ROS Curiosity rover demo for baseline
  rover behavior.
- Local RViz, Foxglove, or ROS 2 CLI tools for inspection.

## Run

```bash
docker compose -f compositions/lunar_rover_sim.yaml up
```

For the physical runtime skeleton:

```bash
docker compose -f compositions/lunar_rover_hardware.yaml up
```

## Simulation Scenario

The scenario in `config/south_pole_traverse.yaml` drives a lunar rover through a
south-pole crater-rim prospecting traverse. It starts at a lander-like staging
site, crosses a loose-regolith patch, climbs a ridge approach, skirts a
shadowed crater rim, visits two prospecting waypoints, and returns toward the
staging corridor.

The simulator in `config/simulation.yaml` publishes deterministic odometry,
IMU, wheel state, battery, solar illumination, body temperature, slope, and slip
topics. The autonomy node consumes those topics and slows down in high-slip,
high-slope, and low-illumination areas while completing the route without
manual commands.
