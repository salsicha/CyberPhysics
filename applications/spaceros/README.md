
# SpaceROS

This app serves as a demonstration for SpaceROS

See original project:
https://github.com/space-ros/space-ros

Run the curiosity rover demo:
https://github.com/space-ros/demos/blob/main/curiosity_rover/README.md

## Lunar Rover Simulation

The image also includes a lightweight ROS 2 lunar rover simulator and waypoint
autonomy node for the `systems/lunar_rover` south-pole traverse scenario:

```bash
docker compose -f compositions/lunar_rover_sim.yaml up
```

The scenario models crater-rim slope, loose regolith slip, low sun/shadow
illumination, thermal swing, battery draw, and solar recovery so the autonomy
stack can complete a realistic prospecting traverse without manual commands.
