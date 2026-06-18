# Aerostack2 Genesis NavSim

This system runs Genesis headlessly and publishes ROS 2 topics matching the
Gazebo/Isaac navsim contract:

- `/drone_sim_0/sensor_measurements/gps`
- `/drone_sim_0/sensor_measurements/odom`
- `/drone_sim_0/sensor_measurements/imu`
- `/drone_sim_0/sensor_measurements/downward_rgb/image_raw`
- `/drone_sim_0/sensor_measurements/downward_rgb/camera_info`
- `/drone_sim_0/sensor_measurements/downward_rgbd/depth`
- `/drone_sim_0/sensor_measurements/downward_rgbd/depth/camera_info`

Run the headless simulator:

```bash
docker compose -f compositions/aerostack2_genesis.yaml up
```

Run it with DemNav and WildNav consumers:

```bash
docker compose --profile navsim -f compositions/aerostack2_genesis.yaml up
```

The default origin is Crissy Field / Presidio in San Francisco:

```bash
INITIAL_LAT=37.8044
INITIAL_LON=-122.4661
```

The bridge uses Genesis for the drone simulation and publishes deterministic
procedural downward RGB/depth images. It is intended to verify headless ROS 2
sensor plumbing. Algorithm-quality DemNav/WildNav tests still need terrain and
imagery that are georeferenced to the DEM/satellite cache used by those nodes.
