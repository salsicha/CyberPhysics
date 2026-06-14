# Aerostack2 Isaac NavSim

This system runs an Isaac Sim sensor bridge for Aerostack2, DemNav, and WildNav.
It publishes the same simulated contract as the Gazebo navsim system:

- `/drone_sim_0/sensor_measurements/gps`
- `/drone_sim_0/sensor_measurements/odom`
- `/drone_sim_0/downward_rgb/image`
- `/drone_sim_0/downward_rgb/camera_info`
- `/drone_sim_0/downward_rgbd/depth/image`
- `/drone_sim_0/downward_rgbd/camera_info`

The default location is Crissy Field / Presidio in San Francisco:

```bash
INITIAL_LAT=37.8044
INITIAL_LON=-122.4661
```

Run the stack:

```bash
docker compose --profile navsim -f compositions/aerostack2_isaac.yaml up --force-recreate
```

To start a flight somewhere else, override the origin. The `demnav_assets` and
`wildnav_assets` services seed the required DEM and satellite caches into Docker
volumes before DemNav and WildNav start:

```bash
INITIAL_LAT=37.7694 INITIAL_LON=-122.4862 \
docker compose --profile navsim -f compositions/aerostack2_isaac.yaml up --force-recreate
```

Generated DEMs, satellite tiles, descriptors, and Isaac runtime caches live in
Docker volumes or the host Isaac cache directories. Do not store them in this
repository.

The procedural Isaac scene is intended to verify the ROS topic and container
wiring. Algorithm-quality WildNav testing needs an Isaac world whose downward
RGB imagery is georeferenced to the same satellite source used by WildNav.
Algorithm-quality DemNav testing needs terrain geometry/depth that corresponds
to the DEM used by DemNav.
