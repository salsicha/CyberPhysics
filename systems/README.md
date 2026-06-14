# Systems

`systems/` contains hardware-specific calibration and configuration. Put files
here when they describe one physical setup rather than reusable application
software. Examples include URDF/Xacro models, camera calibration, controller
YAML, simulator worlds tied to a robot setup, static transforms, maps, and
vehicle-specific environment files.

Applications should stay generic and accept paths into `systems/` through launch
arguments, environment variables, or compose mounts.

Current systems:

- `so101/`: SO-101 arm description, controller config, and simulator world.
- `realsense/`: RealSense-specific example config.

Expected future systems:

- A downward-OAK-1 drone for MapNav/WildNav.
- MIT RACECAR Neo.
