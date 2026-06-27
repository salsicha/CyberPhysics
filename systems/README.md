# Systems

`systems/` contains hardware-specific and simulation-specific configuration. Put files here when they describe one physical or simulated robot setup rather than reusable application software.

Good examples:

- URDF/Xacro descriptions
- camera calibration
- controller YAML
- Nav2, SLAM, and bridge parameters
- topic and frame naming conventions
- simulator worlds tied to one robot
- static transforms
- runtime map download configuration

Applications should stay generic and accept paths into `systems/` through launch arguments, environment variables, or compose mounts.

## Current Systems

- `so101/`: SO-101 arm description, controller config, and simulator setup.
- `racecarneo/`: MIT RACECAR Neo simulator bridge, URDF, calibration, depth/nvblox/Nav2 config, and topic conventions.
- `aerostack2_gazebo/`: Aerostack2 drone simulation configuration for Gazebo.
- `aerostack2_isaac/`: Aerostack2 drone navigation simulation configuration for Isaac Sim.
- `aerostack2_genesis/`: Aerostack2 headless drone navigation simulation configuration for Genesis.
- `turret/`: PanTiltROS-compatible turret tracking scenario, simulator, and acceptance validation.
- `realsense/`: RealSense-specific example configuration.

## Expected Additions

- A downward-facing OAK-1 drone system for WildNav and DemNav.
- Additional physical calibration for RACECAR Neo when hardware measurements replace simulator defaults.

## Rules Of Thumb

- If the file changes when the robot changes, it belongs in `systems/`.
- If the file can run against many robots, it belongs in `applications/`.
- If the file starts multiple containers together, it belongs in `compositions/`.
- Do not store maps, bags, or downloaded simulator assets in this directory.
