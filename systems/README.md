# Systems

`systems/` contains configuration for physical systems. The same system definition is mounted by hardware and simulation compositions so a simulator models a real vehicle instead of becoming a second, synthetic system.

Files that belong here describe the vehicle or device itself:

- controller parameters and hardware configuration
- measured calibration, camera intrinsics, and sensor extrinsics
- URDF/Xacro geometry and static transforms
- stable topic, frame, and hardware-interface contracts

Simulator runtimes, worlds, physics adapters, generated terrain, missions, and validation programs belong under `applications/`. Container orchestration belongs under `compositions/`.

## Current systems

- `aerodrone/`: the repository single quadrotor, used by hardware, Gazebo, Genesis, and Isaac deployments.
- `airplane/`: ArduPlane/BlueOS fixed-wing configuration.
- `boat/`: BlueBoat/BlueOS/ArduRover configuration.
- `submarine/`: BlueROV2/BlueOS/ArduSub configuration.
- `lunar_rover/`: lunar rover configuration.
- `racecarneo/`: MIT RACECAR Neo configuration and calibration.
- `so101/`: SO-101 arm description and controller configuration.
- `realsense/` and `blackfly/`: physical camera configuration.
- `components/`: reusable composition fragments retained for compatibility.

## Ownership rule

If changing the physical vehicle or its measured calibration changes the file, it belongs in `systems/`. If changing the simulator changes the file, it belongs in that simulator `applications/<simulator>/` directory.
