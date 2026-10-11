# Systems

`systems/` contains configuration for physical systems. The same system definition is mounted by hardware and simulation compositions so a simulator models a real vehicle instead of becoming a second, synthetic system.

Files that belong here describe the vehicle or device itself:

- controller parameters and hardware configuration
- measured calibration, camera intrinsics, and sensor extrinsics
- URDF/Xacro geometry and static transforms
- stable topic, frame, and hardware-interface contracts

Data that is specific to one vehicle may also live with its system: `racecarneo/`, `so101/`, and `airplane/` keep their missions, scenarios, worlds, and validation thresholds here. Simulator runtimes, physics adapters, generated terrain, and the programs that run or score missions belong under `applications/`. Container orchestration belongs under `compositions/`.

## Current systems

- `aerodrone/`: the repository single quadrotor, used by hardware, Gazebo, Genesis, and Isaac deployments.
- `airplane/`: ArduPlane/BlueOS fixed-wing configuration.
- `boat/`: BlueBoat/BlueOS/ArduRover configuration.
- `submarine/`: BlueROV2/BlueOS/ArduSub configuration.
- `lunar_rover/`: lunar rover configuration.
- `racecarneo/`: MIT RACECAR Neo configuration and calibration.
- `racecar_turret/`: the RACECAR Neo with the PanTiltROS turret on a raised plate: its interface contract, combined URDF, turret tracker profiles, turret camera calibration and lidar mask (`hardware/racecar_turret.md`, `compositions/racecar_turret_hardware.yaml`).
- `so101/`: SO-101 arm description and controller configuration.
- `realsense/` and `blackfly/`: physical camera configuration.
- `vinsmono/`: camera/IMU calibration for the VINS-Mono stack.
- `components/`: compose fragments and opt-in overrides, layered onto a stack with an extra `-f` (for example `./start.sh blackfly -f systems/components/record-host-shutdown.yaml`); runnable stacks live in `compositions/`.

## Ownership rule

If changing the physical vehicle or its measured calibration changes the file, it belongs in `systems/`. If changing the simulator changes the file, it belongs in that simulator `applications/<simulator>/` directory.
