# Compositions

`compositions/` contains Docker Compose stacks. A composition should describe a runnable system assembled from reusable application images and mounted system configuration.

## Run

```bash
docker compose -f compositions/<stack>.yaml up
```

or, from the repository root, `./start.sh <stack> [--profile <name>]... [-f <override.yaml>]... [up args...]`, which runs the same command (profiles and extra compose files go before `up`, other arguments after it). Run `./start.sh` without arguments to list the stacks:

```bash
./start.sh racecarneo
./start.sh turret_sim --profile validation
./start.sh vinsmono --build
```

Stop and clean up orphaned services after changing a stack:

```bash
docker compose -f compositions/<stack>.yaml down --remove-orphans
```

## Examples

`smolvla_workcell.yaml` runs the [learned SO-ARM100 contact workcell](../applications/smolvla/WORKCELL.md).
`arduino.yaml` runs [native Zenoh MCU routers](../applications/embedded_zenoh/README.md)
after flashing the new firmware; `arduino_xrce.yaml` preserves the legacy DDS/XRCE demo.

```bash
docker compose -f compositions/marimo.yaml up
docker compose -f compositions/racecarneo.yaml up
docker compose -f compositions/turret_sim.yaml --profile validation up
docker compose -f compositions/so101_groot_isaac.yaml --profile sim up
docker compose -f compositions/so101_smolvla_isaac.yaml --profile sim up
docker compose -f compositions/nvblox.yaml up
docker compose -f compositions/boat_sim.yaml up
docker compose -f compositions/submarine_sim.yaml up
docker compose -f compositions/lunar_rover_sim.yaml up
docker compose -f compositions/spaceros.yaml up                 # Space ROS Curiosity demo (SPACEROS_GUI=false: headless)
docker compose -f compositions/airplane_sim.yaml up
docker compose -f compositions/airplane_hardware.yaml up
docker compose -f compositions/turret_hardware.yaml up
docker compose -f compositions/racecar_turret_hardware.yaml up -d  # RACECAR Turret on the car (systems/racecar_turret/README.md)
docker compose -f compositions/turret_uno_q.yaml --profile board up    # turret bridge on an Arduino UNO Q (host: --profile host)
docker compose -f compositions/arduino_uno_q.yaml up            # UNO Q LED demo (Linux side; run on the board)
docker compose -f compositions/lunar_rover_hardware.yaml up
docker compose -f compositions/vinsmono.yaml up --build
docker compose -f compositions/cactusrt.yaml up                 # 1 kHz real-time loop demo
docker compose -f compositions/tello.yaml up                    # DJI Tello driver (join its WiFi first)
docker compose -f compositions/rangefinder.yaml up              # Benewake TFmini-S/TF-Luna on a serial port
docker compose -f compositions/whereami.yaml up                 # WiFi room-level localization
docker compose -f compositions/unrealengine.yaml up             # your packaged Unreal Engine build + ROS bridge
```

## RACECAR Neo

`racecarneo.yaml` is the full autonomous RACECAR Neo simulation stack. It starts the headless simulator, ROS bridge, Depth Anything, nvblox, SLAM Toolbox, Nav2, NiceGUI, and Foxglove bridge.

```bash
docker compose -f compositions/racecarneo.yaml up
```

NiceGUI is served on `http://localhost:8080`.

## RACECAR Turret

`racecar_turret_hardware.yaml` runs the RACECAR Neo hardware services (extended from `racecarneo_hardware.yaml`) with the PanTiltROS turret: the USB turret camera (`usb_cam`), the ST3215 servos on the Jetson's header UART (`turret_arduino_bridge.py`, direct STS transport), the YOLO tracker with `systems/racecar_turret/config/turret_tracker.yaml`, and the web UI with the turret page (`UI_TURRET=1`). Bring-up order and safety: `systems/racecar_turret/README.md`; simulation: `applications/isaac/RACECAR_TURRET.md`.

```bash
TURRET_CAMERA_DEVICE=/dev/v4l/by-id/<camera>-video-index0 \
  docker compose -f compositions/racecar_turret_hardware.yaml up -d
```

## Guidelines

- Mount robot-specific config from `systems/<system>/config`.
- Prefer `network_mode: host` for ROS 2 stacks unless a composition has a reason not to.
- Keep generated maps and large runtime assets outside the repository.
- Use profiles for optional demos or heavyweight services when practical.
