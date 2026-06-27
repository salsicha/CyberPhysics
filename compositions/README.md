# Compositions

`compositions/` contains Docker Compose stacks. A composition should describe a runnable system assembled from reusable application images and mounted system configuration.

## Run

```bash
docker compose -f compositions/<stack>.yaml up
```

Stop and clean up orphaned services after changing a stack:

```bash
docker compose -f compositions/<stack>.yaml down --remove-orphans
```

## Examples

```bash
docker compose -f compositions/marimo.yaml up
docker compose -f compositions/racecarneo.yaml up
docker compose -f compositions/turret_sim.yaml --profile validation up
docker compose -f compositions/nvblox.yaml up
docker compose -f compositions/boat_sim.yaml up
docker compose -f compositions/submarine_sim.yaml up
docker compose -f compositions/lunar_rover_sim.yaml up
docker compose -f compositions/airplane_sim.yaml up
docker compose -f compositions/airplane_hardware.yaml up
docker compose -f compositions/turret_hardware.yaml up
docker compose -f compositions/lunar_rover_hardware.yaml up
```

## RACECAR Neo

`racecarneo.yaml` is the full autonomous RACECAR Neo simulation stack. It starts the headless simulator, ROS bridge, Depth Anything, nvblox, SLAM Toolbox, Nav2, NiceGUI, and Foxglove bridge.

```bash
docker compose -f compositions/racecarneo.yaml up
docker compose -f compositions/turret_sim.yaml --profile validation up
```

NiceGUI is served on `http://localhost:8080`.

## Guidelines

- Mount robot-specific config from `systems/<system>/config`.
- Prefer `network_mode: host` for ROS 2 stacks unless a composition has a reason not to.
- Keep generated maps and large runtime assets outside the repository.
- Use profiles for optional demos or heavyweight services when practical.
