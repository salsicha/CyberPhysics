# Aerostack2 Genesis simulation

[`aerostack2_genesis.yaml`](../../compositions/aerostack2_genesis.yaml) loads
the shared real SRTM/orthophoto terrain as a native collision heightfield and
renders the shared georeferenced terrain mesh (`terrain.obj`, the one Gazebo
and Isaac draw), uses a Genesis drone entity for propeller dynamics, and
publishes actual rendered RGB/depth images.
ArduCopter SITL owns the flight loop and connects to Aerostack2 through MAVROS
and `as2_platform_blueos`.

```bash
docker compose -f compositions/aerostack2_genesis.yaml up
```

It requires NVIDIA Container Toolkit (the cameras render through EGL) and the
locally built `salsicha/cyberphysics:genesis-latest` image. Physics runs on the
CPU by default (`GENESIS_BACKEND=cpu`): for this one small rigid body the GPU
backend is bound by kernel launches and steps about 1.4x slower.

The drone is Genesis' 27 g Crazyflie (`urdf/drones/cf2x.urdf`).
`scripts/aerodrone_navsim.py` turns ArduPilot's quad X motor outputs into
rotor thrusts with ArduPilot's own thrust curve and applies them, with the
rotor reaction moments, as one body wrench: the URDF's propeller links have no
offset from the body, so Genesis' `set_propellers_rpm()` cannot roll or pitch
it. `config/aerodrone_sitl.params` pins that motor model and scales the
ArduCopter rate gains to the Crazyflie's control authority.
