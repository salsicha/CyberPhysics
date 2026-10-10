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

The normal `GENESIS_BACKEND=gpu` path requires NVIDIA Container Toolkit and the
locally built `salsicha/cyberphysics:genesis-latest` image. CPU mode is intended only for
diagnostics.
