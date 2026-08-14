# Isaac Sim

The `cyberphysics/isaac` image is built from NVIDIA Isaac Sim 6.0.1 and is the single Isaac runtime used by compositions in this repository. It bakes the shared configuration and simulator integration scripts from this directory into `/workspace/applications/isaac`.

Build it after authenticating Docker with NVIDIA NGC:

```bash
make -C applications build_isaac
```

## Aerodrone simulation

[`aerostack2_isaac.yaml`](../../compositions/aerostack2_isaac.yaml) runs the
owned image with PhysX rigid-body dynamics and ROS 2 camera
helpers. It loads the same real SRTM/orthophoto terrain and detailed behavior
tree as the Gazebo backend. ArduCopter SITL supplies motor commands through the
JSON external-physics protocol; MAVROS and `as2_platform_blueos` provide the
Navigator-compatible control boundary. The custom Isaac vehicle model includes
Gazebo-matched quad-X motor placement and spool constants, passive rigid-body
damping, and body-frame IMU conversion.

```bash
docker compose -f compositions/aerostack2_isaac.yaml up
```

An NVIDIA GPU, NVIDIA Container Toolkit, NGC access while building, and acceptance of
the NVIDIA EULA are required. Persistent Isaac caches are initialized by the
composition. RGB/depth data comes from the Isaac render product rather than
procedural arrays. The supplied UDP-only Fast DDS profile is required for
interoperability between Isaac's bundled Humble middleware and the other ROS 2
containers.
