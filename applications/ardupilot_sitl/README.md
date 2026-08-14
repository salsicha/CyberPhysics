# ArduPilot SITL

This image pins ArduPilot `Plane-4.6.3` and prebuilds ArduPlane, ArduCopter,
ArduRover, and ArduSub. It also includes JSBSim for the airplane backend.

Build it with:

```bash
make -C applications build_ardupilot_sitl
```

The Aerostack2 simulator compositions run ArduCopter through its JSON
external-physics protocol. Boat and submarine compositions run ArduRover and
ArduSub. Every launcher uses `--no-rebuild`; no firmware is cloned or compiled
at simulation startup. Shared JSON transport and SITL launch helpers live in
`scripts/`; vehicle parameters remain in each physical `systems/` directory.
