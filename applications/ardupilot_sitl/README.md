# ardupilot_sitl

ArduPlane SITL prebuilt at the pinned `ARDUPILOT_REF` (see
`systems/airplane/config/sitl.env`), plus JSBSim for the optional
higher-fidelity airframe model. Replaces the old clone-and-compile-at-
startup pattern so simulations boot offline in seconds and CI can run them.

Build: `make -C applications build_ardupilot_sitl`

Run (see `compositions/airplane_sim.yaml`):

    python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f plane \
        --no-rebuild --no-mavproxy --custom-location=... \
        --add-param-file=...

`--no-rebuild` is required — the binary is already at
`/ardupilot/build/sitl/bin/arduplane`.
