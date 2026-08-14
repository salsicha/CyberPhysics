

# Gazebo Harmonic

Gazebo-owned runtime integration lives in `scripts/`; native models and simulator-only parameters live in `models/` and `config/`:

- `drone_bridges_isolated.py` bridges native Gazebo topics to ROS 2 while
  keeping simulation-truth TF separate from Navigator localization TF.
- `gazebo_ardupilot_backend.py` connects Navigator/ArduPilot's JSON external
  physics protocol to Gazebo's native rotor model and physical state.

The Aerodrone-specific launch contract is documented in `AERODRONE.md`. Physical vehicle configuration remains in `systems/aerodrone/`.


