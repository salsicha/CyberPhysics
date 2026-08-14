# Genesis Simulator

https://github.com/Genesis-Embodied-AI/Genesis

https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/hover_env.html


Run drone in simulation:

python3 applications/genesis/scripts/drone/ros2_drone.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard



The production Aerodrone backend is `scripts/aerodrone_navsim.py`; see `AERODRONE.md`. Simulator tuning belongs in `config/`, while the one physical vehicle definition remains in `systems/aerodrone/`.
