

Update:
SO-101 with IsaacSim
https://lycheeai-hub.com/isaac-sim/ros2/so-arm101-moveit-in-isaac-sim-with-ros2



Make lerobot run in simulation with pi0 and SO-100

https://github.com/huggingface/lerobot/blob/main/lerobot/scripts/control_sim_robot.py

python lerobot/scripts/control_robot.py record \
    --fps 30 \
    --robot-path lerobot/configs/robot/your_robot_config.yaml \
    --sim-config lerobot/configs/env/your_sim_config.yaml
    --pretrained_policy_name_or_path=/path/to/pretrained/pi0


Make lerobot control real SO-100 with pi0

https://github.com/huggingface/lerobot/blob/main/lerobot/scripts/control_robot.py

python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=teleoperate
    --control.policy.path=/path/to/pretrained/pi0



https://github.com/huggingface/lerobot

https://github.com/TheRobotStudio/SO-ARM100

https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md

https://huggingface.co/lerobot/pi0



Hardware used to train and evaluate SO-100/101: https://github.com/TheRobotStudio/SO-ARM100

Base model https://huggingface.co/lerobot/smolvla_base
