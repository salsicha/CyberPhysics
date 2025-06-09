


# huggingface.co/blog/smolvla


git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[smolvla]"






conda create -y -n lerobot python=3.10

conda activate lerobot

git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/huggingface/lerobot.git ~/lerobot

cd ~/lerobot && pip install -e ".[feetech]"
pip install -e ".[aloha, xarm, pusht]"
pip install -e ".[pi0]"

conda install -y -c conda-forge ffmpeg
pip uninstall -y opencv-python
conda install -y -c conda-forge "opencv>=4.10.0"




## running pi0
# policy = Pi0Policy.from_pretrained("lerobot/pi0")
# action = policy.select_action(batch)

