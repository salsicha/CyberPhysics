
# ComfyUI and LTX Video

https://github.com/comfyanonymous/ComfyUI?tab=readme-ov-file#manual-install-windows-linux

https://github.com/Lightricks/LTX-Video?tab=readme-ov-file#inference


## Text to video, CLI:
python inference.py --prompt "PROMPT" --height HEIGHT --width WIDTH --num_frames NUM_FRAMES --seed SEED --pipeline_config configs/ltxv-13b-0.9.7-dev-fp8.yaml


## Image to video, CLI:
python inference.py --prompt "PROMPT" --conditioning_media_paths IMAGE_PATH --conditioning_start_frames 0 --height HEIGHT --width WIDTH --num_frames NUM_FRAMES --seed SEED --pipeline_config configs/ltxv-13b-0.9.7-dev-fp8.yaml
