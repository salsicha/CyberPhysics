
# ComfyUI and LTX Video

https://github.com/comfyanonymous/ComfyUI?tab=readme-ov-file#manual-install-windows-linux

https://github.com/Lightricks/LTX-Video?tab=readme-ov-file#inference


## Text to video, CLI:
cd /LTX-Video  
python inference.py --prompt "PROMPT" --height 480 --width 640 --num_frames 30 --seed 123 --pipeline_config configs/ltxv-2b-0.9.6-distilled.yaml  


## Image to video, CLI:
cd /LTX-Video  
python inference.py --prompt "PROMPT" --conditioning_media_paths IMAGE_PATH --conditioning_start_frames 0 --height 480 --width 640 --num_frames 30 --seed 123 --pipeline_config configs/ltxv-2b-0.9.6-dev.yaml  
