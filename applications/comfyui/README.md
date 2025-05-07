
https://github.com/comfyanonymous/ComfyUI?tab=readme-ov-file#manual-install-windows-linux

https://github.com/Lightricks/LTX-Video?tab=readme-ov-file#inference



POST INSTALL:
Before using compose file run:
sudo mkdir /ComfyUI/models/  
sudo mkdir /ComfyUI/custom_nodes  



DOWNLOAD MODELS AND NODES:

cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Lightricks/LTX-Video/resolve/main/ltx-video-2b-v0.9.1.safetensors  

Post install: Use ComfyUI-Manager. Simply search for ComfyUI-LTXVideo in list of nodes  

Post install: Install one of the t5 text encoders, for example google_t5-v1_1-xxl_encoderonly. You can install it using ComfyUI Model Manager.  
https://huggingface.co/mcmonkey/google_t5-v1_1-xxl_encoderonly/tree/main  

Post install: press "Install Missing Custom Nodes" button in ComfyUI Manager  

Example workflow:  
https://github.com/Lightricks/ComfyUI-LTXVideo/blob/master/example_workflows/ltxv-13b-i2v-base.json  

