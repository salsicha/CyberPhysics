
FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim git wget ffmpeg && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

WORKDIR /

COPY configs /configs/
COPY workflows /workflows/

RUN . /venv/bin/activate && pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu128

RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/Lightricks/LTX-Video.git
RUN . /venv/bin/activate && cd LTX-Video && python3 -m pip install -e .
RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/comfyanonymous/ComfyUI.git
RUN . /venv/bin/activate && cd ComfyUI && pip3 install -r requirements.txt
RUN cd /ComfyUI/custom_nodes && git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/Lightricks/ComfyUI-LTXVideo.git
RUN . /venv/bin/activate && cd /ComfyUI/custom_nodes/ComfyUI-LTXVideo && pip3 install -r requirements.txt

RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Lightricks/LTX-Video/resolve/main/ltx-video-2b-v0.9.5.safetensors
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Lightricks/LTX-Video/resolve/main/ltxv-13b-0.9.7-dev-fp8.safetensors
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/mcmonkey/google_t5-v1_1-xxl_encoderonly/resolve/main/model.safetensors
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/runwayml/stable-diffusion-v1-5/resolve/main/v1-5-pruned-emaonly.ckpt
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Comfy-Org/stable-diffusion-v1-5-archive/resolve/main/v1-5-pruned-emaonly-fp16.safetensors
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Lightricks/LTX-Video/resolve/main/ltxv-spatial-upscaler-0.9.7.safetensors
RUN cd /ComfyUI/models/checkpoints && wget https://huggingface.co/Lightricks/LTX-Video/resolve/main/ltxv-temporal-upscaler-0.9.7.safetensors

RUN cd /ComfyUI/models/vae/ && wget https://huggingface.co/stabilityai/sd-vae-ft-mse-original/resolve/main/vae-ft-mse-840000-ema-pruned.safetensors

RUN cd /ComfyUI/custom_nodes && git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/Comfy-Org/ComfyUI-Manager.git comfyui-manager
RUN cd /ComfyUI/custom_nodes && git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/kijai/ComfyUI-KJNodes.git
RUN . /venv/bin/activate && cd /ComfyUI/custom_nodes/ComfyUI-KJNodes && pip3 install -r requirements.txt
RUN cd /ComfyUI/custom_nodes && git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/Kosinkadink/ComfyUI-VideoHelperSuite.git
RUN . /venv/bin/activate && cd /ComfyUI/custom_nodes/ComfyUI-VideoHelperSuite && pip3 install -r requirements.txt

COPY prestart.py /ComfyUI/prestart.py
RUN . /venv/bin/activate && cd /ComfyUI && python3 prestart.py

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /ComfyUI

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "main.py"]
