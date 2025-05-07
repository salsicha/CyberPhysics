
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
    curl gnupg2 lsb-release python3-pip python3-venv vim git wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

WORKDIR /

RUN . /venv/bin/activate && pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu128

RUN git clone https://github.com/Lightricks/LTX-Video.git
RUN . /venv/bin/activate && cd LTX-Video && python3 -m pip install -e .

RUN git clone https://github.com/comfyanonymous/ComfyUI.git
RUN . /venv/bin/activate && cd ComfyUI && pip3 install -r requirements.txt

RUN cd /ComfyUI/custom_nodes && git clone https://github.com/Lightricks/ComfyUI-LTXVideo.git
RUN . /venv/bin/activate && cd /ComfyUI/custom_nodes/ComfyUI-LTXVideo && pip3 install -r requirements.txt

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

CMD ["python", "main.py"]

