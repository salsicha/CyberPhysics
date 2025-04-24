
FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim git software-properties-common build-essential gcc cmake && \
    rm -rf /var/lib/apt/lists/*

COPY scripts/ /scripts/

RUN git clone https://github.com/microsoft/BitNet.git /bitnet

WORKDIR /bitnet

RUN git submodule update --init --recursive

RUN python3 -m venv /venv
RUN pip3 install setuptools --upgrade
RUN . /venv/bin/activate && pip3 install -r requirements.txt

RUN huggingface-cli download microsoft/BitNet-b1.58-2B-4T-gguf --local-dir models/BitNet-b1.58-2B-4T
RUN . /venv/bin/activate && python3 setup_env.py -md models/BitNet-b1.58-2B-4T -q i2_s

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /bitnet

ENTRYPOINT ["/entrypoint.sh"]

CMD [ "python run_inference.py", "-m", "models/BitNet-b1.58-2B-4T/ggml-model-i2_s.gguf",_"-p",_"'You are a helpful assistant'", "-cnv" ]
