
# https://github.com/msf4-0/so101_ros2


FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    CONDA_DIR=/opt/miniconda \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin:/usr/lib/nvidia" \
    PATH="$PATH:/opt/miniconda/bin:/venv/bin:$LD_LIBRARY_PATHx" \
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
    
# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release \
    imagemagick libboost-all-dev libgts-dev libjansson-dev \
    less util-linux htop atop nvtop sysstat \
    apt-transport-https python3-dev build-essential libssl-dev libffi-dev libxml2-dev \
    libxslt1-dev zlib1g-dev libglew-dev libglew2.2 \
    vim git jq wget flex software-properties-common build-essential gcc && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
RUN mv bazel-archive-keyring.gpg /usr/share/keyrings
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

RUN apt update && apt install -y bazel && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /

# Install miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /miniconda.sh
RUN /bin/bash /miniconda.sh -b -p /opt/miniconda

RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/huggingface/lerobot.git /lerobot

RUN mkdir -p /root/.mujoco
# RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.3.3/mujoco-3.3.3-linux-x86_64.tar.gz -O mujoco-linux-x86_64.tar.gz
RUN wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz -O mujoco-linux-x86_64.tar.gz
RUN tar -xvf mujoco-linux-x86_64.tar.gz -C /root/.mujoco/

WORKDIR /lerobot

RUN conda create -y -n lerobot python=3.10 && conda init && \
    . /opt/miniconda/bin/activate lerobot && \
    conda init bash && \
    conda install pip && \
    pip install .

RUN . /opt/miniconda/bin/activate lerobot && \
    pip install mujoco mujoco-py gym-xarm gym-aloha gym_pusht

RUN . /opt/miniconda/bin/activate lerobot && \
    pip install ".[pusht]" && \
    pip install ".[pi0]" && \
    pip install ".[smolvla]" && \
    pip install ".[xarm]" && \
    pip install ".[aloha]"

# Smolval base
RUN mkdir -p /lerobot/models/smolvla_base
WORKDIR /lerobot/models/smolvla_base

RUN wget https://huggingface.co/lerobot/smolvla_base/resolve/main/model.safetensors

COPY scripts/ /lerobot/scripts

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
    PATH="/venv/bin:$PATH" \
    CONDA_DIR=/opt/miniconda \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin:/usr/lib/nvidia" \
    PATH="/venv/bin:$LD_LIBRARY_PATH:/opt/miniconda/bin:$PATH" \
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

COPY --from=build / /

WORKDIR /lerobot/scripts

ENTRYPOINT ["/entrypoint.sh"]
