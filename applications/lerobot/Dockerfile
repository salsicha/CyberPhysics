


FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    CONDA_DIR=/opt/miniconda \
    PATH="/opt/miniconda/bin:$PATH"

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv \
    imagemagick libboost-all-dev libgts-dev libjansson-dev \
    less util-linux htop atop nvtop sysstat \
    vim git jq wget flex software-properties-common build-essential gcc && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /

# RUN wget https://repo.anaconda.com/archive/Anaconda3-2024.06-1-Linux-x86_64.sh
# RUN bash Anaconda3-2024.06-1-Linux-x86_64.sh

# source ~/.bashrc
# conda activate /home/ubuntu/anaconda3
# 
# Install miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /miniconda.sh
RUN /bin/bash /miniconda.sh -b -p /opt/miniconda
# . /opt/miniconda/bin/activate

# # Put conda in path so we can use conda activate
# 
# RUN mkdir -p /opt/conda 
# RUN wget https://repo.anaconda.com/miniconda/Miniconda3-py38_4.12.0-Linux-x86_64.sh -O /opt/conda/miniconda.sh \ && bash /opt/conda/miniconda.sh -b -p /opt/miniconda # Install your environment.yaml deps into base env 
# # Uncomment once you are ready to start productionizing the image 
# # COPY environment.yaml /tmp 
# # RUN . /opt/miniconda/bin/activate && conda env update --name base --file /tmp/environment.yaml # Install your softwares 
# # Uncomment once you have software to install 
# # RUN mkdir /app 
# # WORKDIR /app
# # COPY * ./ # Run the software in conda base environment 
# CMD ["/opt/miniconda/bin/conda", "run", "--no-capture-output", "-n", "base", "python" "app.py"]

RUN conda create -y -n lerobot python 
# =3.10
# RUN conda activate lerobot

# RUN python3 -m venv /venv

# RUN . /venv/bin/activate && 

RUN . /opt/miniconda/bin/activate && pip3 install ffmpeg opencv-python hf_xet

WORKDIR /

RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/huggingface/lerobot.git /lerobot

WORKDIR /lerobot

# Smolval base
RUN mkdir -p /lerobot/models/smolvla_base
WORKDIR /lerobot/models/smolvla_base

RUN wget https://huggingface.co/lerobot/smolvla_base/resolve/main/model.safetensors

RUN . /opt/miniconda/bin/activate && pip3 install ".[pusht]"
RUN . /opt/miniconda/bin/activate && pip3 install ".[feetech]"
RUN . /opt/miniconda/bin/activate && pip3 install ".[xarm]"
RUN . /opt/miniconda/bin/activate && pip3 install ".[aloha]"
RUN . /opt/miniconda/bin/activate && pip3 install ".[pi0]"
RUN . /opt/miniconda/bin/activate && pip3 install ".[smolvla]"

# RUN conda install -y -c conda-forge ffmpeg
# RUN pip uninstall -y opencv-python
# RUN conda install -y -c conda-forge "opencv>=4.10.0"

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
    PATH="/opt/miniconda/bin:$PATH"

COPY --from=build / /

WORKDIR /lerobot/scripts

ENTRYPOINT ["/entrypoint.sh"]

