


## running pi0
# policy = Pi0Policy.from_pretrained("lerobot/pi0")
# action = policy.select_action(batch)


FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

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
# ENV CONDA_DIR /opt/conda
# RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
#     /bin/bash ~/miniconda.sh -b -p /opt/conda
# # Put conda in path so we can use conda activate
# ENV PATH=$CONDA_DIR/bin:$PATH
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

# RUN conda create -y -n lerobot python=3.10
# RUN conda activate lerobot

RUN python3 -m venv /venv

RUN . /venv/bin/activate &&  pip3 install ffmpeg opencv hf_xet

RUN mkdir /lerobot

RUN WORKDIR /lerobot

# Smolval base
RUN wget https://huggingface.co/lerobot/smolvla_base/resolve/main/model.safetensors

RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/huggingface/lerobot.git /lerobot

RUN . /venv/bin/activate && pip3 install -e ".[feetech]"
RUN . /venv/bin/activate && pip3 install -e ".[aloha, xarm, pusht]"
RUN . /venv/bin/activate && pip3 install -e ".[pi0]"
RUN . /venv/bin/activate && pip3 install -e ".[smolvla]"

# RUN conda install -y -c conda-forge ffmpeg
# RUN pip uninstall -y opencv-python
# RUN conda install -y -c conda-forge "opencv>=4.10.0"

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

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

