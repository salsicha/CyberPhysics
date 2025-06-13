
FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    CONDA_DIR=/opt/miniconda \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/mujoco/mujoco-3.3.3/bin/:/usr/lib/nvidia" \
    PATH="$PATH:/opt/miniconda/bin:/venv/bin:$LD_LIBRARY_PATHx" \
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so \
    MUJOCO_PATH=/mujoco/mujoco-3.3.3/bin/ \
    MUJOCO_PLUGIN_PATH=/mujoco/mujoco-3.3.3/

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

RUN mkdir /mujoco
RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.3.3/mujoco-3.3.3-linux-x86_64.tar.gz -O mujoco-linux-x86_64.tar.gz
RUN tar -xvf mujoco-linux-x86_64.tar.gz -C /mujoco/

WORKDIR /lerobot

RUN conda create -y -n lerobot python=3.10 && \
    . /opt/miniconda/bin/activate lerobot && \
    conda init bash && \
    conda install pip && \
    pip install .

RUN . /opt/miniconda/bin/activate lerobot && \
    pip install mujoco mujoco-py gym-xarm gym-aloha

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
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/mujoco/mujoco-3.3.3/bin/:/usr/lib/nvidia" \
    PATH="/venv/bin:$LD_LIBRARY_PATH:/opt/miniconda/bin:$PATH" \
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so \
    MUJOCO_PATH=/mujoco/mujoco-3.3.3/bin/ \
    MUJOCO_PLUGIN_PATH=/mujoco/mujoco-3.3.3/

COPY --from=build / /

WORKDIR /lerobot/scripts

ENTRYPOINT ["/entrypoint.sh"]






# RUN wget https://repo.anaconda.com/archive/Anaconda3-2024.06-1-Linux-x86_64.sh
# RUN bash Anaconda3-2024.06-1-Linux-x86_64.sh

# source ~/.bashrc
# conda activate /home/ubuntu/anaconda3
# 


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


# =3.10
# RUN conda activate lerobot

# RUN python3 -m venv /venv

# RUN . /venv/bin/activate && 


# RUN conda create -y -n lerobot
# RUN conda env create -f /lerobot.yml




# RUN conda install -y -c conda-forge ffmpeg
# RUN pip uninstall -y opencv-python
# RUN conda install -y -c conda-forge "opencv>=4.10.0"



# \
#     pip \
#     numpy \
#     pandas \
#     ffmpeg \
#     opencv \
#     hf_xet \
#     && conda clean -afy

# RUN conda init

# RUN source activate lerobot

    # python=3.10 \
    # scikit-learn \
    # scikit-image \
    # matplotlib \
    # seaborn \
    # jupyterlab \
    # ipykernel \

# RUN conda activate lerobot && conda install -y -c conda-forge \
#     "python>=3.10" \
#     "scikit-learn>=1.4.0" \
#     "scikit-image>=0.22.0" \
#     "matplotlib>=3.8.0" \
#     "seaborn>=0.13.2" \
#     "jupyterlab>=4.0.0" \
#     "ipykernel>=6.25.2" \
#     "ffmpeg>=5.1.2" \
#     "opencv>=4.10.0"
# RUN conda activate lerobot && conda install -y -c huggingface \
#     "hf_xet>=0.1.0" \
#     "hf_hub>=0.16.4" \
#     "hf_transfer>=0.1.0" \
#     "hf_transformers>=4.40.2" \
#     "hf_datasets>=2.14.3" \
#     "hf_tokenizers
