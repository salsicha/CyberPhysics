# JUPYTER

FROM ubuntu:noble AS build
# FROM nvidia/cuda:12.6.0-runtime-ubuntu24.04 AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    python3-pip python3-venv curl gnupg2 lsb-release unzip ca-certificates cmake ccache wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt && \
    rm -rf /root/.cache

# Install Cuda:
# https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu
WORKDIR /
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb
RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    cuda-toolkit nvidia-gds && \
    rm -rf /var/lib/apt/lists/*

# Compile Open3D:
# https://www.open3d.org/docs/release/compilation.html
RUN curl -L -o /open3d.zip https://github.com/isl-org/Open3D/archive/refs/tags/main-devel.zip && \
    unzip /open3d.zip && rm /open3d.zip
RUN /Open3D-main-devel/util/install_deps_ubuntu.sh
WORKDIR /Open3D-main-devel/
RUN mkdir build && cd build && \
    cmake -DBUILD_CUDA_MODULE=ON \
    -DGLIBCXX_USE_CXX11_ABI=OFF \
    -DBUILD_PYTORCH_OPS=ON \
    -DBUILD_TENSORFLOW_OPS=ON \
    -DBUNDLE_OPEN3D_ML=ON \
    -DOPEN3D_ML_ROOT=https://github.com/isl-org/Open3D-ML.git \
    .. && \
    make -j$(nproc) && \
    . /venv/bin/activate && make install && make install-pip-package

RUN mkdir -p /notebooks


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

WORKDIR /notebooks

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip", "0.0.0.0", "--no-browser", \
    "--allow-root", "--ServerApp.token=docker_jupyter", \
    "--NotebookApp.allow_password_change=False"]
