# JUPYTER

FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    python3-pip python3-venv curl gnupg2 lsb-release unzip ca-certificates cmake \
    ccache wget sudo git xorg-dev libxcb-shm0 libglu1-mesa-dev python3-dev clang \
    libc++-dev libc++abi-dev libsdl2-dev ninja-build libxi-dev \
    libtbb-dev libosmesa6-dev libudev-dev autoconf libtool && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt && \
    rm -rf /root/.cache

### Install Cuda:
### https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu
WORKDIR /
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb
RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    cuda-toolkit nvidia-gds && \
    rm -rf /var/lib/apt/lists/*

# Compile Open3D:
# https://www.open3d.org/docs/release/compilation.html
# RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/isl-org/Open3D
# WORKDIR /Open3D/
#
## export CUDA_ARCH_BIN=8.9 && \
# RUN export CUDACXX=/usr/local/cuda/bin/nvcc
#     export PATH="/usr/local/cuda/bin:$PATH" && \
#     export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH" && \
#     mkdir build && cd build && \
#     cmake -DBUILD_CUDA_MODULE=ON \
#     -DGLIBCXX_USE_CXX11_ABI=OFF \
#     -DBUILD_PYTORCH_OPS=ON \
#     -DBUNDLE_OPEN3D_ML=ON \
#     -DOPEN3D_ML_ROOT=https:///github.com/isl-org/Open3D-ML.git \
#     -DCMAKE_CUDA_COMPILER=$(which nvcc) \
#     .. && make -j$(nproc) && \
#     . /venv/bin/activate && make install-pip-package
#
### This is the nightly build as is already out of date, if this fails get a new URL from here:
### https://github.com/isl-org/Open3D/releases
RUN wget https://github.com/isl-org/Open3D/releases/download/main-devel/open3d-0.18.0+c8856fc-cp312-cp312-manylinux_2_31_x86_64.whl
RUN pip3 install ./open3d-0.18.0+c8856fc-cp312-cp312-manylinux_2_31_x86_64.whl

# TODO: install 
# https://github.com/salsicha/ArrayDataEngine.git

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
