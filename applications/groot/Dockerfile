# syntax=docker/dockerfile:1
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04 AS build

SHELL ["/bin/bash", "-c"]

ARG GR00T_REPO=https://github.com/NVIDIA/Isaac-GR00T.git
ARG GR00T_REF=main

ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64 \
    UV_INSTALL_DIR=/usr/local/bin

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -euo pipefail && \
    find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) -exec sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' \
      {} + && \
    apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      curl \
      ffmpeg \
      git \
      git-lfs \
      libegl1 \
      python-is-python3 \
      python3-pip \
      python3.10 \
      python3.10-dev \
      python3.10-venv \
      software-properties-common \
      wget && \
    python -m pip install --upgrade --retries 5 pip setuptools wheel && \
    curl --retry 5 --retry-delay 10 --retry-max-time 300 -LsSf https://astral.sh/uv/0.8.14/install.sh | sh

WORKDIR /opt

RUN GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --recurse-submodules --shallow-submodules --branch ${GR00T_REF} ${GR00T_REPO} Isaac-GR00T

WORKDIR /opt/Isaac-GR00T

RUN --mount=type=cache,target=/root/.cache/uv,sharing=locked \
    set -euo pipefail && \
    UV_PREVIEW=1 UV_HTTP_TIMEOUT=300 UV_CONCURRENT_DOWNLOADS=4 uv sync --frozen --no-install-project --extra dev && \
    uv pip install --python /opt/Isaac-GR00T/.venv/bin/python -e . --no-deps

COPY scripts/smoke_test.py /workspace/smoke_test.py
COPY scripts/so101_policy_server.py /workspace/so101_policy_server.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /workspace/smoke_test.py /workspace/so101_policy_server.py

RUN mkdir -p /usr/share/glvnd/egl_vendor.d && \
    printf '%s\n' \
      '{' \
      '    "file_format_version" : "1.0.0",' \
      '    "ICD" : {' \
      '        "library_path" : "libEGL_nvidia.so.0"' \
      '    }' \
      '}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json && \
    mkdir -p /usr/share/vulkan/icd.d && \
    printf '%s\n' \
      '{' \
      '    "file_format_version": "1.0.0",' \
      '    "ICD": {' \
      '        "library_path": "libGLX_nvidia.so.0",' \
      '        "api_version": "1.2.140"' \
      '    }' \
      '}' > /usr/share/vulkan/icd.d/nvidia_icd.json

FROM scratch

ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    CUDA_HOME=/usr/local/cuda \
    PATH=/opt/Isaac-GR00T/.venv/bin:/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64 \
    VIRTUAL_ENV=/opt/Isaac-GR00T/.venv \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

COPY --from=build / /

WORKDIR /opt/Isaac-GR00T
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python", "/workspace/smoke_test.py"]
