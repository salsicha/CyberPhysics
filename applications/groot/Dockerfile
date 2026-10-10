# syntax=docker/dockerfile:1
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

SHELL ["/bin/bash", "-c"]

ARG GR00T_REPO=https://github.com/NVIDIA/Isaac-GR00T.git
# Pinned Isaac-GR00T commit (main as of 2026-10-07). The policy server, bridge
# healthcheck, modality config, open-loop eval and finetune wrappers were
# checked against this revision's gr00t.policy/data/configs/eval APIs and
# examples/finetune.sh. Update deliberately; a branch name also works.
ARG GR00T_REF=d2b7e75b937e3ec9aa5dbc798f08b89692c49734

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

# `git clone --branch` cannot take a commit SHA; fetch the exact ref instead.
RUN set -euo pipefail && \
    mkdir Isaac-GR00T && cd Isaac-GR00T && \
    git init -q && git remote add origin "${GR00T_REPO}" && \
    GIT_LFS_SKIP_SMUDGE=1 git fetch --depth 1 origin "${GR00T_REF}" && \
    GIT_LFS_SKIP_SMUDGE=1 git checkout -q FETCH_HEAD && \
    GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive --depth 1

WORKDIR /opt/Isaac-GR00T

RUN --mount=type=cache,target=/root/.cache/uv,sharing=locked \
    set -euo pipefail && \
    UV_PREVIEW=1 UV_HTTP_TIMEOUT=300 UV_CONCURRENT_DOWNLOADS=4 uv sync --frozen --no-install-project --extra dev && \
    uv pip install --python /opt/Isaac-GR00T/.venv/bin/python -e . --no-deps && \
    uv pip install --python /opt/Isaac-GR00T/.venv/bin/python "rosbags>=0.10,<0.11"

COPY scripts/smoke_test.py /workspace/smoke_test.py
COPY scripts/so101_policy_server.py /workspace/so101_policy_server.py
COPY scripts/so101_task_policy.py /workspace/so101_task_policy.py
# Loads systems/so101/config/joint_limits.json from the /workspace/systems mount.
COPY scripts/so101_joint_limits.py /workspace/so101_joint_limits.py
COPY scripts/policy_healthcheck.py /workspace/policy_healthcheck.py
COPY config/ /workspace/groot_config/
COPY scripts/ /workspace/groot_scripts/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /workspace/smoke_test.py /workspace/so101_policy_server.py /workspace/policy_healthcheck.py && \
    find /workspace/groot_scripts -type f -name '*.sh' -exec chmod +x {} +

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

# Runtime environment: the Isaac-GR00T venv first on PATH, EGL rendering
ENV PATH=/opt/Isaac-GR00T/.venv/bin:${PATH} \
    VIRTUAL_ENV=/opt/Isaac-GR00T/.venv \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

WORKDIR /opt/Isaac-GR00T
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python", "/workspace/smoke_test.py"]
