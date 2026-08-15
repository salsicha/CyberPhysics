# syntax=docker/dockerfile:1
FROM nvidia/cuda:12.8.0-runtime-ubuntu24.04

SHELL ["/bin/bash", "-c"]

ARG LEROBOT_REPO=https://github.com/huggingface/lerobot.git
ARG LEROBOT_REF=v0.6.0
ARG UV_VERSION=0.8.14
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu126

ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    UV_INSTALL_DIR=/usr/local/bin \
    HF_HOME=/workspace/cache/huggingface \
    HF_HUB_CACHE=/workspace/cache/huggingface/hub \
    TORCH_HOME=/workspace/cache/torch \
    PYTHONUNBUFFERED=1

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -euo pipefail && \
    apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      curl \
      ffmpeg \
      git \
      git-lfs \
      python3.12 \
      python3.12-dev \
      python3.12-venv && \
    curl --retry 5 --retry-delay 5 --retry-max-time 300 \
      -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" | sh

WORKDIR /opt

RUN GIT_LFS_SKIP_SMUDGE=1 git clone \
      --branch "${LEROBOT_REF}" \
      --depth 1 \
      "${LEROBOT_REPO}" lerobot

WORKDIR /opt/lerobot
# The LeRobot lock selects cu128. PyTorch 2.11 cu128 dropped Volta kernels;
# the matching cu126 wheels retain sm_70 support for 32 GB V100 systems.


RUN --mount=type=cache,target=/root/.cache/uv,sharing=locked \
    set -euo pipefail && \
    export UV_HTTP_TIMEOUT=300 UV_CONCURRENT_DOWNLOADS=4 && \
      uv sync --frozen --no-dev --no-install-project \
        --extra smolvla \
        --extra training \
        --extra peft && \
    uv pip install --python /opt/lerobot/.venv/bin/python --reinstall \
      --index-url "${PYTORCH_INDEX_URL}" \
      "torch==2.11.0" \
      "torchvision==0.26.0" \
      "numpy==2.2.6" \
      "fsspec==2026.2.0" && \
    uv pip install --python /opt/lerobot/.venv/bin/python -e . --no-deps && \
    uv pip install --python /opt/lerobot/.venv/bin/python \
      "msgpack>=1.0,<2" \
      "msgpack-numpy>=0.4.8,<0.5" \
      "pyzmq>=26,<28" && \
    uv pip check --python /opt/lerobot/.venv/bin/python

ENV PATH=/opt/lerobot/.venv/bin:/usr/local/bin:/usr/bin:/bin \
    VIRTUAL_ENV=/opt/lerobot/.venv

COPY scripts/ /workspace/smolvla_scripts/
COPY smolvla/ /workspace/data/
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh /workspace/smolvla_scripts/*.py && \
    mkdir -p /workspace/cache \
      /workspace/data/models \
      /workspace/data/training \
      /workspace/data/telemetry && \
    python /workspace/smolvla_scripts/smoke_test.py

WORKDIR /opt/lerobot

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python", "/workspace/smolvla_scripts/so101_policy_server.py"]
