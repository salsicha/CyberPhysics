# syntax=docker/dockerfile:1
FROM python:3.12-slim
ARG PERACT_REF=02fb87681c5a47be9dbf20141bedb836ee2d3ef9
ENV PYTHONUNBUFFERED=1 PYTHONPATH=/opt/peract:/workspace/peract_scripts \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility TORCH_HOME=/workspace/cache/torch
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/peract/peract.git /opt/peract && \
    git -C /opt/peract checkout "$PERACT_REF" && rm -rf /opt/peract/.git
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --index-url https://download.pytorch.org/whl/cu126 torch==2.11.0 torchvision==0.26.0 && \
    pip install numpy==2.2.6 einops==0.8.2 scipy==1.15.3 ftfy==6.3.1 regex==2025.11.3 tqdm==4.67.1
ENV PYTHONDONTWRITEBYTECODE=1
RUN groupadd --gid 1234 peract && useradd --uid 1234 --gid 1234 --create-home peract
COPY scripts /workspace/peract_scripts
WORKDIR /workspace
USER 1234:1234
CMD ["python", "/workspace/peract_scripts/policy_server.py"]
