ARG ISAAC_SIM_VERSION=6.0.1
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

LABEL org.opencontainers.image.title="CyberPhysics Isaac Sim"
LABEL org.opencontainers.image.description="Isaac Sim 6.0.1 runtime with CyberPhysics integration assets"

USER root

COPY config/ /workspace/applications/isaac/config/
COPY scripts/ /workspace/applications/isaac/scripts/

RUN chmod +x /workspace/applications/isaac/scripts/*.py

USER isaac-sim

WORKDIR /isaac-sim
