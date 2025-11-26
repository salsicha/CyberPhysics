
# FROM ros:humble-ros-base AS build
FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" 

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

# WORKDIR /
# RUN wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip 
# RUN unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip -d /isaac-sim
# RUN /isaac-sim/post_install.sh
# RUN /isaac-sim/isaac-sim.selector.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"    

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD [ "/isaac-sim/isaac-sim.sh" ]
