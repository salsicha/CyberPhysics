
FROM ros:humble-ros-base AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim wget \
    ros-${ROS_DISTRO}-rai-interfaces && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

RUN . /venv/bin/activate && pip3 install rai-core && rai-config-init

WORKDIR /
RUN curl -sSL https://install.python-poetry.org | python3 -
RUN git clone https://github.com/RobotecAI/rai.git
WORKDIR /rai
RUN vcs import < ros_deps.repos
RUN poetry install
RUN rosdep install --from-paths src --ignore-src -r -y
RUN poetry run streamlit run src/rai_core/rai/frontend/configurator.py
RUN colcon build --symlink-install

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

