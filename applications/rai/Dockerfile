ARG TAG=latest

# ROS 2 Jazzy (Ubuntu 24.04, Python 3.12) base shared with the other apps.
# RAI 2.12.4 supports Humble and Jazzy (its CI builds and tests both; Python
# >=3.10,<3.13).
FROM salsicha/cyberphysics:ros2-${TAG}

# Pinned RAI release; RAI manages its Python environment with uv (uv.lock)
ARG RAI_REF=2.12.4
ARG UV_VERSION=0.12.24

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy \
    UV_PYTHON_DOWNLOADS=never \
    UV_LINK_MODE=copy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl python3-pip python3-venv python3-vcstool \
    ros-${ROS_DISTRO}-rai-interfaces && \
    rm -rf /var/lib/apt/lists/*

RUN curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" | env UV_INSTALL_DIR=/usr/local/bin sh

RUN git clone --depth 1 --branch ${RAI_REF} https://github.com/RobotecAI/rai.git /rai
WORKDIR /rai
RUN vcs import < ros_deps.repos
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

# Python environment in /rai/.venv, using the ROS distro's system Python;
# --no-cache keeps uv's download cache out of the layer
RUN uv sync --locked --no-cache --python /usr/bin/python3

# Example demos:
# https://robotecai.github.io/rai/demos/manipulation/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV PATH="/rai/.venv/bin:$PATH" \
    STREAMLIT_SERVER_HEADLESS=true \
    STREAMLIT_BROWSER_GATHER_USAGE_STATS=false

ENTRYPOINT ["/entrypoint.sh"]

# RAI configurator (web UI on http://localhost:8501); writes /rai/config.toml
CMD ["streamlit", "run", "src/rai_core/rai/frontend/configurator.py"]
