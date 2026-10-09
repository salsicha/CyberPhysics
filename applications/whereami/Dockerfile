ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# WiFi scanners: nmcli asks the host's NetworkManager over the mounted system
# D-Bus socket; iwlist/iw scan directly (host network, NET_ADMIN to trigger a
# fresh scan). The NetworkManager daemon itself never runs in the container.
RUN apt update && apt install -y --no-install-recommends \
    network-manager \
    wireless-tools \
    iw \
    && rm -rf /var/lib/apt/lists/*

# Vendored kootenpv/whereami (AGPL-3.0: /opt/whereami/LICENSE) and its
# dependencies, pinned; numpy is the ros2 base venv's version.
COPY whereami/ /opt/whereami/
RUN . /venv/bin/activate && pip install --no-cache-dir \
      numpy==2.5.3 \
      scipy==1.18.1 \
      scikit-learn==1.9.1 \
      joblib==1.6.0 \
      threadpoolctl==3.7.0 \
      narwhals==2.26.0 \
      tqdm==4.70.1 \
      access_points==0.4.73 && \
    pip install --no-cache-dir --no-deps --no-build-isolation /opt/whereami && \
    pip check

COPY scripts/ /scripts/
RUN chmod +x /scripts/whereami_node.py /scripts/whereami_cli.py

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

COPY --from=build / /

WORKDIR /scripts

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "python3", "/scripts/whereami_node.py" ]
