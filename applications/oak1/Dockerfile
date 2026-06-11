ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

COPY requirements.txt /

RUN --mount=type=cache,target=/root/.cache/pip \
    . /venv/bin/activate && \
    pip install --timeout 300 --retries 10 -r /requirements.txt

COPY scripts/ /scripts/
COPY models/ /models/
WORKDIR /scripts

RUN test "$(stat -c %s /models/megadepth_192x256_openvino_2021.4_6shave.blob)" -gt 1000000
RUN chmod +x /scripts/megadepth_ros.py

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]
