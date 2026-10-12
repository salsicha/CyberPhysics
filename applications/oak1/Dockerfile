ARG TAG=latest

FROM salsicha/cyberphysics:aerostack2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

COPY requirements.txt /

RUN --mount=type=cache,target=/root/.cache/pip \
    . /venv/bin/activate && \
    pip install --timeout 300 --retries 10 -r /requirements.txt

COPY scripts/ /scripts/
COPY models/ /models/
WORKDIR /scripts

RUN test "$(stat -c %s /models/megadepth_192x256_openvino_2021.4_6shave.blob)" -gt 1000000
RUN chmod +x /scripts/megadepth_ros.py /scripts/sim_tracking_detector.py

# The tracking models in /models are real (not Git LFS pointers), build
# into the OAK-1 pipeline and run in ONNX Runtime; vision_msgs is there.
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && . /venv/bin/activate && \
    python3 -c "from vision_msgs.msg import Detection2DArray" && \
    python3 /scripts/check_tracking_models.py /models

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Containers start in / (the build steps above ran in /scripts)
WORKDIR /

ENTRYPOINT [ "/entrypoint.sh" ]
