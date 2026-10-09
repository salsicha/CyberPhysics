ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# Serial driver for the Benewake TFmini-S / TF-Luna UART stream
RUN . /venv/bin/activate && pip install --no-cache-dir pyserial==3.5

COPY scripts/ /scripts/
RUN chmod +x /scripts/rangefinder_node.py /scripts/fake_benewake.py

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

WORKDIR /scripts

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "python3", "/scripts/rangefinder_node.py" ]
