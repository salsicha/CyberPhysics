# JUPYTER

FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt && \
    rm -rf /root/.cache

RUN mkdir -p /notebooks

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /notebooks

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter-lab", "--ip", "0.0.0.0", "--no-browser", \
    "--allow-root", "--ServerApp.token=docker_jupyter", \
    "--NotebookApp.allow_password_change=False"]
