ARG TAG=latest
FROM salsicha/cyberphysics:aerostack2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

SHELL ["/bin/bash", "-c"]

COPY scripts/ /scripts/

# Reuse the aerostack2 base venv: it is created with --system-site-packages.
# Re-running `python3 -m venv /venv` here rewrote pyvenv.cfg with
# include-system-site-packages = false, hiding the apt Python packages.
RUN grep -qx 'include-system-site-packages = true' /venv/pyvenv.cfg
RUN . /venv/bin/activate && pip3 install --no-cache-dir 'torch>=2.8,<2.9' --index-url https://download.pytorch.org/whl/cu128
RUN . /venv/bin/activate && pip3 install --no-cache-dir genesis-world==1.3.3
# genesis-world brings NumPy 2 into the venv, but genesis imports matplotlib
# and the apt one (3.5, built against NumPy 1.21 and visible through the
# system site packages) fails to import with it ("numpy.core.multiarray
# failed to import"). Give the venv a NumPy 2 build.
RUN . /venv/bin/activate && pip3 install --no-cache-dir matplotlib==3.10.9 && \
    cd / && python3 -c "import genesis"

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
