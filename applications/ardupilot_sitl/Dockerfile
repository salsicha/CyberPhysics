ARG TAG=amd64
ARG ARDUPILOT_BASE_TAG=v0.2.0

FROM ardupilot/ardupilot-dev-base:${ARDUPILOT_BASE_TAG} AS build

# Must match ARDUPILOT_REF in systems/airplane/config/sitl.env (the param
# linter, tools/check_ardupilot_params.py, validates against that pin).
ARG ARDUPILOT_REF=Plane-4.6.3
ARG JSBSIM_REF=v1.2.2

USER root
SHELL ["/bin/bash", "-c"]

# ArduPlane SITL built at the pinned ref, so containers boot offline in
# seconds instead of cloning and compiling on first start.
RUN git clone --recurse-submodules --depth=1 --branch "${ARDUPILOT_REF}" \
        https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
RUN ./waf configure --board sitl && ./waf plane

# JSBSim for the optional higher-fidelity airframe model
# (AIRPLANE_SITL_MODEL=jsbsim).
RUN git clone --depth=1 --branch "${JSBSIM_REF}" \
        https://github.com/JSBSim-Team/jsbsim.git /jsbsim && \
    cmake -S /jsbsim -B /jsbsim/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /jsbsim/build -j"$(nproc)" --target JSBSim && \
    rm -rf /jsbsim/build/CMakeFiles

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
