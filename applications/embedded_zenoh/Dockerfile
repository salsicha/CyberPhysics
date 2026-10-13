ARG TAG=latest
FROM salsicha/cyberphysics:ros2-${TAG}
SHELL ["/bin/bash", "-c"]
ARG PICOROS_REF=b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2
ARG PICO_SDK_REF=a1438dff1d38bd9c65dbd693f0e5db4b9ae91779
ARG FREERTOS_REF=88971102bf8aa34b4fcc24f176db620dd9c854f7
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/Pico-ROS/Pico-ROS-software.git /opt/picoros && \
    cd /opt/picoros && git checkout "${PICOROS_REF}" && \
    git config submodule.thirdparty/zenoh-pico.url https://github.com/g4sp3r/zenoh-pico.git && \
    git submodule update --init --depth 1 thirdparty/zenoh-pico thirdparty/Micro-CDR
RUN git clone https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk && \
    cd /opt/pico-sdk && git checkout "${PICO_SDK_REF}" && git submodule update --init --depth 1
RUN git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git /opt/FreeRTOS-Kernel && \
    cd /opt/FreeRTOS-Kernel && git checkout "${FREERTOS_REF}" && git submodule update --init --depth 1
ENV PICO_SDK_PATH=/opt/pico-sdk FREERTOS_KERNEL_PATH=/opt/FreeRTOS-Kernel
COPY . /workspace/embedded_zenoh
RUN git -C /opt/picoros/thirdparty/zenoh-pico apply /workspace/embedded_zenoh/patches/*.patch && \
    cmake -S /workspace/embedded_zenoh -B /opt/embedded-host -G Ninja && \
    cmake --build /opt/embedded-host -j 4 && \
    cmake -S /workspace/embedded_zenoh -B /opt/embedded-pico -G Ninja -DCP_PICO=ON && \
    cmake --build /opt/embedded-pico -j 4
CMD ["/opt/embedded-host/embedded_zenoh", "tcp/127.0.0.1:7448", "0"]
