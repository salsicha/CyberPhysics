# MATLAB/ROS Noetic

FROM cyberphysics/ros2:latest

# setup environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/tmp/matlab/bin/glnxa64/"

RUN apt update && apt install -q -y --no-install-recommends \
    wget unzip ca-certificates tini vim libudev-dev \
    sudo libxft-dev default-jre xfce4 libnss3 libgbm1 \
    glibc-source \
    && apt autoremove -y && apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY network.lic /network.lic

# Patch glibc for Simulink
# https://www.mathworks.com/support/bugreports/2632298
COPY glibc /tmp/glibc
WORKDIR /tmp/glibc
RUN dpkg -i libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_amd64.deb
RUN dpkg -i libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_amd64.deb
RUN dpkg -i libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_amd64.deb
RUN dpkg -i libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_amd64.deb

RUN wget -q https://www.mathworks.com/mpm/glnxa64/mpm && \ 
    chmod +x mpm && \
    ./mpm install \
        --release=R2022b \
        --destination=/tmp/matlab \
        --products MATLAB Simulink Aerospace_Toolbox Aerospace_Blockset Navigation_Toolbox UAV_Toolbox \
            Automated_Driving_Toolbox Computer_Vision_Toolbox DSP_System_Toolbox Deep_Learning_Toolbox \
            Fixed-Point_Designer Image_Acquisition_Toolbox Image_Processing_Toolbox ROS_Toolbox \
            Lidar_Toolbox MATLAB_Report_Generator Mapping_Toolbox Optimization_Toolbox Control_System_Toolbox \
            Parallel_Computing_Toolbox Phased_Array_System_Toolbox Radar_Toolbox Stateflow Simulink_Control_Design \
            Reinforcement_Learning_Toolbox Robotics_System_Toolbox Sensor_Fusion_and_Tracking_Toolbox \
            Signal_Processing_Toolbox Simulink_Report_Generator Statistics_and_Machine_Learning_Toolbox && \
    rm -f mpm /tmp/mathworks_root.log && \
    ln -s /tmp/matlab/bin/matlab /usr/local/bin/matlab

RUN pip install matlabengine==9.13.7

RUN adduser --shell /bin/bash --disabled-password --gecos "" matlab \
    && echo "matlab ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/matlab \
    && chmod 0440 /etc/sudoers.d/matlab

ENV MW_DDUX_FORCE_ENABLE=true MW_CONTEXT_TAGS=MATLAB:DOCKERFILE:V1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

USER matlab
RUN mkdir /home/matlab/
WORKDIR /home/matlab/
CMD ["matlab"]
