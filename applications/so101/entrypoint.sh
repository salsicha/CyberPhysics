#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# The base image's ROS setup predates vendor packages installed in this image,
# so add their runtime prefixes explicitly.
for vendor_prefix in "/opt/ros/${ROS_DISTRO:-jazzy}"/opt/*; do
    if [ -d "$vendor_prefix/bin" ]; then
        PATH="$vendor_prefix/bin:$PATH"
    fi
    if [ -d "$vendor_prefix/lib" ]; then
        LD_LIBRARY_PATH="$vendor_prefix/lib:${LD_LIBRARY_PATH:-}"
    fi
    if [ -d "$vendor_prefix/lib64" ]; then
        LD_LIBRARY_PATH="$vendor_prefix/lib64:${LD_LIBRARY_PATH:-}"
    fi
    if [ -d "$vendor_prefix/share/gz" ]; then
        GZ_CONFIG_PATH="$vendor_prefix/share/gz:${GZ_CONFIG_PATH:-}"
    fi
done
GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/${ROS_DISTRO:-jazzy}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export PATH LD_LIBRARY_PATH GZ_CONFIG_PATH GZ_SIM_SYSTEM_PLUGIN_PATH

source /workspace/install/setup.bash
exec "$@"
