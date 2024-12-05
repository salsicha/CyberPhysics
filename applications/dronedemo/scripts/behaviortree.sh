export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/root/.local/bin:$PATH
export PATH=/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:$PATH
export PATH=/usr/lib/ccache:$PATH

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.sh

export ROS_MASTER_URI=http://localhost:11311

python3 /workspace/misc/lamp_autonomy/scripts/drone.py


# Dugway
# lat: 40.06644051683807, lon: -113.061433065372, elevation 1324.061810551025, scale 14

# RFS
# lat: 37.916120, lon: -122.336566, elevation 5, scale 18

