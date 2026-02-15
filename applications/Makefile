# Container build system

# Environmental variables
.EXPORT_ALL_VARIABLES:
SHELL := /bin/bash
NAMESPACE := cyberphysics
DOCKER_BUILDKIT := 1
PLATFORM := amd64

# No files to check for these targets
.PHONY: help cleanup build

# Helper
help:
	@echo ''
	@echo 'Usage: make [OPTION]'
	@echo ''
	@echo 'BUILD BASE IMAGE:'
	@echo '    build_ros_base'
	@echo ''

# Remove windows line endings
cleanup:
	find . -iname '*.sh' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.py' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.js' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.cmake' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.txt' -exec sed -i -e 's/\r//' {} \;

target_arm64:
	$(eval PLATFORM=arm64)
	@echo ${PLATFORM}

build_aerostack2: cleanup
	APP=aerostack2 ./_build_it.sh
build_arduino: cleanup
	APP=arduino ./_build_it.sh
build_bcflight: cleanup
	APP=bcflight ./_build_it.sh
build_bcflightcontroller: cleanup
	APP=bcflightcontroller ./_build_it.sh
build_blackfly: cleanup build_ros2
	APP=blackfly ./_build_it.sh
build_cbfkit: cleanup
	APP=cbfkit ./_build_it.sh
build_comfyui: cleanup
	APP=comfyui ./_build_it.sh
build_dia: cleanup
	APP=dia ./_build_it.sh
build_dronedemo: cleanup
	APP=dronedemo ./_build_it.sh
build_gazebo: cleanup build_ros2
	APP=gazebo ./_build_it.sh
build_genesis: cleanup
	APP=genesis ./_build_it.sh
build_isaacsim: cleanup
	APP=isaacsim ./_build_it.sh
build_jupyter: cleanup
	APP=jupyter ./_build_it.sh
build_matlab: cleanup
	APP=matlab ./_build_it.sh
build_navigation2: cleanup build_ros2
	APP=navigation2 ./_build_it.sh
build_nicegui: cleanup build_ros2
	APP=nicegui ./_build_it.sh
build_nvblox: cleanup build_ros2
	APP=nvblox ./_build_it.sh
build_oak1: cleanup build_ros2
	APP=oak1 ./_build_it.sh
build_openclaw: cleanup
	APP=openclaw ./_build_it.sh
build_ozzmaker: cleanup build_ros2
	APP=ozzmaker ./_build_it.sh
build_pysimcoder: cleanup
	APP=pysimcoder ./_build_it.sh
build_qgroundcontrol: cleanup
	APP=qgroundcontrol ./_build_it.sh
build_racecarneo: cleanup
	APP=racecarneo ./_build_it.sh
build_rai: cleanup
	APP=rai ./_build_it.sh
build_realsense: cleanup
	APP=realsense ./_build_it.sh
build_record: cleanup build_ros2
	APP=record ./_build_it.sh
build_roboticstoolbox: cleanup
	APP=roboticstoolbox ./_build_it.sh
build_robotransformers: cleanup
	APP=robotransformers ./_build_it.sh
build_ros2: cleanup
	APP=ros2 ./_build_it.sh
build_rtx2: cleanup
	APP=rtx2 ./_build_it.sh
build_sdfnmpc: cleanup
	APP=sdfnmpc ./_build_it.sh
build_simstudio: cleanup
	APP=simstudio ./_build_it.sh
build_so101: cleanup
	APP=so101 ./_build_it.sh
build_spaceros: cleanup
	APP=spaceros ./_build_it.sh
build_tello: cleanup build_ros2
	APP=tello ./_build_it.sh
build_unrealengine: cleanup
	APP=unrealengine ./_build_it.sh
build_videpth: cleanup
	APP=videpth ./_build_it.sh
build_vinsmono: cleanup
	APP=vinsmono ./_build_it.sh
build_whereami: cleanup
	APP=whereami ./_build_it.sh
build_wildnav: cleanup
	APP=wildnav ./_build_it.sh

# Run rviz in ROS2 container with native display mounting
run_ros2:
	xhost +
	docker run -it --rm \
	--gpus=all \
	--privileged \
	--network=host \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix \
	--env="DISPLAY=$(DISPLAY)" \
	$(NAMESPACE)/ros2:latest rviz2
