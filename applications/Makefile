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
build_blackfly: cleanup build_ros2
	APP=blackfly ./_build_it.sh
build_boat: cleanup build_aerostack2
	APP=boat ./_build_it.sh
build_depthanything: cleanup build_ros2
	APP=depthanything ./_build_it.sh
build_gazebo: cleanup build_ros2
	APP=gazebo ./_build_it.sh
build_genesis: cleanup build_aerostack2
	APP=genesis ./_build_it.sh
build_groot: cleanup
	APP=groot ./_build_it.sh
build_demnav: cleanup build_aerostack2
	APP=demnav ./_build_it.sh
build_matlab: cleanup build_ros2
	APP=matlab ./_build_it.sh
build_marimo: cleanup
	APP=marimo ./_build_it.sh
build_lerobot: cleanup
	APP=so101 TARGET=lerobot BUILD_STAGE=lerobot ./_build_it.sh
build_navigation2: cleanup build_ros2
	APP=navigation2 ./_build_it.sh
build_navigator: cleanup
	APP=navigator ./_build_it.sh
build_nicegui: cleanup build_ros2
	APP=nicegui ./_build_it.sh
build_nvblox: cleanup build_ros2
	APP=nvblox ./_build_it.sh
build_oak1: cleanup build_aerostack2
	APP=oak1 ./_build_it.sh
build_pantiltros: cleanup
	APP=pantiltros ./_build_it.sh
build_pysimcoder: cleanup
	APP=pysimcoder ./_build_it.sh
build_qgroundcontrol: cleanup build_ros2
	APP=qgroundcontrol ./_build_it.sh
build_rai: cleanup
	APP=rai ./_build_it.sh
build_rangefinder: cleanup build_ros2
	APP=rangefinder ./_build_it.sh
build_realsense: cleanup build_ros2
	APP=realsense ./_build_it.sh
build_racecarneo: cleanup build_ros2
	APP=racecarneo ./_build_it.sh
build_record: cleanup build_ros2
	APP=record ./_build_it.sh
build_roboticstoolbox: cleanup build_ros2
	APP=roboticstoolbox ./_build_it.sh
build_ros2: cleanup
	APP=ros2 ./_build_it.sh
build_simstudio: cleanup
	APP=simstudio ./_build_it.sh
build_so101: cleanup
	APP=so101 ./_build_it.sh
build_spaceros: cleanup
	APP=spaceros ./_build_it.sh
build_submarine: cleanup build_aerostack2
	APP=submarine ./_build_it.sh
build_tello: cleanup build_ros2
	APP=tello ./_build_it.sh
build_unrealengine: cleanup build_ros2
	APP=unrealengine ./_build_it.sh
build_vinsmono: cleanup build_ros2
	APP=vinsmono ./_build_it.sh
build_wildnav: cleanup build_aerostack2
	APP=wildnav ./_build_it.sh

# Run rviz in ROS2 container with native display mounting
run_ros2:
	xhost +local:docker
	docker run -it --rm \
	--gpus=all \
	--privileged \
	--network=host \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix \
	--env="DISPLAY=$(DISPLAY)" \
	$(NAMESPACE)/ros2:latest rviz2
