# Container build system

# Environmental variables
.EXPORT_ALL_VARIABLES:
SHELL := /bin/bash
# Docker Hub repository; each app is a tag: <app>-latest (amd64) or <app>-arm64
REPOSITORY := salsicha/cyberphysics
DOCKER_BUILDKIT := 1
PLATFORM := amd64

# Every build_<app> target defined below
BUILD_TARGETS := $(shell sed -n 's/^\(build_[a-z0-9_]*\):.*/\1/p' $(lastword $(MAKEFILE_LIST)))

# No files to check for these targets
.PHONY: help cleanup target_arm64 run_ros2 $(BUILD_TARGETS)

# Helper
help:
	@echo ''
	@echo 'Usage: make [target_arm64] build_<app> [NOCACHE=1]'
	@echo ''
	@echo 'BUILD TARGETS (each one builds its base image first):'
	@for t in $(sort $(BUILD_TARGETS)); do echo "    $$t"; done
	@echo ''
	@echo 'OTHER TARGETS:'
	@echo '    target_arm64   build the following targets for linux/arm64 (tagged :arm64)'
	@echo '    cleanup        strip Windows line endings from sources under applications/'
	@echo '    run_ros2       run rviz2 from the ros2 image on the host display'
	@echo ''

# Remove windows line endings (explicit maintenance target; .gitattributes
# already normalises line endings on checkout, so builds do not run this)
cleanup:
	find . -iname '*.sh' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.py' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.js' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.cmake' -exec sed -i -e 's/\r//' {} \;
	find . -iname '*.txt' -exec sed -i -e 's/\r//' {} \;

target_arm64:
	$(eval PLATFORM=arm64)
	@echo ${PLATFORM}

build_aerostack2:
	APP=aerostack2 ./_build_it.sh
build_arduino:
	APP=arduino ./_build_it.sh
build_blackfly: build_ros2
	APP=blackfly ./_build_it.sh
build_ardupilot_sitl:
	APP=ardupilot_sitl ./_build_it.sh
build_boat: build_aerostack2
	APP=boat ./_build_it.sh
build_cactusrt:
	APP=cactusrt ./_build_it.sh
build_depthanything: build_ros2
	APP=depthanything ./_build_it.sh
build_gazebo: build_ros2
	APP=gazebo ./_build_it.sh
build_genesis: build_aerostack2
	APP=genesis ./_build_it.sh
build_isaac:
	APP=isaac ./_build_it.sh
build_groot:
	APP=groot ./_build_it.sh
build_smolvla:
	APP=smolvla ./_build_it.sh
build_demnav: build_aerostack2
	APP=demnav ./_build_it.sh
build_matlab:
	APP=matlab ./_build_it.sh
build_marimo:
	APP=marimo ./_build_it.sh
build_lerobot:
	APP=so101 TARGET=lerobot BUILD_STAGE=lerobot ./_build_it.sh
build_navigation2: build_ros2
	APP=navigation2 ./_build_it.sh
build_navigator: build_ardupilot_sitl
	APP=navigator ./_build_it.sh
build_nicegui: build_ros2
	APP=nicegui ./_build_it.sh
build_nvblox: build_ros2
	APP=nvblox ./_build_it.sh
build_oak1: build_aerostack2
	APP=oak1 ./_build_it.sh
build_pantiltros:
	APP=pantiltros ./_build_it.sh
build_pysimcoder:
	APP=pysimcoder ./_build_it.sh
build_qgroundcontrol: build_ros2
	APP=qgroundcontrol ./_build_it.sh
build_rai:
	APP=rai ./_build_it.sh
build_rangefinder: build_ros2
	APP=rangefinder ./_build_it.sh
build_realsense: build_ros2
	APP=realsense ./_build_it.sh
build_racecarneo: build_ros2
	APP=racecarneo ./_build_it.sh
build_record: build_ros2
	APP=record ./_build_it.sh
build_roboticstoolbox: build_ros2
	APP=roboticstoolbox ./_build_it.sh
build_ros2:
	APP=ros2 ./_build_it.sh
build_simstudio:
	APP=simstudio ./_build_it.sh
build_so101:
	APP=so101 ./_build_it.sh
build_spaceros:
	APP=spaceros ./_build_it.sh
build_submarine: build_aerostack2
	APP=submarine ./_build_it.sh
build_tello: build_ros2
	APP=tello ./_build_it.sh
build_turret_sim: build_pantiltros
	APP=turret_sim ./_build_it.sh
build_unrealengine: build_ros2
	APP=unrealengine ./_build_it.sh
build_vinsmono:
	APP=vinsmono ./_build_it.sh
build_whereami: build_ros2
	APP=whereami ./_build_it.sh
build_wildnav: build_aerostack2
	APP=wildnav ./_build_it.sh

# Run rviz in ROS2 container with native display mounting
run_ros2:
	xhost +SI:localuser:root
	docker run -it --rm \
	--gpus=all \
	--privileged \
	--network=host \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix \
	--env="DISPLAY=$(DISPLAY)" \
	$(REPOSITORY):ros2-latest rviz2
