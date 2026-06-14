cmake_minimum_required(VERSION 3.8)
project(so101_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gazebo_command_relay.py
    scripts/genesis_so101.py
    scripts/isaacsim_so101.py
    scripts/so101_cbf_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/so101_common.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
