cmake_minimum_required(VERSION 3.8)
project(parking_lot_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  behavior_trees
  models
  maps
  config
  launch
  rviz
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gz_gui.sh
  DESTINATION lib/${PROJECT_NAME}
)

# GZ_SIM_RESOURCE_PATH 에 models / worlds 자동 등록
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
