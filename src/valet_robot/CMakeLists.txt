cmake_minimum_required(VERSION 3.8)
project(valet_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  launch
  rviz
  tools
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# 실행 권한이 붙은 채 install 된다 (Windows 공유폴더에서 복사해도 안전)
install(PROGRAMS
  scripts/twist_to_ackermann.py
  scripts/ackermann_teleop_key.py
  tools/check_model.py
  tools/smoke_probe.py
  tools/preview_model.py
  DESTINATION lib/${PROJECT_NAME}
)

# GZ_SIM_RESOURCE_PATH 에 install/share 를 등록한다.
# URDF 의 package://valet_robot/meshes/... 를 Gazebo 가 찾을 수 있게 하려면
# share 의 부모가 아니라 share 자체가 들어가야 한다
# (gz 가 package:// 의 스킴을 떼고 valet_robot/meshes/... 로 검색한다).
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
