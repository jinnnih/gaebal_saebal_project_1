cmake_minimum_required(VERSION 3.8)
project(valet_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  launch
  rviz
  tools
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

ament_package()
