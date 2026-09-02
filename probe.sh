#!/usr/bin/env bash
# 주행 계측(smoke_probe) 실행
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"
exec python3 "$(ros2 pkg prefix valet_robot)/share/valet_robot/tools/smoke_probe.py"
