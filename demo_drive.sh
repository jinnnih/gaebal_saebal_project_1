#!/usr/bin/env bash
# 보여주기용 연속 주행 데모 실행
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"
exec python3 "$HOME/valet_parking_ws/demo_drive.py"
