#!/usr/bin/env bash
# 로봇을 스폰 포즈로 되돌린다 (반복 계측용)
source /opt/ros/jazzy/setup.bash
gz service -s /world/parking_lot/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "valet_car", position: {x: -23.0, y: -18.3, z: 0.05}, orientation: {w: 1.0}' \
  >/dev/null 2>&1
echo "로봇 리셋 -> (-23.0, -18.3, yaw 0)"
