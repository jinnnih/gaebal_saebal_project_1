#!/usr/bin/env bash
# 로봇을 Gazebo 에서 옮기고 AMCL 도 같이 재초기화한다.
#
# ! gz set_pose 만 하면 AMCL 의 map->odom 이 옛 위치를 가리켜서
#   플래너가 엉뚱한 곳을 시작점으로 잡고 "Start occupied" 로 실패한다.
#   (실측으로 걸린 함정)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
X="${1:--23.0}"; Y="${2:--18.3}"; YAW="${3:-0.0}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
QZ=$(python3 -c "import math;print(math.sin($YAW/2))")
QW=$(python3 -c "import math;print(math.cos($YAW/2))")
gz service -s /world/parking_lot/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"valet_car\", position: {x: $X, y: $Y, z: 0.06}, orientation: {z: $QZ, w: $QW}" \
  > /dev/null 2>&1
sleep 2
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" > /dev/null 2>&1
sleep 3
echo "리셋 완료 -> ($X, $Y, $YAW rad)  [gz + AMCL]"
