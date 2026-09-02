#!/usr/bin/env bash
# 로봇을 Gazebo 에서 옮기고 AMCL 도 같이 재초기화한다.
#
# ! gz set_pose 만 하면 AMCL 의 map->odom 이 옛 위치를 가리켜서
#   플래너가 엉뚱한 곳을 시작점으로 잡고 "Start occupied" 로 실패한다.
#   (실측으로 걸린 함정 — 이걸 회귀로 오인했었다)
# ! set_pose 가 조용히 안 먹는 경우가 있어 /odom 으로 확인하고 재시도한다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# 기본값은 parking_spots.json 의 entry_pose. 기하가 바뀌어도 따라간다.
read -r DX DY < <(python3 -c "
import json,os,subprocess
try:
    s=subprocess.check_output(['ros2','pkg','prefix','--share','parking_lot_world'],text=True).strip()
except Exception:
    s=os.path.join('$WS','src','parking_lot_world')
d=json.load(open(os.path.join(s,'config','parking_spots.json'),encoding='utf-8'))
print(d['entry_pose'][0], d['entry_pose'][1])")
X="${1:-$DX}"; Y="${2:-$DY}"; YAW="${3:-0.0}"
QZ=$(python3 -c "import math;print(math.sin($YAW/2))")
QW=$(python3 -c "import math;print(math.cos($YAW/2))")

for try in 1 2 3; do
  gz service -s /world/parking_lot/set_pose --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"valet_car\", position: {x: $X, y: $Y, z: 0.06}, orientation: {z: $QZ, w: $QW}" \
    > /dev/null 2>&1
  sleep 2
  OK=$(timeout 8 python3 -c "
import math,rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n=rclpy.create_node('rp')
got=[]
n.create_subscription(Odometry,'/odom',lambda m: got.append(m),10)
import time; t=time.time()
while time.time()-t<5 and not got: rclpy.spin_once(n,timeout_sec=0.2)
if got:
    p=got[-1].pose.pose.position
    print('1' if math.dist((p.x,p.y),($X,$Y))<0.5 else '0')
else:
    print('?')
" 2>/dev/null)
  [ "$OK" = "1" ] && break
  echo "  set_pose 미반영 (시도 $try) — 재시도"
done

ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" > /dev/null 2>&1
sleep 3
echo "리셋 완료 -> ($X, $Y, $YAW rad)  [gz=$OK, AMCL 재초기화]"
