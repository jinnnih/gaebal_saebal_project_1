#!/usr/bin/env bash
# 팀 시연 세팅 — 시뮬 + Nav2 + GUI + 액션 서버를 띄우고 카메라를 잡는다.
#
#   bash scripts/setup_demo.sh
#   (준비되면)  bash scripts/demo.sh A08
#
# ! GUI 는 소프트웨어 렌더링이라 3 코어쯤 먹는다. 부하가 올라가면 제어 루프가
#   주기를 못 맞춰 주차 정확도가 흔들린다 (실측: 부하 25/10 에서 방향오차
#   -34 deg). 시연 전에 부하를 확인할 것. 정확도가 중요하면 GUI 없이 돌리고
#   결과만 보여주는 편이 낫다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "[1/5] 기존 프로세스 정리"
bash "$WS/scripts/kill_sim.sh"
pkill -f 'lib/valet[_]robot/park' 2>/dev/null
sleep 3

echo "[2/5] 시뮬 + Nav2 기동"
setsid nohup bash "$WS/scripts/run_sim.sh" nav2:=true use_costmap_filters:=true \
  > /tmp/demo_sim.log 2>&1 < /dev/null &
disown
for i in $(seq 1 80); do
  sleep 3
  grep -aq "Activating velocity_smoother" /tmp/demo_sim.log && \
    { echo "      Nav2 준비 ($((i*3)) s)"; break; }
done

echo "[3/5] Gazebo GUI"
setsid nohup bash "$WS/scripts/show_gui.sh" > /tmp/demo_gui.log 2>&1 < /dev/null &
disown
sleep 25

echo "[4/5] 주차/출차 액션 서버"
setsid nohup ros2 run valet_robot park_action_server.py \
  > /tmp/demo_action.log 2>&1 < /dev/null &
disown
sleep 10
tail -1 /tmp/demo_action.log

echo "[5/5] 카메라를 주차 구역으로"
# 남통로 남쪽에서 A 행을 내려다본다 (주차 장면이 잘 보이는 각도)
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req "pose: {position: {x: 0.7, y: -30.0, z: 12.0}, orientation: {x: -0.24468, y: 0.24496, z: 0.66299, w: 0.66375}}" \
  > /dev/null 2>&1
bash "$WS/scripts/nav2_tests/reset_pose.sh"

echo
echo "=== 상태 ==="
printf '  gz sim   %s\n' "$(pgrep -fc 'gz[ ]sim' 2>/dev/null; true)"
printf '  nav2     %s\n' "$(pgrep -fc 'lib/nav2[_]' 2>/dev/null; true)"
printf '  액션서버 %s\n' "$(pgrep -fc 'lib/valet[_]robot/park' 2>/dev/null; true)"
uptime | sed 's/.*load/  부하    /'
echo
echo "준비 완료.  시작하려면:  bash scripts/demo.sh A08"
