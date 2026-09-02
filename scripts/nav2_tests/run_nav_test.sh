#!/usr/bin/env bash
# Nav2 를 띄우고 준비될 때까지 기다린 뒤 경로검사 + 주행시험을 돌린다.
#   bash scripts/nav2_tests/run_nav_test.sh [goal_x] [goal_y]
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GX="${1:-0.0}"; GY="${2:-0.0}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

bash "$WS/scripts/kill_sim.sh" > /dev/null 2>&1
sleep 4
setsid nohup "$WS/scripts/run_sim.sh" nav2:=true use_costmap_filters:=true \
  > /tmp/nav_test.log 2>&1 < /dev/null &

echo "Nav2 활성화 대기 (최대 240 s)..."
for i in $(seq 1 80); do
  sleep 3
  ST=$(timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
  case "$ST" in *active*) echo "  준비 완료 ($((i*3)) s)"; break;; esac
done
for n in map_server amcl planner_server controller_server bt_navigator velocity_smoother; do
  printf "  %-18s %s\n" "$n" "$(timeout 6 ros2 lifecycle get /$n 2>/dev/null | head -1)"
done
sleep 5

echo
echo "########## 경로 검사 ##########"
timeout 120 python3 "$WS/scripts/nav2_tests/path_check.py" "$GX" "$GY"
echo
echo "########## 주행 시험 ##########"
timeout 280 python3 "$WS/scripts/nav2_tests/nav_goal_test.py" "$GX" "$GY" 0
