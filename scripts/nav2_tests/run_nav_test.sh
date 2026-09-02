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
# ! 준비 확인은 로그로 한다. ros2 lifecycle get 을 반복 호출하면 안 된다.
#   CLI 는 호출마다 DDS 참가자를 새로 만드는데, 노드 20 개가 떠 있고 CPU 가
#   포화된 상태에서는 한 번에 수 초씩 걸린다. 80 회 돌리면 12 분이 넘어가
#   바깥 timeout 이 스크립트를 통째로 죽인다. Nav2 는 멀쩡히 떠 있는데도
#   "활성화 대기" 에서 끝난 것처럼 보인다. (2026-09-02 실측)
READY=""
for i in $(seq 1 120); do
  sleep 2
  if grep -aq "Activating velocity_smoother" /tmp/nav_test.log 2>/dev/null; then
    READY=yes; echo "  준비 완료 ($((i*2)) s)"; break
  fi
done
[ -z "$READY" ] && echo "  ! 240 s 안에 활성화 못 함 — 그대로 진행"
# 상태 조회는 딱 한 번만 (비싸다)
timeout 30 ros2 lifecycle get -a 2>/dev/null | sed 's/^/  /' | head -14
sleep 5

echo
echo "########## 경로 검사 ##########"
timeout 120 python3 "$WS/scripts/nav2_tests/path_check.py" "$GX" "$GY"
echo
echo "########## 주행 시험 ##########"
timeout 280 python3 "$WS/scripts/nav2_tests/nav_goal_test.py" "$GX" "$GY" 0
