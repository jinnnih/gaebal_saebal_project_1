#!/usr/bin/env bash
# 팀 시연용 — 입차 -> 주차 -> 출차 -> 출구 전 주기를 한 번에 돌린다.
#
#   bash scripts/demo.sh [주차면ID]      기본 A08
#
# 사전 조건 (setup_demo.sh 가 해 둔다):
#   - run_sim.sh nav2:=true 로 시뮬 + Nav2 가 떠 있을 것
#   - park_action_server.py 가 떠 있을 것
#   - Gazebo GUI 가 떠 있을 것 (눈으로 볼 용도)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SPOT="${1:-A08}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

line() { printf '\n\033[1;36m%s\033[0m\n' "$1"; }

line "[0/3] 로봇을 입구로 되돌린다"
bash "$WS/scripts/nav2_tests/reset_pose.sh"

line "[1/3] 입차 -> 주차  (주차면 $SPOT)"
echo "      Nav2 로 통로까지 -> 미세정렬 -> 후진 선회 -> 중심선 추종 후진"
python3 -u "$WS/scripts/nav2_tests/park_demo.py" "$SPOT" || {
  echo "주차 실패 — 중단"; exit 1; }

line "[2/3] 잠시 정차 (주차 상태 확인)"
sleep 8

line "[3/3] 출차 -> 출구  (전진만 사용)"
echo "      뒤로 넣었으므로 앞으로 나온다 = 계획서의 '출차는 항상 전진'"
python3 -u "$WS/scripts/nav2_tests/park_demo.py" "$SPOT" unpark

line "시연 끝"
