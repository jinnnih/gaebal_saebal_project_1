#!/usr/bin/env bash
# 시뮬 관련 프로세스 전부 정리.
# 패턴에 대괄호를 써서 이 스크립트/호출자 자신이 매치되지 않게 한다.
# 워크스페이스 경로는 스크립트 위치에서 구한다 (어디에 체크아웃하든 동작).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PATS=(
  'gz[ ]sim'
  'ruby.*gz[ ]tools'
  'valet[_]sim.launch'
  'spawn[_]valet_car'
  'parking[_]world.launch'
  'nav2[_]valet'
  'parameter[_]bridge'
  'robot[_]state_publisher'
  'twist[_]to_ackermann'
  'ros2[_]control_node'
  'controller[_]manager'
  'smoke[_]probe'
  'demo[_]drive'
  # Nav2 노드들. ros2 launch 를 죽여도 자식 노드가 살아남아 좀비가 된다.
  # 좀비 lifecycle_manager 가 여러 개 뜨면 서로 노드를 뺏으며 부트가
  # 무한 대기에 빠진다 (실측: manager 3개가 동시에 떠 있었다).
  'lifecycle[_]manager'
  'nav2[_]map_server/map_server'
  'nav2[_]amcl/amcl'
  'nav2[_]planner/planner_server'
  'nav2[_]controller/controller_server'
  'nav2[_]smoother/smoother_server'
  'nav2[_]behaviors/behavior_server'
  'nav2[_]bt_navigator/bt_navigator'
  'nav2[_]velocity_smoother/velocity_smoother'
)
for p in "${PATS[@]}"; do pkill -f "$p" 2>/dev/null; done
sleep 2
for p in "${PATS[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 2
LEFT=$(pgrep -fc 'gz[ ]sim' 2>/dev/null || echo 0)
NAV=$(pgrep -fc 'lifecycle[_]manager' 2>/dev/null || echo 0)
echo "남은 프로세스 — gz: ${LEFT:-0}, nav2 lifecycle_manager: ${NAV:-0}"
[ "${LEFT:-0}" != "0" ] && pgrep -fa 'gz[ ]sim' | sed 's/^/  /'
exit 0
