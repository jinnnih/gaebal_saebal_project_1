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
)
for p in "${PATS[@]}"; do pkill -f "$p" 2>/dev/null; done
sleep 2
for p in "${PATS[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 2
LEFT=$(pgrep -fc 'gz[ ]sim' 2>/dev/null || echo 0)
echo "남은 gz 프로세스: ${LEFT:-0}"
[ "${LEFT:-0}" != "0" ] && pgrep -fa 'gz[ ]sim' | sed 's/^/  /'
exit 0
