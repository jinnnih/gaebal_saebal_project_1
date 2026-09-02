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
  # ! 노드를 하나씩 나열하지 않는다. 실행파일 이름이 노드 이름과 달라서
  #   빠뜨리기 쉽다. 실제로 nav2_map_server/costmap_filter_info_server 가
  #   목록에 없어 살아남았고, 다음 실행에서 같은 노드 이름이 두 번 떠
  #   부트가 map_server 단계에서 무한 대기에 빠졌다. (2026-09-02 실측)
  #   nav2 는 전부 .../lib/nav2_* 에 설치되므로 그 경로로 한 번에 잡는다.
  'lib/nav2[_]'
)
for p in "${PATS[@]}"; do pkill -f "$p" 2>/dev/null; done
sleep 2
for p in "${PATS[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 2
# ! `|| echo 0` 을 붙이면 안 된다. pgrep -fc 는 하나도 없을 때
#   "0" 을 출력하면서 종료코드 1 을 내므로 0 이 두 번 찍힌다.
LEFT=$(pgrep -fc 'gz[ ]sim' 2>/dev/null; true)
NAV=$(pgrep -fc 'lib/nav2[_]' 2>/dev/null; true)
# ! 죽은 DDS 공유메모리 세그먼트를 치운다.
#   시뮬을 반복해서 죽이면 /dev/shm/fastrtps_* 가 계속 쌓인다.
#   181 개까지 쌓였을 때 FastDDS 디스커버리가 조용히 실패해서,
#   nav2 노드는 전부 뜨는데 lifecycle_manager 가 로그 한 줄 못 내고
#   부팅이 무한 대기에 빠졌다. 에러도 안 난다. (2026-09-02 실측)
#   ROS 프로세스가 남아 있으면 건드리지 않는다 (쓰는 중일 수 있다).
if [ "${NAV:-0}" = "0" ] && ! pgrep -f '/opt/ros/' > /dev/null 2>&1; then
  SHM=$(ls /dev/shm 2>/dev/null | grep -c '^s\?e\?m\?\.\?fast')
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*         /dev/shm/fast_datasharing* /dev/shm/sem.fast_* 2>/dev/null
  [ "${SHM:-0}" != "0" ] && echo "DDS 공유메모리 정리 — ${SHM} 개"
fi
echo "남은 프로세스 — gz: ${LEFT:-0}, nav2: ${NAV:-0}"
[ "${LEFT:-0}" != "0" ] && pgrep -fa 'gz[ ]sim' | sed 's/^/  /'
[ "${NAV:-0}" != "0" ] && pgrep -fa 'lib/nav2[_]' | sed 's|.*/lib/|  |; s/ .*//'
exit 0
