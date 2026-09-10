#!/usr/bin/env bash
# Nav2 라이프사이클 스택을 되살린다.
#
# 관리 노드의 bond 가 끊기면 lifecycle_manager 가 스택 전체를 리셋한다.
#
#   CRITICAL FAILURE: SERVER <node> IS DOWN after not receiving a heartbeat
#   for 4000 ms. Shutting down related nodes.
#
# 부하가 걸린 상태에서는 이어지는 자동 재기동이 중간에 실패하고
# (Failed to change state for node: map_server. Aborting bringup) 노드마다
# 상태가 제각각으로 남는다. 그러면 STARTUP 만 다시 불러도 소용없다.
# 매니저는 unconfigured 부터 순서대로 올리려 하는데 이미 active 인 노드가
# 있으면 그 전이가 거부돼서 또 중단되기 때문이다. RESET 으로 바닥을
# 맞춘 뒤 STARTUP 해야 한다.
#
# 증상: 모든 목표가 "Action server is inactive. Rejecting the goal" 로 거부됨.
set -u
MGR=${1:-/lifecycle_manager_navigation}
SRV=$MGR/manage_nodes
TYPE=nav2_msgs/srv/ManageLifecycleNodes

call() {  # $1: 명령 번호, $2: 이름
    printf '%-8s ' "$2"
    timeout 300 ros2 service call "$SRV" "$TYPE" "{command: $1}" 2>&1 \
        | grep -o 'success=[A-Za-z]*' | tail -1
}

call 3 RESET
sleep 5
call 0 STARTUP
sleep 5

fail=0
for n in map_server amcl controller_server planner_server bt_navigator \
         behavior_server smoother_server velocity_smoother waypoint_follower; do
    st=$(timeout 10 ros2 lifecycle get "/$n" 2>&1 | head -1)
    printf '%-22s %s\n' "$n" "$st"
    case "$st" in active*) ;; *) fail=1 ;; esac
done
[ $fail -eq 0 ] && echo "Nav2 정상" || { echo "아직 안 올라옴"; exit 1; }
