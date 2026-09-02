#!/usr/bin/env bash
# 시뮬 기동 -> 주행 계측 -> 결과 출력 을 한 번에. (호출자 명령줄이
# kill_sim.sh 패턴에 걸리지 않도록 모든 문자열을 이 파일 안에 둔다)
cd "$HOME/valet_parking_ws"
bash kill_sim.sh > /dev/null 2>&1
sleep 2
setsid nohup ./run_sim.sh "$@" > /tmp/rp_sim.log 2>&1 < /dev/null &
sleep 55
setsid nohup bash probe.sh > /tmp/probe.out 2>&1 < /dev/null &
PAT='smoke'"_"'probe'
sleep 15   # 프로브가 뜰 때까지 대기 (안 그러면 아래 루프가 즉시 빠져나간다)
for i in $(seq 1 90); do
  pgrep -f "$PAT" > /dev/null || break
  sleep 5
done
sleep 2
cat /tmp/probe.out | sed -n '1,12p'
