#!/usr/bin/env bash
# 데모만 재시작 (시뮬은 그대로 둔다). 호출자 명령줄이 pkill 패턴에
# 걸리지 않도록 모든 문자열을 이 파일 안에 둔다.
cd "$HOME/valet_parking_ws"
pkill -f 'demo[_]drive' 2>/dev/null
sleep 2
setsid nohup bash demo_drive.sh > /tmp/demo.log 2>&1 < /dev/null &
sleep 3
echo "데모 재시작됨"
