#!/usr/bin/env bash
# GUI 창만 재시작 (서버/데모는 건드리지 않는다)
cd "$HOME/valet_parking_ws"
pkill -f 'gz[ ]sim -g' 2>/dev/null
pkill -f 'gz-sim-gui' 2>/dev/null
sleep 4
setsid nohup bash show_gui.sh > /tmp/gui_sw.log 2>&1 < /dev/null &
sleep 3
echo "GUI 재시작 요청됨"
