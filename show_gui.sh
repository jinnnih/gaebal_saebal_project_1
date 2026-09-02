#!/usr/bin/env bash
# 이미 돌고 있는 헤드리스 서버에 GUI 창만 붙인다.
#
# 왜 나누는가: 서버는 라이다 때문에 llvmpipe(소프트웨어)로 돌려야 하는데,
# GUI 창까지 llvmpipe 로 그리면 1228x875 를 소프트웨어로 래스터라이즈해서
# 너무 느리다. GUI 는 별도 프로세스라 하드웨어 GL 로 따로 띄울 수 있다.
#
#   터미널 1:  bash run_sim.sh      (서버 + 라이다, llvmpipe)
#   터미널 2:  bash show_gui.sh     (GUI 창, 하드웨어 GL)
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"
U=$(id -u)
export XDG_RUNTIME_DIR=/run/user/$U
export DISPLAY="${DISPLAY_OVERRIDE:-:1}"
X=$(ls -1 /run/user/$U/.mutter-Xwaylandauth* 2>/dev/null | head -1)
[ -n "$X" ] && export XAUTHORITY=$X
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER   # GUI 는 하드웨어 GL
export QT_QPA_PLATFORM=xcb
echo "GUI 창 기동 (DISPLAY=$DISPLAY, 하드웨어 GL)"
exec gz sim -g -v2
