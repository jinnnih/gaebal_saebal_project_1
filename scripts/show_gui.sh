#!/usr/bin/env bash
# 돌고 있는 헤드리스 서버에 GUI 창만 붙인다.
#
#   bash "$WS/scripts/show_gui.sh"        소프트웨어 렌더링 (기본) — 깜빡임 없음
#   HW=1 bash "$WS/scripts/show_gui.sh"   하드웨어 GL — 빠르지만 SVGA3D 깜빡임 발생
#
# 왜 GUI 를 소프트웨어로 돌려도 되는가:
#   서버(gz sim -s)와 GUI(gz sim -g)는 별개 프로세스다. 물리와 센서 렌더링은
#   전부 서버에서 돌기 때문에, GUI 를 llvmpipe 로 그려도 RTF 와 라이다 성능은
#   그대로다. 느려지는 건 창 그리는 것뿐이다.
#   (예전에 GUI+소프트웨어가 못 쓸 정도로 느렸던 건 서버까지 같은 프로세스
#    환경으로 묶여 있었기 때문이다)
#
#   터미널 1:  bash run_sim.sh      서버 + 라이다 (llvmpipe)
#   터미널 2:  bash "$WS/scripts/show_gui.sh"     GUI 창
# 워크스페이스 경로는 스크립트 위치에서 구한다 (어디에 체크아웃하든 동작).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

U=$(id -u)
export XDG_RUNTIME_DIR=/run/user/$U
export DISPLAY="${DISPLAY_OVERRIDE:-:1}"
for x in /run/user/$U/.mutter-Xwaylandauth* /run/user/$U/gdm/Xauthority "$HOME/.Xauthority"; do
  [ -r "$x" ] && export XAUTHORITY="$x" && break
done
export QT_QPA_PLATFORM=xcb

if [ "${HW:-0}" = "1" ]; then
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
  echo "GUI: 하드웨어 GL (SVGA3D) — 깜빡일 수 있음"
else
  # VMware SVGA3D 하드웨어 경로에서 화면 전체가 깜빡인다 (기존 이슈 #4).
  # llvmpipe 로 그리면 사라진다. QSG_RENDER_LOOP=basic 은 Qt 쪽 티어링 방지.
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  export QSG_RENDER_LOOP=basic
  export vblank_mode=1
  echo "GUI: llvmpipe 소프트웨어 렌더링 — 깜빡임 없음 (서버 성능에는 영향 없음)"
fi

echo "  DISPLAY=$DISPLAY  XAUTHORITY=${XAUTHORITY:-<none>}"
exec gz sim -g -v2
