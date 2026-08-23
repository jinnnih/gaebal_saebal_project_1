#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# VMware 게스트에서 Gazebo GUI 를 깜빡임 없이 실행하는 래퍼.
#
#   bash gz_gui.sh                 기본 월드
#   bash gz_gui.sh parking_lot     월드 이름 지정
#
# 배경
#   VMware SVGA3D 드라이버 + ogre2 조합에서 화면 전체가 깜빡인다.
#   아래 순서로 배제 테스트를 거쳐 원인을 특정했다.
#     - Wayland 합성기      : Xorg 로 바꿔도 깜빡임 유지  -> 원인 아님
#     - ogre1 렌더 엔진     : 깜빡임 유지, 성능은 더 나쁨 -> 원인 아님
#     - Qt Quick 렌더 루프  : basic 으로 바꿔도 유지      -> 원인 아님
#     - vCPU 부족           : 4->8 로 RTF 는 4배 개선됐으나 깜빡임은 유지
#     - SVGA3D 드라이버     : 소프트웨어 렌더링으로 우회하니 해결  <-- 원인
#   참고로 glxgears 는 SVGA3D 에서도 멀쩡했다. ogre2 가 쓰는 경로에서만 발생.
#
# 대가
#   llvmpipe(CPU 렌더링)라 느리다. RTF 약 0.17.
#   Nav2 개발은 헤드리스(RTF 1.12) + RViz 조합을 쓰고,
#   이 스크립트는 월드를 눈으로 확인할 때만 쓰는 것을 권장한다.
# ---------------------------------------------------------------------------
set -u

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/valet_parking_ws/install/setup.bash" ] && \
    source "$HOME/valet_parking_ws/install/setup.bash"

UID_=$(id -u)
export XDG_RUNTIME_DIR=/run/user/$UID_
unset WAYLAND_DISPLAY

# Xorg 세션 자동 탐지 (GDM 은 보통 :1)
export QT_QPA_PLATFORM=xcb
export XAUTHORITY=${XAUTHORITY:-/run/user/$UID_/gdm/Xauthority}
if [ -z "${DISPLAY:-}" ]; then
  DISPLAY=$(who | awk '/\(:[0-9]+\)/{gsub(/[()]/,"",$NF); print $NF; exit}')
  export DISPLAY=${DISPLAY:-:1}
fi

# --- 깜빡임 해결 조합 ---
export LIBGL_ALWAYS_SOFTWARE=1    # SVGA3D 우회
export GALLIUM_DRIVER=llvmpipe    # CPU 래스터라이저
export QSG_RENDER_LOOP=basic      # Qt Quick 단일 스레드
export vblank_mode=1

WORLD_NAME=${1:-parking_lot}
WORLD="$(ros2 pkg prefix parking_lot_world)/share/parking_lot_world/worlds/${WORLD_NAME}.sdf"
if [ ! -e "$WORLD" ]; then
  echo "월드를 찾을 수 없음: $WORLD" >&2
  exit 1
fi

echo "DISPLAY=$DISPLAY  renderer=llvmpipe(SW)  world=$WORLD_NAME"
exec gz sim -r "$WORLD"
