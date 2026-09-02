#!/usr/bin/env bash
# 헤드리스 시뮬 기동 (GUI 없음, 라이다 정상 동작).
#
# ===== 렌더링 경로 (실측으로 결정) =========================================
# VMware SVGA3D 에서 gpu_lidar 는 렌더 경로에 따라 결과가 완전히 다르다.
# 같은 월드/로봇으로 40 프레임씩 재본 값:
#
#   경로                                   유효 프레임  평균 빔   최대거리
#   EGL + SVGA3D 하드웨어 (기본 헤드리스)      8 %      173/720    4.9 m
#   GLX + SVGA3D 하드웨어 (DISPLAY 지정)      15 %      125/720   29.5 m
#   GLX + llvmpipe 소프트웨어                100 %      427/720   29.5 m   <-- 채택
#
# 즉 "소프트웨어 렌더링을 쓰지 말라"는 EGL 경로에 한정된 얘기다.
# EGL 은 mesa 가 소프트웨어 강제를 거부하고("Not allowed to force software
# rendering when API explicitly selects a hardware device") 조용히 깨진다.
# X 디스플레이를 물려 GLX 로 가면 llvmpipe 가 정상 동작한다.
# 라이다 렌더 타깃이 720x1 로 작아서 소프트웨어로도 RTF 1.0 이 나온다.
#
# 전제: 데스크톱 세션이 로그인돼 있어야 한다 (:1 과 /dev/dri ACL).
#       로그인이 없으면 EGL 로 폴백하고 경고한다 — 그때 라이다는 못 믿는다.
# ===========================================================================
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"

U=$(id -u)
export XDG_RUNTIME_DIR=/run/user/$U
export QT_QPA_PLATFORM=offscreen

DISP="${DISPLAY_OVERRIDE:-}"
if [ -z "$DISP" ]; then
  for d in /tmp/.X11-unix/X*; do
    [ -e "$d" ] && DISP=":${d##*/X}" && break
  done
fi

if [ -n "$DISP" ] && [ -r /dev/dri/renderD128 ]; then
  export DISPLAY="$DISP"
  X=$(ls -1 /run/user/$U/.mutter-Xwaylandauth* 2>/dev/null | head -1)
  [ -n "$X" ] && export XAUTHORITY=$X
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  echo "렌더링: GLX + llvmpipe (DISPLAY=$DISPLAY)  — 라이다 정상"
else
  echo "!! 데스크톱 세션이 없거나 /dev/dri 접근 불가 -> EGL 폴백."
  echo "!! 이 경로에서는 라이다 프레임이 90% 이상 유실된다."
  echo "!! VM 화면에서 로그인한 뒤 다시 실행할 것."
fi

exec ros2 launch valet_robot valet_sim.launch.py gui:=false "$@"
