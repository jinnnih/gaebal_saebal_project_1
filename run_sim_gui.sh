#!/usr/bin/env bash
# VM 데스크톱 세션(:1) 에 Gazebo GUI + RViz 로 띄운다. 사람이 보는 용도.
#
#   bash run_sim_gui.sh              하드웨어 GL (SVGA3D) - 기본
#   SW=1 bash run_sim_gui.sh         소프트웨어 렌더링 (깜빡임 대응, 매우 느림)
#   bash run_sim_gui.sh nav2:=true   Nav2 까지
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"

U=$(id -u)
export XDG_RUNTIME_DIR=/run/user/$U
unset WAYLAND_DISPLAY
export QT_QPA_PLATFORM=xcb
export DISPLAY=${DISPLAY_OVERRIDE:-:1}
X=$(ls -1 /run/user/$U/.mutter-Xwaylandauth* 2>/dev/null | head -1)
[ -n "$X" ] && export XAUTHORITY=$X

# 기본은 하드웨어 GL. llvmpipe 를 강제하면 RTF 가 0.0x 까지 떨어져 쓸 수 없다.
if [ "${SW:-0}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  export QSG_RENDER_LOOP=basic
fi

echo "DISPLAY=$DISPLAY  SW=${SW:-0}  renderer=$(glxinfo -B 2>/dev/null | awk -F': ' '/OpenGL renderer/{print $2}')"
exec ros2 launch valet_robot valet_sim.launch.py gui:=true rviz:=true "$@"
