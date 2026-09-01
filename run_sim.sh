#!/usr/bin/env bash
# 헤드리스 시뮬 기동.
#
# ! 렌더링 주의
#   show_swrender.sh 의 LIBGL_ALWAYS_SOFTWARE=1 / GALLIUM_DRIVER=llvmpipe 는
#   GUI 깜빡임(SVGA3D) 대응이다. 헤드리스(-s)에서는 절대 쓰면 안 된다.
#   Ogre2 가 헤드리스에서 EGL 을 쓰는데, mesa 가
#     "Not allowed to force software rendering when API explicitly
#      selects a hardware device"
#   로 소프트웨어 강제를 거부하고, 그 뒤 렌더링이 조용히 실패한다.
#   결과: gpu_lidar 의 720 빔이 전부 inf 로 나온다 (실측 확인함).
#   -> 헤드리스에서는 EGL 하드웨어(vmwgfx /dev/dri/renderD128)를 그대로 쓴다.
source /opt/ros/jazzy/setup.bash
source "$HOME/valet_parking_ws/install/setup.bash"
export QT_QPA_PLATFORM=offscreen
exec ros2 launch valet_robot valet_sim.launch.py gui:=false "$@"
