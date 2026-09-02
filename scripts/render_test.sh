#!/usr/bin/env bash
# 라이다 렌더링 경로 3종 비교 (헤드리스 서버, GUI 없음)
#   egl_hw : 현재 방식. Ogre2 가 EGL 로 SVGA3D 하드웨어를 잡는다
#   glx_hw : X 디스플레이(:1)를 물려 GLX 경로 + 하드웨어
#   glx_sw : X 디스플레이 + llvmpipe 순수 소프트웨어
#            (EGL 에서는 mesa 가 소프트웨어 강제를 거부하지만 GLX 는 된다)
# 워크스페이스 경로는 스크립트 위치에서 구한다 (어디에 체크아웃하든 동작).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
U=$(id -u)

measure() {
  timeout 90 python3 - <<'PY'
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n=Node("c"); g=[]
n.create_subscription(LaserScan,"/scan",lambda m:g.append(m),qos_profile_sensor_data)
t0=time.time()
while len(g)<40 and time.time()-t0<45: rclpy.spin_once(n,timeout_sec=0.1)
if not g:
    print("    스캔 없음"); rclpy.shutdown(); raise SystemExit
good=[[r for r in m.ranges if math.isfinite(r)] for m in g]
good=[f for f in good if f]
print("    프레임 %d 중 유효 %d (%.0f%%)" % (len(g), len(good), 100.0*len(good)/len(g)), end="")
if good:
    print("  평균 %.0f/720 빔, 최대 %.1f m"
          % (sum(len(f) for f in good)/len(good), max(max(f) for f in good)))
else:
    print()
rclpy.shutdown()
PY
}

for MODE in egl_hw glx_hw glx_sw; do
  bash "$WS/scripts/kill_sim.sh" > /dev/null 2>&1
  sleep 3
  ( unset DISPLAY LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
    export XDG_RUNTIME_DIR=/run/user/$U
    export QT_QPA_PLATFORM=offscreen
    case "$MODE" in
      glx_hw|glx_sw)
        export DISPLAY=:1
        X=$(ls -1 /run/user/$U/.mutter-Xwaylandauth* 2>/dev/null | head -1)
        [ -n "$X" ] && export XAUTHORITY=$X ;;
    esac
    [ "$MODE" = "glx_sw" ] && export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
    exec ros2 launch valet_robot valet_sim.launch.py gui:=false
  ) > /tmp/rt_$MODE.log 2>&1 &
  sleep 60
  RTF=$(timeout 20 gz topic -e -t /stats -n 6 2>/dev/null | grep real_time_factor \
        | awk '{s+=$2;n++} END {if(n) printf "%.3f", s/n; else print "n/a"}')
  echo "== $MODE  (RTF=$RTF) =="
  measure
  grep -iE 'unable to open display|libEGL|Not allowed|render engine' /tmp/rt_$MODE.log \
    | sed 's/\x1b\[[0-9;]*m//g' | sort -u | head -2 | sed 's/^/    /'
done
bash "$WS/scripts/kill_sim.sh" > /dev/null 2>&1
