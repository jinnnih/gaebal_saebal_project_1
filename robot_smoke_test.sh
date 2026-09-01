#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# valet_robot 스모크 테스트 (헤드리스)
#   빌드 -> 모델 자기검증 -> 월드+로봇 스폰 -> 실제 주행 계측
#
#   bash robot_smoke_test.sh
#   bash robot_smoke_test.sh --no-build
# ---------------------------------------------------------------------------
# set -u 는 쓰지 않는다 — /opt/ros/jazzy/setup.bash 가
# AMENT_TRACE_SETUP_FILES 미정의 상태에서 죽는다.
WS="${WS:-$HOME/valet_parking_ws}"
LOG=/tmp/valet_sim.log
MODE="${1:-}"

source /opt/ros/jazzy/setup.bash

# ! 헤드리스에서는 소프트웨어 렌더링을 강제하지 말 것.
#   LIBGL_ALWAYS_SOFTWARE=1 을 주면 Ogre2 의 EGL 헤드리스 경로에서 mesa 가
#   "Not allowed to force software rendering when API explicitly selects a
#    hardware device" 로 거부하고 렌더링이 조용히 실패한다.
#   -> gpu_lidar 720 빔이 전부 inf. (실측 확인)
#   show_swrender.sh 의 설정은 GUI 깜빡임 대응이라 여기엔 해당 없음.
export QT_QPA_PLATFORM=offscreen

cleanup() {
  echo
  echo "########## 정리 ##########"
  pkill -f 'valet_sim.launch' 2>/dev/null
  pkill -f 'spawn_valet_car'  2>/dev/null
  pkill -f 'parking_world'    2>/dev/null
  pkill -f 'gz[ ]sim'         2>/dev/null
  pkill -f 'ruby.*gz'         2>/dev/null
  pkill -f parameter_bridge   2>/dev/null
  pkill -f robot_state_pub    2>/dev/null
  sleep 2
  echo "  완료"
}
trap cleanup EXIT

cd "$WS" || exit 1

if [ "$MODE" != "--no-build" ]; then
  echo "########## 1. 빌드 ##########"
  colcon build --packages-select parking_lot_world valet_robot \
    --event-handlers console_cohesion- 2>&1 | tail -12
  [ "${PIPESTATUS[0]}" -eq 0 ] || { echo "!! 빌드 실패"; exit 1; }
else
  echo "########## 1. 빌드 건너뜀 ##########"
fi
source "$WS/install/setup.bash"

echo
echo "########## 2. 모델 자기검증 ##########"
python3 "$(ros2 pkg prefix valet_robot)/share/valet_robot/tools/check_model.py"
[ $? -eq 0 ] || { echo "!! 모델 검증 실패"; exit 1; }

echo
echo "########## 3. URDF -> SDF 변환 확인 ##########"
XACRO="$(ros2 pkg prefix valet_robot)/share/valet_robot/urdf/valet_car.urdf.xacro"
xacro "$XACRO" sim:=true > /tmp/valet_car.urdf || exit 1
gz sdf -p /tmp/valet_car.urdf > /tmp/valet_car.sdf 2>/tmp/valet_sdf.err
echo "  URDF $(stat -c%s /tmp/valet_car.urdf) B  ->  SDF $(stat -c%s /tmp/valet_car.sdf) B"
echo "  --- 변환 경고/오류 ---"
grep -iE 'error|warning|err\]|wrn\]' /tmp/valet_sdf.err | sort -u | head -12
echo "  (위에 아무것도 없으면 정상)"
echo "  --- SDF 에 남은 링크 / 조인트 / 센서 ---"
grep -oE "<link name='[^']*'" /tmp/valet_car.sdf | sed "s/.*'\(.*\)'/    link  \1/"
grep -oE "<joint name='[^']*'" /tmp/valet_car.sdf | sed "s/.*'\(.*\)'/    joint \1/"
grep -oE "<sensor name='[^']*' type='[^']*'" /tmp/valet_car.sdf | sed 's/^/    /'

echo
echo "########## 4. 월드 + 로봇 헤드리스 기동 ##########"
setsid nohup ros2 launch valet_robot valet_sim.launch.py gui:=false \
  > "$LOG" 2>&1 < /dev/null &
echo "  로그: $LOG"

echo "  컨트롤러 활성화 대기 (최대 180 s)..."
for i in $(seq 1 60); do
  sleep 3
  OUT=$(timeout 10 ros2 control list_controllers 2>/dev/null)
  if echo "$OUT" | grep -q 'ackermann_steering_controller.*active' && \
     echo "$OUT" | grep -q 'joint_state_broadcaster.*active'; then
    echo "  활성화 완료 ($((i*3)) s)"
    break
  fi
done
echo "  --- ros2 control list_controllers ---"
timeout 10 ros2 control list_controllers 2>&1 | sed 's/^/    /'
echo "  --- ros2 control list_hardware_interfaces ---"
timeout 10 ros2 control list_hardware_interfaces 2>&1 | sed 's/^/    /' | head -30

echo
echo "########## 5. 토픽 확인 ##########"
for t in /scan /odom /joint_states /clock /cmd_vel /ackermann_steering_controller/reference; do
  printf '    %-46s %s\n' "$t" "$(timeout 5 ros2 topic info "$t" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g')"
done

echo
echo "########## 6. 주행 계측 ##########"
python3 "$(ros2 pkg prefix valet_robot)/share/valet_robot/tools/smoke_probe.py"
RC=$?

echo
echo "########## 7. 시뮬 로그 오류 ##########"
grep -iE '\[ERROR\]|\[Err\]|exception|failed|died' "$LOG" | sort -u | head -20
echo "  (위에 아무것도 없으면 정상)"

exit $RC
