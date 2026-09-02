#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# valet_robot 스모크 테스트 — 빌드부터 주행 계측까지 한 번에.
#
#   bash scripts/smoke_test.sh              빌드 포함
#   bash scripts/smoke_test.sh --no-build   빌드 건너뜀
#
# 1 빌드 -> 2 모델 자기검증 -> 3 URDF/SDF 변환 확인 -> 4 헤드리스 기동
# -> 5 토픽/하드웨어 인터페이스 -> 6 주행 계측 8 항목 -> 7 로그 오류
#
# 렌더 경로 설정은 run_sim.sh 가 잡는다. 그걸 거치지 않으면 라이다가
# EGL 경로로 떨어져 프레임의 90% 가 유실된다 (자세한 건 run_sim.sh 주석).
# ---------------------------------------------------------------------------
# set -u 는 쓰지 않는다 — /opt/ros/jazzy/setup.bash 가
# AMENT_TRACE_SETUP_FILES 미정의 상태에서 죽는다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/valet_sim.log
MODE="${1:-}"

source /opt/ros/jazzy/setup.bash
trap 'echo; echo "########## 정리 ##########"; bash "$WS/scripts/kill_sim.sh"' EXIT
cd "$WS" || exit 1

if [ "$MODE" != "--no-build" ]; then
  echo "########## 1. 빌드 ##########"
  colcon build --packages-select parking_lot_world valet_robot \
    --event-handlers console_cohesion- 2>&1 | tail -8
  [ "${PIPESTATUS[0]}" -eq 0 ] || { echo "!! 빌드 실패"; exit 1; }
else
  echo "########## 1. 빌드 건너뜀 ##########"
fi
source "$WS/install/setup.bash"

echo
echo "########## 2. 모델 자기검증 ##########"
ros2 run valet_robot check_model.py || { echo "!! 모델 검증 실패"; exit 1; }

echo
echo "########## 3. URDF -> SDF 변환 확인 ##########"
XACRO="$(ros2 pkg prefix valet_robot)/share/valet_robot/urdf/valet_car.urdf.xacro"
xacro "$XACRO" sim:=true > /tmp/valet_car.urdf || exit 1
gz sdf -p /tmp/valet_car.urdf > /tmp/valet_car.sdf 2>/tmp/valet_sdf.err
echo "  URDF $(stat -c%s /tmp/valet_car.urdf) B  ->  SDF $(stat -c%s /tmp/valet_car.sdf) B"
echo "  --- 변환 오류 (gz_frame_id 경고는 정상) ---"
grep -iE 'error|unable to find' /tmp/valet_sdf.err | sort -u | head -6
echo "  --- SDF 에 남은 링크 / 조인트 / 센서 ---"
grep -oE "<link name='[^']*'"   /tmp/valet_car.sdf | sed "s/.*'\(.*\)'/    link  \1/"
grep -oE "<joint name='[^']*'"  /tmp/valet_car.sdf | sed "s/.*'\(.*\)'/    joint \1/"
grep -oE "<sensor name='[^']*' type='[^']*'" /tmp/valet_car.sdf | sed 's/^/    /'

echo
echo "########## 4. 헤드리스 기동 ##########"
bash "$WS/scripts/kill_sim.sh" > /dev/null
setsid nohup "$WS/scripts/run_sim.sh" > "$LOG" 2>&1 < /dev/null &
echo "  로그: $LOG"
echo "  컨트롤러 활성화 대기 (최대 180 s)..."
for i in $(seq 1 60); do
  sleep 3
  OUT=$(timeout 10 ros2 control list_controllers 2>/dev/null)
  if echo "$OUT" | grep -q 'ackermann_steering_controller.*active' && \
     echo "$OUT" | grep -q 'joint_state_broadcaster.*active'; then
    echo "  활성화 완료 ($((i*3)) s)"; break
  fi
done
timeout 10 ros2 control list_controllers 2>&1 | sed 's/^/    /'
echo "  --- 렌더 경로 ---"
head -1 "$LOG" | sed 's/^/    /'

echo
echo "########## 5. 토픽 ##########"
for t in /scan /odom /joint_states /clock /cmd_vel /ackermann_steering_controller/reference; do
  printf '    %-46s %s\n' "$t" \
    "$(timeout 5 ros2 topic info "$t" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g')"
done
echo "  --- RTF ---"
timeout 20 gz topic -e -t /stats -n 8 2>/dev/null | grep real_time_factor \
  | awk '{s+=$2;n++} END {if(n) printf "    %.3f\n", s/n; else print "    n/a"}'

echo
echo "########## 6. 주행 계측 ##########"
ros2 run valet_robot smoke_probe.py
RC=$?

echo
echo "########## 7. 시뮬 로그 오류 ##########"
grep -iE '\[ERROR\]|\[Err\]|exception|died' "$LOG" | sed 's/\x1b\[[0-9;]*m//g' \
  | sort -u | head -10
echo "  (위에 아무것도 없으면 정상)"
exit $RC
