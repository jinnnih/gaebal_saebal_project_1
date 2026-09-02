# valet_robot

차량형(**Ackermann**) 자율 발렛파킹 로봇 **`valet_car`** 의 URDF/Xacro 모델 +
ros2_control(조향·구동) 구성 패키지. 계획서 **1주차 산출물**
"차량형 URDF·ros2_control 구성, Gazebo 스폰, 수동 조향 주행 확인" 에 해당한다.

대상 환경: **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic (gz-sim 8) / Nav2 1.3+**
(계획서에는 Humble 로 적혀 있으나 `parking_lot_world` 와 동일하게 Jazzy 기준이다)

이 패키지는 `parking_lot_world` 와 **한 쌍**이다. 그쪽 README "4. 로봇 스펙 전제"에
못박힌 제원에 맞춰 설계했고, `tools/check_model.py` 가 두 패키지의 수치가
어긋나면 실패하도록 되어 있다.

---

## 1. 제원

| 항목 | 값 | 출처 / 근거 |
|---|---|---|
| 전장 × 전폭 | **4.50 × 1.90 m** | `parking_spots.json` `robot_spec` |
| 축거(wheelbase) | **2.50 m** | 〃 |
| 윤거(track) | 1.38 m | 아래 "설계 결정 4" |
| 바퀴 | 반지름 0.33 m, 폭 0.22 m | — |
| 최대 조향각(자전거 모델) | **35°** (0.6109 rad) | `robot_spec.max_steer_rad` |
| 조향 조인트 리밋(안쪽 바퀴) | **43.0°** | Ackermann 내륜 소요각 40.96° + 5% |
| **최소 회전반경** | **3.5704 m** | `2.50 / tan 35°`, Smac·MPPI 값과 동일 |
| 전진 / 후진 최대속도 | 1.60 / 0.60 m/s | `nav2_ackermann.yaml` `vx_max` / `vx_min` |
| 전고 | **1.45 m** (라이다 포함) | 실제 세단급 (쏘나타 1.445 / 캠리 1.455) |
| 형상 | 세단 — 보닛·캐빈·앞뒤유리·휠아치·등화·사이드미러 | |
| 총 질량 | 913 kg | — |
| 구동 방식 | 후륜 구동 + 전륜 조향 | `ackermann_steering_controller` |

차체는 **실제 승용차(세단) 형상**이다. 하부는 축 사이에만 플로어팬을 두고 바퀴 x 구간을
비워 휠아치를 만들었고, 그 위를 전폭 벨트라인이 덮는다. 앞머리는 벨트라인을 앞뒤로 나눠
낮췄고 보닛에 경사를 줬다. 라이다는 지붕 위 1.42 m — 이유는 아래 참고.

---

## 2. 핵심 설계 결정 4가지

### 1) `base_link` 를 후륜축이 아니라 **차체 중심**에 뒀다

Ackermann 로봇은 보통 `base_link` 를 후륜축에 둔다(자전거 모델의 원점이라서).
그런데 이 프로젝트에서는 그러면 안 된다.

* `nav2_ackermann.yaml` 의 footprint 가 `[±2.30, ±1.00]` 으로 **대칭**이고,
* `parking_spots.yaml` 의 `goal_pose` 가 **주차면 중심**이다.

`base_link` 를 후륜축에 두면 후진 주차 완료 시 차체가 주차면 밖으로
**1.25 m 튀어나온다.** 그래서 차체 중심으로 잡고, 앞/뒤 오버행을 각 1.00 m 로
맞췄다 (2.50 + 1.00 + 1.00 = 4.50).

### 2) 라이다를 지붕 위 **1.42 m** 에 뒀다

실차 형상이면 자기 지붕(1.34 m)보다 위에 달아야 360도가 안 가린다. 그런데 그 위에서
주차 차량이 존재하는 높이대는 하나뿐이다.

```
 parked_car 차체   0.27 ~ 0.99 m   (4.4 x 1.8)
 parked_car 캐빈   0.99 ~ 1.51 m   (2.2 x 1.66)   <- 지붕보다 위에서 유일하게 겹치는 대역
 벽 / 기둥         0.00 ~ 2.60 m
 EV 충전기         0.00 ~ 1.30 m    게이트            0.00 ~ 1.20 m    >  전부 정적 맵(parking_lot.pgm)에 포함
 라바콘            0.00 ~ 0.59 m   /
 ------------------------------------------------------
 자차 지붕                 1.34 m
 라이다                    1.42 m   <- 캐빈 상단까지 9 cm 여유
```

**트레이드오프가 있다.** 이 높이에서는 주차 차량이 차체(4.4 m)가 아니라 캐빈(2.2 m)으로
보인다. 즉 차량 외형을 앞뒤로 약 1.1 m 씩 작게 본다. 빈 주차면 탐색(있다/없다 판정)에는
충분하고, 통로 주행 중 옆 주차면까지 3.5 m 여유가 있어 실용상 문제없다.

풀 차체를 보려면 범퍼 장착(0.55 m)이 정답이지만, 그러면 자기 차체에 가려 FOV 를 270도로
제한하고 전/후방 2개를 써야 한다. **이 GL 스택은 gpu_lidar 를 하나만 렌더링**해서(13장 참고)
2개 구성이 불가능하다. GPU 문제가 해결되면 범퍼 2개로 옮기는 것이 더 낫다.

### 3) odom TF 는 컨트롤러가 아니라 **Gazebo 플러그인**이 낸다

`ackermann_steering_controller` 의 오도메트리는 **후륜축 원점 자전거 모델**이다
(`v_by = 0`, `ω = v·tanδ/L`). 그런데 이 로봇의 `base_link` 는 차체 중심이라
후륜축에서 1.25 m 앞이다. 컨트롤러 TF 를 그대로 쓰면 `odom → base_footprint` 가
차량이 선회할 때마다 **최대 1.25 m 씩 어긋난다** (회전각에 따라 달라지므로 AMCL 이
흡수할 수 있는 고정 오프셋도 아니다). 주차 허용오차가 **xy 0.12 m** 인데 말이 안 된다.

그래서:

| | 발행 주체 | 용도 |
|---|---|---|
| `odom → base_footprint` TF | `gz-sim-odometry-publisher-system` (`robot_base_frame: base_footprint`) | Nav2 / AMCL |
| `/odom` 토픽 | 〃 (가우시안 노이즈 0.004) | Nav2 `odom_topic` |
| `/ackermann_steering_controller/odometry` | 컨트롤러 (`enable_odom_tf: false`, `base_frame_id: base_rear_axle`) | 휠 오도메트리 비교·디버깅용 |

후륜축 참조 프레임 `base_rear_axle` 을 URDF 에 실제 링크로 넣어 뒀으므로
두 오도메트리를 RViz 에서 직접 비교할 수 있다.

### 4) 조향 조인트 리밋은 35°가 아니라 **43°**다 (윤거 1.38 m)

여기서 한 번 틀렸다가 실측으로 잡았다.

`max_steer = 35°`는 **자전거 모델**(가상 중앙륜)의 각도이고, 최소회전반경
`R = L/tan(35°) = 3.5704 m`를 정의하는 값이다. 그런데 실제 Ackermann 링크에서
**안쪽 바퀴는 그보다 더 꺾여야 한다**:

```
 delta_inner = atan(L / (R - track/2)) = atan(2.5 / 2.8804) = 40.96°
```

조인트 리밋을 35°로 잡았더니 시뮬에서 **안쪽 바퀴가 리밋에 포화**됐다
(실측: 좌 29.9° / 우 35.0°(포화) → 회전반경이 3.57 이 아니라 7.5 m).
40.96° + 5% 여유 = **43.0°**로 올려서 해결했다 (실측: 좌 32.2° / 우 42.0°, 포화 없음).

윤거 1.38 m 는 그 43° 에서도 앞바퀴 바깥끝이 Nav2 풋프린트 반폭 1.00 m 를
넘지 않도록 잡은 값이다.

```
 0.69 + 0.33·sin43° + (0.22/2)·cos43° = 0.996 m  <  1.00 m
```

MPPI `CostCritic.consider_footprint: true` 라서 이 여유가 실제로 의미가 있다.

---

## 3. 프레임과 링크

```
base_footprint                      지면. URDF 루트. AMCL base_frame_id
└─ base_link            z +0.33     차체 중심. Nav2 robot_base_frame, footprint 기준
   ├─ base_rear_axle    x -1.25     후륜축 (컨트롤러 오도메트리 원점, 참조용)
   ├─ lidar_link        z +0.59     2D 라이다 (절대 0.92 m)
   ├─ imu_link          z +0.05
   ├─ front_left_steer_link   (1.25,  0.70)  revolute z, ±35°
   │  └─ front_left_wheel_link       continuous y  (비구동)
   ├─ front_right_steer_link  (1.25, -0.70)  revolute z, ±35°
   │  └─ front_right_wheel_link      continuous y  (비구동)
   ├─ rear_left_wheel_link   (-1.25,  0.70)  continuous y  (구동)
   └─ rear_right_wheel_link  (-1.25, -0.70)  continuous y  (구동)
```

`front_*_steer_link` 는 충돌 형상이 없다. 앞바퀴는 `base_link` 와 **직접** 조인트로
이어져 있지 않아 자기충돌이 비활성화되지 않기 때문이다(뒷바퀴는 직접 연결이라 괜찮다).
같은 이유로 하부 섀시 폭을 1.10 m 로 좁혀 앞바퀴 안쪽끝(0.59 m)과 4 cm 띄웠다.

## 4. 명령 경로

```
 [키보드 teleop]  또는  [Nav2 velocity_smoother]
        │ geometry_msgs/Twist
        ▼
   /cmd_vel  (Nav2 사용 시 /cmd_vel_smoothed)
        │
        ▼
 twist_to_ackermann          ← 차량 기구학 제한이 걸리는 지점
        │  · vx 를 [-0.60, +1.60] 으로 클램프
        │  · |wz| ≤ |vx| / 3.5704   (최소회전반경 초과 요구 차단)
        │  · |vx| ≈ 0 이면 wz = 0   (제자리 회전 불가)
        │  · 0.5 s 입력 없으면 0 발행 (워치독)
        │ geometry_msgs/TwistStamped
        ▼
 /ackermann_steering_controller/reference
        │
        ▼
 ackermann_steering_controller  (ros2_control, gz_ros2_control 안에서 구동)
        ├─ front_*_steer_joint   position  ← 좌/우 조향각을 Ackermann 기하로 분리
        └─ rear_*_wheel_joint    velocity  ← 좌/우 구동륜 속도차
```

`twist_to_ackermann` 이 단순 릴레이가 아니라 **기구학 리미터**라는 점이 중요하다.
Nav2 가 제자리 회전을 요구해도(복구행동 등) 차량형은 못 하므로 여기서 잘라낸다.
`nav2_ackermann.yaml` 이 `behavior_plugins` 에서 Spin 을 뺀 것과 짝을 이룬다.

## 5. 발행 토픽

| 토픽 | 타입 | 발행 | 소비처 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | gz gpu_lidar → 브리지 | AMCL, costmap `obstacle_layer` |
| `/odom` | `nav_msgs/Odometry` | gz OdometryPublisher → 브리지 | Nav2 `odom_topic` |
| `/tf` (odom→base_footprint) | `tf2_msgs/TFMessage` | 〃 (`/tf_gz` 브리지) | Nav2 전반 |
| `/tf` (나머지) | 〃 | robot_state_publisher | — |
| `/imu` | `sensor_msgs/Imu` | gz imu → 브리지 | (선택) robot_localization |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | RSP |
| `/ackermann_steering_controller/odometry` | `nav_msgs/Odometry` | 컨트롤러 | 휠 오도메트리 비교용 |

## 6. 파일 구성

```
valet_robot/
├── urdf/
│   ├── valet_car.urdf.xacro          최상위. sim:=true/false 로 Gazebo 부분 토글
│   ├── common.xacro                  치수 상수·관성 매크로·재질  ★ 수치는 전부 여기
│   ├── wheels.xacro                  조향 너클 / 바퀴 매크로
│   ├── valet_car.ros2_control.xacro  ros2_control 인터페이스 + gz_ros2_control 플러그인
│   └── sensors.gazebo.xacro          라이다 / IMU / 오도메트리 / 바퀴 마찰
├── config/
│   ├── controllers.yaml              controller_manager + ackermann_steering_controller
│   └── gz_bridge.yaml                ros_gz_bridge 토픽 매핑
├── launch/
│   ├── valet_sim.launch.py           ★ 월드 + 로봇 + (선택)Nav2 한 방에
│   ├── spawn_valet_car.launch.py     실행 중인 Gazebo 에 스폰만
│   └── description.launch.py         Gazebo 없이 RViz 로 모델만 확인
├── scripts/
│   ├── twist_to_ackermann.py         cmd_vel → 컨트롤러 reference + 기구학 제한
│   └── ackermann_teleop_key.py       속도/조향각 방식 키보드 수동주행
└── tools/
    └── check_model.py                ★ 모델 자기검증 + 맵 패키지 정합성 검사
```

**치수를 바꿀 때는 `urdf/common.xacro` 만 고치고 `tools/check_model.py` 를 돌린다.**

---

## 7. 빌드

```bash
cd ~/valet_parking_ws
colcon build --packages-select parking_lot_world valet_robot
source install/setup.bash
```

필요 패키지 (`install_ros2.sh` 4단계에 이미 들어 있다):

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ackermann-steering-controller ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui
```

## 8. 실행

### (a) 모델만 확인 — Gazebo 불필요

```bash
ros2 launch valet_robot description.launch.py
```

`joint_state_publisher_gui` 슬라이더로 `front_*_steer_joint` 를 ±35° 까지 돌려
좌/우 조향각과 바퀴 위치를 눈으로 확인한다.

### (b) 계획서 1주차 — 수동 조향 주행

```bash
# 터미널 1 : 주차장 월드 + 로봇 + RViz
ros2 launch valet_robot valet_sim.launch.py rviz:=true

# 터미널 2 : 키보드 수동주행
ros2 run valet_robot ackermann_teleop_key.py
#   w/s 속도 ±0.2   a/d 조향 ±3°   e 조향중립   space 정지   q 종료
```

토픽으로 직접 넣어도 된다:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.28}}"
```

`wz = 0.28` 이면 회전반경 `1.0/0.28 = 3.57 m` 로 **최소 회전반경 주행**이 된다.
더 큰 값을 넣으면 `twist_to_ackermann` 이 잘라낸다(경고 없이 클램프).

확인할 것:

```bash
ros2 control list_controllers          # 둘 다 active
ros2 topic echo /scan --once           # ranges 가 벽 거리로 채워지는지
ros2 run tf2_tools view_frames         # odom -> base_footprint -> base_link
ros2 topic echo /joint_states --once   # 좌/우 조향각이 서로 다른지 (Ackermann)
```

### (c) 2주차 이후 — Nav2 자율주행

```bash
ros2 launch valet_robot valet_sim.launch.py nav2:=true rviz:=true
```

RViz 의 **2D Goal Pose** 로 목표를 찍으면 Smac Hybrid-A* 가 후진 포함 경로를 낸다.
`nav2:=true` 이면 `twist_to_ackermann` 의 입력이 자동으로 `/cmd_vel_smoothed` 로
바뀐다 — `nav2_valet.launch.py` 가 `velocity_smoother` 까지만 띄우고
`collision_monitor` 는 안 띄우기 때문에 최종 속도 토픽이 그것이다.

> `collision_monitor` 를 나중에 추가하면 최종 토픽이 `/cmd_vel` 이 된다.
> 그때는 `cmd_vel_topic:=/cmd_vel` 로 되돌릴 것.

### 스폰 포즈

기본값은 `parking_lot_world` README 의 입구 진입 직후 지점이다.

```bash
ros2 launch valet_robot valet_sim.launch.py x:=-23.00 y:=-18.30 yaw:=0.0
```

`x`/`y`/`yaw` 는 **차체 중심(base_link)** 기준이다(후륜축이 아님).

## 9. 검증

```bash
python3 tools/check_model.py       # 소스 트리에서 바로 (colcon build 불필요)
```

7개 항목을 검사한다 — 트리 구조 / 기구학 / 치수·간섭 / 라이다 높이 /
관성 / ros2_control 인터페이스 / **`parking_lot_world` 정합성**.

마지막 항목이 핵심이다. URDF 의 축거·최소회전반경·풋프린트가
`parking_spots.json` `robot_spec` 및 `nav2_ackermann.yaml` 의
`minimum_turning_radius` · `min_turning_r` · `footprint` 와 어긋나면 실패한다.
**둘 중 하나만 고치는 사고를 막는 장치다.**

현재 상태:

```
 축거 2.5000 m | 윤거 1.4000 m | 최대조향 35.00 deg | 최소회전반경 3.5704 m
 전장 4.500 m | 전폭 1.900 m
 [ok] 데크 하단(0.680) > 바퀴 상단(0.660), 여유 0.020 m
 [ok] 섀시 반폭(0.550) < 앞바퀴 안쪽끝(0.590), 자기충돌 없음
 [ok] 최대조향에도 footprint 반폭 1.00 안쪽 (여유 0.021 m)
 [ok] 주차 차량을 라이다로 본다 / 자기 데크 상단(0.820) 위
 [ok] robot_spec 5개 항목 · Smac · MPPI · footprint 전부 일치
 전체 통과
```

## 10. 회전반경에 대한 주석 하나

`minimum_turning_radius = 3.5704` 는 **후륜축 중심**의 회전반경이다.
`base_link` 가 차체 중심이므로 base_link 궤적의 회전반경은
`sqrt(3.5704² + 1.25²) = 3.783 m` 로 조금 더 크다.
Smac 이 쓰는 값이 실제보다 **작은**(=더 급한) 쪽이라 보수적이지 않은데,
차이가 6% 이고 풋프린트 충돌검사(`consider_footprint: true`)가 별도로 돌기 때문에
현재 튜닝에서는 문제가 없다. 3주차 후진 주차에서 경로가 주차면 모서리를 스치면
`nav2_ackermann.yaml` 의 값을 3.78 로 올려서 재시험할 것.

## 11. 다음 단계 (계획서 마일스톤)

| 주차 | 할 일 | 이 패키지와의 관계 |
|---|---|---|
| 1주 | 수동 조향 주행 ✅ | 이 패키지가 산출물 |
| 2주 | SLAM 맵 / Smac+MPPI 기본 주행 | `/scan` `/odom` 그대로 사용 |
| 3주 | 후진 주차 기동 | `parking_goal_checker` (xy 0.12 / yaw 0.06) 로 정차 오차 측정 |
| 4주 | BT 커스텀 노드 4종 | 별도 패키지(`valet_bt_nodes`)로 분리할 것 |
| 5주 | 허용오차·속도 프로파일 튜닝 | `common.xacro` 는 건드리지 말고 Nav2 쪽에서 |

---

## 12. 실측 결과 (Ubuntu 24.04 VM, Gazebo Harmonic)

`tools/smoke_probe.py` 를 실제 시뮬에서 돌린 결과다.

| 항목 | 결과 |
|---|---|
| B. `odom → base_footprint` TF | 정상 |
| C. 전진 8 s @1.0 m/s | 이동 **8.00 m**, 방향변화 0.0° |
| D. 최소회전반경 선회 | 실측 **R = 3.49 m** (요구 3.5704 이하 — 만족) |
| E. Ackermann 내륜차 | 좌 **39.60°** / 우 **30.61°** (차 8.99°) — 좌회전에서 좌측이 내륜 ✅ |
| F. 후진 5 s @-0.5 m/s | 차체 전방 기준 **-2.49 m** |
| G. 제자리회전 차단 | 이동 0.000 m, 회전 0.00° |
| H. 각속도 클램프 | 요청 2.0 rad/s → 실측 **0.251**, 상한 0.280 |
| A. `/scan` | 별도 — 13장 참고 (GL 드라이버 이슈) |

컨트롤러 상태:

```
ackermann_steering_controller  ackermann_steering_controller/AckermannSteeringController  active
joint_state_broadcaster        joint_state_broadcaster/JointStateBroadcaster              active
```

### 형상 미리보기

Gazebo 없이 URDF 만으로 4면도를 그릴 수 있다. 렌더링이 막힌 환경에서 형상을 확인할 때 쓴다.

```bash
python3 tools/preview_model.py preview.png 25    # 25 = 조향각(deg)
```

## 13. 이 VM 에서 겪은 환경 문제 (모델 문제 아님)

가장 시간을 많이 쓴 부분이라 남긴다. **전부 시뮬 환경 이슈였고 로봇 모델은 정상이었다.**

### (1) 헤드리스에서 소프트웨어 렌더링을 강제하면 라이다가 죽는다

`show_swrender.sh` 의 `LIBGL_ALWAYS_SOFTWARE=1` 은 **GUI 깜빡임 대응**이다.
헤드리스(`gz sim -s`)에서 같이 쓰면 Ogre2 가 EGL 경로를 타는데 mesa 가

```
libEGL warning: Not allowed to force software rendering when API
                explicitly selects a hardware device.
```

로 거부하고, 그 뒤 렌더링이 **조용히 실패**한다. gpu_lidar 720 빔이 전부 `inf`.
→ 헤드리스에서는 EGL 하드웨어(vmwgfx `/dev/dri/renderD128`)를 그대로 쓴다.
`run_sim.sh` 에 이 내용을 주석으로 박아 뒀다.

### (2) gz-sim 이 gpu_lidar 를 **하나만** 렌더링한다

월드에 렌더링 센서가 2개 이상이면 하나만 동작하고 나머지는 전부 `inf` 를 낸다.
A/B 로 확인했다:

| 월드 구성 | 로봇 `/scan` 유효 프레임 |
|---|---|
| 월드에 더미 라이다 1개 추가 + 로봇 | **0 %** |
| 더미 없음 + 로봇만 | **32 %** |

"월드 로드 시점에 센서를 하나 둬서 렌더 씬을 워밍업한다"는 대응을 넣었다가
**오히려 로봇 라이다를 죽이는 것**을 확인하고 되돌렸다.
→ `parking_lot_world` 월드에는 렌더링 센서를 두지 않는다.

### (3) VMware SVGA3D 헤드리스 렌더링 자체가 불안정

위 문제를 다 제거해도 **프레임의 약 32% 만 유효**하다. 나머지는 통째로 `inf`
이거나 일부만 채워져 온다. 부하를 낮춰도(720→180 빔, 15→5 Hz) 12~32% 사이에서
움직일 뿐 근본 해결이 안 된다. 렌더 엔진을 `ogre`(1.x) 로 바꾸면 헤드리스에서
아예 죽는다(`Couldn't open X display`, GLX 전용).

**유효 프레임 자체는 정확하다** — 최대 관측거리 29.9 m 로 스펙(30 m)대로 나온다.
즉 모델·센서 설정은 맞고, GL 드라이버가 프레임을 흘리는 것이다.

대응 우선순위:

1. VM 설정에서 **3D 가속 활성화 + 그래픽 메모리 증량** (또는 실 GPU 머신/듀얼부팅)
2. 물리 스텝 완화 (아래 (4)) — CPU 여유가 생겨 유효율이 18% → 32% 로 개선됨
3. 그래도 부족하면 Nav2 는 **정적 맵 + AMCL** 위주로 돌리고
   `obstacle_layer` 의 `raytrace_max_range` 를 줄여 잘못된 클리어링을 억제

### (4) 물리 스텝 1 ms → 4 ms (RTF 0.09 → 0.73)

`parking_lot_world` 의 월드가 `max_step_size 0.001` 이라 RTF 가 0.088 밖에 안 나왔다.
913 kg 다관절 차량에는 과하다. 4 ms 로 올려 **RTF 0.45~0.73** 을 얻었다.
`worlds/*.sdf` 와 `tools/generate_parking_lot.py` 양쪽에 반영했다.

| max_step_size | RTF | 스캔 유효 프레임 |
|---|---|---|
| 0.001 | 0.088 | 18 % |
| 0.002 | 0.242 | 32 % |
| **0.004** | **0.540** | **32 %** |

### (5) `ackermann_steering_controller` 조인트 순서가 **[오른쪽, 왼쪽]** 이다

가장 오래 헤맨 버그다. `traction_joints_names` / `steering_joints_names` 를 상식대로
`[left, right]` 로 줬더니 **좌우가 뒤바뀐 Ackermann** 이 걸렸다.

```
좌회전(wz > 0) 인데 실측:
  좌 조향 29.9°  우 조향 41.1°     <- 우측이 내륜처럼 더 꺾임 (반대)
  좌 구동 3.62   우 구동 2.44      <- 우측이 내륜처럼 더 느림 (반대)
```

`steering_controllers_library` 의 `SteeringOdometry::get_commands()` 가
`{alpha_r, alpha_l}` 순으로 반환하기 때문이다. 즉 **index 0 이 오른쪽**이다.

순서를 뒤집자 좌 39.6° / 우 30.6° 로 정상이 됐고, 타이어 스크럽이 사라지면서
**실측 최소회전반경이 4.02 m → 3.49 m** 로, 각속도 추종이 **0.029 → 0.251 rad/s** 로 회복됐다.

증상이 "조향은 되는데 차가 안 돈다" 라서 원인을 찾기 어려웠다. 좌우 조향각을
따로 찍어보지 않으면 안 보인다.

### (6) 그 밖에 고친 것

* `parking_world.launch.py` 의 `IfCondition(['not ', gui])` → Jazzy 에서
  `invalid condition expression ... but got 'not false'` 로 죽는다.
  `UnlessCondition(gui)` 로 교체. (`gui:=false` 경로가 아예 안 떴다)
* `ackermann_steering_controller` 파라미터명이 바뀌었다. `front_wheels_names` /
  `rear_wheels_names` / `*_wheel_track` / `*_wheels_radius` 는 DEPRECATED 경고가 뜬다.
  → `traction_joints_names` / `steering_joints_names` / `traction_track_width` /
  `steering_track_width` / `traction_wheels_radius` 로 교체.
* controller spawner 가 헤드리스 기동 직후 `Switch controller timed out after
  5 seconds` 로 실패한다 (gz 가 렌더러 초기화로 바쁨). `--switch-timeout 60` 추가.
* 좀비 `gz sim` 프로세스가 남으면 **다른 인스턴스의 토픽을 잡아** 측정이 오염된다.
  `kill_sim.sh` 로 항상 정리하고, `pgrep -fc 'gz[ ]sim'` 로 1개인지 확인할 것.
