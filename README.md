# 개발새발 — 차량형 로봇 자율주행 · 자율 발렛파킹

제자리 회전이 불가능한 Ackermann 조향 로봇의 Nav2 자율주행과 자율 발렛파킹 시뮬레이션.
2인 프로젝트, 6주.

| 항목 | 내용 |
|---|---|
| 대상 환경 | Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic (gz-sim 8) / Nav2 1.3+ |
| 자율주행 | Smac Planner Hybrid-A\* + MPPI Controller |
| 로봇 | 커스텀 URDF/Xacro + ros2_control (후륜구동 · 전륜조향) |
| 관제 | React 19 + Express 5 + MySQL, rosbridge 연동 |

> 계획서에는 ROS 2 Humble 로 적혀 있으나 Ubuntu 24.04 에서는 설치가 불가능해
> **Jazzy + Gazebo Harmonic** 으로 확정했다. 경위는 [#3](../../issues/3).

---

## 브랜치 전략

`main` 은 **공통 문서와 설정만** 둔다. 코드는 각자 브랜치에서 작업하고 마지막에 합친다.

| 브랜치 | 담당 | 범위 |
|---|---|---|
| [`ks`](../../tree/ks) | 문규석 (팀원 A) | 로봇 모델, Nav2, Gazebo 월드, Behavior Tree |
| [`hj`](../../tree/hj) | 팀원 B | 관제 대시보드 (프런트 · API · DB) |
| `main` | — | README, 라이선스, git 설정 |

서로의 산출물이 필요하면 브랜치를 직접 참조한다. 예를 들어 대시보드는 주차면 좌표를
`ks` 에서 읽는다.

```bash
git show origin/ks:src/parking_lot_world/config/parking_spots.json
```

### 통합할 때

각자 브랜치에서 `main` 을 먼저 당겨 두면 마지막에 충돌이 나지 않는다.

```bash
git checkout ks        # 또는 hj
git merge main
```

두 브랜치는 디렉터리가 겹치지 않아 **코드 충돌은 없다.** 겹치는 파일은 이 README
하나뿐이고, 그래서 양쪽 내용을 여기 미리 합쳐 뒀다.

---

## 디렉터리 구조

### `ks` — 로봇 / Nav2

```
scripts/                 워크스페이스 개발용 실행 스크립트
└── nav2_tests/          주행 시험 도구 (mppi_probe, path_check …)
src/
├── parking_lot_world/   주차장 월드 · 맵 · Nav2 파라미터 · 주차면 테이블
└── valet_robot/         차량형 로봇 모델 + ros2_control 구성
```

`src/` 가 colcon 워크스페이스 루트라 ROS 패키지는 반드시 그 아래에 둔다.
패키지별 상세는 각 README 에 있다.

* `src/valet_robot/README.md` — 로봇 제원 · 설계 근거 · 실측값 · 환경 이슈
* `src/parking_lot_world/README.md` — 주차장 사양 · 주차면 데이터 · Nav2 설정 근거
* `src/valet_robot/meshes/README.md` — 차량 외형 메시 출처와 변형 방법

### `hj` — 관제 대시보드

```
dashboard/
├── frontend/            React 19 + Vite 7
│   └── src/  api/  hooks/  components/  types/  styles/
├── backend/             Express 5 + mysql2
│   └── src/  routes/  ros/  middleware/
├── db/                  MySQL 스키마 · seed
└── tools/               rosbridge 목 서버 · 문서 스냅샷 생성기
```

디렉터리마다 README 를 둬서 그 안의 구조와 설계 의도를 적는다.
`backend/src/ros/contract.ts` 가 토픽 계약을 코드로 옮긴 유일한 곳이다.

---

## 두 파트가 만나는 지점

### 1. 주차면 좌표 — `parking_spots.json`

주차장 기하의 **유일한 원본**이다. 54면의 좌표와 후진 주차용 3개 포즈
(`goal_pose`, `prepark_pose`, `aisle_point`), 기둥, 해치존이 들어있다.

대시보드는 이 파일을 그대로 읽어 도면을 그리고 **DB 에는 복사하지 않는다.**
파일의 sha256 만 `lot_version.checksum` 에 저장해 재생성을 감지한다.

실제로 [#13](../../issues/13) 에서 통로를 넓혀 재생성하면서 주차면 y 좌표가 전부
바뀌었는데, 대시보드는 코드 수정 없이 새 좌표로 그렸다. 좌표 사본을 뒀다면 그 시점에
조용히 어긋났을 것이다.

기하를 바꾸려면 `tools/generate_parking_lot.py` 를 고쳐 재생성한다.
`worlds/`, `maps/`, `config/` 는 전부 자동 생성물이라 직접 고치면 덮어써진다.

### 2. rosbridge 토픽 계약

[#9](../../issues/9) 에서 확정했다. 메시지는 `std_msgs/String` 에 JSON 을 싣는다.

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/valet/spot_states` | 로봇 → 대시보드 | 54면 점유 스냅샷 (`transient_local`) |
| `/valet/mission_status` | 로봇 → 대시보드 | BT 노드 진행 보고 + 정차 오차 |
| `/valet/request` | 대시보드 → 로봇 | 입차/출차 요청 |
| `/amcl_pose` | 로봇 → 대시보드 | 로봇 현재 위치 (Nav2 표준 타입 그대로) |

정차 오차와 전후진 전환 횟수는 로봇 쪽에서 계산해 `PARK_DONE` 에 싣는다.
합격선은 `nav2_ackermann.yaml` 의 주차용 goal checker 값인 **0.12 m**.

---

## 실행

### 로봇 (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
colcon build --packages-select parking_lot_world valet_robot
source install/setup.bash
```

```bash
bash scripts/run_sim.sh                        # 서버 + 라이다 (헤드리스)
bash scripts/show_gui.sh                       # GUI 창을 따로 붙인다
ros2 run valet_robot ackermann_teleop_key.py   # 수동 주행 (w/s a/d e space)
ros2 run valet_robot demo_drive.py             # 자동 슬라롬 데모
bash scripts/kill_sim.sh                       # 전부 정리
```

> **`ros2 launch valet_robot valet_sim.launch.py` 를 직접 쓰지 말 것.**
> 렌더 경로 설정이 빠져 라이다 프레임이 90% 이상 유실된다.
> 반드시 `scripts/run_sim.sh` 를 거친다. 근거는 `src/valet_robot/README.md` 13장.

GUI 는 서버와 별개 프로세스로 띄운다. 이렇게 하면 깜빡임 없이 서버 RTF 1.0 을
유지한다 ([#12](../../issues/12)).

### 관제 대시보드 (macOS / Linux)

```bash
cd dashboard
npm install
```

MySQL 계정은 root 권한이 필요하므로 직접 만든다.

```bash
mysql -u root -p -e "
  CREATE DATABASE IF NOT EXISTS valet CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;
  CREATE USER IF NOT EXISTS 'valet'@'localhost' IDENTIFIED BY '원하는_비밀번호';
  GRANT ALL PRIVILEGES ON valet.* TO 'valet'@'localhost';
  FLUSH PRIVILEGES;"
```

```bash
cp .env.example .env     # DB_PASSWORD 채우기
npm run db:schema
npm run db:seed -- --dummy 8
npm run dev:api          # 터미널 1 — http://localhost:5174
npm run dev:web          # 터미널 2 — http://localhost:5173
```

백엔드나 MySQL 없이 프런트만 띄우면 **더미 데이터로 동작**한다.
주차장 도면은 언제나 실제 `parking_spots.json` 을 쓰므로 좌표는 더미가 아니다.

ROS 없이 전체 흐름을 보려면 목 rosbridge 를 띄운다.

```bash
npm run mock:ros         # 터미널 3 — ws://127.0.0.1:9090
```

실제 로봇에 붙일 때는 주소만 바꾼다.

```bash
ROSBRIDGE_URL=ws://<VM_IP>:9090 npm run dev:api
```

---

## 검증

```bash
bash scripts/smoke_test.sh              # 빌드 -> 모델검증 -> 기동 -> 주행 계측 8항목
ros2 run valet_robot check_model.py     # 모델 자기검증 (맵 패키지 정합성 포함)
bash scripts/render_test.sh             # 라이다 렌더 경로 3종 비교 (진단용)
```

`check_model.py` 는 URDF 의 축거·최소회전반경·풋프린트가 `parking_spots.json` 및
`nav2_ackermann.yaml` 과 어긋나면 실패한다. 두 패키지 중 하나만 고치는 사고를 막는다.

---

## 이슈

진행 상황과 설계 결정은 이슈로 남긴다.

| 이슈 | 내용 |
|---|---|
| [#3](../../issues/3) | Humble → Jazzy 전환 경위 |
| [#4](../../issues/4) | VM Gazebo 렌더링 문제와 RTF 측정값 |
| [#6](../../issues/6) | 최소 회전반경과 통로 폭 — #13 으로 해소 |
| [#7](../../issues/7) | keepout 필터를 켜면 주차가 막히는 함정 |
| [#8](../../issues/8) | 대시보드 DB 설계 |
| [#9](../../issues/9) | rosbridge 토픽 계약 |
| [#10](../../issues/10) | 로봇 1주차 완료 · 실측 8/8 |
| [#11](../../issues/11) | 대시보드 구현 경과 |
| [#12](../../issues/12) | 라이다 렌더링 해결 · 조향 조인트 순서 버그 |
| [#13](../../issues/13) | 통로 확장 재생성 · 코너 실패 원인 5건 |

---

## 라이선스

코드는 [MIT](LICENSE).

차량 외형 메시는 Gazebo Fuel 의 **Prius Hybrid** (OpenRobotics, 저자 Ian Chen) 를
우리 제원에 맞게 변형한 것이며 **CC0 1.0 퍼블릭 도메인**이다.
상세는 `src/valet_robot/meshes/README.md` 참고.
