# 개발새발 — 차량형 로봇 자율주행 · 자율 발렛파킹

제자리 회전이 불가능한 Ackermann 조향 로봇의 Nav2 자율주행과 자율 발렛파킹 시뮬레이션.
2인 프로젝트, 6주.

| 항목 | 내용 |
|---|---|
| 시뮬레이터 | Gazebo Harmonic (gz-sim) |
| 미들웨어 | ROS 2 Jazzy / Ubuntu 24.04 |
| 자율주행 | Nav2 — Smac Hybrid-A\* 플래너 + MPPI 컨트롤러 |
| 로봇 | 커스텀 URDF/Xacro + ros2_control (후륜구동 · 전륜조향) |
| 관제 | React + Express + MySQL, rosbridge 연동 |

> 계획서에는 ROS 2 Humble 로 적혀 있으나 Ubuntu 24.04 에서는 설치가 불가능해
> **Jazzy + Gazebo Harmonic** 으로 확정했다. 경위는 이슈 #3 참고.

---

## 브랜치 전략

`main` 은 **마지막에 통합**한다. 평소 작업은 각자 브랜치에서 한다.

| 브랜치 | 담당 | 범위 |
|---|---|---|
| `ks` | 문규석 (팀원 A) | 로봇 모델, Nav2, Gazebo 월드, Behavior Tree |
| `hj` | 팀원 B | 관제 대시보드 (프런트 · API · DB) |
| `main` | — | 통합 전까지 비워둔다 |

서로의 산출물이 필요하면 브랜치를 직접 참조한다. 예를 들어 대시보드는 주차면 좌표를
`ks` 에서 읽는다.

```bash
git show origin/ks:src/parking_lot_world/config/parking_spots.json
```

---

## 디렉터리 구조

두 사람의 산출물이 한 레포에 공존하되 디렉터리로 분리된다.

```
src/                        ROS 2 colcon 워크스페이스 (ks 브랜치)
├── parking_lot_world/      주차장 월드 · 맵 · Nav2 설정 · 주차면 좌표
└── valet_robot/            차량형 로봇 URDF · ros2_control · 텔레옵

dashboard/                  관제 대시보드 (hj 브랜치)
├── frontend/               React + Vite + TypeScript
├── backend/                Express + mysql2
└── db/                     MySQL 스키마 · seed
```

`src/` 를 colcon 워크스페이스 루트로 쓰는 구조라 ROS 패키지는 반드시 그 아래에 둔다.

---

## 두 파트가 만나는 지점

### 1. 주차면 좌표 — `parking_spots.json`

주차장 기하의 **유일한 원본**이다. 54면의 좌표와 후진 주차용 3개 포즈
(`goal_pose`, `prepark_pose`, `aisle_point`), 기둥, 해치존이 들어있다.

대시보드는 이 파일을 그대로 읽어 도면을 그리고, **DB 에는 복사하지 않는다.**
좌표가 바뀌는 경우(#6) DB 사본이 조용히 어긋나기 때문이다. 대신 파일의 sha256 을
`lot_version.checksum` 에 저장해 재생성을 감지한다.

기하를 바꾸려면 `tools/generate_parking_lot.py` 를 고쳐 재생성한다.
`worlds/`, `maps/`, `config/` 는 전부 자동 생성물이다.

### 2. rosbridge 토픽 계약

이슈 #9 에서 확정했다. 메시지는 `std_msgs/String` 에 JSON 을 싣는다.

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/valet/spot_states` | 로봇 → 대시보드 | 54면 점유 스냅샷 (`transient_local`) |
| `/valet/mission_status` | 로봇 → 대시보드 | BT 노드 진행 보고 + 정차 오차 |
| `/valet/request` | 대시보드 → 로봇 | 입차/출차 요청 |
| `/amcl_pose` | 로봇 → 대시보드 | 로봇 현재 위치 |

정차 오차와 전후진 전환 횟수는 로봇 쪽에서 계산해 `PARK_DONE` 에 싣는다.
합격선은 `nav2_ackermann.yaml` 의 주차용 goal checker 값인 **0.12 m**.

---

## 실행

### 로봇 (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
colcon build --packages-select parking_lot_world valet_robot
source install/setup.bash
ros2 launch valet_robot valet_sim.launch.py rviz:=true
```

Gazebo GUI 는 VM 에서 RTF 0.17 까지 떨어지므로 헤드리스 + RViz 를 권장한다. 자세한 내용은 #4.

### 관제 대시보드 (macOS / Linux)

```bash
cd dashboard
npm install
cp .env.example .env        # MySQL 계정 정보를 채운다
npm run db:schema
npm run db:seed -- --dummy 8
npm run dev:api             # 터미널 1 — http://localhost:5174
npm run dev:web             # 터미널 2 — http://localhost:5173
```

백엔드나 MySQL 없이 프런트만 띄우면 **더미 데이터로 동작**한다.
주차장 도면은 언제나 실제 `parking_spots.json` 을 쓰므로 좌표는 더미가 아니다.

---

## 이슈

진행 상황과 설계 결정은 이슈로 남긴다. 특히 아래는 다시 볼 일이 많다.

| 이슈 | 내용 |
|---|---|
| #3 | Humble → Jazzy 전환 경위 |
| #4 | VM Gazebo 렌더링 문제와 RTF 측정값 |
| #6 | 최소 회전반경과 통로 폭 (URDF 확정으로 해소) |
| #7 | keepout 필터를 켜면 주차가 막히는 함정 |
| #8 | 대시보드 DB 설계 |
| #9 | rosbridge 토픽 계약 |
| #10 | 로봇 1주차 완료 · 실측 8/8 |
