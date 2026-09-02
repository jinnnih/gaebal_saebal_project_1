# gaebal_saebal_project_1 — `ks` 브랜치

차량형(Ackermann) 로봇 **자율 발렛파킹** 시뮬레이션. 팀원 A(로봇/Nav2) 작업 브랜치.

대상 환경: **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic (gz-sim 8) / Nav2 1.3+**

## 구성

```
src/parking_lot_world/   주차장 월드·맵·Nav2 파라미터·주차면 테이블
src/valet_robot/         차량형 로봇 모델 + ros2_control 구성
scripts/                 워크스페이스 개발용 실행 스크립트
```

패키지별 상세는 각 README 를 볼 것.

* [`src/valet_robot/README.md`](src/valet_robot/README.md) — 로봇 제원·설계 근거·실측값·환경 이슈
* [`src/parking_lot_world/README.md`](src/parking_lot_world/README.md) — 주차장 사양·주차면 데이터·Nav2 설정 근거
* [`src/valet_robot/meshes/README.md`](src/valet_robot/meshes/README.md) — 차량 외형 메시 출처와 변형 방법

## 빌드

```bash
colcon build --packages-select parking_lot_world valet_robot
source install/setup.bash
```

## 실행

```bash
# 서버 + 라이다 (헤드리스)
bash scripts/run_sim.sh

# 보고 싶으면 GUI 창을 따로 붙인다 (VM 데스크톱 로그인 필요)
bash scripts/show_gui.sh

# 수동 주행 (w/s 속도, a/d 조향, e 조향중립, space 정지)
ros2 run valet_robot ackermann_teleop_key.py

# 자동 슬라롬 데모
ros2 run valet_robot demo_drive.py

# 전부 정리
bash scripts/kill_sim.sh
```

> `ros2 launch valet_robot valet_sim.launch.py` 를 직접 쓰면 렌더 경로 설정이
> 빠져서 라이다 프레임이 90% 이상 유실된다. **반드시 `scripts/run_sim.sh` 를 거칠 것.**
> 근거는 `src/valet_robot/README.md` 13장.

## 검증

```bash
bash scripts/smoke_test.sh          # 빌드 -> 모델검증 -> 기동 -> 주행 계측 8항목
ros2 run valet_robot check_model.py # 모델 자기검증만 (맵 패키지 정합성 포함)
bash scripts/render_test.sh         # 라이다 렌더 경로 3종 비교 (진단용)
```
