# ros — 로봇 / Nav2 (팀원 A)

`ks` 브랜치의 로봇 코드가 들어올 자리다. 아직 병합 전이라 비어 있다.

## 들어올 모양

```
ros/
├── scripts/                 워크스페이스 개발용 실행 스크립트
│   └── nav2_tests/          주행 시험 도구
└── src/                     ← colcon 워크스페이스 루트
    ├── parking_lot_world/
    └── valet_robot/
```

`ks` 브랜치에서는 이것들이 저장소 최상단에 있다(`src/`, `scripts/`).
병합할 때 `ros/` 아래로 한 겹 내려온다.

`colcon build` 는 `ros/` 에서 실행한다.

```bash
cd ros
colcon build --packages-select parking_lot_world valet_robot
source install/setup.bash
```

## 대시보드가 이 경로를 본다

주차면 좌표를 아래 경로에서 읽는다.

```
ros/src/parking_lot_world/config/parking_spots.json
```

병합 전에는 이 파일이 없으므로 `git show origin/ks:src/...` 로 대신 읽는다.
백엔드·프런트·목 서버 모두 같은 순서로 찾는다 — 워킹트리에 있으면 그걸 쓰고,
없으면 `ks` 브랜치에서 읽는다.

즉 **병합 전후 모두 동작한다.** 병합되면 자동으로 워킹트리 쪽을 보게 된다.

## 주의

`src/` 가 colcon 워크스페이스 루트라는 규칙 때문에 ROS 패키지는 반드시
`ros/src/` 바로 아래에 둔다. 한 겹 더 넣으면 `colcon build` 가 못 찾는다.
