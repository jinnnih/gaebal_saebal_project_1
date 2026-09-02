# 관제 대시보드

차량형 로봇 자율 발렛파킹 프로젝트의 관제 대시보드. 담당은 팀원 B, 브랜치는 `hj`.
설계 근거는 이슈 #8, 토픽 계약은 #9.

```
frontend/   React 19 + Vite 7 + TypeScript
backend/    Express 5 + mysql2
db/         MySQL 스키마 + seed
```

npm workspaces 로 묶여 있어 루트에서 `npm install` 한 번이면 된다.

---

## 준비

### 1. MySQL 계정

root 비밀번호가 필요하므로 직접 실행한다.

```bash
mysql -u root -p -e "
  CREATE DATABASE IF NOT EXISTS valet CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;
  CREATE USER IF NOT EXISTS 'valet'@'localhost' IDENTIFIED BY '원하는_비밀번호';
  GRANT ALL PRIVILEGES ON valet.* TO 'valet'@'localhost';
  FLUSH PRIVILEGES;"
```

### 2. 환경 변수

```bash
cp .env.example .env     # DB_PASSWORD 를 위에서 정한 값으로
```

### 3. 스키마와 데이터

```bash
mysql -u valet -p valet < db/schema.sql
node --env-file=.env db/seed.ts --dummy 8
```

`seed` 는 멱등하다. 같은 checksum 의 레이아웃이 이미 있으면 건너뛴다.

---

## 실행

```bash
npm run dev:api    # 터미널 1 — http://localhost:5174
npm run dev:web    # 터미널 2 — http://localhost:5173
```

프런트는 `/api` 를 백엔드로 프록시한다.

### 백엔드 없이 프런트만

```bash
npm run dev:web
```

MySQL 이 없어도 화면이 뜬다. 헤더에 **더미 데이터** 배지가 뜨고 요청 폼은 잠긴다.
주차장 도면은 더미가 아니라 언제나 실제 좌표를 쓴다.

---

## 좌표는 DB 에 없다

주차면 좌표의 원본은 `ks` 브랜치의 파일 하나뿐이다.

```
src/parking_lot_world/config/parking_spots.json
```

DB 는 `spot_id`(`A01` 형식)로 참조만 한다. URDF 확정 후 최소 회전반경이 기준을 넘으면
주차장을 통째로 재생성하는데(#6), DB 에 좌표 사본이 있으면 그 시점에 조용히 어긋나기 때문이다.

`seed.ts` 는 JSON 의 sha256 을 `lot_version.checksum` 에 저장한다. 재생성으로 체크섬이
바뀌면 새 `lot_version` 을 만들고 기존 버전은 이력으로 남긴다.

개발 중에는 Vite 플러그인이 `/parking_spots.json` 경로로 원본을 직접 서빙한다.
규석의 `src/parking_lot_world/README.md` 7장이 문서화한 경로와 같다.

---

## 스키마

| 테이블 | 역할 |
|---|---|
| `lot_version` | 레이아웃 스냅샷 (checksum, 재생성 감지) |
| `parking_spot` | 주차면 카탈로그 — 좌표 없음 |
| `spot_state` | 현재 점유 상태 |
| `valet_request` | 입차/출차 요청 — 5주차 큐 UI 의 원천 |
| `mission_event` | BT 노드 이벤트 로그 (`payload` JSON) |
| `park_metric` | 정량 지표 — 계획서 6장 산출물 |

뷰 `v_current_spots` 는 최신 레이아웃의 주차면과 상태를 합쳐 준다.
`v_request_timeline` 은 요청별 이벤트 수와 지표를 묶어 큐 UI 가 한 번에 읽는다.

`park_metric.within_tolerance` 는 생성 컬럼이다. `nav2_ackermann.yaml` 의 주차용
goal checker 허용오차 **0.12 m** 를 그대로 반영한다.

`mission_event.seq` 는 요청 내 순번이다. #9 에서 규석이 요청한 필드로,
이벤트가 순서 뒤집혀 도착해도 정렬할 수 있고 유실이 드러난다.

---

## API

| 메서드 | 경로 | 내용 |
|---|---|---|
| GET | `/api/layout` | 정적 레이아웃 (DB 무관) |
| GET | `/api/spots` | 주차면 현재 상태 |
| GET | `/api/requests` | 요청 큐 |
| POST | `/api/requests` | 요청 생성 |
| POST | `/api/requests/:id/cancel` | 요청 취소 (`ABORTED` 이벤트 기록) |
| GET | `/api/requests/:id/events` | 이벤트 타임라인 |
| GET | `/api/metrics` | 정량 지표 집계 |
| GET | `/api/health` | 상태 확인 |

---

## 아직 안 된 것

로봇 쪽에 주차면 관리 노드와 BT 노드가 없어서 `/valet/spot_states` 와
`/valet/mission_status` 는 **현재 발행자가 없다**(#9 규석 답변). 그래서 지금은
더미 데이터로 화면을 검증하고 있다.

계약이 확정돼 있으므로 실제 노드가 붙을 때 바꿀 것은 수집기 한 겹뿐이다.

- [ ] rosbridge WebSocket 수집기 (`mission_event` / `spot_state` 적재)
- [ ] `POST /api/requests` 에서 `/valet/request` 토픽 발행
- [ ] 로봇 위치 오버레이 (`/amcl_pose`)
- [ ] 폴링을 WebSocket 구독으로 교체
