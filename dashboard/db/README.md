# db — MySQL 스키마와 적재

```bash
mysql -u valet -p valet < db/schema.sql
node --env-file=.env db/seed.ts --dummy 8
```

## 파일

```
schema.sql   테이블 6종 + 뷰 2종
seed.ts      parking_spots.json 적재 (+ --dummy 로 더미 요청 생성)
```

## 테이블

| 테이블 | 역할 |
|---|---|
| `lot_version` | 레이아웃 스냅샷. `checksum` 으로 재생성을 감지한다 |
| `parking_spot` | 주차면 카탈로그 — **좌표 없음** |
| `spot_state` | 현재 점유 상태 |
| `valet_request` | 입차/출차 요청 |
| `mission_event` | BT 노드 이벤트 로그 (`payload` JSON) |
| `park_metric` | 정량 지표 — 계획서 6장 산출물 |

뷰 `v_current_spots` 는 최신 레이아웃의 주차면과 상태를 합쳐 준다.
`v_request_timeline` 은 요청별 이벤트 수와 지표를 묶어 큐 UI 가 한 번에 읽는다.

## 좌표를 넣지 않는 이유

`parking_spots.json` 이 단일 진실원본이고 DB 는 `spot_id` 로 참조만 한다.

#13 에서 실제로 주차장이 재생성돼 주차면 y 좌표가 전부 바뀌었다. 좌표 사본이
DB 에 있었다면 그때 조용히 어긋나 대시보드가 틀린 자리에 로봇을 그렸을 것이다.

대신 JSON 의 sha256 을 `lot_version.checksum` 에 저장한다. 체크섬이 바뀌면
seed 가 새 `lot_version` 을 만들고 기존 버전은 이력으로 남는다.
수집기도 로봇이 보낸 `lot_checksum` 이 DB 와 다르면 경고한다.

## seed 는 멱등하다

같은 checksum 이 이미 있으면 아무것도 하지 않는다. 여러 번 돌려도 안전하다.

`--dummy N` 은 요청 N 건과 그 이벤트·지표를 만든다. 실제 로봇 노드가 아직
발행하지 않는 동안 화면과 지표를 검증하기 위한 것이다.

## 스키마를 고칠 때

`mission_event.event` ENUM 은 `backend/src/ros/contract.ts` 의 `EVENTS` 와
**같아야 한다.** 한쪽만 고치면 적재가 조용히 실패한다.
