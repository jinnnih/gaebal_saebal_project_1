# backend — 관제 API 서버

Express 5 + mysql2. rosbridge 수집기도 이 프로세스에서 같이 돈다.

```bash
npm run dev:api      # http://localhost:5174
```

## 구조

```
src/
├── index.ts              서버 부트스트랩. 라우트 붙이고 수집기 시작
├── config.ts             환경 변수를 읽는 유일한 곳
├── db.ts                 mysql2 커넥션 풀 + currentLotVersion()
├── layout.ts             parking_spots.json 로더 (워킹트리 → origin/ks 순)
├── middleware/
│   └── errors.ts         DB 미준비 시 원인을 알려주는 503 래퍼
├── routes/
│   ├── index.ts          /api 아래로 모아 붙인다
│   ├── layout.ts         GET /api/layout
│   ├── spots.ts          GET /api/spots
│   ├── requests.ts       요청 생성·조회·취소·이벤트
│   └── metrics.ts        GET /api/metrics, /api/health
└── ros/
    ├── contract.ts       #9 토픽 계약 — 토픽명·QoS·event 13종·상태 매핑
    ├── client.ts         rosbridge WebSocket 연결·재접속·발행
    └── collector.ts      구독한 메시지를 DB 에 적재
```

## 왜 수집기가 API 와 같은 프로세스인가

`POST /api/requests` 가 `/valet/request` 를 발행해야 하는데, 이때 수집기가 이미
열어 둔 WebSocket 을 그대로 쓰는 게 간단하다. 프로세스를 나누면 요청 발행 경로를
따로 만들어야 한다.

DB 가 준비 안 됐으면 수집기만 조용히 꺼지고 API 는 계속 뜬다. 그래야 처음
세팅할 때 무엇이 빠졌는지 `/api/health` 로 확인할 수 있다.

## ros/contract.ts 를 먼저 볼 것

로봇과 합의한 내용이 전부 여기 있다. 토픽 이름, QoS, `event` 13종,
이벤트 → 요청 상태 매핑, 주차 허용오차 0.12 m.
계약이 바뀌면 이 파일만 고치면 나머지가 따라온다.

`event` 목록과 DB `mission_event.event` ENUM 은 **같아야 한다.**
한쪽만 고치면 적재가 조용히 실패한다.
