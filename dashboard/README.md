# 관제 대시보드 (팀원 B)

차량형 로봇 자율 발렛파킹 프로젝트의 관제 대시보드 백엔드. 계획 근거는 이슈 #8.

## 실행

```bash
npm run db:seed     # parking_spots.json -> SQLite (멱등)
npm run db:reset    # DB 삭제 후 재적재
```

의존성 설치가 필요 없다. Node 22.5+ 내장 `node:sqlite` 와 네이티브 TypeScript
타입 스트리핑만 쓴다. (`npm install` 불필요)

## 구조

```
db/schema.sql   스키마 (SQLite)
db/seed.ts      parking_spots.json 적재 스크립트
data/valet.db   생성되는 DB — .gitignore 처리됨
```

## 좌표는 이 DB에 없다

주차면 좌표의 원본은 `ks` 브랜치의 다음 파일 하나뿐이다.

```
src/parking_lot_world/config/parking_spots.json
```

DB는 `spot_id`(`A01` 형식)로 참조만 하고 좌표를 갖지 않는다.
URDF 확정 후 최소 회전반경이 3.57 m를 넘으면 주차장을 통째로 재생성하는데(이슈 #6),
DB에 좌표 사본이 있으면 그 시점에 조용히 어긋나기 때문이다.

`seed.ts`는 JSON의 sha256을 `lot_version.checksum`에 저장한다. 재생성으로 체크섬이
바뀌면 새 `lot_version`을 만들고 기존 버전은 이력으로 남긴다.

프런트엔드는 좌표를 JSON에서 직접 받고, 상태를 API에서 받아 `spot_id`로 합친다.

## 현재 적재 결과

| 항목 | 값 |
|---|---|
| 주차면 | 54면 (A행 12 · B행 14 · C행 14 · D행 14) |
| 타입 | standard 50 · accessible 2 · ev 2 |
| 초기 상태 | FREE 32 · OCCUPIED 22 |

## 다음 단계

이슈 #8의 착수 순서를 따른다. 다음은 **규석과 토픽 계약 확정** 이며,
그전까지 수집기(`mission_event` 적재)는 시작하지 않는다.
