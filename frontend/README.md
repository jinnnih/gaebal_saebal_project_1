# frontend — 관제 대시보드 화면

React 19 + Vite 7 + TypeScript.

```bash
npm run dev:web      # http://localhost:5173
```

백엔드나 MySQL 이 없어도 뜬다. 헤더에 **더미 데이터** 배지가 붙고 요청 폼이 잠긴다.

## 구조

```
src/
├── main.tsx              진입점
├── App.tsx               레이아웃 조립만
├── types/index.ts        API·레이아웃 타입
├── api/
│   ├── client.ts         fetch 래퍼. 백엔드가 없으면 더미로 대체
│   └── dummy.ts          더미 상태·요청·지표 생성
├── hooks/
│   └── useValetState.ts  전체 상태 + 폴링. 나중에 WebSocket 으로 교체할 지점
├── components/
│   ├── Header.tsx        주차장 이름·공차·점유율·데이터 출처 배지
│   ├── ParkingMap.tsx    주차장 평면도 (SVG)
│   ├── RequestQueue.tsx  요청 폼 + 진행 중/완료 목록
│   └── SidePanel.tsx     점유 현황·정량 지표·레이아웃 정보
└── styles/index.css      CSS 커스텀 프로퍼티 기반 다크 테마
```

## 좌표계

`parking_spots.json` 의 map 프레임을 그대로 쓴다. 단위는 m 이고 y 축이 **위쪽이 +** 인데
SVG 는 아래쪽이 + 라서 `transform="scale(1,-1)"` 로 뒤집는다.
글자는 뒤집히면 안 되므로 별도 그룹에서 y 부호만 바꿔 그린다.

좌표를 코드에 하드코딩하지 않는다. 규석이 주차장을 재생성하면(#13 처럼) 좌표가
전부 바뀌는데, 원본을 읽고 있으면 코드 수정 없이 따라간다.

## 레이아웃은 어디서 오나

`/parking_spots.json` 으로 받는다. 개발 중에는 `vite.config.ts` 의 플러그인이
`ks` 브랜치 원본을 직접 서빙한다. 프런트에 좌표 사본을 두지 않기 위함이다.

경로는 규석의 `src/parking_lot_world/README.md` 7장이 문서화한 것과 같다.

## 실시간 연동

지금은 `useValetState` 가 폴링한다. 수집기가 rosbridge 로 받은 것을 DB 에 넣고
있으므로, 나중에 이 훅만 WebSocket 구독으로 바꾸면 나머지는 그대로 둬도 된다.
