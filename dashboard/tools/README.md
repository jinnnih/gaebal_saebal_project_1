# tools — 개발용 도구

## mock_rosbridge.ts

ROS 없이 전체 흐름을 돌려보기 위한 rosbridge 목 서버.

```bash
npm run mock:ros     # ws://127.0.0.1:9090
```

로봇 쪽에 주차면 관리 노드와 BT 노드가 아직 없어서 `/valet/spot_states` 와
`/valet/mission_status` 는 발행자가 없다(#9). 규석이 "더미 퍼블리셔로 테스트하시는
게 빠를 것" 이라고 한 그것이다.

`#9` 계약대로만 말하므로 실제 노드로 바꿔도 수집기는 그대로 돈다.

- 구독 즉시 스냅샷 1회 (`transient_local` 흉내) + 1 Hz 하트비트
- `/valet/request` 를 받으면 주차 시나리오를 실제 시간 흐름대로 재생
- 정차 오차를 무작위로 내서 허용오차(0.12 m) 초과 시 `FAILED` 경로도 탄다

레이아웃은 더미가 아니라 `ks` 브랜치의 `parking_spots.json` 을 읽는다.
