/**
 * rosbridge 목 서버 — ROS 없이 수집기와 대시보드를 검증한다.
 *
 *   node tools/mock_rosbridge.ts        # ws://127.0.0.1:9090
 *
 * 규석이 #9 답변에서 "주차면 관리 노드와 BT 노드가 없어서 T1/T2 는 아직 아무도
 * 발행하지 않는다. 더미 퍼블리셔로 테스트하시는 게 빠를 것" 이라고 한 그 더미다.
 * 확정된 계약대로만 말하므로, 실제 노드로 바꿔도 수집기는 그대로 돈다.
 *
 * 하는 일
 *   - /valet/spot_states 를 latched 처럼 구독 즉시 1회 + 1 Hz 하트비트로 발행 (#9 Q4)
 *   - /valet/request 를 받으면 주차 시나리오를 실제 시간 흐름대로 재생
 *   - /valet/mission_status 로 이벤트를 순서대로 발행 (seq 포함)
 */
import { WebSocketServer, type WebSocket } from 'ws';
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { createHash } from 'node:crypto';

const PORT = Number(process.env.MOCK_PORT ?? 9090);
const REPO_ROOT = resolve(import.meta.dirname, '../..');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
const raw = existsSync(local)
  ? readFileSync(local, 'utf8')
  : execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`],
      { cwd: REPO_ROOT, encoding: 'utf8', maxBuffer: 32 << 20 });
const layout = JSON.parse(raw);
const checksum = createHash('sha256').update(raw).digest('hex').slice(0, 8);

type Status = 'FREE' | 'RESERVED' | 'OCCUPIED' | 'BLOCKED';
const state = new Map<string, { status: Status; request_id: number | null }>(
  layout.spots.map((s: any) => [s.id, {
    status: (s.initially_occupied ? 'OCCUPIED' : 'FREE') as Status,
    request_id: null,
  }]));

const subs = new Map<WebSocket, Set<string>>();
const now = () => new Date().toISOString();
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

function publish(topic: string, body: object) {
  const frame = JSON.stringify({ op: 'publish', topic, msg: { data: JSON.stringify(body) } });
  for (const [sock, topics] of subs) {
    if (topics.has(topic) && sock.readyState === sock.OPEN) sock.send(frame);
  }
}

const snapshot = () => ({
  stamp: now(),
  lot_checksum: checksum,
  spots: [...state].map(([id, v]) => ({ id, status: v.status, request_id: v.request_id })),
});

/** 입차 요청 하나를 실제 시간 흐름대로 재생한다. */
async function runParkMission(requestId: number, vehicleTag: string, wanted: string | null) {
  const spot = wanted && state.get(wanted)?.status === 'FREE'
    ? wanted
    : [...state].find(([, v]) => v.status === 'FREE')?.[0];
  if (!spot) { console.log('빈 주차면이 없습니다'); return; }

  let seq = 1;
  const emit = (event: string, bt_node: string | null, payload: object = {}) =>
    publish('/valet/mission_status',
      { stamp: now(), request_id: requestId, seq: seq++, event, bt_node, payload });

  console.log(`요청 #${requestId} ${vehicleTag} → ${spot}`);

  emit('REQUEST_ACCEPTED', null, { vehicle_tag: vehicleTag });
  await sleep(700);

  emit('SPOT_SELECTED', 'FindParkingSpot', { spot_id: spot });
  state.set(spot, { status: 'RESERVED', request_id: requestId });
  emit('SPOT_RESERVED', 'FindParkingSpot', { spot_id: spot });
  publish('/valet/spot_states', snapshot());
  await sleep(900);

  emit('NAV_STARTED', null, { spot_id: spot });
  await sleep(2500);

  emit('PREPARK_REACHED', 'ParkManeuver', { spot_id: spot, err_m: 0.05 });
  await sleep(900);

  emit('PARK_STARTED', 'ParkManeuver', { spot_id: spot });
  await sleep(2200);

  // 대부분 허용오차(0.12 m) 이내, 가끔 벗어나게 해서 실패 경로도 확인한다
  const errM = +(0.04 + Math.random() * 0.13).toFixed(3);
  const ok = errM <= 0.12;
  emit(ok ? 'PARK_DONE' : 'FAILED', 'ParkManeuver', {
    spot_id: spot,
    err_m: errM,
    heading_deg: +((Math.random() * 7 - 3.5).toFixed(2)),
    shunts: Math.floor(Math.random() * 4),
  });

  state.set(spot, ok
    ? { status: 'OCCUPIED', request_id: requestId }
    : { status: 'FREE', request_id: null });
  publish('/valet/spot_states', snapshot());
  console.log(`  → ${ok ? '주차 완료' : '실패'} (오차 ${errM} m)`);
}

const wss = new WebSocketServer({ port: PORT });

wss.on('connection', (sock) => {
  subs.set(sock, new Set());
  console.log(`클라이언트 접속 (총 ${wss.clients.size})`);

  sock.on('message', (buf) => {
    let op: any;
    try { op = JSON.parse(String(buf)); } catch { return; }

    if (op.op === 'subscribe') {
      subs.get(sock)!.add(op.topic);
      console.log(`구독: ${op.topic}${op.qos?.durability ? ` (${op.qos.durability})` : ''}`);
      // transient_local 흉내 — 구독 즉시 마지막 값을 준다 (#9 Q5)
      if (op.topic === '/valet/spot_states') {
        sock.send(JSON.stringify({
          op: 'publish', topic: op.topic, msg: { data: JSON.stringify(snapshot()) },
        }));
      }
    } else if (op.op === 'advertise') {
      console.log(`발행 등록: ${op.topic}`);
    } else if (op.op === 'publish' && op.topic === '/valet/request') {
      const req = JSON.parse(op.msg?.data ?? '{}');
      runParkMission(req.request_id, req.vehicle_tag ?? '무명', req.spot_id ?? null)
        .catch((e) => console.error(e));
    }
  });

  sock.on('close', () => { subs.delete(sock); console.log('클라이언트 종료'); });
});

// 하트비트 — latch 만으로는 로봇이 살아있는지 알 수 없다는 규석 지적 (#9 Q4)
setInterval(() => publish('/valet/spot_states', snapshot()), 1000);

const free = [...state.values()].filter((v) => v.status === 'FREE').length;
console.log(`목 rosbridge  ws://127.0.0.1:${PORT}`);
console.log(`레이아웃      ${layout.lot_name} · ${layout.spots.length}면 · lot_checksum ${checksum}`);
console.log(`초기 상태     FREE ${free} / OCCUPIED ${state.size - free}`);
