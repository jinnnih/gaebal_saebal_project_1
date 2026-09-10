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
const REPO_ROOT = resolve(import.meta.dirname, '..');
/** 통합 후 로봇 패키지가 놓일 자리. 아직 병합 전이라 없으면 ks 브랜치에서 읽는다. */
const SPOTS_LOCAL = 'ros/src/parking_lot_world/config/parking_spots.json';
const SPOTS_ON_KS = 'src/parking_lot_world/config/parking_spots.json';

const local = resolve(REPO_ROOT, SPOTS_LOCAL);
const raw = existsSync(local)
  ? readFileSync(local, 'utf8')
  : execFileSync('git', ['show', `origin/ks:${SPOTS_ON_KS}`],
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

/** std_msgs/String 토픽 — JSON 을 문자열로 싣는다 (#9 Q2). */
function publish(topic: string, body: object) {
  publishRaw(topic, { data: JSON.stringify(body) });
}

/** 표준 메시지 토픽 — msg 자체가 객체다 (/amcl_pose). */
function publishRaw(topic: string, msg: object) {
  const frame = JSON.stringify({ op: 'publish', topic, msg });
  for (const [sock, topics] of subs) {
    if (topics.has(topic) && sock.readyState === sock.OPEN) sock.send(frame);
  }
}

// ── 로봇 위치 시뮬레이션 ─────────────────────────────────────
const robot = {
  x: layout.entry_pose[0],
  y: layout.entry_pose[1],
  yaw: layout.entry_pose[2] ?? 0,
};

function publishPose() {
  publishRaw('/amcl_pose', {
    header: { frame_id: 'map' },
    pose: {
      pose: {
        position: { x: robot.x, y: robot.y, z: 0 },
        // 평면 주행이라 yaw 만 쓴다
        orientation: { x: 0, y: 0, z: Math.sin(robot.yaw / 2), w: Math.cos(robot.yaw / 2) },
      },
      covariance: new Array(36).fill(0),
    },
  });
}

/** 목표 지점까지 직선 보간으로 이동시킨다. 실제 Nav2 경로는 아니고 화면 확인용이다. */
async function driveTo(tx: number, ty: number, tyaw: number | null, seconds: number) {
  const steps = Math.max(1, Math.round(seconds * 10));   // 10 Hz
  const [sx, sy, syaw] = [robot.x, robot.y, robot.yaw];
  const goalYaw = tyaw ?? Math.atan2(ty - sy, tx - sx);
  // 짧은 쪽으로 회전
  let dyaw = goalYaw - syaw;
  while (dyaw > Math.PI) dyaw -= 2 * Math.PI;
  while (dyaw < -Math.PI) dyaw += 2 * Math.PI;

  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    robot.x = sx + (tx - sx) * t;
    robot.y = sy + (ty - sy) * t;
    robot.yaw = syaw + dyaw * t;
    await sleep(seconds * 1000 / steps);
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
  // 통로 합류점 -> 주차면 앞 대기지점 순으로 실제로 움직인다
  const meta = layout.spots.find((s: any) => s.id === spot);
  await driveTo(meta.aisle_point[0], meta.aisle_point[1], null, 2.5);

  await driveTo(meta.prepark_pose[0], meta.prepark_pose[1], meta.prepark_pose[2], 1.2);
  emit('PREPARK_REACHED', 'ParkManeuver', { spot_id: spot, err_m: 0.05 });
  await sleep(400);

  emit('PARK_STARTED', 'ParkManeuver', { spot_id: spot });
  // 후진 주차 — 주차면 중심까지
  await driveTo(meta.goal_pose[0], meta.goal_pose[1], meta.goal_pose[2], 2.2);

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

/** 출차 요청 하나를 재생한다. 주차의 역순이되 마지막이 출구다. */
async function runRetrieveMission(requestId: number, vehicleTag: string, spot: string | null) {
  const target = spot && state.get(spot)?.status === 'OCCUPIED'
    ? spot
    : [...state].find(([, v]) => v.status === 'OCCUPIED')?.[0];
  if (!target) { console.log('점유된 주차면이 없습니다'); return; }

  const meta = layout.spots.find((s: any) => s.id === target);
  let seq = 1;
  const emit = (event: string, bt_node: string | null, payload: object = {}) =>
    publish('/valet/mission_status',
      { stamp: now(), request_id: requestId, seq: seq++, event, bt_node, payload });

  console.log(`출차 #${requestId} ${vehicleTag} ← ${target}`);

  emit('REQUEST_ACCEPTED', null, { vehicle_tag: vehicleTag, kind: 'RETRIEVE' });
  await sleep(600);

  // 회수하러 가는 동안 그 면을 잠근다 — 다른 요청이 배정하면 안 된다
  state.set(target, { status: 'RESERVED', request_id: requestId });
  publish('/valet/spot_states', snapshot());

  emit('NAV_STARTED', null, { spot_id: target });
  await driveTo(meta.aisle_point[0], meta.aisle_point[1], null, 2.0);

  emit('UNPARK_STARTED', 'UnparkManeuver', { spot_id: target });
  // 전진 탈출 — 주차면 중심에서 대기지점으로
  await driveTo(meta.prepark_pose[0], meta.prepark_pose[1], meta.prepark_pose[2], 1.5);
  await driveTo(meta.aisle_point[0], meta.aisle_point[1], null, 1.0);
  emit('UNPARK_DONE', 'UnparkManeuver', { spot_id: target });

  // 면을 비운다
  state.set(target, { status: 'FREE', request_id: null });
  publish('/valet/spot_states', snapshot());

  // 출구까지
  await driveTo(layout.exit_pose[0], layout.exit_pose[1], layout.exit_pose[2], 3.0);
  emit('EXIT_REACHED', 'ReportStatus', { spot_id: target });
  console.log(`  → 출차 완료 (${target} 비움)`);
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
      if (op.topic === '/amcl_pose') publishPose();
      if (op.topic === '/valet/spot_states') {
        sock.send(JSON.stringify({
          op: 'publish', topic: op.topic, msg: { data: JSON.stringify(snapshot()) },
        }));
      }
    } else if (op.op === 'advertise') {
      console.log(`발행 등록: ${op.topic}`);
    } else if (op.op === 'publish' && op.topic === '/valet/request') {
      const req = JSON.parse(op.msg?.data ?? '{}');
      const run = req.kind === 'RETRIEVE' ? runRetrieveMission : runParkMission;
      run(req.request_id, req.vehicle_tag ?? '무명', req.spot_id ?? null)
        .catch((e) => console.error(e));
    }
  });

  sock.on('close', () => { subs.delete(sock); console.log('클라이언트 종료'); });
});

// 하트비트 — latch 만으로는 로봇이 살아있는지 알 수 없다는 규석 지적 (#9 Q4)
setInterval(() => publish('/valet/spot_states', snapshot()), 1000);

// 로봇 위치는 10 Hz. 실제 AMCL 도 이 정도 주기로 낸다.
setInterval(publishPose, 100);

const free = [...state.values()].filter((v) => v.status === 'FREE').length;
console.log(`목 rosbridge  ws://127.0.0.1:${PORT}`);
console.log(`레이아웃      ${layout.lot_name} · ${layout.spots.length}면 · lot_checksum ${checksum}`);
console.log(`초기 상태     FREE ${free} / OCCUPIED ${state.size - free}`);
