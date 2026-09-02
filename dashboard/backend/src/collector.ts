/**
 * rosbridge → MySQL 수집기
 *
 *   node --env-file=../.env src/collector.ts
 *
 * #9 에서 확정된 토픽 계약을 그대로 구현한다.
 *
 *   구독  /valet/spot_states     54면 점유 스냅샷 (transient_local)
 *   구독  /valet/mission_status  BT 노드 진행 보고 → mission_event
 *   발행  /valet/request         대시보드 요청
 *
 * 메시지는 std_msgs/String 에 JSON 문자열을 싣는다 (#9 Q2).
 * Node 22+ 내장 WebSocket 을 쓰므로 별도 의존성이 없다.
 */
import { pool, currentLotVersion } from './db.ts';

const URL = process.env.ROSBRIDGE_URL ?? 'ws://127.0.0.1:9090';
const RECONNECT_MS = 3000;

type MissionStatus = {
  stamp?: string;
  request_id: number;
  seq: number;
  event: string;
  bt_node?: string | null;
  payload?: Record<string, unknown>;
};
type SpotSnapshot = {
  stamp?: string;
  lot_checksum?: string;
  spots: { id: string; status: string; request_id?: number | null }[];
};

/** 이벤트 → valet_request.status. RECOVERY 는 상태를 바꾸지 않는다. */
const STATUS_OF: Record<string, string | null> = {
  REQUEST_ACCEPTED: 'PENDING',
  SPOT_SELECTED: 'ASSIGNED',
  SPOT_RESERVED: 'ASSIGNED',
  NAV_STARTED: 'NAVIGATING',
  PREPARK_REACHED: 'PARKING',
  PARK_STARTED: 'PARKING',
  PARK_DONE: 'PARKED',
  UNPARK_STARTED: 'UNPARKING',
  UNPARK_DONE: 'COMPLETED',
  EXIT_REACHED: 'COMPLETED',
  RECOVERY: null,
  FAILED: 'FAILED',
  ABORTED: 'CANCELLED',
};

const toMysqlTime = (iso?: string) =>
  (iso ? new Date(iso) : new Date()).toISOString().slice(0, 23).replace('T', ' ');

let versionId: number | null = null;
let lastChecksumWarn = '';

async function handleSpotStates(snap: SpotSnapshot) {
  if (!versionId) return;

  // 맵이 재생성됐는데 대시보드가 옛 좌표를 그리고 있는 상황을 잡는다. (#6, #8)
  if (snap.lot_checksum) {
    const [[v]] = await pool.query<any>('SELECT checksum FROM lot_version WHERE id = ?', [versionId]);
    if (v && !v.checksum.startsWith(snap.lot_checksum) && lastChecksumWarn !== snap.lot_checksum) {
      lastChecksumWarn = snap.lot_checksum;
      console.warn(`[경고] 로봇의 lot_checksum(${snap.lot_checksum}) 이 DB(${v.checksum.slice(0, 8)}) 와 다릅니다.`);
      console.warn('       parking_spots.json 이 재생성된 것 같습니다. seed 를 다시 돌리세요.');
    }
  }

  // 전체 스냅샷이므로(#9 Q3) 바뀐 것만 골라 쓴다.
  const [rows] = await pool.query<any[]>(
    'SELECT spot_id, status, request_id FROM spot_state WHERE lot_version_id = ?', [versionId]);
  const cur = new Map(rows.map((r) => [r.spot_id, r]));

  const changed = snap.spots.filter((s) => {
    const c = cur.get(s.id);
    return c && (c.status !== s.status || (c.request_id ?? null) !== (s.request_id ?? null));
  });
  if (!changed.length) return;

  const conn = await pool.getConnection();
  try {
    await conn.beginTransaction();
    for (const s of changed) {
      await conn.execute(
        `UPDATE spot_state SET status = ?, request_id = ?
          WHERE lot_version_id = ? AND spot_id = ?`,
        [s.status, s.request_id ?? null, versionId, s.id]);
    }
    await conn.commit();
    console.log(`[spot_states] ${changed.length}면 갱신`);
  } catch (e) {
    await conn.rollback();
    throw e;
  } finally {
    conn.release();
  }
}

async function handleMissionStatus(m: MissionStatus) {
  const ts = toMysqlTime(m.stamp);
  const payload = JSON.stringify(m.payload ?? {});

  // UNIQUE (request_id, seq) 라 재전송·중복 도착이 그냥 무시된다. (#9 Q6)
  const [res] = await pool.execute<any>(
    `INSERT IGNORE INTO mission_event (request_id, seq, event, bt_node, payload, ts)
     VALUES (?, ?, ?, ?, ?, ?)`,
    [m.request_id, m.seq, m.event, m.bt_node ?? null, payload, ts]);
  if (!res.affectedRows) {
    console.log(`[mission_status] #${m.request_id} seq ${m.seq} 중복 — 무시`);
    return;
  }

  const next = STATUS_OF[m.event];
  if (next) {
    const finished = ['PARKED', 'COMPLETED', 'FAILED', 'CANCELLED'].includes(next);
    await pool.execute(
      `UPDATE valet_request
          SET status = ?,
              started_at  = COALESCE(started_at, ?),
              finished_at = ${finished ? '?' : 'finished_at'}
        WHERE id = ?`,
      finished ? [next, ts, ts, m.request_id] : [next, ts, m.request_id]);
  }

  if (m.event === 'PARK_DONE') await writeMetric(m, ts);
  console.log(`[mission_status] #${m.request_id} seq ${m.seq} ${m.event}`);
}

/** 정차 오차는 로봇이 계산해 보낸다 (#9 Q7). 여기서는 소요시간만 더한다. */
async function writeMetric(m: MissionStatus, ts: string) {
  const p = (m.payload ?? {}) as Record<string, number>;
  const [[start]] = await pool.query<any>(
    `SELECT ts FROM mission_event
      WHERE request_id = ? AND event = 'REQUEST_ACCEPTED' ORDER BY seq LIMIT 1`,
    [m.request_id]);
  const duration = start
    ? (new Date(ts + 'Z').getTime() - new Date(start.ts).getTime()) / 1000
    : null;

  const err = p.err_m ?? null;
  await pool.execute(
    `INSERT INTO park_metric
       (request_id, duration_sec, position_err_m, heading_err_deg, shunt_count, succeeded)
     VALUES (?, ?, ?, ?, ?, ?)
     ON DUPLICATE KEY UPDATE
       duration_sec = VALUES(duration_sec), position_err_m = VALUES(position_err_m),
       heading_err_deg = VALUES(heading_err_deg), shunt_count = VALUES(shunt_count),
       succeeded = VALUES(succeeded)`,
    [m.request_id, duration, err, p.heading_deg ?? null, p.shunts ?? null,
     err != null && err <= 0.12 ? 1 : 0]);
  console.log(`  └ park_metric — 오차 ${err} m / ${duration?.toFixed(1)}초 / 전후진 ${p.shunts ?? '?'}회`);
}

// ── rosbridge 연결 ───────────────────────────────────────────
let ws: WebSocket | null = null;

function connect() {
  console.log(`rosbridge 연결 시도 ${URL}`);
  ws = new WebSocket(URL);

  ws.onopen = () => {
    console.log('rosbridge 연결됨');
    // #9 Q5: 자동 QoS 매칭은 퍼블리셔가 먼저 떠 있어야 해서 불안정하다.
    //        subscribe op 에 명시 지정하는 쪽을 규석이 권했다.
    ws!.send(JSON.stringify({
      op: 'subscribe', topic: '/valet/spot_states', type: 'std_msgs/String',
      qos: { history: 'keep_last', depth: 1, reliability: 'reliable', durability: 'transient_local' },
    }));
    ws!.send(JSON.stringify({
      op: 'subscribe', topic: '/valet/mission_status', type: 'std_msgs/String',
      qos: { history: 'keep_last', depth: 20, reliability: 'reliable', durability: 'volatile' },
    }));
    ws!.send(JSON.stringify({
      op: 'advertise', topic: '/valet/request', type: 'std_msgs/String',
    }));
  };

  ws.onmessage = async (ev) => {
    try {
      const frame = JSON.parse(String(ev.data));
      if (frame.op !== 'publish') return;
      const body = JSON.parse(frame.msg?.data ?? '{}');
      if (frame.topic === '/valet/spot_states') await handleSpotStates(body);
      else if (frame.topic === '/valet/mission_status') await handleMissionStatus(body);
    } catch (e: any) {
      console.error('메시지 처리 실패:', e?.message ?? e);
    }
  };

  ws.onclose = () => {
    console.log(`연결 끊김 — ${RECONNECT_MS / 1000}초 후 재시도`);
    setTimeout(connect, RECONNECT_MS);
  };
  ws.onerror = () => { /* onclose 가 이어서 불린다 */ };
}

/** API 서버가 요청을 만들 때 호출한다. */
export function publishRequest(req: object) {
  if (ws?.readyState !== WebSocket.OPEN) return false;
  ws.send(JSON.stringify({
    op: 'publish', topic: '/valet/request', msg: { data: JSON.stringify(req) },
  }));
  return true;
}

/**
 * API 서버와 같은 프로세스에서 돈다. `/valet/request` 발행에 같은 소켓을 쓰기 때문이다.
 * DB 가 아직 준비 안 됐으면 수집기만 끄고 API 는 계속 뜬다.
 */
export async function startCollector() {
  try {
    const v = await currentLotVersion();
    if (!v) {
      console.warn('수집기: 레이아웃 미적재 — db/seed.ts 실행 후 재시작하세요. 수집기는 시작하지 않습니다.');
      return;
    }
    versionId = v.id;
    console.log(`수집기: lot_version #${v.id} (${v.checksum.slice(0, 8)}) 기준`);
    connect();
  } catch (e: any) {
    console.warn(`수집기: DB 연결 실패로 시작하지 않습니다 — ${e?.code ?? e?.message ?? e}`);
  }
}

// 단독 실행도 지원한다: node --env-file=../.env src/collector.ts
if (import.meta.main) await startCollector();
