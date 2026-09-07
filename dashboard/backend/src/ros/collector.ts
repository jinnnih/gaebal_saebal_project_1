/**
 * rosbridge → MySQL 적재.
 *
 * 연결은 client.ts, 계약은 contract.ts 가 맡는다. 여기는 "받은 걸 어떻게 넣느냐" 만 다룬다.
 */
import { pool, currentLotVersion } from '../db.ts';
import { publish, startRosClient } from './client.ts';
import {
  PARK_TOLERANCE_M, STATUS_OF, TERMINAL_STATUS, TOPIC,
  type MissionStatusMsg, type SpotStatesMsg, type ValetRequestMsg,
} from './contract.ts';

let versionId: number | null = null;
let warnedChecksum = '';

const toMysqlTime = (iso?: string) =>
  (iso ? new Date(iso) : new Date()).toISOString().slice(0, 23).replace('T', ' ');

/** 로봇이 보는 레이아웃과 DB 가 다르면 알린다. 맵 재생성 감지용 (#6). */
async function checkChecksum(sent?: string) {
  if (!sent || sent === warnedChecksum) return;
  const [[v]] = await pool.query<any>('SELECT checksum FROM lot_version WHERE id = ?', [versionId]);
  if (v && !v.checksum.startsWith(sent)) {
    warnedChecksum = sent;
    console.warn(`[수집기] 로봇 lot_checksum(${sent}) ≠ DB(${v.checksum.slice(0, 8)})`);
    console.warn('         parking_spots.json 이 재생성됐습니다. db/seed.ts 를 다시 실행하세요.');
  }
}

async function onSpotStates(snap: SpotStatesMsg) {
  if (!versionId) return;
  await checkChecksum(snap.lot_checksum);

  // 전체 스냅샷으로 오므로(#9 Q3) 바뀐 것만 골라 쓴다.
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
    console.log(`[수집기] spot_states — ${changed.length}면 갱신`);
  } catch (e) {
    await conn.rollback();
    throw e;
  } finally {
    conn.release();
  }
}

async function onMissionStatus(m: MissionStatusMsg) {
  const ts = toMysqlTime(m.stamp);

  // UNIQUE (request_id, seq) 라 재전송·중복 도착이 그냥 무시된다 (#9 Q6).
  const [res] = await pool.execute<any>(
    `INSERT IGNORE INTO mission_event (request_id, seq, event, bt_node, payload, ts)
     VALUES (?, ?, ?, ?, ?, ?)`,
    [m.request_id, m.seq, m.event, m.bt_node ?? null, JSON.stringify(m.payload ?? {}), ts]);
  if (!res.affectedRows) {
    console.log(`[수집기] #${m.request_id} seq ${m.seq} 중복 — 무시`);
    return;
  }

  const next = STATUS_OF[m.event];
  if (next) {
    const done = TERMINAL_STATUS.includes(next);
    await pool.execute(
      `UPDATE valet_request
          SET status = ?, started_at = COALESCE(started_at, ?),
              finished_at = ${done ? '?' : 'finished_at'}
        WHERE id = ?`,
      done ? [next, ts, ts, m.request_id] : [next, ts, m.request_id]);
  }

  if (m.event === 'PARK_DONE') await writeMetric(m, ts);
  console.log(`[수집기] #${m.request_id} seq ${m.seq} ${m.event}`);
}

/** 정차 오차는 로봇이 계산해 보낸다 (#9 Q7). 여기서는 소요시간만 더한다. */
async function writeMetric(m: MissionStatusMsg, ts: string) {
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
     err != null && err <= PARK_TOLERANCE_M ? 1 : 0]);
  console.log(`  └ park_metric — 오차 ${err} m / ${duration?.toFixed(1)}초 / 전후진 ${p.shunts ?? '?'}회`);
}

/** API 서버가 요청을 만들 때 호출한다. */
export const publishRequest = (req: ValetRequestMsg) => publish(TOPIC.request, req);

/**
 * API 서버와 같은 프로세스에서 돈다. `/valet/request` 발행에 같은 소켓을 쓰기 때문이다.
 * DB 가 준비 안 됐으면 수집기만 끄고 API 는 계속 뜬다.
 */
export async function startCollector() {
  try {
    const v = await currentLotVersion();
    if (!v) {
      console.warn('[수집기] 레이아웃 미적재 — db/seed.ts 실행 후 재시작하세요.');
      return;
    }
    versionId = v.id;
    console.log(`[수집기] lot_version #${v.id} (${v.checksum.slice(0, 8)}) 기준`);

    startRosClient(async (topic, body) => {
      if (topic === TOPIC.spotStates) await onSpotStates(body);
      else if (topic === TOPIC.missionStatus) await onMissionStatus(body);
    });
  } catch (e: any) {
    console.warn(`[수집기] DB 연결 실패로 시작하지 않습니다 — ${e?.code ?? e?.message ?? e}`);
  }
}
