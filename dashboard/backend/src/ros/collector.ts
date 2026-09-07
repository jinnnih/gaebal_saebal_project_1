/**
 * rosbridge → MySQL 적재.
 *
 * 연결은 client.ts, 계약은 contract.ts 가 맡는다. 여기는 "받은 걸 어떻게 넣느냐" 만 다룬다.
 */
import { pool, currentLotVersion } from '../db.ts';
import { publish, startRosClient } from './client.ts';
import {
  PARK_TOLERANCE_M, STATUS_OF, TERMINAL_STATUS, TOPIC, toRobotPose,
  type MissionStatusMsg, type RobotPose, type SpotStatesMsg, type ValetRequestMsg,
} from './contract.ts';

let versionId: number | null = null;
let warnedChecksum = '';

/**
 * 로봇 위치는 DB 에 넣지 않는다.
 *
 * 초당 수십 건이 들어오는데 남겨도 쓸 데가 없고 테이블만 부풀린다.
 * 최신값 하나만 메모리에 두고 대시보드가 API 로 가져간다.
 * 경로 리플레이가 필요해지면 그때 별도 테이블을 만든다 (#8 열린 질문).
 */
let latestPose: RobotPose | null = null;
export const getRobotPose = () => latestPose;

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

  // FindParkingSpot 이 고른 면을 요청에 붙인다. 큐 UI 가 이 값을 보여준다.
  const spotId = (m.payload as any)?.spot_id;
  if (spotId && (m.event === 'SPOT_SELECTED' || m.event === 'SPOT_RESERVED')) {
    await pool.execute(
      'UPDATE valet_request SET assigned_spot_id = ? WHERE id = ?', [spotId, m.request_id]);
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

  // 성공만 기록하면 성공률이 늘 100% 가 된다. 실패도 남겨야 지표가 의미를 가진다.
  if (m.event === 'PARK_DONE' || m.event === 'FAILED') {
    await writeMetric(m, ts, m.event === 'PARK_DONE');
  }
  console.log(`[수집기] #${m.request_id} seq ${m.seq} ${m.event}`);
}

/**
 * 정차 오차는 로봇이 계산해 보낸다 (#9 Q7). 여기서는 소요시간만 더한다.
 *
 * `parked` 는 로봇이 주차 완료를 선언했는지다. 허용오차(0.12 m) 이내인지는
 * 별개로 판정해서 `succeeded` 에 넣는다 — 완료를 선언했어도 오차가 크면 실패다.
 */
async function writeMetric(m: MissionStatusMsg, ts: string, parked: boolean) {
  const p = (m.payload ?? {}) as Record<string, number>;

  // 소요시간은 MySQL 안에서 계산한다. JS 로 빼면 DATETIME 문자열을 다시 파싱해야 하는데
  // 'YYYY-MM-DD HH:MM:SS.mmm' 은 ISO 가 아니라서 한쪽이 로컬시간으로 해석돼 어긋난다.
  const [[row]] = await pool.query<any>(
    `SELECT TIMESTAMPDIFF(MICROSECOND,
              (SELECT ts FROM mission_event
                WHERE request_id = ? AND event = 'REQUEST_ACCEPTED'
                ORDER BY seq LIMIT 1), ?) / 1000000 AS dur`,
    [m.request_id, ts]);
  const duration = row?.dur ?? null;
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
     parked && err != null && err <= PARK_TOLERANCE_M ? 1 : 0]);
  console.log(`  └ park_metric — ${parked ? '완료' : '실패'} / 오차 ${err ?? '?'} m`
            + ` / ${duration != null ? Number(duration).toFixed(1) : '?'}초 / 전후진 ${p.shunts ?? '?'}회`);
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

    startRosClient(async (topic, msg) => {
      // 로봇 위치만 표준 메시지고 나머지는 std_msgs/String 에 실린 JSON 이다.
      if (topic === TOPIC.robotPose) {
        latestPose = toRobotPose(msg) ?? latestPose;
        return;
      }
      const body = JSON.parse(msg?.data ?? '{}');
      if (topic === TOPIC.spotStates) await onSpotStates(body);
      else if (topic === TOPIC.missionStatus) await onMissionStatus(body);
    });
  } catch (e: any) {
    console.warn(`[수집기] DB 연결 실패로 시작하지 않습니다 — ${e?.code ?? e?.message ?? e}`);
  }
}
