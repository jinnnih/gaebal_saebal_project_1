/**
 * parking_spots.json -> MySQL seed (+ 더미 요청/이벤트 생성)
 *
 *   node --env-file=.env db/seed.ts             # 레이아웃만
 *   node --env-file=.env db/seed.ts --dummy 8   # 더미 요청 8건까지 추가
 *
 * 멱등하다. 같은 checksum 의 레이아웃이 이미 있으면 새로 만들지 않는다.
 * JSON 이 재생성되어 checksum 이 바뀌면 새 lot_version 을 만든다. (#6)
 */
import { createHash, randomInt } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import mysql from 'mysql2/promise';

const ROOT = resolve(import.meta.dirname, '..');
const REPO_ROOT = resolve(ROOT, '..');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

type Spot = {
  id: string; row: string; index: number;
  type: 'standard' | 'accessible' | 'ev';
  entry_side: 'N' | 'S';
  initially_occupied: boolean;
};

/** 좌표 원본은 ks 브랜치의 JSON 하나뿐이다. 복사본을 두지 않는다. (#8) */
function loadSpotsJson(): { raw: string; source: string } {
  const arg = process.argv.find((a) => a.endsWith('.json'));
  if (arg) return { raw: readFileSync(arg, 'utf8'), source: arg };

  const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
  if (existsSync(local)) return { raw: readFileSync(local, 'utf8'), source: SPOTS_IN_REPO };

  return {
    raw: execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`],
      { cwd: REPO_ROOT, encoding: 'utf8', maxBuffer: 32 << 20 }),
    source: `origin/ks:${SPOTS_IN_REPO}`,
  };
}

const { raw, source } = loadSpotsJson();
const checksum = createHash('sha256').update(raw).digest('hex');
const layout = JSON.parse(raw);
const spots: Spot[] = layout.spots;

const dummyIdx = process.argv.indexOf('--dummy');
const dummyCount = dummyIdx >= 0 ? Number(process.argv[dummyIdx + 1] ?? 8) : 0;

const db = await mysql.createConnection({
  host: process.env.DB_HOST ?? '127.0.0.1',
  port: Number(process.env.DB_PORT ?? 3306),
  user: process.env.DB_USER ?? 'valet',
  password: process.env.DB_PASSWORD ?? '',
  database: process.env.DB_NAME ?? 'valet',
  multipleStatements: false,
});

console.log(`레이아웃 원본 : ${source}`);
console.log(`checksum     : ${checksum.slice(0, 16)}…`);

const [existing] = await db.query<any[]>(
  'SELECT id, imported_at FROM lot_version WHERE checksum = ?', [checksum]);

let versionId: number;
if (existing.length) {
  versionId = existing[0].id;
  console.log(`\n레이아웃은 이미 적재됨 (lot_version #${versionId}). 건너뜀.`);
} else {
  const [prior] = await db.query<any[]>('SELECT COUNT(*) AS n FROM lot_version');
  if (prior[0].n > 0) {
    console.log('\n주의: checksum 이 다른 레이아웃이 이미 있습니다.');
    console.log('      parking_spots.json 이 재생성된 것으로 보입니다 (#6).');
  }

  await db.beginTransaction();
  try {
    const [r] = await db.execute<any>(
      `INSERT INTO lot_version (lot_name, scale, checksum, spot_count, min_turning_radius)
       VALUES (?, ?, ?, ?, ?)`,
      [layout.lot_name, layout.scale, checksum, spots.length,
       layout.robot_spec?.min_turning_radius ?? null]);
    versionId = r.insertId;

    await db.query(
      `INSERT INTO parking_spot
         (lot_version_id, spot_id, row_label, idx, type, entry_side, initially_occupied)
       VALUES ?`,
      [spots.map((s) => [versionId, s.id, s.row, s.index, s.type, s.entry_side,
                         s.initially_occupied ? 1 : 0])]);

    await db.query(
      `INSERT INTO spot_state (lot_version_id, spot_id, status) VALUES ?`,
      [spots.map((s) => [versionId, s.id, s.initially_occupied ? 'OCCUPIED' : 'FREE'])]);

    await db.commit();
    console.log(`\nlot_version #${versionId} 적재 완료 — 주차면 ${spots.length}면`);
  } catch (e) {
    await db.rollback();
    throw e;
  }
}

// ── 더미 요청 생성 ────────────────────────────────────────────
// 실제 로봇 노드가 아직 /valet/mission_status 를 발행하지 않으므로(#9),
// 계약대로 생긴 데이터를 미리 만들어 프런트와 지표 화면을 검증한다.
if (dummyCount > 0) {
  const [[{ n: already }]] = await db.query<any>('SELECT COUNT(*) AS n FROM valet_request');
  if (already >= dummyCount) {
    console.log(`\n더미 요청이 이미 ${already}건 있습니다. 건너뜀.`);
  } else {
    const [freeRows] = await db.query<any[]>(
      `SELECT spot_id FROM spot_state
        WHERE lot_version_id = ? AND status = 'FREE' ORDER BY spot_id`, [versionId]);
    const pool = freeRows.map((r) => r.spot_id);
    const tags = ['12가3456', '34나7890', '56다1234', '78라5678', '90마9012',
                  '11바3344', '22사5566', '33아7788', '44자9900', '55차1122'];

    let made = 0;
    for (let i = already; i < dummyCount && pool.length; i++) {
      const spot = pool.splice(randomInt(pool.length), 1)[0];
      const tag = tags[i % tags.length];
      // 대부분 완료, 마지막 2건은 진행 중으로 남겨 큐 UI 가 비지 않게 한다
      const inFlight = i >= dummyCount - 2;
      const started = new Date(Date.now() - (dummyCount - i) * 6 * 60_000);
      const durationSec = 38 + randomInt(45);
      const finished = new Date(started.getTime() + durationSec * 1000);
      const errM = +(0.04 + randomInt(140) / 1000).toFixed(3);   // 0.04 ~ 0.18
      const headDeg = +((randomInt(700) - 350) / 100).toFixed(2); // -3.5 ~ +3.5
      const shunts = randomInt(5);
      const ok = errM <= 0.12;

      const fmt = (d: Date) => d.toISOString().slice(0, 23).replace('T', ' ');

      const [rr] = await db.execute<any>(
        `INSERT INTO valet_request
           (kind, status, vehicle_tag, lot_version_id, assigned_spot_id,
            requested_at, started_at, finished_at, failure_reason)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)`,
        ['PARK', inFlight ? (i === dummyCount - 1 ? 'NAVIGATING' : 'PARKING')
                          : (ok ? 'PARKED' : 'FAILED'),
         tag, versionId, spot, fmt(started), fmt(started),
         inFlight ? null : fmt(finished),
         !inFlight && !ok ? '정차 오차 허용치 초과' : null]);
      const rid = rr.insertId;

      const evs: [string, string | null, object][] = [
        ['REQUEST_ACCEPTED', null, { vehicle_tag: tag }],
        ['SPOT_SELECTED', 'FindParkingSpot', { spot_id: spot }],
        ['SPOT_RESERVED', 'FindParkingSpot', { spot_id: spot }],
        ['NAV_STARTED', null, { spot_id: spot }],
      ];
      if (!inFlight) evs.push(
        ['PREPARK_REACHED', 'ParkManeuver', { spot_id: spot, err_m: 0.05 }],
        ['PARK_STARTED', 'ParkManeuver', { spot_id: spot }],
        ['PARK_DONE', 'ParkManeuver',
          { spot_id: spot, err_m: errM, heading_deg: headDeg, shunts }]);
      else if (i === dummyCount - 2) evs.push(['PREPARK_REACHED', 'ParkManeuver', { spot_id: spot }]);

      await db.query(
        `INSERT INTO mission_event (request_id, seq, event, bt_node, payload, ts) VALUES ?`,
        [evs.map((e, k) => [rid, k + 1, e[0], e[1], JSON.stringify(e[2]),
                            fmt(new Date(started.getTime() + k * 6000))])]);

      await db.execute(
        `UPDATE spot_state SET status = ?, request_id = ?
          WHERE lot_version_id = ? AND spot_id = ?`,
        [inFlight ? 'RESERVED' : (ok ? 'OCCUPIED' : 'FREE'), rid, versionId, spot]);

      if (!inFlight) {
        await db.execute(
          `INSERT INTO park_metric
             (request_id, duration_sec, position_err_m, heading_err_deg, shunt_count, succeeded)
           VALUES (?, ?, ?, ?, ?, ?)`,
          [rid, durationSec, errM, headDeg, shunts, ok ? 1 : 0]);
      }
      made++;
    }
    console.log(`더미 요청 ${made}건 생성 (완료 + 진행 중 혼합)`);
  }
}

const [summary] = await db.query<any[]>(
  `SELECT status, COUNT(*) AS n FROM v_current_spots GROUP BY status ORDER BY status`);
console.log('\n주차면 :', summary.map((r) => `${r.status} ${r.n}`).join(' / '));
const [reqs] = await db.query<any[]>(
  `SELECT status, COUNT(*) AS n FROM valet_request GROUP BY status ORDER BY status`);
if (reqs.length) console.log('요청   :', reqs.map((r) => `${r.status} ${r.n}`).join(' / '));

await db.end();
