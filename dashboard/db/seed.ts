/**
 * parking_spots.json -> SQLite seed
 *
 * 멱등하다. 같은 checksum 의 레이아웃이 이미 들어있으면 아무것도 하지 않는다.
 * JSON 이 재생성되어 checksum 이 바뀌면 새 lot_version 을 만든다. (이슈 #6)
 *
 *   node db/seed.ts                    # origin/ks 에서 자동으로 가져옴
 *   node db/seed.ts <경로>             # 파일 직접 지정
 */
import { DatabaseSync } from 'node:sqlite';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';

const DB_PATH = process.env.VALET_DB ?? resolve(import.meta.dirname, '../data/valet.db');
const SCHEMA = resolve(import.meta.dirname, 'schema.sql');
const REPO_ROOT = resolve(import.meta.dirname, '../..');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

type Spot = {
  id: string;
  row: string;
  index: number;
  type: 'standard' | 'accessible' | 'ev';
  entry_side: 'N' | 'S';
  initially_occupied: boolean;
};

type Layout = {
  lot_name: string;
  scale: number;
  spots: Spot[];
  robot_spec?: { min_turning_radius?: number };
};

/** 인자 > 워킹트리 > origin/ks 순으로 찾는다. 좌표 원본을 복사해두지 않기 위함. */
function loadSpotsJson(): { raw: string; source: string } {
  const arg = process.argv[2];
  if (arg) return { raw: readFileSync(arg, 'utf8'), source: arg };

  const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
  if (existsSync(local)) return { raw: readFileSync(local, 'utf8'), source: SPOTS_IN_REPO };

  const raw = execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`], {
    cwd: REPO_ROOT,
    encoding: 'utf8',
    maxBuffer: 32 * 1024 * 1024,
  });
  return { raw, source: `origin/ks:${SPOTS_IN_REPO}` };
}

const { raw, source } = loadSpotsJson();
const checksum = createHash('sha256').update(raw).digest('hex');
const layout: Layout = JSON.parse(raw);

mkdirSync(dirname(DB_PATH), { recursive: true });
const db = new DatabaseSync(DB_PATH);
db.exec(readFileSync(SCHEMA, 'utf8'));

console.log(`레이아웃 원본 : ${source}`);
console.log(`checksum     : ${checksum.slice(0, 16)}…`);
console.log(`DB           : ${DB_PATH}`);

const existing = db
  .prepare('SELECT id, imported_at FROM lot_version WHERE checksum = ?')
  .get(checksum) as { id: number; imported_at: string } | undefined;

if (existing) {
  console.log(`\n이미 적재됨 (lot_version #${existing.id}, ${existing.imported_at}). 변경 없음.`);
  process.exit(0);
}

const prior = db.prepare('SELECT COUNT(*) AS n FROM lot_version').get() as { n: number };
if (prior.n > 0) {
  console.log(`\n주의: checksum 이 다른 레이아웃이 ${prior.n}건 있습니다.`);
  console.log('      parking_spots.json 이 재생성된 것으로 보입니다 (이슈 #6).');
  console.log('      새 lot_version 을 만들고 기존 버전은 이력으로 남깁니다.');
}

db.exec('BEGIN');
try {
  const { lastInsertRowid } = db
    .prepare(
      `INSERT INTO lot_version (lot_name, scale, checksum, spot_count, min_turning_radius)
       VALUES (?, ?, ?, ?, ?)`,
    )
    .run(
      layout.lot_name,
      layout.scale,
      checksum,
      layout.spots.length,
      layout.robot_spec?.min_turning_radius ?? null,
    );
  const versionId = Number(lastInsertRowid);

  const insSpot = db.prepare(
    `INSERT INTO parking_spot
       (lot_version_id, spot_id, row_label, idx, type, entry_side, initially_occupied)
     VALUES (?, ?, ?, ?, ?, ?, ?)`,
  );
  const insState = db.prepare(
    `INSERT INTO spot_state (lot_version_id, spot_id, status) VALUES (?, ?, ?)`,
  );

  for (const s of layout.spots) {
    const occupied = s.initially_occupied ? 1 : 0;
    insSpot.run(versionId, s.id, s.row, s.index, s.type, s.entry_side, occupied);
    insState.run(versionId, s.id, occupied ? 'OCCUPIED' : 'FREE');
  }

  db.exec('COMMIT');
  console.log(`\nlot_version #${versionId} 적재 완료 — 주차면 ${layout.spots.length}면`);
} catch (err) {
  db.exec('ROLLBACK');
  throw err;
}

const summary = db
  .prepare(
    `SELECT status, COUNT(*) AS n FROM v_current_spots GROUP BY status ORDER BY status`,
  )
  .all() as { status: string; n: number }[];
const byType = db
  .prepare(`SELECT type, COUNT(*) AS n FROM v_current_spots GROUP BY type ORDER BY type`)
  .all() as { type: string; n: number }[];

console.log('\n상태별 :', summary.map((r) => `${r.status} ${r.n}`).join(' / '));
console.log('타입별 :', byType.map((r) => `${r.type} ${r.n}`).join(' / '));
