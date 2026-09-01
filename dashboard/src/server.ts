/**
 * 관제 대시보드 개발 서버 — 의존성 0개 (node:http + node:sqlite)
 *
 *   node src/server.ts          # http://localhost:5173
 *
 * 엔드포인트
 *   GET /                  대시보드 페이지
 *   GET /api/layout        정적 레이아웃 (parking_spots.json 원본 그대로)
 *   GET /api/spots/state   현재 점유 상태 (SQLite)
 */
import { createServer } from 'node:http';
import { DatabaseSync } from 'node:sqlite';
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const PORT = Number(process.env.PORT ?? 5173);
const ROOT = resolve(import.meta.dirname, '..');
const REPO_ROOT = resolve(ROOT, '..');
const DB_PATH = process.env.VALET_DB ?? resolve(ROOT, 'data/valet.db');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

/** 좌표 원본은 ks 브랜치의 JSON 하나뿐이다. 복사본을 두지 않는다. (#8) */
function loadLayout(): string {
  const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
  if (existsSync(local)) return readFileSync(local, 'utf8');
  return execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`], {
    cwd: REPO_ROOT,
    encoding: 'utf8',
    maxBuffer: 32 * 1024 * 1024,
  });
}

const layoutJson = loadLayout();

const send = (res: any, code: number, type: string, body: string) => {
  res.writeHead(code, { 'content-type': type, 'cache-control': 'no-store' });
  res.end(body);
};

const server = createServer((req, res) => {
  const path = (req.url ?? '/').split('?')[0];

  if (path === '/api/layout') {
    return send(res, 200, 'application/json; charset=utf-8', layoutJson);
  }

  if (path === '/api/spots/state') {
    if (!existsSync(DB_PATH)) {
      return send(res, 503, 'application/json', JSON.stringify({
        error: 'DB 없음. npm run db:seed 를 먼저 실행하세요.',
      }));
    }
    const db = new DatabaseSync(DB_PATH);
    const rows = db.prepare(
      `SELECT spot_id, status, request_id, updated_at FROM v_current_spots`,
    ).all();
    const version = db.prepare(
      `SELECT id, checksum, imported_at FROM lot_version ORDER BY id DESC LIMIT 1`,
    ).get();
    db.close();
    return send(res, 200, 'application/json; charset=utf-8',
      JSON.stringify({ lot_version: version, spots: rows }));
  }

  if (path === '/' || path === '/index.html') {
    const html = resolve(ROOT, 'public/index.html');
    return send(res, 200, 'text/html; charset=utf-8', readFileSync(html, 'utf8'));
  }

  send(res, 404, 'text/plain; charset=utf-8', 'not found');
});

server.listen(PORT, () => {
  console.log(`관제 대시보드  http://localhost:${PORT}`);
  console.log(`레이아웃 원본  ${existsSync(resolve(REPO_ROOT, SPOTS_IN_REPO)) ? SPOTS_IN_REPO : 'origin/ks:' + SPOTS_IN_REPO}`);
  console.log(`DB             ${existsSync(DB_PATH) ? DB_PATH : '(없음 — npm run db:seed)'}`);
});
