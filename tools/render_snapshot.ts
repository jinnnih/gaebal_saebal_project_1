/**
 * 문서용 대시보드 스냅샷 생성기.
 *
 *   node --env-file=.env tools/render_snapshot.ts
 *   qlmanage -t -s 1200 -o docs docs/dashboard.svg && mv docs/dashboard.svg.png docs/dashboard.png
 *
 * 이슈·README 에 붙일 정적 이미지를 만든다. 브라우저 없이 parking_spots.json 과
 * MySQL 을 직접 읽으므로 CI 나 원격에서도 돈다.
 *
 * 캔버스를 정사각으로 잡은 이유: qlmanage 가 썸네일을 정사각으로 패딩하기 때문에
 * 미리 맞춰두지 않으면 아래쪽에 흰 여백이 붙는다.
 */
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import mysql from 'mysql2/promise';

const ROOT = resolve(import.meta.dirname, '..');
const REPO_ROOT = ROOT;   // 저장소 루트 = project_1
/** 통합 후 로봇 패키지가 놓일 자리. 아직 병합 전이라 없으면 ks 브랜치에서 읽는다. */
const SPOTS_LOCAL = 'ros/src/parking_lot_world/config/parking_spots.json';
const SPOTS_ON_KS = 'src/parking_lot_world/config/parking_spots.json';
const OUT = resolve(ROOT, 'docs/dashboard.svg');

const C = {
  bg: '#0d1117', panel: '#161b22', line: '#30363d', lot: '#0b0f14',
  fg: '#e6edf3', muted: '#8b949e',
  FREE: '#2ea043', OCCUPIED: '#57606a', RESERVED: '#d29922', BLOCKED: '#f85149',
  entry: '#58a6ff', exit: '#a371f7', pillar: '#8b949e', robot: '#39d0d8',
};
const KO = { FREE: '공차', OCCUPIED: '점유', RESERVED: '예약', BLOCKED: '차단' } as const;
const FONT = "'Helvetica Neue', Helvetica, 'Apple SD Gothic Neo', sans-serif";

const layout = JSON.parse(
  existsSync(resolve(REPO_ROOT, SPOTS_LOCAL))
    ? readFileSync(resolve(REPO_ROOT, SPOTS_LOCAL), 'utf8')
    : execFileSync('git', ['show', `origin/ks:${SPOTS_ON_KS}`],
        { cwd: REPO_ROOT, encoding: 'utf8', maxBuffer: 32 << 20 }));

const db = await mysql.createConnection({
  host: process.env.DB_HOST ?? '127.0.0.1',
  port: Number(process.env.DB_PORT ?? 3306),
  user: process.env.DB_USER ?? 'valet',
  password: process.env.DB_PASSWORD ?? '',
  database: process.env.DB_NAME ?? 'valet',
});
await db.query("SET time_zone = '+00:00'");

const [states] = await db.query<any[]>('SELECT spot_id, status FROM v_current_spots');
const [[version]] = await db.query<any>(
  'SELECT id, checksum FROM lot_version ORDER BY id DESC LIMIT 1');
const [[m]] = await db.query<any>(
  `SELECT COUNT(*) total, SUM(succeeded) ok, SUM(within_tolerance) within,
          ROUND(AVG(duration_sec),1) dur, ROUND(AVG(position_err_m),3) err,
          ROUND(AVG(shunt_count),2) shunts FROM park_metric`);
const [reqs] = await db.query<any[]>(
  `SELECT status, COUNT(*) n FROM valet_request GROUP BY status`);
await db.end();

const statusOf = new Map(states.map((s) => [s.spot_id, s.status as keyof typeof KO]));
const { bounds: b, spots, pillars, hatched_zones, entry_pose, exit_pose } = layout;

const counts: Record<string, number> = { FREE: 0, OCCUPIED: 0, RESERVED: 0, BLOCKED: 0 };
for (const s of spots) counts[statusOf.get(s.id) ?? 'FREE']++;
const active = reqs.filter((r) => !['PARKED', 'COMPLETED', 'FAILED', 'CANCELLED'].includes(r.status))
                   .reduce((a, r) => a + r.n, 0);

const W = 1200, H = 1200, PAD = 1.2;
const vb = `${b.x_min - PAD} ${-b.y_max - PAD} `
         + `${b.x_max - b.x_min + PAD * 2} ${b.y_max - b.y_min + PAD * 2}`;

const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
const out: string[] = [];
const p = (s: string) => out.push(s);

p(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" font-family="${FONT}">`);
p(`<rect width="${W}" height="${H}" fill="${C.bg}"/>`);
p(`<text x="28" y="46" fill="${C.fg}" font-size="24" font-weight="600">발렛파킹 관제 대시보드</text>`);
p(`<text x="28" y="74" fill="${C.muted}" font-size="15">${esc(layout.lot_name)} · 주차면 ${spots.length}면 · 공차 ${counts.FREE}면 · 점유율 ${(100 * (1 - counts.FREE / spots.length)).toFixed(0)}%</text>`);
p(`<line x1="0" y1="96" x2="${W}" y2="96" stroke="${C.line}"/>`);

p(`<svg x="20" y="112" width="840" height="1070" viewBox="${vb}" preserveAspectRatio="xMidYMid meet">`);
p(`<g transform="scale(1,-1)">`);
p(`<rect x="${b.x_min}" y="${b.y_min}" width="${b.x_max - b.x_min}" height="${b.y_max - b.y_min}" fill="${C.lot}" stroke="${C.line}" stroke-width=".12"/>`);
for (const z of hatched_zones ?? []) {
  const [x0, y0, x1, y1] = z.rect;
  p(`<rect x="${x0}" y="${y0}" width="${x1 - x0}" height="${y1 - y0}" fill="rgba(139,148,158,.10)" stroke="#484f58" stroke-width=".07" stroke-dasharray=".5 .35"/>`);
}
for (const s of spots) {
  const [x0, y0, x1, y1] = s.rect;
  const st = statusOf.get(s.id) ?? 'FREE';
  p(`<rect x="${x0}" y="${y0}" width="${x1 - x0}" height="${y1 - y0}" rx=".18" fill="${C[st]}" fill-opacity="${st === 'OCCUPIED' ? .55 : .78}" stroke="${C.bg}" stroke-width=".09"/>`);
}
for (const pl of pillars ?? []) {
  p(`<rect x="${pl.x - pl.size / 2}" y="${pl.y - pl.size / 2}" width="${pl.size}" height="${pl.size}" fill="${C.pillar}" stroke="#c9d1d9" stroke-width=".04"/>`);
}
for (const [pose, col] of [[entry_pose, C.entry], [exit_pose, C.exit]] as [number[], string][]) {
  p(`<circle cx="${pose[0]}" cy="${pose[1]}" r=".75" fill="${col}" fill-opacity=".25" stroke="${col}" stroke-width=".12"/>`);
}
// 로봇은 입구에 세워 둔다 — 정적 이미지라 실제 위치를 쓰면 매번 달라진다
const rs = layout.robot_spec;
p(`<g transform="translate(${entry_pose[0]} ${entry_pose[1]}) rotate(${(entry_pose[2] * 180) / Math.PI})">`);
p(`  <rect x="${-rs.length / 2}" y="${-rs.width / 2}" width="${rs.length}" height="${rs.width}" rx=".25" fill="${C.robot}" fill-opacity=".85" stroke="${C.bg}" stroke-width=".1"/>`);
p(`  <path d="M ${rs.length / 2 - 0.1} 0 L ${rs.length / 2 - 0.9} ${rs.width / 2 - 0.25} L ${rs.length / 2 - 0.9} ${-(rs.width / 2 - 0.25)} Z" fill="${C.bg}"/>`);
p(`</g>`);
p(`</g>`);
for (const s of spots) {
  p(`<text x="${s.center[0]}" y="${-s.center[1] + .35}" text-anchor="middle" font-size=".95" font-weight="700" fill="${C.bg}">${s.id}</text>`);
  const mark = s.type === 'accessible' ? '장애인' : s.type === 'ev' ? 'EV' : '';
  if (mark) p(`<text x="${s.center[0]}" y="${-s.center[1] + 1.7}" text-anchor="middle" font-size=".62" font-weight="600" fill="${C.bg}">${mark}</text>`);
}
for (const [pose, col, label] of [[entry_pose, C.entry, '입구'], [exit_pose, C.exit, '출구']] as [number[], string, string][]) {
  p(`<text x="${pose[0]}" y="${-pose[1] + 3.0}" text-anchor="middle" font-size=".95" font-weight="700" fill="${col}">${label}</text>`);
}
p(`</svg>`);

const PX = 884, PW = 292;
let py = 112;
const panel = (title: string, rows: [string, string, string?][]) => {
  const h = 44 + rows.length * 26;
  p(`<rect x="${PX}" y="${py}" width="${PW}" height="${h}" rx="8" fill="${C.panel}" stroke="${C.line}"/>`);
  p(`<text x="${PX + 16}" y="${py + 26}" fill="${C.muted}" font-size="11.5" font-weight="600" letter-spacing="1">${title}</text>`);
  rows.forEach(([k, v, sw], i) => {
    const y = py + 50 + i * 26;
    if (sw) p(`<rect x="${PX + 16}" y="${y - 9}" width="10" height="10" rx="2" fill="${sw}"/>`);
    p(`<text x="${PX + (sw ? 34 : 16)}" y="${y}" fill="${C.fg}" font-size="13.5">${esc(k)}</text>`);
    p(`<text x="${PX + PW - 16}" y="${y}" text-anchor="end" fill="${C.fg}" font-size="13.5" font-weight="600">${esc(v)}</text>`);
  });
  py += h + 14;
};

panel('점유 현황', (Object.keys(KO) as (keyof typeof KO)[])
  .map((k) => [KO[k], String(counts[k]), C[k]] as [string, string, string]));

const pct = (n: number, d: number) => (d ? `${((100 * n) / d).toFixed(0)}%` : '—');
panel('정량 지표', m.total ? [
  ['완료 요청', `${m.total}건`],
  ['성공률', pct(Number(m.ok), m.total)],
  ['허용오차 0.12m 이내', pct(Number(m.within), m.total)],
  ['평균 소요', `${m.dur}초`],
  ['평균 정차오차', `${m.err} m`],
  ['평균 전후진', `${m.shunts}회`],
] : [['완료된 요청', '없음']]);

panel('레이아웃', [
  ['크기', `${(b.x_max - b.x_min).toFixed(0)} × ${(b.y_max - b.y_min).toFixed(1)} m`],
  ['주차면', `${spots.length}면`],
  ['기둥', `${(pillars ?? []).length}개`],
  ['최소회전반경', `${rs.min_turning_radius.toFixed(2)} m`],
  ['lot_version', `#${version.id} ${version.checksum.slice(0, 8)}`],
]);

p(`<rect x="${PX}" y="${py}" width="${PW}" height="100" rx="8" fill="${C.panel}" stroke="${C.line}"/>`);
p(`<text x="${PX + 16}" y="${py + 26}" fill="${C.muted}" font-size="11.5" font-weight="600" letter-spacing="1">실시간 연동</text>`);
[`진행 중 요청 ${active}건`,
 'rosbridge 로 점유·이벤트·위치를',
 '받아 MySQL 에 적재한다.'].forEach((t, i) =>
  p(`<text x="${PX + 16}" y="${py + 50 + i * 17}" fill="${C.muted}" font-size="12">${t}</text>`));

p(`</svg>`);

mkdirSync(resolve(ROOT, 'docs'), { recursive: true });
writeFileSync(OUT, out.join('\n'));
console.log(`생성: ${OUT}`);
console.log(`주차면: ${Object.entries(counts).filter(([, n]) => n).map(([k, n]) => `${KO[k as keyof typeof KO]} ${n}`).join(' / ')}`);
console.log(`지표  : 완료 ${m.total}건 · 성공률 ${pct(Number(m.ok), m.total)} · 평균오차 ${m.err} m`);
