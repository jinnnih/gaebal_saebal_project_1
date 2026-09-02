/**
 * 관제 대시보드 API 서버 (Express 5 + MySQL)
 *
 *   npm run dev --workspace=backend
 *
 * 프런트(Vite)는 5173 에서 뜨고 /api 를 이 서버로 프록시한다.
 */
import express from 'express';
import { pool, currentLotVersion } from './db.ts';
import { layoutJson, layoutSource } from './layout.ts';

const app = express();
const PORT = Number(process.env.PORT ?? 5174);

app.use(express.json());

/** DB 가 아직 준비 안 됐을 때 500 대신 이유를 알려준다. */
const wrap = (fn: express.RequestHandler): express.RequestHandler =>
  async (req, res, next) => {
    try { await fn(req, res, next); }
    catch (e: any) {
      const known = e?.code === 'ER_ACCESS_DENIED_ERROR' ? 'DB 인증 실패 — .env 의 DB_USER/DB_PASSWORD 확인'
        : e?.code === 'ER_BAD_DB_ERROR' ? 'valet 데이터베이스 없음 — npm run db:schema 실행'
        : e?.code === 'ECONNREFUSED' ? 'MySQL 서버에 연결할 수 없음'
        : e?.code === 'ER_NO_SUCH_TABLE' ? '테이블 없음 — npm run db:schema && npm run db:seed'
        : null;
      res.status(known ? 503 : 500).json({ error: known ?? String(e?.message ?? e), code: e?.code });
    }
  };

// ── 정적 레이아웃 (DB 무관) ──────────────────────────────────
app.get('/api/layout', (_req, res) => {
  res.type('application/json').set('x-layout-source', layoutSource).send(layoutJson);
});

// ── 주차면 현재 상태 ─────────────────────────────────────────
app.get('/api/spots', wrap(async (_req, res) => {
  const version = await currentLotVersion();
  const [spots] = await pool.query('SELECT * FROM v_current_spots');
  res.json({ lot_version: version, spots });
}));

// ── 요청 큐 ──────────────────────────────────────────────────
app.get('/api/requests', wrap(async (req, res) => {
  const limit = Math.min(Number(req.query.limit ?? 50), 200);
  const [rows] = await pool.query(
    'SELECT * FROM v_request_timeline ORDER BY requested_at DESC LIMIT ?', [limit]);
  res.json(rows);
}));

app.post('/api/requests', wrap(async (req, res) => {
  const { kind = 'PARK', vehicle_tag, spot_id = null } = req.body ?? {};
  if (!vehicle_tag) return res.status(400).json({ error: 'vehicle_tag 는 필수입니다' });
  if (!['PARK', 'RETRIEVE'].includes(kind))
    return res.status(400).json({ error: 'kind 는 PARK 또는 RETRIEVE' });

  const version = await currentLotVersion();
  if (!version) return res.status(503).json({ error: '레이아웃 미적재 — npm run db:seed' });

  const conn = await pool.getConnection();
  try {
    await conn.beginTransaction();
    const [r] = await conn.execute<any>(
      `INSERT INTO valet_request (kind, vehicle_tag, lot_version_id, assigned_spot_id)
       VALUES (?, ?, ?, ?)`, [kind, vehicle_tag, version.id, spot_id]);
    const id = r.insertId;
    await conn.execute(
      `INSERT INTO mission_event (request_id, seq, event, payload)
       VALUES (?, 1, 'REQUEST_ACCEPTED', ?)`,
      [id, JSON.stringify({ vehicle_tag, kind, spot_id })]);
    await conn.commit();
    // TODO(#9): 여기서 /valet/request 토픽으로 발행. 계약은 확정됐고 발행자만 남았다.
    res.status(201).json({ id, kind, vehicle_tag, status: 'PENDING' });
  } catch (e) {
    await conn.rollback();
    throw e;
  } finally {
    conn.release();
  }
}));

app.post('/api/requests/:id/cancel', wrap(async (req, res) => {
  const id = Number(req.params.id);
  const conn = await pool.getConnection();
  try {
    await conn.beginTransaction();
    const [[cur]] = await conn.query<any>(
      'SELECT status FROM valet_request WHERE id = ? FOR UPDATE', [id]);
    if (!cur) { await conn.rollback(); return res.status(404).json({ error: '없는 요청' }); }
    if (['COMPLETED', 'PARKED', 'FAILED', 'CANCELLED'].includes(cur.status)) {
      await conn.rollback();
      return res.status(409).json({ error: `이미 종료된 요청입니다 (${cur.status})` });
    }
    await conn.execute(
      `UPDATE valet_request SET status='CANCELLED', finished_at=CURRENT_TIMESTAMP(3) WHERE id=?`, [id]);
    await conn.execute(
      `UPDATE spot_state SET status='FREE', request_id=NULL WHERE request_id=?`, [id]);
    const [[m]] = await conn.query<any>(
      'SELECT COALESCE(MAX(seq),0)+1 AS next FROM mission_event WHERE request_id=?', [id]);
    await conn.execute(
      `INSERT INTO mission_event (request_id, seq, event, payload)
       VALUES (?, ?, 'ABORTED', ?)`, [id, m.next, JSON.stringify({ by: 'dashboard' })]);
    await conn.commit();
    res.json({ id, status: 'CANCELLED' });
  } catch (e) {
    await conn.rollback();
    throw e;
  } finally {
    conn.release();
  }
}));

// ── 이벤트 타임라인 ──────────────────────────────────────────
app.get('/api/requests/:id/events', wrap(async (req, res) => {
  const [rows] = await pool.query(
    'SELECT seq, event, bt_node, payload, ts FROM mission_event WHERE request_id = ? ORDER BY seq',
    [Number(req.params.id)]);
  res.json(rows);
}));

// ── 정량 지표 (계획서 6장 산출물) ────────────────────────────
app.get('/api/metrics', wrap(async (_req, res) => {
  const [[agg]] = await pool.query<any>(
    `SELECT COUNT(*)                          AS total,
            SUM(succeeded)                    AS succeeded,
            SUM(within_tolerance)             AS within_tolerance,
            ROUND(AVG(duration_sec), 1)       AS avg_duration_sec,
            ROUND(AVG(position_err_m), 3)     AS avg_err_m,
            ROUND(MAX(position_err_m), 3)     AS max_err_m,
            ROUND(AVG(shunt_count), 2)        AS avg_shunts
       FROM park_metric`);
  res.json({ ...agg, tolerance_m: 0.12 });
}));

app.get('/api/health', wrap(async (_req, res) => {
  await pool.query('SELECT 1');
  res.json({ ok: true, layout_source: layoutSource });
}));

app.listen(PORT, () => {
  console.log(`API      http://localhost:${PORT}`);
  console.log(`레이아웃  ${layoutSource}`);
  console.log(`DB       ${process.env.DB_USER ?? 'valet'}@${process.env.DB_HOST ?? '127.0.0.1'}/${process.env.DB_NAME ?? 'valet'}`);
});
