import { Router } from 'express';
import { pool, currentLotVersion } from '../db.ts';
import { wrap } from '../middleware/errors.ts';
import { publishRequest } from '../ros/collector.ts';

export const requestRoutes = Router();

/** 요청 큐. 이벤트 수와 지표를 뷰가 미리 묶어 준다. */
requestRoutes.get('/requests', wrap(async (req, res) => {
  const limit = Math.min(Number(req.query.limit ?? 50), 200);
  const [rows] = await pool.query(
    'SELECT * FROM v_request_timeline ORDER BY requested_at DESC LIMIT ?', [limit]);
  res.json(rows);
}));

requestRoutes.post('/requests', wrap(async (req, res) => {
  const {
    kind = 'PARK', vehicle_tag, spot_id = null,
    source = 'CONSOLE', has_occupant = false,
  } = req.body ?? {};
  if (!vehicle_tag) return res.status(400).json({ error: 'vehicle_tag 는 필수입니다' });
  if (!['PARK', 'RETRIEVE'].includes(kind))
    return res.status(400).json({ error: 'kind 는 PARK 또는 RETRIEVE 여야 합니다' });
  if (!['IN_CAR', 'APP', 'CONSOLE'].includes(source))
    return res.status(400).json({ error: 'source 는 IN_CAR / APP / CONSOLE 중 하나입니다' });

  const version = await currentLotVersion();
  if (!version) return res.status(503).json({ error: '레이아웃 미적재 — db/seed.ts 실행' });

  // 출차는 어느 차를 뺄지가 정해져야 한다. 면을 안 주면 그 차량의 주차 기록에서 찾는다.
  let target: string | null = spot_id;
  if (kind === 'RETRIEVE') {
    if (!target) {
      const [[prev]] = await pool.query<any>(
        `SELECT assigned_spot_id FROM valet_request
          WHERE vehicle_tag = ? AND status = 'PARKED' AND assigned_spot_id IS NOT NULL
          ORDER BY requested_at DESC LIMIT 1`, [vehicle_tag]);
      if (!prev) {
        return res.status(409).json({ error: `${vehicle_tag} 의 주차 기록을 찾을 수 없습니다` });
      }
      target = prev.assigned_spot_id;
    }
    const [[st]] = await pool.query<any>(
      `SELECT status FROM spot_state WHERE lot_version_id = ? AND spot_id = ?`,
      [version.id, target]);
    if (!st) return res.status(404).json({ error: `${target} 주차면이 없습니다` });
    if (st.status !== 'OCCUPIED') {
      return res.status(409).json({ error: `${target} 은 점유 상태가 아닙니다 (${st.status})` });
    }
  }

  const conn = await pool.getConnection();
  try {
    await conn.beginTransaction();
    const [r] = await conn.execute<any>(
      `INSERT INTO valet_request
         (kind, vehicle_tag, lot_version_id, assigned_spot_id, source, has_occupant)
       VALUES (?, ?, ?, ?, ?, ?)`,
      [kind, vehicle_tag, version.id, target, source, has_occupant ? 1 : 0]);
    const id = r.insertId;
    await conn.execute(
      `INSERT INTO mission_event (request_id, seq, event, payload)
       VALUES (?, 1, 'REQUEST_ACCEPTED', ?)`,
      [id, JSON.stringify({ vehicle_tag, kind, spot_id: target, source, has_occupant })]);
    await conn.commit();

    // rosbridge 가 없으면 DB 에만 남기고 published:false 로 알린다.
    // has_occupant 는 로봇이 주행 프로파일을 낮출지 판단하라고 같이 보낸다.
    const published = publishRequest({
      request_id: id, kind, vehicle_tag, spot_id: target, has_occupant: !!has_occupant });
    res.status(201).json({
      id, kind, vehicle_tag, spot_id: target, source,
      has_occupant: !!has_occupant, status: 'PENDING', published });
  } catch (e) {
    await conn.rollback();
    throw e;
  } finally {
    conn.release();
  }
}));

/** 취소. ABORTED 이벤트를 남겨 FAILED(로봇이 못 함)와 구분한다 (#9 Q6). */
requestRoutes.post('/requests/:id/cancel', wrap(async (req, res) => {
  const id = Number(req.params.id);
  const conn = await pool.getConnection();
  try {
    await conn.beginTransaction();
    const [[cur]] = await conn.query<any>(
      'SELECT status FROM valet_request WHERE id = ? FOR UPDATE', [id]);
    if (!cur) { await conn.rollback(); return res.status(404).json({ error: '없는 요청입니다' }); }
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

requestRoutes.get('/requests/:id/events', wrap(async (req, res) => {
  const [rows] = await pool.query(
    `SELECT seq, event, bt_node, payload, ts FROM mission_event
      WHERE request_id = ? ORDER BY seq`, [Number(req.params.id)]);
  res.json(rows);
}));
