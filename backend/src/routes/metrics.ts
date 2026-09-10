import { Router } from 'express';
import { pool } from '../db.ts';
import { wrap } from '../middleware/errors.ts';
import { PARK_TOLERANCE_M } from '../ros/contract.ts';

export const metricRoutes = Router();

/** 계획서 6장 '주차 소요시간 · 정차 오차 정량 지표' 산출물. */
metricRoutes.get('/metrics', wrap(async (_req, res) => {
  const [[agg]] = await pool.query<any>(
    `SELECT COUNT(*)                      AS total,
            SUM(succeeded)                AS succeeded,
            SUM(within_tolerance)         AS within_tolerance,
            ROUND(AVG(duration_sec), 1)   AS avg_duration_sec,
            ROUND(AVG(position_err_m), 3) AS avg_err_m,
            ROUND(MAX(position_err_m), 3) AS max_err_m,
            ROUND(AVG(shunt_count), 2)    AS avg_shunts
       FROM park_metric`);
  res.json({ ...agg, tolerance_m: PARK_TOLERANCE_M });
}));

metricRoutes.get('/health', wrap(async (_req, res) => {
  await pool.query('SELECT 1');
  res.json({ ok: true });
}));
