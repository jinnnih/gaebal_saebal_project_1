import { Router } from 'express';
import { pool, currentLotVersion } from '../db.ts';
import { wrap } from '../middleware/errors.ts';

export const spotRoutes = Router();

/** 주차면 현재 상태. 좌표는 없다 — 프런트가 parking_spots.json 과 spot_id 로 합친다. */
spotRoutes.get('/spots', wrap(async (_req, res) => {
  const lot_version = await currentLotVersion();
  const [spots] = await pool.query('SELECT * FROM v_current_spots');
  res.json({ lot_version, spots });
}));
