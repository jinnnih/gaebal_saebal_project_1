import mysql from 'mysql2/promise';
import { config } from './config.ts';

export const pool = mysql.createPool({
  ...config.db,
  waitForConnections: true,
  connectionLimit: 10,
  timezone: 'Z',
});

/** 최신 레이아웃 버전. 모든 조회가 이 값을 기준으로 한다. */
export async function currentLotVersion() {
  const [rows] = await pool.query<any[]>(
    `SELECT id, lot_name, checksum, spot_count, min_turning_radius, imported_at
       FROM lot_version ORDER BY id DESC LIMIT 1`);
  return rows[0] ?? null;
}
