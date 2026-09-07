import mysql from 'mysql2/promise';
import { config } from './config.ts';

export const pool = mysql.createPool({
  ...config.db,
  waitForConnections: true,
  connectionLimit: 10,
  timezone: 'Z',
});

/**
 * 모든 커넥션의 세션 시간대를 UTC 로 고정한다.
 *
 * 이걸 안 하면 `CURRENT_TIMESTAMP(3)` 기본값은 서버 로컬시간(KST)으로 들어가고
 * 수집기가 ROS stamp 를 UTC 로 변환해 넣은 값과 9시간 어긋난다.
 * 실제로 소요시간이 -32392 초로 나왔었다.
 */
pool.on('connection', (conn) => {
  conn.query("SET time_zone = '+00:00'");
});

/** 최신 레이아웃 버전. 모든 조회가 이 값을 기준으로 한다. */
export async function currentLotVersion() {
  const [rows] = await pool.query<any[]>(
    `SELECT id, lot_name, checksum, spot_count, min_turning_radius, imported_at
       FROM lot_version ORDER BY id DESC LIMIT 1`);
  return rows[0] ?? null;
}
