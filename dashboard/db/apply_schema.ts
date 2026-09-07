/**
 * schema.sql 적용.
 *
 *   node --env-file=.env db/apply_schema.ts
 *
 * mysql CLI 를 쓰면 비밀번호가 명령줄이나 셸 히스토리에 남는다.
 * 여기서는 .env 에서 읽어 붙이므로 그럴 일이 없다.
 */
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import mysql from 'mysql2/promise';

const sql = readFileSync(resolve(import.meta.dirname, 'schema.sql'), 'utf8');

const conn = await mysql.createConnection({
  host: process.env.DB_HOST ?? '127.0.0.1',
  port: Number(process.env.DB_PORT ?? 3306),
  user: process.env.DB_USER ?? 'valet',
  password: process.env.DB_PASSWORD ?? '',
  database: process.env.DB_NAME ?? 'valet',
  multipleStatements: true,
});

await conn.query(sql);

const [tables] = await conn.query<any[]>(
  `SELECT TABLE_NAME AS n, TABLE_TYPE AS t FROM information_schema.TABLES
    WHERE TABLE_SCHEMA = DATABASE() ORDER BY TABLE_TYPE, TABLE_NAME`);

console.log(`스키마 적용 완료 — ${process.env.DB_NAME ?? 'valet'}`);
console.log('  테이블:', tables.filter((r) => r.t === 'BASE TABLE').map((r) => r.n).join(', '));
console.log('  뷰    :', tables.filter((r) => r.t === 'VIEW').map((r) => r.n).join(', '));

await conn.end();
