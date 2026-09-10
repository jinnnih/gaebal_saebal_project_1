import type { RequestHandler } from 'express';

/**
 * DB 가 아직 준비 안 됐을 때 500 대신 무엇을 하면 되는지 알려준다.
 * 처음 세팅할 때 원인을 찾느라 헤매지 않도록.
 */
const HINT: Record<string, string> = {
  ER_ACCESS_DENIED_ERROR: 'DB 인증 실패 — .env 의 DB_USER / DB_PASSWORD 를 확인하세요',
  ER_BAD_DB_ERROR: 'valet 데이터베이스가 없습니다 — db/schema.sql 을 적용하세요',
  ER_NO_SUCH_TABLE: '테이블이 없습니다 — db/schema.sql 적용 후 db/seed.ts 를 실행하세요',
  ECONNREFUSED: 'MySQL 서버에 연결할 수 없습니다 — brew services start mysql',
  PROTOCOL_CONNECTION_LOST: 'MySQL 연결이 끊겼습니다',
};

export const wrap = (fn: RequestHandler): RequestHandler =>
  async (req, res, next) => {
    try {
      await fn(req, res, next);
    } catch (e: any) {
      const hint = HINT[e?.code];
      res.status(hint ? 503 : 500).json({
        error: hint ?? String(e?.message ?? e),
        code: e?.code,
      });
    }
  };
