/**
 * 관제 대시보드 API 서버 (Express 5 + MySQL)
 *
 *   npm run dev:api
 *
 * 프런트(Vite)는 5173 에서 뜨고 /api 를 이 서버로 프록시한다.
 * rosbridge 수집기도 같은 프로세스에서 돈다 — /valet/request 발행에 같은 소켓을 쓴다.
 */
import express from 'express';
import { config } from './config.ts';
import { api } from './routes/index.ts';
import { layoutSource } from './layout.ts';
import { startCollector } from './ros/collector.ts';

const app = express();
app.use(express.json());
app.use('/api', api);

app.listen(config.port, async () => {
  console.log(`API       http://localhost:${config.port}`);
  console.log(`레이아웃   ${layoutSource}`);
  console.log(`DB        ${config.db.user}@${config.db.host}:${config.db.port}/${config.db.database}`);
  console.log(`rosbridge ${config.rosbridgeUrl}`);
  await startCollector();
});
