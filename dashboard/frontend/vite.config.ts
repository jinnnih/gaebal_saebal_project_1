import { defineConfig, type Plugin } from 'vite';
import react from '@vitejs/plugin-react';
import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const REPO_ROOT = resolve(import.meta.dirname, '../..');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

/**
 * 개발 중 /parking_spots.json 을 ks 브랜치의 원본에서 바로 서빙한다.
 * 경로는 규석의 src/parking_lot_world/README.md 7장이 문서화한 것과 같다.
 *
 * 덕분에 백엔드나 MySQL 없이도 주차장 도면이 뜬다. 좌표 사본을 프런트에
 * 두지 않으므로 맵이 재생성돼도(#6) 어긋나지 않는다.
 */
function parkingLayout(): Plugin {
  return {
    name: 'valet-parking-layout',
    configureServer(server) {
      server.middlewares.use('/parking_spots.json', (_req, res) => {
        const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
        const json = existsSync(local)
          ? readFileSync(local, 'utf8')
          : execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`],
              { cwd: REPO_ROOT, encoding: 'utf8', maxBuffer: 32 << 20 });
        res.setHeader('content-type', 'application/json; charset=utf-8');
        res.end(json);
      });
    },
  };
}

export default defineConfig({
  plugins: [react(), parkingLayout()],
  server: {
    port: 5173,
    proxy: {
      // DB 를 쓰는 엔드포인트만 백엔드로. 없으면 프런트가 더미로 대체한다.
      '/api': { target: 'http://localhost:5174', changeOrigin: true },
    },
  },
});
