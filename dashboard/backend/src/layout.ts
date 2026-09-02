import { execFileSync } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const REPO_ROOT = resolve(import.meta.dirname, '../../..');
const SPOTS_IN_REPO = 'src/parking_lot_world/config/parking_spots.json';

/**
 * 주차장 정적 레이아웃.
 *
 * 좌표의 원본은 ks 브랜치의 parking_spots.json 하나뿐이고 DB 에 복사하지 않는다. (#8)
 * 워킹트리에 있으면 그걸 쓰고, 없으면 origin/ks 에서 직접 읽는다.
 */
function load(): { json: string; source: string } {
  const local = resolve(REPO_ROOT, SPOTS_IN_REPO);
  if (existsSync(local)) return { json: readFileSync(local, 'utf8'), source: SPOTS_IN_REPO };
  return {
    json: execFileSync('git', ['show', `origin/ks:${SPOTS_IN_REPO}`],
      { cwd: REPO_ROOT, encoding: 'utf8', maxBuffer: 32 << 20 }),
    source: `origin/ks:${SPOTS_IN_REPO}`,
  };
}

const loaded = load();
export const layoutJson = loaded.json;
export const layoutSource = loaded.source;
export const layout = JSON.parse(loaded.json);
