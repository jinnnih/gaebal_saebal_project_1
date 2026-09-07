import { useEffect, useState } from 'react';
import type { PoseResponse } from '../types/index.ts';
import { fetchPose } from '../api/client.ts';

/**
 * 로봇 위치만 따로, 더 자주 가져온다.
 *
 * 점유 현황이나 요청 큐는 3초마다면 충분하지만 위치는 그러면 뚝뚝 끊긴다.
 * DB 를 타지 않고 백엔드 메모리에서 바로 오므로 이 주기가 부담되지 않는다.
 */
const POLL_MS = 400;

export function useRobotPose(enabled: boolean): PoseResponse | null {
  const [pose, setPose] = useState<PoseResponse | null>(null);

  useEffect(() => {
    if (!enabled) { setPose(null); return; }
    let stopped = false;
    let timer: number;

    const tick = async () => {
      if (stopped) return;
      setPose(await fetchPose());
      timer = window.setTimeout(tick, POLL_MS);
    };
    tick();

    return () => { stopped = true; clearTimeout(timer); };
  }, [enabled]);

  return pose;
}
