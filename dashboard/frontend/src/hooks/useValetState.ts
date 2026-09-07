import { useCallback, useEffect, useState } from 'react';
import type { Layout, LotVersion, Metrics, RequestRow, SpotState } from '../types/index.ts';
import { fetchLayout, fetchState, type Source } from '../api/client.ts';

/**
 * 백엔드가 붙어 있으면 자주, 더미 모드면 드물게 확인한다.
 * 더미일 때 3초마다 찌르면 프록시 500 이 콘솔에 쌓이기만 하고 얻는 게 없다.
 */
const POLL_LIVE_MS = 3000;
const POLL_DUMMY_MS = 15000;

export interface ValetState {
  layout: Layout | null;
  states: SpotState[];
  requests: RequestRow[];
  metrics: Metrics | null;
  lotVersion: LotVersion | null;
  source: Source;
  error: string | null;
  refresh: () => Promise<void>;
}

/**
 * 대시보드 전체 상태를 한 곳에서 들고 있는다.
 *
 * 실시간 토픽이 붙기 전까지는 폴링이다. 수집기가 rosbridge 로 받은 것을 DB 에
 * 넣고 있으므로, 나중에 이 훅만 WebSocket 구독으로 바꾸면 된다 (#9).
 */
export function useValetState(): ValetState {
  const [layout, setLayout] = useState<Layout | null>(null);
  const [states, setStates] = useState<SpotState[]>([]);
  const [requests, setRequests] = useState<RequestRow[]>([]);
  const [metrics, setMetrics] = useState<Metrics | null>(null);
  const [lotVersion, setLotVersion] = useState<LotVersion | null>(null);
  const [source, setSource] = useState<Source>('dummy');
  const [error, setError] = useState<string | null>(null);

  const apply = useCallback(async (l: Layout): Promise<Source> => {
    const s = await fetchState(l);
    setStates(s.spots);
    setRequests(s.requests);
    setMetrics(s.metrics);
    setLotVersion(s.lotVersion);
    setSource(s.source);
    return s.source;
  }, []);

  const refresh = useCallback(async () => {
    if (layout) await apply(layout);
  }, [layout, apply]);

  useEffect(() => {
    let timer: number;
    let stopped = false;

    const loop = (l: Layout, src: Source) => {
      if (stopped) return;
      timer = window.setTimeout(async () => {
        const next = await apply(l).catch(() => src);
        loop(l, next);
      }, src === 'live' ? POLL_LIVE_MS : POLL_DUMMY_MS);
    };

    (async () => {
      try {
        const l = await fetchLayout();
        setLayout(l);
        loop(l, await apply(l));
      } catch (e: any) {
        setError(e.message ?? String(e));
      }
    })();

    return () => { stopped = true; clearTimeout(timer); };
  }, [apply]);

  return { layout, states, requests, metrics, lotVersion, source, error, refresh };
}
