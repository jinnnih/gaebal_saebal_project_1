import { useCallback, useEffect, useState } from 'react';
import type { Layout, LotVersion, Metrics, RequestRow, SpotState } from './types.ts';
import { cancelRequest, createRequest, fetchLayout, fetchState, type Source } from './api.ts';
import { ParkingMap } from './components/ParkingMap.tsx';
import { SidePanel } from './components/SidePanel.tsx';
import { RequestQueue } from './components/RequestQueue.tsx';

// 백엔드가 붙어 있으면 자주, 더미 모드면 드물게 확인한다.
// 더미일 때 3초마다 찌르면 프록시 500 이 콘솔에 쌓이기만 하고 얻는 게 없다.
const POLL_LIVE_MS = 3000;
const POLL_DUMMY_MS = 15000;

export default function App() {
  const [layout, setLayout] = useState<Layout | null>(null);
  const [states, setStates] = useState<SpotState[]>([]);
  const [requests, setRequests] = useState<RequestRow[]>([]);
  const [metrics, setMetrics] = useState<Metrics | null>(null);
  const [lotVersion, setLotVersion] = useState<LotVersion | null>(null);
  const [source, setSource] = useState<Source>('dummy');
  const [selected, setSelected] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const refresh = useCallback(async (l: Layout): Promise<Source> => {
    const s = await fetchState(l);
    setStates(s.spots);
    setRequests(s.requests);
    setMetrics(s.metrics);
    setLotVersion(s.lotVersion);
    setSource(s.source);
    return s.source;
  }, []);

  useEffect(() => {
    let timer: number;
    let stopped = false;

    // 실시간 토픽이 붙기 전까지는 폴링. rosbridge 연동 후 WebSocket 으로 교체한다. (#9)
    const loop = (l: Layout, src: Source) => {
      if (stopped) return;
      timer = window.setTimeout(async () => {
        const next = await refresh(l).catch(() => src);
        loop(l, next);
      }, src === 'live' ? POLL_LIVE_MS : POLL_DUMMY_MS);
    };

    (async () => {
      try {
        const l = await fetchLayout();
        setLayout(l);
        loop(l, await refresh(l));
      } catch (e: any) {
        setError(e.message ?? String(e));
      }
    })();

    return () => { stopped = true; clearTimeout(timer); };
  }, [refresh]);

  const onCreate = async (tag: string, kind: 'PARK' | 'RETRIEVE') => {
    await createRequest(tag, kind);
    if (layout) await refresh(layout);
  };
  const onCancel = async (id: number) => {
    await cancelRequest(id);
    if (layout) await refresh(layout);
  };

  if (error) return <div className="fatal">레이아웃을 불러오지 못했습니다: {error}</div>;
  if (!layout || !metrics) return <div className="loading">불러오는 중…</div>;

  const free = states.filter((s) => s.status === 'FREE').length;
  const total = layout.spots.length;

  return (
    <>
      <header>
        <h1>발렛파킹 관제 대시보드</h1>
        <span className="meta">
          {layout.lot_name} · 주차면 {total}면 · 공차 {free}면 ·
          점유율 {(100 * (1 - free / total)).toFixed(0)}%
        </span>
        <span className={`src ${source}`}>
          {source === 'live' ? 'MySQL 연결됨' : '더미 데이터'}
        </span>
      </header>

      <main>
        <div className="card">
          <ParkingMap layout={layout} states={states}
                      selected={selected} onSelect={setSelected} />
        </div>

        <div className="col">
          <div className="card">
            <RequestQueue requests={requests} readOnly={source === 'dummy'}
                          onCreate={onCreate} onCancel={onCancel} onHover={setSelected} />
          </div>
          <div className="card">
            <SidePanel layout={layout} states={states} metrics={metrics}
                       lotVersion={lotVersion} selected={null} />
          </div>
        </div>
      </main>
    </>
  );
}
