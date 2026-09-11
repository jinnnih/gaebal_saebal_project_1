import { useState } from 'react';
import { useValetState } from '../hooks/useValetState.ts';
import { useRobotPose } from '../hooks/useRobotPose.ts';
import { cancelRequest, createRequest } from '../api/client.ts';
import { Header } from '../components/Header.tsx';
import { ParkingMap } from '../components/ParkingMap.tsx';
import { SidePanel } from '../components/SidePanel.tsx';
import { RequestQueue } from '../components/RequestQueue.tsx';

export default function ConsoleView() {
  const { layout, states, requests, metrics, lotVersion, source, error, refresh } = useValetState();
  const [selected, setSelected] = useState<string | null>(null);
  // 더미 모드에서는 로봇 위치를 받을 곳이 없다
  const pose = useRobotPose(source === 'live');

  if (error) return <div className="fatal">레이아웃을 불러오지 못했습니다: {error}</div>;
  if (!layout || !metrics) return <div className="loading">불러오는 중…</div>;

  const onCreate = async (tag: string, kind: 'PARK' | 'RETRIEVE', spotId?: string | null) => {
    await createRequest(tag, kind, spotId);
    await refresh();
  };
  const onCancel = async (id: number) => {
    await cancelRequest(id);
    await refresh();
  };

  return (
    <>
      <Header layout={layout} states={states} source={source} />

      <main>
        <div className="card">
          <ParkingMap layout={layout} states={states} pose={pose}
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
