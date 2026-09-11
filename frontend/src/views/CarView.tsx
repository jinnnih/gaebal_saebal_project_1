import { useState } from 'react';
import { useValetState } from '../hooks/useValetState.ts';
import { useRobotPose } from '../hooks/useRobotPose.ts';
import { useMyVehicle, myCarState } from '../hooks/useMyVehicle.ts';
import { createRequest } from '../api/client.ts';
import { ParkingMap } from '../components/ParkingMap.tsx';
import type { RequestStatus } from '../types/index.ts';

const STEP: Partial<Record<RequestStatus, string>> = {
  PENDING: '요청을 접수했습니다',
  ASSIGNED: '주차면을 배정했습니다',
  NAVIGATING: '주차면으로 이동 중입니다',
  PARKING: '주차하고 있습니다',
  UNPARKING: '차를 빼고 있습니다',
};

/**
 * 차 안 디스플레이 (IVI).
 *
 * 발렛의 인수인계 순간을 담당한다. 운전자가 하차장에 세우고 내리기 직전,
 * 그리고 돌아와서 다시 타는 순간이다. 주행 중에는 사람이 차에 없으므로
 * 이 화면은 "지금 어디까지 됐나" 를 보여주는 역할만 한다.
 *
 * 가로 와이드에 큰 터치 타깃으로 잡았다. 차량용 화면은 운전석에서 팔을 뻗어
 * 누르므로 데스크톱 밀도로 만들면 못 쓴다.
 */
export default function CarView() {
  const { layout, states, requests, source, error, refresh } = useValetState();
  const pose = useRobotPose(source === 'live');
  const { tag, setTag } = useMyVehicle();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [editing, setEditing] = useState(false);

  if (error) return <div className="fatal">불러오지 못했습니다: {error}</div>;
  if (!layout) return <div className="loading">불러오는 중…</div>;

  const { active, parked } = myCarState(requests, tag);
  const readOnly = source === 'dummy';

  const send = async (kind: 'PARK' | 'RETRIEVE', hasOccupant: boolean) => {
    setBusy(true); setErr(null);
    try {
      await createRequest(tag, kind, parked?.assigned_spot_id ?? null, 'IN_CAR', hasOccupant);
      await refresh();
    } catch (e: any) { setErr(e.message ?? '요청 실패'); }
    finally { setBusy(false); }
  };

  return (
    <div className="car">
      <header className="car-head">
        <span className="car-brand">VALET</span>
        {editing ? (
          <input className="car-plate-edit" autoFocus value={tag}
                 onChange={(e) => setTag(e.target.value)}
                 onBlur={() => setEditing(false)}
                 onKeyDown={(e) => e.key === 'Enter' && setEditing(false)} />
        ) : (
          <button className="car-plate" onClick={() => setEditing(true)}>{tag}</button>
        )}
        <span className={`src ${source}`}>
          {source === 'live' ? '연결됨' : '더미 데이터'}
        </span>
      </header>

      <main className="car-body">
        <div className="car-map">
          <ParkingMap layout={layout} states={states} pose={pose}
                      selected={active?.assigned_spot_id ?? parked?.assigned_spot_id ?? null}
                      onSelect={() => {}} />
        </div>

        <aside className="car-panel">
          {active ? (
            <>
              <p className="car-eyebrow">{active.kind === 'PARK' ? '발렛 주차' : '출차'}</p>
              <h1 className="car-title">{STEP[active.status] ?? active.status}</h1>
              <p className="car-sub">
                {active.assigned_spot_id
                  ? <>주차면 <b>{active.assigned_spot_id}</b></>
                  : '주차면을 고르는 중입니다'}
              </p>
              {active.has_occupant
                ? <p className="car-note">동승 모드 — 안전을 위해 천천히 이동합니다</p>
                : <p className="car-note">차량이 무인으로 이동 중입니다</p>}
            </>
          ) : parked ? (
            <>
              <p className="car-eyebrow">주차 완료</p>
              <h1 className="car-title">{parked.assigned_spot_id} 에 세워 뒀습니다</h1>
              {parked.position_err_m != null && (
                <p className="car-sub">정차 오차 {parked.position_err_m} m</p>
              )}
              <button className="car-btn primary" disabled={readOnly || busy}
                      onClick={() => send('RETRIEVE', false)}>
                {busy ? '요청 중…' : '차 가져오기'}
              </button>
            </>
          ) : (
            <>
              <p className="car-eyebrow">발렛 파킹 구역</p>
              <h1 className="car-title">주차를 맡기시겠습니까?</h1>
              <p className="car-sub">발렛을 고르면 <b>하차 후</b> 차량이 스스로 주차합니다.</p>
              <div className="car-actions">
                <button className="car-btn primary" disabled={readOnly || busy}
                        onClick={() => send('PARK', false)}>
                  발렛 주차
                  <small>내려서 맡기기</small>
                </button>
                <button className="car-btn" disabled={readOnly || busy}
                        onClick={() => send('PARK', true)}>
                  동승 주차
                  <small>탄 채로 자동 주차</small>
                </button>
              </div>
            </>
          )}

          {readOnly && <p className="car-note">더미 모드에서는 요청을 보낼 수 없습니다.</p>}
          {err && <p className="err">{err}</p>}
        </aside>
      </main>
    </div>
  );
}
