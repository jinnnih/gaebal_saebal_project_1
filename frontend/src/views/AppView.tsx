import { useState } from 'react';
import { useValetState } from '../hooks/useValetState.ts';
import { useRobotPose } from '../hooks/useRobotPose.ts';
import { useMyVehicle, myCarState } from '../hooks/useMyVehicle.ts';
import { cancelRequest, createRequest } from '../api/client.ts';
import { ParkingMap } from '../components/ParkingMap.tsx';
import type { RequestStatus } from '../types/index.ts';

const KO: Partial<Record<RequestStatus, string>> = {
  PENDING: '접수됨', ASSIGNED: '주차면 배정', NAVIGATING: '이동 중',
  PARKING: '주차 중', UNPARKING: '출차 중',
};

/**
 * 사용자 앱 (운전자 폰).
 *
 * 차 밖에서 쓰는 화면이다. 발렛은 하차 후에 진행되므로 대부분의 시간을
 * 여기서 보게 된다. 세로 모바일 레이아웃으로 잡았다.
 *
 * 무개입 AVP 라 테슬라 Smart Summon 처럼 버튼을 누르고 있을 필요는 없다.
 * 요청을 넣고 자리를 떠도 된다.
 */
export default function AppView() {
  const { layout, states, requests, source, error, refresh } = useValetState();
  const pose = useRobotPose(source === 'live');
  const { tag, setTag } = useMyVehicle();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  if (error) return <div className="fatal">불러오지 못했습니다: {error}</div>;
  if (!layout) return <div className="loading">불러오는 중…</div>;

  const { active, parked } = myCarState(requests, tag);
  const readOnly = source === 'dummy';
  const free = states.filter((s) => s.status === 'FREE').length;

  const act = async (fn: () => Promise<unknown>) => {
    setBusy(true); setErr(null);
    try { await fn(); await refresh(); }
    catch (e: any) { setErr(e.message ?? '요청 실패'); }
    finally { setBusy(false); }
  };

  return (
    <div className="phone">
      <header className="phone-head">
        <div>
          <p className="phone-eyebrow">내 차</p>
          <input className="phone-plate" value={tag} onChange={(e) => setTag(e.target.value)} />
        </div>
        <span className={`src ${source}`}>{source === 'live' ? '연결됨' : '더미'}</span>
      </header>

      <section className="phone-status">
        {active ? (
          <>
            <span className="dot pulse" />
            <div>
              <b>{KO[active.status] ?? active.status}</b>
              <p>{active.assigned_spot_id
                ? `${active.kind === 'PARK' ? '주차면' : '출발지'} ${active.assigned_spot_id}`
                : '주차면을 고르는 중'}</p>
            </div>
            {!readOnly && (
              <button className="link" onClick={() => act(() => cancelRequest(active.id))}>
                취소
              </button>
            )}
          </>
        ) : parked ? (
          <>
            <span className="dot ok" />
            <div>
              <b>주차 완료</b>
              <p>{parked.assigned_spot_id}
                {parked.position_err_m != null && ` · 오차 ${parked.position_err_m} m`}</p>
            </div>
          </>
        ) : (
          <>
            <span className="dot idle" />
            <div>
              <b>주차되어 있지 않음</b>
              <p>공차 {free}면</p>
            </div>
          </>
        )}
      </section>

      <div className="phone-map">
        <ParkingMap layout={layout} states={states} pose={pose}
                    selected={active?.assigned_spot_id ?? parked?.assigned_spot_id ?? null}
                    onSelect={() => {}} />
      </div>

      {err && <p className="err">{err}</p>}

      <footer className="phone-actions">
        {parked ? (
          <button className="phone-btn primary" disabled={readOnly || busy}
                  onClick={() => act(() => createRequest(tag, 'RETRIEVE', parked.assigned_spot_id, 'APP'))}>
            {busy ? '요청 중…' : '차 가져오기'}
          </button>
        ) : (
          <button className="phone-btn primary" disabled={readOnly || busy || !!active}
                  onClick={() => act(() => createRequest(tag, 'PARK', null, 'APP'))}>
            {busy ? '요청 중…' : active ? '진행 중입니다' : '발렛 주차 요청'}
          </button>
        )}
      </footer>
    </div>
  );
}
