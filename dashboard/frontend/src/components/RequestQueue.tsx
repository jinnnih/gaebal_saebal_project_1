import { useState } from 'react';
import type { RequestRow, RequestStatus } from '../types.ts';

const ACTIVE: RequestStatus[] = ['PENDING', 'ASSIGNED', 'NAVIGATING', 'PARKING', 'UNPARKING'];
const KO: Record<RequestStatus, string> = {
  PENDING: '대기', ASSIGNED: '배정', NAVIGATING: '주행', PARKING: '주차중',
  PARKED: '주차완료', UNPARKING: '출차중', COMPLETED: '완료',
  FAILED: '실패', CANCELLED: '취소',
};

interface Props {
  requests: RequestRow[];
  readOnly: boolean;
  onCreate: (tag: string, kind: 'PARK' | 'RETRIEVE') => Promise<void>;
  onCancel: (id: number) => Promise<void>;
  onHover: (spotId: string | null) => void;
}

export function RequestQueue({ requests, readOnly, onCreate, onCancel, onHover }: Props) {
  const [tag, setTag] = useState('');
  const [kind, setKind] = useState<'PARK' | 'RETRIEVE'>('PARK');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!tag.trim()) return;
    setBusy(true); setErr(null);
    try { await onCreate(tag.trim(), kind); setTag(''); }
    catch (e: any) { setErr(e.message ?? '요청 실패'); }
    finally { setBusy(false); }
  };

  const active = requests.filter((r) => ACTIVE.includes(r.status));
  const done = requests.filter((r) => !ACTIVE.includes(r.status));

  return (
    <div className="queue">
      <form className="req-form" onSubmit={submit}>
        <select value={kind} onChange={(e) => setKind(e.target.value as any)} disabled={readOnly}>
          <option value="PARK">입차</option>
          <option value="RETRIEVE">출차</option>
        </select>
        <input value={tag} onChange={(e) => setTag(e.target.value)}
               placeholder="차량번호 (예: 12가3456)" disabled={readOnly} />
        <button type="submit" disabled={readOnly || busy || !tag.trim()}>
          {busy ? '요청 중…' : '요청'}
        </button>
      </form>
      {readOnly && <p className="note">더미 모드에서는 요청을 보낼 수 없습니다. 백엔드를 연결하세요.</p>}
      {err && <p className="err">{err}</p>}

      <h3>진행 중 <span className="badge">{active.length}</span></h3>
      {active.length === 0 && <p className="note">진행 중인 요청이 없습니다.</p>}
      <ul className="req-list">
        {active.map((r) => (
          <li key={r.id} onPointerEnter={() => onHover(r.assigned_spot_id)}
              onPointerLeave={() => onHover(null)}>
            <span className={`chip s-${r.status}`}>{KO[r.status]}</span>
            <b>{r.vehicle_tag}</b>
            <span className="muted">{r.assigned_spot_id ?? '면 배정 전'}</span>
            {!readOnly && (
              <button className="link" onClick={() => onCancel(r.id)}>취소</button>
            )}
          </li>
        ))}
      </ul>

      <h3>완료 <span className="badge">{done.length}</span></h3>
      <ul className="req-list">
        {done.slice(0, 8).map((r) => (
          <li key={r.id} onPointerEnter={() => onHover(r.assigned_spot_id)}
              onPointerLeave={() => onHover(null)}>
            <span className={`chip s-${r.status}`}>{KO[r.status]}</span>
            <b>{r.vehicle_tag}</b>
            <span className="muted">{r.assigned_spot_id}</span>
            {r.position_err_m != null && (
              <span className={r.within_tolerance ? 'ok' : 'bad'}>
                {r.position_err_m} m
              </span>
            )}
            {r.duration_sec != null && <span className="muted">{r.duration_sec}초</span>}
          </li>
        ))}
      </ul>
    </div>
  );
}
