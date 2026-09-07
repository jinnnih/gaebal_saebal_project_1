import { useState } from 'react';
import type { RequestRow, RequestStatus } from '../types/index.ts';

const ACTIVE: RequestStatus[] = ['PENDING', 'ASSIGNED', 'NAVIGATING', 'PARKING', 'UNPARKING'];
const KO: Record<RequestStatus, string> = {
  PENDING: '대기', ASSIGNED: '배정', NAVIGATING: '주행', PARKING: '주차중',
  PARKED: '주차완료', UNPARKING: '출차중', COMPLETED: '완료',
  FAILED: '실패', CANCELLED: '취소',
};

interface Props {
  requests: RequestRow[];
  readOnly: boolean;
  onCreate: (tag: string, kind: 'PARK' | 'RETRIEVE', spotId?: string | null) => Promise<void>;
  onCancel: (id: number) => Promise<void>;
  onHover: (spotId: string | null) => void;
}

export function RequestQueue({ requests, readOnly, onCreate, onCancel, onHover }: Props) {
  const [tag, setTag] = useState('');
  const [kind, setKind] = useState<'PARK' | 'RETRIEVE'>('PARK');
  const [retrieveId, setRetrieveId] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  // 출차는 세워 둔 차 중에서 고른다. 번호를 직접 치게 하면 오타로 실패한다.
  const parked = requests.filter((r) => r.status === 'PARKED' && r.assigned_spot_id);
  const picked = parked.find((r) => String(r.id) === retrieveId);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setBusy(true); setErr(null);
    try {
      if (kind === 'PARK') {
        if (!tag.trim()) return;
        await onCreate(tag.trim(), 'PARK');
        setTag('');
      } else {
        if (!picked) return;
        await onCreate(picked.vehicle_tag, 'RETRIEVE', picked.assigned_spot_id);
        setRetrieveId('');
      }
    } catch (e: any) { setErr(e.message ?? '요청 실패'); }
    finally { setBusy(false); }
  };

  const canSubmit = kind === 'PARK' ? !!tag.trim() : !!picked;

  const active = requests.filter((r) => ACTIVE.includes(r.status));
  const done = requests.filter((r) => !ACTIVE.includes(r.status));

  return (
    <div className="queue">
      <form className="req-form" onSubmit={submit}>
        <select value={kind} onChange={(e) => setKind(e.target.value as any)} disabled={readOnly}>
          <option value="PARK">입차</option>
          <option value="RETRIEVE">출차</option>
        </select>
        {kind === 'PARK' ? (
          <input value={tag} onChange={(e) => setTag(e.target.value)}
                 placeholder="차량번호 (예: 12가3456)" disabled={readOnly} />
        ) : (
          <select value={retrieveId} onChange={(e) => setRetrieveId(e.target.value)}
                  disabled={readOnly || parked.length === 0}>
            <option value="">
              {parked.length ? '차량 선택' : '주차된 차량 없음'}
            </option>
            {parked.map((r) => (
              <option key={r.id} value={r.id}>
                {r.vehicle_tag} · {r.assigned_spot_id}
              </option>
            ))}
          </select>
        )}
        <button type="submit" disabled={readOnly || busy || !canSubmit}>
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
            <span className={`chip k-${r.kind}`}>{r.kind === 'PARK' ? '입차' : '출차'}</span>
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
