import { useCallback, useState } from 'react';
import type { RequestRow } from '../types/index.ts';

const KEY = 'valet.myVehicle';
const DEFAULT = '12가3456';

/**
 * 차 안 화면과 사용자 앱이 "내 차"를 알아야 한다.
 *
 * 관제 콘솔은 주차장 전체를 보지만 이 둘은 차량 한 대의 시점이다.
 * 실제 제품이면 차량 VIN 이나 로그인 계정에서 오겠지만, 데모에서는
 * 브라우저에 저장해 둔 차량번호를 쓴다.
 */
export function useMyVehicle() {
  const [tag, setTagState] = useState<string>(() => {
    try { return localStorage.getItem(KEY) || DEFAULT; } catch { return DEFAULT; }
  });

  const setTag = useCallback((v: string) => {
    setTagState(v);
    try { localStorage.setItem(KEY, v); } catch { /* 사생활 모드 등 — 무시 */ }
  }, []);

  return { tag, setTag };
}

const ACTIVE = ['PENDING', 'ASSIGNED', 'NAVIGATING', 'PARKING', 'UNPARKING'];

/** 내 차의 지금 상태를 요청 목록에서 골라낸다. */
export function myCarState(requests: RequestRow[], tag: string) {
  const mine = requests.filter((r) => r.vehicle_tag === tag);
  const active = mine.find((r) => ACTIVE.includes(r.status)) ?? null;
  const parked = mine.find((r) => r.status === 'PARKED') ?? null;
  const last = mine[0] ?? null;   // requested_at 내림차순으로 온다
  return { active, parked, last, mine };
}
