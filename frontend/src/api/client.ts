/**
 * API 클라이언트.
 *
 * 레이아웃은 언제나 실제 원본(/parking_spots.json)에서 받는다.
 * DB 를 쓰는 나머지는 백엔드가 없으면 더미로 대체하고, 그 사실을 화면에 표시한다.
 */
import type { Layout, Metrics, PoseResponse, RequestRow, SpotState, LotVersion } from '../types/index.ts';
import { dummyMetrics, dummyRequests, dummySpotStates } from './dummy.ts';

export type Source = 'live' | 'dummy';

export async function fetchLayout(): Promise<Layout> {
  const res = await fetch('/parking_spots.json');
  if (!res.ok) throw new Error(`레이아웃을 불러오지 못했습니다 (${res.status})`);
  return res.json();
}

async function tryJson<T>(url: string): Promise<T | null> {
  try {
    const res = await fetch(url);
    if (!res.ok) return null;
    return (await res.json()) as T;
  } catch {
    return null;
  }
}

export async function fetchState(layout: Layout): Promise<{
  source: Source; spots: SpotState[]; requests: RequestRow[];
  metrics: Metrics; lotVersion: LotVersion | null;
}> {
  const live = await tryJson<{ lot_version: LotVersion; spots: SpotState[] }>('/api/spots');

  if (live?.spots?.length) {
    const [requests, metrics] = await Promise.all([
      tryJson<RequestRow[]>('/api/requests'),
      tryJson<Metrics>('/api/metrics'),
    ]);
    return {
      source: 'live',
      spots: live.spots,
      requests: requests ?? [],
      metrics: metrics ?? dummyMetrics(requests ?? []),
      lotVersion: live.lot_version,
    };
  }

  const requests = dummyRequests(layout);
  return {
    source: 'dummy',
    spots: dummySpotStates(layout),
    requests,
    metrics: dummyMetrics(requests),
    lotVersion: null,
  };
}

export async function createRequest(
  vehicleTag: string,
  kind: 'PARK' | 'RETRIEVE',
  spotId?: string | null,
  /** 어느 창구에서 눌렀는지. 같은 기능을 세 화면이 공유한다. */
  source: 'IN_CAR' | 'APP' | 'CONSOLE' = 'CONSOLE',
  hasOccupant = false,
) {
  const res = await fetch('/api/requests', {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify({
      vehicle_tag: vehicleTag, kind, spot_id: spotId ?? null,
      source, has_occupant: hasOccupant,
    }),
  });
  if (!res.ok) throw new Error((await res.json().catch(() => ({}))).error ?? '요청 실패');
  return res.json();
}

export async function cancelRequest(id: number) {
  const res = await fetch(`/api/requests/${id}/cancel`, { method: 'POST' });
  if (!res.ok) throw new Error((await res.json().catch(() => ({}))).error ?? '취소 실패');
  return res.json();
}

/** 로봇 위치. 백엔드가 메모리의 최신값을 주므로 자주 불러도 부담이 없다. */
export async function fetchPose(): Promise<PoseResponse | null> {
  return tryJson<PoseResponse>('/api/pose');
}
