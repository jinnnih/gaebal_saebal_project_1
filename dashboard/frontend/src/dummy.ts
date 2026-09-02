/**
 * 백엔드/MySQL 없이도 화면이 뜨도록 하는 더미 데이터.
 *
 * 실제 로봇 노드가 아직 /valet/spot_states 와 /valet/mission_status 를 발행하지
 * 않으므로(#9 규석 답변), 계약대로 생긴 데이터를 만들어 UI 를 먼저 검증한다.
 * 레이아웃(좌표)은 더미가 아니라 언제나 ks 브랜치의 원본을 쓴다.
 */
import type { Layout, Metrics, RequestRow, SpotState } from './types.ts';

const TAGS = ['12가3456', '34나7890', '56다1234', '78라5678', '90마9012',
              '11바3344', '22사5566', '33아7788'];

/** 시드 고정 난수 — 새로고침해도 화면이 안 튄다. */
function rng(seed: number) {
  let s = seed;
  return () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
}

export function dummySpotStates(layout: Layout): SpotState[] {
  const r = rng(20260901);
  const now = new Date().toISOString();
  const free = layout.spots.filter((s) => !s.initially_occupied).map((s) => s.id);
  const reserved = new Set([free[3], free[11]]);          // 진행 중 요청 2건이 잡은 면
  return layout.spots.map((s) => ({
    spot_id: s.id,
    status: s.initially_occupied ? 'OCCUPIED' : reserved.has(s.id) ? 'RESERVED' : 'FREE',
    request_id: reserved.has(s.id) ? (s.id === free[3] ? 7 : 8) : null,
    updated_at: now,
  }));
}

export function dummyRequests(layout: Layout): RequestRow[] {
  const r = rng(4242);
  const free = layout.spots.filter((s) => !s.initially_occupied).map((s) => s.id);
  const rows: RequestRow[] = [];

  for (let i = 0; i < 6; i++) {
    const err = +(0.04 + r() * 0.14).toFixed(3);
    const ok = err <= 0.12;
    const dur = +(38 + r() * 45).toFixed(1);
    const at = new Date(Date.now() - (8 - i) * 6 * 60_000);
    rows.push({
      id: i + 1, kind: 'PARK', status: ok ? 'PARKED' : 'FAILED',
      vehicle_tag: TAGS[i], assigned_spot_id: free[i * 2],
      requested_at: at.toISOString(),
      finished_at: new Date(at.getTime() + dur * 1000).toISOString(),
      event_count: 7, last_seq: 7,
      duration_sec: dur, position_err_m: err,
      heading_err_deg: +((r() * 7 - 3.5).toFixed(2)),
      shunt_count: Math.floor(r() * 5),
      succeeded: ok ? 1 : 0, within_tolerance: ok ? 1 : 0,
    });
  }
  // 진행 중 2건 — 큐 UI 가 비지 않도록
  rows.push({
    id: 7, kind: 'PARK', status: 'PARKING', vehicle_tag: TAGS[6],
    assigned_spot_id: free[3], requested_at: new Date(Date.now() - 95_000).toISOString(),
    finished_at: null, event_count: 6, last_seq: 6,
    duration_sec: null, position_err_m: null, heading_err_deg: null,
    shunt_count: null, succeeded: null, within_tolerance: null,
  });
  rows.push({
    id: 8, kind: 'PARK', status: 'NAVIGATING', vehicle_tag: TAGS[7],
    assigned_spot_id: free[11], requested_at: new Date(Date.now() - 32_000).toISOString(),
    finished_at: null, event_count: 4, last_seq: 4,
    duration_sec: null, position_err_m: null, heading_err_deg: null,
    shunt_count: null, succeeded: null, within_tolerance: null,
  });

  return rows.sort((a, b) => b.requested_at.localeCompare(a.requested_at));
}

export function dummyMetrics(rows: RequestRow[]): Metrics {
  const done = rows.filter((r) => r.duration_sec != null);
  const avg = (f: (r: RequestRow) => number) =>
    done.length ? +(done.reduce((s, r) => s + f(r), 0) / done.length).toFixed(3) : null;
  return {
    total: done.length,
    succeeded: done.filter((r) => r.succeeded).length,
    within_tolerance: done.filter((r) => r.within_tolerance).length,
    avg_duration_sec: avg((r) => r.duration_sec!),
    avg_err_m: avg((r) => r.position_err_m!),
    max_err_m: done.length ? Math.max(...done.map((r) => r.position_err_m!)) : null,
    avg_shunts: avg((r) => r.shunt_count!),
    tolerance_m: 0.12,
  };
}
