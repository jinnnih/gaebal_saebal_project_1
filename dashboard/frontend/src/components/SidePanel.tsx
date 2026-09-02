import type { Layout, LotVersion, Metrics, RequestRow, SpotState, SpotStatus } from '../types.ts';

const SWATCH: Record<SpotStatus, string> = {
  FREE: 'var(--free)', RESERVED: 'var(--reserved)',
  OCCUPIED: 'var(--occupied)', BLOCKED: 'var(--blocked)',
};
const KO: Record<SpotStatus, string> = {
  FREE: '공차', RESERVED: '예약', OCCUPIED: '점유', BLOCKED: '차단',
};

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return <section className="panel"><h2>{title}</h2>{children}</section>;
}

function Row({ label, value, swatch }: { label: string; value: React.ReactNode; swatch?: string }) {
  return (
    <div className="row">
      <span>{swatch && <i className="swatch" style={{ background: swatch }} />}{label}</span>
      <span className="num">{value}</span>
    </div>
  );
}

interface Props {
  layout: Layout;
  states: SpotState[];
  metrics: Metrics;
  lotVersion: LotVersion | null;
  selected: RequestRow | null;
}

export function SidePanel({ layout, states, metrics, lotVersion }: Props) {
  const counts = states.reduce<Record<string, number>>(
    (a, s) => ((a[s.status] = (a[s.status] ?? 0) + 1), a), {});
  const types = layout.spots.reduce<Record<string, number>>(
    (a, s) => ((a[s.type] = (a[s.type] ?? 0) + 1), a), {});

  const pct = (n: number | null, d: number) =>
    d ? `${((100 * (n ?? 0)) / d).toFixed(0)}%` : '—';

  return (
    <aside className="side">
      <Section title="점유 현황">
        {(Object.keys(KO) as SpotStatus[]).map((k) => (
          <Row key={k} label={KO[k]} value={counts[k] ?? 0} swatch={SWATCH[k]} />
        ))}
      </Section>

      <Section title="정량 지표">
        {metrics.total ? (
          <>
            <Row label="완료 요청" value={`${metrics.total}건`} />
            <Row label="성공률" value={pct(metrics.succeeded, metrics.total)} />
            <Row label={`허용오차 ${metrics.tolerance_m}m 이내`}
                 value={pct(metrics.within_tolerance, metrics.total)} />
            <Row label="평균 소요" value={metrics.avg_duration_sec != null
                 ? `${metrics.avg_duration_sec}초` : '—'} />
            <Row label="평균 정차오차" value={metrics.avg_err_m != null
                 ? `${metrics.avg_err_m} m` : '—'} />
            <Row label="최대 정차오차" value={metrics.max_err_m != null
                 ? `${metrics.max_err_m} m` : '—'} />
            <Row label="평균 전후진" value={metrics.avg_shunts != null
                 ? `${metrics.avg_shunts}회` : '—'} />
          </>
        ) : <p className="note">아직 완료된 요청이 없습니다.</p>}
      </Section>

      <Section title="주차면 타입">
        {Object.entries(types).map(([k, n]) => <Row key={k} label={k} value={n} />)}
      </Section>

      <Section title="레이아웃">
        <Row label="주차장" value={layout.lot_name} />
        <Row label="크기" value={`${(layout.bounds.x_max - layout.bounds.x_min).toFixed(0)} × ${(layout.bounds.y_max - layout.bounds.y_min).toFixed(1)} m`} />
        <Row label="주차면" value={`${layout.spots.length}면`} />
        <Row label="기둥" value={`${layout.pillars.length}개`} />
        <Row label="최소회전반경" value={`${layout.robot_spec.min_turning_radius.toFixed(2)} m`} />
        <Row label="lot_version" value={lotVersion
          ? `#${lotVersion.id} ${lotVersion.checksum.slice(0, 8)}` : 'DB 미연결'} />
      </Section>
    </aside>
  );
}
