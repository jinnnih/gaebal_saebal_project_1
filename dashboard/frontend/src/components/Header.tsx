import type { Layout, SpotState } from '../types/index.ts';
import type { Source } from '../api/client.ts';

interface Props {
  layout: Layout;
  states: SpotState[];
  source: Source;
}

export function Header({ layout, states, source }: Props) {
  const total = layout.spots.length;
  const free = states.filter((s) => s.status === 'FREE').length;

  return (
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
  );
}
