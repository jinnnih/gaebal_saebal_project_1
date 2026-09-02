import { useMemo, useState } from 'react';
import type { Layout, SpotState, SpotStatus } from '../types.ts';

const FILL: Record<SpotStatus, string> = {
  FREE: 'var(--free)', RESERVED: 'var(--reserved)',
  OCCUPIED: 'var(--occupied)', BLOCKED: 'var(--blocked)',
};
const KO: Record<SpotStatus, string> = {
  FREE: '공차', RESERVED: '예약', OCCUPIED: '점유', BLOCKED: '차단',
};
const MARK: Record<string, string> = { accessible: '장애인', ev: 'EV', standard: '' };

interface Props {
  layout: Layout;
  states: SpotState[];
  selected: string | null;
  onSelect: (spotId: string | null) => void;
}

/**
 * 주차장 평면도.
 *
 * 좌표는 parking_spots.json 의 map 프레임(m)을 그대로 쓴다. map 은 y 가 위쪽이 +,
 * SVG 는 아래쪽이 + 라서 scale(1,-1) 로 뒤집고, 글자만 뒤집히지 않게 따로 그린다.
 */
export function ParkingMap({ layout, states, selected, onSelect }: Props) {
  const [hover, setHover] = useState<{ x: number; y: number; text: string } | null>(null);
  const statusOf = useMemo(
    () => new Map(states.map((s) => [s.spot_id, s.status])), [states]);

  const { bounds: b, spots, pillars, hatched_zones, entry_pose, exit_pose } = layout;
  const pad = 1.2;
  const viewBox = `${b.x_min - pad} ${-b.y_max - pad} ` +
                  `${b.x_max - b.x_min + pad * 2} ${b.y_max - b.y_min + pad * 2}`;

  return (
    <div className="map-wrap">
      <svg className="map" viewBox={viewBox} preserveAspectRatio="xMidYMid meet"
           onPointerLeave={() => setHover(null)}>
        <g transform="scale(1,-1)">
          <rect x={b.x_min} y={b.y_min}
                width={b.x_max - b.x_min} height={b.y_max - b.y_min}
                fill="var(--lot)" stroke="var(--line)" strokeWidth={0.12} />

          {hatched_zones?.map((z) => {
            const [x0, y0, x1, y1] = z.rect;
            return <rect key={z.id} x={x0} y={y0} width={x1 - x0} height={y1 - y0}
                         fill="rgba(139,148,158,.10)" stroke="#484f58"
                         strokeWidth={0.07} strokeDasharray=".5 .35" />;
          })}

          {spots.map((s) => {
            const [x0, y0, x1, y1] = s.rect;
            const st = statusOf.get(s.id) ?? 'FREE';
            const isSel = selected === s.id;
            return (
              <rect key={s.id} className="spot" x={x0} y={y0}
                    width={x1 - x0} height={y1 - y0} rx={0.18}
                    fill={FILL[st]} fillOpacity={st === 'OCCUPIED' ? 0.55 : 0.78}
                    stroke={isSel ? 'var(--accent)' : 'var(--bg)'}
                    strokeWidth={isSel ? 0.22 : 0.09}
                    onClick={() => onSelect(isSel ? null : s.id)}
                    onPointerMove={(e) => setHover({
                      x: e.clientX, y: e.clientY,
                      text: `${s.id} · ${KO[st]} · ${s.type} · 진입 ${s.entry_side}`,
                    })} />
            );
          })}

          {pillars?.map((p, i) => (
            <rect key={i} x={p.x - p.size / 2} y={p.y - p.size / 2}
                  width={p.size} height={p.size}
                  fill="var(--muted)" stroke="#c9d1d9" strokeWidth={0.04} />
          ))}

          <circle cx={entry_pose[0]} cy={entry_pose[1]} r={0.75}
                  fill="var(--accent)" fillOpacity={0.25}
                  stroke="var(--accent)" strokeWidth={0.12} />
          <circle cx={exit_pose[0]} cy={exit_pose[1]} r={0.75}
                  fill="var(--exit)" fillOpacity={0.25}
                  stroke="var(--exit)" strokeWidth={0.12} />
        </g>

        {/* 글자는 뒤집으면 안 되므로 y 부호만 바꿔 그린다 */}
        <g pointerEvents="none">
          {spots.map((s) => (
            <g key={s.id}>
              <text x={s.center[0]} y={-s.center[1] + 0.35} textAnchor="middle"
                    fontSize={0.95} fontWeight={700} fill="var(--bg)">{s.id}</text>
              {MARK[s.type] && (
                <text x={s.center[0]} y={-s.center[1] + 1.7} textAnchor="middle"
                      fontSize={0.62} fontWeight={600} fill="var(--bg)">{MARK[s.type]}</text>
              )}
            </g>
          ))}
          <text x={entry_pose[0]} y={-entry_pose[1] + 2.1} textAnchor="middle"
                fontSize={0.95} fontWeight={700} fill="var(--accent)">입구</text>
          <text x={exit_pose[0]} y={-exit_pose[1] + 2.1} textAnchor="middle"
                fontSize={0.95} fontWeight={700} fill="var(--exit)">출구</text>
        </g>
      </svg>

      {hover && (
        <div className="tip" style={{ left: hover.x + 14, top: hover.y + 14 }}>{hover.text}</div>
      )}
    </div>
  );
}
