export type SpotStatus = 'FREE' | 'RESERVED' | 'OCCUPIED' | 'BLOCKED';
export type SpotType = 'standard' | 'accessible' | 'ev';
export type RequestKind = 'PARK' | 'RETRIEVE';
export type RequestStatus =
  | 'PENDING' | 'ASSIGNED' | 'NAVIGATING' | 'PARKING' | 'PARKED'
  | 'UNPARKING' | 'COMPLETED' | 'FAILED' | 'CANCELLED';

/** parking_spots.json 의 주차면 한 칸. 좌표는 map 프레임(m), y 는 위쪽이 + */
export interface LayoutSpot {
  id: string;
  row: string;
  index: number;
  type: SpotType;
  entry_side: 'N' | 'S';
  rect: [number, number, number, number];
  center: [number, number];
  goal_pose: [number, number, number];
  prepark_pose: [number, number, number];
  aisle_point: [number, number];
  initially_occupied: boolean;
}

export interface Layout {
  lot_name: string;
  scale: number;
  bounds: { x_min: number; x_max: number; y_min: number; y_max: number };
  stall: { width: number; depth: number };
  entry_pose: [number, number, number];
  exit_pose: [number, number, number];
  robot_spec: { length: number; width: number; wheelbase: number; min_turning_radius: number };
  spots: LayoutSpot[];
  hatched_zones: { id: string; rect: [number, number, number, number] }[];
  pillars: { x: number; y: number; size: number }[];
}

export interface SpotState {
  spot_id: string;
  status: SpotStatus;
  request_id: number | null;
  updated_at: string;
}

export interface RequestRow {
  id: number;
  kind: RequestKind;
  status: RequestStatus;
  vehicle_tag: string;
  assigned_spot_id: string | null;
  requested_at: string;
  finished_at: string | null;
  event_count: number;
  last_seq: number | null;
  duration_sec: number | null;
  position_err_m: number | null;
  heading_err_deg: number | null;
  shunt_count: number | null;
  succeeded: number | null;
  within_tolerance: number | null;
}

export interface Metrics {
  total: number;
  succeeded: number;
  within_tolerance: number;
  avg_duration_sec: number | null;
  avg_err_m: number | null;
  max_err_m: number | null;
  avg_shunts: number | null;
  tolerance_m: number;
}

export interface LotVersion {
  id: number;
  lot_name: string;
  checksum: string;
  spot_count: number;
  min_turning_radius: number | null;
  imported_at: string;
}
