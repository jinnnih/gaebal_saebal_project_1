/**
 * #9 에서 확정된 rosbridge 토픽 계약.
 *
 * 로봇과 대시보드가 합의한 내용을 코드로 옮긴 곳은 여기 하나다.
 * 계약이 바뀌면 이 파일만 고치면 된다.
 */

export const TOPIC = {
  spotStates: '/valet/spot_states',
  missionStatus: '/valet/mission_status',
  request: '/valet/request',
  robotPose: '/amcl_pose',
} as const;

/** 메시지는 std_msgs/String 에 JSON 문자열을 싣는다 (#9 Q2). */
export const MSG_TYPE = 'std_msgs/String';

/** 로봇 위치만 예외. Nav2 표준 토픽이라 계약 대상이 아니고 원래 타입을 그대로 쓴다. */
export const POSE_MSG_TYPE = 'geometry_msgs/PoseWithCovarianceStamped';

/**
 * 구독 QoS.
 *
 * 자동 매칭은 subscribe 시점에 퍼블리셔가 이미 떠 있어야 하고, volatile 퍼블리셔가
 * 하나라도 있으면 volatile 로 떨어진다. 규석이 명시 지정을 권했다 (#9 Q5).
 */
export const QOS = {
  spotStates: {
    history: 'keep_last', depth: 1,
    reliability: 'reliable', durability: 'transient_local',
  },
  missionStatus: {
    history: 'keep_last', depth: 20,
    reliability: 'reliable', durability: 'volatile',
  },
  /** 위치는 최신값만 의미가 있다. 밀려도 버리는 게 낫다. */
  robotPose: {
    history: 'keep_last', depth: 1,
    reliability: 'best_effort', durability: 'volatile',
  },
} as const;

/** #9 Q6 에서 확정된 13종. DB 의 mission_event.event ENUM 과 같아야 한다. */
export const EVENTS = [
  'REQUEST_ACCEPTED', 'SPOT_SELECTED', 'SPOT_RESERVED', 'NAV_STARTED',
  'PREPARK_REACHED', 'PARK_STARTED', 'PARK_DONE', 'UNPARK_STARTED',
  'UNPARK_DONE', 'EXIT_REACHED', 'RECOVERY', 'FAILED', 'ABORTED',
] as const;
export type MissionEvent = (typeof EVENTS)[number];

/** 이벤트 → valet_request.status. RECOVERY 는 상태를 바꾸지 않는다. */
export const STATUS_OF: Record<MissionEvent, string | null> = {
  REQUEST_ACCEPTED: 'PENDING',
  SPOT_SELECTED: 'ASSIGNED',
  SPOT_RESERVED: 'ASSIGNED',
  NAV_STARTED: 'NAVIGATING',
  PREPARK_REACHED: 'PARKING',
  PARK_STARTED: 'PARKING',
  PARK_DONE: 'PARKED',
  UNPARK_STARTED: 'UNPARKING',
  UNPARK_DONE: 'COMPLETED',
  EXIT_REACHED: 'COMPLETED',
  RECOVERY: null,
  FAILED: 'FAILED',
  ABORTED: 'CANCELLED',
};

export const TERMINAL_STATUS = ['PARKED', 'COMPLETED', 'FAILED', 'CANCELLED'];

/**
 * 주차 성공 판정 기준 (m).
 * nav2_ackermann.yaml 의 parking_goal_checker 허용오차를 그대로 쓴다.
 * DB 의 park_metric.within_tolerance 생성 컬럼도 같은 값이다.
 */
export const PARK_TOLERANCE_M = 0.12;

// ── 메시지 모양 ──────────────────────────────────────────────
export interface MissionStatusMsg {
  stamp?: string;
  request_id: number;
  seq: number;                    // 요청 내 순번 (#9 Q6)
  event: MissionEvent;
  bt_node?: string | null;
  payload?: Record<string, unknown>;
}

export interface SpotStatesMsg {
  stamp?: string;
  lot_checksum?: string;          // parking_spots.json sha256 앞 8자 (#6 감지용)
  spots: { id: string; status: string; request_id?: number | null }[];
}

/** geometry_msgs/PoseWithCovarianceStamped 중 우리가 쓰는 부분만. */
export interface PoseMsg {
  header?: { stamp?: { sec: number; nanosec: number }; frame_id?: string };
  pose?: {
    pose?: {
      position?: { x: number; y: number; z: number };
      orientation?: { x: number; y: number; z: number; w: number };
    };
  };
}

/** 대시보드가 쓰는 형태 — map 프레임 좌표(m)와 yaw(rad). */
export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
  stamp: string;
}

/** 쿼터니언에서 yaw 만 뽑는다. 평면 주행이라 roll/pitch 는 볼 필요가 없다. */
export function toRobotPose(msg: PoseMsg): RobotPose | null {
  const p = msg?.pose?.pose;
  if (!p?.position || !p?.orientation) return null;
  const { x: qx, y: qy, z: qz, w: qw } = p.orientation;
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  return { x: p.position.x, y: p.position.y, yaw, stamp: new Date().toISOString() };
}

export interface ValetRequestMsg {
  request_id: number;
  kind: 'PARK' | 'RETRIEVE';
  vehicle_tag: string;
  spot_id: string | null;
}
