-- 관제 대시보드 DB 스키마 (MySQL 8.0+ / 9.x)
--
-- 이슈 #8 의 설계를 MySQL 로 구현한 것. #9 에서 확정된 토픽 계약을 반영했다.
--   - ENUM, JSON 이 네이티브라 SQLite 판에서 썼던 CHECK 우회가 필요 없다
--   - mission_event.seq 는 #9 Q6 에서 규석이 요청한 요청 내 이벤트 순번
--   - park_metric.within_tolerance 는 nav2_ackermann.yaml 의 주차용
--     goal checker 허용오차 0.12 m 를 그대로 반영한 생성 컬럼
--
-- 주의: 주차면 좌표는 여기에 없다. 원본은 ks 브랜치의
--       src/parking_lot_world/config/parking_spots.json 이고 DB 는 spot_id 로 참조만 한다.

CREATE DATABASE IF NOT EXISTS valet
  CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;
USE valet;

SET FOREIGN_KEY_CHECKS = 0;
DROP TABLE IF EXISTS park_metric, mission_event, spot_state, valet_request, parking_spot, lot_version;
DROP VIEW  IF EXISTS v_current_spots, v_request_timeline;
SET FOREIGN_KEY_CHECKS = 1;

-- ─────────────────────────────────────────────────────────────
-- 레이아웃 스냅샷. parking_spots.json 이 재생성되면 새 행이 생긴다. (#6)
-- ─────────────────────────────────────────────────────────────
CREATE TABLE lot_version (
  id                 INT UNSIGNED     NOT NULL AUTO_INCREMENT,
  lot_name           VARCHAR(64)      NOT NULL,
  scale              DECIMAL(6,3)     NOT NULL,
  checksum           CHAR(64)         NOT NULL COMMENT 'parking_spots.json sha256',
  spot_count         SMALLINT UNSIGNED NOT NULL,
  min_turning_radius DECIMAL(6,4)     NULL COMMENT '#6 재검증 추적용',
  imported_at        DATETIME(3)      NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
  PRIMARY KEY (id),
  UNIQUE KEY uq_checksum (checksum)
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 주차면 카탈로그. 좌표 없음 — 조회/필터에 필요한 최소 정보만.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE parking_spot (
  lot_version_id     INT UNSIGNED      NOT NULL,
  spot_id            VARCHAR(8)        NOT NULL COMMENT 'A01 형식',
  row_label          CHAR(1)           NOT NULL,
  idx                SMALLINT UNSIGNED NOT NULL,
  type               ENUM('standard','accessible','ev') NOT NULL,
  entry_side         ENUM('N','S')     NOT NULL,
  initially_occupied BOOLEAN           NOT NULL,
  PRIMARY KEY (lot_version_id, spot_id),
  KEY idx_spot_type (lot_version_id, type),
  CONSTRAINT fk_spot_version FOREIGN KEY (lot_version_id)
    REFERENCES lot_version(id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 입차/출차 요청. 5주차 다중 요청 큐 UI 의 원천.
-- spot_state 가 참조하므로 먼저 정의한다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE valet_request (
  id               BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  kind             ENUM('PARK','RETRIEVE') NOT NULL,
  status           ENUM('PENDING','ASSIGNED','NAVIGATING','PARKING','PARKED',
                        'UNPARKING','COMPLETED','FAILED','CANCELLED')
                   NOT NULL DEFAULT 'PENDING',
  vehicle_tag      VARCHAR(32)     NOT NULL COMMENT '데모용 차량 식별자',
  lot_version_id   INT UNSIGNED    NULL,
  assigned_spot_id VARCHAR(8)      NULL,
  requested_at     DATETIME(3)     NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
  started_at       DATETIME(3)     NULL,
  finished_at      DATETIME(3)     NULL,
  failure_reason   VARCHAR(255)    NULL,
  PRIMARY KEY (id),
  KEY idx_queue (status, requested_at),
  CONSTRAINT fk_request_version FOREIGN KEY (lot_version_id)
    REFERENCES lot_version(id)
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 현재 점유 상태. 주차면 관리 노드가 /valet/spot_states 로 발행하는 값으로 갱신.
-- PK 가 (lot_version_id, spot_id) 라 한 면이 두 요청에 잡히는 일이 구조적으로 막힌다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE spot_state (
  lot_version_id INT UNSIGNED    NOT NULL,
  spot_id        VARCHAR(8)      NOT NULL,
  status         ENUM('FREE','RESERVED','OCCUPIED','BLOCKED') NOT NULL DEFAULT 'FREE',
  request_id     BIGINT UNSIGNED NULL COMMENT '이 면을 예약/점유한 요청',
  updated_at     DATETIME(3)     NOT NULL DEFAULT CURRENT_TIMESTAMP(3)
                                 ON UPDATE CURRENT_TIMESTAMP(3),
  PRIMARY KEY (lot_version_id, spot_id),
  KEY idx_state_status (lot_version_id, status),
  CONSTRAINT fk_state_spot FOREIGN KEY (lot_version_id, spot_id)
    REFERENCES parking_spot(lot_version_id, spot_id) ON DELETE CASCADE,
  CONSTRAINT fk_state_request FOREIGN KEY (request_id)
    REFERENCES valet_request(id) ON DELETE SET NULL
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 이벤트 로그 = 정량 지표의 원천. /valet/mission_status 를 그대로 적재한다.
-- event 13종은 #9 에서 확정 (규석이 SPOT_RESERVED / RECOVERY / ABORTED 추가 요청).
-- ─────────────────────────────────────────────────────────────
CREATE TABLE mission_event (
  id         BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  request_id BIGINT UNSIGNED NULL,
  seq        INT UNSIGNED    NOT NULL COMMENT '요청 내 순번 (#9 Q6) — 순서 뒤집힘·유실 감지',
  event      ENUM('REQUEST_ACCEPTED','SPOT_SELECTED','SPOT_RESERVED','NAV_STARTED',
                  'PREPARK_REACHED','PARK_STARTED','PARK_DONE','UNPARK_STARTED',
                  'UNPARK_DONE','EXIT_REACHED','RECOVERY','FAILED','ABORTED') NOT NULL,
  bt_node    VARCHAR(32)     NULL COMMENT 'FindParkingSpot, ParkManeuver, ...',
  payload    JSON            NOT NULL,
  ts         DATETIME(3)     NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
  PRIMARY KEY (id),
  UNIQUE KEY uq_request_seq (request_id, seq),
  KEY idx_event_request (request_id, ts),
  CONSTRAINT fk_event_request FOREIGN KEY (request_id)
    REFERENCES valet_request(id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 계획서 6장 '주차 소요시간 · 정차 오차 정량 지표' 산출물.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE park_metric (
  request_id       BIGINT UNSIGNED   NOT NULL,
  duration_sec     DECIMAL(8,2)      NULL COMMENT 'REQUEST_ACCEPTED -> PARK_DONE',
  position_err_m   DECIMAL(6,3)      NULL COMMENT 'goal_pose 대비 xy 거리',
  heading_err_deg  DECIMAL(6,2)      NULL COMMENT 'yaw 오차, + 는 반시계',
  shunt_count      SMALLINT UNSIGNED NULL COMMENT 'cmd_vel 부호 전환 실측 횟수',
  succeeded        BOOLEAN           NOT NULL,
  -- nav2_ackermann.yaml 의 parking_goal_checker 허용오차 0.12 m 를 그대로 반영
  within_tolerance BOOLEAN GENERATED ALWAYS AS (position_err_m <= 0.120) STORED,
  PRIMARY KEY (request_id),
  CONSTRAINT fk_metric_request FOREIGN KEY (request_id)
    REFERENCES valet_request(id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ─────────────────────────────────────────────────────────────
-- 뷰
-- ─────────────────────────────────────────────────────────────

-- 최신 레이아웃의 주차면 + 현재 상태. 좌표는 프런트가 JSON 에서 받아 spot_id 로 합친다.
CREATE OR REPLACE VIEW v_current_spots AS
SELECT s.spot_id, s.row_label, s.idx, s.type, s.entry_side,
       st.status, st.request_id, st.updated_at
FROM parking_spot s
JOIN spot_state st
  ON st.lot_version_id = s.lot_version_id AND st.spot_id = s.spot_id
WHERE s.lot_version_id = (SELECT MAX(id) FROM lot_version);

-- 요청별 타임라인 요약. 큐 UI 와 지표 화면이 같이 쓴다.
CREATE OR REPLACE VIEW v_request_timeline AS
SELECT r.id, r.kind, r.status, r.vehicle_tag, r.assigned_spot_id,
       r.requested_at, r.finished_at,
       COUNT(e.id)  AS event_count,
       MAX(e.seq)   AS last_seq,
       m.duration_sec, m.position_err_m, m.heading_err_deg,
       m.shunt_count, m.succeeded, m.within_tolerance
FROM valet_request r
LEFT JOIN mission_event e ON e.request_id = r.id
LEFT JOIN park_metric   m ON m.request_id = r.id
GROUP BY r.id, m.duration_sec, m.position_err_m, m.heading_err_deg,
         m.shunt_count, m.succeeded, m.within_tolerance;
