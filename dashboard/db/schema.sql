-- 관제 대시보드 DB 스키마 (SQLite)
-- 이슈 #8 의 스키마 초안을 SQLite 방언으로 옮긴 것.
--   PostgreSQL ENUM  -> TEXT + CHECK 제약
--   JSONB            -> TEXT + json_valid() 제약 (json1 확장)
--   TIMESTAMPTZ      -> TEXT (ISO8601 UTC, 밀리초 포함)
--
-- 주의: 주차면 좌표는 여기에 없다. 원본은 ks 브랜치의
--       src/parking_lot_world/config/parking_spots.json 이며,
--       이 DB 는 spot_id 로 참조만 한다. (이슈 #8 1장, 이슈 #6)

PRAGMA foreign_keys = ON;
PRAGMA journal_mode = WAL;

-- ─────────────────────────────────────────────────────────────
-- 레이아웃 스냅샷. parking_spots.json 이 재생성되면 새 행이 생긴다.
-- checksum 이 달라지는 순간을 감지해서 DB 가 조용히 어긋나는 걸 막는다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS lot_version (
  id                 INTEGER PRIMARY KEY AUTOINCREMENT,
  lot_name           TEXT    NOT NULL,
  scale              REAL    NOT NULL,
  checksum           TEXT    NOT NULL UNIQUE,   -- parking_spots.json sha256
  spot_count         INTEGER NOT NULL,
  min_turning_radius REAL,                      -- 이슈 #6 재검증 추적용
  imported_at        TEXT    NOT NULL
                     DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now'))
);

-- ─────────────────────────────────────────────────────────────
-- 주차면 카탈로그. 좌표 없음 — 조회/필터에 필요한 최소 정보만.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS parking_spot (
  lot_version_id     INTEGER NOT NULL
                     REFERENCES lot_version(id) ON DELETE CASCADE,
  spot_id            TEXT    NOT NULL,          -- 'A01'
  row_label          TEXT    NOT NULL CHECK(length(row_label) = 1),
  idx                INTEGER NOT NULL,
  type               TEXT    NOT NULL
                     CHECK(type IN ('standard','accessible','ev')),
  entry_side         TEXT    NOT NULL
                     CHECK(entry_side IN ('N','S')),
  initially_occupied INTEGER NOT NULL CHECK(initially_occupied IN (0,1)),
  PRIMARY KEY (lot_version_id, spot_id)
);

-- ─────────────────────────────────────────────────────────────
-- 입차/출차 요청. 5주차 다중 요청 큐 UI 의 원천.
-- spot_state 가 이 테이블을 참조하므로 먼저 정의한다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS valet_request (
  id               INTEGER PRIMARY KEY AUTOINCREMENT,
  kind             TEXT    NOT NULL CHECK(kind IN ('PARK','RETRIEVE')),
  status           TEXT    NOT NULL DEFAULT 'PENDING'
                   CHECK(status IN ('PENDING','ASSIGNED','NAVIGATING','PARKING',
                                    'PARKED','UNPARKING','COMPLETED',
                                    'FAILED','CANCELLED')),
  vehicle_tag      TEXT    NOT NULL,            -- 데모용 차량 식별자
  lot_version_id   INTEGER REFERENCES lot_version(id),
  assigned_spot_id TEXT,
  requested_at     TEXT    NOT NULL
                   DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
  started_at       TEXT,
  finished_at      TEXT,
  failure_reason   TEXT
);

CREATE INDEX IF NOT EXISTS idx_request_queue
  ON valet_request(status, requested_at);

-- ─────────────────────────────────────────────────────────────
-- 현재 점유 상태. 주차면 관리 노드가 rosbridge 로 발행하는 토픽으로 갱신.
-- PK 가 (lot_version_id, spot_id) 라 한 면이 두 요청에 잡히는 일이 구조적으로 막힌다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS spot_state (
  lot_version_id INTEGER NOT NULL,
  spot_id        TEXT    NOT NULL,
  status         TEXT    NOT NULL DEFAULT 'FREE'
                 CHECK(status IN ('FREE','RESERVED','OCCUPIED','BLOCKED')),
  request_id     INTEGER REFERENCES valet_request(id) ON DELETE SET NULL,
  updated_at     TEXT    NOT NULL
                 DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
  PRIMARY KEY (lot_version_id, spot_id),
  FOREIGN KEY (lot_version_id, spot_id)
    REFERENCES parking_spot(lot_version_id, spot_id) ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_spot_state_status
  ON spot_state(lot_version_id, status);

-- ─────────────────────────────────────────────────────────────
-- 이벤트 로그 = 정량 지표의 원천.
-- payload 는 규석의 BT 노드 출력이 확정될 때까지 JSON 으로 흡수한다.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS mission_event (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  request_id INTEGER REFERENCES valet_request(id) ON DELETE CASCADE,
  event      TEXT    NOT NULL,   -- 'SPOT_SELECTED','PREPARK_REACHED','PARK_DONE'
  bt_node    TEXT,               -- 'FindParkingSpot','ParkManeuver',...
  payload    TEXT    NOT NULL DEFAULT '{}' CHECK(json_valid(payload)),
  ts         TEXT    NOT NULL
             DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now'))
);

CREATE INDEX IF NOT EXISTS idx_event_request ON mission_event(request_id, ts);

-- ─────────────────────────────────────────────────────────────
-- 계획서 6장 '주차 소요시간 · 정차 오차 정량 지표' 산출물이 그대로 나오는 테이블.
-- ─────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS park_metric (
  request_id      INTEGER PRIMARY KEY
                  REFERENCES valet_request(id) ON DELETE CASCADE,
  duration_sec    REAL,      -- 요청 → 주차 완료
  position_err_m  REAL,      -- goal_pose 대비 정차 오차
  heading_err_deg REAL,
  shunt_count     INTEGER,   -- 전후진 전환 횟수 (Hybrid-A* 다점 기동 지표)
  succeeded       INTEGER NOT NULL CHECK(succeeded IN (0,1))
);

-- ─────────────────────────────────────────────────────────────
-- 대시보드가 쓰기 좋은 뷰: 최신 레이아웃의 주차면 + 현재 상태
-- 좌표는 프런트가 parking_spots.json 에서 받아 spot_id 로 합친다.
-- ─────────────────────────────────────────────────────────────
CREATE VIEW IF NOT EXISTS v_current_spots AS
SELECT s.spot_id, s.row_label, s.idx, s.type, s.entry_side,
       st.status, st.request_id, st.updated_at
FROM parking_spot s
JOIN spot_state  st USING (lot_version_id, spot_id)
WHERE s.lot_version_id = (SELECT MAX(id) FROM lot_version);
