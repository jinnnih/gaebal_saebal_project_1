#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
차량형(Ackermann) 로봇 자율 발렛파킹 - 주차장 맵 생성기
=========================================================
하나의 기하 정의(SPEC)로부터 아래 산출물을 전부 생성한다.

  worlds/parking_lot.sdf          Gazebo Harmonic (gz-sim 8) 월드
  maps/parking_lot.{pgm,yaml}     Nav2 static map (벽 + 기둥만)
  maps/parking_lot_occupied.*     참고용 (주차된 차량까지 포함)
  maps/keepout_mask.*             Nav2 Keepout Filter (주차면 내부 진입 금지)
  maps/speed_mask.*               Nav2 Speed Filter (주차면 앞 / 게이트 감속)
  config/parking_spots.yaml       주차면 관리 노드 ROS2 파라미터
  config/parking_spots.json       관제 대시보드(팀원 B)용
  config/nav2_ackermann.yaml      Smac Hybrid-A* + MPPI 파라미터
  config/costmap_filters.yaml     keepout / speed filter 서버 파라미터

좌표계
  map 프레임 원점 = 주차장 바닥 중심. X = 동(오른쪽), Y = 북(위), yaw = ENU 반시계.

스케일 변경
  SCALE 상수만 바꾸면 전체 주차장/차량 치수가 비례 축소된다.
  실차 크기(SCALE=1.0)가 기본. 소형 실습 로봇이면 SCALE=0.4 권장.

사용법
  python3 generate_parking_lot.py [출력_패키지_루트]
"""

import json
import math
import os
import sys

# ===========================================================================
# 0. 전역 스케일
# ===========================================================================
SCALE = 1.0  # 1.0 = 실차 스케일. 소형 로봇이면 0.4 로 낮추고 재생성.




def S(v):
    """스케일 적용."""
    return v * SCALE


# ===========================================================================
# 1. 주차장 기하 정의 (단위: m, SCALE 적용 전 실치수)
# ===========================================================================
STALL_W = S(2.50)   # 주차면 폭
STALL_D = S(5.40)   # 주차면 깊이 (뒤쪽 기둥 여유 포함)
AISLE_SIDE = S(7.00)   # 남/북 외곽 통로 폭
AISLE_CENTER = S(8.00)   # 중앙 통로 폭 (B/C 열 공용, 회전 여유)
LANE_W = S(7.00)   # 동/서 연결 차로 폭
N_STALLS = 14      # 열당 주차면 수

LINE_W = S(0.12)   # 노면 표시 선 두께
LINE_Z = S(0.006)  # 노면 표시 두께(높이)
WALL_H = S(2.60)   # 벽 높이
WALL_T = S(0.20)   # 벽 두께
PILLAR = S(0.50)   # 기둥 한 변
GATE_W = S(5.00)   # 입/출구 개구부 폭

PREPARK_BACK = S(5.00)  # 대기지점 -> 최종 주차 위치까지의 직선 후진 거리

# 차량(주차된 차) 치수
CAR_L, CAR_W, CAR_H = S(4.40), S(1.80), S(1.45)
WHEEL_R, WHEEL_T = S(0.33), S(0.22)

# 로봇 스펙 (URDF 설계 목표치 — Nav2 파라미터와 연동)
ROBOT_L = S(4.50)
ROBOT_W = S(1.90)
ROBOT_WHEELBASE = S(2.50)
ROBOT_MAX_STEER = math.radians(35.0)
ROBOT_MIN_R = ROBOT_WHEELBASE / math.tan(ROBOT_MAX_STEER)  # ≈ 3.57 m

# --- 열(Row) 배치: 남 -> 북 ---------------------------------------------
#   Aisle_S | Row A | Row B | Aisle_C | Row C | Row D | Aisle_N
_H = AISLE_SIDE + STALL_D * 2 + AISLE_CENTER + STALL_D * 2 + AISLE_SIDE
Y_MIN, Y_MAX = -_H / 2.0, _H / 2.0

_y = Y_MIN
AISLE_S = (_y, _y + AISLE_SIDE);      _y += AISLE_SIDE
ROW_A_Y = (_y, _y + STALL_D);         _y += STALL_D
ROW_B_Y = (_y, _y + STALL_D);         _y += STALL_D
AISLE_C = (_y, _y + AISLE_CENTER);    _y += AISLE_CENTER
ROW_C_Y = (_y, _y + STALL_D);         _y += STALL_D
ROW_D_Y = (_y, _y + STALL_D);         _y += STALL_D
AISLE_N = (_y, _y + AISLE_SIDE)

BLOCK_W = STALL_W * N_STALLS          # 주차면 블록 가로 길이
_W = LANE_W + BLOCK_W + LANE_W + S(1.0)   # 좌우 0.5m 씩 여유
X_MIN, X_MAX = -_W / 2.0, _W / 2.0
BLOCK_X0 = -BLOCK_W / 2.0             # 주차면 블록 서쪽 끝

AISLE_S_C = (AISLE_S[0] + AISLE_S[1]) / 2.0   # 남 통로 중심선
AISLE_C_C = (AISLE_C[0] + AISLE_C[1]) / 2.0   # 중앙 통로 중심선
AISLE_N_C = (AISLE_N[0] + AISLE_N[1]) / 2.0   # 북 통로 중심선
LANE_W_C = X_MIN + LANE_W / 2.0                # 서측 차로 중심선
LANE_E_C = X_MAX - LANE_W / 2.0                # 동측 차로 중심선

# 입구(서벽) / 출구(동벽)
GATE_IN = (LANE_W_C * 0 + AISLE_S_C, X_MIN)   # (y중심, x)
GATE_OUT = (AISLE_N_C, X_MAX)

START_POSE = (X_MIN + S(2.0), AISLE_S_C, 0.0)          # 입차 시작 지점
EXIT_POSE = (X_MAX - S(1.5), AISLE_N_C, 0.0)           # 출차 최종 목표
QUEUE_POSE = (LANE_W_C, AISLE_S_C + S(3.0), math.pi / 2)  # 대기열 지점

# --- 열 메타 ------------------------------------------------------------
#   entry: 'S' = 남쪽 통로에서 진입(차 앞머리는 남쪽), 'N' = 그 반대
ROWS = [
    {"name": "A", "y": ROW_A_Y, "entry": "S", "aisle_c": AISLE_S_C},
    {"name": "B", "y": ROW_B_Y, "entry": "N", "aisle_c": AISLE_C_C},
    {"name": "C", "y": ROW_C_Y, "entry": "S", "aisle_c": AISLE_C_C},
    {"name": "D", "y": ROW_D_Y, "entry": "N", "aisle_c": AISLE_N_C},
]

# 특수 주차면
ACCESSIBLE = {"A01", "A03"}          # 장애인 전용
HATCHED = {"A02", "A04"}             # 안전 통로(빗금) — 주차 불가
EV_STALLS = {"D13", "D14"}           # 전기차 충전

# 초기 점유 주차면 (결정론적 — 데모 재현성 확보)
OCCUPIED_INIT = {
    "A05", "A06", "A09", "A12", "A14",
    "B02", "B03", "B07", "B08", "B11", "B14",
    "C01", "C04", "C05", "C09", "C10", "C13",
    "D02", "D03", "D06", "D07", "D11",
}

# 통로에 놓인 정적 장애물(코스트맵/회피 데모용) — (x, y, 반지름, 높이)
CONES = [
    (S(-6.0), AISLE_S_C - S(2.0), S(0.22), S(0.55)),
    (S(4.5), AISLE_C_C + S(2.6), S(0.22), S(0.55)),
    (S(-11.0), AISLE_N_C - S(2.2), S(0.22), S(0.55)),
]

# --- 맵(Occupancy grid) 범위 -------------------------------------------
MAP_RES = S(0.05)
MAP_PAD = S(1.0)
MAP_X0, MAP_X1 = X_MIN - MAP_PAD, X_MAX + MAP_PAD
MAP_Y0, MAP_Y1 = Y_MIN - MAP_PAD, Y_MAX + MAP_PAD
MAP_W = int(round((MAP_X1 - MAP_X0) / MAP_RES))
MAP_H = int(round((MAP_Y1 - MAP_Y0) / MAP_RES))

FREE, OCC, UNK = 254, 0, 205

# --- 색상 (ambient, diffuse) -------------------------------------------
COL = {
    "asphalt":  ("0.22 0.22 0.24 1", "0.26 0.26 0.28 1"),
    "white":    ("0.85 0.85 0.85 1", "0.95 0.95 0.95 1"),
    "yellow":   ("0.72 0.58 0.06 1", "0.90 0.74 0.10 1"),
    "blue":     ("0.06 0.18 0.55 1", "0.10 0.30 0.80 1"),
    "green":    ("0.05 0.35 0.18 1", "0.10 0.60 0.32 1"),
    "red":      ("0.55 0.08 0.06 1", "0.80 0.14 0.10 1"),
    "wall":     ("0.55 0.55 0.53 1", "0.72 0.72 0.70 1"),
    "pillar":   ("0.42 0.42 0.45 1", "0.58 0.58 0.62 1"),
    "curb":     ("0.60 0.60 0.58 1", "0.78 0.78 0.76 1"),
    "carbody":  ("0.10 0.12 0.16 1", "0.30 0.36 0.46 1"),
    "glass":    ("0.06 0.08 0.10 1", "0.12 0.16 0.20 1"),
    "tire":     ("0.03 0.03 0.03 1", "0.08 0.08 0.08 1"),
    "cone":     ("0.60 0.22 0.03 1", "0.95 0.38 0.05 1"),
    "charger":  ("0.10 0.30 0.20 1", "0.15 0.55 0.35 1"),
    "gate":     ("0.60 0.20 0.05 1", "0.90 0.35 0.08 1"),
}


# ===========================================================================
# 2. 주차면 테이블 생성
# ===========================================================================
def build_spots():
    """모든 주차면의 기하/포즈를 계산해 리스트로 반환."""
    spots = []
    for row in ROWS:
        y0, y1 = row["y"]
        entry = row["entry"]
        cy = (y0 + y1) / 2.0

        if entry == "S":
            mouth_y = y0                 # 통로와 접한 면
            final_yaw = -math.pi / 2     # 앞머리가 남쪽(통로) — 전진 탈출 가능
            prepark_y = cy - PREPARK_BACK
        else:
            mouth_y = y1
            final_yaw = math.pi / 2      # 앞머리가 북쪽(통로)
            prepark_y = cy + PREPARK_BACK

        for i in range(N_STALLS):
            sx0 = BLOCK_X0 + STALL_W * i
            cx = sx0 + STALL_W / 2.0
            sid = "%s%02d" % (row["name"], i + 1)

            kind = "standard"
            if sid in HATCHED:
                kind = "hatched"
            elif sid in ACCESSIBLE:
                kind = "accessible"
            elif sid in EV_STALLS:
                kind = "ev"

            spots.append({
                "id": sid,
                "row": row["name"],
                "index": i + 1,
                "type": kind,
                "entry_side": entry,
                "x0": sx0, "x1": sx0 + STALL_W,
                "y0": y0, "y1": y1,
                "center": (cx, cy),
                "mouth": (cx, mouth_y),
                # 최종 주차 포즈 (후진 완료 지점, base_link 기준)
                "goal_pose": (cx, cy, final_yaw),
                # 후진 직전 대기 포즈 (통로 위, 주차면을 등지고 정렬)
                "prepark_pose": (cx, prepark_y, final_yaw),
                # 통로 주행 중 경유점 (통로 중심선 위)
                "aisle_point": (cx, row["aisle_c"]),
                "parkable": kind != "hatched",
                "occupied": sid in OCCUPIED_INIT,
            })
    return spots


SPOTS = build_spots()
SPOT_BY_ID = {s["id"]: s for s in SPOTS}
PARKABLE = [s for s in SPOTS if s["parkable"]]


# ===========================================================================
# 3. 정적 구조물 (벽 / 기둥 / 연석) 기하
# ===========================================================================
def build_walls():
    """(name, cx, cy, sx, sy, sz) 목록. 입/출구 개구부 반영."""
    t, h = WALL_T, WALL_H
    xw, xe = X_MIN - t / 2.0, X_MAX + t / 2.0
    ys, yn = Y_MIN - t / 2.0, Y_MAX + t / 2.0
    span_x = (X_MAX - X_MIN) + 2 * t

    w = []
    # 남/북 벽 (연속)
    w.append(("wall_south", 0.0, ys, span_x, t, h))
    w.append(("wall_north", 0.0, yn, span_x, t, h))

    # 서벽 — 입구 개구부 (중심 AISLE_S_C)
    g0, g1 = AISLE_S_C - GATE_W / 2.0, AISLE_S_C + GATE_W / 2.0
    w.append(("wall_west_a", xw, (Y_MIN - t + g0) / 2.0, t, (g0 - (Y_MIN - t)), h))
    w.append(("wall_west_b", xw, (g1 + Y_MAX + t) / 2.0, t, ((Y_MAX + t) - g1), h))

    # 동벽 — 출구 개구부 (중심 AISLE_N_C)
    e0, e1 = AISLE_N_C - GATE_W / 2.0, AISLE_N_C + GATE_W / 2.0
    w.append(("wall_east_a", xe, (Y_MIN - t + e0) / 2.0, t, (e0 - (Y_MIN - t)), h))
    w.append(("wall_east_b", xe, (e1 + Y_MAX + t) / 2.0, t, ((Y_MAX + t) - e1), h))
    return w


def build_pillars():
    """A/B, C/D 열 경계선 위, 주차면 2칸마다 구조 기둥."""
    out = []
    for by, tag in ((ROW_A_Y[1], "AB"), (ROW_C_Y[1], "CD")):
        for j in range(0, N_STALLS + 1, 2):
            x = BLOCK_X0 + STALL_W * j
            out.append(("pillar_%s_%02d" % (tag, j), x, by, PILLAR, PILLAR, WALL_H))
    return out


WALLS = build_walls()
PILLARS = build_pillars()


# ===========================================================================
# 4. SDF 생성
# ===========================================================================
def mat(key):
    a, d = COL[key]
    return ("<material><ambient>%s</ambient><diffuse>%s</diffuse>"
            "<specular>0.1 0.1 0.1 1</specular></material>" % (a, d))


def box_vis(name, cx, cy, cz, sx, sy, sz, key, yaw=0.0, collide=True):
    """collision + visual 쌍(또는 visual만)을 문자열로."""
    pose = "<pose>%.4f %.4f %.4f 0 0 %.6f</pose>" % (cx, cy, cz, yaw)
    geo = "<geometry><box><size>%.4f %.4f %.4f</size></box></geometry>" % (sx, sy, sz)
    out = ""
    if collide:
        out += "        <collision name='%s_col'>%s%s</collision>\n" % (name, pose, geo)
    out += "        <visual name='%s_vis'>%s%s%s</visual>\n" % (name, pose, geo, mat(key))
    return out


def cyl_vis(name, cx, cy, cz, r, h, key, collide=True):
    pose = "<pose>%.4f %.4f %.4f 0 0 0</pose>" % (cx, cy, cz)
    geo = ("<geometry><cylinder><radius>%.4f</radius><length>%.4f</length>"
           "</cylinder></geometry>" % (r, h))
    out = ""
    if collide:
        out += "        <collision name='%s_col'>%s%s</collision>\n" % (name, pose, geo)
    out += "        <visual name='%s_vis'>%s%s%s</visual>\n" % (name, pose, geo, mat(key))
    return out


# 노면 표시 z 레이어 — 겹치는 표시끼리 높이를 벌려 z-fighting(깜빡임) 방지.
#   L0 바닥 도색면 / L1 하위 선 / L2 상위 선 / L3 최상위
# 바닥 윗면이 z=0 이므로 최하층도 7mm 는 띄운다 (원거리 시점 깊이정밀도 확보).
LINE_LAYER = (S(0.010), S(0.017), S(0.024), S(0.031))


def line(name, cx, cy, sx, sy, key="white", yaw=0.0, layer=2):
    """노면 도색 — 충돌체 없음. layer 로 z 를 분리한다."""
    return box_vis(name, cx, cy, LINE_LAYER[layer], sx, sy, LINE_Z, key, yaw,
                   collide=False)


def build_markings():
    """노면 표시 전체(주차면 선 / 통로 중심선 / 화살표 / 특수면 도색)."""
    b = []
    n = 0

    for row in ROWS:
        y0, y1 = row["y"]
        cy = (y0 + y1) / 2.0
        rn = row["name"]

        # 주차면 구분선 (세로) — 15개 경계. 후면선과 교차하므로 상위 레이어.
        for j in range(N_STALLS + 1):
            x = BLOCK_X0 + STALL_W * j
            b.append(line("l_%s_v%02d" % (rn, j), x, cy, LINE_W, STALL_D, layer=2))
        # 주차면 안쪽 끝(후면) 선 — 세로선 아래 레이어
        back_y = y1 if row["entry"] == "S" else y0
        off = -LINE_W / 2.0 if row["entry"] == "S" else LINE_W / 2.0
        b.append(line("l_%s_back" % rn, 0.0, back_y + off, BLOCK_W, LINE_W, layer=1))

    # 특수 주차면 바닥 도색
    for s in SPOTS:
        cx, cy = s["center"]
        if s["type"] == "accessible":
            b.append(line("pf_%s" % s["id"], cx, cy,
                          STALL_W - S(0.30), STALL_D - S(0.30), "blue", layer=0))
        elif s["type"] == "ev":
            b.append(line("pf_%s" % s["id"], cx, cy,
                          STALL_W - S(0.30), STALL_D - S(0.30), "green", layer=0))
        elif s["type"] == "hatched":
            # 빗금(45도) — 주차 금지 안전 통로
            k = 0
            step = S(0.70)
            d = STALL_D
            x0 = s["x0"] + S(0.15)
            while x0 < s["x1"] - S(0.15):
                b.append(line("hz_%s_%02d" % (s["id"], k), x0, cy,
                              S(0.10), d * 1.05, "yellow", yaw=math.radians(35),
                              layer=3))
                x0 += step
                k += 1

    # 통로 중심선 (노란 점선)
    dash, gap = S(1.60), S(1.20)
    for cy_, tag in ((AISLE_S_C, "S"), (AISLE_C_C, "C"), (AISLE_N_C, "N")):
        x = X_MIN + LANE_W
        k = 0
        while x < X_MAX - LANE_W:
            ln = min(dash, (X_MAX - LANE_W) - x)
            b.append(line("dash_%s_%03d" % (tag, k), x + ln / 2.0, cy_,
                          ln, LINE_W, "yellow", layer=1))
            x += dash + gap
            k += 1
    # 좌우 연결 차로 중심선 (세로 점선)
    for cx_, tag in ((LANE_W_C, "W"), (LANE_E_C, "E")):
        y = Y_MIN + S(1.0)
        k = 0
        while y < Y_MAX - S(1.0):
            ln = min(dash, (Y_MAX - S(1.0)) - y)
            b.append(line("dashv_%s_%03d" % (tag, k), cx_, y + ln / 2.0,
                          LINE_W, ln, "yellow", layer=1))
            y += dash + gap
            k += 1

    # 입구/출구 정지선 + 진행 화살표
    b.append(line("stop_in", X_MIN + S(2.6), AISLE_S_C, S(0.25), GATE_W * 0.9, "white"))
    b.append(line("stop_out", X_MAX - S(2.6), AISLE_N_C, S(0.25), GATE_W * 0.9, "white"))
    for i in range(3):
        ax = X_MIN + S(4.5) + S(3.0) * i
        b.append(line("arr_in_%d" % i, ax, AISLE_S_C, S(2.0), S(0.28), "white"))
        b.append(line("arrh_in_%da" % i, ax + S(1.1), AISLE_S_C + S(0.28),
                      S(0.9), S(0.20), "white", yaw=math.radians(-35), layer=3))
        b.append(line("arrh_in_%db" % i, ax + S(1.1), AISLE_S_C - S(0.28),
                      S(0.9), S(0.20), "white", yaw=math.radians(35), layer=3))
    for i in range(3):
        ax = X_MAX - S(4.5) - S(3.0) * i
        b.append(line("arr_out_%d" % i, ax, AISLE_N_C, S(2.0), S(0.28), "white"))
        b.append(line("arrh_out_%da" % i, ax + S(1.1), AISLE_N_C + S(0.28),
                      S(0.9), S(0.20), "white", yaw=math.radians(-35), layer=3))
        b.append(line("arrh_out_%db" % i, ax + S(1.1), AISLE_N_C - S(0.28),
                      S(0.9), S(0.20), "white", yaw=math.radians(35), layer=3))

    return "".join(b)


def car_model(name, cx, cy, yaw, color="carbody"):
    """정적 주차 차량 (충돌체 포함 — 라이다에 잡힘)."""
    hb = CAR_H * 0.52
    body = ""
    body += box_vis("body", 0, 0, WHEEL_R + S(0.30), CAR_L, CAR_W, S(0.72), color)
    body += box_vis("cabin", -S(0.25), 0, WHEEL_R + S(0.30) + S(0.62),
                    CAR_L * 0.50, CAR_W * 0.92, S(0.52), "glass")
    for sx_, sy_, tag in ((CAR_L * 0.32, CAR_W * 0.42, "fl"),
                          (CAR_L * 0.32, -CAR_W * 0.42, "fr"),
                          (-CAR_L * 0.32, CAR_W * 0.42, "rl"),
                          (-CAR_L * 0.32, -CAR_W * 0.42, "rr")):
        body += ("        <collision name='w_%s_col'><pose>%.4f %.4f %.4f "
                 "1.5708 0 0</pose><geometry><cylinder><radius>%.4f</radius>"
                 "<length>%.4f</length></cylinder></geometry></collision>\n"
                 % (tag, sx_, sy_, WHEEL_R, WHEEL_R, WHEEL_T))
        body += ("        <visual name='w_%s_vis'><pose>%.4f %.4f %.4f "
                 "1.5708 0 0</pose><geometry><cylinder><radius>%.4f</radius>"
                 "<length>%.4f</length></cylinder></geometry>%s</visual>\n"
                 % (tag, sx_, sy_, WHEEL_R, WHEEL_R, WHEEL_T, mat("tire")))
    _ = hb
    return ("    <model name='%s'>\n"
            "      <static>true</static>\n"
            "      <pose>%.4f %.4f 0 0 0 %.6f</pose>\n"
            "      <link name='link'>\n%s      </link>\n"
            "    </model>\n" % (name, cx, cy, yaw, body))


def build_sdf(low_gfx=False):
    """low_gfx=True 면 그림자·보조조명을 끈 저사양 버전을 만든다.

    VMware / VirtualBox 처럼 3D 가속이 소프트웨어 폴백(llvmpipe)으로 도는
    환경에서는 ogre2 의 그림자맵 렌더링이 프레임 시간을 크게 잡아먹어
    화면 전체가 깜빡인다. 그림자를 끄는 것만으로 대부분 해결된다.
    """
    o = []
    o.append("<?xml version='1.0' ?>\n")
    o.append("<!-- 자동 생성 파일 - tools/generate_parking_lot.py 를 수정하고 재생성할 것 -->\n")
    if low_gfx:
        o.append("<!-- 저사양 버전: 그림자 OFF, 보조조명 OFF (VM / 소프트웨어 렌더링용) -->\n")
    o.append("<sdf version='1.10'>\n  <world name='parking_lot'>\n")
    head = """    <physics name='4ms' type='ignored'>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
    <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'/>
    <plugin filename='gz-sim-contact-system' name='gz::sim::systems::Contact'/>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.35 0.40 0.46 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen='0'>
      <plugin filename='MinimalScene' name='3D View'>
        <gz-gui><property key='state' type='string'>docked</property></gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.35 0.40 0.46</background_color>
        <camera_pose>0 -46 42 0 0.72 1.5708</camera_pose>
        <!-- 근평면을 키워 원거리 깊이버퍼 정밀도 확보 (노면표시 깜빡임 방지).
             기본 0.01 은 40m 상공 시점에서 z-fighting 을 유발한다. -->
        <camera_clip>
          <near>0.30</near>
          <far>400.0</far>
        </camera_clip>
      </plugin>
      <plugin filename='GzSceneManager' name='Scene Manager'/>
      <plugin filename='InteractiveViewControl' name='Interactive view control'/>
      <plugin filename='CameraTracking' name='Camera Tracking'/>
      <plugin filename='WorldControl' name='World control'>
        <gz-gui><property key='state' type='string'>floating</property></gz-gui>
        <use_event>true</use_event>
      </plugin>
      <plugin filename='WorldStats' name='World stats'>
        <gz-gui><property key='state' type='string'>floating</property></gz-gui>
      </plugin>
    </gui>

    <light type='directional' name='sun'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
"""
    if low_gfx:
        # 그림자맵이 소프트웨어 렌더링에서 가장 비싼 항목이라 우선 끈다.
        head = head.replace("<shadows>true</shadows>", "<shadows>false</shadows>")
        head = head.replace("<cast_shadows>true</cast_shadows>",
                            "<cast_shadows>false</cast_shadows>")
        # 태양광만으로 어두워지지 않게 환경광을 올린다.
        head = head.replace("<ambient>0.55 0.55 0.58 1</ambient>",
                            "<ambient>0.72 0.72 0.75 1</ambient>")
        head = head.replace("<ambient_light>0.5 0.5 0.5</ambient_light>",
                            "<ambient_light>0.7 0.7 0.7</ambient_light>")
    o.append(head)

    # 실내 조명 (통로 상부) — 저사양 버전에서는 생략 (픽셀당 조명 비용)
    lamps = [] if low_gfx else [(AISLE_S_C, AISLE_C_C, AISLE_N_C)]
    for i, cy in enumerate(lamps[0] if lamps else ()):
        for j, cx in enumerate((X_MIN + S(9.0), 0.0, X_MAX - S(9.0))):
            o.append("""    <light type='point' name='lamp_%d_%d'>
      <pose>%.3f %.3f %.3f 0 0 0</pose>
      <diffuse>0.55 0.55 0.50 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>%.2f</range><constant>0.4</constant>
        <linear>0.02</linear><quadratic>0.002</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>\n""" % (i, j, cx, cy, WALL_H - S(0.3), S(22.0)))

    # --- 바닥 ---
    o.append("    <model name='ground'>\n      <static>true</static>\n      <link name='link'>\n")
    o.append("        <collision name='c'><geometry><plane><normal>0 0 1</normal>"
             "<size>%.1f %.1f</size></plane></geometry>"
             "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"
             "</collision>\n" % (MAP_X1 - MAP_X0 + S(20), MAP_Y1 - MAP_Y0 + S(20)))
    o.append(box_vis("floor", 0, 0, -S(0.01), (X_MAX - X_MIN) + S(2.0),
                     (Y_MAX - Y_MIN) + S(2.0), S(0.02), "asphalt", collide=False))
    o.append("      </link>\n    </model>\n")

    # --- 벽 ---
    o.append("    <model name='perimeter_walls'>\n      <static>true</static>\n      <link name='link'>\n")
    for name, cx, cy, sx, sy, sz in WALLS:
        o.append(box_vis(name, cx, cy, sz / 2.0, sx, sy, sz, "wall"))
    o.append("      </link>\n    </model>\n")

    # --- 기둥 ---
    o.append("    <model name='pillars'>\n      <static>true</static>\n      <link name='link'>\n")
    for name, cx, cy, sx, sy, sz in PILLARS:
        o.append(box_vis(name, cx, cy, sz / 2.0, sx, sy, sz, "pillar"))
        # 기둥 하부 노란 경고 밴드
        o.append(box_vis(name + "_band", cx, cy, S(0.55), sx * 1.04, sy * 1.04,
                         S(0.25), "yellow", collide=False))
    o.append("      </link>\n    </model>\n")

    # --- 노면 표시 ---
    o.append("    <model name='road_markings'>\n      <static>true</static>\n      <link name='link'>\n")
    o.append(build_markings())
    o.append("      </link>\n    </model>\n")

    # --- 입/출구 게이트 (차단바 열림 상태) ---
    o.append("    <model name='gates'>\n      <static>true</static>\n      <link name='link'>\n")
    for tag, gx, gy in (("in", X_MIN + S(0.4), AISLE_S_C - GATE_W / 2.0 - S(0.5)),
                        ("out", X_MAX - S(0.4), AISLE_N_C + GATE_W / 2.0 + S(0.5))):
        o.append(box_vis("gpost_%s" % tag, gx, gy, S(0.60), S(0.30), S(0.30), S(1.20), "gate"))
        # 차단바 = 세워진 상태(통과 가능)
        o.append(box_vis("gbar_%s" % tag, gx, gy, S(1.80), S(0.12), S(0.12), S(1.10), "red",
                         collide=False))
    o.append("      </link>\n    </model>\n")

    # --- EV 충전기 ---
    o.append("    <model name='ev_chargers'>\n      <static>true</static>\n      <link name='link'>\n")
    for sid in sorted(EV_STALLS):
        s = SPOT_BY_ID[sid]
        cx, cy = s["center"]
        back_y = s["y1"] if s["entry_side"] == "S" else s["y0"]
        sgn = -1.0 if s["entry_side"] == "S" else 1.0
        py = back_y + sgn * S(0.35)
        o.append(box_vis("chg_%s" % sid, cx, py, S(0.65), S(0.28), S(0.28), S(1.30), "charger"))
        o.append(box_vis("chgtop_%s" % sid, cx, py, S(1.42), S(0.36), S(0.22), S(0.24), "white",
                         collide=False))
    o.append("      </link>\n    </model>\n")

    # --- 라바콘 (동적 장애물 데모) ---
    for i, (cx, cy, r, h) in enumerate(CONES):
        o.append("    <model name='cone_%02d'>\n      <static>true</static>\n"
                 "      <link name='link'>\n" % i)
        o.append(cyl_vis("cone", cx, cy, h / 2.0, r, h, "cone"))
        o.append(cyl_vis("base", cx, cy, S(0.02), r * 1.7, S(0.04), "cone"))
        o.append("      </link>\n    </model>\n")

    # --- 초기 주차 차량 ---
    palette = ["carbody", "wall", "red", "green", "blue"]
    for k, s in enumerate(sorted([x for x in SPOTS if x["occupied"]], key=lambda z: z["id"])):
        gx, gy, gyaw = s["goal_pose"]
        o.append(car_model("parked_car_%s" % s["id"], gx, gy, gyaw,
                           palette[k % len(palette)]))

    o.append("  </world>\n</sdf>\n")
    return "".join(o)


# ===========================================================================
# 5. Occupancy grid (PGM/YAML) 생성
# ===========================================================================
def new_grid(fill):
    return bytearray([fill]) * (MAP_W * MAP_H)


def _cols(x0, x1):
    c0 = max(0, int(math.floor((x0 - MAP_X0) / MAP_RES)))
    c1 = min(MAP_W, int(math.ceil((x1 - MAP_X0) / MAP_RES)))
    return c0, c1


def _rows(y0, y1):
    r0 = max(0, int(math.floor((MAP_Y1 - y1) / MAP_RES)))
    r1 = min(MAP_H, int(math.ceil((MAP_Y1 - y0) / MAP_RES)))
    return r0, r1


def fill_rect(g, x0, y0, x1, y1, val):
    c0, c1 = _cols(x0, x1)
    r0, r1 = _rows(y0, y1)
    if c1 <= c0 or r1 <= r0:
        return
    span = bytearray([val]) * (c1 - c0)
    for r in range(r0, r1):
        base = r * MAP_W
        g[base + c0:base + c1] = span


def fill_box(g, cx, cy, sx, sy, val):
    fill_rect(g, cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0, val)


def draw_static(g):
    """자유공간 + 벽 + 기둥."""
    fill_rect(g, X_MIN, Y_MIN, X_MAX, Y_MAX, FREE)
    # 게이트 밖 진입/진출 램프도 자유공간으로 (로컬라이제이션 여유)
    fill_rect(g, MAP_X0, AISLE_S_C - GATE_W / 2, X_MIN, AISLE_S_C + GATE_W / 2, FREE)
    fill_rect(g, X_MAX, AISLE_N_C - GATE_W / 2, MAP_X1, AISLE_N_C + GATE_W / 2, FREE)
    for _n, cx, cy, sx, sy, _sz in WALLS:
        fill_box(g, cx, cy, sx, sy, OCC)
    for _n, cx, cy, sx, sy, _sz in PILLARS:
        fill_box(g, cx, cy, sx, sy, OCC)
    for cx, cy, r, _h in CONES:
        fill_box(g, cx, cy, r * 2, r * 2, OCC)
    for sid in sorted(EV_STALLS):
        s = SPOT_BY_ID[sid]
        cx, _ = s["center"]
        back_y = s["y1"] if s["entry_side"] == "S" else s["y0"]
        sgn = -1.0 if s["entry_side"] == "S" else 1.0
        fill_box(g, cx, back_y + sgn * S(0.35), S(0.28), S(0.28), OCC)


def write_pgm(path, grid, comment):
    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(("# %s\n" % comment).encode("ascii", "replace"))
        f.write(("%d %d\n255\n" % (MAP_W, MAP_H)).encode("ascii"))
        f.write(bytes(grid))


def write_map_yaml(path, pgm_name, mode="trinary", negate=0,
                   occ_th=0.65, free_th=0.196):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("image: %s\n" % pgm_name)
        f.write("mode: %s\n" % mode)
        f.write("resolution: %.6f\n" % MAP_RES)
        f.write("origin: [%.4f, %.4f, 0.0]\n" % (MAP_X0, MAP_Y0))
        f.write("negate: %d\n" % negate)
        f.write("occupied_thresh: %.3f\n" % occ_th)
        f.write("free_thresh: %.3f\n" % free_th)


# ===========================================================================
# 6. 설정 파일 텍스트
# ===========================================================================
def spots_yaml():
    L = []
    L.append("# 자동 생성 - tools/generate_parking_lot.py")
    L.append("# 주차면 관리 노드(parking_spot_manager) ROS2 파라미터")
    L.append("#   goal_pose    : [x, y, yaw] 후진 주차 최종 포즈 (앞머리가 통로를 향함)")
    L.append("#   prepark_pose : [x, y, yaw] 후진 시작 대기 포즈 (통로 위, 주차면을 등짐)")
    L.append("#   aisle_point  : [x, y]      통로 중심선 경유점")
    L.append("parking_spot_manager:")
    L.append("  ros__parameters:")
    L.append("    frame_id: \"map\"")
    L.append("    lot_name: \"B1_parking_lot\"")
    L.append("    scale: %.3f" % SCALE)
    L.append("    stall_size: [%.3f, %.3f]" % (STALL_W, STALL_D))
    L.append("    reverse_distance: %.3f" % PREPARK_BACK)
    L.append("    entry_pose: [%.3f, %.3f, %.4f]" % START_POSE)
    L.append("    exit_pose: [%.3f, %.3f, %.4f]" % EXIT_POSE)
    L.append("    queue_pose: [%.3f, %.3f, %.4f]" % QUEUE_POSE)
    L.append("    spot_ids: [%s]" % ", ".join('"%s"' % s["id"] for s in PARKABLE))
    L.append("    spots:")
    for s in PARKABLE:
        gx, gy, gyaw = s["goal_pose"]
        px, py, pyaw = s["prepark_pose"]
        ax, ay = s["aisle_point"]
        cx, cy = s["center"]
        L.append("      %s:" % s["id"])
        L.append("        row: \"%s\"" % s["row"])
        L.append("        type: \"%s\"" % s["type"])
        L.append("        entry_side: \"%s\"" % s["entry_side"])
        L.append("        center: [%.3f, %.3f]" % (cx, cy))
        L.append("        goal_pose: [%.3f, %.3f, %.4f]" % (gx, gy, gyaw))
        L.append("        prepark_pose: [%.3f, %.3f, %.4f]" % (px, py, pyaw))
        L.append("        aisle_point: [%.3f, %.3f]" % (ax, ay))
        L.append("        initially_occupied: %s" % ("true" if s["occupied"] else "false"))
    return "\n".join(L) + "\n"


def spots_json():
    payload = {
        "lot_name": "B1_parking_lot",
        "frame_id": "map",
        "scale": SCALE,
        "bounds": {"x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX},
        "map": {"resolution": MAP_RES, "origin": [MAP_X0, MAP_Y0],
                "width_px": MAP_W, "height_px": MAP_H},
        "stall": {"width": STALL_W, "depth": STALL_D},
        "reverse_distance": PREPARK_BACK,
        "entry_pose": list(START_POSE),
        "exit_pose": list(EXIT_POSE),
        "aisles": {
            "south": {"center_y": AISLE_S_C, "width": AISLE_SIDE},
            "center": {"center_y": AISLE_C_C, "width": AISLE_CENTER},
            "north": {"center_y": AISLE_N_C, "width": AISLE_SIDE},
            "west": {"center_x": LANE_W_C, "width": LANE_W},
            "east": {"center_x": LANE_E_C, "width": LANE_W},
        },
        "robot_spec": {
            "length": ROBOT_L, "width": ROBOT_W,
            "wheelbase": ROBOT_WHEELBASE,
            "max_steer_rad": ROBOT_MAX_STEER,
            "min_turning_radius": ROBOT_MIN_R,
        },
        "spots": [
            {
                "id": s["id"], "row": s["row"], "index": s["index"],
                "type": s["type"], "entry_side": s["entry_side"],
                "rect": [s["x0"], s["y0"], s["x1"], s["y1"]],
                "center": list(s["center"]),
                "goal_pose": list(s["goal_pose"]),
                "prepark_pose": list(s["prepark_pose"]),
                "aisle_point": list(s["aisle_point"]),
                "initially_occupied": s["occupied"],
            } for s in PARKABLE
        ],
        "hatched_zones": [
            {"id": s["id"], "rect": [s["x0"], s["y0"], s["x1"], s["y1"]]}
            for s in SPOTS if s["type"] == "hatched"
        ],
        "pillars": [{"x": cx, "y": cy, "size": PILLAR}
                    for _n, cx, cy, _sx, _sy, _sz in PILLARS],
    }
    return json.dumps(payload, indent=2, ensure_ascii=False) + "\n"


NAV2_YAML = """# 자동 생성 - tools/generate_parking_lot.py
# Ackermann 차량형 로봇용 Nav2 파라미터 (ROS2 Jazzy / Nav2 1.3+)
#   전역: Smac Planner Hybrid-A*  (후진 포함 motion primitive)
#   지역: MPPI Controller         (AckermannConstraint)
# 최소회전반경 {MIN_R:.3f} m  = 축거 {WB:.3f} / tan({STEER_DEG:.0f} deg)

amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: "scan"
    laser_max_range: 30.0
    laser_min_range: 0.2
    max_particles: 3000
    min_particles: 800
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    update_min_a: 0.15
    update_min_d: 0.20
    resample_interval: 1
    transform_tolerance: 1.0
    set_initial_pose: true
    initial_pose:
      x: {START_X:.3f}
      y: {START_Y:.3f}
      z: 0.0
      yaw: {START_YAW:.4f}

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"

    # ! 기본 BT 를 쓰면 안 된다.
    #   Nav2 기본 트리는 복구행동에 Spin(제자리 회전)을 쓰는데, 차량형은
    #   제자리 회전이 불가능해서 behavior_plugins 에서 Spin 을 뺐다.
    #   그러면 기본 트리가 로드 단계에서 죽는다:
    #     "spin" action server not available after waiting for 1.00s
    #   Spin 자리를 BackUp / DriveOnHeading 으로 바꾼 트리를 쓴다.
    #   경로는 nav2_valet.launch.py 가 절대경로로 넘긴다.
    #   ($(find-pkg-share ...) 는 런치 파일에서만 치환되고 YAML 파라미터
    #    파일에서는 문자열 그대로 남는다 — 실측으로 확인)
    # ! plugin_lib_names 를 여기서 오버라이드하지 말 것.
    #   Nav2 Jazzy 는 표준 BT 노드 전체를 기본값으로 갖고 있는데, 일부만 적어
    #   덮어쓰면 bt_navigator 가 뜨다가 죽는다 (실측):
    #     FATAL: Failed to create navigator id navigate_to_pose.
    #            Exception: ID [ComputePathToPose] already registered
    #   커스텀 BT 노드(4주차 valet_* 4종)를 붙일 때는 Nav2 의 기본 목록을
    #   그대로 복사한 뒤 뒤에 추가해야 한다.

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.02
    min_theta_velocity_threshold: 0.02
    failure_tolerance: 0.5
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker", "parking_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.30
      movement_time_allowance: 20.0

    # 통로 주행용 — 여유 있는 허용오차
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.35
      yaw_goal_tolerance: 0.20
      stateful: true

    # 주차 정차용 — 정밀 허용오차 (계획서 5주차 정량지표)
    parking_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.12
      yaw_goal_tolerance: 0.06
      stateful: true

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # 전장 4.5 m 차량이 7 m 통로에서 90도 코너를 돌려면 예측 구간이
      # 차체 길이보다 충분히 길어야 한다. 60 x 0.05 = 3.0 s (최고속에서 4.8 m,
      # 차체 한 대 길이) 로는 코너를 못 읽고 밖으로 밀려난다.
      # 90 x 0.05 = 4.5 s (7.2 m) 로 늘린다.
      time_steps: 90
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.20
      vy_std: 0.0
      # ! wz_std 는 wz_max 보다 작아야 한다.
      #   wz_max 를 0.280 (= vx_max/R_min) 으로 낮췄는데 wz_std 가 0.30 이면
      #   샘플의 대부분이 실행 불가 영역으로 나가 경계로 클램프된다.
      #   결과적으로 조향이 bang-bang 이 되어 추종이 나빠진다
      #   (실측: 서통로 직선에서 경로 대비 1.6 m 이탈).
      #   wz_max 의 1/3 수준으로 잡는다.
      wz_std: 0.10
      vx_max: {VMAX:.2f}
      vx_min: -{VREV:.2f}          # 후진 허용 — 발렛파킹 핵심
      vy_max: 0.0
      wz_max: {WZMAX:.3f}
      # RTF 여유가 있어 2 회 반복으로 수렴을 개선한다 (추종 오차 감소)
      iteration_count: 2
      prune_distance: 6.0
      transform_tolerance: 0.2
      temperature: 0.3
      gamma: 0.015
      motion_model: "Ackermann"
      AckermannConstraints:
        min_turning_r: {MIN_R:.3f}
      visualize: true
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic",
                "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
                "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0        # 통로에서는 전진 선호. 주차 시엔 낮춘 프로파일 사용
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        # keepout 경계를 넘어서면 플래너가 "Start occupied" 로 재계획을
        # 거부한다. 경계 접근 자체를 강하게 억제한다. 3.81 -> 8.0
        cost_weight: 8.0
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
      PathAlignCritic:
        enabled: true
        cost_power: 1
        # 코너에서 경로 이탈이 커 주차행에 차체가 걸쳤다. 14 -> 22.
        cost_weight: 22.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: true
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 8.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 2                 # 2 = 전/후진 모두 허용

planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.30
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: false
      max_iterations: 1200000
      max_on_approach_iterations: 1200
      max_planning_time: 5.0
      motion_model_for_search: "REEDS_SHEPP"   # 전진+후진 (DUBIN은 전진만)
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: {MIN_R:.3f}
      reverse_penalty: 2.1        # 낮출수록 후진 적극 사용 (주차 프로파일 1.3)
      change_penalty: 0.15
      non_straight_penalty: 1.20
      # 통로 중앙 선호도. inflation_radius 를 외접원 이상으로 키워
      # 통로에 비용 기울기가 생겼으므로 이 값이 실제로 작동한다.
      cost_penalty: 4.0
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: true
      # ! 스무더는 최소회전반경을 보장하지 않는다. Smac 이 낸 원경로는
      #   REEDS_SHEPP motion primitive 라 3.5704 m 를 지키지만, 스무더가
      #   그걸 깨면 MPPI 가 물리적으로 못 따라가 코너에서 이탈한다.
      #   곡률 측정 결과에 따라 켜고 끈다 (path_check.py 가 재준다).
      #   끄면 경로가 격자 티가 나 MPPI 추종이 더 나빠질 수 있어 기본은 켠다.
      smooth_path: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    cycle_frequency: 10.0
    behavior_plugins: ["backup", "drive_on_heading", "wait", "assisted_teleop"]
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    # Ackermann 은 제자리 회전 불가 -> Spin 복구행동 사용 금지
    simulate_ahead_time: 2.0

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [{VMAX:.2f}, 0.0, {WZMAX:.3f}]
    min_velocity: [-{VREV:.2f}, 0.0, -{WZMAX:.3f}]
    max_accel: [1.2, 0.0, 2.0]
    max_decel: [-1.6, 0.0, -2.0]
    odom_topic: "odom"
    odom_duration: 0.1

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: {RES:.3f}
      track_unknown_space: true
      # 직사각 풋프린트 — 차량형이므로 원형 근사 금지
      footprint: "[ [{FP_FX:.3f}, {FP_HW:.3f}], [{FP_FX:.3f}, -{FP_HW:.3f}], [{FP_RX:.3f}, -{FP_HW:.3f}], [{FP_RX:.3f}, {FP_HW:.3f}] ]"
      footprint_padding: 0.03
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      filters: ["keepout_filter", "speed_filter"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 30.0
          obstacle_max_range: 25.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        # ! 풋프린트 4.6 x 2.0 -> 내접원 1.03 m / 외접원 2.55 m.
        #   외접원(2.55) 이상으로 잡아야 하는 이유가 두 가지 있다.
        #   1) Smac 이 코스트맵 포텐셜 필드로 충돌검사를 최적화한다.
        #      작으면 매 포즈마다 전체 풋프린트를 검사해 계획이 느려진다.
        #   2) 더 중요한 것 — 통로 폭이 7.6 m 인데 인플레이션이 1.10 이면
        #      통로 중앙부 비용이 전부 0 이라 "중앙을 타라"는 기울기가 없다.
        #      그래서 경로가 keepout 경계에 0.16 m 까지 붙어 나오고,
        #      MPPI 추종 오차 0.66 m 가 더해지면 차체가 주차행에 걸쳐
        #      플래너가 "Start occupied" 로 재계획을 거부한다. (실측)
        #   부풀려도 주차면이 막히지는 않는다. 내접원(1.03) 안쪽만 치명값이고
        #   그 바깥은 "비싼 셀"일 뿐이라 계획은 가능하다.
        cost_scaling_factor: 2.0
        inflation_radius: 2.60
      # 아래 두 필터는 filter 서버가 떠 있어야 동작한다.
      #   ros2 launch parking_lot_world nav2_valet.launch.py use_costmap_filters:=true
      # keepout: 주차면 내부를 진입금지로 만들어 통로 주행 중 가로지르기를 막는다.
      #          ParkManeuver 진입 직전에 런타임으로 꺼야 주차가 가능하다.
      #   ros2 param set /global_costmap/global_costmap keepout_filter.enabled false
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        # ! 런타임 토글(ros2 param set)은 실제로 안 먹는다 (실측).
        #   CostmapFilter 가 enabled 를 초기화 때 캐시하고 동적 파라미터
        #   콜백을 등록하지 않아서, param 값만 바뀌고 레이어 동작은 그대로다.
        #   -> 설정 파일에서 켜 둔다. 주차 기동에서 끄는 방법은 이슈 #7.
        #   (필터 서버가 안 떠 있으면 마스크를 못 받아 아무 영향 없다)
        enabled: true
        filter_info_topic: "/costmap_filter_info"
      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        # keepout 과 같은 이유로 설정에서 켠다 (런타임 토글 불가).
        # 마스크: 통로 직선 100% / 코너·게이트 50% / 주차면 앞 30%
        enabled: true
        filter_info_topic: "/speed_filter_info"
        speed_limit_topic: "/speed_limit"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 16
      height: 16
      resolution: {RES:.3f}
      footprint: "[ [{FP_FX:.3f}, {FP_HW:.3f}], [{FP_FX:.3f}, -{FP_HW:.3f}], [{FP_RX:.3f}, -{FP_HW:.3f}], [{FP_RX:.3f}, {FP_HW:.3f}] ]"
      footprint_padding: 0.03
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 20.0
          obstacle_max_range: 16.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        # 로컬도 global 과 같은 이유로 외접원 이상. 위 주석 참고.
        cost_scaling_factor: 2.5
        inflation_radius: 2.60

map_server:
  ros__parameters:
    yaml_filename: "parking_lot.yaml"

collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    transform_tolerance: 0.2
    source_timeout: 1.0
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.5
      simulation_time_step: 0.05
      min_points: 6
      visualize: false
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      enabled: true
"""


def nav2_yaml():
    fp_front = ROBOT_L * 0.5 + S(0.05)
    fp_rear = -(ROBOT_L * 0.5 + S(0.05))
    fp_hw = ROBOT_W * 0.5 + S(0.05)
    vmax = S(1.60)
    vrev = S(0.60)
    wzmax = vmax / ROBOT_MIN_R
    return NAV2_YAML.format(
        MIN_R=ROBOT_MIN_R, WB=ROBOT_WHEELBASE,
        STEER_DEG=math.degrees(ROBOT_MAX_STEER),
        START_X=START_POSE[0], START_Y=START_POSE[1], START_YAW=START_POSE[2],
        VMAX=vmax, VREV=vrev, WZMAX=wzmax, RES=MAP_RES,
        FP_FX=fp_front, FP_RX=fp_rear, FP_HW=fp_hw,
    )


FILTERS_YAML = """# 자동 생성 - tools/generate_parking_lot.py
# Nav2 Costmap Filters — keepout(진입금지) / speed(감속) 마스크 서버

# 1) 주차면 내부 진입 금지 마스크
#    통로 주행 중 플래너가 주차면을 가로지르지 않게 한다.
#    ParkManeuver 진입 시에는 keepout_filter.enabled=false 로 런타임 해제.
filter_mask_server:
  ros__parameters:
    frame_id: "map"
    topic_name: "/keepout_filter_mask"
    yaml_filename: "keepout_mask.yaml"

costmap_filter_info_server:
  ros__parameters:
    type: 0                       # 0 = keepout
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/keepout_filter_mask"
    base: 0.0
    multiplier: 1.0

# 2) 감속 마스크 (주차면 앞 / 게이트 / 코너)
speed_mask_server:
  ros__parameters:
    frame_id: "map"
    topic_name: "/speed_filter_mask"
    yaml_filename: "speed_mask.yaml"

speed_filter_info_server:
  ros__parameters:
    type: 1                       # 1 = speed limit (percent)
    filter_info_topic: "/speed_filter_info"
    mask_topic: "/speed_filter_mask"
    base: 0.0
    multiplier: 1.0
"""


# ===========================================================================
# 7. 파일 출력
# ===========================================================================
def w(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("  %-52s %8d B" % (os.path.relpath(path, ROOT), len(text.encode("utf-8"))))


def main(root):
    global ROOT
    ROOT = root
    print("주차장 맵 생성 (SCALE=%.2f)" % SCALE)
    print("  주차장 내부 : %.2f x %.2f m" % (X_MAX - X_MIN, Y_MAX - Y_MIN))
    print("  주차면      : %d 면 (주차가능 %d / 초기점유 %d / 빗금 %d)"
          % (len(SPOTS), len(PARKABLE),
             sum(1 for s in PARKABLE if s["occupied"]),
             sum(1 for s in SPOTS if s["type"] == "hatched")))
    print("  맵 그리드   : %d x %d px @ %.3f m/px" % (MAP_W, MAP_H, MAP_RES))
    print("  최소회전반경: %.3f m" % ROBOT_MIN_R)
    print()

    # --- SDF (일반 / 저사양 두 버전) ---
    w(os.path.join(root, "worlds", "parking_lot.sdf"), build_sdf())
    w(os.path.join(root, "worlds", "parking_lot_lowgfx.sdf"), build_sdf(low_gfx=True))

    # --- static map ---
    md = os.path.join(root, "maps")
    os.makedirs(md, exist_ok=True)

    g = new_grid(UNK)
    draw_static(g)
    write_pgm(os.path.join(md, "parking_lot.pgm"), g, "static infrastructure only")
    write_map_yaml(os.path.join(md, "parking_lot.yaml"), "parking_lot.pgm")
    print("  maps/parking_lot.pgm / .yaml")

    # --- occupied 참고맵 ---
    g2 = new_grid(UNK)
    draw_static(g2)
    for s in SPOTS:
        if s["occupied"]:
            cx, cy = s["center"]
            fill_box(g2, cx, cy, CAR_L, CAR_W, OCC) if abs(s["goal_pose"][2]) < 0.1 \
                else fill_box(g2, cx, cy, CAR_W, CAR_L, OCC)
    write_pgm(os.path.join(md, "parking_lot_occupied.pgm"), g2, "with initially parked cars")
    write_map_yaml(os.path.join(md, "parking_lot_occupied.yaml"), "parking_lot_occupied.pgm")
    print("  maps/parking_lot_occupied.pgm / .yaml")

    # --- keepout mask: 주차면 내부 + 빗금존 ---
    gk = new_grid(FREE)
    fill_rect(gk, MAP_X0, MAP_Y0, MAP_X1, MAP_Y1, FREE)
    for s in SPOTS:
        # 주차면 안쪽 80% 만 금지 (입구쪽 20% 는 진입 여유로 남김)
        d = (s["y1"] - s["y0"])
        if s["type"] == "hatched":
            fill_rect(gk, s["x0"], s["y0"], s["x1"], s["y1"], OCC)
        elif s["entry_side"] == "S":
            fill_rect(gk, s["x0"], s["y0"] + d * 0.20, s["x1"], s["y1"], OCC)
        else:
            fill_rect(gk, s["x0"], s["y0"], s["x1"], s["y1"] - d * 0.20, OCC)
    write_pgm(os.path.join(md, "keepout_mask.pgm"), gk, "stall interiors = keepout")
    write_map_yaml(os.path.join(md, "keepout_mask.yaml"), "keepout_mask.pgm")
    print("  maps/keepout_mask.pgm / .yaml")

    # --- speed mask: 흰색(255)=제한없음, 178=30%, 128=50% ---
    SLOW30, SLOW50 = 178, 128
    gs = new_grid(255)
    for row in ROWS:
        y0, y1 = row["y"]
        my = y0 if row["entry"] == "S" else y1
        sgn = -1.0 if row["entry"] == "S" else 1.0
        # 주차면 입구 앞 2.5 m 대역 = 30 %
        fill_rect(gs, BLOCK_X0, min(my, my + sgn * S(2.5)),
                  BLOCK_X0 + BLOCK_W, max(my, my + sgn * S(2.5)), SLOW30)
    # 게이트 주변 = 50 %
    fill_rect(gs, MAP_X0, AISLE_S_C - GATE_W, X_MIN + S(6.0), AISLE_S_C + GATE_W, SLOW50)
    fill_rect(gs, X_MAX - S(6.0), AISLE_N_C - GATE_W, MAP_X1, AISLE_N_C + GATE_W, SLOW50)
    # 좌우 연결차로 코너 = 50 %
    fill_rect(gs, X_MIN, Y_MIN, X_MIN + LANE_W, Y_MAX, SLOW50)
    fill_rect(gs, X_MAX - LANE_W, Y_MIN, X_MAX, Y_MAX, SLOW50)
    write_pgm(os.path.join(md, "speed_mask.pgm"), gs, "speed limit percent mask")
    write_map_yaml(os.path.join(md, "speed_mask.yaml"), "speed_mask.pgm", mode="scale")
    print("  maps/speed_mask.pgm / .yaml")

    # --- config ---
    w(os.path.join(root, "config", "parking_spots.yaml"), spots_yaml())
    w(os.path.join(root, "config", "parking_spots.json"), spots_json())
    w(os.path.join(root, "config", "nav2_ackermann.yaml"), nav2_yaml())
    w(os.path.join(root, "config", "costmap_filters.yaml"), FILTERS_YAML)

    print("\n완료.")


ROOT = "."

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
