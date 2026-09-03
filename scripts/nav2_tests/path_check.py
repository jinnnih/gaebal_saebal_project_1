#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""플래너에 경로를 요청해 주차면을 가로지르는지 검사한다."""
import io, json, math, os, subprocess, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# ! 기하는 하드코딩하지 않는다. 통로 폭을 바꾸면 y 좌표가 전부 이동해서
#   옛 값이 남아 있으면 "주차행 통과" 판정이 조용히 틀린다 (실제로 겪음).


def _spots():
    try:
        share = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'parking_lot_world'],
            text=True).strip()
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', '..',
                             'src', 'parking_lot_world')
    with io.open(os.path.join(share, 'config', 'parking_spots.json'),
                 encoding='utf-8') as f:
        return json.load(f)


_D = _spots()
_rows = {}
for _s in _D['spots']:
    x0, y0, x1, y1 = _s['rect']
    r = _rows.setdefault(_s['row'], [y0, y1, x0, x1])
    r[0] = min(r[0], y0); r[1] = max(r[1], y1)
    r[2] = min(r[2], x0); r[3] = max(r[3], x1)
ROWS = [(k, v[0], v[1]) for k, v in sorted(_rows.items())]
BLK_X0 = min(v[2] for v in _rows.values())
BLK_X1 = max(v[3] for v in _rows.values())
# 주차면 경계에서 풋프린트 반폭 + 여유를 뺀, 차체 중심의 안전 상한
_HW = _D['robot_spec']['width'] / 2.0
SAFE_X = BLK_X0 - _HW - 0.5


def in_row_y(y):
    """그 y 높이에 실제로 주차면이 있는가. 통로 구간이면 False."""
    return any(y0 <= y <= y1 for _, y0, y1 in ROWS)

SX, SY, SYAW = _D['entry_pose']
# ! 시작 yaw 를 빼먹으면 안 된다. 입구가 45 deg 로 바뀌었는데 0 으로
#   계획을 요청하면 실제로 못 따라가는 경로가 나온다.
MIN_R = _D['robot_spec']['min_turning_radius']
GX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
GY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0


def main():
    rclpy.init()
    n = Node('path_check')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=20):
        print('compute_path_to_pose 서버 없음'); return 1
    g = ComputePathToPose.Goal()
    g.use_start = True
    for p, (x, y, th) in (('start', (SX, SY, SYAW)), ('goal', (GX, GY, 0.0))):
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = x; ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(th / 2.0)
        ps.pose.orientation.w = math.cos(th / 2.0)
        setattr(g, p, ps)
    t0 = time.time()
    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('거부됨'); return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=60)
    res = rf.result()
    if res is None or not res.result.path.poses:
        print('경로 없음 (status=%s)' % (res.status if res else '?')); return 1
    pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
    L = sum(math.dist(pts[i], pts[i+1]) for i in range(len(pts)-1))
    print('경로 %d 점, 길이 %.1f m, 계획시간 %.1f s' % (len(pts), L, time.time()-t0))
    # 곡률 반경: 약 1 m 간격 3점의 외접원. 인접점(0.25 m)으로 재면
    # 격자·각도 양자화 잡음 때문에 말도 안 되는 작은 값이 나온다.
    step = max(1, int(1.0 / max(1e-6, L / max(1, len(pts) - 1))))
    radii = []
    for i in range(step, len(pts) - step):
        (x1, y1), (x2, y2), (x3, y3) = pts[i - step], pts[i], pts[i + step]
        a = math.dist(pts[i - step], pts[i]); b = math.dist(pts[i], pts[i + step])
        c = math.dist(pts[i - step], pts[i + step])
        area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
        if area2 > 1e-6 and a * b * c > 1e-9:
            radii.append(a * b * c / (2 * area2))
    if radii:
        radii.sort()
        # ! 5% 여유를 둔다. 딱 R_min 으로 세면 3.5699 같은 부동소수 잡음이
        #   전부 "위반" 으로 잡혀 멀쩡한 경로에 경고가 뜬다.
        lim = MIN_R * 0.95
        tight = [r for r in radii if r < lim]
        print('  곡률(약 %.2f m 간격): 최소 %.2f m, 5%%분위 %.2f m, '
              '%.2f 미만 %d/%d'
              % (step * L / max(1, len(pts) - 1), radii[0],
                 radii[len(radii) // 20], lim, len(tight), len(radii)))
        if len(tight) > len(radii) * 0.1:
            print('    !! 최소회전반경을 깨는 구간이 많다 — 차량이 못 따라간다')
            print('       (전진<->후진 전환점은 곡률이 무의미하게 작게 나온다.'
                  ' REEDS_SHEPP 은 전환을 허용하므로 몇 점은 정상)')

    bad = {}
    for x, y in pts:
        for name, y0, y1 in ROWS:
            if y0 < y < y1 and BLK_X0 <= x <= BLK_X1:
                bad[name] = bad.get(name, 0) + 1
    if bad:
        print('  !! 주차행 통과: %s' % ', '.join('%s행 %d점' % (k, v) for k, v in sorted(bad.items())))
    else:
        print('  통로만 사용 (주차행 통과 없음)')
    # 서통로 구간(x < -18)에서 keepout 경계(x=-17.5)까지 여유
    # ! 주차면이 있는 y 높이만 본다. 중앙통로 구간까지 포함하면
    #   거기엔 막을 주차면이 없는데도 '경계 침범' 으로 잘못 잡힌다.
    w = [p for p in pts if p[0] < BLK_X0 and in_row_y(p[1])]
    if w:
        east = max(p[0] for p in w)
        print('  서통로 구간 최동단 x = %.2f  (안전 상한 %.2f, 여유 %.2f m)'
              % (east, SAFE_X, SAFE_X - east))
    print('  경로 샘플:', ' -> '.join('(%.1f,%.1f)' % p for p in pts[::max(1, len(pts)//8)]))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
