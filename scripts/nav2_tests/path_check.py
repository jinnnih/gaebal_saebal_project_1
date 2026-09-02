#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""플래너에 경로를 요청해 주차면을 가로지르는지 검사한다."""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# 주차 행 y 범위 (parking_lot_world README)
ROWS = [('A', -14.8, -9.4), ('B', -9.4, -4.0), ('C', 4.0, 9.4), ('D', 9.4, 14.8)]

SX, SY = -23.0, -18.3
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
    for p, (x, y) in (('start', (SX, SY)), ('goal', (GX, GY))):
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.orientation.w = 1.0
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
        tight = [r for r in radii if r < 3.5704]
        print('  곡률(약 %.2f m 간격): 최소 %.2f m, 5%%분위 %.2f m, '
              '3.57 미만 %d/%d'
              % (step * L / max(1, len(pts) - 1), radii[0],
                 radii[len(radii) // 20], len(tight), len(radii)))
        if len(tight) > len(radii) * 0.1:
            print('    !! 최소회전반경을 깨는 구간이 많다 — 차량이 못 따라간다')

    bad = {}
    for x, y in pts:
        for name, y0, y1 in ROWS:
            if y0 < y < y1 and -17.5 <= x <= 17.5:
                bad[name] = bad.get(name, 0) + 1
    if bad:
        print('  !! 주차행 통과: %s' % ', '.join('%s행 %d점' % (k, v) for k, v in sorted(bad.items())))
    else:
        print('  통로만 사용 (주차행 통과 없음)')
    # 서통로 구간(x < -18)에서 keepout 경계(x=-17.5)까지 여유
    w = [p for p in pts if p[0] < -18.0 and -16 < p[1] < 4]
    if w:
        east = max(p[0] for p in w)
        print('  서통로 구간 최동단 x = %.2f  (안전 상한 -19.56, 여유 %.2f m)'
              % (east, -19.56 - east))
    print('  경로 샘플:', ' -> '.join('(%.1f,%.1f)' % p for p in pts[::max(1, len(pts)//8)]))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
