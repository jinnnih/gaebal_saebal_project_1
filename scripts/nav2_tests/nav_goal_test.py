#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 로 목표 지점까지 자율주행시키고 결과를 잰다 (2주차 완료 기준 검증).

  python3 nav_goal_test.py  [x] [y] [yaw_deg]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

GX = float(sys.argv[1]) if len(sys.argv) > 1 else -10.0
GY = float(sys.argv[2]) if len(sys.argv) > 2 else -18.3
GYAW = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0


def main():
    rclpy.init()
    n = Node('nav_goal_test')
    n.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    amcl = {}
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                          lambda m: amcl.__setitem__('p', m), 10)

    print('AMCL 수렴 대기...')
    t0 = time.time()
    while 'p' not in amcl and time.time() - t0 < 40:
        rclpy.spin_once(n, timeout_sec=0.2)
    if 'p' in amcl:
        p = amcl['p'].pose.pose
        c = amcl['p'].pose.covariance
        print('  AMCL  x=%.2f y=%.2f  공분산 xx=%.4f yy=%.4f yaw=%.4f'
              % (p.position.x, p.position.y, c[0], c[7], c[35]))
    else:
        print('  ! /amcl_pose 없음 — 초기 포즈가 안 잡혔을 수 있다')

    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not ac.wait_for_server(timeout_sec=20.0):
        print('navigate_to_pose 액션 서버 없음'); return 1

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    goal.pose.pose.position.x = GX
    goal.pose.pose.position.y = GY
    goal.pose.pose.orientation.z = math.sin(GYAW / 2)
    goal.pose.pose.orientation.w = math.cos(GYAW / 2)
    print('목표 (%.2f, %.2f, %.0f deg) 전송' % (GX, GY, math.degrees(GYAW)))

    t_start = time.time()
    fut = ac.send_goal_async(goal, feedback_callback=lambda f: None)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('  목표 거부됨'); return 1
    print('  수락됨. 주행 중...')

    rf = gh.get_result_async()
    last = 0
    while rclpy.ok() and not rf.done():
        rclpy.spin_once(n, timeout_sec=0.3)
        el = time.time() - t_start
        if el - last >= 10:
            last = el
            if 'p' in amcl:
                p = amcl['p'].pose.pose
                d = math.hypot(GX - p.position.x, GY - p.position.y)
                print('  t=%4.0fs  위치 (%7.2f, %7.2f)  목표까지 %5.2f m'
                      % (el, p.position.x, p.position.y, d))
        if el > 180:
            print('  180 s 초과 — 취소'); ac._cancel_goal_async(gh); break

    el = time.time() - t_start
    res = rf.result() if rf.done() else None
    code = res.status if res else -1
    p = amcl.get('p')
    if p:
        pp = p.pose.pose
        err = math.hypot(GX - pp.position.x, GY - pp.position.y)
        print('\n  결과 status=%s  소요 %.0f s  최종 (%.2f, %.2f)  목표오차 %.2f m'
              % (code, el, pp.position.x, pp.position.y, err))
        print('  판정:', '도달' if err < 0.5 else '미도달')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
