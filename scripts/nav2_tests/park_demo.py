#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주차 / 출차 액션 클라이언트 (CLI).

    python3 park_demo.py A08            주차
    python3 park_demo.py A08 unpark     출차

기동 로직은 전부 액션 서버에 있다 (valet_robot/scripts/park_action_server.py).
이 파일은 목표를 보내고 피드백·결과를 보기 좋게 찍는 것뿐이다.

! ros2 action send_goal (CLI) 로도 보낼 수 있지만 부하가 걸린 상태에서
  "rcl node's context is invalid" 로 실패하는 경우가 있다. 이 클라이언트는
  노드를 직접 만들어 붙어서 그 문제를 피한다.

    ros2 action send_goal /valet/park valet_robot/action/Park \
         "{spot_id: 'A08', mode: 0}"
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from valet_robot.action import Park

SPOT = sys.argv[1] if len(sys.argv) > 1 else 'A08'
MODE = Park.Goal.UNPARK if (len(sys.argv) > 2 and
                            sys.argv[2].startswith('un')) else Park.Goal.PARK


def main():
    rclpy.init()
    n = Node('park_client')
    n.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    ac = ActionClient(n, Park, '/valet/park')
    if not ac.wait_for_server(timeout_sec=25.0):
        print('액션 서버 없음 — ros2 run valet_robot park_action_server.py')
        rclpy.shutdown(); return 1

    g = Park.Goal()
    g.spot_id = SPOT
    g.mode = MODE
    print('===== %s  주차면 %s ====='
          % ('출차' if MODE == Park.Goal.UNPARK else '주차', SPOT))

    last = [0.0, '']

    def on_fb(msg):
        f = msg.feedback
        if f.phase != last[1] or time.time() - last[0] > 10:
            last[0], last[1] = time.time(), f.phase
            r = ('' if f.remaining_m < 0
                 else '  남은 %.1f m' % f.remaining_m)
            print('  [%-8s] (%6.2f,%6.2f, %+4.0f deg)%s'
                  % (f.phase, f.x, f.y, f.yaw_deg, r))

    fut = ac.send_goal_async(g, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('  목표 거부됨'); rclpy.shutdown(); return 1

    rf = gh.get_result_async()
    t0 = time.time()
    while rclpy.ok() and not rf.done() and time.time() - t0 < 600:
        rclpy.spin_once(n, timeout_sec=0.3)
    if not rf.done():
        print('  시간 초과'); gh.cancel_goal_async(); rclpy.shutdown(); return 1

    r = rf.result().result
    print()
    if not r.success:
        print('  실패: %s' % r.message)
    elif MODE == Park.Goal.PARK:
        print('  성공  위치오차 %.2f m,  방향오차 %+.1f deg' % (r.err_m, r.err_deg))
        print('  주차면 안: %s   전후진 전환 %d 회   소요 %.0f s'
              % ('예' if r.inside_rect else '아니오', r.shunts_park,
                 r.duration_s))
    else:
        print('  성공  전후진 전환 %d 회   소요 %.0f s'
              % (r.shunts_park, r.duration_s))
    rclpy.shutdown()
    return 0 if r.success else 1


if __name__ == '__main__':
    sys.exit(main())
