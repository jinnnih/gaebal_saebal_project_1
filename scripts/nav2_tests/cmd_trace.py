#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPPI 가 낸 명령과 twist_to_ackermann 통과 후 명령을 비교한다.

리미터(|wz| <= |vx|/R_min)가 MPPI 를 자르고 있으면 여기서 드러난다.
"""
import math, time
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

R_MIN = 3.5704


def main():
    rclpy.init()
    n = Node('cmd_trace')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    S = {'in': None, 'out': None}
    n.create_subscription(Twist, '/cmd_vel_smoothed', lambda m: S.__setitem__('in', m), 10)
    n.create_subscription(TwistStamped, '/ackermann_steering_controller/reference',
                          lambda m: S.__setitem__('out', m), 10)
    print(' t(s)   MPPI vx    wz   |  통과 vx    wz   | 요구R   클램프')
    t0 = time.time(); last = -1; clamped = 0; total = 0
    while time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
        i, o = S['in'], S['out']
        if i is None or o is None:
            continue
        total += 1
        want_r = abs(i.linear.x) / abs(i.angular.z) if abs(i.angular.z) > 1e-4 else float('inf')
        lim = abs(i.linear.x) / R_MIN
        is_clamped = abs(i.angular.z) > lim + 1e-3
        if is_clamped:
            clamped += 1
        el = time.time() - t0
        if el - last >= 3.0:
            last = el
            print(' %4.0f  %7.3f %7.3f  | %7.3f %7.3f  | %6.2f  %s'
                  % (el, i.linear.x, i.angular.z, o.twist.linear.x, o.twist.angular.z,
                     want_r, 'YES' if is_clamped else '-'))
    print('\n  표본 %d 개 중 클램프 %d 개 (%.0f%%)'
          % (total, clamped, 100.0 * clamped / max(1, total)))
    print('  클램프가 많으면 MPPI 가 최소회전반경보다 급한 선회를 요구하는 것이다.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
