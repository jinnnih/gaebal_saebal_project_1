#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정상 선회를 시켜 base_link 의 실제 회전반경을 잰다.

왜 필요한가.

  Nav2 는 base_link 기준으로 (v, w) 를 낸다. 그런데 MPPI 의 Ackermann 모델도
  ackermann_steering_controller 도 자전거 모델이고, 그 모델의 기준점은
  **뒤축**이다. base_link 은 축거 중점이라 뒤축보다 축거/2 = 1.25 m 앞이다.

  기준점이 어긋나 있으면 같은 (v, w) 에 대해 실제 반경이 달라진다.

    기준이 base_link 이면   R_meas = v / w
    기준이 뒤축이면          R_meas = hypot(v / w, 1.25)

  두 예측이 3~6 % 차이라 원 적합으로 구분할 수 있다.

    python3 turn_radius_test.py [v] [w] [초]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

V = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
W = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
HALF_WB = 1.25          # base_link -> 뒤축


def slip_angle(samples):
    """차체 방향과 실제 진행 방향의 차 (슬립각) 를 잰다.

    ! 원 적합은 못 쓴다. 반경 4~7 m 원이 통로(폭 10.5 m)에 안 들어가서
      차가 주차행에 부딪힌다. 슬립각은 짧은 호로도 재진다.

      기준이 base_link 이면  beta ~ 0
      기준이 뒤축이면        beta = atan(w * 1.25 / v)
    """
    out = []
    for k in range(1, len(samples)):
        (x0, y0, th0, t0) = samples[k - 1]
        (x1, y1, th1, t1) = samples[k]
        dt = t1 - t0
        if dt < 1e-3:
            continue
        dx, dy = x1 - x0, y1 - y0
        if math.hypot(dx, dy) < 1e-3:
            continue
        course = math.atan2(dy, dx)
        th = math.atan2(math.sin((th0 + th1) / 2), math.cos((th0 + th1) / 2))
        b = math.atan2(math.sin(course - th), math.cos(course - th))
        out.append(b)
    return out


def main():
    rclpy.init()
    n = rclpy.create_node('turn_radius_test')
    n.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    odom = []
    n.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 20)
    tw = Twist(); tw.linear.x = V; tw.angular.z = W

    t0 = time.time()
    samples = []
    settle = 3.0
    while rclpy.ok() and time.time() - t0 < DUR:
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
        if odom and time.time() - t0 > settle:
            m = odom[-1]
            p = m.pose.pose.position; q = m.pose.pose.orientation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y ** 2 + q.z ** 2))
            if not samples or math.dist((p.x, p.y),
                                        (samples[-1][0], samples[-1][1])) > 0.05:
                samples.append((p.x, p.y, th, time.time()))
    for _ in range(20):
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)

    print('명령  v=%.2f m/s  w=%.3f rad/s   표본 %d' % (V, W, len(samples)))
    if len(samples) < 20:
        print('  표본 부족'); rclpy.shutdown(); return
    bs = slip_angle(samples)
    bs.sort()
    med = bs[len(bs) // 2]
    pred_rear = math.atan2(W * HALF_WB, V)
    print('  슬립각 중앙값   %+6.2f deg  (뒤축 기준이면 %+.2f, 물리적으로 남는다)'
          % (math.degrees(med), math.degrees(pred_rear)))

    # ! 변환이 제대로 됐는지는 슬립각이 아니라 **base_link 의 속도**로 본다.
    #   슬립각은 base_link 이 뒤축 앞에 있는 한 선회 중 물리적으로 생긴다.
    #   변환이 고치는 것은 base_link 의 속도와 반경이다.
    #     변환 전:  |v_base| = hypot(V, W*d)  -> 명령보다 빠르다
    #     변환 후:  |v_base| = V              -> 명령과 같다
    sp = []
    for k in range(1, len(samples)):
        (x0, y0, _, t0) = samples[k - 1]
        (x1, y1, _, t1) = samples[k]
        if t1 - t0 > 1e-3:
            sp.append(math.dist((x0, y0), (x1, y1)) / (t1 - t0))
    sp.sort()
    vmed = sp[len(sp) // 2]
    before = math.hypot(V, W * HALF_WB)
    print('  base_link 실측 속도  %.4f m/s' % vmed)
    print('    변환 전 예측       %.4f  (오차 %+.1f %%)'
          % (before, 100.0 * (vmed - before) / before))
    print('    변환 후 예측       %.4f  (오차 %+.1f %%)'
          % (V, 100.0 * (vmed - V) / V))
    print('  -> %s'
          % ('변환 적용됨' if abs(vmed - V) < abs(vmed - before) else '변환 미적용'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
