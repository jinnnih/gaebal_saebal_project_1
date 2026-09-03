#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPPI 가 "Optimizer fail to compute path" 를 낼 때 무슨 상태였는지 계측한다.

로그에는 실패했다는 사실만 나오고 왜인지는 안 나온다. 그래서 매 주기마다
  - 차체 자세와 실제 속도 (/odom)
  - 컨트롤러가 낸 명령 (/cmd_vel)
  - 로컬 코스트맵에서 풋프린트가 밟고 있는 최대 비용
를 같이 찍어 둔다. 실패 직전 몇 초를 보면 원인이 드러난다.

  253 = 내접원 안쪽(치명 취급), 254 = 치명, 255 = 미지
  MPPI 의 CostCritic 은 consider_footprint=true 라 풋프린트 전체를 본다.

    python3 mppi_probe.py [초]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
# 풋프린트 (nav2_ackermann.yaml 과 같아야 한다)
FP = [(2.30, 1.00), (2.30, -1.00), (-2.30, -1.00), (-2.30, 1.00)]


class Probe(Node):
    def __init__(self):
        super().__init__('mppi_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.cm = None
        self.odom = None
        self.cmd = None
        self.cmd_t = 0.0
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self._cm, qos)
        self.create_subscription(Odometry, '/odom', self._od, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cv, 10)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def _cm(self, m):
        self.cm = m

    def _od(self, m):
        self.odom = m

    def _cv(self, m):
        self.cmd = m
        self.cmd_t = time.time()

    def foot_cost(self, x, y, yaw):
        """풋프린트 외곽선이 밟는 셀의 최대 비용과 그 위치."""
        if self.cm is None:
            return None, None
        md = self.cm.metadata
        res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
        w, h = md.size_x, md.size_y
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c) for p in FP]
        best, at = -1, None
        for i in range(len(pts)):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % len(pts)]
            n = max(2, int(math.dist((x0, y0), (x1, y1)) / res))
            for k in range(n + 1):
                px = x0 + (x1 - x0) * k / n
                py = y0 + (y1 - y0) * k / n
                cx = int((px - ox) / res)
                cy = int((py - oy) / res)
                v = 255 if not (0 <= cx < w and 0 <= cy < h) else \
                    self.cm.data[cy * w + cx]
                if v > best:
                    best, at = v, (px, py)
        return best, at


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    print('  t     x       y     yaw°   v[m/s]  w[r/s] | cmd v    w   | 풋프린트최대비용  위치')
    last = 0.0
    while rclpy.ok() and time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - last < 0.5 or n.odom is None:
            continue
        last = time.time()
        p = n.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        tw = n.odom.twist.twist
        cost, at = n.foot_cost(p.position.x, p.position.y, yaw)
        cv = n.cmd
        stale = (time.time() - n.cmd_t) > 1.0
        print('%5.1f %7.2f %7.2f %6.1f  %6.2f  %6.3f | %s | %s  %s'
              % (time.time() - t0, p.position.x, p.position.y,
                 math.degrees(yaw), tw.linear.x, tw.angular.z,
                 ('  --      -- ' if (cv is None or stale)
                  else '%5.2f %6.3f' % (cv.linear.x, cv.angular.z)),
                 ('  ??' if cost is None else '%4d' % cost),
                 ('' if at is None else '(%6.2f,%6.2f)' % at)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
