#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""보여주기용 연속 주행 데모.

중앙 통로(Aisle_C, 폭 8 m)에서 최소회전반경으로 계속 선회한다.
Ackermann 조향(좌우 바퀴 각도가 다름)과 최소회전반경이 눈으로 보인다.
일정 시간마다 시작 위치로 리셋해서 벽에 안 닿게 한다.

  ros2 run 없이:  python3 demo_drive.py
  중지: Ctrl-C (정지 명령 보내고 종료)
"""
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

R_MIN = 3.5704


def reset(x=0.0, y=0.0, yaw=0.0):
    req = ('name: "valet_car", position: {x: %f, y: %f, z: 0.06}, '
           'orientation: {z: %f, w: %f}'
           % (x, y, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
    subprocess.run(['gz', 'service', '-s', '/world/parking_lot/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000', '--req', req],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    rclpy.init()
    n = Node('valet_demo_drive')
    n.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    # (설명, vx, wz, 지속시간[s])
    seq = [
        ('직진 (전진 1.0 m/s)',            1.0,  0.0,          6.0),
        ('좌선회 (최소회전반경 3.57 m)',    1.0,  1.0 / R_MIN, 22.0),
        ('직진',                           1.0,  0.0,          4.0),
        ('우선회 (최소회전반경 3.57 m)',    1.0, -1.0 / R_MIN, 22.0),
        ('정지',                           0.0,  0.0,          2.0),
        ('후진 (0.6 m/s)',                -0.6,  0.0,          6.0),
        ('정지',                           0.0,  0.0,          2.0),
    ]

    print('데모 시작 — Gazebo 창에서 로봇을 보세요. Ctrl-C 로 중지.')
    try:
        while rclpy.ok():
            reset(-12.0, 0.0, 0.0)
            print('  [리셋] 중앙 통로 (-12.0, 0.0) 로 이동')
            t0 = time.time()
            while time.time() - t0 < 2.0:
                pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
            for label, vx, wz, dur in seq:
                print('  %-32s vx=%+.2f  wz=%+.3f  (%.0f s)' % (label, vx, wz, dur))
                m = Twist(); m.linear.x = vx; m.angular.z = wz
                t0 = time.time()
                while time.time() - t0 < dur and rclpy.ok():
                    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        t0 = time.time()
        while time.time() - t0 < 1.0:
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('정지 명령 발행 후 종료')


if __name__ == '__main__':
    main()
