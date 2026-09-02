#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""보여주기용 연속 주행 데모.

중앙 통로(Aisle_C, 폭 8 m)를 슬라롬으로 왕복한다.
Ackermann 조향(좌우 바퀴 각도가 다름), 최소회전반경, 후진 조향이 눈으로 보인다.
한 사이클이 끝나면 시작 위치로 리셋해서 주차 차량에 안 닿게 한다.

  ros2 run valet_robot demo_drive.py
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
    #
    # ! 원 궤적을 그리면 안 된다. 최소회전반경 3.5 m 원의 지름이 7 m 인데
    #   중앙 통로(Aisle_C) 폭이 8 m 라, 한 바퀴 돌면 주차면으로 들어가
    #   주차 차량에 부딪힌다. 좌우 번갈아 꺾는 슬라롬으로 통로 안에 머문다.
    #   60도 선회의 횡방향 이동량 = R(1-cos60) = 1.75 m.
    W = 1.0 / R_MIN            # 최소회전반경 각속도
    seq = [
        ('직진 (전진 1.0 m/s)',              1.0,  0.0,  5.0),
        ('좌선회 60도 (최소회전반경 3.5 m)',  1.0,  W,    3.7),
        ('우선회 120도',                     1.0, -W,    7.3),
        ('좌선회 60도 (직진 복귀)',           1.0,  W,    3.7),
        ('직진',                             1.0,  0.0,  3.0),
        ('정지',                             0.0,  0.0,  1.5),
        ('후진 (0.6 m/s)',                  -0.6,  0.0,  6.0),
        ('후진하며 우선회',                  -0.6, -W*0.6, 5.0),
        ('정지',                             0.0,  0.0,  2.0),
    ]

    print('데모 시작 — Gazebo 창에서 로봇을 보세요. Ctrl-C 로 중지.')
    try:
        while rclpy.ok():
            reset(-16.0, 0.0, 0.0)
            print('  [리셋] 중앙 통로 서쪽 (-16.0, 0.0) 로 이동')
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
