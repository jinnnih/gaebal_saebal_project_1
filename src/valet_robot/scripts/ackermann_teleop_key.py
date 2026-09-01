#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차량형 로봇 키보드 수동주행 (계획서 1주차 "수동 조향 주행 확인"용).

diff-drive 용 teleop_twist_keyboard 는 각속도를 직접 주므로 차량형에는 감이
안 온다. 이 노드는 자동차처럼 **속도 + 조향각**을 조작하고, 내부에서
자전거 모델로 각속도를 만들어 ``/cmd_vel`` (geometry_msgs/Twist) 로 낸다.

    wz = v * tan(steer) / wheelbase

즉 명령 경로는 Nav2 와 완전히 동일하다:
    teleop --> /cmd_vel --> twist_to_ackermann --> 컨트롤러

키
    w / s     목표 속도  +/- 0.2 m/s   (전진 최대 1.60, 후진 최대 -0.60)
    a / d     조향각     +/- 3 deg     (최대 +/- 35 deg)
    e         조향각 중립
    space     즉시 정지 (속도 0, 조향 0)
    q         종료
"""
import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
=====================================================================
 valet_car 키보드 수동주행
   w / s   속도 +/- 0.2 m/s      a / d   조향 +/- 3 deg
   e       조향 중립             space   정지
   q       종료
=====================================================================
"""


class AckermannTeleop(Node):

    def __init__(self):
        super().__init__('ackermann_teleop_key')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheelbase', 2.50)
        self.declare_parameter('max_steer_deg', 35.0)
        self.declare_parameter('max_speed_forward', 1.60)
        self.declare_parameter('max_speed_reverse', 0.60)
        self.declare_parameter('speed_step', 0.2)
        self.declare_parameter('steer_step_deg', 3.0)

        g = self.get_parameter
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic').value, 10)
        self.L = float(g('wheelbase').value)
        self.max_steer = math.radians(float(g('max_steer_deg').value))
        self.v_fwd = float(g('max_speed_forward').value)
        self.v_rev = float(g('max_speed_reverse').value)
        self.dv = float(g('speed_step').value)
        self.dsteer = math.radians(float(g('steer_step_deg').value))

        self.speed = 0.0
        self.steer = 0.0
        self.create_timer(0.05, self._publish)

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.speed * math.tan(self.steer) / self.L
        self.pub.publish(msg)

    def status(self):
        r = (abs(self.L / math.tan(self.steer))
             if abs(self.steer) > 1e-4 else float('inf'))
        r_txt = '직진' if r == float('inf') else '{:6.2f} m'.format(r)
        return '\r 속도 {:+5.2f} m/s | 조향 {:+6.1f} deg | 회전반경 {} '.format(
            self.speed, math.degrees(self.steer), r_txt)

    def key(self, c):
        if c == 'w':
            self.speed = min(self.v_fwd, self.speed + self.dv)
        elif c == 's':
            self.speed = max(-self.v_rev, self.speed - self.dv)
        elif c == 'a':
            self.steer = min(self.max_steer, self.steer + self.dsteer)
        elif c == 'd':
            self.steer = max(-self.max_steer, self.steer - self.dsteer)
        elif c == 'e':
            self.steer = 0.0
        elif c == ' ':
            self.speed = 0.0
            self.steer = 0.0
        elif c == 'q':
            return False
        return True


def read_key(timeout=0.1):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main():
    if not sys.stdin.isatty():
        print('이 노드는 터미널에서 직접 실행해야 한다 (키 입력 필요).')
        return

    rclpy.init()
    node = AckermannTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            c = read_key(0.05)
            if c is not None and not node.key(c):
                break
            sys.stdout.write(node.status())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.speed = 0.0
        node.steer = 0.0
        node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(os.linesep + '정지 명령 발행 후 종료.')


if __name__ == '__main__':
    main()
