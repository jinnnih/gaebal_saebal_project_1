#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_vel -> ackermann_steering_controller/reference 변환 + 차량 기구학 제한.

ackermann_steering_controller 는 geometry_msgs/TwistStamped 를
``<controller>/reference`` 로 받는다. Nav2(velocity_smoother) 는 Twist 를
``/cmd_vel`` 계열로 낸다. 그 사이를 잇는 게 이 노드의 1차 역할이다.

2차 역할이 더 중요하다 — **차량형 기구학 제한**:

* 제자리 회전 불가.  |v| ~ 0 인데 wz 만 들어오면 조향해도 차는 안 돈다.
  그대로 흘려보내면 컨트롤러가 조향각만 꺾은 채 멈춰 있고 Nav2 는
  "회전 중"이라 오판한다. 여기서 wz 를 0 으로 죽인다.
* |wz| <= |v| / R_min.  최소회전반경보다 급한 요구는 잘라낸다.
  (R_min = wheelbase / tan(max_steer) = 2.50 / tan(35deg) = 3.5704 m)
* 전/후진 속도 상한 분리 (전진 1.60 / 후진 0.60 m/s).
* 워치독 — 입력이 timeout 동안 없으면 0 을 계속 발행해 정지 유지.

파라미터
  input_topic          기본 /cmd_vel
  input_stamped        입력이 TwistStamped 면 true
  output_topic         기본 /ackermann_steering_controller/reference
  max_speed_forward    1.60
  max_speed_reverse    0.60
  min_turning_radius   3.5704
  timeout              0.5
  publish_rate         50.0
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TwistToAckermann(Node):

    def __init__(self):
        super().__init__('twist_to_ackermann')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('input_stamped', False)
        self.declare_parameter('output_topic',
                               '/ackermann_steering_controller/reference')
        self.declare_parameter('max_speed_forward', 1.60)
        self.declare_parameter('max_speed_reverse', 0.60)
        self.declare_parameter('min_turning_radius', 3.5704)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        g = self.get_parameter
        self.in_topic = g('input_topic').value
        self.stamped = g('input_stamped').value
        self.out_topic = g('output_topic').value
        self.v_fwd = float(g('max_speed_forward').value)
        self.v_rev = float(g('max_speed_reverse').value)
        self.r_min = float(g('min_turning_radius').value)
        self.timeout = float(g('timeout').value)
        rate = float(g('publish_rate').value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(TwistStamped, self.out_topic, qos)
        if self.stamped:
            self.sub = self.create_subscription(
                TwistStamped, self.in_topic, self._on_stamped, qos)
        else:
            self.sub = self.create_subscription(
                Twist, self.in_topic, self._on_twist, qos)

        self._cmd = (0.0, 0.0)          # (vx, wz)
        self._last = None               # 마지막 수신 시각
        self._warned_spin = False
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            '[twist_to_ackermann] {} ({}) -> {}'.format(
                self.in_topic, 'TwistStamped' if self.stamped else 'Twist',
                self.out_topic))
        self.get_logger().info(
            '  제한: v [-{:.2f}, +{:.2f}] m/s,  R_min {:.3f} m '
            '(|wz| <= |v|/R_min),  제자리회전 금지'.format(
                self.v_rev, self.v_fwd, self.r_min))

    # ---------------- 콜백 ----------------
    def _on_twist(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)
        self._last = self.get_clock().now()

    def _on_stamped(self, msg):
        self._cmd = (msg.twist.linear.x, msg.twist.angular.z)
        self._last = self.get_clock().now()

    # ---------------- 기구학 제한 ----------------
    def _limit(self, vx, wz):
        vx = max(-self.v_rev, min(self.v_fwd, vx))

        if abs(vx) < 1.0e-3:
            # 차량형은 제자리 회전이 불가능하다. wz 만 남기면 조향만 꺾인 채
            # 정지하므로 아예 0 으로 만든다.
            if abs(wz) > 1.0e-3 and not self._warned_spin:
                self._warned_spin = True
                self.get_logger().warn(
                    '정지 상태에서 각속도 명령이 들어왔다 (제자리 회전 불가). '
                    'wz 를 0 으로 처리한다. Nav2 라면 behavior_plugins 에서 '
                    'Spin 이 빠졌는지 확인할 것.')
            return 0.0, 0.0

        wz_max = abs(vx) / self.r_min
        wz = max(-wz_max, min(wz_max, wz))
        return vx, wz

    # ---------------- 주기 발행 ----------------
    def _tick(self):
        now = self.get_clock().now()
        if self._last is None:
            vx, wz = 0.0, 0.0
        else:
            dt = (now - self._last).nanoseconds * 1e-9
            if dt > self.timeout:
                vx, wz = 0.0, 0.0
            else:
                vx, wz = self._limit(*self._cmd)

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = float(vx)
        out.twist.angular.z = float(wz)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
