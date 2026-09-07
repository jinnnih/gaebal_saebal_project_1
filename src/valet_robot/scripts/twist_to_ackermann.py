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
  R_min 은 **base_link 기준** 3.7829 m 다. 차량 물리 제원(뒤축 기준)은
  3.5704 지만 base_link 이 축거/2 앞이라 base_link 이 그리는 반경은
  hypot(3.5704, 1.25) = 3.7829 가 된다.

3차 역할 — **기준점 변환**:

  Nav2 는 base_link 기준으로 (v, wz) 를 낸다. 그런데 자전거 모델의 기준점은
  **뒤축**이고, ackermann_steering_controller 도 그 기준으로 해석한다
  (base_frame_id: base_rear_axle). base_link 은 축거 중점이라 뒤축보다
  1.25 m 앞이므로, 그대로 흘려보내면 차가 명령보다 빠르고 크게 돈다.

  실측으로 확인했다 (scripts/nav2_tests/turn_radius_test.py, 2026-09-07).
  정상 선회 중 base_link 의 슬립각(차체 방향과 진행 방향의 차):

      명령 v=1.0 wz=0.15 -> 실측 +9.69 deg  (뒤축 예측 +10.62, base_link 예측 0)
      명령 v=1.0 wz=0.25 -> 실측 +15.95 deg (뒤축 예측 +17.35, base_link 예측 0)

  그래서 뒤축 기준으로 바꿔서 내보낸다.

      v_rear = sqrt(v^2 - (wz * d)^2),   wz 는 그대로 (강체라 불변)

  이러면 base_link 의 속도가 정확히 v, 반경이 정확히 v/wz 가 된다.
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
        # ! base_link 기준 값이다. 차량 물리 제원(뒤축 기준)은 3.5704 지만
        #   base_link 이 축거/2 만큼 앞이라 base_link 이 그리는 최소 반경은
        #   hypot(3.5704, 1.25) = 3.7829 다. Nav2 는 base_link 로 명령한다.
        self.declare_parameter('min_turning_radius', 3.7829)
        # base_link 에서 뒤축까지 거리 (= 축거/2). 0 이면 변환을 끈다.
        self.declare_parameter('rear_axle_offset', 1.25)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        g = self.get_parameter
        self.in_topic = g('input_topic').value
        self.stamped = g('input_stamped').value
        self.out_topic = g('output_topic').value
        self.v_fwd = float(g('max_speed_forward').value)
        self.v_rev = float(g('max_speed_reverse').value)
        self.r_min = float(g('min_turning_radius').value)
        self.d_rear = float(g('rear_axle_offset').value)
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

        # r_min 은 base_link 기준이므로 이 제한도 base_link 기준이다.
        wz_max = abs(vx) / self.r_min
        wz = max(-wz_max, min(wz_max, wz))

        # base_link -> 뒤축 변환. 각속도는 강체라 그대로다.
        if self.d_rear > 0.0:
            inner = vx * vx - (wz * self.d_rear) ** 2
            # 위 제한 덕에 inner 는 항상 양수지만, 파라미터를 손대는 경우를
            # 대비해 막아 둔다.
            vx = math.copysign(math.sqrt(inner), vx) if inner > 0.0 else 0.0
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
        # 변환 뒤이므로 뒤축 기준 트위스트다
        out.header.frame_id = 'base_rear_axle'
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
