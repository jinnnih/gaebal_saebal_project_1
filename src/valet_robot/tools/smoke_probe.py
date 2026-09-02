#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""valet_car 동적 스모크 테스트 — 실제로 굴려보고 숫자로 확인한다.

전제: 월드 + 로봇 + 컨트롤러가 이미 떠 있을 것.
      (ros2 launch valet_robot valet_sim.launch.py gui:=false)

검사 항목
  A. /scan     수신 · 유효 range 비율 · 벽까지 거리
  B. TF        odom -> base_footprint
  C. 전진      vx=1.0 로 8 s, odom 이동거리
  D. 선회      vx=1.0 wz=0.28 (R=3.57 m) 로 12 s, 실제 회전반경
  E. Ackermann 좌/우 조향각이 서로 다른가 (내륜차)
  F. 후진      vx=-0.5 로 5 s, 뒤로 가는가
  G. 리미터    vx=0 wz=0.5 -> 안 움직여야 함 (제자리 회전 불가)
  H. 리미터    vx=1.0 wz=2.0 -> |w| 가 vx/3.5704 로 잘려야 함
"""
import math
import subprocess
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan

R_MIN = 3.5704
RESULTS = []


def rec(name, passed, detail):
    RESULTS.append((name, passed, detail))
    print('  [%s] %-28s %s' % ('ok  ' if passed else 'FAIL', name, detail))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):

    def __init__(self):
        super().__init__('valet_smoke_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.scan = None
        self.odom = None
        self.js = None
        self.steer_at_end = (None, None)
        self.create_subscription(LaserScan, '/scan', self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _scan(self, m):
        self.scan = m

    def _odom(self, m):
        self.odom = m

    def _js(self, m):
        self.js = m

    # ------------------------------------------------------------------
    def wait(self, cond, timeout=90.0, what=''):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
                print('    ! %s 대기 시간 초과 (%.0f s)' % (what, timeout))
                return False
        return False

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def drive(self, vx, wz, seconds):
        """sim time 기준으로 seconds 동안 명령하고 (시작포즈, 끝포즈) 반환."""
        self.wait(lambda: self.odom is not None, 30, 'odom')
        start = self.pose()
        t0 = self.get_clock().now()
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        while rclpy.ok():
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 >= seconds:
                break
        self.steer_at_end = self.steer_angles()   # 정지 전에 조향각을 잡아둔다
        stop = Twist()
        t0 = self.get_clock().now()
        while rclpy.ok():
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 >= 2.0:
                break
        return start, self.pose()

    def reset(self, x, y, yaw=0.0):
        """gz set_pose 로 로봇을 빈 통로로 되돌린다.

        주차장은 좁아서 연속 주행 시 주차 차량에 끼인다. 각 항목을 독립적으로
        재려면 매번 같은 자리에서 시작해야 한다.
        """
        import math as _m
        req = ('name: "valet_car", position: {x: %f, y: %f, z: 0.06}, '
               'orientation: {z: %f, w: %f}'
               % (x, y, _m.sin(yaw / 2.0), _m.cos(yaw / 2.0)))
        subprocess.run(
            ['gz', 'service', '-s', '/world/parking_lot/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds * 1e-9 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

    def steer_angles(self):
        if self.js is None:
            return None, None
        d = dict(zip(self.js.name, self.js.position))
        return (d.get('front_left_steer_joint'),
                d.get('front_right_steer_joint'))


def main():
    rclpy.init()
    n = Probe()
    print('\n===== valet_car 동적 스모크 테스트 =====')

    # ---- A. /scan ----
    # 이 VM 의 GL 스택(VMware SVGA3D) 에서는 gpu_lidar 프레임이 간헐적으로
    # 통째로 비어 나온다. 한 프레임만 보면 오판하므로 여러 프레임을 모아
    # "유효 프레임 비율"과 "유효 프레임의 최대 관측거리"로 판정한다.
    frames = []
    t0 = n.get_clock().now()
    while len(frames) < 40 and             (n.get_clock().now() - t0).nanoseconds * 1e-9 < 60.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.scan is not None and (not frames or n.scan is not frames[-1]):
            frames.append(n.scan)
    if frames:
        good = []
        for m in frames:
            f = [r for r in m.ranges if math.isfinite(r)]
            if f:
                good.append(f)
        ratio = len(good) / len(frames)
        far = max(max(f) for f in good) if good else 0.0
        beams = (sum(len(f) for f in good) / len(good)) if good else 0
        rec('A. /scan 수신', len(good) > 0 and far > 10.0,
            '%d 프레임 중 유효 %d (%.0f%%), 유효시 평균 %.0f 빔, 최대 %.1f m, '
            'frame=%s' % (len(frames), len(good), ratio * 100, beams, far,
                          frames[-1].header.frame_id))
        if ratio < 0.8:
            print('         ! 프레임 유실은 VMware SVGA3D 헤드리스 렌더링 문제다 '
                  '(모델 문제 아님)')
    else:
        rec('A. /scan 수신', False, '메시지 없음')

    # ---- B. TF ----
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    got = n.wait(lambda: buf.can_transform('odom', 'base_footprint',
                                           rclpy.time.Time()), 60, 'TF')
    if got:
        t = buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        rec('B. odom->base_footprint', True,
            'x=%.3f y=%.3f' % (t.transform.translation.x,
                               t.transform.translation.y))
    else:
        rec('B. odom->base_footprint', False, 'TF 없음')

    if not n.wait(lambda: n.odom is not None, 60, '/odom'):
        rec('C~H', False, '/odom 이 없어 주행 시험 불가')
        return report()

    # ---- C. 전진 ----
    n.reset(-12.0, 0.0, 0.0)
    a, b = n.drive(1.0, 0.0, 8.0)
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    dyaw = abs(math.degrees(b[2] - a[2]))
    rec('C. 전진 8 s @1.0 m/s', 5.0 < dist < 9.0,
        '이동 %.2f m (기대 5~9), 방향변화 %.1f deg' % (dist, dyaw))

    # ---- D. 선회 (최소 회전반경) ----
    # Aisle_C 는 폭 8 m (y -4~+4) 인데 최소회전반경 원의 지름이 7.6 m 라
    # 한 바퀴가 안 들어간다. 남쪽에서 시작해 90도만 돌고 끝낸다.
    n.reset(-12.0, -2.0, 0.0)
    a, b = n.drive(1.0, 1.0 / R_MIN, 7.0)
    fl, fr = n.steer_at_end          # 정지 전 값 (정지하면 0 으로 돌아간다)
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    dyaw = b[2] - a[2]
    while dyaw > math.pi:
        dyaw -= 2 * math.pi
    while dyaw < -math.pi:
        dyaw += 2 * math.pi
    if abs(dyaw) > 1e-3:
        # 현(chord) 과 회전각으로 반경 역산: chord = 2R sin(dyaw/2)
        radius = dist / (2.0 * abs(math.sin(dyaw / 2.0)))
    else:
        radius = float('inf')
    rec('D. 최소회전반경 선회', 2.8 < radius < 4.6,
        '실측 R=%.2f m (기대 ~%.2f), 회전 %.1f deg' % (radius, R_MIN,
                                                    math.degrees(dyaw)))

    # ---- E. Ackermann 내륜차 ----
    if fl is not None and fr is not None:
        diff = abs(fl - fr)
        rec('E. Ackermann 좌우 조향각차 (주행중)', diff > 0.05,
            '좌 %.2f deg / 우 %.2f deg / 차 %.2f deg'
            % (math.degrees(fl), math.degrees(fr), math.degrees(diff)))
    else:
        rec('E. Ackermann 좌우 조향각차', False, '/joint_states 에 조향 조인트 없음')

    # ---- F. 후진 ----
    n.reset(-12.0, 0.0, 0.0)
    a, b = n.drive(-0.5, 0.0, 5.0)
    fwd = ((b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2]))
    rec('F. 후진 5 s @-0.5 m/s', fwd < -1.2,
        '차체 전방 기준 %.2f m (음수여야 함)' % fwd)

    # ---- G. 제자리 회전 차단 ----
    n.reset(-12.0, 0.0, 0.0)
    a, b = n.drive(0.0, 0.5, 4.0)
    moved = math.hypot(b[0] - a[0], b[1] - a[1])
    turned = abs(math.degrees(b[2] - a[2]))
    rec('G. 제자리회전 차단', moved < 0.15 and turned < 3.0,
        '이동 %.3f m, 회전 %.2f deg (둘 다 ~0 이어야 함)' % (moved, turned))

    # ---- H. 각속도 클램프 ----
    n.reset(-12.0, -2.0, 0.0)
    a, b = n.drive(1.0, 2.0, 6.0)
    dyaw = b[2] - a[2]
    while dyaw > math.pi:
        dyaw -= 2 * math.pi
    while dyaw < -math.pi:
        dyaw += 2 * math.pi
    w = abs(dyaw) / 6.0
    limit = 1.0 / R_MIN
    rec('H. |w| <= vx/R_min 클램프', w < limit * 1.35,
        '실측 %.3f rad/s, 상한 %.3f (요청 2.0)' % (w, limit))

    return report()


def report():
    print('\n' + '=' * 58)
    bad = [r for r in RESULTS if not r[1]]
    for name, passed, detail in RESULTS:
        print('  %-4s %-28s %s' % ('ok' if passed else 'FAIL', name, detail))
    print('=' * 58)
    if bad:
        print('실패 %d / %d' % (len(bad), len(RESULTS)))
        return 1
    print('전체 통과 (%d 항목)' % len(RESULTS))
    return 0


if __name__ == '__main__':
    try:
        code = main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)
