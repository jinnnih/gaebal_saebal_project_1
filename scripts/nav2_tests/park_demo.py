#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주차 기동 데모 — 접근(Nav2) + 후진 진입(직선).

계획서의 ParkManeuver 를 눈으로 볼 수 있게 두 단계로 돌린다.

  1단계  Nav2 로 **통로를 따라** 접근점까지. 차는 통로 방향(동쪽)을 본다.
  2단계  ParkManeuver — 후진하며 90 도 선회 -> 직선 후진 -> 주차면 안.

! 1단계에서 prepark_pose(통로에 수직인 자세)를 Nav2 목표로 주면 안 된다.
  8 m 통로에서 90 도 자세를 잡으려면 여러 번 끊어야 하는데, Nav2 의 일반
  목표검사로는 300 s 를 헤매고도 못 맞춘다 (실측). 계획서가 ParkManeuver 를
  따로 둔 이유다. Nav2 는 통로까지만 데려오고, 주차면 진입은 기구학을 아는
  스크립트가 한다.

  접근점 = (주차면 x + R, 통로 y), 차는 통로 방향.
  거기서 R 반경으로 후진 선회하면 정확히 (주차면 x, 통로 y -+ R) 에
  주차면을 향해 선다. 그다음은 직선 후진이다.

! 2단계를 Nav2 로 못 하는 이유:
  keepout 필터가 주차면 내부를 진입금지로 막고 있어서 플래너가 주차면 안으로
  경로를 못 낸다. 런타임 토글은 실제로 안 먹는다 (이슈 #7 — CostmapFilter 가
  enabled 를 초기화 때 캐시하고 동적 파라미터 콜백을 등록하지 않는다).
  그래서 후진 진입은 cmd_vel 을 직접 내보내 실행한다. 주차면 진입은 원래
  직선 후진이라 플래너가 필요 없기도 하다.

    python3 park_demo.py [주차면ID]        예: park_demo.py A07
"""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

SPOT = sys.argv[1] if len(sys.argv) > 1 else 'A07'


def load_spots():
    try:
        share = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'parking_lot_world'],
            text=True).strip()
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', '..',
                             'src', 'parking_lot_world')
    with open(os.path.join(share, 'config', 'parking_spots.json'),
              encoding='utf-8') as f:
        return json.load(f)


class Park(Node):
    def __init__(self):
        super().__init__('park_demo')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = None
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom', m), 10)
        # ! 토픽을 잘못 고르면 명령이 아무 데도 안 간다.
        #   nav2:=true 면 twist_to_ackermann 의 입력이 /cmd_vel 이 아니라
        #   /cmd_vel_smoothed 다 (valet_sim.launch.py 가 그렇게 넘긴다).
        #   여기에 쏘려면 velocity_smoother 를 먼저 재워야 한다 — 그 노드가
        #   20 Hz 로 0 을 계속 발행하고 있어서 그냥 쏘면 경합한다.
        self.topic = os.environ.get('PARK_CMD_TOPIC', '/cmd_vel_smoothed')
        self.cmd = self.create_publisher(Twist, self.topic, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y ** 2 + q.z ** 2)))

    def wait_odom(self, sec=10.0):
        t = time.time()
        while self.odom is None and time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.odom is not None

    # ---------------- 1단계: 접근 ----------------
    def approach(self, x, y, yaw, limit=300.0):
        if not self.ac.wait_for_server(timeout_sec=20.0):
            print('  navigate_to_pose 서버 없음'); return False
        g = NavigateToPose.Goal()
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = x; ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        g.pose = ps
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  목표 거부됨'); return False
        rf = gh.get_result_async()
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and time.time() - t0 < limit:
            rclpy.spin_once(self, timeout_sec=0.3)
            if rf.done():
                break
            if time.time() - last > 15:
                last = time.time()
                c = self.pose()
                if c:
                    print('    t=%4ds  (%6.2f,%6.2f)  남은 %.1f m'
                          % (time.time() - t0, c[0], c[1],
                             math.dist(c[:2], (x, y))))
        if not rf.done():
            print('  시간 초과'); gh.cancel_goal_async(); return False
        st = rf.result().status
        c = self.pose()
        print('  접근 종료 status=%d  (%.2f, %.2f, %.0f deg)  오차 %.2f m'
              % (st, c[0], c[1], math.degrees(c[2]), math.dist(c[:2], (x, y))))
        return st == 4          # SUCCEEDED

    # ---------------- 1.5단계: 통로 방향 미세정렬 ----------------
    def align_x(self, target_x, tol=0.08, speed=0.20, limit=40.0):
        """통로를 따라 곧게 움직여 x 를 맞춘다.

        ! Nav2 의 목표 허용오차는 0.60 m 다. 통로 주행에는 충분하지만
          주차 기동에는 너무 크다. 선회 시작점이 0.5 m 어긋나면 선회 뒤에는
          0.9 m 가 되어 옆 주차면으로 밀린다 (실측). 차가 통로 방향을 보고
          있으므로 직선 전후진으로 x 를 정확히 맞출 수 있다.
        """
        tw = Twist()
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < limit:
            c = self.pose()
            if c is None:
                rclpy.spin_once(self, timeout_sec=0.05); continue
            e = target_x - c[0]
            if abs(e) < tol:
                break
            # 차체가 동쪽(+x)을 보므로 e>0 이면 전진
            v = speed if e > 0 else -speed
            if abs(e) < 0.35:
                v *= 0.5
            tw.linear.x = v; tw.angular.z = 0.0
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(20):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        return self.pose()

    # ---------------- 2단계: 후진 선회 ----------------
    def reverse_turn(self, target_yaw, ccw, radius, speed=0.25, limit=90.0):
        """목표 방향이 될 때까지 반경 radius 로 후진 선회한다.

        ! 열린 루프로 각도를 시간으로 계산하면 안 된다. 조향 지연과 슬립으로
          어긋난다. odom 의 yaw 를 보고 닫는다.
        """
        tw = Twist()
        tw.linear.x = -abs(speed)
        # 후진(v<0)에서 좌조향이면 시계방향이다. w = v*tan(d)/L 이므로
        # 원하는 회전 방향에 맞춰 부호를 준다.
        tw.angular.z = (1.0 if ccw else -1.0) * abs(speed) / radius
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < limit:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            c = self.pose()
            if c is None:
                continue
            d = math.atan2(math.sin(target_yaw - c[2]),
                           math.cos(target_yaw - c[2]))
            if abs(d) < math.radians(4.0):
                break
        for _ in range(20):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        return self.pose()

    # ---------------- 2단계: 직선 후진 ----------------
    def reverse(self, dist, speed=0.25):
        """차체 방향 기준으로 dist 만큼 곧게 후진한다."""
        start = self.pose()
        if start is None:
            return None
        tw = Twist(); tw.linear.x = -abs(speed); tw.angular.z = 0.0
        t0 = time.time()
        moved = 0.0
        while rclpy.ok() and time.time() - t0 < 60.0:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            c = self.pose()
            if c is None:
                continue
            moved = math.dist(c[:2], start[:2])
            if moved >= dist:
                break
        for _ in range(30):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        return moved


def lifecycle(node, action):
    """velocity_smoother 를 재우거나 깨운다. 실패해도 계속 진행한다."""
    try:
        subprocess.run(['ros2', 'lifecycle', 'set', '/velocity_smoother', action],
                       capture_output=True, timeout=25)
        return True
    except Exception:
        return False


def main():
    rclpy.init()
    n = Park()
    if not n.wait_odom():
        print('/odom 없음 — 시뮬이 안 돌고 있다'); rclpy.shutdown(); return 1
    D = load_spots()
    spot = next((s for s in D['spots'] if s['id'] == SPOT), None)
    if spot is None:
        print('주차면 %s 없음' % SPOT); rclpy.shutdown(); return 1

    gx, gy, gyaw = spot['goal_pose']
    ax, ay = spot['aisle_point']
    R = D['robot_spec'].get('min_turning_radius_base_link', 3.78)
    ccw = gyaw > 0.0                 # +90 도로 끝나면 반시계
    # ! 이상적인 90 도 선회라면 x 이동이 정확히 R 이어야 하는데, 실측은
    #   4.43 m 였다 (2026-09-07, 후진 0.25 m/s). 조향이 목표각에 도달하는
    #   동안 차가 더 나아가기 때문이다. 그대로 R 을 쓰면 선회가 주차면
    #   중심선에서 0.65 m 서쪽으로 끝나 옆 주차면을 친다.
    #   접근점을 그만큼 동쪽으로 밀어 보정한다.
    TURN_DX = float(os.environ.get('PARK_TURN_DX', 4.43))
    appr_x, appr_y = ax + TURN_DX, ay
    turn_end_y = ay + (-R if ccw else R)

    print('===== 주차면 %s (행 %s, 진입 %s) ====='
          % (spot['id'], spot['row'], spot['entry_side']))
    print('  통로점 (%.2f, %.2f)   목표 (%.2f, %.2f, %+.0f deg)'
          % (ax, ay, gx, gy, math.degrees(gyaw)))
    print('  접근점 (%.2f, %.2f, 0 deg)  선회 x이동 %.2f (실측 보정)'
          % (appr_x, appr_y, TURN_DX))
    print()
    print('[1/3] 접근 — Nav2 로 통로를 따라 접근점까지 (통로 방향 유지)')
    if not n.approach(appr_x, appr_y, 0.0):
        print('  접근 실패'); rclpy.shutdown(); return 1

    print()
    print('  Nav2 에서 제어권 인수 (velocity_smoother 정지) — 발행 %s' % n.topic)
    lifecycle(n, 'deactivate')
    time.sleep(1.5)

    print()
    print('[1.5/3] 미세정렬 — 통로를 따라 x 를 %.2f 로' % appr_x)
    c = n.align_x(appr_x)
    print('  정렬 후 (%.2f, %.2f, %+.0f deg)   x 오차 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), abs(c[0] - appr_x)))
    print()
    print('[2/3] 후진 선회 — 반경 %.2f m 로 %+.0f deg 까지' % (R, math.degrees(gyaw)))
    before = n.pose()
    c = n.reverse_turn(gyaw, ccw, R)
    print('  선회 후 (%.2f, %.2f, %+.0f deg)   주차면 중심선까지 x 오차 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), abs(c[0] - gx)))
    # ! 여기서 x 가 많이 어긋나 있으면 그대로 후진하면 옆 주차면을 친다.
    if abs(c[0] - gx) > 0.45:
        print('  ! x 오차가 크다 (%.2f m). 그대로 후진하면 옆 주차면을 친다.'
              % abs(c[0] - gx))
        print('    선회 시작점 (%.2f) 과 실제 (%.2f) 차이가 증폭된 것이다.'
              % (appr_x, before[0]))

    rest = abs(gy - c[1])
    print()
    print('[3/3] 직선 후진 %.2f m — 주차면 안으로' % rest)
    moved = n.reverse(rest)
    c = n.pose()
    err = math.dist(c[:2], (gx, gy))
    dyaw = math.degrees(math.atan2(math.sin(c[2] - gyaw),
                                   math.cos(c[2] - gyaw)))
    print('  후진 %.2f m  최종 (%.2f, %.2f, %+.0f deg)'
          % (moved, c[0], c[1], math.degrees(c[2])))
    print()
    print('  goal 대비  위치오차 %.2f m,  방향오차 %+.1f deg' % (err, dyaw))
    x0, y0, x1, y1 = spot['rect']
    inside = x0 <= c[0] <= x1 and y0 <= c[1] <= y1
    print('  주차면 안에 있는가: %s  (rect x %.2f~%.2f, y %.2f~%.2f)'
          % ('예' if inside else '아니오', x0, x1, y0, y1))
    print()
    print('  Nav2 에 제어권 반납 (velocity_smoother 재개)')
    lifecycle(n, 'activate')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
