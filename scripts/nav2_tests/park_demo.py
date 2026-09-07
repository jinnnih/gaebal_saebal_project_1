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

    python3 park_demo.py A08            주차
    python3 park_demo.py A08 unpark     출차
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
MODE = sys.argv[2] if len(sys.argv) > 2 else 'park'   # park | unpark


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

    # ---------------- 출차: 전진 탈출 ----------------
    def forward_hold(self, gx, gy, gyaw, dist, speed=0.22,
                     k_e=0.35, k_s=0.8, r_min=3.7829, limit=60.0):
        """방향을 유지하며 주차면 밖으로 곧게 전진한다.

        ! 전진은 후진과 부호가 반대다. 차가 헤딩 방향으로 가므로
              de/ds = +sin(phi)
          이라 phi_cmd = -k_e * e 로 두어야 수렴한다. 후진용 부호를 그대로
          쓰면 발산한다.
        """
        lx, ly = -math.sin(gyaw), math.cos(gyaw)
        start = self.pose()
        tw = Twist()
        t0 = time.time()
        moved = 0.0
        while rclpy.ok() and time.time() - t0 < limit:
            c = self.pose()
            if c is None:
                rclpy.spin_once(self, timeout_sec=0.05); continue
            moved = math.dist(c[:2], start[:2])
            if moved >= dist:
                break
            e = (c[0] - gx) * lx + (c[1] - gy) * ly
            phi = -max(-0.20, min(0.20, k_e * e))       # 전진이라 부호 반대
            psi = math.atan2(math.sin(gyaw + phi - c[2]),
                             math.cos(gyaw + phi - c[2]))
            v = abs(speed)
            w_max = v / r_min
            tw.linear.x = v
            tw.angular.z = max(-w_max, min(w_max, v * k_s * psi))
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(20):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        return moved

    def forward_turn(self, target_yaw, ccw, radius, speed=0.22, limit=90.0):
        """목표 방향까지 전진 선회한다 (주차의 후진 선회와 대칭)."""
        tw = Twist()
        tw.linear.x = abs(speed)
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

    # ---------------- 3단계: 중심선 추종 후진 ----------------
    def reverse_track(self, gx, gy, gyaw, speed=0.22, limit=90.0,
                      k_e=0.35, k_s=0.8, r_min=3.7829, taper=2.0):
        """주차면 중심선을 따라가며 후진한다.

        ! 눈 감고 곧게 후진하면 선회 후 남은 횡오차가 그대로 남는다.
          실측 0.54 m 중 대부분이 이것이었다. 후진하는 4 m 동안 오차를
          줄일 수 있는데 안 쓰고 있었다.

        제어칙 (s = 후진 거리, phi = 목표 방향에서의 이탈각):

            차는 헤딩 반대로 가므로  d(pos)/ds = -(cos th, sin th)
            횡오차를 e = (pos - goal) . (-sin gyaw, cos gyaw) 로 두면
            de/ds = sin(gyaw - th) = -sin(phi) ~= -phi

            phi_cmd = k_e * e  로 두면  de/ds = -k_e * e  -> 지수 수렴.
            (전진이면 부호가 반대라 발산한다. 후진 전용이다)

        ! 종단 정렬이 필요하다. phi_cmd 를 끝까지 살려 두면 e 가 0 이 아닌
          채로 주차면에 들어가 차가 기울어진다. 실측으로 방향오차 -33.7 deg
          가 났다. 남은 거리가 taper 이내면 phi_cmd 를 0 으로 줄여 차를
          중심선에 나란히 세운다.

        ! k_e 는 0.35 다. 0.60 으로 올려 봤다가 되돌렸다. 방향 게인을
          고친 뒤라 위치 게인은 올려도 될 줄 알았는데 아니었다.
          위치 루프가 방향 루프보다 빠르면 결합계가 진동한다.
            k_e 0.35 / taper 2.0  ->  0.16 / 0.21 / 0.18 m  (안정)
            k_e 0.60 / taper 1.2  ->  0.05 / 0.19 / 0.43 m  (3회중 1회 발산,
                                      방향오차 -33 deg)
          잔류 0.18 m 는 taper 구간에서 보정을 멈추기 때문이다.
          더 줄이려면 게인이 아니라 선회 정확도를 올려야 한다.

        ! 방향 게인은 **거리영역**으로 잡아야 한다. 조향 한계는 곡률
          (1/R_min = 0.264 /m) 이라 속도와 무관한데, ω = k*psi 로 시간영역
          게인을 쓰면 저속에서 ω 한계(v/R_min)가 작아져 조금만 벗어나도
          포화된다. 그러면 조향이 bang-bang 이 되어 릴레이 진동에 빠진다.
          실측으로 방향오차 -34.7 deg 가 났다 (같은 코드가 어떤 회차엔
          되고 어떤 회차엔 안 됐다 — 한계 안정이었다).
              omega = v * k_s * psi,   k_s [1/m] 는 곡률 게인
          k_s=0.8 이면 psi 0.33 rad 까지 포화되지 않는다.
        """
        ax, ay = math.cos(gyaw), math.sin(gyaw)          # 주차면 축
        lx, ly = -math.sin(gyaw), math.cos(gyaw)         # 횡방향
        tw = Twist()
        t0 = time.time()
        start = self.pose()
        travelled = 0.0
        while rclpy.ok() and time.time() - t0 < limit:
            c = self.pose()
            if c is None:
                rclpy.spin_once(self, timeout_sec=0.05); continue
            dx, dy = c[0] - gx, c[1] - gy
            proj = dx * ax + dy * ay                     # 축방향 남은 거리
            e = dx * lx + dy * ly                        # 횡오차
            if proj <= 0.03:
                break
            travelled = math.dist(c[:2], start[:2])
            if travelled > 8.0:                          # 안전장치
                break
            phi = max(-0.25, min(0.25, k_e * e))
            phi *= min(1.0, proj / taper)      # 종단에서 중심선과 나란히
            th_cmd = gyaw + phi
            psi = math.atan2(math.sin(th_cmd - c[2]), math.cos(th_cmd - c[2]))
            v = abs(speed) * (0.5 if proj < 0.5 else 1.0)
            w_max = v / r_min
            tw.linear.x = -v
            tw.angular.z = max(-w_max, min(w_max, v * k_s * psi))
            self.cmd.publish(tw)
            if os.environ.get('PARK_DEBUG') and                     time.time() - getattr(self, '_dbg', 0) > 0.8:
                self._dbg = time.time()
                print('    proj %5.2f  e %+6.3f  phi %+5.1f  th %+6.1f  '
                      'psi %+5.1f  w %+6.3f'
                      % (proj, e, math.degrees(phi), math.degrees(c[2]),
                         math.degrees(psi), tw.angular.z))
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(25):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        return travelled

    # ---------------- (예전) 직선 후진 ----------------
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


def do_park(n, D, spot):
    gx, gy, gyaw = spot['goal_pose']
    ax, ay = spot['aisle_point']
    TURN_DX = float(os.environ.get('PARK_TURN_DX', 4.43))
    ccw = gyaw > 0.0
    appr_x, appr_y = ax + TURN_DX, ay

    print('  통로점 (%.2f, %.2f)   목표 (%.2f, %.2f, %+.0f deg)'
          % (ax, ay, gx, gy, math.degrees(gyaw)))
    print()
    print('[1/4] 접근 — Nav2 로 통로를 따라 접근점 (%.2f, %.2f) 까지'
          % (appr_x, appr_y))
    if not n.approach(appr_x, appr_y, 0.0):
        return False
    print()
    print('  Nav2 에서 제어권 인수 (velocity_smoother 정지)')
    lifecycle(n, 'deactivate'); time.sleep(1.5)

    print()
    print('[2/4] 미세정렬 — 통로를 따라 x 를 %.2f 로' % appr_x)
    c = n.align_x(appr_x)
    print('  정렬 후 (%.2f, %.2f, %+.0f deg)   x 오차 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), abs(c[0] - appr_x)))

    print()
    print('[3/4] 후진 선회 — 반경 %.2f m 로 %+.0f deg 까지'
          % (n.r_base, math.degrees(gyaw)))
    c = n.reverse_turn(gyaw, ccw, n.r_base)
    print('  선회 후 (%.2f, %.2f, %+.0f deg)   중심선까지 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), abs(c[0] - gx)))

    print()
    print('[4/4] 중심선 추종 후진 — 횡오차를 줄이며 주차면 안으로')
    n.reverse_track(gx, gy, gyaw)
    return True


def do_unpark(n, D, spot):
    """출차 — 주차의 역순. 전진으로만 나온다 (계획서 요구사항)."""
    gx, gy, gyaw = spot['goal_pose']
    ax, ay = spot['aisle_point']
    ccw = gyaw < 0.0        # -90 에서 0 으로 가려면 반시계
    # 전진 선회가 끝나는 지점이 통로 중심선이 되도록 탈출 거리를 잡는다
    turn_start_y = ay + (n.r_base if gyaw < 0 else -n.r_base)
    c = n.pose()
    out = abs(c[1] - turn_start_y)

    print('  현재 (%.2f, %.2f, %+.0f deg)   통로 %.2f'
          % (c[0], c[1], math.degrees(c[2]), ay))
    print()
    print('  Nav2 에서 제어권 인수 (velocity_smoother 정지)')
    lifecycle(n, 'deactivate'); time.sleep(1.5)

    print()
    print('[1/3] 전진 탈출 %.2f m — 주차면 밖으로 (후진 안 함)' % out)
    moved = n.forward_hold(gx, gy, gyaw, out)
    c = n.pose()
    print('  탈출 후 (%.2f, %.2f, %+.0f deg)  이동 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), moved))

    print()
    print('[2/3] 전진 선회 — 통로 방향(0 deg)까지, 반경 %.2f m' % n.r_base)
    c = n.forward_turn(0.0, ccw, n.r_base)
    print('  선회 후 (%.2f, %.2f, %+.0f deg)   통로 중심선까지 %.2f m'
          % (c[0], c[1], math.degrees(c[2]), abs(c[1] - ay)))

    print()
    print('  Nav2 에 제어권 반납')
    lifecycle(n, 'activate'); time.sleep(2.0)
    ex = D['exit_pose']
    print()
    print('[3/3] 출구로 — Nav2 (%.2f, %.2f)' % (ex[0], ex[1]))
    return n.approach(ex[0], ex[1], ex[2], limit=420.0)


def main():
    rclpy.init()
    n = Park()
    if not n.wait_odom():
        print('/odom 없음 — 시뮬이 안 돌고 있다'); rclpy.shutdown(); return 1
    D = load_spots()
    n.r_base = D['robot_spec'].get('min_turning_radius_base_link', 3.78)
    spot = next((s for s in D['spots'] if s['id'] == SPOT), None)
    if spot is None:
        print('주차면 %s 없음' % SPOT); rclpy.shutdown(); return 1

    print('===== %s  주차면 %s (행 %s, 진입 %s) ====='
          % ('출차' if MODE == 'unpark' else '주차',
             spot['id'], spot['row'], spot['entry_side']))
    ok = do_unpark(n, D, spot) if MODE == 'unpark' else do_park(n, D, spot)
    if not ok:
        print('  실패'); lifecycle(n, 'activate'); rclpy.shutdown(); return 1

    c = n.pose()
    gx, gy, gyaw = spot['goal_pose']
    if MODE == 'park':
        err = math.dist(c[:2], (gx, gy))
        dyaw = math.degrees(math.atan2(math.sin(c[2] - gyaw),
                                       math.cos(c[2] - gyaw)))
        x0, y0, x1, y1 = spot['rect']
        inside = x0 <= c[0] <= x1 and y0 <= c[1] <= y1
        print()
        print('  최종 (%.2f, %.2f, %+.0f deg)' % (c[0], c[1], math.degrees(c[2])))
        print('  goal 대비  위치오차 %.2f m,  방향오차 %+.1f deg' % (err, dyaw))
        print('  주차면 안에 있는가: %s' % ('예' if inside else '아니오'))
    else:
        ex = D['exit_pose']
        print()
        print('  최종 (%.2f, %.2f, %+.0f deg)   출구까지 %.2f m'
              % (c[0], c[1], math.degrees(c[2]), math.dist(c[:2], ex[:2])))
    print()
    print('  Nav2 에 제어권 반납')
    lifecycle(n, 'activate')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
