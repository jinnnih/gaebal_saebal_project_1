#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주차 / 출차 기동 액션 서버 — 계획서의 ParkManeuver / UnparkManeuver.

    ros2 run valet_robot park_action_server.py
    ros2 action send_goal /valet/park valet_robot/action/Park          "{spot_id: 'A08', mode: 0}"      # 0=PARK, 1=UNPARK

## 왜 액션이 Nav2 대신 주차면 진입을 맡는가

keepout 필터가 주차면 내부를 진입금지로 막고 있어서 플래너가 그 안으로 경로를
못 낸다. 런타임 토글은 실제로 안 먹는다 (CostmapFilter 가 enabled 를 초기화
때 캐시하고 동적 파라미터 콜백을 등록하지 않는다 — 이슈 #7).

그리고 막혀 있지 않더라도 Nav2 로는 어렵다. 통로에 수직인 자세(prepark_pose)를
목표로 주면 8 m 통로에서 300 s 를 헤매고도 못 맞춘다 (실측). 계획서가
ParkManeuver 를 따로 둔 이유다.

## 단계

  주차   접근(Nav2) -> 통로방향 미세정렬 -> 후진 선회 -> 중심선 추종 후진
  출차   전진 탈출 -> 전진 선회 -> 출구로(Nav2)

출차가 전진만 쓰는 것은 뒤로 넣었기 때문이다. 계획서의 "출차는 항상 전진" 이
자동으로 만족된다.

## 제어권

주차 기동 동안 velocity_smoother 를 재우고 /cmd_vel_smoothed 로 직접 명령한다.
그 노드가 20 Hz 로 0 을 계속 발행해서 그냥 쏘면 경합한다. 끝나면 돌려준다.
"""
import json
import math
import os
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from valet_robot.action import Park


def load_spots():
    try:
        share = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'parking_lot_world'],
            text=True).strip()
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', '..',
                             'parking_lot_world')
    with open(os.path.join(share, 'config', 'parking_spots.json'),
              encoding='utf-8') as f:
        return json.load(f)


class ParkServer(Node):

    def __init__(self):
        super().__init__('park_action_server')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('cmd_topic', '/cmd_vel_smoothed')
        self.declare_parameter('turn_dx', 4.43)
        self.declare_parameter('smoother_node', '/velocity_smoother')
        g = self.get_parameter
        self.topic = g('cmd_topic').value
        self.turn_dx = float(g('turn_dx').value)
        self.smoother = g('smoother_node').value

        self.D = load_spots()
        self.r_base = self.D['robot_spec'].get('min_turning_radius_base_link',
                                               3.7829)
        self.odom = None
        self._goal_lock = threading.Lock()
        self._fb = None                 # 현재 goal handle (피드백용)
        self._shunt_sign = 0
        self._shunt_since = 0.0
        self._shunt_counted = False
        self._shunt_prev = None
        self.shunts = 0

        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom', m), 10,
                                 callback_group=cb)
        self.cmd = self.create_publisher(Twist, self.topic, 10)

        # ! Nav2 액션 클라이언트는 **별도 노드 + 별도 executor** 에 둔다.
        #   이 서버의 execute 콜백(executor 스레드) 안에서 같은 executor 의
        #   액션 클라이언트를 기다리면 wait set 이 두 스레드에서 건드려져
        #   깨진다. 실제로 주차 성공 직후 이렇게 죽었다:
        #     RCLError: wait set index for status subscription is out of bounds
        #   별도 executor 를 데몬 스레드로 돌리면 서로 안 겹친다.
        self.navnode = rclpy.create_node('park_nav_client')
        self.navnode.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.nav = ActionClient(self.navnode, NavigateToPose,
                                'navigate_to_pose')
        self._navex = SingleThreadedExecutor()
        self._navex.add_node(self.navnode)
        threading.Thread(target=self._navex.spin, daemon=True).start()
        self.srv = ActionServer(
            self, Park, '/valet/park',
            execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            '주차/출차 액션 서버 준비. 액션 /valet/park, 명령 %s' % self.topic)

    # ! 기동 루프에서 rclpy.spin_once(self, ...) 를 부르면 안 된다.
    #   이 노드는 이미 MultiThreadedExecutor 가 돌리고 있어서, 콜백 안에서
    #   중첩 spin 을 하면 executor 의 wait set 이 깨진다. 실측 증상:
    #     - approach() 중 self.odom 이 얼어붙어 피드백이 한 좌표에 고정
    #     - 첫 목표를 처리한 뒤 서버가 다음 목표를 아예 못 받음 (목표 거부)
    #   구독 갱신은 executor 에 맡기고 루프는 time.sleep 만 한다.

    # ------------- 피드백 / 전후진 전환 계수 -------------
    def publish_cmd(self, tw):
        """명령을 내면서 전후진 전환을 센다.

        ! 채터링 제거는 #9 Q8 에서 확정된 대로다.
              |vx| < 0.05 m/s 는 무시 (정지 구간)
              부호가 0.3 s 이상 유지될 때만 1 회로 계수
          이게 없으면 감속 구간의 미세한 부호 흔들림이 전부 세어진다.
        """
        v = tw.linear.x
        now = time.time()
        if abs(v) < 0.05:
            self.cmd.publish(tw); return
        sign = 1 if v > 0 else -1
        if sign != self._shunt_sign:
            self._shunt_sign = sign
            self._shunt_since = now
            self._shunt_counted = False
        elif not self._shunt_counted and now - self._shunt_since >= 0.3:
            self._shunt_counted = True
            if self._shunt_prev is not None and sign != self._shunt_prev:
                self.shunts += 1
            self._shunt_prev = sign
        self.cmd.publish(tw)

    def feedback(self, phase, remaining=-1.0):
        if self._fb is None:
            return
        c = self.pose()
        if c is None:
            return
        f = Park.Feedback()
        f.phase = phase
        f.x, f.y = c[0], c[1]
        f.yaw_deg = math.degrees(c[2])
        f.remaining_m = float(remaining)
        self._fb.publish_feedback(f)

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
            time.sleep(0.2)
        return self.odom is not None

    # ---------------- 1단계: 접근 ----------------

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
                time.sleep(0.05); continue
            e = target_x - c[0]
            if abs(e) < tol:
                break
            # 차체가 동쪽(+x)을 보므로 e>0 이면 전진
            v = speed if e > 0 else -speed
            if abs(e) < 0.35:
                v *= 0.5
            tw.linear.x = v; tw.angular.z = 0.0
            self.publish_cmd(tw)
            time.sleep(0.05)
        for _ in range(20):
            self.publish_cmd(Twist()); time.sleep(0.02)
        return self.pose()

    # ---------------- 2단계: 후진 선회 ----------------

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
                time.sleep(0.05); continue
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
            self.publish_cmd(tw)
            time.sleep(0.05)
        for _ in range(20):
            self.publish_cmd(Twist()); time.sleep(0.02)
        return moved

    def forward_turn(self, target_yaw, ccw, radius, speed=0.22, limit=90.0):
        """목표 방향까지 전진 선회한다 (주차의 후진 선회와 대칭)."""
        tw = Twist()
        tw.linear.x = abs(speed)
        tw.angular.z = (1.0 if ccw else -1.0) * abs(speed) / radius
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < limit:
            self.publish_cmd(tw)
            time.sleep(0.05)
            c = self.pose()
            if c is None:
                continue
            d = math.atan2(math.sin(target_yaw - c[2]),
                           math.cos(target_yaw - c[2]))
            if abs(d) < math.radians(4.0):
                break
        for _ in range(20):
            self.publish_cmd(Twist()); time.sleep(0.02)
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
                time.sleep(0.05); continue
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
            self.publish_cmd(tw)
            if os.environ.get('PARK_DEBUG') and                     time.time() - getattr(self, '_dbg', 0) > 0.8:
                self._dbg = time.time()
                print('    proj %5.2f  e %+6.3f  phi %+5.1f  th %+6.1f  '
                      'psi %+5.1f  w %+6.3f'
                      % (proj, e, math.degrees(phi), math.degrees(c[2]),
                         math.degrees(psi), tw.angular.z))
            time.sleep(0.05)
        for _ in range(25):
            self.publish_cmd(Twist()); time.sleep(0.02)
        return travelled

    # ---------------- (예전) 직선 후진 ----------------

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
            self.publish_cmd(tw)
            time.sleep(0.05)
            c = self.pose()
            if c is None:
                continue
            d = math.atan2(math.sin(target_yaw - c[2]),
                           math.cos(target_yaw - c[2]))
            if abs(d) < math.radians(4.0):
                break
        for _ in range(20):
            self.publish_cmd(Twist()); time.sleep(0.02)
        return self.pose()

    # ---------------- 출차: 전진 탈출 ----------------

    # ---------------- Nav2 접근 ----------------
    def approach(self, x, y, yaw, phase, limit=300.0):
        if not self.nav.wait_for_server(timeout_sec=20.0):
            return False, 'navigate_to_pose 서버 없음'
        g = NavigateToPose.Goal()
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = float(x); ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        g.pose = ps
        fut = self.nav.send_goal_async(g)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 20:
            time.sleep(0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, '목표 거부됨'
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t0 < limit:
            time.sleep(0.3)
            c = self.pose()
            if c:
                self.feedback(phase, math.dist(c[:2], (x, y)))
        if not rf.done():
            gh.cancel_goal_async()
            return False, '접근 시간 초과'
        if rf.result().status != 4:
            return False, '접근 실패 (status=%d)' % rf.result().status
        return True, ''

    # ---------------- 제어권 ----------------
    def smoother_set(self, action):
        try:
            subprocess.run(['ros2', 'lifecycle', 'set', self.smoother, action],
                           capture_output=True, timeout=25)
        except Exception:
            pass

    # ---------------- 주차 ----------------
    def run_park(self, spot):
        gx, gy, gyaw = spot['goal_pose']
        ax, ay = spot['aisle_point']
        ccw = gyaw > 0.0
        appr_x = ax + self.turn_dx

        ok, msg = self.approach(appr_x, ay, 0.0, 'APPROACH')
        if not ok:
            return ok, msg
        self.smoother_set('deactivate'); time.sleep(1.5)
        self.feedback('ALIGN')
        self.align_x(appr_x)
        self.feedback('TURN')
        self.reverse_turn(gyaw, ccw, self.r_base)
        self.feedback('TRACK')
        self.reverse_track(gx, gy, gyaw)
        self.smoother_set('activate')
        return True, ''

    # ---------------- 출차 ----------------
    def run_unpark(self, spot):
        gx, gy, gyaw = spot['goal_pose']
        ax, ay = spot['aisle_point']
        ccw = gyaw < 0.0
        turn_start_y = ay + (self.r_base if gyaw < 0 else -self.r_base)
        c = self.pose()
        if c is None:
            return False, '/odom 없음'
        self.smoother_set('deactivate'); time.sleep(1.5)
        self.feedback('ESCAPE')
        self.forward_hold(gx, gy, gyaw, abs(c[1] - turn_start_y))
        self.feedback('TURN')
        self.forward_turn(0.0, ccw, self.r_base)
        self.smoother_set('activate'); time.sleep(2.0)
        ex = self.D['exit_pose']
        return self.approach(ex[0], ex[1], ex[2], 'EXIT', limit=420.0)

    # ---------------- 액션 진입점 ----------------
    def execute(self, goal_handle):
        req = goal_handle.request
        res = Park.Result()
        t0 = time.time()
        with self._goal_lock:
            self._fb = goal_handle
            self.shunts = 0
            self._shunt_sign = 0
            self._shunt_prev = None
            self._shunt_counted = False
            try:
                if not self.wait_odom():
                    res.success = False; res.message = '/odom 없음'
                    goal_handle.abort(); return res
                spot = next((s for s in self.D['spots']
                             if s['id'] == req.spot_id), None)
                if spot is None:
                    res.success = False
                    res.message = '주차면 %s 없음' % req.spot_id
                    goal_handle.abort(); return res

                if req.mode == Park.Goal.UNPARK:
                    ok, msg = self.run_unpark(spot)
                else:
                    ok, msg = self.run_park(spot)

                res.success = ok
                res.message = msg
                res.shunts_park = self.shunts
                res.duration_s = time.time() - t0
                c = self.pose()
                if ok and req.mode == Park.Goal.PARK and c:
                    gx, gy, gyaw = spot['goal_pose']
                    x0, y0, x1, y1 = spot['rect']
                    res.err_m = math.dist(c[:2], (gx, gy))
                    res.err_deg = math.degrees(
                        math.atan2(math.sin(c[2] - gyaw),
                                   math.cos(c[2] - gyaw)))
                    res.inside_rect = bool(x0 <= c[0] <= x1 and
                                           y0 <= c[1] <= y1)
                if ok:
                    goal_handle.succeed()
                else:
                    self.smoother_set('activate')   # 실패해도 제어권은 돌려준다
                    goal_handle.abort()
            finally:
                self._fb = None
        self.get_logger().info(
            '%s %s — %s  err %.2f m / %.1f deg  안: %s  shunts %d  %.0f s'
            % ('출차' if req.mode == Park.Goal.UNPARK else '주차',
               req.spot_id, '성공' if res.success else ('실패: ' + res.message),
               res.err_m, res.err_deg, res.inside_rect, res.shunts_park,
               res.duration_s))
        return res


def main():
    rclpy.init()
    n = ParkServer()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
