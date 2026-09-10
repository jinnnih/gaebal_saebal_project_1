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

주차 기동 명령은 velocity_smoother 의 입력인 /cmd_vel_nav 로 넣는다. Nav2 에
목표가 없는 동안 그 토픽은 비어 있으므로 경합하지 않고, 스무더가 그대로
흘려보내 /cmd_vel_smoothed 로 나간다. 라이프사이클은 건드리지 않는다
(cmd_topic 파라미터의 주석 참고 — 그게 Nav2 를 통째로 무너뜨렸다).
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
from nav2_msgs.srv import ClearEntireCostmap
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

        # velocity_smoother 의 **입력** 토픽. 여기에 넣으면 스무더가 그대로
        # 흘려보내 /cmd_vel_smoothed 로 나가고 twist_to_ackermann 이 받는다.
        #
        # ! 예전에는 /cmd_vel_smoothed 에 직접 쓰면서 스무더를
        #   `ros2 lifecycle set /velocity_smoother deactivate` 로 재웠는데,
        #   그게 Nav2 스택 전체를 무너뜨리고 있었다. 라이프사이클 노드를
        #   수동으로 내리면 bond 가 끊기고 lifecycle_manager 가 이렇게 본다.
        #
        #     CRITICAL FAILURE: SERVER velocity_smoother IS DOWN after not
        #     receiving a heartbeat for 4000 ms. Shutting down related nodes.
        #
        #   그러고는 관리 노드를 전부 리셋한다. 로그 하나에서만 16 번
        #   일어났고, 부하가 걸리면 복구마저 실패해서
        #   (Failed to change state for node: map_server. Aborting bringup)
        #   그 뒤 모든 목표가 "Action server is inactive. Rejecting the goal"
        #   으로 거부됐다. 접근 실패(status=6)와 실행 간 편차의 상당 부분이
        #   여기서 나왔다. 라이프사이클은 lifecycle_manager 만 만진다.
        self.declare_parameter('cmd_topic', '/cmd_vel_nav')
        # 90 도 후진 선회의 시작 횡오차는 선회 반경 그 자체여야 한다.
        # 뒤축 기준 최소 3.5704 이므로 base_link 기준으로는 그 + 1.25.
        # 하한에 딱 맞추면 닫힌 루프가 반경을 줄이는 쪽으로 손을 못 쓰니
        # 0.73 m 여유를 둬서 목표 반경을 4.30 으로 잡는다.
        self.declare_parameter('turn_dx', 5.55)
        # 선회 전에 통로를 따라 곧게 달려 자세를 잡을 거리
        self.declare_parameter('pre_run', 4.0)
        g = self.get_parameter
        self.topic = g('cmd_topic').value
        self.turn_dx = float(g('turn_dx').value)
        self.pre_run = float(g('pre_run').value)

        self.D = load_spots()
        spec = self.D['robot_spec']
        self.r_base = spec.get('min_turning_radius_base_link', 3.7829)
        # 뒤축 기준 최소 반경과, base_link 가 뒤축보다 앞선 거리
        self.r_rear = spec.get('min_turning_radius', 3.5704)
        self.d_rear = spec.get('wheelbase', 2.5) / 2.0
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
        self.clear_srv = [
            self.navnode.create_client(
                ClearEntireCostmap,
                '/%s/clear_entirely_%s' % (n, n))
            for n in ('local_costmap', 'global_costmap')]
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

    def align_lane(self, target_x, lane_y, speed=0.22, k_e=0.35, k_s=0.8,
                   r_min=3.7829, limit=90.0):
        """통로 중심선을 따라 전진하며 x, y, yaw 를 한꺼번에 맞춘다.

        ! 이게 주차 정확도 편차의 지배 항이었다. 예전 align_x 는 angular.z
          를 0 으로 두고 "차가 동쪽을 본다"고 가정한 채 직진했는데, Nav2 의
          기본 목표 허용오차가 yaw 0.50 rad 이라 접근이 끝난 시점의 방향이
          실측에서 -37 도 ~ +11 도까지 흩어졌다. 그 상태로 직진하면 방향은
          그대로인 채 y 까지 틀어지고, 선회 진입 psi 가 53 도 ~ 101 도로
          벌어져 최종 오차가 0.17 m 와 1.15 m 로 갈렸다.

        기준선 y = lane_y, 기준 방향 0 (동쪽) 을 두고 전진 추종하면
        방향과 횡오차가 같이 수렴한다. 37 도에서 4 도까지 줄이는 데
        ln(9.25)/k_s = 2.8 m 가 필요해서 pre_run 을 4 m 로 잡았다.
        """
        c = self.pose()
        if c is not None and c[0] > target_x - 2.0:
            # 조주 거리가 없으면 일단 거칠게 물러선다 (정밀도는 이 뒤에서)
            self.align_x(target_x - 3.5, tol=0.20)
        tw = Twist()
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < limit:
            c = self.pose()
            if c is None:
                time.sleep(0.05); continue
            remain = target_x - c[0]
            if remain <= 0.0:
                break
            e = c[1] - lane_y                      # 좌횡오차
            phi = -max(-0.20, min(0.20, k_e * e))  # 전진이라 부호 반대
            psi = math.atan2(math.sin(phi - c[2]), math.cos(phi - c[2]))
            v = abs(speed) * max(0.3, min(1.0, remain / 1.0))
            w_max = v / r_min
            tw.linear.x = v
            tw.angular.z = max(-w_max, min(w_max, v * k_s * psi))
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

    # 이 각도 아래에서는 (1-cos psi) 가 너무 작아 보정 권한이 없다
    PSI_HOLD = math.radians(15.0)

    def reverse_turn(self, target_yaw, ccw, radius, gx=None, gy=None,
                     speed=0.25, limit=90.0):
        """목표 방향이 될 때까지 후진 선회한다.

        열린 루프(고정 반경)로 돌리면 종료 지점 횡오차가 0.28~0.54 m 씩
        흔들리고, 그게 최종 주차 오차의 지배 항이 된다. gx, gy 를 주면
        남은 회전각을 보고 매 주기 반경을 다시 잡는 닫힌 루프로 돈다.

        ## 기하

        후진(v<0) 중 횡오차 e 와 남은 회전각 psi 는

            e_dot = v sin(psi),   psi_dot = w

        이므로 de/dpsi = v sin(psi) / w 이고, psi 를 0 까지 적분하면

            cw  (w<0):  e(0) = e - R (1 - cos psi)   =>  R = +e / (1 - cos psi)
            ccw (w>0):  e(0) = e + R (1 - cos psi)   =>  R = -e / (1 - cos psi)

        **선회 방향에 따라 부호가 반대다.** abs(e) 를 쓰면 cw 에서만 우연히
        맞고 ccw 스팟(북쪽 통로, 전체의 절반)에서는 처음부터 틀린다.

        ## 함정 셋, 전부 실측으로 데었다

        1) 이 유도는 "속도 방향 == 차체 방향" 인 점에서만 성립한다. 그건
           뒤축이지 base_link 가 아니다. base_link 는 휠베이스 중점이라
           최소 반경 선회 중 슬립각이 atan(1.25/3.57)=17.8 도나 된다.
           실측에서도 코드가 믿는 진행 방향과 실제 변위각이 15.6 도
           어긋났다. 그래서 e, psi 를 전부 **뒤축 좌표**로 옮겨서 푼다.

        2) e 의 부호를 버리면 발산한다. 중심선을 넘어간 뒤 |e| 가 커지면
           R 이 커지고 -> 선회를 덜 하고 -> 기운 채 직진해서 -> |e| 가 더
           커진다. 실측에서 psi 23 도에 e 0.00 이던 게 psi 5 도에 -0.99
           까지 벌어졌다. 부호를 살리면 R 이 음수가 되어 하한으로 잡히고,
           최대한 조여서 psi 를 빨리 죽이는 게 맞는 대응이 된다.

        3) psi 가 작아지면 분모가 0 으로 가서 R 이 폭주한다. 하한만으로는
           부족하고 PSI_HOLD 아래에서는 보정을 동결한다. 그 지점에 남은
           횡방향 이동량은 R(1-cos 15도) = 0.12 m 뿐이라 잃는 게 없고,
           잔차는 다음 단계 reverse_track 이 절반으로 줄인다.
        """
        d = self.d_rear
        lx, ly = -math.sin(target_yaw), math.cos(target_yaw)
        sgn = -1.0 if ccw else 1.0
        # 목표 자세에서의 뒤축 위치
        gxr = gyr = 0.0
        if gx is not None:
            gxr = gx - d * math.cos(target_yaw)
            gyr = gy - d * math.sin(target_yaw)
        tw = Twist()
        r_hold = radius
        prev = None
        first = True
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < limit:
            c = self.pose()
            if c is None:
                time.sleep(0.05); continue
            psi = math.atan2(math.sin(c[2] - target_yaw),
                             math.cos(c[2] - target_yaw))
            if abs(psi) < math.radians(4.0):
                break
            e = 0.0
            if gx is not None:
                xr = c[0] - d * math.cos(c[2])
                yr = c[1] - d * math.sin(c[2])
                e = (xr - gxr) * lx + (yr - gyr) * ly
                if abs(psi) > self.PSI_HOLD:
                    r_new = min(30.0, max(self.r_rear,
                                          sgn * e / (1.0 - math.cos(psi))))
                    # 오돔 잡음이 동결값을 튀게 하지 않도록 1차 저역통과
                    r_hold = 0.7 * r_hold + 0.3 * r_new
            # 뒤축 반경 -> base_link twist. twist_to_ackermann 이 다시
            # 뒤축으로 환산하므로 wz = v / hypot(R_rear, d) 로 줘야 한다
            tw.linear.x = -abs(speed)
            tw.angular.z = -sgn * abs(speed) / math.hypot(r_hold, d)
            self.publish_cmd(tw)
            if os.environ.get('PARK_DEBUG') and first:
                first = False
                need = sgn * e / max(1.0 - math.cos(psi), 1e-6)
                print('    TURN START  psi %+5.1f  e %+6.3f  R_need %6.2f'
                      % (math.degrees(psi), e, need), flush=True)
            if os.environ.get('PARK_DEBUG') and                     time.time() - getattr(self, '_dbgt', 0) > 1.0:
                self._dbgt = time.time()
                r_act = float('nan')
                if prev is not None:
                    dpsi = abs(math.atan2(math.sin(c[2] - prev[2]),
                                          math.cos(c[2] - prev[2])))
                    if dpsi > 1e-3:
                        r_act = math.hypot(c[0] - prev[0],
                                           c[1] - prev[1]) / dpsi
                print('    TURN  psi %+5.1f  e %+6.3f  R_cmd %5.2f  '
                      'R_act %5.2f  x %6.2f y %6.2f'
                      % (math.degrees(psi), e, r_hold, r_act, c[0], c[1]),
                      flush=True)
                prev = c
            time.sleep(0.05)
        for _ in range(20):
            self.publish_cmd(Twist()); time.sleep(0.02)
        return self.pose()

    # ---------------- Nav2 접근 ----------------
    def clear_costmaps(self, wait=6.0):
        """코스트맵을 비운다.

        ! bt_navigator 의 복구 행동도 같은 서비스를 부르는데, 부하가 걸리면
          그쪽이 먼저 터진다 (Node timed out while executing service call to
          local_costmap/clear_entirely_local_costmap). 그러면 복구가 통째로
          실패하고 목표가 status=6 으로 죽는다. 재시도 전에 직접 비운다.
        """
        for cli in self.clear_srv:
            if not cli.wait_for_service(timeout_sec=wait):
                continue
            fut = cli.call_async(ClearEntireCostmap.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < wait:
                time.sleep(0.1)

    def approach(self, x, y, yaw, phase, limit=300.0, tries=2):
        """Nav2 로 목표 자세까지 간다.

        ! MPPI 가 간헐적으로 모든 표본을 버리고 (Optimizer fail to compute
          path) 목표를 status=6 으로 중단시킨다. 부하가 높을 때 나오고,
          같은 목표를 그대로 다시 쏘면 대개 통과한다. 코스트맵을 비우고
          한 번 재시도한다.
        """
        if not self.nav.wait_for_server(timeout_sec=20.0):
            return False, 'navigate_to_pose 서버 없음'
        g = NavigateToPose.Goal()
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x = float(x); ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        g.pose = ps
        msg = ''
        for attempt in range(tries):
            if attempt:
                self.get_logger().warn('접근 재시도 %d/%d — %s'
                                       % (attempt + 1, tries, msg))
                self.clear_costmaps()
                time.sleep(2.0)
            ok, msg = self._approach_once(g, x, y, phase, limit)
            if ok:
                return True, ''
        return False, msg

    def _approach_once(self, g, x, y, phase, limit):
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

    # ---------------- 주차 ----------------
    def run_park(self, spot):
        gx, gy, gyaw = spot['goal_pose']
        ax, ay = spot['aisle_point']
        ccw = gyaw > 0.0
        appr_x = ax + self.turn_dx

        ok, msg = self.approach(appr_x - self.pre_run, ay, 0.0, 'APPROACH')
        if not ok:
            return ok, msg
        self.feedback('ALIGN')
        self.align_lane(appr_x, ay)
        self.feedback('TURN')
        self.reverse_turn(gyaw, ccw, self.r_rear, gx, gy)
        self.feedback('TRACK')
        self.reverse_track(gx, gy, gyaw)
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
        self.feedback('ESCAPE')
        self.forward_hold(gx, gy, gyaw, abs(c[1] - turn_start_y))
        self.feedback('TURN')
        self.forward_turn(0.0, ccw, self.r_base)
        time.sleep(2.0)
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
