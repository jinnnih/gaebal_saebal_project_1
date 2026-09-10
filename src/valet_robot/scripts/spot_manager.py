#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주차면 관리 + 발렛 요청 처리 노드.

대시보드(#9 계약)와 로봇 사이를 잇는다.

    ros2 run valet_robot spot_manager.py

## 하는 일

  1. 주차면 54 면의 상태를 들고 /valet/spot_states 로 낸다
  2. /valet/request 를 받아 주차/출차를 수행한다
     - spot_id 가 비면 빈 자리를 고른다 (FindParkingSpot)
     - /valet/park 액션(ParkManeuver/UnparkManeuver)을 호출한다
  3. 진행 상황을 /valet/mission_status 로 낸다

## 계약 (#9 확정)

  토픽                    durability        reliability  depth
  /valet/spot_states      transient_local   reliable     1
  /valet/mission_status   volatile          reliable     20
  /valet/request (구독)    volatile          reliable     10

  spot_states 는 54 면 **전체 스냅샷**이고 변화 시 즉시 + 1 Hz 하트비트다.
  latch 는 "마지막 값"만 주지 노드가 살아있는지는 안 알려주므로 하트비트가
  따로 필요하다. 대시보드는 stamp 가 3 초 이상 안 바뀌면 연결 끊김으로 본다.

  mission_status 는 이벤트 로그라 latch 하지 않는다. 새로 붙은 대시보드가
  과거 이벤트 하나만 받아 오해하는 것보다 DB 에서 읽는 게 맞다.

## 상태

  FREE  RESERVED  OCCUPIED  BLOCKED
  초기값은 parking_spots.json 의 initially_occupied / type=hatched 에서 온다.
"""
import hashlib
import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from valet_robot.action import Park

FREE, RESERVED, OCCUPIED, BLOCKED = 'FREE', 'RESERVED', 'OCCUPIED', 'BLOCKED'

# #9 Q6 에서 확정된 13 종
EVENTS = ('REQUEST_ACCEPTED', 'SPOT_SELECTED', 'SPOT_RESERVED', 'NAV_STARTED',
          'PREPARK_REACHED', 'PARK_STARTED', 'PARK_DONE', 'UNPARK_STARTED',
          'UNPARK_DONE', 'EXIT_REACHED', 'RECOVERY', 'FAILED', 'ABORTED')

# 액션 피드백의 phase -> 이벤트. 같은 phase 가 반복돼도 한 번만 낸다.
# ! 모드별로 나눠야 한다. TURN 은 주차/출차 양쪽에 나오는데, 하나로 두면
#   출차 중에 PARK_STARTED 가 나간다 (실측으로 나왔다).
PHASE_EVENT = {
    Park.Goal.PARK: {
        'APPROACH': 'NAV_STARTED',
        'ALIGN': 'PREPARK_REACHED',
        'TURN': 'PARK_STARTED',
    },
    Park.Goal.UNPARK: {
        'ESCAPE': 'UNPARK_STARTED',
        # TURN 은 UNPARK_STARTED 에 이미 포함된 동작이라 따로 내지 않는다
        'EXIT': 'NAV_STARTED',
    },
}


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds') \
        .replace('+00:00', 'Z')


def load_spots():
    try:
        share = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'parking_lot_world'],
            text=True).strip()
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', '..',
                             'parking_lot_world')
    path = os.path.join(share, 'config', 'parking_spots.json')
    raw = open(path, 'rb').read()
    return json.loads(raw.decode('utf-8')), hashlib.sha256(raw).hexdigest()[:8]


class SpotManager(Node):

    def __init__(self):
        super().__init__('spot_manager')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.D, self.checksum = load_spots()
        self.spots = {}
        for s in self.D['spots']:
            if s['type'] == 'hatched':
                st = BLOCKED
            elif s['initially_occupied']:
                st = OCCUPIED
            else:
                st = FREE
            self.spots[s['id']] = {'status': st, 'request_id': None,
                                   'vehicle_tag': None, 'spec': s}
        self._lock = threading.Lock()
        self._dirty = True
        self._busy = False
        self._seen = set()        # 이미 받은 request_id
        self._seq = {}                    # request_id -> 다음 seq

        # ! QoS 는 #9 에서 확정된 대로다. spot_states 만 latch 한다.
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_spots = self.create_publisher(String, '/valet/spot_states',
                                               latched)
        self.pub_status = self.create_publisher(String, '/valet/mission_status',
                                                20)
        self.create_subscription(String, '/valet/request', self.on_request, 10)

        # ! 액션 클라이언트는 별도 노드 + 별도 executor 에 둔다.
        #   같은 executor 에서 콜백 안에 기다리면 wait set 이 깨진다
        #   (park_action_server.py 에서 같은 함정을 밟았다).
        self.acnode = rclpy.create_node('spot_manager_park_client')
        self.acnode.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.ac = ActionClient(self.acnode, Park, '/valet/park')
        self._acex = SingleThreadedExecutor()
        self._acex.add_node(self.acnode)
        threading.Thread(target=self._acex.spin, daemon=True).start()

        self.create_timer(1.0, self.heartbeat)      # 1 Hz 하트비트
        self.create_timer(0.2, self.flush)          # 변화 시 즉시
        self.get_logger().info(
            '주차면 관리 노드 준비. %d 면 (FREE %d / OCCUPIED %d / BLOCKED %d), '
            'lot_checksum %s'
            % (len(self.spots),
               sum(1 for v in self.spots.values() if v['status'] == FREE),
               sum(1 for v in self.spots.values() if v['status'] == OCCUPIED),
               sum(1 for v in self.spots.values() if v['status'] == BLOCKED),
               self.checksum))

    # ---------------- 발행 ----------------
    def snapshot(self):
        with self._lock:
            return json.dumps({
                'stamp': now_iso(),
                'lot_checksum': self.checksum,
                'spots': [{'id': k, 'status': v['status'],
                           'request_id': v['request_id']}
                          for k, v in sorted(self.spots.items())],
            }, ensure_ascii=False)

    def heartbeat(self):
        self.pub_spots.publish(String(data=self.snapshot()))

    def flush(self):
        if self._dirty:
            self._dirty = False
            self.pub_spots.publish(String(data=self.snapshot()))

    def set_state(self, spot_id, status, request_id=None, tag=None):
        with self._lock:
            s = self.spots[spot_id]
            s['status'] = status
            s['request_id'] = request_id
            if tag is not None:
                s['vehicle_tag'] = tag
        self._dirty = True

    def emit(self, request_id, event, bt_node='', payload=None):
        """mission_status 발행. seq 는 요청 내 순번이다 (#9 Q6)."""
        if event not in EVENTS:
            self.get_logger().warn('계약에 없는 event: %s' % event)
        n = self._seq.get(request_id, 0)
        self._seq[request_id] = n + 1
        msg = {'stamp': now_iso(), 'request_id': request_id, 'seq': n,
               'event': event, 'bt_node': bt_node, 'payload': payload or {}}
        self.pub_status.publish(String(data=json.dumps(msg,
                                                       ensure_ascii=False)))
        self.get_logger().info('  [%d/%d] %s %s' % (request_id, n, event,
                                                    payload or ''))

    # ---------------- 빈 자리 고르기 ----------------
    def pick_spot(self, prefer=None):
        """FindParkingSpot. 지정이 있으면 그 자리, 없으면 입구에서 가까운 빈 자리."""
        with self._lock:
            if prefer:
                s = self.spots.get(prefer)
                if s is None:
                    return None, '주차면 %s 없음' % prefer
                if s['status'] != FREE:
                    return None, '주차면 %s 가 %s 상태' % (prefer, s['status'])
                return prefer, ''
            ent = self.D['entry_pose']
            cands = [(math.dist(v['spec']['aisle_point'], ent[:2]), k)
                     for k, v in self.spots.items() if v['status'] == FREE]
            if not cands:
                return None, '빈 주차면 없음'
            return min(cands)[1], ''

    # ---------------- 액션 호출 ----------------
    def run_action(self, spot_id, mode, request_id):
        """/valet/park 액션을 돌리고 피드백을 mission_status 로 중계한다."""
        if not self.ac.wait_for_server(timeout_sec=20.0):
            return None, '주차 액션 서버 없음'
        g = Park.Goal()
        g.spot_id = spot_id
        g.mode = mode
        seen = set()

        def on_fb(fb):
            ev = PHASE_EVENT.get(mode, {}).get(fb.feedback.phase)
            if ev and ev not in seen:
                seen.add(ev)
                self.emit(request_id, ev,
                          'UnparkManeuver' if mode == Park.Goal.UNPARK
                          else 'ParkManeuver',
                          {'spot_id': spot_id, 'phase': fb.feedback.phase})

        fut = self.ac.send_goal_async(g, feedback_callback=on_fb)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30:
            time.sleep(0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, '액션 목표 거부됨'
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t0 < 700:
            time.sleep(0.3)
        if not rf.done():
            gh.cancel_goal_async()
            return None, '액션 시간 초과'
        return rf.result().result, ''

    # ---------------- 요청 처리 ----------------
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error('요청 파싱 실패: %s' % e)
            return
        # ! 처리는 워커 스레드에서 한다. 구독 콜백 안에서 몇 분씩 붙잡고 있으면
        #   executor 가 막혀 spot_states 하트비트가 끊긴다.
        threading.Thread(target=self.process_request, args=(req,),
                         daemon=True).start()

    # ! 이름을 handle 로 두면 안 된다. rclpy Node 의 같은 이름 속성을 가려서
    #   super().__init__ 안에서 그걸 컨텍스트 매니저로 쓸 때 터진다.
    #     TypeError: 'method' object does not support the context manager protocol
    def process_request(self, req):
        rid = int(req.get('request_id', 0))
        kind = str(req.get('kind', 'PARK')).upper()
        tag = req.get('vehicle_tag')
        want = req.get('spot_id')

        # ! 같은 request_id 가 다시 오면 조용히 무시한다. 실패로 처리하면 안 된다.
        #   rosbridge/네트워크 재전송이나 대시보드의 중복 클릭으로 흔히 온다.
        #   백엔드도 UNIQUE(request_id, seq) + INSERT IGNORE 로 흡수하고 있다 (#14).
        with self._lock:
            if rid in self._seen:
                self.get_logger().info('중복 요청 %d — 무시' % rid)
                return
            self._seen.add(rid)
        if self._busy:
            self.emit(rid, 'FAILED', 'SpotManager',
                      {'reason': '다른 요청 처리 중'})
            return
        self._busy = True
        try:
            self.emit(rid, 'REQUEST_ACCEPTED', 'SpotManager',
                      {'kind': kind, 'vehicle_tag': tag})
            if kind == 'PARK':
                self.do_park(rid, want, tag)
            elif kind == 'RETRIEVE':
                self.do_retrieve(rid, want, tag)
            else:
                self.emit(rid, 'FAILED', 'SpotManager',
                          {'reason': 'kind 가 PARK/RETRIEVE 가 아님: %s' % kind})
        finally:
            self._busy = False

    def do_park(self, rid, want, tag):
        spot_id, err = self.pick_spot(want)
        if spot_id is None:
            self.emit(rid, 'FAILED', 'FindParkingSpot', {'reason': err})
            return
        self.emit(rid, 'SPOT_SELECTED', 'FindParkingSpot', {'spot_id': spot_id})
        self.set_state(spot_id, RESERVED, rid, tag)
        self.emit(rid, 'SPOT_RESERVED', 'FindParkingSpot', {'spot_id': spot_id})

        res, err = self.run_action(spot_id, Park.Goal.PARK, rid)
        if res is None or not res.success:
            self.set_state(spot_id, FREE, None)     # 예약 해제
            self.emit(rid, 'FAILED', 'ParkManeuver',
                      {'spot_id': spot_id,
                       'reason': err or getattr(res, 'message', '')})
            return
        self.set_state(spot_id, OCCUPIED, rid, tag)
        self.emit(rid, 'PARK_DONE', 'ParkManeuver', {
            'spot_id': spot_id,
            'err_m': round(res.err_m, 3),
            'heading_deg': round(res.err_deg, 2),
            'shunts': int(res.shunts_park),
            'inside_rect': bool(res.inside_rect),
            'duration_s': round(res.duration_s, 1),
        })

    def do_retrieve(self, rid, want, tag):
        spot_id = want
        if not spot_id:
            with self._lock:
                cands = [k for k, v in self.spots.items()
                         if v['status'] == OCCUPIED and
                         (tag is None or v['vehicle_tag'] == tag)]
            # 우리가 넣은 차(vehicle_tag 기록이 있는 것)를 우선한다
            with self._lock:
                ours = [k for k in cands if self.spots[k]['vehicle_tag']]
            spot_id = (ours or cands or [None])[0]
        if spot_id is None or self.spots.get(spot_id, {}).get(
                'status') != OCCUPIED:
            self.emit(rid, 'FAILED', 'SpotManager',
                      {'reason': '출차할 차량을 찾지 못함', 'spot_id': spot_id})
            return

        res, err = self.run_action(spot_id, Park.Goal.UNPARK, rid)
        if res is None or not res.success:
            self.emit(rid, 'FAILED', 'UnparkManeuver',
                      {'spot_id': spot_id,
                       'reason': err or getattr(res, 'message', '')})
            return
        self.set_state(spot_id, FREE, None, None)
        self.emit(rid, 'UNPARK_DONE', 'UnparkManeuver',
                  {'spot_id': spot_id,
                   'shunts': int(res.shunts_park),
                   'duration_s': round(res.duration_s, 1)})
        self.emit(rid, 'EXIT_REACHED', 'UnparkManeuver', {'spot_id': spot_id})


def main():
    rclpy.init()
    n = SpotManager()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
