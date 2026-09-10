#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""대시보드 흉내 — /valet/request 를 보내고 /valet/mission_status 를 지켜본다.

    python3 request_test.py PARK [주차면ID]
    python3 request_test.py RETRIEVE

#9 계약대로만 말한다. 실제 대시보드가 rosbridge 로 하는 것과 같다.
"""
import json
import sys
import time

import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

KIND = (sys.argv[1] if len(sys.argv) > 1 else 'PARK').upper()
SPOT = sys.argv[2] if len(sys.argv) > 2 else None
RID = int(time.time()) % 100000


def main():
    rclpy.init()
    n = rclpy.create_node('request_test')
    n.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    pub = n.create_publisher(String, '/valet/request', 10)

    events = []
    n.create_subscription(String, '/valet/mission_status',
                          lambda m: events.append(json.loads(m.data)), 20)
    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    spots = []
    n.create_subscription(String, '/valet/spot_states',
                          lambda m: spots.append(json.loads(m.data)), latched)

    t = time.time()
    while time.time() - t < 5 and not spots:
        rclpy.spin_once(n, timeout_sec=0.2)
    if spots:
        from collections import Counter
        c = Counter(s['status'] for s in spots[-1]['spots'])
        print('요청 전 점유:', dict(c))

    req = {'request_id': RID, 'kind': KIND,
           'vehicle_tag': 'DEMO-01', 'spot_id': SPOT}
    # ! 여러 번 쏘지 않는다. 예전에 전달 보장하려고 5 번 쐈다가 중복 요청이
    #   되어 "다른 요청 처리 중" 실패가 났다. 구독자가 붙은 뒤 한 번만 쏜다.
    t = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    print('요청 전송:', json.dumps(req, ensure_ascii=False))
    pub.publish(String(data=json.dumps(req)))
    for _ in range(5):
        rclpy.spin_once(n, timeout_sec=0.2)

    print()
    seen = 0
    t = time.time()
    while rclpy.ok() and time.time() - t < 700:
        rclpy.spin_once(n, timeout_sec=0.3)
        while seen < len(events):
            e = events[seen]; seen += 1
            print('  [%2d] %-16s %-16s %s'
                  % (e['seq'], e['event'], e['bt_node'],
                     json.dumps(e['payload'], ensure_ascii=False)))
            if e['event'] in ('PARK_DONE', 'EXIT_REACHED', 'FAILED',
                              'ABORTED'):
                # ! sleep 만 하면 안 된다. 그 사이 온 스냅샷을 처리하지 않아
                #   낡은 상태를 읽는다 (실제로 OCCUPIED 인데 RESERVED 로 봤다).
                tt = time.time()
                while time.time() - tt < 2.0:
                    rclpy.spin_once(n, timeout_sec=0.2)
                if spots:
                    from collections import Counter
                    c = Counter(s['status'] for s in spots[-1]['spots'])
                    mine = [s for s in spots[-1]['spots']
                            if s['request_id'] == RID]
                    print()
                    print('요청 후 점유:', dict(c))
                    if mine:
                        print('내 요청의 주차면:', mine)
                rclpy.shutdown(); return 0
    print('  시간 초과')
    rclpy.shutdown(); return 1


if __name__ == '__main__':
    sys.exit(main())
