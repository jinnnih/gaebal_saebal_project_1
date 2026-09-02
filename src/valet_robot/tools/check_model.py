#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""valet_car 모델 자기검증.

xacro 로 URDF 를 펼친 뒤
  1) 링크/조인트 트리 구조
  2) 차량 기구학 (축거 / 윤거 / 최대조향 / 최소회전반경)
  3) 차체 치수와 링크 간 충돌 간섭
  4) 라이다 장착 높이 vs 주차장 월드 장애물 높이
  5) 관성 유효성
  6) ros2_control 인터페이스 조합
  7) parking_lot_world 의 Nav2 파라미터와의 정합성   <- 가장 중요
을 검사한다. 기하를 손대면 반드시 다시 돌릴 것.

  python3 tools/check_model.py            # 소스 트리에서
  ros2 run valet_robot check_model.py     # 설치 후
"""
import json
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

WHEEL_R = 0.33
WHEEL_W = 0.22

# parking_lot_world/worlds/parking_lot.sdf 의 장애물 z 범위
WORLD_OBSTACLES = [
    ('parked_car 차체', 0.27, 0.99),
    ('parked_car 캐빈', 0.99, 1.51),
    ('벽 / 기둥',       0.00, 2.60),
    ('EV 충전기',       0.00, 1.30),
    ('게이트',          0.00, 1.20),
    ('라바콘',          0.00, 0.59),
]

FAILURES = []


def ok(msg):
    print('  [ok]   %s' % msg)


def bad(msg):
    FAILURES.append(msg)
    print('  [FAIL] %s' % msg)


def head(msg):
    print('\n== %s ==' % msg)


def expand(xacro_file):
    last = None
    # controllers_file 을 명시해 $(find valet_robot) 평가를 피한다.
    # 덕분에 colcon build / source 없이 소스 트리에서 그대로 돌릴 수 있다.
    cfg = os.path.join(os.path.dirname(os.path.dirname(xacro_file)),
                       'config', 'controllers.yaml')
    args = [xacro_file, 'sim:=true', 'controllers_file:=' + cfg]
    inline = ('import sys,xacro;sys.argv=["xacro"]+%r;xacro.main()' % args)
    for argv in (['xacro'] + args,
                 [sys.executable, '-c', inline]):
        try:
            out = subprocess.run(argv, check=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            return out.stdout.decode('utf-8')
        except (OSError, subprocess.CalledProcessError) as e:
            last = e
    print('xacro 실행 실패:', last)
    sys.exit(2)


def find_world_pkg():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('parking_lot_world')
    except Exception:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cand = os.path.join(os.path.dirname(here), 'parking_lot_world')
        return cand if os.path.isdir(cand) else None


def origin(e):
    o = e.find('origin')
    if o is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(v) for v in o.get('xyz', '0 0 0').split())


def build_tree(root):
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    head('1. 링크 / 조인트 트리')
    print('  링크 %d, 조인트 %d' % (len(links), len(joints)))
    children = set()
    for n, j in joints.items():
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        if p not in links:
            bad('조인트 %s 의 parent %s 가 없음' % (n, p))
        if c not in links:
            bad('조인트 %s 의 child %s 가 없음' % (n, c))
        if c in children:
            bad('링크 %s 가 두 조인트의 child (트리 아님)' % c)
        children.add(c)
    roots = set(links) - children
    if roots == {'base_footprint'}:
        ok('루트 링크 = base_footprint')
    else:
        bad('루트 링크가 %s (base_footprint 여야 함)' % sorted(roots))

    absxyz = {}

    def resolve(link):
        if link in absxyz:
            return absxyz[link]
        for j in joints.values():
            if j.find('child').get('link') == link:
                p = resolve(j.find('parent').get('link'))
                o = origin(j)
                absxyz[link] = tuple(p[i] + o[i] for i in range(3))
                return absxyz[link]
        absxyz[link] = (0.0, 0.0, 0.0)
        return absxyz[link]

    for l in links:
        resolve(l)
    return links, joints, absxyz


def check_kinematics(joints, absxyz):
    head('2. 차량 기구학')
    fl = absxyz['front_left_steer_link']
    rl = absxyz['rear_left_wheel_link']
    wheelbase = fl[0] - rl[0]
    track = 2.0 * abs(fl[1])
    lim = joints['front_left_steer_joint'].find('limit')
    steer_limit = float(lim.get('upper'))

    # 조인트 리밋은 "안쪽 바퀴" 각도다. 자전거 모델 각도가 아니다.
    #   R_inner_axle = L / tan(delta_inner)
    #   R_center     = R_inner_axle + track/2      (후륜축 중심 기준)
    r_achievable = wheelbase / math.tan(steer_limit) + track / 2.0
    delta_bicycle = math.atan(wheelbase / r_achievable)

    print('  축거 %.4f m | 윤거 %.4f m' % (wheelbase, track))
    print('  조향 조인트 리밋(안쪽 바퀴) %.2f deg' % math.degrees(steer_limit))
    print('  -> 물리적으로 가능한 최소회전반경 %.4f m '
          '(자전거 모델 환산 %.2f deg)'
          % (r_achievable, math.degrees(delta_bicycle)))
    if abs(float(lim.get('lower')) + steer_limit) < 1e-9:
        ok('조향 리밋 좌우 대칭')
    else:
        bad('조향 리밋이 비대칭')
    return wheelbase, track, steer_limit, r_achievable


def check_body(links, absxyz, wheelbase, track, steer_limit):
    head('3. 차체 치수와 바퀴 간섭')
    boxes = []
    for name, l in links.items():
        if 'wheel' in name:
            continue
        for e in l.findall('collision'):
            b = e.find('geometry/box')
            if b is None:
                continue
            sz = [float(v) for v in b.get('size').split()]
            o, a = origin(e), absxyz[name]
            boxes.append((e.get('name') or name,
                          [a[i] + o[i] for i in range(3)], sz))
    xmax = max(c[0] + s[0] / 2 for _, c, s in boxes)
    xmin = min(c[0] - s[0] / 2 for _, c, s in boxes)
    ymax = max(c[1] + s[1] / 2 for _, c, s in boxes)
    ymin = min(c[1] - s[1] / 2 for _, c, s in boxes)
    zmax = max(c[2] + s[2] / 2 for _, c, s in boxes)
    length, width = xmax - xmin, ymax - ymin
    print('  전장 %.3f m | 전폭 %.3f m | 전고 %.3f m (충돌 형상 기준)'
          % (length, width, zmax))
    print('  차체 충돌 박스 %d 개' % len(boxes))
    if abs(xmax + xmin) < 1e-6:
        ok('base_link 가 차체 중심 (앞/뒤 오버행 각 %.2f m)'
           % (xmax - wheelbase / 2))
    else:
        bad('차체가 base_link 기준 비대칭 — 주차면 중심 goal_pose 와 안 맞는다')

    def wheel_aabb(name):
        a = absxyz[name]
        st = steer_limit if 'front' in name else 0.0
        ex = WHEEL_R * math.cos(st) + WHEEL_W / 2 * math.sin(st)
        ey = WHEEL_R * math.sin(st) + WHEEL_W / 2 * math.cos(st)
        return (a[0] - ex, a[0] + ex, a[1] - ey, a[1] + ey,
                a[2] - WHEEL_R, a[2] + WHEEL_R)

    hits = []
    for wn in [n for n in links if n.endswith('wheel_link')]:
        wx0, wx1, wy0, wy1, wz0, wz1 = wheel_aabb(wn)
        for bn, c, s in boxes:
            ox = min(c[0] + s[0] / 2, wx1) - max(c[0] - s[0] / 2, wx0)
            oy = min(c[1] + s[1] / 2, wy1) - max(c[1] - s[1] / 2, wy0)
            oz = min(c[2] + s[2] / 2, wz1) - max(c[2] - s[2] / 2, wz0)
            if ox > 1e-9 and oy > 1e-9 and oz > 1e-9:
                hits.append((wn, bn, ox, oy, oz))
    if hits:
        for wn, bn, ox, oy, oz in hits[:4]:
            bad('%s 가 %s 와 겹침 (x %.3f y %.3f z %.3f)' % (bn, wn, ox, oy, oz))
    else:
        ok('전 바퀴(최대조향 포락선 포함)가 차체와 안 겹침 — 휠아치 확보')

    steer_out = (abs(absxyz['front_left_wheel_link'][1])
                 + WHEEL_R * math.sin(steer_limit)
                 + WHEEL_W / 2 * math.cos(steer_limit))
    print('  최대조향 시 앞바퀴 바깥끝 y = %.4f m' % steer_out)
    return length, width, zmax, steer_out


def check_lidar(absxyz, roof_top):
    head('4. 라이다 장착 높이')
    if 'lidar_link' not in absxyz:
        print('  라이다 없음 (lidar:=false) — 건너뜀')
        return
    lz = absxyz['lidar_link'][2]
    print('  절대 높이 %.3f m  (자기 차체 최상단 %.3f m)' % (lz, roof_top))
    for name, low, high in WORLD_OBSTACLES:
        print('    %-16s %.2f ~ %.2f m  ->  %s'
              % (name, low, high, '감지' if low < lz < high else '통과(미감지)'))
    if 0.27 < lz < 0.99:
        ok('주차 차량을 "차체"(4.4 x 1.8)로 본다 — 외형 정확도 최상')
    elif 0.99 < lz < 1.51:
        ok('주차 차량을 "캐빈"(2.2 x 1.66)으로 본다 — 빈 주차면 탐색에는 충분')
        print('         ! 차량 외형을 앞뒤로 약 1.1 m 씩 작게 본다')
    else:
        bad('주차 차량을 전혀 못 본다 — 빈 주차면 탐색이 무의미해진다')
    if lz > roof_top:
        ok('자기 차체 최상단(%.3f) 위 — 360도 자기가림 없음' % roof_top)
    else:
        bad('라이다가 자기 차체(%.3f)에 가린다' % roof_top)


def check_inertia(links):
    head('5. 관성')
    total = 0.0
    for name, l in links.items():
        i = l.find('inertial')
        if i is None:
            continue
        total += float(i.find('mass').get('value'))
        n = i.find('inertia')
        ixx, iyy, izz = (float(n.get(k)) for k in ('ixx', 'iyy', 'izz'))
        if min(ixx, iyy, izz) <= 0:
            bad('%s 의 관성모멘트가 0 이하' % name)
        elif not (ixx + iyy >= izz and iyy + izz >= ixx and izz + ixx >= iyy):
            bad('%s 관성 삼각부등식 위반' % name)
    print('  총 질량 %.1f kg' % total)
    ok('전 링크 관성 유효')


def check_ros2_control(root):
    head('6. ros2_control 인터페이스')
    rc = root.find('ros2_control')
    if rc is None:
        bad('ros2_control 블록이 없다 (sim:=true 인데)')
        return
    for n in ('front_left_steer_joint', 'front_right_steer_joint'):
        ci = rc.find("joint[@name='%s']/command_interface" % n)
        if ci is None or ci.get('name') != 'position':
            bad('%s 에 position 명령 인터페이스가 없다' % n)
    for n in ('rear_left_wheel_joint', 'rear_right_wheel_joint'):
        ci = rc.find("joint[@name='%s']/command_interface" % n)
        if ci is None or ci.get('name') != 'velocity':
            bad('%s 에 velocity 명령 인터페이스가 없다' % n)
    ok('조향=position / 구동=velocity (ackermann_steering_controller 요구사항)')


def check_map_pkg(length, width, wheelbase, track, steer_limit, r_achievable,
                  steer_out):
    head('7. parking_lot_world 정합성')
    wp = find_world_pkg()
    if not wp:
        print('  parking_lot_world 를 못 찾아 건너뜀')
        return
    spec_file = os.path.join(wp, 'config', 'parking_spots.json')
    nav_file = os.path.join(wp, 'config', 'nav2_ackermann.yaml')

    r_required = None
    if os.path.isfile(spec_file):
        with open(spec_file, encoding='utf-8') as f:
            spec = json.load(f)['robot_spec']
        r_required = spec['min_turning_radius']
        for key, mine, want in (('length', length, spec['length']),
                                ('width', width, spec['width']),
                                ('wheelbase', wheelbase, spec['wheelbase'])):
            if abs(mine - want) < 1e-3:
                ok('%-18s %.4f == parking_spots.json' % (key, mine))
            else:
                bad('%-18s URDF %.4f != 맵 패키지 %.4f' % (key, mine, want))

        # 요구 회전반경을 실제로 낼 수 있는가 (안쪽 바퀴가 포화되지 않는가)
        need_inner = math.atan(wheelbase / (r_required - track / 2.0))
        if steer_limit >= need_inner:
            ok('안쪽 바퀴 소요각 %.2f deg <= 조인트 리밋 %.2f deg (여유 %.2f deg)'
               % (math.degrees(need_inner), math.degrees(steer_limit),
                  math.degrees(steer_limit - need_inner)))
        else:
            bad('안쪽 바퀴가 %.2f deg 필요한데 리밋이 %.2f deg — 포화되어 '
                '최소회전반경 %.3f m 를 낼 수 없다'
                % (math.degrees(need_inner), math.degrees(steer_limit),
                   r_required))
        if r_achievable <= r_required + 1e-6:
            ok('가능 최소회전반경 %.3f m <= 요구 %.3f m' % (r_achievable, r_required))
        else:
            bad('가능 최소회전반경 %.3f m > 요구 %.3f m' % (r_achievable, r_required))
    else:
        print('  parking_spots.json 없음 — 제원 대조 생략')

    if not os.path.isfile(nav_file):
        print('  nav2_ackermann.yaml 없음 — Nav2 대조 생략')
        return
    with open(nav_file, encoding='utf-8') as f:
        txt = f.read()
    for label, pat in (('Smac minimum_turning_radius',
                        r'minimum_turning_radius:\s*([0-9.]+)'),
                       ('MPPI min_turning_r', r'min_turning_r:\s*([0-9.]+)')):
        m = re.search(pat, txt)
        if not m or r_required is None:
            continue
        val = float(m.group(1))
        if abs(val - r_required) < 1e-2:
            ok('%s %.3f == parking_spots.json' % (label, val))
        else:
            bad('%s %.3f != %.3f' % (label, val, r_required))
    m = re.search(r'footprint:\s*"\[\s*\[\s*([0-9.\-]+),\s*([0-9.\-]+)', txt)
    if m:
        fx, fy = float(m.group(1)), float(m.group(2))
        if fx * 2 >= length and fy * 2 >= width:
            ok('Nav2 footprint %.2f x %.2f 가 차체 %.2f x %.2f 를 포함'
               % (fx * 2, fy * 2, length, width))
        else:
            bad('Nav2 footprint %.2f x %.2f 가 차체보다 작다' % (fx * 2, fy * 2))
        if steer_out > fy:
            bad('최대조향 시 앞바퀴 바깥끝 %.3f 가 footprint 반폭 %.3f 초과'
                % (steer_out, fy))
        else:
            ok('최대조향에도 footprint 반폭 %.2f 안쪽 (여유 %.3f m)'
               % (fy, fy - steer_out))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    xacro_file = os.path.join(os.path.dirname(here), 'urdf',
                              'valet_car.urdf.xacro')
    if not os.path.isfile(xacro_file):
        print('URDF xacro 를 못 찾음:', xacro_file)
        return 2

    root = ET.fromstring(expand(xacro_file))
    links, joints, absxyz = build_tree(root)
    wheelbase, track, steer_limit, r_achievable = check_kinematics(joints, absxyz)
    length, width, roof_top, steer_out = check_body(
        links, absxyz, wheelbase, track, steer_limit)
    check_lidar(absxyz, roof_top)
    check_inertia(links)
    check_ros2_control(root)
    check_map_pkg(length, width, wheelbase, track, steer_limit,
                  r_achievable, steer_out)

    print('\n' + '=' * 62)
    if FAILURES:
        print('실패 %d 건:' % len(FAILURES))
        for f in FAILURES:
            print('  - %s' % f)
        return 1
    print('전체 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
