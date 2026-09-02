#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Prius Hybrid(OBJ)를 valet_car 제원에 맞춰 차체/바퀴 메시로 분리·변형한다.

원본: Gazebo Fuel, OpenRobotics/Prius Hybrid, CC0 1.0 (퍼블릭 도메인)

원본과 우리 제원이 다르다.

              전장     전폭    전고    축거    윤거    바퀴반경
  Prius      4.618    2.012   1.386  2.865   1.537   0.305
  valet_car  4.50     1.90    1.10*  2.50    1.38    0.33
  (* 차체 높이. 지붕 1.34 = 라이다 1.42 아래여야 한다)

그냥 균일 스케일하면 축거가 안 맞아 바퀴가 휠아치에서 15 cm 어긋난다.
그래서 길이축을 **구간별 선형 리매핑** 한다.

  앞 오버행 (노즈 ~ 앞축)   0.877 -> 1.00  (1.141 배 늘림)
  축간      (앞축 ~ 뒷축)   2.865 -> 2.50  (0.873 배 줄임)
  뒤 오버행 (뒷축 ~ 테일)   0.877 -> 1.00  (1.141 배 늘림)

이러면 전장 4.50, 축거 2.50 이 정확히 맞고 휠아치도 바퀴 위에 온다.

  python3 tools/fit_car_mesh.py <원본 Hybrid.obj> <출력 디렉터리>
"""
import os
import sys

# ---- valet_car 목표 제원 (common.xacro 와 일치시킬 것) --------------------
LENGTH = 4.50
WIDTH = 1.90
WHEELBASE = 2.50
BODY_BOTTOM = 0.24        # 절대 높이
ROOF = 1.34               # 절대 높이 (라이다 1.42 보다 낮아야 한다)
WHEEL_R = 0.33
WHEEL_W = 0.22

BODY_GROUPS = ('Hybrid', 'Hybrid_Interior', 'Hybrid_Windows')
WHEEL_GROUP = 'Wheel_Front_Left_'

SRC = sys.argv[1]
DST = sys.argv[2] if len(sys.argv) > 2 else '.'


def load(path):
    """OBJ 를 읽어 (정점, 법선, uv, 그룹별 면) 으로 돌려준다."""
    v, vn, vt, groups, cur = [], [], [], {}, None
    for line in open(path, encoding='utf-8', errors='ignore'):
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            v.append([float(x) for x in p[1:4]])
        elif p[0] == 'vn':
            vn.append([float(x) for x in p[1:4]])
        elif p[0] == 'vt':
            vt.append([float(x) for x in p[1:3]])
        elif p[0] in ('o', 'g'):
            cur = ' '.join(p[1:])
            groups.setdefault(cur, {'f': [], 'mtl': None})
        elif p[0] == 'usemtl' and cur:
            groups[cur]['mtl'] = p[1]
        elif p[0] == 'f' and cur:
            groups[cur]['f'].append(p[1:])
    return v, vn, vt, groups


def idx_of(group, v, groups):
    out = set()
    for face in groups[group]['f']:
        for tok in face:
            i = int(tok.split('/')[0])
            out.add(i - 1 if i > 0 else len(v) + i)
    return out


def bbox(pts):
    return ([min(p[k] for p in pts) for k in range(3)],
            [max(p[k] for p in pts) for k in range(3)])


def write_obj(path, v, vn, vt, groups, names, mtllib):
    """지정한 그룹만 새 OBJ 로 쓴다. 인덱스를 새로 매긴다."""
    used_v, used_vn, used_vt = {}, {}, {}
    lines = []
    for g in names:
        lines.append('g %s' % g)
        if groups[g]['mtl']:
            lines.append('usemtl %s' % groups[g]['mtl'])
        for face in groups[g]['f']:
            toks = []
            for tok in face:
                parts = (tok.split('/') + ['', ''])[:3]
                vi = int(parts[0]); vi = vi - 1 if vi > 0 else len(v) + vi
                if vi not in used_v:
                    used_v[vi] = len(used_v) + 1
                s = str(used_v[vi])
                ti = ni = ''
                if parts[1]:
                    t = int(parts[1]); t = t - 1 if t > 0 else len(vt) + t
                    if t not in used_vt:
                        used_vt[t] = len(used_vt) + 1
                    ti = str(used_vt[t])
                if parts[2]:
                    nn = int(parts[2]); nn = nn - 1 if nn > 0 else len(vn) + nn
                    if nn not in used_vn:
                        used_vn[nn] = len(used_vn) + 1
                    ni = str(used_vn[nn])
                toks.append(s + ('/' + ti if (ti or ni) else '') +
                            ('/' + ni if ni else ''))
            lines.append('f ' + ' '.join(toks))

    head = ['# valet_robot 용으로 변형한 메시',
            '# 원본: Gazebo Fuel OpenRobotics/Prius Hybrid (CC0 1.0)',
            '# tools/fit_car_mesh.py 로 생성 — 직접 편집하지 말 것',
            'mtllib %s' % mtllib]
    for vi in sorted(used_v, key=used_v.get):
        head.append('v %.6f %.6f %.6f' % tuple(v[vi]))
    for ti in sorted(used_vt, key=used_vt.get):
        head.append('vt %.6f %.6f' % tuple(vt[ti]))
    for ni in sorted(used_vn, key=used_vn.get):
        head.append('vn %.6f %.6f %.6f' % tuple(vn[ni]))
    open(path, 'w', encoding='utf-8').write('\n'.join(head + lines) + '\n')
    return len(used_v), sum(len(groups[g]['f']) for g in names)


def main():
    v, vn, vt, groups = load(SRC)
    print('원본: 정점 %d, 그룹 %s' % (len(v), list(groups)))

    S = 0.01                       # 원본은 cm 단위
    # 메시 좌표계: X=폭, Y=길이, Z=높이.  ROS: X=길이, Y=폭, Z=높이
    # 먼저 cm->m 스케일만 하고 축은 나중에 바꾼다.
    for p in v:
        p[0] *= S; p[1] *= S; p[2] *= S

    fl = [v[i] for i in idx_of('Wheel_Front_Left_', v, groups)]
    rr = [v[i] for i in idx_of('Wheels_Rear', v, groups)]
    body = [v[i] for i in idx_of('Hybrid', v, groups)]
    y_front = sum(p[1] for p in fl) / len(fl)
    y_rear = sum(p[1] for p in rr) / len(rr)
    bmin, bmax = bbox(body)
    print('  앞축 y=%.3f  뒷축 y=%.3f  축거=%.3f' % (y_front, y_rear,
                                                    abs(y_rear - y_front)))
    print('  차체 bbox  x %.3f~%.3f  y %.3f~%.3f  z %.3f~%.3f'
          % (bmin[0], bmax[0], bmin[1], bmax[1], bmin[2], bmax[2]))

    # 길이축(메시 y)을 구간별 선형 리매핑.  원본은 y+ 가 뒤쪽이다.
    nose, tail = bmin[1], bmax[1]
    f_over = abs(y_front - nose)      # 앞 오버행
    r_over = abs(tail - y_rear)       # 뒤 오버행
    tgt_over = (LENGTH - WHEELBASE) / 2.0
    k_mid = WHEELBASE / abs(y_rear - y_front)
    k_f = tgt_over / f_over
    k_r = tgt_over / r_over
    print('  리매핑: 앞오버행 x%.3f, 축간 x%.3f, 뒤오버행 x%.3f'
          % (k_f, k_mid, k_r))

    def remap_len(y):
        if y < y_front:
            return -WHEELBASE / 2.0 - (y_front - y) * k_f
        if y > y_rear:
            return WHEELBASE / 2.0 + (y - y_rear) * k_r
        return -WHEELBASE / 2.0 + (y - y_front) * k_mid

    sy = WIDTH / (bmax[0] - bmin[0])                 # 폭
    sz = (ROOF - BODY_BOTTOM) / (bmax[2] - bmin[2])  # 높이
    print('  폭 스케일 x%.3f, 높이 스케일 x%.3f' % (sy, sz))

    wl = [v[i] for i in idx_of(WHEEL_GROUP, v, groups)]
    wmin, wmax = bbox(wl)
    wc = [(wmin[k] + wmax[k]) / 2 for k in range(3)]
    w_r = (wmax[2] - wmin[2]) / 2.0
    print('  원본 바퀴 반경 %.3f -> %.3f' % (w_r, WHEEL_R))

    # --- 바퀴: 원점 중심, 회전축이 y 가 되도록 (URDF 바퀴 규약) ----------
    wv = [list(p) for p in v]
    for p in wv:
        x = (p[0] - wc[0]) * (WHEEL_W / (wmax[0] - wmin[0]))
        z = (p[2] - wc[2]) * (WHEEL_R / w_r)
        yy = (p[1] - wc[1]) * (WHEEL_R / w_r)
        p[0], p[1], p[2] = yy, x, z       # 메시 y(길이) -> ROS x
    write_obj(os.path.join(DST, 'wheel.obj'), wv, vn, vt, groups,
              [WHEEL_GROUP], 'valet_car.mtl')

    # --- 차체: 길이 리매핑 + 축 변환 -------------------------------------
    bv = [list(p) for p in v]
    for p in bv:
        L = remap_len(p[1])
        W = (p[0] - (bmin[0] + bmax[0]) / 2) * sy
        H = BODY_BOTTOM + (p[2] - bmin[2]) * sz
        p[0], p[1], p[2] = -L, W, H       # -L: 원본 y+ 가 뒤쪽이라 뒤집는다
    nv, nf = write_obj(os.path.join(DST, 'body.obj'), bv, vn, vt, groups,
                       list(BODY_GROUPS), 'valet_car.mtl')

    fin = [bv[i] for i in idx_of('Hybrid', bv, groups)]
    fmin, fmax = bbox(fin)
    print('\n결과 차체: 전장 %.3f  전폭 %.3f  높이 %.3f~%.3f'
          % (fmax[0] - fmin[0], fmax[1] - fmin[1], fmin[2], fmax[2]))
    print('  body.obj  정점 %d, 면 %d' % (nv, nf))


if __name__ == '__main__':
    main()
