#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDF 를 파싱해 차량 형상 미리보기 PNG 를 만든다 (Gazebo 없이 확인용).

  python3 tools/preview_model.py [출력.png] [조향각deg]

측면 / 정면 / 평면 / 등각 4분할. 실제 URDF 의 visual 박스·원통을 그대로 그린다.
"""
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

OUT = sys.argv[1] if len(sys.argv) > 1 else 'model_preview.png'
STEER = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
XACRO = os.path.join(PKG, 'urdf', 'valet_car.urdf.xacro')
CFG = os.path.join(PKG, 'config', 'controllers.yaml')

COLORS = {}


def expand():
    args = [XACRO, 'sim:=true', 'controllers_file:=' + CFG]
    inline = 'import sys,xacro;sys.argv=["xacro"]+%r;xacro.main()' % args
    for argv in (['xacro'] + args, [sys.executable, '-c', inline]):
        try:
            return subprocess.run(argv, check=True,
                                  stdout=subprocess.PIPE).stdout.decode('utf-8')
        except (OSError, subprocess.CalledProcessError):
            pass
    sys.exit('xacro 실행 실패')


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mul(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def collect(root):
    """(중심, 회전행렬, 반크기, 색, 종류) 리스트를 링크 절대좌표로 만든다."""
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    for m in root.findall('material'):
        c = m.find('color')
        if c is not None:
            COLORS[m.get('name')] = [float(v) for v in c.get('rgba').split()]

    world = {}

    def resolve(name):
        if name in world:
            return world[name]
        for j in joints:
            if j.find('child').get('link') == name:
                p = resolve(j.find('parent').get('link'))
                o = j.find('origin')
                xyz = [float(v) for v in (o.get('xyz', '0 0 0').split()
                                          if o is not None else '0 0 0'.split())]
                rpy = [float(v) for v in (o.get('rpy', '0 0 0').split()
                                          if o is not None else '0 0 0'.split())]
                R = matmul(p[1], rpy_mat(*rpy))
                # 조향 조인트는 STEER 만큼 더 돌린다
                if j.get('name', '').endswith('steer_joint'):
                    R = matmul(R, rpy_mat(0, 0, STEER))
                world[name] = ([p[0][i] + mul(p[1], xyz)[i] for i in range(3)], R)
                return world[name]
        world[name] = ([0.0, 0.0, 0.0], rpy_mat(0, 0, 0))
        return world[name]

    prims = []
    for name, l in links.items():
        lp, lR = resolve(name)
        for v in l.findall('visual'):
            o = v.find('origin')
            xyz = [float(t) for t in (o.get('xyz', '0 0 0').split()
                                      if o is not None else '0 0 0'.split())]
            rpy = [float(t) for t in (o.get('rpy', '0 0 0').split()
                                      if o is not None else '0 0 0'.split())]
            c = [lp[i] + mul(lR, xyz)[i] for i in range(3)]
            R = matmul(lR, rpy_mat(*rpy))
            mat = v.find('material')
            col = COLORS.get(mat.get('name') if mat is not None else '',
                             [0.6, 0.6, 0.6, 1])
            b = v.find('geometry/box')
            cy = v.find('geometry/cylinder')
            me = v.find('geometry/mesh')
            if b is not None:
                h = [float(t) / 2 for t in b.get('size').split()]
                prims.append((c, R, h, col, 'box'))
            elif cy is not None:
                r = float(cy.get('radius')); ln = float(cy.get('length'))
                prims.append((c, R, [r, r, ln / 2], col, 'cyl'))
            elif me is not None:
                mv, mf = load_obj(me.get('filename'))
                if mv:
                    prims.append((c, R, (mv, mf), col, 'mesh'))
    return prims


CORNERS = [(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
         (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]

NSEG = 20


MESH_CACHE = {}


def load_obj(uri):
    """package://valet_robot/meshes/x.obj 를 읽어 (정점, 삼각형면) 반환."""
    path = uri.replace('package://valet_robot/', PKG + '/')
    if path in MESH_CACHE:
        return MESH_CACHE[path]
    if not os.path.isfile(path):
        MESH_CACHE[path] = ([], [])
        return MESH_CACHE[path]
    verts, faces = [], []
    for line in open(path, encoding='utf-8', errors='ignore'):
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            verts.append([float(x) for x in p[1:4]])
        elif p[0] == 'f':
            idx = []
            for tok in p[1:]:
                i = int(tok.split('/')[0])
                idx.append(i - 1 if i > 0 else len(verts) + i)
            for k in range(1, len(idx) - 1):      # 팬 삼각형화
                faces.append((idx[0], idx[k], idx[k + 1]))
    MESH_CACHE[path] = (verts, faces)
    return MESH_CACHE[path]


def mesh(h, kind):
    """로컬 좌표계의 (정점, 면) 반환. 원통은 z 축이 축방향."""
    if kind == 'mesh':
        return h                      # (정점, 면) 그대로
    if kind == 'box':
        return ([[h[0] * s[0], h[1] * s[1], h[2] * s[2]] for s in CORNERS],
                FACES)
    r, hz = h[0], h[2]
    verts, faces = [], []
    for i in range(NSEG):
        a = 2 * math.pi * i / NSEG
        verts.append([r * math.cos(a), r * math.sin(a), -hz])
        verts.append([r * math.cos(a), r * math.sin(a), hz])
    for i in range(NSEG):
        j = (i + 1) % NSEG
        faces.append((2 * i, 2 * j, 2 * j + 1, 2 * i + 1))
    faces.append(tuple(range(0, 2 * NSEG, 2)))
    faces.append(tuple(range(1, 2 * NSEG, 2)))
    return verts, faces


def draw(ax, prims, proj, shade=True):
    """proj: (u축, v축) 인덱스. 깊이 순으로 정렬해 뒤에서부터 그린다."""
    u, v = proj
    depth_ax = ({0, 1, 2} - {u, v}).pop()
    items = []
    for c, R, h, col, kind in prims:
        lv, faces = mesh(h, kind)
        pts = [[c[i] + sum(R[i][k] * p[k] for k in range(3))
                for i in range(3)] for p in lv]
        items.append((c[depth_ax], pts, col, faces))
    items.sort(key=lambda t: t[0])
    for _, pts, col, faces in items:
        for f in faces:
            poly = [(pts[i][u], pts[i][v]) for i in f]
            ax.add_patch(Polygon(poly, closed=True,
                                 facecolor=col[:3], edgecolor=(0, 0, 0, 0.35),
                                 linewidth=0.4, alpha=col[3] if len(col) > 3 else 1))


def iso_project(prims, az=math.radians(35), el=math.radians(22)):
    """등각 투영: 3D 점을 2D 로 눌러 새 prim 리스트를 만든다."""
    ca, sa, ce, se = math.cos(az), math.sin(az), math.cos(el), math.sin(el)
    out = []
    for c, R, h, col, kind in prims:
        lv, faces = mesh(h, kind)
        pts = [[c[i] + sum(R[i][k] * p[k] for k in range(3))
                for i in range(3)] for p in lv]
        p2 = []
        for p in pts:
            xr = p[0] * ca - p[1] * sa
            yr = p[0] * sa + p[1] * ca
            p2.append((xr, -yr * se + p[2] * ce))
        depth = c[0] * sa + c[1] * ca
        out.append((depth, p2, col, faces))
    out.sort(key=lambda t: t[0])
    return out


def main():
    root = ET.fromstring(expand())
    prims = collect(root)

    fig = plt.figure(figsize=(15, 9), facecolor='white')
    fig.suptitle('valet_car  —  4.50 x 1.90 x 1.45 m,  wheelbase 2.50 m, '
                 'roof lidar @ 1.42 m', fontsize=13)

    views = [('Side  (X-Z)', (0, 2), (-2.6, 2.6), (-0.1, 1.7)),
             ('Front (Y-Z)', (1, 2), (-1.3, 1.3), (-0.1, 1.7)),
             ('Top   (X-Y)', (0, 1), (-2.6, 2.6), (-1.3, 1.3))]
    for i, (title, proj, xl, yl) in enumerate(views):
        ax = fig.add_subplot(2, 2, i + 1)
        draw(ax, prims, proj)
        ax.set_xlim(*xl); ax.set_ylim(*yl)
        ax.set_aspect('equal'); ax.set_title(title, fontsize=11)
        ax.grid(True, alpha=0.25, linewidth=0.5)
        if proj[1] == 2:
            ax.axhline(0, color='#666', linewidth=1.2)

    ax = fig.add_subplot(2, 2, 4)
    for _, p2, col, faces in iso_project(prims):
        for f in faces:
            ax.add_patch(Polygon([p2[i] for i in f], closed=True,
                                 facecolor=col[:3], edgecolor=(0, 0, 0, 0.3),
                                 linewidth=0.4,
                                 alpha=col[3] if len(col) > 3 else 1))
    ax.set_xlim(-3.4, 3.4); ax.set_ylim(-0.6, 2.4)
    ax.set_aspect('equal'); ax.set_title('Isometric', fontsize=11)
    ax.grid(True, alpha=0.25, linewidth=0.5)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=110)
    print('%s  (%d B)  조향 %.0f deg' % (OUT, os.path.getsize(OUT),
                                        math.degrees(STEER)))


if __name__ == '__main__':
    main()
