#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""README 용 배치도(docs/layout.png)를 그린다.

기하를 손으로 그리지 않는다. parking_spots.json 과 maps/*.pgm 만 읽으므로
generate_parking_lot.py 를 다시 돌리면 이 그림도 항상 따라온다.

    python3 tools/render_layout.py [패키지루트]
"""
import io
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mp
import matplotlib.pyplot as plt

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")


def read_pgm(path):
    with open(path, "rb") as f:
        d = f.read()
    i, toks = 0, []
    while len(toks) < 4:
        j = d.index(b"\n", i)
        ln = d[i:j]
        i = j + 1
        if ln.startswith(b"#"):
            continue
        toks += ln.split()
    w, h = int(toks[1]), int(toks[2])
    return w, h, d[i:i + w * h]


def main():
    with io.open(os.path.join(ROOT, "config", "parking_spots.json"),
                 encoding="utf-8") as f:
        D = json.load(f)
    b = D["bounds"]
    org = D["map"]["origin"]
    res = D["map"]["resolution"]

    fig, ax = plt.subplots(figsize=(13, 11.5))

    # 정적 맵을 옅은 배경으로 (벽·기둥이 어디 있는지 그대로 보인다)
    w, h, px = read_pgm(os.path.join(ROOT, "maps", "parking_lot.pgm"))
    import numpy as np
    g = np.frombuffer(px, dtype=np.uint8).reshape(h, w)
    ax.imshow(g, cmap="gray", vmin=0, vmax=255, alpha=0.35,
              extent=[org[0], org[0] + w * res, org[1], org[1] + h * res],
              origin="upper", zorder=0)

    col = {"standard": "#8fb8e0", "accessible": "#7fc98a",
           "ev": "#f2c14e", "hatched": "#d9d9d9"}
    for s in D["spots"]:
        x0, y0, x1, y1 = s["rect"]
        c = col.get(s["type"], col["standard"])
        ax.add_patch(mp.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                  facecolor="#c94f4f" if s["initially_occupied"] else c,
                                  edgecolor="#4a4a4a", linewidth=0.5, zorder=2))
        if s["index"] in (1, 14):
            ax.text((x0 + x1) / 2, (y0 + y1) / 2, s["id"], ha="center",
                    va="center", fontsize=6, zorder=3)
    for z in D.get("hatched_zones", []):
        x0, y0, x1, y1 = z["rect"] if isinstance(z, dict) else z
        ax.add_patch(mp.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                  hatch="///", edgecolor="#888", lw=0.6, zorder=3))

    # 통로 중심선
    A = D["aisles"]
    for k in ("south", "center", "north"):
        ax.axhline(A[k]["center_y"], color="#2a6fb0", ls="--", lw=0.9,
                   alpha=0.7, zorder=1)
        ax.text(b["x_min"] + 0.4, A[k]["center_y"] + 0.35,
                "%s  %.1f m" % (k, A[k]["width"]), fontsize=7,
                color="#2a6fb0", zorder=4)
    for k in ("west", "east"):
        ax.axvline(A[k]["center_x"], color="#2a6fb0", ls="--", lw=0.9,
                   alpha=0.7, zorder=1)
        ax.text(A[k]["center_x"] + 0.2, b["y_min"] + 0.6,
                "%s %.1f m" % (k, A[k]["width"]), fontsize=7, rotation=90,
                color="#2a6fb0", zorder=4)

    # ! 라벨은 영문으로 둔다. matplotlib 기본 폰트(DejaVu Sans)에 한글
    #   글리프가 없어서, 한글을 쓰면 팀원 환경에서 네모로 깨진다.
    for nm, key, c in (("ENTRY", "entry_pose", "#1a9641"),
                       ("EXIT", "exit_pose", "#d7191c")):
        x, y, _ = D[key]
        ax.plot(x, y, "o", ms=9, color=c, zorder=5)
        # 위쪽 라벨은 범례에 가리므로, 북쪽 포즈는 점 아래에 쓴다
        dy, va = (-1.9, "top") if y > 0 else (1.2, "bottom")
        ax.text(x, y + dy, "%s (%.1f, %.1f)" % (nm, x, y), ha="center",
                va=va, fontsize=8, color=c, weight="bold", zorder=5)

    r = D["robot_spec"]
    ax.add_patch(mp.Rectangle((D["entry_pose"][0] - r["length"] / 2,
                               D["entry_pose"][1] - r["width"] / 2),
                              r["length"], r["width"], fill=False,
                              edgecolor="#1a9641", lw=1.4, zorder=5))

    ax.set_xlim(org[0], org[0] + w * res)
    ax.set_ylim(org[1], org[1] + h * res)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]  (East +)")
    ax.set_ylabel("y [m]  (North +)")
    ax.set_title("%s  %.1f x %.1f m  |  %d spots (%d occupied)  |  R_min %.2f m"
                 % (D["lot_name"], b["x_max"] - b["x_min"],
                    b["y_max"] - b["y_min"], len(D["spots"]),
                    sum(1 for s in D["spots"] if s["initially_occupied"]),
                    r["min_turning_radius"]), fontsize=11)
    ax.grid(alpha=0.15, lw=0.4)
    leg = [mp.Patch(facecolor=col["standard"], edgecolor="#4a4a4a", label="standard"),
           mp.Patch(facecolor=col["accessible"], edgecolor="#4a4a4a", label="accessible"),
           mp.Patch(facecolor=col["ev"], edgecolor="#4a4a4a", label="EV charge"),
           mp.Patch(facecolor="#c94f4f", edgecolor="#4a4a4a", label="occupied at start"),
           mp.Patch(facecolor=col["hatched"], edgecolor="#4a4a4a", label="hatched (no parking)")]
    ax.legend(handles=leg, loc="upper right", fontsize=8, framealpha=0.9)

    out = os.path.join(ROOT, "docs", "layout.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("%s  (%.1f KB)" % (out, os.path.getsize(out) / 1024.0))


if __name__ == "__main__":
    main()
