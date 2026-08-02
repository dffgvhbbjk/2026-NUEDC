# -*- coding: utf-8 -*-
"""
separability.py - 基于 feature_table.csv 的组间可分性统计
组标签: 好棒=5,6,7,8 (以及3=正常棒旧批次)
        坏棒: 305/307mm -> 1,13,14,15,16 ; 695 -> 4,9,10,11,12
              800 -> 17,18,19,20 ; 200 -> 21,22
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path

TMP = Path(__file__).parent.parent / "tmp"
rows = []
with open(TMP / "feature_table.csv", encoding="utf-8") as f:
    hdr = f.readline().strip().split(",")
    for line in f:
        a = line.strip().split(",")
        if len(a) != len(hdr):
            continue
        d = dict(zip(hdr, a))
        rows.append(d)

GOOD = {3, 5, 6, 7, 8}
BAD = {1: 305, 4: 695, 9: 695, 10: 695, 11: 695, 12: 695,
       13: 307, 14: 307, 15: 307, 16: 307,
       17: 800, 18: 800, 19: 800, 20: 800, 21: 200, 22: 200}

def col(rs, k):
    return np.array([float(r[k]) for r in rs])

groups = sorted({int(r["grp"]) for r in rows})
print(f"{'grp':<4}{'n':<6}{'f1_lag(25/50/75)':<22}{'f1Hz':<8}{'r1':<6}"
      f"{'ac_half':<9}{'a2dB':<7}{'a3dB':<7}{'ceps':<6}{'e_early':<8}")
for g in groups:
    rs = [r for r in rows if int(r["grp"]) == g]
    if not rs:
        continue
    lag = col(rs, "f1_lag"); f1 = col(rs, "f1"); r1 = col(rs, "r1")
    ah = col(rs, "ac_half"); a2 = col(rs, "a2"); a3 = col(rs, "a3")
    cl = col(rs, "ceps_lag"); ee = col(rs, "e_early")
    tag = "好" if g in GOOD else ("干扰" if g == 2 else f"坏{BAD.get(g,'?')}")
    print(f"{g:<4}{len(rs):<6}"
          f"{np.percentile(lag,25):5.1f}/{np.median(lag):5.1f}/{np.percentile(lag,75):5.1f}   "
          f"{np.median(f1):<8.0f}{np.median(r1):<6.2f}"
          f"{np.median(ah):<9.2f}{np.median(a2):<7.1f}{np.median(a3):<7.1f}"
          f"{np.median(cl):<6.0f}{np.median(ee):<8.3f} {tag}")

# ---- 好/坏 二分类: 单特征阈值扫描 ----
print("\n==== 单特征好/坏可分性 (排除组2干扰) ====")
lab, feats_all = [], {}
keys = ["f1_lag", "f1", "r1", "ac_half", "a2", "a3", "e_early"]
sel = [r for r in rows if int(r["grp"]) != 2]
y = np.array([0 if int(r["grp"]) in GOOD else 1 for r in sel])
for k in keys:
    v = col(sel, k)
    # 扫描最佳阈值(两个方向)
    best = (0, 0, 0)
    for thr in np.percentile(v, np.arange(1, 100)):
        for sign in (1, -1):
            pred = (sign * v > sign * thr).astype(int)
            acc = (pred == y).mean()
            if acc > best[0]:
                best = (acc, thr, sign)
    acc, thr, sign = best
    op = ">" if sign == 1 else "<"
    # 分别统计好/坏召回
    pred = (sign * v > sign * thr).astype(int)
    rec_bad = (pred[y == 1] == 1).mean()
    rec_good = (pred[y == 0] == 0).mean()
    print(f"{k:<9} 判坏条件: {op}{thr:8.2f}  acc={acc:.3f} "
          f"好棒特异={rec_good:.3f} 坏棒检出={rec_bad:.3f}")

# f1_lag 联合 f1 的散点粗表
print("\n==== f1(Hz) 分桶 x 好/坏 ====")
v = col(sel, "f1")
for lo in range(840, 1060, 20):
    m = (v >= lo) & (v < lo + 20)
    if m.sum() == 0:
        continue
    ng = ((y == 0) & m).sum(); nb = ((y == 1) & m).sum()
    print(f"{lo}-{lo+20}Hz: 好{ng:5d} 坏{nb:5d}")
