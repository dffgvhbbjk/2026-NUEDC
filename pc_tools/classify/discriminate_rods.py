# -*- coding: utf-8 -*-
# ============================================================================
# discriminate_rods.py -- 305mm 棒 vs 695mm 棒的单端区分特征验证
#
# 物理预测 (D_bot=105, 敲击端=传感器端):
#   305 棒: 直达回波 32 (盲区) / 二次往返 64 / 组合路径 32+105=137
#   695 棒: 直达回波 73        / 二次往返 146 / 组合路径 73+105=178
# 逐帧特征:
#   F1: 精细窗 56~68  出峰 (305棒二次往返 64)
#   F2: 精细窗 69~84  出峰 (695棒直达 73)
#   F3: 精细窗 128~146 出峰 (305棒组合 137 / 695棒二次往返 146)
#   F4: 精细窗 170~186 出峰 (695棒组合 178)
#   F5: 中回波精确位置 (58~88 窗内最大峰 delta)
# ============================================================================
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
from statistics import median

DATA = "data"
WIN = 5
SAT = 8388600
WEAK = 1000000
ONSET_MAX = 70

FEATS = [("F1_56-68", 56, 68), ("F2_69-84", 69, 84),
         ("F3_128-146", 128, 146), ("F4_170-186", 170, 186)]

TARGETS = {"02": "305棒轻", "03": "305棒中", "04": "695棒轻",
           "05": "695棒中", "06": "好棒轻", "07": "好棒中"}


def load_hit(path):
    s = []
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if row and not row[0].startswith("#") and row[0] != "index":
                s.append(int(row[1]))
    return s


def envelope(samples):
    base = median(samples[:24])
    mag = [abs(v - base) for v in samples]
    return [max(mag[max(0, i - WIN):i + WIN + 1]) for i in range(len(mag))]


def peak_in(env, on, lo, hi, floor):
    best = None
    for i in range(max(on + lo, 1), min(on + hi, len(env) - 1)):
        if env[i] >= floor and env[i] >= env[i - 1] and env[i] > env[i + 1]:
            if best is None or env[i] > best[1]:
                best = (i - on, env[i])
    return best


def main():
    hdr = "%-8s %-4s" % ("组", "帧数")
    for name, _, _ in FEATS:
        hdr += " %-12s" % name
    hdr += " 中回波位置(58~88)"
    print(hdr)
    print("-" * 100)
    for g in sorted(os.listdir(DATA)):
        gid = g[:2]
        if gid not in TARGETS:
            continue
        gdir = os.path.join(DATA, g)
        n = 0
        cnt = [0] * len(FEATS)
        mid_pos = []
        for h in sorted(os.listdir(gdir)):
            if not h.startswith("hit_"):
                continue
            s = load_hit(os.path.join(gdir, h))
            if len(s) < 128:
                continue
            env = envelope(s)
            pk = max(env)
            if pk < WEAK or max(abs(v) for v in s) >= SAT:
                continue
            on = next((i for i, v in enumerate(env) if v >= pk // 5), 999)
            if on > ONSET_MAX:
                continue
            n += 1
            floor = pk * 5 // 100
            for k, (_, lo, hi) in enumerate(FEATS):
                if peak_in(env, on, lo, hi, floor):
                    cnt[k] += 1
            pm = peak_in(env, on, 58, 88, floor)
            if pm:
                mid_pos.append(pm[0])
        row = "%-8s %-4d" % (TARGETS[gid], n)
        for k in range(len(FEATS)):
            row += " %2d/%-2d (%3d%%)" % (cnt[k], n, cnt[k] * 100 // max(n, 1))
        if mid_pos:
            row += "  中位 %d [%d..%d]" % (median(mid_pos), min(mid_pos),
                                            max(mid_pos))
        print(row)


if __name__ == "__main__":
    main()
