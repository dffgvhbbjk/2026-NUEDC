# -*- coding: utf-8 -*-
# ============================================================================
# verify_defect.py -- 用户真值确认后的判据验证 (缺陷 695mm, 03/04/05=缺陷棒)
#
# 逐帧执行方案 §2.4 的比例测距:
#   1. 起振点 onset 对齐
#   2. 棒底峰: 在 delta 90~120 找最大局部峰  (D_bot)
#   3. 缺陷峰: 在 delta 60~88  找最大局部峰  (D_def)
#   4. dist_mm = 1000 × D_def / D_bot, 对答案 695mm
# 同时输出各组"缺陷窗内出峰率"(不设 40% 门限), 检验好棒/坏棒可分性
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
DEF_LO, DEF_HI = 60, 88     # 缺陷搜索窗 (点)
BOT_LO, BOT_HI = 90, 120    # 棒底搜索窗 (点)
TRUTH = 695                 # 卷尺真值 mm

GROUPS = {                  # 用户确认的标签
    "02": ("好棒", "轻敲"), "03": ("缺陷棒", "中敲"),
    "04": ("缺陷棒", "轻敲"), "05": ("缺陷棒", "中敲"),
    "06": ("好棒", "轻敲"), "07": ("好棒", "中敲"),
}


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
    """窗内最大局部峰 (delta, amp) 或 None"""
    best = None
    for i in range(on + lo, min(on + hi, len(env) - 1)):
        if env[i] >= floor and env[i] >= env[i - 1] and env[i] > env[i + 1]:
            if best is None or env[i] > best[1]:
                best = (i - on, env[i])
    return best


def main():
    print("%-4s %-6s %-4s | %-8s %-10s %-14s %s" % (
        "组", "棒", "敲法", "缺陷出峰", "D_bot中位", "测距mm(中位)", "误差mm"))
    print("-" * 78)
    for g in sorted(os.listdir(DATA)):
        gid = g[:2]
        if gid not in GROUPS:
            continue
        rod, tap = GROUPS[gid]
        gdir = os.path.join(DATA, g)
        n_def = n_kept = 0
        bots, dists = [], []
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
            n_kept += 1
            floor = pk * 5 // 100
            pb = peak_in(env, on, BOT_LO, BOT_HI, floor)
            pd = peak_in(env, on, DEF_LO, DEF_HI, floor)
            if pd:
                n_def += 1
            if pb:
                bots.append(pb[0])
                if pd:
                    dists.append(1000 * pd[0] // pb[0])
        rate = "%d/%d" % (n_def, n_kept)
        bmed = "%d" % median(bots) if bots else "-"
        if dists:
            dmed = median(dists)
            derr = "%+d" % (dmed - TRUTH)
            dstr = "%d (n=%d)" % (dmed, len(dists))
        else:
            dstr, derr = "-", "-"
        print("%-4s %-6s %-4s | %-8s %-10s %-14s %s" % (
            gid, rod, tap, rate, bmed, dstr, derr))


if __name__ == "__main__":
    main()
