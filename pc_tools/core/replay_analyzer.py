# -*- coding: utf-8 -*-
# ============================================================================
# replay_analyzer.py -- defect_analyzer.v 的按位精确 Python 复现 + 真实数据回放
#
# 管线复现 (与 RTL 一致):
#   raw --(基线扣除: 预触发前24点均值)--> corrected
#       --(三点平滑 y=(x[n-1]+2x[n]+x[n+1]+2)>>2)--> smooth
#       --(8点绝对值滑动平均包络)--> envelope
#       --> defect_analyzer 状态机 (ST_IMPACT/ST_SEARCH/分类)
#
# 触发点 = 帧内 index 32 (预触发 32 点)。
# 用途: 在改 RTL 前, 用 170 帧实测波形验证参数组合的三态判定表现。
# ============================================================================
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
from statistics import median

DATA = "data"
TRIG_IDX = 32          # 预触发点数

# ---- 参数组定义: (名称, trigger_thr, MIN_REFLECT_ENV, noise_mult_x2,
#                    impact_gate_shift, d_ref, imp_mult128, ref_trig, bw_lo, bw_hi) ----
# noise_mult_x2: 反射阈值中 trigger_thr 系数×2 (3=1.5倍, 0=去掉该项)
# impact_gate_shift: meas_ok 比较 impact_peak >= thr >> shift
# imp_mult128: 反射阈值 = imp_pk × N/128 (当前RTL N=13 ≈10%)
# ref_trig: 1=delta 以触发点为基准 (当前RTL 0=以主冲击包络峰为基准)
# bw_lo/bw_hi: 棒底窗口 ×N/32 (当前RTL 21/43 = 0.65..1.35)
# split: 1=波包内谷底分裂 (包络 < 包内峰×15/16 时收尾当前包)
PARAM_SETS = [
    ("A_当前RTL",   20000,   30000,  3, 0, 83, 13, 0, 21, 43, 0),
    ("G_触发基准窄窗25%", 1200000, 400000, 0, 1, 105, 32, 1, 27, 37, 0),
    ("H_触发基准窄窗20%", 1200000, 400000, 0, 1, 105, 26, 1, 27, 37, 0),
    ("J_H+谷底分裂", 1200000, 400000, 0, 1, 105, 26, 1, 27, 37, 1),
    ("K_G+谷底分裂", 1200000, 400000, 0, 1, 105, 32, 1, 27, 37, 1),
]

# 每组: (类型, 手法, 缺陷真值mm distance-from-敲击端; 好棒=None)
# 2026-07-29 用户更正: 02/03 = 305mm (原"颠倒敲=695"记录有误), 04/05 = 695mm
GROUPS = {"02": ("缺陷", "轻", 305), "03": ("缺陷", "中", 305),
          "04": ("缺陷", "轻", 695), "05": ("缺陷", "中", 695),
          "06": ("好棒", "轻", None), "07": ("好棒", "中", None),
          "09": ("缺陷", "中", 695), "11": ("缺陷", "中", 305)}

# RTL 固定参数
MIN_IMPACT_POINTS = 12
MAX_IMPACT_POINTS = 48
IMPACT_QUIET_POINTS = 6
SEARCH_ARM_QUIET_POINTS = 6
PACKET_CONFIRM = 3
BOTTOM_MARGIN = 8
IMPACT_SAT_LIMIT = 8300000


def load_hit(path):
    s = []
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if row and not row[0].startswith("#") and row[0] != "index":
                s.append(int(row[1]))
    return s


def preprocess(raw):
    """基线扣除 + 三点平滑 (与 wave_trigger/wave_smooth 一致)"""
    base = sum(raw[:24]) // 24
    x = [v - base for v in raw]
    y = [0] * len(x)
    for i in range(1, len(x) - 1):
        y[i] = (x[i - 1] + 2 * x[i] + x[i + 1] + 2) >> 2
    y[0], y[-1] = x[0], x[-1]
    return y


def analyze(smooth, thr, min_refl, nm_x2, gate_shift, d_ref, imp_mult=13,
            ref_trig=0, bw_lo=21, bw_hi=43, split=0):
    """defect_analyzer 状态机复现, 返回 (state, conf, imp, dfc, bot, d_bot)"""
    # 棒底窗口 [bw_lo/32, bw_hi/32] × d_ref
    win_lo = (d_ref * bw_lo) >> 5
    win_hi = (d_ref * bw_hi) >> 5

    hist = [0] * 8                     # 8 点包络历史
    env_sum = 0
    state = "IMPACT"
    idx = -1                           # sample_index (触发点=0)
    imp_pk, imp_pi = 0, 0
    quiet = arm_quiet = 0
    armed = False
    saturated = False
    in_pkt = False
    pw = pp = ppi = pe = 0
    cands = []                         # (index, peak, energy, width)

    def reflect_thr():
        noise_ref = (thr * nm_x2) >> 1
        impact_ref = (imp_pk * imp_mult) >> 7
        return max(min_refl, noise_ref, impact_ref)

    for i, v in enumerate(smooth):
        mag = -v if v < 0 else v
        env_sum += mag - hist[7]
        hist = [mag] + hist[:7]
        env = env_sum >> 3
        if i < TRIG_IDX:
            continue                   # 触发前仅暖包络
        if i == TRIG_IDX:
            imp_pk, imp_pi = env, 0
            saturated = mag >= IMPACT_SAT_LIMIT
            idx = 0
            continue
        idx += 1
        if state == "IMPACT":
            if mag >= IMPACT_SAT_LIMIT:
                saturated = True
            if env > imp_pk:
                imp_pk, imp_pi = env, idx
            if idx >= MIN_IMPACT_POINTS and env < (imp_pk >> 2):
                quiet += 1
                if quiet >= IMPACT_QUIET_POINTS:
                    state = "SEARCH"
            else:
                quiet = 0
            if idx >= MAX_IMPACT_POINTS:
                state = "SEARCH"
        elif state == "SEARCH":
            rt = reflect_thr()
            if not armed:
                in_pkt = False
                if env <= rt:
                    arm_quiet += 1
                    if arm_quiet >= SEARCH_ARM_QUIET_POINTS:
                        armed = True
                else:
                    arm_quiet = 0
            elif not in_pkt:
                if env > rt:
                    in_pkt, pw, pp, ppi, pe = True, 1, env, idx, env
            else:
                if env > rt:
                    pw += 1
                    pe += env
                    if env > pp:
                        pp, ppi = env, idx
                    elif split and env < pp - (pp >> 4):
                        # 谷底分裂: 回落超过峰值 1/16, 收尾当前包重开新包
                        if pw >= PACKET_CONFIRM and len(cands) < 4:
                            cands.append((ppi, pp, pe, pw))
                        pw, pp, ppi, pe = 1, env, idx, env
                else:
                    in_pkt = False
                    if pw >= PACKET_CONFIRM and len(cands) < 4:
                        cands.append((ppi, pp, pe, pw))
    # capture_done 冲刷
    if in_pkt and pw >= PACKET_CONFIRM and len(cands) < 4:
        cands.append((ppi, pp, pe, pw))

    # ---- 分类 ----
    rt = reflect_thr()
    base = 0 if ref_trig else imp_pi   # delta 基准: 触发点 or 主冲击包络峰
    hits = [(c[0] - base, c) for c in cands
            if win_lo <= (c[0] - base) <= win_hi]
    bot = max(hits, key=lambda x: x[1][2]) if hits else None
    bot_delta = bot[0] if bot else 0

    dfc = None
    for c in cands:
        d = c[0] - base
        if bot:
            if d + BOTTOM_MARGIN <= bot_delta:
                dfc = (d, c)
                break
        else:
            if d < win_lo and c[1] >= 2 * rt:
                dfc = (d, c)
                break

    meas_ok = (imp_pk >= (thr >> gate_shift)) and not saturated and cands
    if not meas_ok:
        return ("INVALID", 0, imp_pi, 0, 0, 0)
    if dfc and bot:
        return ("DEFECT", 2, imp_pi, dfc[0], bot_delta, bot_delta)
    if dfc:
        return ("DEFECT", 1, imp_pi, dfc[0], 0, 0)
    if bot:
        return ("NORMAL", 2, imp_pi, 0, bot_delta, bot_delta)
    return ("INVALID", 0, imp_pi, 0, 0, 0)


def main():
    for pname, thr, mre, nm, gs, dref, im, rtb, blo, bhi, sp in PARAM_SETS:
        print("=" * 76)
        print("参数组 %s: thr=%d min_refl=%d noise×%.1f gate>>%d d_ref=%d "
              "反射%d/128 基准=%s 窗%d..%d/32 分裂=%d" % (
            pname, thr, mre, nm / 2.0, gs, dref, im,
            "触发" if rtb else "冲击峰", blo, bhi, sp))
        print("%-10s %-18s %-22s %s" % ("组", "判定分布", "缺陷delta(中位)", "测距mm(中位/误差)"))
        for g in sorted(os.listdir(DATA)):
            gid = g[:2]
            if gid not in GROUPS:
                continue
            rod, tap, truth = GROUPS[gid]
            res, deltas, dists = [], [], []
            gdir = os.path.join(DATA, g)
            for h in sorted(os.listdir(gdir)):
                if not h.startswith("hit_"):
                    continue
                raw = load_hit(os.path.join(gdir, h))
                if len(raw) < 128:
                    continue
                st, cf, imp, dd, bd, botd = analyze(
                    preprocess(raw), thr, mre, nm, gs, dref, im,
                    rtb, blo, bhi, sp)
                res.append(st)
                if st == "DEFECT" and dd:
                    deltas.append(dd)
                    dists.append(dd * 1000 // (botd if botd else dref))
            cnt = {s: res.count(s) for s in sorted(set(res))}
            dist_s = "-"
            if dists:
                dm = median(dists)
                tag = " (err%+d)" % (dm - truth) if truth else ""
                dist_s = "%d%s n=%d" % (dm, tag, len(dists))
            print("%-10s %-18s %-22s %s" % (
                "%s %s%s" % (gid, rod, tap),
                ",".join("%s×%d" % kv for kv in cnt.items()),
                "%d" % median(deltas) if deltas else "-",
                dist_s))


if __name__ == "__main__":
    main()
