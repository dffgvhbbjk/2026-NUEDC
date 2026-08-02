# -*- coding: utf-8 -*-
# ============================================================================
# analyze_v2.py -- 逐帧精细分析 (尼龙棒版)
#
# 相对第一版的修正:
#   1. 主冲击基准 = 起振点 onset (首次越过 20% 峰值), 而非包络最大值
#      (振铃增长型波形里 env.index(max) 会落在窗口尾部, 全错)
#   2. 逐帧质量筛选, 坏帧剔除并记录原因:
#      - 弱帧 (峰值 < 100 万, 噪声误触发)
#      - 饱和帧 (|s| >= 8388600, 削顶失真)
#      - 起振过晚 (onset > 70, 尾部长度不够看棒底回波)
#   3. 回波统计改为全组直方图聚类: 每帧找出全部 >=5% 局部峰,
#      把 delta 投到 4 点宽的桶里, 看跨帧一致的聚集位置
#      (单帧最强峰易被振铃干扰, 跨帧聚类才可靠)
#
# 物理预期 (尼龙, c≈1200~1800 m/s, Fs=95.88kHz):
#   1m 棒底回波 delta = 2L/(c·Ts) ≈ 106(c=1800) ~ 160(c=1200) 点
#   70mm 缺陷回波 delta ≈ 棒底delta × 0.07 ≈ 7~11 点 (可能被主冲击掩盖)
# ============================================================================
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
from collections import Counter
from statistics import median

DATA = "data"
WIN = 5
SAT = 8388600
WEAK = 1000000
ONSET_MAX = 70        # 起振点晚于此丢弃 (尾部不足以看 ~154 点回波)
BIN = 4               # 直方图桶宽 (点)


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


def onset_of(env, pk):
    thr = pk // 5                      # 20% 峰值
    for i, v in enumerate(env):
        if v >= thr:
            return i
    return len(env)


def local_peaks(env, start, floor, gap=10):
    peaks = []
    for i in range(start, len(env) - 1):
        if env[i] >= floor and env[i] >= env[i - 1] and env[i] > env[i + 1]:
            if peaks and i - peaks[-1][0] < gap:
                if env[i] > peaks[-1][1]:
                    peaks[-1] = (i, env[i])
            else:
                peaks.append((i, env[i]))
    return peaks


def analyze_group(gdir):
    hits = sorted(x for x in os.listdir(gdir) if x.startswith("hit_"))
    kept, drop = [], Counter()
    hist = Counter()                   # delta 桶 -> 出现帧数
    amps = {}                          # delta 桶 -> 幅值千分比列表
    for h in hits:
        s = load_hit(os.path.join(gdir, h))
        if len(s) < 128:
            drop["短帧"] += 1
            continue
        env = envelope(s)
        pk = max(env)
        if pk < WEAK:
            drop["弱帧(噪声触发)"] += 1
            continue
        if max(abs(v) for v in s) >= SAT:
            drop["饱和"] += 1
            continue
        on = onset_of(env, pk)
        if on > ONSET_MAX:
            drop["起振过晚"] += 1
            continue
        kept.append(h)
        # 尾部找全部 >=5% 局部峰, 投直方图 (每帧每桶只投一次)
        floor = pk * 5 // 100
        seen = set()
        for p, v in local_peaks(env, on + 20, floor):
            b = (p - on) // BIN * BIN
            if b in seen:
                continue
            seen.add(b)
            hist[b] += 1
            amps.setdefault(b, []).append(v * 1000 // pk)
    return hits, kept, drop, hist, amps


def main():
    print("质量规则: 剔除 弱帧<%d / 饱和>=%d / 起振>%d" % (WEAK, SAT, ONSET_MAX))
    for g in sorted(os.listdir(DATA)):
        gdir = os.path.join(DATA, g)
        if not os.path.isdir(gdir):
            continue
        hits, kept, drop, hist, amps = analyze_group(gdir)
        print("=" * 72)
        print("组: %s   保留 %d/%d   剔除: %s" % (
            g, len(kept), len(hits),
            ", ".join("%s×%d" % kv for kv in drop.items()) or "无"))
        if not kept:
            continue
        # 打印跨帧一致性 >= 40% 的回波聚集桶
        n = len(kept)
        rows = [(b, c, median(amps[b])) for b, c in hist.items()
                if c >= max(3, n * 2 // 5)]
        rows.sort(key=lambda x: -x[1])
        if not rows:
            print("  无跨帧一致回波聚集 (>=40%帧)")
            continue
        print("  回波聚集 (delta桶, 出现率, 幅值中位千分比):")
        for b, c, a in rows[:6]:
            print("    delta=%3d~%-3d  %2d/%d 帧 (%3d%%)  amp=%d/1000" % (
                b, b + BIN - 1, c, n, c * 100 // n, a))


if __name__ == "__main__":
    main()
