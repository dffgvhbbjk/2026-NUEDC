# -*- coding: utf-8 -*-
# 波形级复核: 对关键组的原始采样点做包络+找峰, 验证真实棒底间隔与缺陷可见性
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
import sys
from statistics import median

DATA = "data"


def load_hit(path):
    samples = []
    meta = {}
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if not row:
                continue
            if row[0].startswith("#"):
                meta[row[0][2:]] = row[1] if len(row) > 1 else ""
            elif row[0] != "index":
                samples.append(int(row[1]))
    return meta, samples


def envelope(samples, win=6):
    """基线扣除(中位) + 绝对值 + 滑窗最大包络"""
    base = median(samples[:24])          # 预触发区做基线
    mag = [abs(s - base) for s in samples]
    env = []
    for i in range(len(mag)):
        lo = max(0, i - win)
        env.append(max(mag[lo:i + win + 1]))
    return env


def find_peaks(env, floor, min_gap=8):
    """粗找局部峰: 高于 floor, 间隔至少 min_gap"""
    peaks = []
    i = 1
    while i < len(env) - 1:
        if env[i] >= floor and env[i] >= env[i - 1] and env[i] >= env[i + 1]:
            if peaks and i - peaks[-1][0] < min_gap:
                if env[i] > peaks[-1][1]:
                    peaks[-1] = (i, env[i])
            else:
                peaks.append((i, env[i]))
            i += 1
        else:
            i += 1
    return peaks


def analyze_group(gdir, n_show=6):
    hits = sorted(x for x in os.listdir(gdir) if x.startswith("hit_"))
    print("=" * 70)
    print("组:", os.path.basename(gdir), " (%d hits)" % len(hits))
    deltas_2nd = []       # 主峰 -> 第二反射峰间隔
    sats = 0
    for h in hits:
        meta, samples = load_hit(os.path.join(gdir, h))
        if len(samples) < 64:
            continue
        if max(abs(s) for s in samples) >= 8388607:
            sats += 1
        env = envelope(samples)
        pk_main = max(env)
        # 主冲击位置 = 最大包络处
        imp = env.index(pk_main)
        # 在主峰尾振之后找反射 (阈值 = 主峰 8%)
        floor = pk_main * 8 // 100
        tail_start = imp + 15
        peaks = find_peaks(env[tail_start:], floor)
        peaks = [(p + tail_start, v) for p, v in peaks]
        if peaks:
            # 取幅值最大的反射峰
            best = max(peaks, key=lambda x: x[1])
            deltas_2nd.append(best[0] - imp)
    if deltas_2nd:
        print("主峰->最强反射峰间隔: 中位 %d, 范围 [%d, %d], n=%d" % (
            median(deltas_2nd), min(deltas_2nd), max(deltas_2nd),
            len(deltas_2nd)))
    else:
        print("尾部未发现显著反射 (>=8% 主峰)")
    if sats:
        print("!! 饱和帧数: %d / %d (|sample| 达 8388607 满幅)" % (sats, len(hits)))


def main():
    groups = sorted(d for d in os.listdir(DATA)
                    if os.path.isdir(os.path.join(DATA, d)))
    for g in groups:
        analyze_group(os.path.join(DATA, g))


if __name__ == "__main__":
    main()
