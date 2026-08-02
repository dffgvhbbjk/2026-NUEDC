# -*- coding: utf-8 -*-
# 模板统计: 各组波形按主冲击对齐 -> 幅值归一化 -> 逐点中位包络 -> 找反射峰
# (方案 §3.3 的 1~6 步, 用于确定真实棒底间隔 D_REF 与反射幅值比例)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
from statistics import median

DATA = "data"
WIN = 5           # 包络滑窗半宽
PRE = 24          # 预触发基线区


def load_hit(path):
    samples = []
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if row and not row[0].startswith("#") and row[0] != "index":
                samples.append(int(row[1]))
    return samples


def envelope(samples):
    base = median(samples[:PRE])
    mag = [abs(s - base) for s in samples]
    return [max(mag[max(0, i - WIN):i + WIN + 1]) for i in range(len(mag))]


def group_template(gdir):
    hits = sorted(x for x in os.listdir(gdir) if x.startswith("hit_"))
    aligned = []
    for h in hits:
        s = load_hit(os.path.join(gdir, h))
        if len(s) < 128:
            continue
        env = envelope(s)
        pk = max(env)
        if pk < 1000000:          # 丢弃无真实冲击的弱帧 (噪声误触发)
            continue
        imp = env.index(pk)
        # 对齐: 以主冲击为 0 点, 归一化到 1000
        norm = [v * 1000 // pk for v in env[imp:]]
        aligned.append(norm)
    if len(aligned) < 3:
        return None, len(hits), len(aligned)
    # 逐点中位 (截到最短)
    L = min(len(a) for a in aligned)
    tmpl = [median(a[i] for a in aligned) for i in range(L)]
    return tmpl, len(hits), len(aligned)


def find_peaks(tmpl, start=15, floor=30, gap=10):
    """模板上找局部峰: 千分幅 >= floor"""
    peaks = []
    for i in range(start, len(tmpl) - 1):
        if tmpl[i] >= floor and tmpl[i] >= tmpl[i - 1] and tmpl[i] > tmpl[i + 1]:
            if peaks and i - peaks[-1][0] < gap:
                if tmpl[i] > peaks[-1][1]:
                    peaks[-1] = (i, tmpl[i])
            else:
                peaks.append((i, tmpl[i]))
    return peaks


def main():
    for g in sorted(os.listdir(DATA)):
        gdir = os.path.join(DATA, g)
        if not os.path.isdir(gdir):
            continue
        tmpl, n_all, n_used = group_template(gdir)
        print("=" * 70)
        print("组: %s   (有效强冲击 %d/%d)" % (g, n_used, n_all))
        if tmpl is None:
            print("  强冲击帧不足, 跳过 (弱帧=噪声误触发)")
            continue
        peaks = find_peaks(tmpl)
        if not peaks:
            print("  主冲击后无 >=3%% 反射峰 (模板长 %d)" % len(tmpl))
            continue
        print("  主冲击后反射峰 (间隔点数, 幅值千分比):")
        for d, v in peaks[:8]:
            print("    delta=%3d   amp=%3d/1000  (%.1f%%)" % (d, v, v / 10))


if __name__ == "__main__":
    main()
