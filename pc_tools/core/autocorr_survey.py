# -*- coding: utf-8 -*-
"""
autocorr_survey.py - 全量数据自相关/频域规律普查
目的: 验证 "好棒=只有棒底周期(~105pt), 坏棒=出现缺陷短周期" 这一物理假设,
      并统计各组主导周期(lag)分布, 评估好/坏分类与距离定位可行性。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os, sys, csv, math
import numpy as np
from pathlib import Path

FS = 95880.0
DATA = Path(r"d:\FPGA\dian_sai\Project\God3.11\God3.0\data")

def load_hit(p):
    """返回 (raw int array, header dict)"""
    hdr = {}
    vals = []
    with open(p, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line[1:].split(",", 1)
                if len(parts) == 2:
                    hdr[parts[0].strip()] = parts[1].strip()
                continue
            if line.startswith("index"):
                continue
            a = line.split(",")
            if len(a) >= 2:
                try:
                    vals.append(int(a[1]))
                except ValueError:
                    pass
    return np.array(vals, dtype=np.float64), hdr

def analyze_hit(x):
    """提取特征: 主导lag(自相关), 次lag, FFT主频bin等"""
    n = len(x)
    if n < 256:
        return None
    ax = np.abs(x)
    peak = ax.max()
    if peak < 3e5:
        return None  # 无有效敲击
    imp = int(np.argmax(ax))
    # 取冲击之后的振铃段做分析(跳过主冲击前沿, 保留衰减振荡)
    seg = x[imp:]
    if len(seg) < 200:
        seg = x  # 冲击太靠后, 退化用全帧
    seg = seg - seg.mean()
    # 归一化自相关
    m = len(seg)
    r = np.correlate(seg, seg, mode="full")[m-1:]
    if r[0] <= 0:
        return None
    r = r / r[0]
    # 在 lag 12..150 内找峰(缺陷200mm约21pt, 棒底约105pt)
    lo, hi = 12, min(150, m - 4)
    lag_best, val_best = -1, -2.0
    for k in range(lo, hi):
        if r[k] > r[k-1] and r[k] >= r[k+1]:
            if r[k] > val_best:
                val_best, lag_best = r[k], k
    # 首个显著峰(最早出现的、超过0.3的局部峰) -> 对应最短往返周期
    lag_first, val_first = -1, 0.0
    for k in range(lo, hi):
        if r[k] > r[k-1] and r[k] >= r[k+1] and r[k] > 0.25:
            lag_first, val_first = k, r[k]
            break
    # FFT 主频
    w = seg * np.hanning(m)
    sp = np.abs(np.fft.rfft(w, 1024))
    sp[:4] = 0  # 去直流/超低频
    fbin = int(np.argmax(sp))
    fdom = fbin * FS / 1024
    return dict(peak=peak, imp=imp,
                lag_best=lag_best, val_best=val_best,
                lag_first=lag_first, val_first=val_first,
                fdom=fdom)

def main():
    groups = sorted([d for d in DATA.iterdir() if d.is_dir()])
    out = []
    for g in groups:
        files = sorted(g.glob("hit_*.csv"))
        feats = []
        for p in files:
            try:
                x, hdr = load_hit(p)
            except Exception:
                continue
            f = analyze_hit(x)
            if f:
                feats.append(f)
        if not feats:
            out.append(f"{g.name[:46]:<48} n=0 (无有效敲击)")
            continue
        lb = np.array([f["lag_best"] for f in feats if f["lag_best"] > 0])
        lf = np.array([f["lag_first"] for f in feats if f["lag_first"] > 0])
        vb = np.array([f["val_best"] for f in feats])
        fd = np.array([f["fdom"] for f in feats])
        pk = np.array([f["peak"] for f in feats])
        def q(a):
            if len(a) == 0:
                return "-"
            return f"{np.percentile(a,25):.0f}/{np.median(a):.0f}/{np.percentile(a,75):.0f}"
        out.append(
            f"{g.name[:46]:<48} n={len(feats):<5}"
            f" lag_best[{q(lb)}] lag_first[{q(lf)}]"
            f" corr_med={np.median(vb):.2f}"
            f" fdom_med={np.median(fd):.0f}Hz peak_med={np.median(pk):.2e}")
        # 直方图: lag_first 分布(粗)
        if len(lf) > 0:
            hcounts, hedges = np.histogram(lf, bins=range(10, 160, 10))
            hs = " ".join(f"{int(hedges[i])}-{int(hedges[i+1])}:{hcounts[i]}"
                          for i in range(len(hcounts)) if hcounts[i] > 0)
            out.append(f"    lag_first直方: {hs}")
    txt = "\n".join(out)
    print(txt)
    with open(Path(__file__).parent.parent / "tmp" / "autocorr_survey.txt",
              "w", encoding="utf-8") as f:
        f.write(txt)

if __name__ == "__main__":
    main()
