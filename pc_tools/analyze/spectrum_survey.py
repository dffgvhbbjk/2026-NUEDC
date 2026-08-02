# -*- coding: utf-8 -*-
"""
spectrum_survey.py - 逐组平均功率谱 + 平均自相关曲线对比
物理假设(冲击回波): 缺陷距敲击端 d 会产生特征频率 f = v/2d 的反射序列,
对应自相关 lag = 2d/v*Fs。棒底 1000mm 对应 lag~105pt / f~913Hz。
预期: 305mm->lag 32 / 3000Hz, 695->73 / 1314Hz, 800->84 / 1141Hz, 200->21 / 4566Hz
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA, FS

NFFT = 2048

def group_curves(g, max_files=400):
    files = sorted(g.glob("hit_*.csv"))
    if len(files) > max_files:
        step = len(files) // max_files
        files = files[::step]
    spec_acc = np.zeros(NFFT // 2 + 1)
    ac_acc = np.zeros(200)
    n = 0
    for p in files:
        try:
            x, _ = load_hit(p)
        except Exception:
            continue
        if len(x) < 400:
            continue
        ax = np.abs(x)
        if ax.max() < 3e5:
            continue
        imp = int(np.argmax(ax))
        if imp > 200:
            continue
        seg = x[imp:imp + 400]
        if len(seg) < 300:
            continue
        seg = seg - seg.mean()
        seg = seg / (np.sqrt((seg ** 2).mean()) + 1e-9)  # 能量归一化
        m = len(seg)
        # 平均功率谱
        w = seg * np.hanning(m)
        sp = np.abs(np.fft.rfft(w, NFFT)) ** 2
        spec_acc += sp
        # 平均自相关
        r = np.correlate(seg, seg, mode="full")[m - 1:]
        r = r / (r[0] + 1e-9)
        L = min(200, len(r))
        ac_acc[:L] += r[:L]
        n += 1
    if n == 0:
        return None
    return spec_acc / n, ac_acc / n, n

def peak_list(spec, topn=6):
    """谱峰列表(freq, 相对dB)"""
    pk = []
    for k in range(6, len(spec) - 1):
        f = k * FS / NFFT
        if f > 8000:
            break
        if spec[k] > spec[k - 1] and spec[k] >= spec[k + 1]:
            pk.append((f, spec[k]))
    pk.sort(key=lambda t: -t[1])
    ref = pk[0][1] if pk else 1.0
    return [(f"{f:.0f}Hz", f"{10*np.log10(v/ref):.1f}dB") for f, v in pk[:topn]]

def ac_peaks(ac, topn=5):
    pk = []
    for k in range(12, len(ac) - 1):
        if ac[k] > ac[k - 1] and ac[k] >= ac[k + 1] and ac[k] > 0.05:
            pk.append((k, ac[k]))
    pk.sort(key=lambda t: -t[1])
    return [(k, f"{v:.2f}") for k, v in sorted(pk[:topn])]

def main():
    groups = sorted([d for d in DATA.iterdir() if d.is_dir()])
    lines = []
    curves = {}
    for g in groups:
        r = group_curves(g)
        if r is None:
            lines.append(f"== {g.name}: 无有效数据")
            continue
        spec, ac, n = r
        curves[g.name] = (spec, ac)
        lines.append(f"== {g.name} (n={n})")
        lines.append(f"   谱峰: {peak_list(spec)}")
        lines.append(f"   自相关峰(lag,val): {ac_peaks(ac)}")
    txt = "\n".join(lines)
    print(txt)
    outdir = Path(__file__).parent.parent / "tmp"
    with open(outdir / "spectrum_survey.txt", "w", encoding="utf-8") as f:
        f.write(txt)
    # 保存曲线供后续绘图/细看
    np.savez(outdir / "group_curves.npz",
             **{k: np.concatenate([v[0], v[1]]) for k, v in curves.items()})

if __name__ == "__main__":
    main()
