# -*- coding: utf-8 -*-
"""
feature_table.py - 逐敲击特征提取, 输出CSV供可分性分析
特征:
  pk        : 峰值绝对值
  f1_lag    : 自相关基波周期(lag 80..125, 抛物线插值)
  r1        : 基波自相关幅值
  ac_half   : lag=f1_lag/2 处自相关值(模态2对齐度)
  f1,f2,f3  : FFT谱峰频率(Hz, 插值), a2,a3为相对模1幅度(dB)
  ceps_lag  : 倒谱在[15,95]内最大峰位置(回波周期)
  ceps_val  : 相应峰值
  e_early   : 冲击后 8..40 点包络能量 / 全段能量
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA, FS

NFFT = 4096
MAXN = 600   # 每组最多分析条数

def parab(y, k):
    d = (y[k-1] - 2*y[k] + y[k+1])
    if d == 0:
        return float(k)
    return k + 0.5*(y[k-1] - y[k+1]) / d

def feats(x):
    ax = np.abs(x)
    pk = ax.max()
    if pk < 3e5:
        return None
    imp = int(np.argmax(ax))
    if imp > len(x) - 320:
        return None
    seg = x[imp:imp+448].astype(np.float64)
    m = len(seg)
    seg = seg - seg.mean()
    en = (seg**2).sum()
    if en <= 0:
        return None
    # ---- 自相关 ----
    r = np.correlate(seg, seg, "full")[m-1:]
    r = r / r[0]
    lo, hi = 80, min(126, m-2)
    k = lo + int(np.argmax(r[lo:hi]))
    f1_lag = parab(r, k) if 0 < k < len(r)-1 else float(k)
    r1 = r[k]
    kh = int(round(f1_lag/2))
    ac_half = r[kh] if kh < len(r) else 0.0
    # ---- FFT 谱峰 ----
    w = seg * np.hanning(m)
    sp = np.abs(np.fft.rfft(w, NFFT))**2
    sp[:int(400/FS*NFFT)] = 0     # <400Hz 去掉低频干扰
    df = FS/NFFT
    def pkin(flo, fhi):
        a, b = int(flo/df), int(fhi/df)
        kk = a + int(np.argmax(sp[a:b]))
        if kk <= 0 or kk >= len(sp)-1:
            return 0.0, 0.0
        return parab(sp, kk)*df, sp[kk]
    f1, A1 = pkin(600, 1300)
    f2, A2 = pkin(1300, 2300)
    f3, A3 = pkin(2300, 3200)
    a2 = 10*np.log10(A2/A1) if A1 > 0 and A2 > 0 else -60.0
    a3 = 10*np.log10(A3/A1) if A1 > 0 and A3 > 0 else -60.0
    # ---- 倒谱(回波检测) ----
    lsp = np.log(np.abs(np.fft.rfft(seg*np.hanning(m), 1024))**2 + 1e-6)
    cep = np.fft.irfft(lsp)
    q0, q1 = 15, 95
    kc = q0 + int(np.argmax(cep[q0:q1]))
    # ---- 早期能量占比 ----
    env = np.convolve(np.abs(seg), np.ones(4)/4, "same")
    e_early = (env[8:40]**2).sum() / ((env**2).sum() + 1e-9)
    return dict(pk=pk, imp=imp, f1_lag=f1_lag, r1=r1, ac_half=ac_half,
                f1=f1, f2=f2, f3=f3, a2=a2, a3=a3,
                ceps_lag=kc, ceps_val=cep[kc], e_early=e_early)

def main():
    groups = sorted([d for d in DATA.iterdir() if d.is_dir()])
    rows = []
    for gi, g in enumerate(groups):
        files = sorted(g.glob("hit_*.csv"))
        if len(files) > MAXN:
            files = files[::max(1, len(files)//MAXN)]
        cnt = 0
        for p in files:
            try:
                x, _ = load_hit(p)
            except Exception:
                continue
            f = feats(x)
            if f is None:
                continue
            f["grp"] = gi + 1
            f["file"] = p.name
            rows.append(f)
            cnt += 1
        print(f"group {gi+1} {g.name[:30]} -> {cnt}")
    keys = ["grp","pk","imp","f1_lag","r1","ac_half","f1","f2","f3",
            "a2","a3","ceps_lag","ceps_val","e_early","file"]
    out = Path(__file__).parent.parent / "tmp" / "feature_table.csv"
    with open(out, "w", encoding="utf-8") as fo:
        fo.write(",".join(keys) + "\n")
        for r0 in rows:
            fo.write(",".join(str(r0[k]) for k in keys) + "\n")
    print("saved", out, len(rows))

if __name__ == "__main__":
    main()
