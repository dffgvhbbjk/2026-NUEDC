# -*- coding: utf-8 -*-
"""
eval_lda_sweep.py - LDA方案参数扫描: 窗口起点/长度/预白化
划分:
  A 偶训奇测
  D 轻敲训练->重敲测试 (跨力度)
  E 重敲->轻敲
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA

CLS = {5:"good",6:"good",7:"good",8:"good",
       21:"d200",22:"d200",13:"d307",14:"d307",15:"d307",16:"d307",
       9:"d695",10:"d695",11:"d695",12:"d695",
       17:"d800",18:"d800",19:"d800",20:"d800"}
LIGHT = {5,7,9,11,13,15,17,19,21,22}   # 轻敲组(21,22无重敲对应,归轻)
NAMES = ["good","d200","d307","d695","d800"]
RIDGE = 1e-2

_raw_cache = []

def load_raw():
    global _raw_cache
    if _raw_cache: return _raw_cache
    for g in sorted(DATA.iterdir()):
        if not g.is_dir(): continue
        try: gi = int(g.name.split("_")[0])
        except ValueError: continue
        if gi not in CLS: continue
        files = sorted(g.glob("hit_*.csv"))
        if len(files) > 600: files = files[::len(files)//600]
        for idx,p in enumerate(files):
            try: x,_ = load_hit(p)
            except Exception: continue
            ax = np.abs(x)
            if ax.max() < 3e5: continue
            imp = int(np.argmax(ax))
            if imp > 96 or len(x) < 500: continue
            _raw_cache.append((gi, CLS[gi], gi in LIGHT, idx, x.astype(np.float64), imp))
    return _raw_cache

def ac_feature(x, imp, skip, seg_len, whiten, lag1):
    seg = x[imp+skip : imp+skip+seg_len]
    if len(seg) < seg_len: 
        seg = x[imp+skip:]
        if len(seg) < 256: return None
    seg = seg - seg.mean()
    if whiten:
        seg = np.diff(seg)
    m = len(seg)
    r = np.correlate(seg,seg,"full")[m-1:]
    if r[0] <= 0 or len(r) < lag1: return None
    return r[8:lag1]/r[0]

def train_lda(X, labs):
    Y = np.zeros((len(X), len(NAMES)))
    for i,c in enumerate(labs): Y[i, NAMES.index(c)] = 1.0
    Xa = np.hstack([X, np.ones((len(X),1))])
    A = Xa.T @ Xa + RIDGE*len(X)*np.eye(Xa.shape[1])
    return np.linalg.solve(A, Xa.T @ Y)

def run(skip, seg_len, whiten, lag1=136):
    raw = load_raw()
    feats = []
    for gi,c,light,idx,x,imp in raw:
        v = ac_feature(x, imp, skip, seg_len, whiten, lag1)
        if v is not None:
            feats.append((gi,c,light,idx,v))
    res = []
    for tag, trs, tes in [
        ("A", lambda d: d[3]%2==0, lambda d: d[3]%2==1),
        ("D", lambda d: d[2], lambda d: not d[2]),
        ("E", lambda d: not d[2], lambda d: d[2])]:
        tr=[d for d in feats if trs(d)]; te=[d for d in feats if tes(d)]
        if not tr or not te: continue
        W = train_lda(np.array([v for *_,v in tr]), [c for _,c,_,_,_ in tr])
        ok=0; g_ok=0; g_t=0; b_ok=0; b_t=0; loc_ok=0; loc_t=0
        for _,c,_,_,v in te:
            s = np.append(v,1.0) @ W
            pred = NAMES[int(np.argmax(s))]
            ok += (pred==c)
            if c=="good":
                g_t+=1; g_ok+=(pred=="good")
            else:
                b_t+=1; b_ok+=(pred!="good")
                if pred!="good":
                    loc_t+=1; loc_ok+=(pred==c)
        res.append(f"{tag}: acc5={ok/len(te):.3f} 好判好={g_ok/max(g_t,1):.3f} "
                   f"坏判坏={b_ok/max(b_t,1):.3f} 定位对={loc_ok/max(loc_t,1):.3f}")
    return res

def main():
    out=[]
    for skip, seg_len, whiten in [
            (0,384,False),(16,384,False),(32,384,False),
            (0,448,False),(32,416,False),
            (0,384,True),(16,384,True),(32,416,True),
            (48,384,False),(64,384,False)]:
        r = run(skip, seg_len, whiten)
        out.append(f"skip={skip:<3} len={seg_len:<4} whiten={int(whiten)}  | " + " | ".join(r))
        print(out[-1])
    with open(Path(__file__).parent.parent/"tmp"/"eval_lda_sweep.txt","w",encoding="utf-8") as f:
        f.write("\n".join(out))

if __name__=="__main__":
    main()
