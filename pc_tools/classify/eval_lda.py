# -*- coding: utf-8 -*-
"""
eval_lda.py - 自相关向量 + 岭回归one-hot判别(线性打分, FPGA=点积+argmax)
特征: v = r[LAG0..LAG1]/r[0]  (归一化自相关, 128维)
打分: score_c = w_c . v + b_c  -> argmax
FPGA换算: score_c*r[0] = w_c . r + b_c*r[0]  (免除法)
验证: A 偶训奇测 B 塑料->铁 C 铁->塑料 + 5击多数投票
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA

SEG = 384
LAG0, LAG1 = 8, 136
RIDGE = 1e-2
CLS = {5:"good",6:"good",7:"good",8:"good",
       21:"d200",22:"d200",13:"d307",14:"d307",15:"d307",16:"d307",
       9:"d695",10:"d695",11:"d695",12:"d695",
       17:"d800",18:"d800",19:"d800",20:"d800"}
DIST = {"d200":200,"d307":307,"d695":695,"d800":800}
IRON = {7,8,11,12,15,16,19,20,22}
NAMES = ["good","d200","d307","d695","d800"]

def ac_feature(x):
    ax = np.abs(x)
    if ax.max() < 3e5: return None
    imp = int(np.argmax(ax))
    if imp > len(x)-256: return None
    seg = x[imp:imp+SEG].astype(np.float64)
    m = len(seg); seg = seg-seg.mean()
    r = np.correlate(seg,seg,"full")[m-1:]
    if r[0] <= 0 or len(r) < LAG1: return None
    return r[LAG0:LAG1]/r[0]

def load_all():
    data = []
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
            v = ac_feature(x)
            if v is None: continue
            data.append((gi, CLS[gi], gi in IRON, idx, v))
    return data

def train_lda(train):
    X = np.array([v for *_ ,v in train])
    Y = np.zeros((len(train), len(NAMES)))
    for i,(_,c,_,_,_) in enumerate(train):
        Y[i, NAMES.index(c)] = 1.0
    Xa = np.hstack([X, np.ones((len(X),1))])
    A = Xa.T @ Xa + RIDGE*len(X)*np.eye(Xa.shape[1])
    W = np.linalg.solve(A, Xa.T @ Y)     # (129,5)
    return W

def predict(W, v):
    s = np.append(v,1.0) @ W
    return NAMES[int(np.argmax(s))]

def evaluate(W, test, tag):
    conf = {}; n_ok=0; bin_conf=[[0,0],[0,0]]; derrs=[]
    for _,c,_,_,v in test:
        pred = predict(W, v)
        conf.setdefault(c,{}).setdefault(pred,0); conf[c][pred]+=1
        if pred==c: n_ok+=1
        ti=0 if c=="good" else 1; pi=0 if pred=="good" else 1
        bin_conf[ti][pi]+=1
        if ti==1 and pi==1: derrs.append(abs(DIST[pred]-DIST[c]))
    L=[f"\n===== {tag} (test={len(test)}) ====="]
    L.append(f"5类acc={n_ok/len(test):.3f}")
    tg=sum(bin_conf[0]); tb=sum(bin_conf[1])
    L.append(f"好棒判好={bin_conf[0][0]}/{tg}({bin_conf[0][0]/max(tg,1):.3f}) "
             f"坏棒判坏={bin_conf[1][1]}/{tb}({bin_conf[1][1]/max(tb,1):.3f})")
    if derrs:
        de=np.array(derrs)
        L.append(f"定位: 位置正确率={(de==0).mean():.3f} 中位误差={np.median(de):.0f}mm")
    L.append(f"{'真/判':<7}"+ "".join(f"{c:<7}" for c in NAMES))
    for c in NAMES:
        row=conf.get(c,{})
        L.append(f"{c:<7}"+ "".join(f"{row.get(p,0):<7}" for p in NAMES))
    return "\n".join(L)

def evaluate_vote(W, test, tag, k=5):
    """同组连续k击多数投票"""
    bygrp = {}
    for gi,c,_,idx,v in test:
        bygrp.setdefault(gi, []).append((idx,c,v))
    n_ok=0; n_tot=0; bin_ok_g=0; bin_tot_g=0; bin_ok_b=0; bin_tot_b=0; derrs=[]
    for gi,items in bygrp.items():
        items.sort()
        for i in range(0, len(items)-k+1, k):
            chunk = items[i:i+k]
            c = chunk[0][1]
            votes = {}
            for _,_,v in chunk:
                p = predict(W,v); votes[p]=votes.get(p,0)+1
            pred = max(votes, key=votes.get)
            n_tot+=1
            if pred==c: n_ok+=1
            if c=="good":
                bin_tot_g+=1; bin_ok_g += (pred=="good")
            else:
                bin_tot_b+=1; bin_ok_b += (pred!="good")
                if pred!="good": derrs.append(abs(DIST[pred]-DIST[c]))
    L=[f"--- {tag} {k}击投票: 5类acc={n_ok/max(n_tot,1):.3f} "
       f"好棒判好={bin_ok_g}/{bin_tot_g}({bin_ok_g/max(bin_tot_g,1):.3f}) "
       f"坏棒判坏={bin_ok_b}/{bin_tot_b}({bin_ok_b/max(bin_tot_b,1):.3f})"]
    if derrs:
        de=np.array(derrs)
        L.append(f"    定位位置正确率={(de==0).mean():.3f}")
    return "\n".join(L)

def main():
    data = load_all()
    print("total:", len(data))
    out=[]
    for tag,trsel,tesel in [
        ("A 偶训奇测", lambda d: d[3]%2==0, lambda d: d[3]%2==1),
        ("B 塑料头->铁头", lambda d: not d[2], lambda d: d[2]),
        ("C 铁头->塑料头", lambda d: d[2], lambda d: not d[2])]:
        tr=[d for d in data if trsel(d)]; te=[d for d in data if tesel(d)]
        W = train_lda(tr)
        out.append(evaluate(W,te,tag))
        out.append(evaluate_vote(W,te,tag,5))
    # 全量训练权重保存, 供RTL定点化
    W = train_lda(data)
    np.save(Path(__file__).parent.parent/"tmp"/"lda_weights.npy", W)
    txt="\n".join(out); print(txt)
    with open(Path(__file__).parent.parent/"tmp"/"eval_lda.txt","w",encoding="utf-8") as f:
        f.write(txt)

if __name__=="__main__":
    main()
