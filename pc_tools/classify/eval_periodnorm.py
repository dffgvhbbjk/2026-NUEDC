# -*- coding: utf-8 -*-
"""
eval_periodnorm.py - 周期归一化自相关指纹分类验证
方法: 每次敲击 ->
  1) 冲击后384点去均值, 求归一化自相关 r[0..135]
  2) 在 lag 80..126 找基波周期 T (抛物线插值)
  3) 采样 shape[k] = r(k*T/16), k=2..15 (线性插值) -> 14维形状向量
  4) 特征向量 = [T/128, r(T), shape...] -> 最近质心(余弦) 5类分类
验证: A 偶训奇测  B 塑料->铁  C 铁->塑料
并给出好/坏二分类指标与各类->距离映射的定位误差
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA

SEG = 384
CLS = {5:"good",6:"good",7:"good",8:"good",
       21:"d200",22:"d200",13:"d307",14:"d307",15:"d307",16:"d307",
       9:"d695",10:"d695",11:"d695",12:"d695",
       17:"d800",18:"d800",19:"d800",20:"d800"}
DIST = {"d200":200,"d307":307,"d695":695,"d800":800}
IRON = {7,8,11,12,15,16,19,20,22}

def parab(y,k):
    d = y[k-1]-2*y[k]+y[k+1]
    if d == 0: return float(k)
    off = 0.5*(y[k-1]-y[k+1])/d
    return k + max(-1.0, min(1.0, off))

def features(x):
    ax = np.abs(x)
    if ax.max() < 3e5: return None
    imp = int(np.argmax(ax))
    if imp > len(x)-192: return None
    seg = x[imp:imp+SEG].astype(np.float64)
    m = len(seg); seg = seg-seg.mean()
    r = np.correlate(seg,seg,"full")[m-1:]
    if r[0] <= 0: return None
    r = r/r[0]
    if len(r) < 140: return None
    hi = min(127,len(r)-2)
    k = 80+int(np.argmax(r[80:hi]))
    T = parab(r,k); r1 = r[k]
    shape = []
    for i in range(2,16):
        t = T*i/16.0
        j = int(t); fr = t-j
        shape.append(r[j]*(1-fr)+r[j+1]*fr)
    return T, r1, np.array(shape)

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
            f = features(x)
            if f is None: continue
            T,r1,shape = f
            vec = np.concatenate([[ (T-96)/16.0, r1 ], shape])
            data.append((gi, CLS[gi], gi in IRON, idx, vec, T))
    return data

def evaluate(train, test, tag):
    names = sorted({c for _,c,_,_,_,_ in train})
    cents = {c: None for c in names}
    for c in names:
        vs = np.array([v for _,cc,_,_,v,_ in train if cc==c])
        cents[c] = vs.mean(axis=0)
    n_ok = 0; conf = {}; bin_conf = [[0,0],[0,0]]
    derrs = []
    for _,c,_,_,v,T in test:
        d2 = {cc: float(((v-m)**2).sum()) for cc,m in cents.items()}
        pred = min(d2, key=d2.get)
        conf.setdefault(c,{}).setdefault(pred,0); conf[c][pred]+=1
        if pred==c: n_ok+=1
        ti = 0 if c=="good" else 1; pi = 0 if pred=="good" else 1
        bin_conf[ti][pi]+=1
        if c!="good" and pred!="good":
            derrs.append(abs(DIST[pred]-DIST[c]))
    L = [f"\n===== {tag} (train={len(train)} test={len(test)}) ====="]
    L.append(f"5类acc={n_ok/len(test):.3f}")
    tg=sum(bin_conf[0]); tb=sum(bin_conf[1])
    L.append(f"好棒判好={bin_conf[0][0]}/{tg}({bin_conf[0][0]/max(tg,1):.3f}) "
             f"坏棒判坏={bin_conf[1][1]}/{tb}({bin_conf[1][1]/max(tb,1):.3f})")
    if derrs:
        de = np.array(derrs)
        L.append(f"定位误差(判坏且真坏): 中位={np.median(de):.0f}mm "
                 f"均值={de.mean():.0f}mm <=50mm比例={(de<=50).mean():.3f}")
    L.append(f"{'真/判':<7}"+ "".join(f"{c:<7}" for c in names))
    for c in names:
        row = conf.get(c,{})
        L.append(f"{c:<7}"+ "".join(f"{row.get(p,0):<7}" for p in names))
    return "\n".join(L)

def main():
    data = load_all()
    print("total:", len(data))
    out=[]
    tr=[d for d in data if d[3]%2==0]; te=[d for d in data if d[3]%2==1]
    out.append(evaluate(tr,te,"A 偶训奇测"))
    tr=[d for d in data if not d[2]]; te=[d for d in data if d[2]]
    out.append(evaluate(tr,te,"B 塑料头->铁头"))
    tr=[d for d in data if d[2]]; te=[d for d in data if not d[2]]
    out.append(evaluate(tr,te,"C 铁头->塑料头"))
    txt="\n".join(out); print(txt)
    with open(Path(__file__).parent.parent/"tmp"/"eval_periodnorm.txt","w",encoding="utf-8") as f:
        f.write(txt)

if __name__=="__main__":
    main()
