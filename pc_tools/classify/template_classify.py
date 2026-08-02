# -*- coding: utf-8 -*-
"""
template_classify.py - 归一化自相关向量 + 最近质心模板匹配 验证
类别: good(5-8), d200(21,22), d307(13-16), d695(9-12), d800(17-20)
验证方式:
  A) 组内偶数训练/奇数测试
  B) 跨锤头: 塑料头训练 -> 铁头测试 (检验现场锤头变化的鲁棒性)
输出: 好/坏二分类混淆 + 缺陷位置分类混淆
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA

L0, L1 = 16, 128          # 使用的lag区间
SEG = 384                 # 冲击后分析长度

CLS = {                   # 组 -> (类名, 距离mm)
    5: ("good", None), 6: ("good", None), 7: ("good", None), 8: ("good", None),
    21: ("d200", 200), 22: ("d200", 200),
    13: ("d307", 307), 14: ("d307", 307), 15: ("d307", 307), 16: ("d307", 307),
    9: ("d695", 695), 10: ("d695", 695), 11: ("d695", 695), 12: ("d695", 695),
    17: ("d800", 800), 18: ("d800", 800), 19: ("d800", 800), 20: ("d800", 800),
}
IRON = {7, 8, 11, 12, 15, 16, 19, 20, 22}   # 铁头组

def ac_vec(x):
    ax = np.abs(x)
    if ax.max() < 3e5:
        return None
    imp = int(np.argmax(ax))
    if imp > len(x) - (SEG // 2):
        return None
    seg = x[imp:imp + SEG].astype(np.float64)
    m = len(seg)
    seg = seg - seg.mean()
    r = np.correlate(seg, seg, "full")[m - 1:]
    if r[0] <= 0:
        return None
    r = r / r[0]
    v = r[L0:L1].copy()
    n = np.linalg.norm(v)
    if n <= 0:
        return None
    return v / n          # 单位化, 模板匹配即余弦相似度

def load_all():
    data = []             # (grp, cls, iron, vec)
    for g in sorted(DATA.iterdir()):
        if not g.is_dir():
            continue
        try:
            gi = int(g.name.split("_")[0])
        except ValueError:
            continue
        if gi not in CLS:
            continue
        files = sorted(g.glob("hit_*.csv"))
        if len(files) > 600:
            files = files[::len(files) // 600]
        for idx, p in enumerate(files):
            try:
                x, _ = load_hit(p)
            except Exception:
                continue
            v = ac_vec(x)
            if v is None:
                continue
            data.append((gi, CLS[gi][0], gi in IRON, idx, v))
    return data

def evaluate(train, test, tag):
    names = sorted({c for _, c, _, _, _ in train})
    cents = {}
    for c in names:
        vs = [v for _, cc, _, _, v in train if cc == c]
        m = np.mean(vs, axis=0)
        cents[c] = m / np.linalg.norm(m)
    conf = {}
    n_ok = 0
    bin_conf = [[0, 0], [0, 0]]   # [真好/真坏][判好/判坏]
    for _, c, _, _, v in test:
        sims = {cc: float(v @ m) for cc, m in cents.items()}
        pred = max(sims, key=sims.get)
        conf.setdefault(c, {}).setdefault(pred, 0)
        conf[c][pred] += 1
        if pred == c:
            n_ok += 1
        ti = 0 if c == "good" else 1
        pi = 0 if pred == "good" else 1
        bin_conf[ti][pi] += 1
    lines = [f"\n===== {tag} (train={len(train)}, test={len(test)}) ====="]
    lines.append(f"5类准确率: {n_ok/len(test):.3f}")
    tg = sum(bin_conf[0]); tb = sum(bin_conf[1])
    lines.append(f"好棒判好: {bin_conf[0][0]}/{tg} = {bin_conf[0][0]/max(tg,1):.3f}"
                 f"   坏棒判坏: {bin_conf[1][1]}/{tb} = {bin_conf[1][1]/max(tb,1):.3f}")
    lines.append(f"{'真\\判':<8}" + "".join(f"{c:<8}" for c in names))
    for c in names:
        row = conf.get(c, {})
        lines.append(f"{c:<8}" + "".join(f"{row.get(p,0):<8}" for p in names))
    return "\n".join(lines)

def main():
    data = load_all()
    print(f"total vecs: {len(data)}")
    out = []
    # A) 偶训奇测
    tr = [d for d in data if d[3] % 2 == 0]
    te = [d for d in data if d[3] % 2 == 1]
    out.append(evaluate(tr, te, "A 偶数训练/奇数测试"))
    # B) 塑料头训练 -> 铁头测试
    tr = [d for d in data if not d[2]]
    te = [d for d in data if d[2]]
    out.append(evaluate(tr, te, "B 塑料头训练->铁头测试"))
    # C) 铁头训练 -> 塑料头测试
    tr = [d for d in data if d[2]]
    te = [d for d in data if not d[2]]
    out.append(evaluate(tr, te, "C 铁头训练->塑料头测试"))
    txt = "\n".join(out)
    print(txt)
    with open(Path(__file__).parent.parent / "tmp" / "template_classify.txt",
              "w", encoding="utf-8") as f:
        f.write(txt)

if __name__ == "__main__":
    main()
