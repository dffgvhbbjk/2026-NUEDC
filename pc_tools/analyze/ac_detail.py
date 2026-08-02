# -*- coding: utf-8 -*-
"""
ac_detail.py - 查看各组平均自相关在关键lag处的取值 + 次级峰结构
理论: lag_bottom~105(1000mm往返), 缺陷d -> lag_d=105*d/L 及互补 105*(L-d)/L
      d200->21/84, d307->32/73, d695->73/32, d800->84/21
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path

TMP = Path(__file__).parent.parent / "tmp"
z = np.load(TMP / "group_curves.npz")
NSPEC = 2048 // 2 + 1

lines = []
for name in sorted(z.files):
    v = z[name]
    ac = v[NSPEC:]          # 200点平均自相关
    # 关键lag取值
    kl = {k: ac[k] for k in (21, 26, 32, 42, 52, 63, 73, 84, 95)}
    s = " ".join(f"r[{k}]={val:+.2f}" for k, val in kl.items())
    # 主峰
    k0 = 80 + int(np.argmax(ac[80:126]))
    lines.append(f"{name[:44]:<46} T={k0}({ac[k0]:.2f})")
    lines.append(f"    {s}")
    # 12..95内全部局部峰
    pks = [(k, ac[k]) for k in range(12, 96)
           if ac[k] > ac[k-1] and ac[k] >= ac[k+1] and ac[k] > 0.02]
    lines.append("    局部正峰: " + " ".join(f"{k}:{v2:.2f}" for k, v2 in pks))
    # 负谷(反相反射的特征)
    vls = [(k, ac[k]) for k in range(12, 96)
           if ac[k] < ac[k-1] and ac[k] <= ac[k+1] and ac[k] < -0.10]
    lines.append("    负谷:     " + " ".join(f"{k}:{v2:.2f}" for k, v2 in vls))
txt = "\n".join(lines)
print(txt)
with open(TMP / "ac_detail.txt", "w", encoding="utf-8") as f:
    f.write(txt)
