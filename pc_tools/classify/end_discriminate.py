# -*- coding: utf-8 -*-
# 两端区分特征提取: 同一根棒从两端敲, 找端不对称的观测量
#   F1 = 早期回波峰(55..90点内首包峰) / 主冲击包络峰   <- 近端缺陷应更大
#   F2 = 早期首包峰位置 (直达回波 vs 镜像路径)
#   F3 = 是否存在独立分辨的棒底包 (95..115 且与首包分离)
# 用 H 参数 (无分裂) 的候选结构, 峰值比不受分裂影响
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze.diag_group import trace
from core.replay_analyzer import load_hit, preprocess
from statistics import median

DATA = "data"
GROUPS = ["02", "03", "04", "05", "09", "11"]
LABEL = {"02": "棒A 远端敲(轻) 有效695", "03": "棒A 远端敲(中) 有效695",
         "04": "棒B 远端敲(轻) 有效695", "05": "棒B 远端敲(中) 有效695",
         "09": "棒B 远端敲(中) 有效695", "11": "棒A 近端敲(中) 真值305"}


def main():
    print("%-26s %-6s %-14s %-10s %-8s" % (
        "组", "帧数", "F1比值 中位[范围]", "F2首包位", "F3独立底包率"))
    for gid in GROUPS:
        gdirs = [g for g in sorted(os.listdir(DATA)) if g.startswith(gid)]
        if not gdirs:
            continue
        r1, f2, f3, n = [], [], 0, 0
        for g in gdirs:
            gdir = os.path.join(DATA, g)
            for h in sorted(os.listdir(gdir)):
                if not h.startswith("hit_"):
                    continue
                raw = load_hit(os.path.join(gdir, h))
                if len(raw) < 128:
                    continue
                imp_pk, imp_pi, armed_at, cands, rt, sat = trace(
                    preprocess(raw))
                if imp_pk < 2000000 or sat:
                    continue                    # 弱敲/饱和帧剔除
                early = [c for c in cands if 55 <= c[0] <= 90]
                if not early:
                    continue
                n += 1
                c0 = early[0]
                r1.append(100 * c0[1] // imp_pk)   # F1 %
                f2.append(c0[0])
                # F3: 95..115 有与首包分离的候选 (间隔>10)
                sep = [c for c in cands
                       if 95 <= c[0] <= 115 and c[0] - c0[0] > 10]
                if sep:
                    f3 += 1
        if n:
            print("%-26s %-6d %3d%% [%d..%d]%%   @%-8d %d/%d" % (
                LABEL[gid], n, median(r1), min(r1), max(r1),
                median(f2), f3, n))
        else:
            print("%-26s 无有效帧(早期无候选)" % LABEL[gid])


if __name__ == "__main__":
    main()
