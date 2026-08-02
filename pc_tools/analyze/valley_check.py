# -*- coding: utf-8 -*-
# 检查波包内部谷底: 在 60..130 区间打印包络的局部峰/谷序列
# 用于确定 RTL 波包分裂判据 (谷 < 峰×N/M)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.replay_analyzer import load_hit, preprocess, TRIG_IDX

DATA = "data"
LO, HI = 55, 135          # 触发基准的观察区间 (覆盖缺陷73/棒底105)


def envelope(smooth):
    hist = [0] * 8
    env_sum = 0
    out = []
    for v in smooth:
        mag = -v if v < 0 else v
        env_sum += mag - hist[7]
        hist = [mag] + hist[:7]
        out.append(env_sum >> 3)
    return out


def peaks_valleys(env):
    """区间内局部极值序列 (3点判据), 返回 [(idx, val, 'P'/'V')]"""
    seq = []
    for i in range(LO, HI):
        j = TRIG_IDX + i
        if j < 1 or j + 1 >= len(env):
            continue
        a, b, c = env[j - 1], env[j], env[j + 1]
        if b >= a and b > c:
            seq.append((i, b, "P"))
        elif b <= a and b < c:
            seq.append((i, b, "V"))
    # 合并相邻同类极值 (平顶)
    merged = []
    for it in seq:
        if merged and merged[-1][2] == it[2]:
            if (it[2] == "P") == (it[1] > merged[-1][1]):
                merged[-1] = it
        else:
            merged.append(it)
    return merged


def main(gids):
    for gid in gids:
        for g in sorted(os.listdir(DATA)):
            if not g.startswith(gid):
                continue
            print("==== 组 %s ====" % g)
            gdir = os.path.join(DATA, g)
            for h in sorted(os.listdir(gdir)):
                if not h.startswith("hit_"):
                    continue
                raw = load_hit(os.path.join(gdir, h))
                if len(raw) < 128:
                    continue
                env = envelope(preprocess(raw))
                mx = peaks_valleys(env)
                # 找 60..95 内最深"峰-谷-峰"结构: 缺陷峰 P1, 谷 V, 后续更高峰
                best = None
                for k in range(len(mx) - 2):
                    if mx[k][2] == "P" and mx[k + 1][2] == "V":
                        p1, v = mx[k], mx[k + 1]
                        after = [m for m in mx[k + 2:] if m[2] == "P"]
                        if not after or p1[1] <= 0:
                            continue
                        p2 = max(after, key=lambda m: m[1])
                        if not (60 <= p1[0] <= 95):
                            continue
                        depth = 100 * v[1] // p1[1]   # 谷/前峰 %
                        if best is None or depth < best[0]:
                            best = (depth, p1, v, p2)
                if best:
                    d, p1, v, p2 = best
                    print("%s P1@%d=%.2fM V@%d=%.2fM(%d%%) P2@%d=%.2fM" % (
                        h[:12], p1[0], p1[1] / 1e6, v[0], v[1] / 1e6, d,
                        p2[0], p2[1] / 1e6))
                else:
                    print("%s (60..95 无峰-谷结构)" % h[:12])


if __name__ == "__main__":
    main(sys.argv[1:] if len(sys.argv) > 1 else ["05", "07"])
