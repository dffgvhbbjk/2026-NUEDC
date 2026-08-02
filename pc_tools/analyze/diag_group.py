# -*- coding: utf-8 -*-
# 诊断单组数据在指定参数下的波包细节: 每帧打印候选列表 (delta/峰/宽)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.replay_analyzer import load_hit, preprocess, TRIG_IDX, \
    MIN_IMPACT_POINTS, MAX_IMPACT_POINTS, IMPACT_QUIET_POINTS, \
    SEARCH_ARM_QUIET_POINTS, PACKET_CONFIRM, IMPACT_SAT_LIMIT

DATA = "data"
# H 组参数
THR, MIN_REFL, IMP_MULT = 1200000, 400000, 26
D_REF, BW_LO, BW_HI = 105, 27, 37


def trace(smooth):
    """复现状态机, 返回 (imp_pk, imp_pi, armed_at, cands, rt_final)"""
    hist = [0] * 8
    env_sum = 0
    state = "IMPACT"
    idx = -1
    imp_pk = imp_pi = 0
    quiet = arm_quiet = 0
    armed = False
    armed_at = -1
    saturated = False
    in_pkt = False
    pw = pp = ppi = pe = 0
    cands = []

    def rthr():
        return max(MIN_REFL, (imp_pk * IMP_MULT) >> 7)

    for i, v in enumerate(smooth):
        mag = -v if v < 0 else v
        env_sum += mag - hist[7]
        hist = [mag] + hist[:7]
        env = env_sum >> 3
        if i < TRIG_IDX:
            continue
        if i == TRIG_IDX:
            imp_pk, imp_pi = env, 0
            saturated = mag >= IMPACT_SAT_LIMIT
            idx = 0
            continue
        idx += 1
        if state == "IMPACT":
            if mag >= IMPACT_SAT_LIMIT:
                saturated = True
            if env > imp_pk:
                imp_pk, imp_pi = env, idx
            if idx >= MIN_IMPACT_POINTS and env < (imp_pk >> 2):
                quiet += 1
                if quiet >= IMPACT_QUIET_POINTS:
                    state = "SEARCH"
            else:
                quiet = 0
            if idx >= MAX_IMPACT_POINTS:
                state = "SEARCH"
        elif state == "SEARCH":
            rt = rthr()
            if not armed:
                in_pkt = False
                if env <= rt:
                    arm_quiet += 1
                    if arm_quiet >= SEARCH_ARM_QUIET_POINTS:
                        armed = True
                        armed_at = idx
                else:
                    arm_quiet = 0
            elif not in_pkt:
                if env > rt:
                    in_pkt, pw, pp, ppi, pe = True, 1, env, idx, env
            else:
                if env > rt:
                    pw += 1
                    pe += env
                    if env > pp:
                        pp, ppi = env, idx
                else:
                    in_pkt = False
                    if pw >= PACKET_CONFIRM and len(cands) < 4:
                        cands.append((ppi, pp, pe, pw))
    if in_pkt and pw >= PACKET_CONFIRM and len(cands) < 4:
        cands.append((ppi, pp, pe, pw))
    return imp_pk, imp_pi, armed_at, cands, rthr(), saturated


def main(gid):
    win_lo = (D_REF * BW_LO) >> 5
    win_hi = (D_REF * BW_HI) >> 5
    print("棒底窗(触发基准): %d..%d" % (win_lo, win_hi))
    for g in sorted(os.listdir(DATA)):
        if not g.startswith(gid):
            continue
        gdir = os.path.join(DATA, g)
        for h in sorted(os.listdir(gdir)):
            if not h.startswith("hit_"):
                continue
            raw = load_hit(os.path.join(gdir, h))
            if len(raw) < 128:
                continue
            imp_pk, imp_pi, armed_at, cands, rt, sat = trace(preprocess(raw))
            cs = " ".join("[i%d p%.1fM w%d]" % (c[0], c[1] / 1e6, c[3])
                          for c in cands)
            print("%s imp_pk=%.1fM@%d rt=%.2fM arm@%d sat=%d | %s" % (
                h[:12], imp_pk / 1e6, imp_pi, rt / 1e6, armed_at,
                int(sat), cs if cs else "(无候选)"))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "05")
