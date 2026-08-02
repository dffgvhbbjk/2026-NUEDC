# -*- coding: utf-8 -*-
# 用户方案离线验证: 正常棒模板抵消 + 残差窗口能量 (E_near/E_far)
#   模板 = 07组(好棒中敲)包络按主冲击峰归一化后逐点平均
#   残差 = |帧包络 - 模板×(帧imp_pk/模板imp_pk)|
#   E_near = Σ残差[delta 24..40]  (305mm 直达回波区, 触发点基准)
#   E_far  = Σ残差[delta 60..85]  (695mm 直达回波区)
# 同时输出 08组(踩地面) 包络峰, 用于校准触发阈值回退值
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.replay_analyzer import load_hit, preprocess, TRIG_IDX
from statistics import median

DATA = "data"
# 窗口: 主冲击峰基准 delta (用户方案: 83点棒底 → 305mm≈25点, 695mm≈58点)
NEAR_LO, NEAR_HI = 18, 35
FAR_LO, FAR_HI = 48, 70
TPL_LEN = 160                  # 模板长度 (帧 index)
LABEL = {"02": "305轻", "03": "305中", "04": "695轻", "05": "695中",
         "06": "好棒轻", "07": "好棒中", "09": "695中", "11": "305中"}


def envelope(s):
    """8点滑动平均绝对值包络, 与 RTL 一致"""
    env, acc, hist = [], 0, [0] * 8
    for i, v in enumerate(s):
        m = -v if v < 0 else v
        acc += m - hist[7]
        hist = [m] + hist[:7]
        env.append(acc >> 3)
    return env


def frames(gid):
    out = []
    for g in sorted(os.listdir(DATA)):
        if not g.startswith(gid):
            continue
        gdir = os.path.join(DATA, g)
        for h in sorted(os.listdir(gdir)):
            if h.startswith("hit_"):
                raw = load_hit(os.path.join(gdir, h))
                if len(raw) >= TPL_LEN:
                    out.append(envelope(preprocess(raw)))
    return out


def imp_peak_at(env):
    """主冲击包络峰值与峰位 (触发点后 0..48)"""
    seg = env[TRIG_IDX:TRIG_IDX + 48]
    pk = max(seg)
    return pk, TRIG_IDX + seg.index(pk)


def main():
    # 踩地面(08) 包络/原始量级 -> 触发阈值回退参考
    g08 = frames("08")
    if g08:
        pks = [imp_peak_at(e)[0] for e in g08]
        print("08踩地面: %d帧 包络峰 中位%d 最大%d" % (
            len(g08), median(pks), max(pks)))

    # 模板: 07 组按主冲击峰"对齐+归一化"后逐点平均
    #   对齐 = 以冲击峰位为原点截取 [-8, TPL_LEN-8) 段
    tpl_frames = frames("07")
    aligned = []
    for e in tpl_frames:
        pk, pi = imp_peak_at(e)
        seg = e[pi - 8:pi - 8 + TPL_LEN]
        if len(seg) == TPL_LEN:
            aligned.append((seg, pk))
    ref_pk = median([p for _, p in aligned])
    tpl = []
    for i in range(TPL_LEN):
        vals = [seg[i] * ref_pk // pk for seg, pk in aligned]
        tpl.append(int(median(vals)))
    print("模板: 07组 %d帧 ref_pk=%d (主冲击峰对齐)" % (len(aligned), ref_pk))
    print("%-8s %-22s %-22s %s" % ("组", "E_near 中位[范围]", "E_far 中位[范围]",
                                   "near>far帧数"))
    for gid in ["02", "03", "11", "04", "05", "09", "06", "07"]:
        en_l, ef_l, nwin = [], [], 0
        for e in frames(gid):
            pk, pi = imp_peak_at(e)
            if pk < 1200000:
                continue                      # 弱敲剔除
            seg = e[pi - 8:pi - 8 + TPL_LEN]
            if len(seg) < TPL_LEN:
                continue
            res = [abs(seg[i] - tpl[i] * pk // ref_pk)
                   for i in range(TPL_LEN)]
            # seg 原点 = 冲击峰-8, 冲击峰在 seg[8], delta d -> seg[8+d]
            en = sum(res[8 + NEAR_LO:8 + NEAR_HI + 1])
            ef = sum(res[8 + FAR_LO:8 + FAR_HI + 1])
            # 按主冲击峰归一 (千分比), 消除敲击力度影响
            en_l.append(en * 1000 // (pk * (NEAR_HI - NEAR_LO + 1)))
            ef_l.append(ef * 1000 // (pk * (FAR_HI - FAR_LO + 1)))
            if en > ef:
                nwin += 1
        if en_l:
            print("%-8s %4d [%4d..%4d]      %4d [%4d..%4d]      %d/%d" % (
                LABEL[gid], median(en_l), min(en_l), max(en_l),
                median(ef_l), min(ef_l), max(ef_l), nwin, len(en_l)))


if __name__ == "__main__":
    main()
