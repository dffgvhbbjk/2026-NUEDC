# -*- coding: utf-8 -*-
# ============================================================================
# analyze_capture.py -- 汇总分析 data\ 下所有采集组 (方案 §3.3 预处理前置统计)
#
# 逐组统计 summary.csv:
#   - 三态结果分布 / 置信度分布
#   - impact/defect/bottom 索引的中位数与离散度
#   - bottom_delta = bottom - impact (D_REF 候选) 的一致性
#   - 触发阈值 threshold / 峰值 peak_abs 的分布
#   - err_mm (有真值的组)
# 输出: 控制台 + tmp\data_analysis.txt
# 用法: python tools\analyze_capture.py [--datadir data]
# ============================================================================
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import argparse
import csv
import os
import sys
from statistics import median, mean, pstdev


def load_summary(path):
    with open(path, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    return rows


def ints(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        if v not in ("", None):
            try:
                out.append(int(v))
            except ValueError:
                pass
    return out


def fmt_stats(vals):
    if not vals:
        return "无数据"
    if len(vals) == 1:
        return "%d (单点)" % vals[0]
    return "中位 %d, 均值 %.0f, 范围 [%d, %d], σ=%.1f" % (
        median(vals), mean(vals), min(vals), max(vals), pstdev(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datadir", default="data")
    args = ap.parse_args()

    if not os.path.isdir(args.datadir):
        sys.exit("找不到目录: %s" % args.datadir)

    lines = []

    def emit(s=""):
        lines.append(s)
        print(s)

    groups = sorted(d for d in os.listdir(args.datadir)
                    if os.path.isdir(os.path.join(args.datadir, d)))

    all_normal_delta = []      # 所有 NORMAL 命中的 bottom-impact
    all_thr = []

    for g in groups:
        spath = os.path.join(args.datadir, g, "summary.csv")
        if not os.path.exists(spath):
            emit("[%s] 无 summary.csv, 跳过" % g)
            continue
        rows = load_summary(spath)
        emit("=" * 72)
        emit("组: %s   (%d 次敲击)" % (g, len(rows)))
        note = rows[0].get("note", "") if rows else ""
        if note:
            emit("说明: %s" % note)

        # 三态分布
        states = [r["state"] for r in rows]
        st_cnt = {s: states.count(s) for s in sorted(set(states))}
        emit("结果分布: %s" % ", ".join("%s×%d" % (k, v)
                                         for k, v in st_cnt.items()))
        confs = [r["conf"] for r in rows]
        cf_cnt = {c: confs.count(c) for c in sorted(set(confs))}
        emit("置信度:   %s" % ", ".join("%s×%d" % (k, v)
                                         for k, v in cf_cnt.items()))

        # 索引一致性
        imp = ints(rows, "impact")
        bot = ints(rows, "bottom")
        dfc = [v for v in ints(rows, "defect") if v > 0]
        emit("impact  : %s" % fmt_stats(imp))
        emit("bottom  : %s" % fmt_stats(bot))
        if dfc:
            emit("defect  : %s" % fmt_stats(dfc))

        # bottom_delta (D_REF 候选): 逐行 bottom-impact
        deltas = []
        for r in rows:
            try:
                b, i = int(r["bottom"]), int(r["impact"])
                if b > i:
                    delta = b - i
                    deltas.append(delta)
                    if r["state"] == "NORMAL":
                        all_normal_delta.append(delta)
            except (ValueError, KeyError):
                pass
        if deltas:
            emit("bot-imp : %s   <-- D_REF 候选" % fmt_stats(deltas))

        # 阈值与峰值
        thr = ints(rows, "threshold")
        pk = ints(rows, "peak_abs")
        emit("触发阈值: %s" % fmt_stats(thr))
        emit("峰值|max|: %s" % fmt_stats(pk))
        all_thr.extend(thr)
        # 峰值/阈值比 (触发裕量)
        ratios = [p / t for p, t in zip(pk, thr) if t > 0]
        if ratios:
            emit("峰/阈比 : 最小 %.1f×, 中位 %.1f×" %
                 (min(ratios), median(ratios)))

        # 距离与误差
        dist = [v for v in ints(rows, "dist_mm") if v > 0]
        errs = ints(rows, "err_mm")
        if dist:
            emit("dist_mm : %s" % fmt_stats(dist))
        if errs:
            emit("err_mm  : %s" % fmt_stats(errs))

    emit("=" * 72)
    emit("跨组汇总")
    emit("-" * 72)
    if all_normal_delta:
        emit("NORMAL 命中的 bottom-impact 全体: %s" % fmt_stats(all_normal_delta))
        emit("  当前 RTL 固化 D_REF = 105, 实测中位 = %d"
             % median(all_normal_delta))
    if all_thr:
        emit("全体触发阈值: %s" % fmt_stats(all_thr))
        emit("  当前 RTL: TH_MIN=500000, TH_MAX=1000000")

    os.makedirs("tmp", exist_ok=True)
    out = os.path.join("tmp", "data_analysis.txt")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n报告已存: %s" % out)


if __name__ == "__main__":
    main()
