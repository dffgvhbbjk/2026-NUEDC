#!/usr/bin/env python3
"""
analyze_v3.py — 基桩动测仪 完整数据分析与验证
=================================================
使用 fpga_model.py 的 bit-accurate 算法 + 独立的详细统计。

功能:
  1. 逐组分析自相关特征
  2. 好桩 vs 坏桩 判别
  3. 缺陷位置估计
  4. 详细混淆矩阵
  5. 输出 FPGA 可用的判别规律总结
"""

import sys
import numpy as np
from pathlib import Path
from collections import defaultdict, Counter

# 添加 tools 目录到 path (pc_tools/ -> Project root -> God3.0/tools)
sys.path.insert(0, str(Path(__file__).parent.parent / 'God3.0' / 'tools'))

from core.fpga_model import (fx_feature, fx_predict, train_weights, load_all,
                         CLS, NAMES, DIST, LIGHT, MIN_PEAK, IMP_WIN,
                         SKIP, SEGLEN, NLAG, LAG0, MARGIN_Q, QW)
from core.autocorr_survey import load_hit, DATA

# ============================================================================
# 1. 加载数据
# ============================================================================
print("=" * 80)
print("1. 加载数据...")
print("=" * 80)

data = load_all(maxn=600)
print(f"总有效样本: {len(data)} (已排除 INVALID 帧)")

# 统计每组的有效样本数
grp_counts = Counter(d[0] for d in data)
for g in sorted(grp_counts):
    cls_name = NAMES[CLS[g]]
    is_light = "轻敲" if g in LIGHT else "重敲"
    is_iron = "铁头" if g in {7,8,11,12,15,16,19,20,22} else "塑料头"
    print(f"  组{g:2d}: {grp_counts[g]:5d} 样本 [{cls_name}] {is_light} {is_iron}")

# 检查有多少文件被排除
print(f"\n各目录文件数 vs 有效帧数对比:")
for g_dir in sorted(DATA.iterdir()):
    if not g_dir.is_dir():
        continue
    try:
        gid = int(g_dir.name.split("_")[0])
    except ValueError:
        continue
    if gid not in CLS:
        continue
    total_files = len(list(g_dir.glob("hit_*.csv")))
    valid = grp_counts.get(gid, 0)
    invalid = total_files - valid
    print(f"  组{gid:2d}: {total_files:5d} 文件 → {valid:5d} 有效 ({invalid:5d} INVALID = {invalid/max(total_files,1)*100:.1f}%)")

# ============================================================================
# 2. 自相关特征分析 (逐类)
# ============================================================================
print("\n" + "=" * 80)
print("2. 自相关特征逐类分析 (lag 8..135)")
print("=" * 80)

# 提取每种缺陷类别的自相关特征
cls_features = defaultdict(list)
for gi, cls_id, is_light, idx, r in data:
    cls_features[cls_id].append((gi, r))

for cls_id in range(5):
    items = cls_features.get(cls_id, [])
    if not items:
        continue

    n = len(items)
    # 对每个 lag 统计均值、标准差
    r_matrix = np.array([r for _, r in items])
    r_norm = r_matrix / r_matrix[:, :1]  # 归一化: r[k]/r[0]

    print(f"\n--- {NAMES[cls_id].upper()} (n={n}) ---")
    print(f"  r[0] 范围: {r_matrix[:,0].min():.2e} ~ {r_matrix[:,0].max():.2e}")
    print(f"  r[0] 中位数: {np.median(r_matrix[:,0]):.2e}")

    # 找平均自相关曲线的关键特征
    mean_r_norm = r_norm.mean(axis=0)
    std_r_norm = r_norm.std(axis=0)

    # 基波周期 (lag 90-110 范围内的最大峰)
    T0_range = mean_r_norm[90:111]
    T0 = int(np.argmax(T0_range)) + 90
    print(f"  基波周期 T0 (自相关峰): {T0} (值={mean_r_norm[T0]:.3f})")
    print(f"  T0 分布: 25%={np.percentile([np.argmax(r[90:111])+90 for r in r_norm], 25):.0f} "
          f"50%={np.percentile([np.argmax(r[90:111])+90 for r in r_norm], 50):.0f} "
          f"75%={np.percentile([np.argmax(r[90:111])+90 for r in r_norm], 75):.0f}")

    # 关键 lag 的相关值
    key_lags = [20, 30, 40, 50, 60, 70, 77, 80, 90, 96, 100, 105, 110, 120, 130]
    print(f"  关键 lag 归一化自相关值 (均值±标准差):")
    for lag in key_lags:
        if lag < len(mean_r_norm):
            print(f"    lag{lag:3d}: {mean_r_norm[lag]:+.3f} ± {std_r_norm[lag]:.3f}")

    # 与 good 类的差异性 (判别力最大的 lag)
    if cls_id != 0:
        good_items = cls_features.get(0, [])
        if good_items:
            good_r_norm = np.array([r for _, r in good_items]) / np.array([r for _, r in good_items])[:, :1]
            good_mean = good_r_norm.mean(axis=0)
            diff = np.abs(mean_r_norm - good_mean)
            # 找前 10 个判别力最大的 lag
            top_lags = np.argsort(diff[LAG0:])[::-1][:10] + LAG0
            print(f"  与 good 差异最大的 10 个 lag (判别力):")
            for lag in top_lags:
                if lag < len(diff):
                    print(f"    lag{lag:3d}: Δ={diff[lag]:.3f} (好={good_mean[lag]:+.3f}, 坏={mean_r_norm[lag]:+.3f})")


# ============================================================================
# 3. 训练+测试 (详细分层统计)
# ============================================================================
print("\n" + "=" * 80)
print("3. LDA 分类器训练与测试 (bit-accurate)")
print("=" * 80)

# 偶训奇测
tr = [d for d in data if d[3] % 2 == 0]
te = [d for d in data if d[3] % 2 == 1]
Wf, Wq = train_weights(tr)

print(f"训练集: {len(tr)}, 测试集: {len(te)}")

# 详细预测
results = []
for gi, c, is_light, idx, r in te:
    pred, margin, rs0 = fx_predict(Wq, r)
    low_conf = (margin < MARGIN_Q * rs0)

    # 800mm 二次判别
    d800_override = False
    if pred == 0:
        T0 = int(np.argmax(r[90:111])) + 90
        if T0 >= 103:
            pred = 4
            d800_override = True
            low_conf = True

    correct_5cls = (pred == c)
    correct_bin = (pred == 0) == (c == 0)  # 好/坏二值

    results.append({
        'gi': gi, 'true_cls': c, 'pred_cls': pred,
        'correct_5cls': correct_5cls, 'correct_bin': correct_bin,
        'low_conf': low_conf, 'd800_override': d800_override,
        'margin': margin, 'is_light': is_light,
    })

# 总体统计
n_tot = len(results)
n_ok_5cls = sum(1 for r in results if r['correct_5cls'])
n_ok_bin = sum(1 for r in results if r['correct_bin'])
n_good = sum(1 for r in results if r['true_cls'] == 0)
n_defect = sum(1 for r in results if r['true_cls'] != 0)
n_good_ok = sum(1 for r in results if r['true_cls'] == 0 and r['pred_cls'] == 0)
n_defect_ok = sum(1 for r in results if r['true_cls'] != 0 and r['pred_cls'] != 0)
n_loc_ok = sum(1 for r in results if r['true_cls'] != 0 and r['pred_cls'] == r['true_cls'])

print(f"\n总体 5 类准确率: {n_ok_5cls}/{n_tot} = {n_ok_5cls/max(n_tot,1):.3f}")
print(f"好棒判好: {n_good_ok}/{n_good} = {n_good_ok/max(n_good,1):.3f}")
print(f"坏棒判坏: {n_defect_ok}/{n_defect} = {n_defect_ok/max(n_defect,1):.3f}")
print(f"坏棒定位正确: {n_loc_ok}/{n_defect} = {n_loc_ok/max(n_defect,1):.3f}")

# 混淆矩阵
print(f"\n5 类混淆矩阵 (真\\判):")
print(f"{'':>8}", end="")
for name in NAMES:
    print(f"{name:>8}", end="")
print()
conf = np.zeros((5, 5), dtype=int)
for r in results:
    conf[r['true_cls'], r['pred_cls']] += 1
for i, name in enumerate(NAMES):
    print(f"{name:>8}", end="")
    for j in range(5):
        print(f"{conf[i,j]:>8}", end="")
    print()

# 每类precision/recall
print(f"\n{'类':<8}{'Precision':<12}{'Recall':<12}{'F1':<10}")
for i, name in enumerate(NAMES):
    tp = conf[i, i]
    pred_pos = conf[:, i].sum()
    actual_pos = conf[i, :].sum()
    prec = tp / max(pred_pos, 1)
    rec = tp / max(actual_pos, 1)
    f1 = 2 * prec * rec / max(prec + rec, 1e-10)
    print(f"{name:<8}{prec:<12.3f}{rec:<12.3f}{f1:<10.3f}")

# 按力度分组
print(f"\n按敲击力度/锤头分组:")
for is_light in [True, False]:
    subset = [r for r in results if r['is_light'] == is_light]
    if not subset:
        continue
    n_s = len(subset)
    ok_5 = sum(1 for r in subset if r['correct_5cls'])
    ok_bin = sum(1 for r in subset if r['correct_bin'])
    print(f"  {'轻敲' if is_light else '重敲'}: n={n_s}, 5类acc={ok_5/max(n_s,1):.3f}, "
          f"二值acc={ok_bin/max(n_s,1):.3f}")

# 低置信度统计
low_conf_results = [r for r in results if r['low_conf']]
if low_conf_results:
    n_lc = len(low_conf_results)
    ok_lc = sum(1 for r in low_conf_results if r['correct_5cls'])
    print(f"\n低置信度样本: {n_lc}/{n_tot} = {n_lc/max(n_tot,1)*100:.1f}%")
    print(f"  低置信度下准确率: {ok_lc}/{n_lc} = {ok_lc/max(n_lc,1):.3f}")

# 800mm 二次判别
d800_overrides = [r for r in results if r['d800_override']]
if d800_overrides:
    n_d8 = len(d800_overrides)
    ok_d8 = sum(1 for r in d800_overrides if r['correct_5cls'])
    print(f"\n800mm 二次判别: {n_d8} 个 good 被判为 d800")
    print(f"  其中真正 d800: {sum(1 for r in d800_overrides if r['true_cls']==4)}")
    print(f"  二次判别准确率: {ok_d8}/{n_d8} = {ok_d8/max(n_d8,1):.3f}")


# ============================================================================
# 4. 好/坏判别规律总结
# ============================================================================
print("\n" + "=" * 80)
print("4. 判别规律总结 (可用于简单分类器)")
print("=" * 80)

# 单特征好/坏可分性
print("\n单特征阈值扫描 (好 vs 坏二分类):")
for cls_id in range(5):
    items = cls_features[cls_id]
    if not items:
        continue
    r_mat = np.array([r for _, r in items])
    r0_vals = r_mat[:, 0]

    # 简单特征
    r_norm_96 = r_mat[:, 96] / r0_vals  # lag 96 归一化自相关 (好棒基波处)
    r_norm_70 = r_mat[:, 70] / r0_vals  # lag 70 归一化自相关

    T0_vals = np.array([np.argmax(r[90:111]) + 90 for r in r_mat])

    print(f"  {NAMES[cls_id]:>8}: T0=[{T0_vals.min():.0f},{T0_vals.max():.0f}] "
          f"median={np.median(T0_vals):.0f}, "
          f"r96/r0=[{r_norm_96.min():.3f},{r_norm_96.max():.3f}] "
          f"r70/r0=[{r_norm_70.min():.3f},{r_norm_70.max():.3f}]")

# 最优 T0 阈值
print("\nT0 单特征好/坏可分性:")
good_T0 = np.concatenate([
    np.array([np.argmax(r[90:111])+90 for _, r in cls_features.get(0, [])])
])
all_bad_T0 = np.concatenate([
    np.array([np.argmax(r[90:111])+90 for _, r in cls_features.get(c, [])])
    for c in [1, 2, 3, 4] if cls_features.get(c)
])

print(f"  好棒 T0: [{good_T0.min():.0f}, {good_T0.max():.0f}] "
      f"median={np.median(good_T0):.0f}")
print(f"  坏棒 T0: [{all_bad_T0.min():.0f}, {all_bad_T0.max():.0f}] "
      f"median={np.median(all_bad_T0):.0f}")

# 在各个阈值下的分类性能
for thr in range(95, 115):
    good_ok = (good_T0 < thr).mean()
    bad_ok = (all_bad_T0 >= thr).mean()
    bal_acc = (good_ok + bad_ok) / 2
    if thr % 5 == 0:
        print(f"  T0 < {thr}: 好棒召回={good_ok:.3f}, "
              f"坏棒召回={bad_ok:.3f}, 平衡准确率={bal_acc:.3f}")

# 最佳多特征组合
print("\n最佳判别策略:")

# 策略1: LDA (最优, 但需要乘法器)
print("  策略1 (推荐, FPGA已实现): LDA 128 维自相关 → 5 类线性判别")
print(f"    5类准确率: {n_ok_5cls/max(n_tot,1)*100:.1f}%")
print(f"    好棒判别: {n_good_ok/max(n_good,1)*100:.1f}%")
print(f"    坏棒检出: {n_defect_ok/max(n_defect,1)*100:.1f}%")
print(f"    缺陷定位: {n_loc_ok/max(n_defect,1)*100:.1f}%")

# 策略2: T0 + 简单自相关特征 (不需要完整LDA)
# 找到最佳单特征组合
best_combo = None
best_combo_acc = 0
for lag1 in range(20, 130, 5):
    for lag2 in range(lag1+10, 135, 5):
        # 计算各样本在这两个 lag 的归一化自相关
        X_good = np.array([
            [r[lag1]/r[0], r[lag2]/r[0]]
            for _, r in cls_features.get(0, [])
        ])
        X_bad = np.array([
            [r[lag1]/r[0], r[lag2]/r[0]]
            for c in [1,2,3,4] for _, r in cls_features.get(c, [])
        ])

        if len(X_good) < 10 or len(X_bad) < 10:
            continue

        # 简单线性分类: 使用两个特征的平均值中点
        mean_good = X_good.mean(axis=0)
        mean_bad = X_bad.mean(axis=0)

        # 用欧氏距离最近均值分类
        pred_good = np.array([
            np.linalg.norm(x - mean_good) < np.linalg.norm(x - mean_bad)
            for x in X_good
        ]).mean()
        pred_bad = np.array([
            np.linalg.norm(x - mean_bad) < np.linalg.norm(x - mean_good)
            for x in X_bad
        ]).mean()
        bal = (pred_good + pred_bad) / 2

        if bal > best_combo_acc:
            best_combo_acc = bal
            best_combo = (lag1, lag2, pred_good, pred_bad)

if best_combo:
    print(f"\n  策略2 (简化): 自相关 lag{best_combo[0]} + lag{best_combo[1]} 二特征分类")
    print(f"    好棒判好: {best_combo[2]*100:.1f}%")
    print(f"    坏棒判坏: {best_combo[3]*100:.1f}%")
    print(f"    平衡准确率: {best_combo_acc*100:.1f}%")

# 策略3: 仅 T0
print(f"\n  策略3 (最简单): 仅用基波周期 T0 区分好/坏/800mm")
print(f"    T0 < 103 → GOOD")
print(f"    T0 >= 103 → 800mm DEFECT")
print(f"    好棒判好: {(good_T0 < 103).mean()*100:.1f}%")
# 对于坏棒, 只考虑 d800
bad_d800_T0 = np.concatenate([
    np.array([np.argmax(r[90:111])+90 for _, r in cls_features.get(4, [])])
])
print(f"    800mm缺陷检出: {(bad_d800_T0 >= 103).mean()*100:.1f}%")

print("\n" + "=" * 80)
print("分析完成!")
print("=" * 80)
