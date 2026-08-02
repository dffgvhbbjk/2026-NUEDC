#!/usr/bin/env python3
"""
批量扫描分析窗口参数，找到最优 WIN_START / WIN_END / EDGE_THRESHOLD 组合。

优化: 每个文件只做一次边沿检测，然后对每种窗口组合 O(1) 查表计数。

用法:
  python pc_tools/scan_window.py          # 粗扫(步长10)
  python pc_tools/scan_window.py --step 5 # 细扫
  python pc_tools/scan_window.py --step 3 # 超细扫
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "pc_tools"))

import csv, json, time
import numpy as np

DATA_DIR = Path(r"D:\FPGA\dian_sai\Project\God3.11\God3.0\data")

def is_good_dir(dirname: str) -> bool:
    d = dirname.lower()
    if "好棒" in d or "好柱" in d or "good" in d or "正常" in d:
        return True
    if "坏棒" in d or "坏柱" in d or "缺陷" in d or "defect" in d or "bad" in d:
        return False
    return None

def load_csv(filepath):
    samples = []
    try:
        with open(filepath, encoding='utf-8-sig') as f:
            for row in csv.reader(f):
                if not row: continue
                if row[0].startswith('#'): continue
                if row[0].isdigit():
                    samples.append(int(row[1]))
    except Exception:
        return None
    return samples if len(samples) >= 10 else None

def precompute_edges(samples):
    """预计算: 返回 edge_positions 列表 (已按索引排序)"""
    n = len(samples)
    arr = np.array(samples, dtype=np.float64)
    norm = arr / 128.0 + 200.0

    qiudao = np.zeros(n, dtype=int)
    for i in range(4, n):
        r1, r2, r3, r4, r5 = norm[i], norm[i-1], norm[i-2], norm[i-3], norm[i-4]
        qiudao[i] = 1 if (r1 > r2 and r2 > r3 and r3 > r4 and r4 > r5) else 0

    edges = []
    for i in range(1, n):
        if qiudao[i] != qiudao[i-1]:
            edges.append(i)
    return edges, qiudao

def count_win_edges(edges, ws, we):
    """O(log n) 二分计数 [ws, we] 内的边沿数"""
    import bisect
    left = bisect.bisect_left(edges, ws)
    right = bisect.bisect_right(edges, we)
    return right - left

def scan():
    # ── 1. 收集文件并预计算边沿 ──
    print("=" * 70)
    print("Step 1: 扫描数据目录 & 预计算边沿位置")
    print("=" * 70)

    file_records = []
    dir_stats = {}  # 按目录统计
    for subdir in sorted(DATA_DIR.iterdir()):
        if not subdir.is_dir():
            continue
        label = is_good_dir(subdir.name)
        if label is None:
            continue
        dir_files = []
        for csv_file in sorted(subdir.glob("*.csv")):
            samples = load_csv(str(csv_file))
            if samples is None:
                continue
            edges, qiudao = precompute_edges(samples)
            rec = {
                'path': str(csv_file),
                'label': 'GOOD' if label else 'DEFECT',
                'dirname': subdir.name,
                'n': len(samples),
                'edges': edges,
                'total_edges': len(edges),
            }
            file_records.append(rec)
            dir_files.append(rec)
        dir_stats[subdir.name] = {'label': 'GOOD' if label else 'DEFECT', 'count': len(dir_files)}

    good_files = [f for f in file_records if f['label'] == 'GOOD']
    defect_files = [f for f in file_records if f['label'] == 'DEFECT']
    print(f"好棒: {len(good_files)} 文件, 坏棒: {len(defect_files)} 文件, 共 {len(file_records)}")
    print(f"\n按目录统计:")
    for dn, ds in sorted(dir_stats.items()):
        print(f"  [{ds['label']:6s}] {dn}: {ds['count']} 文件")

    if not good_files or not defect_files:
        print("ERROR: 需要至少一个好棒和一个坏棒的数据!")
        return

    # ── 2. 网格搜索 ──
    print(f"\n{'='*70}")
    print(f"Step 2: 网格搜索最优窗口 (步长={ARGS.step})")
    print(f"       优化目标: 平衡准确率 = (好棒acc + 坏棒acc) / 2")
    print(f"{'='*70}")

    ws_range = range(ARGS.ws_min, ARGS.ws_max + 1, ARGS.step)
    we_range = range(ARGS.we_min, ARGS.we_max + 1, ARGS.step)
    thresh_range = range(ARGS.th_min, ARGS.th_max + 1)

    total_combos = len(ws_range) * len(we_range)
    print(f"窗口组合: {len(ws_range)} × {len(we_range)} = {total_combos}")
    print(f"阈值搜索: {ARGS.th_min}~{ARGS.th_max}")
    print(f"总迭代: {total_combos} × {len(file_records)} = {total_combos * len(file_records)} 次查表")

    results = []
    best_balanced = -1
    best_entry = None
    t0 = time.time()
    done = 0

    for ws in ws_range:
        for we in we_range:
            if we - ws < 50:
                continue
            done += 1
            if done % 200 == 0:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                eta = (total_combos - done) / rate if rate > 0 else 0
                print(f"  [{done}/{total_combos}] ws={ws} we={we}  ({rate:.0f} combo/s, ETA {eta:.0f}s)")

            good_wins = [count_win_edges(f['edges'], ws, we) for f in good_files]
            defect_wins = [count_win_edges(f['edges'], ws, we) for f in defect_files]

            g_mean = np.mean(good_wins)
            d_mean = np.mean(defect_wins)
            g_std = np.std(good_wins)
            d_std = np.std(defect_wins)

            separation = abs(d_mean - g_mean)
            spread = max((g_std + d_std) / 2, 0.1)
            fisher = separation / spread

            # 找最佳阈值 — 同时记录各类准确率
            best_th = None
            best_bal = 0      # 平衡准确率
            best_overall = 0
            best_good_acc = 0
            best_defect_acc = 0
            for th in thresh_range:
                good_correct = sum(1 for w in good_wins if w < th)
                defect_correct = sum(1 for w in defect_wins if w >= th)
                good_acc = good_correct / len(good_files)
                defect_acc = defect_correct / len(defect_files)
                overall = (good_correct + defect_correct) / len(file_records)
                balanced = (good_acc + defect_acc) / 2  # 平衡准确率

                if balanced > best_bal:
                    best_bal = balanced
                    best_th = th
                    best_overall = overall
                    best_good_acc = good_acc
                    best_defect_acc = defect_acc

            entry = {
                'ws': ws, 'we': we,
                'g_mean': round(g_mean, 2), 'd_mean': round(d_mean, 2),
                'g_std': round(g_std, 2), 'd_std': round(d_std, 2),
                'sep': round(separation, 2),
                'fisher': round(fisher, 3),
                'thresh': best_th,
                'overall_acc': round(best_overall * 100, 1),
                'good_acc': round(best_good_acc * 100, 1),
                'defect_acc': round(best_defect_acc * 100, 1),
                'balanced_acc': round(best_bal * 100, 1),
                'min_acc': round(min(best_good_acc, best_defect_acc) * 100, 1),
            }
            results.append(entry)

            if best_bal > best_balanced:
                best_balanced = best_bal
                best_entry = entry

    # 按平衡准确率排序 (平局时按 min_acc)
    results.sort(key=lambda r: (r['balanced_acc'], r['min_acc'], r['fisher']), reverse=True)

    elapsed = time.time() - t0
    print(f"\n扫描完成, 耗时 {elapsed:.1f}s")

    # ── 3. 输出 ──
    print(f"\n{'='*70}")
    print(f"*** 最优参数 (按平衡准确率排序) ***")
    print(f"{'='*70}")
    print(f"  WIN_START = {best_entry['ws']}")
    print(f"  WIN_END   = {best_entry['we']}")
    print(f"  EDGE_THRESHOLD = {best_entry['thresh']}")
    print(f"  好棒窗口翻转: {best_entry['g_mean']} ± {best_entry['g_std']}")
    print(f"  坏棒窗口翻转: {best_entry['d_mean']} ± {best_entry['d_std']}")
    print(f"  分离度:       {best_entry['sep']} (类间距离)")
    print(f"  Fisher Score: {best_entry['fisher']}")
    print(f"  ─────────────────────────────")
    print(f"  好棒准确率:   {best_entry['good_acc']}%  <- 关键!")
    print(f"  坏棒准确率:   {best_entry['defect_acc']}%")
    print(f"  总准确率:     {best_entry['overall_acc']}%")
    print(f"  平衡准确率:   {best_entry['balanced_acc']}%  (好棒+坏棒)/2")
    print(f"  最低单类:     {best_entry['min_acc']}%  <- 越高越均衡")

    # 也找 Fisher 最优的作为参考
    fisher_best = max(results, key=lambda r: r['fisher'])
    print(f"\n  参考: Fisher最优 = ws={fisher_best['ws']} we={fisher_best['we']} th={fisher_best['thresh']} "
          f"好棒={fisher_best['good_acc']}% 坏棒={fisher_best['defect_acc']}% Fisher={fisher_best['fisher']:.3f}")

    # Top 20
    print(f"\n{'='*70}")
    print(f"Top 20 参数组合 (按平衡准确率)")
    print(f"{'='*70}")
    header = f"{'Rank':<5} {'START':<7} {'END':<7} {'好μ':<8} {'坏μ':<8} {'Fisher':<9} {'阈值':<6} {'好棒%':<8} {'坏棒%':<8} {'平衡%':<8} {'总%':<7}"
    print(header)
    print("-" * len(header))
    for i, r in enumerate(results[:20]):
        print(f"{i+1:<5} {r['ws']:<7} {r['we']:<7} {r['g_mean']:<8} {r['d_mean']:<8} "
              f"{r['fisher']:<9.3f} {r['thresh']:<6} {r['good_acc']:<8} {r['defect_acc']:<8} "
              f"{r['balanced_acc']:<8} {r['overall_acc']:<7}")

    # 按目录逐组分析最优参数的效果
    print(f"\n{'='*70}")
    print(f"最优参数 ({best_entry['ws']}-{best_entry['we']} th={best_entry['thresh']}) 按目录逐组分析")
    print(f"{'='*70}")
    print(f"{'目录':<35s} {'标签':<6s} {'文件数':<7} {'好棒%':<8} {'坏棒%':<8}")
    print("-" * 65)
    for dn, ds in sorted(dir_stats.items()):
        dir_files = [f for f in file_records if f['dirname'] == dn]
        dir_wins = [count_win_edges(f['edges'], best_entry['ws'], best_entry['we']) for f in dir_files]
        th = best_entry['thresh']
        if ds['label'] == 'GOOD':
            correct = sum(1 for w in dir_wins if w < th)
            print(f"  {dn:<33s} {'GOOD':<6s} {ds['count']:<7} {correct/ds['count']*100:<8.1f} {'-':<8}")
        else:
            correct = sum(1 for w in dir_wins if w >= th)
            print(f"  {dn:<33s} {'DEFECT':<6s} {ds['count']:<7} {'-':<8} {correct/ds['count']*100:<8.1f}")

    # 热力图提示
    print(f"\n平衡准确率最高的区域:")
    top5 = results[:5]
    ws_vals = sorted(set(r['ws'] for r in top5))
    we_vals = sorted(set(r['we'] for r in top5))
    print(f"  WIN_START 最佳范围: {min(ws_vals)} ~ {max(ws_vals)}")
    print(f"  WIN_END   最佳范围: {min(we_vals)} ~ {max(we_vals)}")
    print(f"\n★ 推荐参数: WIN_START={best_entry['ws']} WIN_END={best_entry['we']} THRESHOLD={best_entry['thresh']}")
    print(f"  好棒={best_entry['good_acc']}% 坏棒={best_entry['defect_acc']}% 平衡={best_entry['balanced_acc']}%")

    # 保存
    out_path = DATA_DIR.parent / "window_scan_results.json"
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({
            'best_balanced': best_entry,
            'best_fisher': fisher_best,
            'top20': results[:20],
            'all': results,
            'n_good': len(good_files),
            'n_defect': len(defect_files),
            'n_total': len(file_records),
            'dir_stats': dir_stats,
            'scan_step': ARGS.step,
        }, f, ensure_ascii=False, indent=2)
    print(f"\n完整结果: {out_path}")


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='扫描最优分析窗口参数')
    ap.add_argument('--step', type=int, default=10)
    ap.add_argument('--ws-min', type=int, default=30)
    ap.add_argument('--ws-max', type=int, default=180)
    ap.add_argument('--we-min', type=int, default=250)
    ap.add_argument('--we-max', type=int, default=520)
    ap.add_argument('--th-min', type=int, default=7)
    ap.add_argument('--th-max', type=int, default=16)
    ARGS = ap.parse_args()
    scan()
