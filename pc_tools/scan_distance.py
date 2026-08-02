#!/usr/bin/env python3
"""
方法5: 匹配滤波 (Matched Filter) + 多方法综合对比

用冲击脉冲自身做模板，在整个波形上做互相关。
互相关峰 = 反射波到达时刻。

同时综合之前所有方法的结果进行对比。
"""

import sys, os
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "pc_tools"))

import csv, json
import numpy as np

DATA_DIR = Path(r"D:\FPGA\dian_sai\Project\God3.11\God3.0\data")

def extract_defect_mm(dirname: str):
    import re
    for kw in ['200','307','695','800']:
        if kw in dirname.lower(): return int(kw)
    m = re.search(r'_(\d{3})$', dirname)
    if m: return int(m.group(1))
    return None

def is_defect_dir(dirname: str):
    d = dirname.lower()
    if any(k in d for k in ['好棒','好柱','good']): return False
    if any(k in d for k in ['坏棒','坏柱','缺陷','defect','bad']): return True
    return None

def load_csv(filepath):
    samples = []
    try:
        with open(filepath, encoding='utf-8-sig') as f:
            for row in csv.reader(f):
                if not row: continue
                if row[0].startswith('#'): continue
                if row[0].isdigit(): samples.append(int(row[1]))
    except Exception: return None
    return samples if len(samples) >= 10 else None

def analyze_all_methods(samples):
    """一次分析，返回所有方法的测距特征"""
    n = len(samples)
    arr = np.array(samples, dtype=np.float64)
    result = {}

    # 1. 冲击峰
    impact_idx = int(np.argmax(np.abs(arr)))
    result['impact_idx'] = impact_idx
    result['impact_val'] = float(arr[impact_idx])

    # 2. qiudao 边沿
    norm = arr / 128.0 + 200.0
    qiudao = np.zeros(n, dtype=int)
    for i in range(4, n):
        r1,r2,r3,r4,r5 = norm[i],norm[i-1],norm[i-2],norm[i-3],norm[i-4]
        qiudao[i] = 1 if (r1>r2 and r2>r3 and r3>r4 and r4>r5) else 0

    edges = []
    for i in range(1, n):
        if qiudao[i] != qiudao[i-1]:
            edges.append(i)

    result['n_edges_total'] = len(edges)
    result['edges'] = edges

    # 冲击峰后的边沿
    edges_after = [e for e in edges if e > impact_idx]
    result['n_edges_after'] = len(edges_after)
    for i, ep in enumerate(edges_after):
        if i < 7:
            result[f'e{i+1}_after'] = ep

    # 冲击峰前的边沿
    edges_before = [e for e in edges if e < impact_idx]
    result['n_edges_before'] = len(edges_before)

    # 3. 互相关匹配滤波
    # 提取冲击脉冲模板: [impact_idx-5, impact_idx+20]
    t_start = max(0, impact_idx - 5)
    t_end = min(n, impact_idx + 25)
    template = arr[t_start:t_end].copy()
    # 去直流
    template = template - np.mean(template)
    template_norm = template / (np.sqrt(np.sum(template**2)) + 1e-6)

    # 在整个信号上做互相关
    arr_norm = arr - np.mean(arr)
    # 滑动点积
    corr = np.zeros(n - len(template) + 1)
    for i in range(len(corr)):
        seg = arr_norm[i:i+len(template)]
        seg_norm = seg / (np.sqrt(np.sum(seg**2)) + 1e-6)
        corr[i] = np.dot(seg_norm, template_norm)

    result['corr'] = corr

    # 在冲击峰后的区域找互相关峰
    # 跳过冲击峰自身 ([impact_idx, impact_idx+len(template)] 是自相关峰)
    skip_start = max(0, impact_idx - 5)
    skip_end = min(len(corr), impact_idx + len(template) + 5)

    # 找冲击后第一个显著相关峰
    corr_thresh = 0.3  # 相关阈值
    best_corr_peak = None
    best_corr_val = -1
    for i in range(skip_end, len(corr)):
        if (corr[i] > corr_thresh and
            (i == skip_end or corr[i] > corr[i-1]) and
            (i == len(corr)-1 or corr[i] > corr[i+1])):
            if corr[i] > best_corr_val:
                best_corr_val = corr[i]
                best_corr_peak = i

    # 也尝试找绝对值最大的相关峰 (可能负相关=反向反射)
    best_corr_abs_peak = None
    best_corr_abs_val = -1
    for i in range(skip_end, len(corr)):
        abs_corr = abs(corr[i])
        if abs_corr > 0.3:
            if (i == skip_end or abs_corr > abs(corr[i-1])) and \
               (i == len(corr)-1 or abs_corr > abs(corr[i+1])):
                if abs_corr > best_corr_abs_val:
                    best_corr_abs_val = abs_corr
                    best_corr_abs_peak = i

    result['corr_peak'] = best_corr_peak
    result['corr_peak_abs'] = best_corr_abs_peak

    # 4. 能量突变检测 (冲击峰后，用低通滤波)
    # 5点滑动平均
    smoothed = np.convolve(np.abs(arr), np.ones(5)/5, mode='same')

    # 找冲击峰后第一个偏离指数衰减的点
    # 简单做法: 看二阶差分
    tail_start = impact_idx + 10
    tail_end = min(impact_idx + 120, n)
    if tail_end - tail_start > 20:
        diff2 = np.diff(np.diff(smoothed[tail_start:tail_end]))
        if len(diff2) > 0:
            energy_peak_offset = int(np.argmax(np.abs(diff2)))
            result['energy_peak'] = tail_start + energy_peak_offset
        else:
            result['energy_peak'] = None
    else:
        result['energy_peak'] = None

    return result


def evaluate_method(pairs, method_name):
    """评估一组 (true_mm, sample_pos) 对"""
    if len(pairs) < 50:
        return None

    trues = np.array([p[0] for p in pairs], dtype=np.float64)
    svals = np.array([p[1] for p in pairs], dtype=np.float64)

    # 过原点最小二乘系数
    coeff = np.sum(trues * svals) / max(np.sum(svals**2), 1e-6)
    preds = svals * coeff
    errors = preds - trues

    return {
        'name': method_name,
        'n': len(pairs),
        'coeff': round(coeff, 2),
        'mae': round(np.mean(np.abs(errors)), 1),
        'rmse': round(np.sqrt(np.mean(errors**2)), 1),
        'w20': round(np.mean(np.abs(errors) <= 20) * 100, 1),
        'w50': round(np.mean(np.abs(errors) <= 50) * 100, 1),
        'w100': round(np.mean(np.abs(errors) <= 100) * 100, 1),
    }


def scan():
    print("="*70)
    print("Step 1: 分析所有缺陷文件")
    print("="*70)

    all_results = []
    for subdir in sorted(DATA_DIR.iterdir()):
        if not subdir.is_dir(): continue
        mm = extract_defect_mm(subdir.name)
        if mm is None or not is_defect_dir(subdir.name): continue

        for csv_file in sorted(subdir.glob("*.csv")):
            samples = load_csv(str(csv_file))
            if samples is None: continue
            feat = analyze_all_methods(samples)
            feat['defect_mm'] = mm
            feat['dirname'] = subdir.name
            feat['path'] = str(csv_file)
            all_results.append(feat)

    print(f"总计: {len(all_results)} 个缺陷文件")

    # Step 2: 评估各种方法
    print(f"\n{'='*70}")
    print(f"Step 2: 多方法评估")
    print(f"理论系数: v=2200m/s, Ts=10.42us -> 11.46 mm/sample")
    print(f"{'='*70}")

    methods_pairs = {}

    for f in all_results:
        imp = f['impact_idx']
        mm = f['defect_mm']

        # M1: 互相关峰 (正峰)
        if f['corr_peak'] is not None and f['corr_peak'] > imp:
            methods_pairs.setdefault('M1_corr_pos', []).append((mm, f['corr_peak']))

        # M2: 互相关峰 (绝对值最大)
        if f['corr_peak_abs'] is not None and f['corr_peak_abs'] > imp:
            methods_pairs.setdefault('M2_corr_abs', []).append((mm, f['corr_peak_abs']))

        # M3: 互相关峰相对冲击
        if f['corr_peak_abs'] is not None and f['corr_peak_abs'] > imp:
            methods_pairs.setdefault('M3_corr_rel', []).append((mm, f['corr_peak_abs'] - imp))

        # M4: 第2次边沿后相对冲击
        if f.get('e2_after') is not None:
            methods_pairs.setdefault('M4_e2_rel', []).append((mm, f['e2_after'] - imp))

        # M5: 第3次边沿后相对冲击
        if f.get('e3_after') is not None:
            methods_pairs.setdefault('M5_e3_rel', []).append((mm, f['e3_after'] - imp))

        # M6: 第1次边沿后相对冲击
        if f.get('e1_after') is not None:
            methods_pairs.setdefault('M6_e1_rel', []).append((mm, f['e1_after'] - imp))

        # M7: 能量突变
        if f.get('energy_peak') is not None:
            methods_pairs.setdefault('M7_energy_rel', []).append((mm, f['energy_peak'] - imp))

        # M8: 第1次边沿后绝对位置
        if f.get('e1_after') is not None:
            methods_pairs.setdefault('M8_e1_abs', []).append((mm, f['e1_after']))

        # M9: 第2次边沿后绝对位置
        if f.get('e2_after') is not None:
            methods_pairs.setdefault('M9_e2_abs', []).append((mm, f['e2_after']))

        # M10: 第3次边沿后绝对位置
        if f.get('e3_after') is not None:
            methods_pairs.setdefault('M10_e3_abs', []).append((mm, f['e3_after']))

    # 评估并排序
    all_evals = []
    for key, pairs in sorted(methods_pairs.items()):
        ev = evaluate_method(pairs, key)
        if ev:
            all_evals.append(ev)

    all_evals.sort(key=lambda e: e['mae'])

    print(f"\n{'方法':<20s} {'N':<7} {'系数':<7} {'MAE':<9} {'RMSE':<9} {'<=20mm':<8} {'<=50mm':<8} {'<=100mm':<8}")
    print("-" * 85)
    for ev in all_evals:
        print(f"  {ev['name']:<19s} {ev['n']:<7} {ev['coeff']:<7.2f} {ev['mae']:<9.1f} {ev['rmse']:<9.1f} {ev['w20']:<8.1f} {ev['w50']:<8.1f} {ev['w100']:<8.1f}")

    # 保存
    out_path = DATA_DIR.parent / "distance_scan_results.json"
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump([ev for ev in all_evals], f, ensure_ascii=False, indent=2)
    print(f"\n结果: {out_path}")


if __name__ == '__main__':
    scan()
