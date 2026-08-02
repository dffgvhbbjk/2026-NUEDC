# -*- coding: utf-8 -*-
"""用已知缺陷距离的 512 点数据，校准PC端波速系数"""
import csv, os, numpy as np

ROOT = r'd:\FPGA\dian_sai\Project\God3.11\God3.0\data'
SAMPLE_US = 10.42  # μs

def read_hit(fpath):
    samples = []
    defect_mm = None
    with open(fpath, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if row and row[0].startswith('# defect_true_mm'):
                v = row[1].strip()
                defect_mm = int(v) if v else None
            if row and row[0].isdigit():
                samples.append(int(row[1]))
    return np.array(samples, dtype=np.float64), defect_mm


def find_impact_peak(samples):
    """找敲击脉冲峰值: 第一个绝对值最大的峰"""
    abs_s = np.abs(samples)
    # 找前 100 点的最大值
    peak_idx = np.argmax(abs_s[:100])
    return peak_idx


def find_defect_peak(samples, search_start):
    """在敲击峰之后找缺陷反射峰"""
    abs_s = np.abs(samples)
    # 跳过敲击峰附近，在后续区域找最大的峰
    skip = 30  # 跳过敲击峰后的振荡
    region = abs_s[search_start + skip:]
    if len(region) == 0:
        return None
    peak_idx = search_start + skip + np.argmax(region)
    return peak_idx


def main():
    # 收集所有 512pt 缺陷数据
    all_results = {}  # defect_mm -> [(Δsamples, velocity), ...]

    for sd in sorted(os.listdir(ROOT)):
        full = os.path.join(ROOT, sd)
        if not os.path.isdir(full):
            continue
        files = sorted([f for f in os.listdir(full)
                       if f.startswith('hit_') and f.endswith('.csv')])

        for fname in files:
            fpath = os.path.join(full, fname)
            samples, defect_mm = read_hit(fpath)

            if len(samples) < 256:
                continue
            if defect_mm is None:
                continue  # 好棒，跳过

            imp = find_impact_peak(samples)
            defect = find_defect_peak(samples, imp)

            if defect is None:
                continue

            dsamples = defect - imp
            # v = 2 * distance / (Δt)
            # Δt = dsamples * SAMPLE_US * 1e-6 (seconds)
            # v(m/s) = 2 * distance(mm) / 1000 / (dsamples * SAMPLE_US * 1e-6)
            v_ms = 2 * defect_mm / 1000 / (dsamples * SAMPLE_US * 1e-6)

            if defect_mm not in all_results:
                all_results[defect_mm] = []
            all_results[defect_mm].append({
                'file': fname,
                'imp': imp,
                'defect': defect,
                'dsamples': dsamples,
                'v_ms': v_ms,
                'defect_mm': defect_mm,
            })

    print("=" * 70)
    print("波速校准 — 基于各缺陷距离的敲击数据")
    print("=" * 70)

    all_velocities = []
    for defect_mm in sorted(all_results.keys()):
        results = all_results[defect_mm]
        velocities = [r['v_ms'] for r in results if 500 < r['v_ms'] < 5000]  # 过滤明显异常
        ds = [r['dsamples'] for r in results if 500 < r['v_ms'] < 5000]

        if velocities:
            v_mean = np.mean(velocities)
            v_std = np.std(velocities)
            d_mean = np.mean(ds)
            print(f"\n缺陷 {defect_mm}mm ({len(velocities)} 次有效):")
            print(f"  平均 Δ样点: {d_mean:.1f}")
            print(f"  平均波速: {v_mean:.0f} m/s  (±{v_std:.0f})")
            print(f"  前5次: {[f'{v:.0f}' for v in velocities[:5]]}")
            all_velocities.extend(velocities)

    print(f"\n{'='*70}")
    if all_velocities:
        v_all_mean = np.mean(all_velocities)
        v_all_std = np.std(all_velocities)
        print(f"全部数据综合:")
        print(f"  最佳波速: {v_all_mean:.0f} m/s")
        print(f"  标准差: {v_all_std:.0f} m/s")
        print(f"  变异系数: {v_all_std/v_all_mean*100:.1f}%")

        # 等效系数 (PC端)
        # distance_mm = v × Δsamples × 10.42μs / 2 × 1000
        #              = v × Δsamples × 0.00521
        #              = coeff × Δsamples
        coeff = v_all_mean * SAMPLE_US * 1e-6 / 2 * 1000
        print(f"\n  PC端等效系数:")
        print(f"  distance_mm = {coeff:.2f} × Δsamples")
        print(f"  (对比 Verilog: distance_mm = 15 × pulse_width)")

        print(f"\n  Web UI 波速输入框建议值: {v_all_mean:.0f}")

        # 各缺陷的理论 Δ样点
        print(f"\n  理论值验证 (v={v_all_mean:.0f} m/s):")
        for d_mm in [200, 305, 307, 695, 800, 1000]:
            theoretical_ds = d_mm * 2 / (v_all_mean * SAMPLE_US * 1e-6) / 1000
            print(f"    缺陷{d_mm}mm → 理论Δ样点 = {theoretical_ds:.1f}")

    else:
        print("没有有效数据!")


if __name__ == '__main__':
    main()
