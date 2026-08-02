# -*- coding: utf-8 -*-
"""Verilog 算法分析：检查 512 样点波形是否到达第 4~6 次方向翻转"""
import csv, os, sys
import numpy as np

DATA_ROOT = r'd:\FPGA\dian_sai\Project\God3.11\God3.0\data'

def analyze_file(fpath):
    """读取 CSV，用 Verilog 算法分析方向翻转"""
    samples = []
    with open(fpath, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if row and row[0].isdigit():
                samples.append(int(row[1]))

    n = len(samples)
    arr = np.array(samples, dtype=np.float64)

    # ── 1. 归一化: data/128 + 200 ──
    norm = arr / 128.0 + 200.0

    # ── 2. 5点连续递减检测 ──
    qiudao = np.zeros(n, dtype=int)
    for i in range(4, n):
        r1, r2, r3, r4, r5 = norm[i], norm[i-1], norm[i-2], norm[i-3], norm[i-4]
        qiudao[i] = 1 if (r1 > r2 and r2 > r3 and r3 > r4 and r4 > r5) else 0

    # ── 3. 方向翻转 (0→1 或 1→0) ──
    edge_positions = []
    for i in range(1, n):
        if qiudao[i] != qiudao[i-1]:
            edge_positions.append(i)

    # edge_cnt 在每个翻转位置的值
    edge_cnt_at = {}  # pos -> count
    cnt = 0
    for ep in edge_positions:
        cnt += 1
        edge_cnt_at[ep] = cnt

    # ── 4. 分析窗口 [81, 412] (ADC样点, 对应Verilog work_cnt [106,540]) ──
    WIN_START, WIN_END = 81, min(412, n)

    # ── 5. 找到 edge_cnt ∈ [4,6] 的区域，测量下降段宽度 ──
    # 找到第4次翻转位置 和 第7次翻转位置（第6次结束）
    edge_4_pos = None
    edge_7_pos = None
    for ep in edge_positions:
        if edge_cnt_at[ep] == 4:
            edge_4_pos = ep
        if edge_cnt_at[ep] == 7:
            edge_7_pos = ep
            break

    return {
        'fname': os.path.basename(fpath),
        'n': n,
        'norm_range': (norm.min(), norm.max()),
        'qiudao_count': int(qiudao.sum()),
        'total_edges': len(edge_positions),
        'edge_positions': edge_positions,
        'edge_cnt_at': edge_cnt_at,
        'edge_4_pos': edge_4_pos,
        'edge_7_pos': edge_7_pos,
        'norm': norm,
        'arr': arr,
        'qiudao': qiudao,
        'win_start': WIN_START,
        'win_end': WIN_END,
    }


def main():
    # 找一个数据目录
    subdirs = sorted([d for d in os.listdir(DATA_ROOT)
                      if os.path.isdir(os.path.join(DATA_ROOT, d))])
    if not subdirs:
        print("未找到数据目录!")
        return

    # 优先找 512 点的数据，否则用第一个
    target_dir = None
    for sd in subdirs:
        full = os.path.join(DATA_ROOT, sd)
        for f in os.listdir(full):
            if f.startswith('hit_') and f.endswith('.csv'):
                fpath = os.path.join(full, f)
                with open(fpath, encoding='utf-8-sig') as fh:
                    for row in csv.reader(fh):
                        if row and row[0].startswith('# points'):
                            pts = int(row[1])
                            if pts >= 512:
                                target_dir = full
                                break
                break
        if target_dir:
            break

    if target_dir is None:
        target_dir = os.path.join(DATA_ROOT, subdirs[0])

    print(f"分析目录: {target_dir}")

    files = sorted([f for f in os.listdir(target_dir)
                    if f.startswith('hit_') and f.endswith('.csv')])[:5]

    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

    for fname in files:
        fpath = os.path.join(target_dir, fname)
        r = analyze_file(fpath)

        print(f"\n{'='*60}")
        print(f"[{r['fname']}]  {r['n']}pt")
        print(f"  norm range: {r['norm_range'][0]:.0f} ~ {r['norm_range'][1]:.0f}")
        print(f"  qiudao (5-pt decreasing) count: {r['qiudao_count']}")
        print(f"  total direction edges: {r['total_edges']}")

        print(f"  First 10 edge positions:")
        for ep in r['edge_positions'][:10]:
            cnt = r['edge_cnt_at'][ep]
            print(f"    edge#{cnt} @ sample {ep}  (ADC={r['arr'][ep]:.0f})")

        e4 = r['edge_4_pos']
        if e4 is not None:
            in_win = 'IN WINDOW' if r['win_start'] <= e4 <= r['win_end'] else 'OUT OF WINDOW!'
            print(f"  edge#4 @ sample {e4}  -> {in_win}")

        e7 = r['edge_7_pos']
        if e7 is not None:
            in_win = 'IN WINDOW' if r['win_start'] <= e7 <= r['win_end'] else 'OUT OF WINDOW!'
            print(f"  edge#7 @ sample {e7}  -> {in_win}")

        if e4 is not None and e7 is not None:
            region_qiudao = r['qiudao'][e4:e7]
            max_len = 0; cur_len = 0; best_start = 0
            for i, v in enumerate(region_qiudao):
                if v == 1:
                    if cur_len == 0:
                        seg_start = e4 + i
                    cur_len += 1
                    if cur_len > max_len:
                        max_len = cur_len
                        best_start = seg_start
                else:
                    cur_len = 0

            dist_verilog = max_len * 15
            print(f"\n  *** Region [edge#4={e4}, edge#7={e7}] analysis: ***")
            print(f"    interval length: {e7 - e4} samples")
            print(f"    longest falling segment: {max_len} cycles @ sample {best_start}")
            print(f"    Verilog distance: {max_len} * 15 = {dist_verilog} mm")
            print(f"    (truth: 305mm at 1000mm rod)")
            print(f"    error: {dist_verilog - 305:+d} mm")
            if max_len > 0:
                equiv_v = 305 * 2 / (max_len * 10.42e-6) / 1000
                print(f"    equiv velocity (back-calc from 305mm): {equiv_v:.0f} m/s")

        win_end_cnt = 0
        for ep in r['edge_positions']:
            if ep <= r['win_end']:
                win_end_cnt = r['edge_cnt_at'][ep]
        if win_end_cnt < 4:
            print(f"\n  WARNING: only {win_end_cnt} edges in window [{r['win_start']}, {r['win_end']}]!")

    print(f"\n{'='*60}")
    print("Summary:")
    print("  1m nylon rod, round-trip ~0.91ms ~87 samples")
    print("  512 samples can hold ~12 direction edges")
    print("  edges #4-#6 should fall around samples 150-250, well within [81,412]")


if __name__ == '__main__':
    main()
