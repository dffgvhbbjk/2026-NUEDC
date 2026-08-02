#!/usr/bin/env python3
"""
精确复现 FPGA defect_qiudao_classifier 的 B2 测距逻辑, 与 PC 方法对比。

FPGA 关键细节:
  1. 峰值扫描: imp = argmax(|sample|) 遍历全部 512 点
  2. 5点流水线: qiudao[i] = (r1>r2)&&(r2>r3)&&(r3>r4)&&(r4>r5)
     在 chk_cnt=k 时 r1=norm(sample[k-1]) ... r5=norm(sample[k-5])
     => qiudao_next 对应 PC 的 qiudao[k-1]
  3. 边沿检测: 在 chk_cnt=k 时比较 PC_qiudao[k-2] ^ PC_qiudao[k-3]
     => 边沿(chk_cnt=k) 对应 PC 过渡点 k-2
  4. B2: imp 之后的第 2 个边沿, result_dist = (chk_cnt - imp) * 235 >> 4
"""

import csv, sys, glob, os
import numpy as np

DATA_DIR = r"D:\FPGA\dian_sai\Project\God3.11\God3.0\data"

def load_csv(filepath):
    samples = []
    with open(filepath, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if not row: continue
            if row[0].startswith('#'): continue
            if row[0].isdigit(): samples.append(int(row[1]))
    return np.array(samples, dtype=np.float64)

def pc_method(arr):
    """PC analyze_samples 的 B2 方法"""
    n = len(arr)
    norm = arr / 128.0 + 200.0
    qiudao = np.zeros(n, dtype=int)
    for i in range(4, n):
        r1,r2,r3,r4,r5 = norm[i],norm[i-1],norm[i-2],norm[i-3],norm[i-4]
        qiudao[i] = 1 if (r1>r2 and r2>r3 and r3>r4 and r4>r5) else 0
    edges = [i for i in range(1,n) if qiudao[i]!=qiudao[i-1]]
    impact_idx = int(np.argmax(np.abs(arr)[:min(100,n)]))
    edges_after = [e for e in edges if e > impact_idx]
    e1 = edges_after[0] if len(edges_after)>0 else None
    e2 = edges_after[1] if len(edges_after)>1 else None
    e3 = edges_after[2] if len(edges_after)>2 else None
    reflection_pos = (e2 - impact_idx) if e2 is not None else 0
    return dict(impact=impact_idx, e1=e1, e2=e2, e3=e3, reflection_pos=reflection_pos,
                dist=reflection_pos*14.67 if e2 else 0, n_edges=len(edges))

def fpga_norm_int(s):
    """FPGA 归一化: 有符号整数除法 (向零) + 200, 结果截断到 17 位有符号"""
    q = int(s / 128.0)          # 向零截断 (同 Verilog 有符号除法)
    v = q + 200
    v = v & 0x1FFFF             # 17位
    if v & 0x10000: v -= 0x20000
    return v

def fpga_norm_raw(s):
    """备选: 直接用原始样点比较 (同 PC 浮点等价)"""
    return int(s)

def fpga_method(arr, use_raw=False):
    """精确复现 FPGA 逻辑"""
    n = len(arr)
    # 归一化: FPGA 有符号整数除法/17位截断; 或直接用原始样点
    norm = np.array([fpga_norm_raw(s) if use_raw else fpga_norm_int(s) for s in arr], dtype=np.float64)
    # qiudao: FPGA 在 chk_cnt=k 时 r1=norm[k-1], r5=norm[k-5]
    qiudao = np.zeros(n, dtype=int)
    for k in range(5, n):
        r1,r2,r3,r4,r5 = norm[k-1],norm[k-2],norm[k-3],norm[k-4],norm[k-5]
        qiudao[k] = 1 if (r1>r2 and r2>r3 and r3>r4 and r4>r5) else 0
    # 边沿: FPGA 在 chk_cnt=k 检测 qiudao[k-2]^qiudao[k-3], 有效 k>=7
    # 等价于 PC 边沿点 ep, FPGA chk_cnt = ep+2
    edges_pc = [i for i in range(2,n) if qiudao[i]!=qiudao[i-1]]  # PC 风格边沿点
    edges_chk = [e+2 for e in edges_pc]  # FPGA 的 chk_cnt

    # 峰值扫描: argmax(|arr|) 前 100 样点 (修复后与 PC 一致; 原来遍历全部会导致 imp 错位)
    abs_arr = np.abs(arr)
    imp = int(np.argmax(abs_arr[:min(100, n)]))
    # FPGA 的边沿在 chk_cnt > imp 才计数
    edges_after_chk = [c for c in edges_chk if c > imp]
    e1c = edges_after_chk[0] if len(edges_after_chk)>0 else None
    e2c = edges_after_chk[1] if len(edges_after_chk)>1 else None
    e3c = edges_after_chk[2] if len(edges_after_chk)>2 else None
    if e2c is not None:
        raw = e2c - imp - 3  # 补偿 +3 拍流水线延迟 (EDGE_LATENCY), 与 PC 距离一致
        dist_fpga = (raw * 235) >> 4
    else:
        raw, dist_fpga = None, 0
    return dict(imp=imp, e1_chk=e1c, e2_chk=e2c, e3_chk=e3c, raw_reflect=raw, dist=dist_fpga)

def main():
    # 选一个 800mm 文件
    globs = glob.glob(os.path.join(DATA_DIR, "*_800*", "*.csv"))
    print(f"找到 {len(globs)} 个 800mm 文件")
    for path in globs[:5]:
        arr = load_csv(path)
        pc = pc_method(arr)
        fp = fpga_method(arr)
        print("="*80)
        print(f"文件: {os.path.basename(path)}  (n={len(arr)})")
        print(f"  PC  : impact={pc['impact']} e1={pc['e1']} e2={pc['e2']} e3={pc['e3']} "
              f"reflection_pos={pc['reflection_pos']} -> {pc['dist']:.0f}mm")
        print(f"  FPGA: imp={fp['imp']} e1_chk={fp['e1_chk']} e2_chk={fp['e2_chk']} e3_chk={fp['e3_chk']} "
              f"raw(chk-imp)={fp['raw_reflect']} -> {fp['dist']}mm (回传)")

if __name__ == '__main__':
    main()
