#!/usr/bin/env python3
"""
pile_detector.py — 基桩动测仪 PC端验证脚本
=============================================

功能:
  1. 实现与FPGA defect_ac_classifier.v 完全一致的算法 (bit-accurate)
  2. 加载预训练的 LDA 权重 (defect_lda_weights.mem)
  3. 支持从CSV文件或UART串口读取数据
  4. 自动判断桩的好坏(GOOD/DEFECT/INVALID/NOISE)
  5. 对坏桩计算缺陷位置 (200/307/695/800mm)
  6. 批量测试所有数据文件, 输出统计报告

算法流程 (与 fpga_model.py 和 RTL 一一对应):
  1) 512点采集帧内找 |x| 峰值位置 imp (主冲击)
  2) 有效性: peak<MIN_PEAK 或 imp>=IMP_WIN -> INVALID
  3) 自动缩放: sh=max(0,bits(peak)-15); s[i]=x[imp+16+i]>>sh
  4) 整数自相关 r[k]=sum s[i]*s[i+k], k=0..135
  5) 二次缩放: rsh=max(0,bits(r0)-30); rs[k]=r[k]>>rsh
  6) LDA打分 + argmax
  7) 800mm 二次判别: 若 pred=good 且 T0>=103 -> 改判 d800

FPGA 资源: EP4CE6, 30×M9K, ~4500 LE
单帧分析周期: ~46k拍 @40MHz ≈ 1.15ms

Author: AI Analysis
Date: 2026-07-30
"""

import os
import sys
import csv
import struct
import json
import time
import argparse
from pathlib import Path
from collections import defaultdict
import warnings
warnings.filterwarnings('ignore')

import numpy as np

# ============================================================================
# Constants (must match FPGA defect_ac_classifier.v exactly)
# ============================================================================
FS = 95880.0                    # 采样率 Hz
SAMPLE_PERIOD_US = 10.42        # 采样周期 us
SKIP = 16                       # 跳过冲击后16点
SEGLEN = 384                    # 分析段长度
NLAG = 136                      # 自相关 lag 数: k=0..135
LAG0 = 8                        # 打分起点 lag
IMP_WIN = 112                   # 冲击峰必须在帧前段
MIN_PEAK = 300000               # 最小峰值 (24位ADC)
MARGIN_Q = 82                   # 低置信门限 (0.02*4096)
QW = 12                         # 权重 Q12 定点

# 800mm 二次判别阈值 (与RTL一致)
D800_LAG_LO = 90
D800_LAG_HI = 110
D800_T_TH = 103                 # 基波周期 >= 103 -> 800mm 缺陷

# 类别定义
CLS_NAMES = ["good", "d200", "d307", "d695", "d800"]
CLS_DIST = [0, 200, 307, 695, 800]

# 组号到类别的映射
GRP_TO_CLS = {
    5: 0, 6: 0, 7: 0, 8: 0,           # good
    21: 1, 22: 1,                       # d200
    13: 2, 14: 2, 15: 2, 16: 2,        # d307
    9: 3, 10: 3, 11: 3, 12: 3,         # d695
    17: 4, 18: 4, 19: 4, 20: 4,        # d800
    1: 2,                               # 305mm旧批次 -> d307
    4: 3,                               # 695mm旧批次 -> d695
    3: 0,                               # 正常棒旧批次 -> good
}

# 铁头组 (用于泛化测试)
IRON_GROUPS = {7, 8, 11, 12, 15, 16, 19, 20, 22}

DATA_DIR = Path(r"D:\FPGA\dian_sai\Project\God3.11\God3.0\data")
WEIGHTS_FILE = Path(r"D:\FPGA\dian_sai\Project\God3.11\God3.0\rtl\defect_lda_weights.mem")


# ============================================================================
# LDA Weights Loading
# ============================================================================
def load_lda_weights(mem_path):
    """
    从 defect_lda_weights.mem 加载 Q12 定点权重。
    格式: 1280行 16位十六进制 (5类 × 256地址, idx=129..255补0)
    线性地址 = cls*256 + idx
    返回: Wq (129, 5), int16
    """
    with open(mem_path, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]

    Wq = np.zeros((129, 5), dtype=np.int16)
    for cls in range(5):
        for idx in range(129):
            addr = cls * 256 + idx
            if addr < len(lines):
                val = int(lines[addr], 16)
                if val > 32767:
                    val -= 65536  # 有符号16位
                Wq[idx, cls] = val
    return Wq


# ============================================================================
# Data Loading
# ============================================================================
def load_csv_hit(filepath):
    """从CSV文件加载波形数据和元数据。返回 (waveform, header_dict, group_id)"""
    hdr = {}
    vals = []
    with open(filepath, 'r', encoding='utf-8-sig', errors='replace') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                parts = line[1:].split(',', 1)
                if len(parts) == 2:
                    hdr[parts[0].strip()] = parts[1].strip()
                continue
            if line.startswith('index'):
                continue
            parts = line.split(',')
            if len(parts) >= 2:
                try:
                    vals.append(int(parts[1]))
                except ValueError:
                    pass

    # Extract group ID from directory name
    parent = Path(filepath).parent.name
    try:
        grp_id = int(parent.split('_')[0])
    except (ValueError, IndexError):
        grp_id = 0

    return np.array(vals, dtype=np.int64), hdr, grp_id


def parse_uart_frame(data_bytes):
    """
    解析UART二进制帧 (与 wave_uart_export.v 格式一致)

    帧格式:
      0xAA 0x55        同步头
      len_sel          0=64点 1=128点 2=256点 3=512点
      sample[k][23:16], [15:8], [7:0] ... (有符号24位)
      checksum         (len_sel + 所有数据字节的累加和)
      ---- 结果段 (13字节) ----
      0x5A             结果段标记
      ver              0x02
      status           bit[1:0]=result_state
      idx_h            impact[8]|defect[8]|bottom[8]
      impact_index[7:0]
      defect_index[7:0]
      bottom_index[7:0]
      distance[11:8], distance[7:0]
      threshold[23:16], [15:8], [7:0]
      rsum             结果段校验和
    """
    if len(data_bytes) < 7:
        return None, "Frame too short"

    # 找同步头
    sync_idx = -1
    for i in range(len(data_bytes) - 1):
        if data_bytes[i] == 0xAA and data_bytes[i+1] == 0x55:
            sync_idx = i
            break

    if sync_idx < 0:
        return None, "No sync header found"

    data_bytes = data_bytes[sync_idx:]

    if len(data_bytes) < 4:
        return None, "Frame too short after sync"

    len_sel = data_bytes[2]
    n_points = {0: 64, 1: 128, 2: 256, 3: 512}.get(len_sel, 256)
    expected_len = 4 + 3 * n_points + 1  # sync(2) + len(1) + data + chksum(1)

    if len(data_bytes) < expected_len:
        return None, f"Expected {expected_len} bytes, got {len(data_bytes)}"

    # 读取样点 (24位有符号)
    waveform = []
    idx = 3
    for i in range(n_points):
        if idx + 2 >= len(data_bytes):
            break
        sample = (data_bytes[idx] << 16) | (data_bytes[idx+1] << 8) | data_bytes[idx+2]
        if sample & 0x800000:  # 符号扩展
            sample -= 0x1000000
        waveform.append(sample)
        idx += 3

    waveform = np.array(waveform, dtype=np.int64)

    # 结果段 (如果有)
    results = {}
    if idx + 13 <= len(data_bytes):
        if data_bytes[idx] == 0x5A:  # 结果段标记
            idx += 1
            ver = data_bytes[idx]; idx += 1
            status = data_bytes[idx]; idx += 1
            idx_h = data_bytes[idx]; idx += 1
            imp_lo = data_bytes[idx]; idx += 1
            def_lo = data_bytes[idx]; idx += 1
            bot_lo = data_bytes[idx]; idx += 1
            dist = (data_bytes[idx] << 8) | data_bytes[idx+1]; idx += 2
            thr = (data_bytes[idx] << 16) | (data_bytes[idx+1] << 8) | data_bytes[idx+2]; idx += 3

            imp = ((idx_h & 0x01) << 8) | imp_lo
            defect = ((idx_h & 0x02) << 7) | def_lo
            bottom = ((idx_h & 0x04) << 6) | bot_lo

            results = {
                'result_state': status & 0x03,
                'confidence': (status >> 2) & 0x03,
                'result_ready': (status >> 4) & 0x01,
                'impact_index': imp,
                'defect_index': defect,
                'bottom_index': bottom,
                'distance_mm': dist,
                'threshold': thr,
            }
    else:
        # 无结果段(纯波形帧)
        idx = expected_len

    return {
        'waveform': waveform,
        'n_points': n_points,
        'len_sel': len_sel,
        'fpga_results': results if results else None,
    }, None


def read_serial_frame(ser):
    """从串口读取一帧数据 (AA 55 同步帧格式)。"""
    # 搜索同步头
    while True:
        b = ser.read(1)
        if not b or len(b) == 0:
            return None
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                break

    # 读 len_sel 字节
    len_byte = ser.read(1)
    if not len_byte:
        return None
    len_sel = len_byte[0]
    n_points = {0: 64, 1: 128, 2: 256, 3: 512}.get(len_sel, 256)

    # 读样点数据
    data_len = 3 * n_points
    sample_data = ser.read(data_len)
    if len(sample_data) < data_len:
        return None

    # 读校验和
    chk = ser.read(1)
    if not chk:
        pass  # 继续尝试读结果段

    # 尝试读结果段 (13字节)
    result_data = ser.read(13)

    # 组装完整帧
    frame = bytes([0xAA, 0x55, len_sel]) + sample_data
    if chk:
        frame += chk
    frame += result_data

    return frame


# ============================================================================
# Core Algorithm (bit-accurate with FPGA defect_ac_classifier.v)
# ============================================================================
def fx_feature(waveform):
    """
    提取定点自相关特征: r[0..135]

    与 fpga_model.py fx_feature() 完全一致。
    返回 None 表示 INVALID (峰值太小或冲击位置异常)。
    """
    if len(waveform) < 512:
        # 对于不足512点的旧格式, 用全帧
        x = np.array(waveform, dtype=np.int64)
        n = len(x)
    else:
        x = np.array(waveform[:512], dtype=np.int64)
        n = 512

    ax = np.abs(x)
    peak = int(ax.max())

    # 有效性检查
    if peak < MIN_PEAK:
        return None, 'LOW_PEAK', peak, 0

    imp = int(np.argmax(ax))

    # 检查二次弹跳: 全帧最大峰必须在帧前段 [0, IMP_WIN)
    # 否则锤头二次弹跳/双击污染 -> INVALID
    if imp >= IMP_WIN:
        return None, 'DOUBLE_HIT', peak, imp

    # 检查段长
    if imp + SKIP + SEGLEN > n:
        return None, 'TOO_SHORT', peak, imp

    # 缩放段: s[i] = x[imp+16+i] >> sh
    seg = x[imp + SKIP : imp + SKIP + SEGLEN].astype(np.int64)
    sh = max(0, peak.bit_length() - 15)
    s = seg >> sh  # 算术右移 (numpy >> 对有符号数是算术右移)

    # 整数自相关 r[k] = sum_{i=0}^{383-k} s[i] * s[i+k], k=0..135
    r = np.zeros(NLAG, dtype=np.int64)
    for k in range(NLAG):
        r[k] = int(np.dot(s[:SEGLEN - k], s[k:]))

    if r[0] <= 0:
        return None, 'R0_ZERO', peak, imp

    return r, 'OK', peak, imp


def fx_predict(Wq, r):
    """
    LDA 定点预测 (与 fpga_model.py fx_predict() 一致)。

    参数:
        Wq: (129, 5) int16 权重矩阵 [idx, cls]
            idx=0..127: lag k=idx+8 的权重
            idx=128: r[0] 偏置
        r: (136,) int64 自相关向量 r[0..135]

    返回:
        (pred_class, margin, rs0, scores)
    """
    r0 = int(r[0])

    # 二次缩放: rsh = max(0, bits(r0) - 30)
    rsh = max(0, r0.bit_length() - 30)
    rs0 = r0 >> rsh

    # 5类打分
    scores = []
    for c in range(5):
        acc = 0
        # Lag 8..135 的权重 (idx 0..127)
        for k in range(LAG0, NLAG):
            rk = int(r[k])
            # 算术右移
            rsk = rk >> rsh if rk >= 0 else -((-rk) >> rsh)
            acc += int(Wq[k - LAG0, c]) * rsk
        # r[0] 偏置 (idx 128)
        acc += int(Wq[128, c]) * rs0
        scores.append(acc)

    order = np.argsort(scores)[::-1]
    top1, top2 = int(order[0]), int(order[1])
    margin = scores[top1] - scores[top2]

    return top1, margin, rs0, scores


def find_fundamental_period(r, lo=90, hi=110):
    """
    在自相关 r[lo..hi] 区间内搜索基波周期 (棒底反射对应的 lag)。
    返回峰值位置和峰值。
    """
    r_seg = r[lo:hi+1]
    peak_idx = int(np.argmax(r_seg))
    peak_val = r_seg[peak_idx]
    return lo + peak_idx, peak_val


def classify_pile(Wq, waveform):
    """
    完整的桩分类和缺陷定位。

    返回:
        dict with:
            prediction: 'GOOD' | 'DEFECT' | 'INVALID'
            defect_class: 0=good, 1=d200, 2=d307, 3=d695, 4=d800
            distance_mm: 缺陷距离 (mm)
            confidence: 'HIGH' | 'LOW'
            details: 调试信息
    """
    result = {
        'prediction': 'INVALID',
        'defect_class': 0,
        'distance_mm': 0,
        'confidence': 'NONE',
        'details': {}
    }

    # Step 1: 提取特征
    feat_result = fx_feature(waveform)
    if feat_result is None:
        # INVALID 未返回 dict
        return result
    r, status, peak, imp = feat_result

    if r is None:
        if status == 'LOW_PEAK':
            result['prediction'] = 'NOISE'
            result['details'] = {'reason': 'low_peak', 'peak': peak}
        else:
            result['details'] = {'reason': status, 'peak': peak, 'imp': imp}
        return result

    r0 = int(r[0])
    result['details']['peak'] = peak
    result['details']['imp'] = imp
    result['details']['r0'] = r0

    # Step 2: LDA 预测
    pred_cls, margin, rs0, scores = fx_predict(Wq, r)
    result['details']['lda_pred'] = pred_cls
    result['details']['margin'] = margin
    result['details']['rs0'] = rs0
    result['details']['scores'] = scores

    # Step 3: 800mm 二次判别
    # 若 LDA 判为 good 但基波周期 >= 103 -> 很可能是 800mm 缺陷
    if pred_cls == 0:
        T0, T0_val = find_fundamental_period(r, D800_LAG_LO, D800_LAG_HI)
        result['details']['T0'] = T0
        result['details']['T0_val'] = T0_val

        if T0 >= D800_T_TH:
            pred_cls = 4  # d800
            result['details']['d800_override'] = True
        else:
            result['details']['d800_override'] = False

    # Step 4: 确定置信度
    low_conf = (margin < MARGIN_Q * rs0)
    if pred_cls == 4 and result['details'].get('d800_override'):
        low_conf = True  # 800mm 二次判别 -> 低置信

    # Step 5: 组装结果
    result['defect_class'] = pred_cls
    result['distance_mm'] = CLS_DIST[pred_cls]
    result['confidence'] = 'LOW' if low_conf else 'HIGH'

    if pred_cls == 0:
        result['prediction'] = 'GOOD'
        result['distance_mm'] = 0
    else:
        result['prediction'] = 'DEFECT'

    return result


# ============================================================================
# Batch Testing
# ============================================================================
def test_all_data(Wq, limit_per_dir=None, verbose=False):
    """
    遍历所有数据文件, 测试分类和定位准确率。
    """
    results = []
    stats = defaultdict(lambda: {'total': 0, 'correct': 0, 'wrong': 0,
                                  'true_positive': 0, 'false_positive': 0,
                                  'true_negative': 0, 'false_negative': 0,
                                  'position_errors': []})

    data_dirs = sorted([d for d in DATA_DIR.iterdir() if d.is_dir()])

    for dirpath in data_dirs:
        dirname = dirpath.name
        try:
            grp_id = int(dirname.split('_')[0])
        except (ValueError, IndexError):
            continue

        # 确定 ground truth
        true_cls = GRP_TO_CLS.get(grp_id)
        if true_cls is None:
            continue

        is_good = (true_cls == 0)
        is_noise = (grp_id == 2)
        true_dist = CLS_DIST[true_cls]

        csv_files = sorted(dirpath.glob('hit_*.csv'))
        if limit_per_dir:
            csv_files = csv_files[:limit_per_dir]

        for csv_file in csv_files:
            try:
                waveform, hdr, _ = load_csv_hit(csv_file)

                if len(waveform) < 100:
                    continue

                result = classify_pile(Wq, waveform)

                # 统计
                pred = result['prediction']
                gt_label = 'NOISE' if is_noise else ('GOOD' if is_good else 'DEFECT')

                stats[gt_label]['total'] += 1

                if is_noise:
                    # 噪声应该被识别为 NOISE 或 INVALID
                    correct = (pred in ('NOISE', 'INVALID'))
                elif is_good:
                    # 好桩应该被识别为 GOOD
                    correct = (pred == 'GOOD')
                    if pred == 'GOOD':
                        stats['GOOD']['true_negative'] += 1
                    else:
                        stats['GOOD']['false_positive'] += 1
                else:
                    # 坏桩应该被识别为 DEFECT
                    correct = (pred == 'DEFECT')
                    if pred == 'DEFECT':
                        stats['DEFECT']['true_positive'] += 1
                        # 定位误差
                        pos_err = abs(result['distance_mm'] - true_dist)
                        stats['DEFECT']['position_errors'].append(pos_err)
                    else:
                        stats['DEFECT']['false_negative'] += 1

                if correct:
                    stats[gt_label]['correct'] += 1
                else:
                    stats[gt_label]['wrong'] += 1

                results.append({
                    'file': str(csv_file.relative_to(DATA_DIR)),
                    'group': grp_id,
                    'ground_truth': gt_label,
                    'true_cls': true_cls,
                    'true_dist': true_dist,
                    'prediction': pred,
                    'defect_class': result['defect_class'],
                    'distance_mm': result['distance_mm'],
                    'confidence': result['confidence'],
                    'correct': correct,
                    'position_error': (abs(result['distance_mm'] - true_dist)
                                      if (not is_good and pred == 'DEFECT') else None),
                })

            except Exception as e:
                if verbose:
                    print(f"  Error processing {csv_file.name}: {e}")
                continue

    return results, stats


def print_stats(results, stats):
    """打印详细统计报告。"""
    print("\n" + "=" * 80)
    print("桩完整性检测 - 测试统计报告")
    print("=" * 80)

    # 总体准确率
    total_all = sum(s['total'] for s in stats.values())
    total_correct = sum(s['correct'] for s in stats.values())

    print(f"\n总体: {total_correct}/{total_all} = {total_correct/max(total_all,1)*100:.1f}%")

    # 各类别统计
    for label in ['GOOD', 'DEFECT', 'NOISE']:
        s = stats[label]
        if s['total'] == 0:
            continue
        acc = s['correct'] / s['total'] * 100
        print(f"\n{label}:")
        print(f"  Total: {s['total']}, Correct: {s['correct']}, Wrong: {s['wrong']}")
        print(f"  Accuracy: {acc:.1f}%")

        if label == 'GOOD':
            tn = s['true_negative']  # 正确判好
            fp = s['false_positive']  # 误判为坏
            print(f"  TN (判好且真好): {tn}/{s['total']} = {tn/max(s['total'],1)*100:.1f}%")
            print(f"  FP (误判为坏): {fp}/{s['total']} = {fp/max(s['total'],1)*100:.1f}%")

        if label == 'DEFECT':
            tp = s['true_positive']
            fn = s['false_negative']
            print(f"  TP (判坏且真坏): {tp}/{s['total']} = {tp/max(s['total'],1)*100:.1f}%")
            print(f"  FN (漏判为坏): {fn}/{s['total']} = {fn/max(s['total'],1)*100:.1f}%")

            pos_errs = s['position_errors']
            if pos_errs:
                pe = np.array(pos_errs)
                print(f"  定位正确率: {(pe==0).mean()*100:.1f}%")
                print(f"  中位误差: {np.median(pe):.0f}mm")
                print(f"  平均误差: {pe.mean():.0f}mm")

    # 5类混淆矩阵
    print("\n" + "-" * 40)
    print("5类混淆矩阵 (true_cls -> pred_cls):")
    print(f"{'真/判':>10}", end="")
    for name in CLS_NAMES:
        print(f"{name:>8}", end="")
    print()

    conf = np.zeros((5, 5), dtype=int)
    for r in results:
        tc = r['true_cls']
        pc = r['defect_class']
        if 0 <= tc < 5 and 0 <= pc < 5:
            conf[tc, pc] += 1

    for i, name in enumerate(CLS_NAMES):
        print(f"{name:>10}", end="")
        for j in range(5):
            print(f"{conf[i,j]:>8}", end="")
        print()

    # 各类精确率/召回率
    print("\n" + "-" * 40)
    print(f"{'类':<10}{'总数':<8}{'精确率':<10}{'召回率':<10}{'F1':<10}")
    for i, name in enumerate(CLS_NAMES):
        tp_i = conf[i, i]
        pred_pos = conf[:, i].sum()
        actual_pos = conf[i, :].sum()
        prec = tp_i / max(pred_pos, 1)
        rec = tp_i / max(actual_pos, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-10)
        print(f"{name:<10}{actual_pos:<8}{prec:<10.3f}{rec:<10.3f}{f1:<10.3f}")


# ============================================================================
# Interactive Mode (for serial port testing)
# ============================================================================
def interactive_mode(Wq, source, baudrate=115200):
    """
    交互式模式: 从串口或文件读取数据, 实时分析并显示结果。
    """
    import serial as pyserial

    if source == 'serial':
        # 自动查找串口
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("ERROR: No serial ports found!")
            return

        print("Available serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        sel = input(f"Select port [0-{len(ports)-1}] (default 0): ").strip()
        try:
            idx = int(sel) if sel else 0
        except ValueError:
            idx = 0

        port_name = ports[idx].device
        print(f"Opening {port_name} at {baudrate} baud...")
        ser = pyserial.Serial(port_name, baudrate, timeout=5)
        print("Waiting for data... (press Ctrl+C to stop)")

        try:
            while True:
                frame = read_serial_frame(ser)
                if frame is None:
                    continue

                parsed, err = parse_uart_frame(frame)
                if parsed is None:
                    print(f"Parse error: {err}")
                    continue

                waveform = parsed['waveform']
                result = classify_pile(Wq, waveform)

                print(f"\n{'='*60}")
                print(f"Frame: {len(waveform)} points")
                print(f"Prediction: {result['prediction']}")
                print(f"Class: {CLS_NAMES[result['defect_class']]}")
                print(f"Distance: {result['distance_mm']} mm")
                print(f"Confidence: {result['confidence']}")
                print(f"Details: peak={result['details'].get('peak',0)}, "
                      f"imp={result['details'].get('imp',0)}, "
                      f"T0={result['details'].get('T0','N/A')}")

                # 如果 FPGA 已经给了结果, 对照显示
                fpga = parsed.get('fpga_results')
                if fpga:
                    fpga_state = {0: 'INVALID', 1: 'NORMAL', 2: 'DEFECT'}.get(
                        fpga['result_state'], 'UNKNOWN')
                    print(f"FPGA result: {fpga_state}, dist={fpga['distance_mm']}mm")

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            ser.close()

    elif source == 'csv':
        # 从CSV文件测试
        filepath = input("Enter CSV file path: ").strip()
        if filepath:
            waveform, hdr, _ = load_csv_hit(Path(filepath))
            result = classify_pile(Wq, waveform)
            print(json.dumps({
                'prediction': result['prediction'],
                'class': CLS_NAMES[result['defect_class']],
                'distance_mm': result['distance_mm'],
                'confidence': result['confidence'],
                'peak': result['details'].get('peak', 0),
                'imp': result['details'].get('imp', 0),
                'T0': result['details'].get('T0', 0),
                'scores': [int(s) for s in result['details'].get('scores', [])],
            }, indent=2))


# ============================================================================
def main():
    parser = argparse.ArgumentParser(description='基桩动测仪 PC端验证')
    parser.add_argument('--mode', choices=['test', 'interactive', 'single'],
                       default='test', help='运行模式')
    parser.add_argument('--limit', type=int, default=None,
                       help='每个目录最多处理的文件数')
    parser.add_argument('--csv', type=str, default=None,
                       help='单文件测试: CSV文件路径')
    parser.add_argument('--serial', action='store_true',
                       help='交互式模式: 从串口读取')
    parser.add_argument('--port', type=str, default=None,
                       help='串口设备名')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='串口波特率')
    parser.add_argument('--verbose', action='store_true',
                       help='详细输出')
    parser.add_argument('--output', type=str, default=None,
                       help='输出JSON结果文件路径')

    args = parser.parse_args()

    # 加载LDA权重
    print(f"Loading LDA weights from {WEIGHTS_FILE}...")
    if not WEIGHTS_FILE.exists():
        print(f"ERROR: Weights file not found: {WEIGHTS_FILE}")
        print("Please run: cd God3.0 && python tools/fpga_model.py")
        sys.exit(1)

    Wq = load_lda_weights(WEIGHTS_FILE)
    print(f"Loaded weights: shape={Wq.shape}, range=[{Wq.min()}, {Wq.max()}]")

    if args.mode == 'test':
        # 批量测试模式
        print(f"\nTesting all data files (limit={args.limit or 'all'})...")
        results, stats = test_all_data(Wq, limit_per_dir=args.limit,
                                       verbose=args.verbose)
        print_stats(results, stats)

        if args.output:
            # 保存结果
            output_path = Path(args.output)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump({
                    'stats': {k: {'total': v['total'], 'correct': v['correct'],
                                 'wrong': v['wrong'],
                                 'tp': v['true_positive'],
                                 'fp': v['false_positive'],
                                 'tn': v['true_negative'],
                                 'fn': v['false_negative'],
                                 'position_errors_median':
                                     float(np.median(v['position_errors']))
                                     if v['position_errors'] else None}
                              for k, v in stats.items()},
                    'total_results': len(results),
                }, f, indent=2, ensure_ascii=False)
            print(f"\nResults saved to {output_path}")

    elif args.mode == 'interactive':
        interactive_mode(Wq, 'serial' if args.serial else 'csv',
                        baudrate=args.baudrate)

    elif args.mode == 'single':
        if not args.csv:
            print("ERROR: --csv required for single mode")
            sys.exit(1)

        csv_path = Path(args.csv)
        if not csv_path.exists():
            print(f"ERROR: File not found: {csv_path}")
            sys.exit(1)

        waveform, hdr, grp_id = load_csv_hit(csv_path)
        result = classify_pile(Wq, waveform)

        print(f"\nFile: {csv_path}")
        print(f"Group: {grp_id}")
        print(f"Waveform length: {len(waveform)}")
        print(f"\nPrediction: {result['prediction']}")
        print(f"Class: {CLS_NAMES[result['defect_class']]} ({result['defect_class']})")
        print(f"Distance: {result['distance_mm']} mm")
        print(f"Confidence: {result['confidence']}")
        print(f"\nDetails:")
        for k, v in result['details'].items():
            if k == 'scores':
                print(f"  {k}: {[int(s) for s in v]}")
            else:
                print(f"  {k}: {v}")


if __name__ == '__main__':
    main()
