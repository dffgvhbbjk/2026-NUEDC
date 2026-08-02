#!/usr/bin/env python3
"""
web_app.py — 基桩动测仪 Web 界面
================================
Flask + SocketIO 后端, Plotly.js 前端, 两个 Tab:
  串口实时监听 | 采集保存

FPGA 帧格式兼容:
  - 纯 ADC 模式:  AA 55 + len_sel + 24bit×N + chksum
  - ADC + 结果段:  上述 + 0x5A + ver + status + ... + rsum
  自动识别, 有结果段则显示 FPGA 比对, 没有则只显示 PC 判别。

用法:
  python web_app.py                    # http://localhost:5000
  python web_app.py --port 8080
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # app/ -> pc_tools/

import argparse, csv, json, os, time
from datetime import datetime
import numpy as np

COEFF_MM_PER_SAMPLE = 11.46  # v=2200m/s: 2200*10.42e-6/2*1000
WIN_START = 135              # 平衡最优窗口起点 (批量扫描优化, 好棒85%+坏棒81%)
WIN_END = 390                # 平衡最优窗口终点

# Keep the raw waveform unchanged.  This debounce is applied only to the
# 5-point trend bit used by the desktop Verilog-equivalent measurement logic.
QIUDAO_GLITCH_MAX_RUN = 4

from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit

from app.pile_check import (
    load_weights, classify,
    CLS_NAMES, DATA_DIR, WEIGHTS_PATH, SAMPLE_PERIOD_US, NYLON_V_MS,
)
from capture.uart_frame_parser import save_frame, recalc_impact_index

# ============================================================================
# Flask + SocketIO
# ============================================================================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'pile_check_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

serial_state = {
    'running': False, 'capture': False, 'thread': None, 'stop_flag': False,
    'outdir': None, 'note': '', 'count': None, 'defect_mm': None, 'hit_count': 0,
    'win_start': WIN_START, 'win_end': WIN_END,  # 可调判定窗口
}

Wq = None

def get_Wq():
    global Wq
    if Wq is None:
        Wq = load_weights()
    return Wq


def _suppress_short_qiudao_runs(qiudao, max_run=QIUDAO_GLITCH_MAX_RUN):
    """Remove isolated short trend runs without filtering the waveform."""
    q = np.asarray(qiudao, dtype=np.int8).copy()
    i = 0
    n = len(q)
    while i < n:
        j = i + 1
        while j < n and q[j] == q[i]:
            j += 1
        if i > 0 and j < n and (j - i) <= max_run and q[i - 1] == q[j]:
            q[i:j] = q[i - 1]
        i = j
    return q


def _is_waveform_csv(path):
    """Recognize measurement CSVs while excluding per-directory summaries."""
    return path.is_file() and path.suffix.lower() == '.csv' and path.name.lower() != 'summary.csv'


def _waveform_files(directory):
    """Find hit CSVs, including nested capture folders when needed."""
    p = Path(directory)
    if not p.is_dir():
        return []

    direct_hits = sorted(
        f for f in p.iterdir()
        if _is_waveform_csv(f) and f.name.lower().startswith('hit_')
    )
    if direct_hits:
        return direct_hits

    nested_hits = sorted(
        f for f in p.rglob('*.csv')
        if _is_waveform_csv(f) and f.name.lower().startswith('hit_')
    )
    if nested_hits:
        return nested_hits

    return sorted(f for f in p.iterdir() if _is_waveform_csv(f))


# ============================================================================
# 帧解析 — 兼容纯 ADC 和 ADC+结果段
# ============================================================================

LEN_TABLE = {0: 64, 1: 128, 2: 256, 3: 512}

def s24(b2, b1, b0):
    v = (b2 << 16) | (b1 << 8) | b0
    return v - (1 << 24) if v & 0x800000 else v


def parse_frame(buf):
    """解析一帧。返回 (frame, consumed) 或 (None, skip)。
    frame 格式:
      {n, samples:[], fpga:{state,confidence,impact_index,defect_index,bottom_index,distance_mm} or None}
    纯 ADC 帧(无结果段): fpga=None
    ADC+结果段: fpga 包含 FPGA 判别信息
    """
    idx = buf.find(b"\xAA\x55")
    if idx < 0:
        return None, max(0, len(buf) - 1)
    if idx > 0:
        return None, idx

    if len(buf) < 3:
        return None, 0

    len_sel = buf[2]
    if len_sel not in LEN_TABLE:
        return None, 1

    n = LEN_TABLE[len_sel]
    data_len = 3 + 3 * n + 1  # header + samples + chksum
    if len(buf) < data_len:
        return None, 0

    # ── 校验和 ──
    data = buf[3:3 + 3 * n]
    chksum = buf[3 + 3 * n]
    calc = (len_sel + sum(data)) & 0xFF
    if calc != chksum:
        return None, 1

    # ── 解析样点 ──
    samples = [s24(data[i], data[i+1], data[i+2]) for i in range(0, 3*n, 3)]

    # ── 尝试解析结果段 (可选) ──
    fpga = None
    tail_start = data_len
    TAIL_LEN = 13  # 0x5A + ver + status + idx_h + imp + def + bot + dist_h + dist_l + thr3 + rsum
    if len(buf) >= tail_start + TAIL_LEN and buf[tail_start] == 0x5A:
        tail = buf[tail_start:tail_start + TAIL_LEN]
        if tail[1] == 0x02:  # ver
            rsum = sum(tail[1:12]) & 0xFF
            if rsum == tail[12]:
                status = tail[2]
                idx_h = tail[3]
                fpga = {
                    'state': status & 0x3,
                    'confidence': (status >> 2) & 0x3,
                    'impact_index': ((idx_h & 1) << 8) | tail[4],
                    'defect_index': (((idx_h >> 1) & 1) << 8) | tail[5],
                    'bottom_index': (((idx_h >> 2) & 1) << 8) | tail[6],
                    'distance_mm': ((tail[7] & 0xF) << 8) | tail[8],
                }
                data_len += TAIL_LEN  # 消耗结果段

    frame = {'n': n, 'samples': samples, 'fpga': fpga}
    return frame, data_len


# ============================================================================
# REST API
# ============================================================================

@app.route('/')
def index():
    return HTML_PAGE


@app.route('/api/ports')
def api_ports():
    """枚举可用串口 — pyserial + Windows 注册表回退, 覆盖高编号 COM 口."""
    ports = []
    seen = set()

    def add(device, description=''):
        if device and device not in seen:
            seen.add(device)
            ports.append({'device': device, 'description': description})

    # ── 方法1: pyserial (标准方式) ──
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            add(p.device, p.description or '')
    except Exception:
        pass

    # ── 方法2: Windows 注册表 (HARDWARE\DEVICEMAP\SERIALCOMM) ──
    try:
        import winreg
        for root in (winreg.HKEY_LOCAL_MACHINE,):
            try:
                key = winreg.OpenKey(root, r'HARDWARE\DEVICEMAP\SERIALCOMM')
                i = 0
                while True:
                    try:
                        name, value, _ = winreg.EnumValue(key, i)
                        add(value, name)
                        i += 1
                    except OSError:
                        break
                winreg.CloseKey(key)
            except OSError:
                pass
    except Exception:
        pass

    # ── 方法3: 直接尝试打开 COM1–COM32 (最终回退) ──
    if not ports:
        try:
            import serial
            for n in range(1, 33):
                p = f'COM{n}'
                if p in seen:
                    continue
                try:
                    s = serial.Serial(f'\\\\.\\{p}', timeout=0.01)
                    s.close()
                    add(p, '')
                except Exception:
                    pass
        except Exception:
            pass

    return jsonify({'ports': sorted(ports, key=lambda x: x['device'])})


@app.route('/api/browse_path', methods=['POST'])
def api_browse_path():
    """目录浏览 — 支持前端选择保存路径.
    请求: {path: "D:\\..."}  path为空时返回驱动器列表
    返回: {parent, current, drives, subdirs: [{name, path}]}
    """
    data = request.get_json(silent=True) or {}
    p = data.get('path', '').strip()

    result = {'parent': None, 'current': '', 'drives': [], 'subdirs': []}

    if not p:
        # 返回 Windows 驱动器列表
        import string
        drives = []
        for letter in string.ascii_uppercase:
            root = f'{letter}:\\'
            if os.path.exists(root):
                drives.append({'name': f'本地磁盘 ({letter}:)', 'path': root})
        result['drives'] = drives
        return jsonify(result)

    # 规范化路径
    p = os.path.normpath(p)
    if not os.path.exists(p):
        return jsonify({'error': f'路径不存在: {p}'}), 404
    if not os.path.isdir(p):
        p = os.path.dirname(p)

    result['current'] = p
    result['parent'] = os.path.dirname(p) if os.path.dirname(p) != p else None

    try:
        for entry in sorted(os.scandir(p), key=lambda e: (not e.is_dir(), e.name.lower())):
            if entry.is_dir() and not entry.name.startswith('.'):
                result['subdirs'].append({
                    'name': entry.name,
                    'path': os.path.join(p, entry.name),
                })
    except PermissionError:
        pass

    return jsonify(result)


@app.route('/api/default_save_dir')
def api_default_save_dir():
    """返回默认保存路径"""
    return jsonify({'path': str(DATA_DIR)})


# ============================================================================
# 文件导入: Verilog 算法分析
# ============================================================================

def analyze_samples(samples, defect_mm=None, win_start=None, win_end=None):
    """对原始样点数组运行 Verilog 算法分析 (通用, 可用于文件导入和串口实时).
    返回前端展示所需的所有数据字典, 不含系数相关计算结果 (由前端根据系数动态算).

    win_start, win_end: 可选的判定窗口参数, 未提供时使用全局默认 WIN_START/WIN_END
    """
    _ws = win_start if win_start is not None else WIN_START
    _we = win_end if win_end is not None else WIN_END
    n = len(samples)
    if n < 10:
        return {'error': f'数据点太少: {n}'}

    arr = np.array(samples, dtype=np.float64)

    # ── 1. 归一化: data/128 + 200 ──
    norm = arr / 128.0 + 200.0

    # ── 2. 5点连续递减检测 ──
    qiudao = np.zeros(n, dtype=int)
    for i in range(4, n):
        r1, r2, r3, r4, r5 = norm[i], norm[i-1], norm[i-2], norm[i-3], norm[i-4]
        qiudao[i] = 1 if (r1 > r2 and r2 > r3 and r3 > r4 and r4 > r5) else 0

    # Reject only short isolated trend pulses caused by small ADC spikes.
    # The original samples remain in `arr` for plotting and peak extraction.
    qiudao = _suppress_short_qiudao_runs(qiudao)

    # ── 3. 方向翻转位置 ──
    edge_positions = []
    for i in range(1, n):
        if qiudao[i] != qiudao[i-1]:
            edge_positions.append(i)

    edge_num = {}
    cnt = 0
    for ep in edge_positions:
        cnt += 1
        edge_num[ep] = cnt

    # ── 4. 找第4/5/6/7次翻转 ──
    e4 = e5 = e6 = e7 = None
    for ep in edge_positions:
        if edge_num[ep] == 4: e4 = ep
        if edge_num[ep] == 5: e5 = ep
        if edge_num[ep] == 6: e6 = ep
        if edge_num[ep] == 7: e7 = ep

    # ── 5. 在第4~6翻转区域内找关键波峰/波谷 ──
    marked_points = []
    falling_width = 0
    falling_start = None
    falling_end = None
    region_start = region_end = None

    if e4 is not None and e6 is not None:
        region_start, region_end = e4, e6

        # 找 e4 前后最近的 qiudao 下降段 (用于波峰/波谷定位)
        qiudao_1_start = None
        qiudao_1_end = None
        for i in range(max(0, e4 - 10), min(e6 + 40, n)):
            if qiudao[i] == 1:
                if qiudao_1_start is None:
                    qiudao_1_start = i
            elif qiudao_1_start is not None:
                qiudao_1_end = i - 1
                break
        else:
            qiudao_1_end = min(e6 + 40, n) - 1 if qiudao_1_start is not None else None

        if qiudao_1_start is not None:
            # 找下降段开始处紧邻的局部波峰 (往回找, 最多10点)
            peak_idx = None
            for i in range(qiudao_1_start, max(0, qiudao_1_start - 10), -1):
                if i > 0 and i < n - 1 and arr[i] > arr[i-1] and arr[i] > arr[i+1]:
                    peak_idx = i
                    break
            if peak_idx:
                marked_points.append({'type': 'peak', 'index': peak_idx, 'value': float(arr[peak_idx])})

            # 找下降段结束处紧邻的局部波谷 (往前找, 最多10点)
            if qiudao_1_end:
                valley_idx = None
                for i in range(qiudao_1_end, min(n - 1, qiudao_1_end + 10)):
                    if i > 0 and i < n - 1 and arr[i] < arr[i-1] and arr[i] < arr[i+1]:
                        valley_idx = i
                        break
                if valley_idx:
                    marked_points.append({'type': 'valley', 'index': valley_idx, 'value': float(arr[valley_idx])})

        # 回退: 离 e4 最近的峰/谷
        if not marked_points:
            local_extrema = []
            for i in range(max(0, e4 - 20), min(n - 1, e6 + 20)):
                if i > 0 and i < n - 1 and arr[i] > arr[i-1] and arr[i] > arr[i+1]:
                    local_extrema.append({'type': 'peak', 'index': i, 'value': float(arr[i])})
                elif i > 0 and i < n - 1 and arr[i] < arr[i-1] and arr[i] < arr[i+1]:
                    local_extrema.append({'type': 'valley', 'index': i, 'value': float(arr[i])})
            peaks = [p for p in local_extrema if p['type'] == 'peak']
            valleys = [v for v in local_extrema if v['type'] == 'valley']
            if peaks:
                marked_points.append(min(peaks, key=lambda p: abs(p['index'] - e4)))
            if valleys:
                marked_points.append(min(valleys, key=lambda v: abs(v['index'] - e4)))

    # ── 5b. Verilog 对齐: falling_width = e4→e7 窗口内全部 qiudao=1 样点 ──
    # Verilog: ok_length_cnt 在 global_edge_cnt ∈ [4,5,6] 时累加 qiudao=1,
    #   即从第4次翻转检测后开始, 到第7次翻转(退出区域)前结束 → 样点 [e4, e7)
    # PC 等效: sum(qiudao[e4:e7]), 与 FPGA 逐周期计数一致
    # 若不足7次翻转(信号弱), 回退用 e6 或数据末尾
    if e4 is not None:
        falling_start = e4
        if e7 is not None:
            falling_end = e7
            falling_width = int(qiudao[e4:e7].sum())
        elif e6 is not None:
            falling_end = e6
            falling_width = int(qiudao[e4:e6 + 1].sum())
        else:
            falling_end = min(e4 + 50, n - 1)
            falling_width = int(qiudao[e4:falling_end + 1].sum())

    # ── 6. 找敲击峰 ──
    abs_arr = np.abs(arr)
    impact_idx = int(np.argmax(abs_arr[:min(100, n)]))

    # ── 6b. 冲击峰后的 qiudao 边沿位置 (用于 B2 测距法) ──
    # B2: reflection_pos = e2_after - impact_idx
    # 距离 = reflection_pos × 14.67 (批量扫描最优系数, v~2200m/s fs~96kHz)
    edges_after_impact = [ep for ep in edge_positions if ep > impact_idx]
    e1_after = edges_after_impact[0] if len(edges_after_impact) > 0 else None
    e2_after = edges_after_impact[1] if len(edges_after_impact) > 1 else None
    e3_after = edges_after_impact[2] if len(edges_after_impact) > 2 else None
    reflection_pos = (e2_after - impact_idx) if e2_after is not None else 0

    # ── 7. 所有翻转位置 (用于前端画竖线) ──
    edge_list = [{'index': ep, 'num': edge_num[ep]} for ep in edge_positions]

    # ── 8. 分析窗口内翻转次数 (对应 Verilog qiudao_panjue_edge_cnt) ──
    win_edges = sum(1 for ep in edge_positions if _ws <= ep <= min(_we, n))
    analysis_win = {'start': _ws, 'end': min(_we, n)}

    return {
        'n': n,
        'defect_mm': defect_mm,
        'samples': [float(x) for x in arr],
        'impact_idx': impact_idx,
        'edges': edge_list,
        'e4': e4, 'e5': e5, 'e6': e6, 'e7': e7,
        'region_start': region_start, 'region_end': region_end,
        'marked_points': marked_points,
        'falling_width': falling_width,
        'falling_start': falling_start,  # Verilog: e4 (计数窗口起点)
        'falling_end': falling_end,      # Verilog: e7 (计数窗口终点, 退出区域前)
        'reflection_pos': reflection_pos,  # B2 测距: e2_after - impact_idx (样点数)
        'e1_after': e1_after,             # 冲击后第1边沿
        'e2_after': e2_after,             # 冲击后第2边沿
        'e3_after': e3_after,             # 冲击后第3边沿
        'analysis_win': analysis_win,
        'total_edges': len(edge_positions),
        'win_edges': win_edges,  # 分析窗口[WIN_START, WIN_END]内的翻转次数
    }


def analyze_waveform_file(filepath, win_start=None, win_end=None):
    """读取 CSV 文件 → analyze_samples()"""
    samples = []
    defect_mm = None
    note = ''
    try:
        with open(filepath, encoding='utf-8-sig') as f:
            for row in csv.reader(f):
                if not row: continue
                if row[0].startswith('# note'):
                    note = row[1] if len(row) > 1 else ''
                elif row[0].startswith('# defect_true_mm'):
                    v = row[1].strip()
                    defect_mm = int(v) if v else None
                elif row[0].isdigit():
                    try:
                        samples.append(int(row[1]))
                    except (ValueError, TypeError):
                        # Ignore malformed summary/metadata rows in imported CSVs.
                        continue
    except Exception as e:
        return {'error': str(e)}

    result = analyze_samples(samples, defect_mm, win_start, win_end)
    if 'error' in result:
        return result
    result['note'] = note
    return result


@app.route('/api/data_dirs')
def api_data_dirs():
    """返回指定路径下的子目录列表 (默认 God3.0/data).
    ?path=D:\\... 可指定自定义路径.
    """
    base = request.args.get('path', '')
    base = Path(base) if base else DATA_DIR
    dirs = []
    try:
        if base.exists() and base.is_dir():
            for d in sorted(base.iterdir()):
                if d.is_dir() and not d.name.startswith('.'):
                    n_files = len(_waveform_files(d))
                    dirs.append({'name': d.name, 'path': str(d), 'n_files': n_files})
    except Exception as e:
        return jsonify({'dirs': [], 'error': str(e), 'current_path': str(base)})
    return jsonify({'dirs': dirs, 'current_path': str(base)})


@app.route('/api/data_files')
def api_data_files():
    """返回指定目录下的 CSV 文件列表"""
    dir_path = request.args.get('dir', '')
    files = []
    try:
        p = Path(dir_path)
        if p.exists() and p.is_dir():
            for f in _waveform_files(p):
                files.append({'name': f.name, 'path': str(f)})
    except Exception as e:
        return jsonify({'files': [], 'error': str(e)})
    return jsonify({'files': files})


@app.route('/api/batch_analyze', methods=['POST'])
def api_batch_analyze():
    """批量分析: 对选中的多个目录下所有 CSV 运行 Verilog 算法并统计分类准确率.

    请求: {dirs: [path, ...], threshold: 11}
    真值从目录名自动提取: "好棒"→GOOD, "坏棒"/"坏帮"→DEFECT
    """
    data = request.get_json(silent=True) or {}
    dirs = data.get('dirs', [])
    threshold = data.get('threshold', 11)

    if not dirs:
        return jsonify({'error': '请选择至少一个目录'}), 400

    all_per_file = []       # 每个文件的详细结果
    dir_results = []        # 每个目录的汇总
    win_edges_good = []     # 好棒 win_edges 分布
    win_edges_defect = []   # 坏棒 win_edges 分布
    distance_data = []      # 缺陷距离分析数据 (坏棒+有真值)
    total_tp = total_tn = total_fp = total_fn = 0
    total_files = 0
    errors = []

    for dpath in dirs:
        p = Path(dpath)
        if not p.is_dir():
            continue
        dname = p.name
        # ── 从目录名提取真值标签 ──
        if '好棒' in dname:
            truth = 'GOOD'
        elif '坏棒' in dname or '坏帮' in dname:
            truth = 'DEFECT'
        else:
            truth = 'UNKNOWN'

        csv_files = _waveform_files(p)
        d_tp = d_tn = d_fp = d_fn = 0
        d_errors = []  # 分类错误的文件

        for f in csv_files:
            total_files += 1
            r = analyze_waveform_file(str(f))
            if 'error' in r:
                d_errors.append({'file': f.name, 'error': r['error']})
                continue

            win_e = r.get('win_edges', 0)
            pred = 'DEFECT' if win_e >= threshold else 'GOOD'
            correct = (pred == truth)

            # ── 距离测量数据 (仅坏棒+有真值) ──
            fw = r.get('falling_width', 0)
            true_mm = r.get('defect_mm')

            if truth == 'DEFECT':
                win_edges_defect.append(win_e)
                if correct:
                    d_tp += 1
                    total_tp += 1
                else:
                    d_fn += 1
                    total_fn += 1
                # 收集距离数据 (需要已知真值, B2反射位置)
                refl = r.get('reflection_pos', 0)
                if true_mm is not None and (fw > 0 or refl > 0):
                    distance_data.append({
                        'file': f.name, 'dir': dname,
                        'defect_true_mm': true_mm,
                        'falling_width': fw,
                        'reflection_pos': refl,
                    })
            else:
                win_edges_good.append(win_e)
                if correct:
                    d_tn += 1
                    total_tn += 1
                else:
                    d_fp += 1
                    total_fp += 1

            # 只保留前 6000 条详细记录避免响应过大
            if len(all_per_file) < 6000:
                refl = r.get('reflection_pos', 0)
                all_per_file.append({
                    'file': f.name, 'dir': dname, 'truth': truth,
                    'pred': pred, 'win_edges': win_e, 'correct': correct,
                    'defect_true_mm': true_mm,
                    'falling_width': fw,
                    'reflection_pos': refl,
                })

        n = d_tp + d_tn + d_fp + d_fn
        acc = round((d_tp + d_tn) / n, 4) if n > 0 else 0
        dir_results.append({
            'name': dname, 'path': str(p), 'truth': truth,
            'n_files': n,
            'n_good_pred': d_tn + d_fn,    # 预测为 GOOD 的数量
            'n_defect_pred': d_tp + d_fp,   # 预测为 DEFECT 的数量
            'TP': d_tp, 'TN': d_tn, 'FP': d_fp, 'FN': d_fn,
            'accuracy': acc,
            'errors': d_errors,
        })

    # ── 汇总 ──
    total = total_tp + total_tn + total_fp + total_fn
    def safe_div(a, b): return round(a / b, 4) if b > 0 else 0
    summary = {
        'TP': total_tp, 'TN': total_tn, 'FP': total_fp, 'FN': total_fn,
        'accuracy': safe_div(total_tp + total_tn, total),
        'precision': safe_div(total_tp, total_tp + total_fp),
        'recall': safe_div(total_tp, total_tp + total_fn),
        'f1': safe_div(2 * total_tp, 2 * total_tp + total_fp + total_fn),
    }
    summary['total'] = total

    return jsonify({
        'threshold': threshold,
        'total_files': total_files,
        'total_good': len(win_edges_good),
        'total_defect': len(win_edges_defect),
        'summary': summary,
        'directories': dir_results,
        'per_file': all_per_file,
        'win_edges_good': win_edges_good,
        'win_edges_defect': win_edges_defect,
        'distance_data': distance_data,  # 缺陷距离分析
        'errors': errors,
    })


# ============================================================================
# SocketIO 事件
# ============================================================================

@socketio.on('connect')
def on_connect():
    emit('status', {'msg': '已连接'})


@socketio.on('start_serial')
def on_start_serial(data):
    if serial_state['running']:
        emit('error', {'msg': '串口已在运行中'})
        return
    serial_state.update(capture=False, stop_flag=False, hit_count=0)
    serial_state['thread'] = socketio.start_background_task(
        serial_worker, data.get('port', 'COM3'), data.get('baud', 115200))
    emit('status', {'msg': f'监听已启动: {data.get("port", "COM3")}'})


@socketio.on('start_capture')
def on_start_capture(data):
    if serial_state['running']:
        # 旧采集/监听线程仍挂着 (例如上一轮长采集未干净退出, 或页面重载后线程残留)。
        # 直接拒绝会让前端按钮卡在"采集中"却无采集在跑, 用户无法再启动 → 先安全停掉再重启。
        emit('status', {'msg': '检测到旧采集仍在运行, 正在停止...'})
        serial_state['stop_flag'] = True
        deadline = time.monotonic() + 3.0
        while serial_state['running'] and time.monotonic() < deadline:
            time.sleep(0.05)
        if serial_state['running']:
            emit('error', {'msg': '旧采集停止超时, 请稍后再试 (或重启服务)'})
            return
        # 等旧线程 finally 里的状态事件发完, 避免与新的 capture_status 乱序
        time.sleep(0.2)

    note = data.get('note', '')
    count = data.get('count', None)
    defect_mm = data.get('defect_mm', None)
    outdir_base = data.get('outdir_base', None)

    def _sanitize(text, maxlen=40):
        bad = '\\/:*?"<>| '
        return "".join(("_" if c in bad else c) for c in text.strip())[:maxlen] or "unnamed"

    # 使用用户选择的保存路径, 否则 fallback 到默认 DATA_DIR
    base = Path(outdir_base) if outdir_base else DATA_DIR
    base.mkdir(parents=True, exist_ok=True)
    # 找最大已有编号 +1 (避免目录删除/不连续编号导致的冲突)
    max_seq = 0
    for d in base.iterdir():
        if d.is_dir():
            try:
                n = int(d.name.split('_', 1)[0])
                if n > max_seq:
                    max_seq = n
            except ValueError:
                pass
    seq = max_seq + 1
    sess = f"{seq:02d}_{datetime.now().strftime('%H%M%S')}_{_sanitize(note)}"
    outdir = base / sess
    outdir.mkdir(parents=True)

    serial_state.update(capture=True, stop_flag=False, hit_count=0,
                        outdir=str(outdir), outdir_base=str(base),
                        note=note, count=count, defect_mm=defect_mm)
    serial_state['thread'] = socketio.start_background_task(
        serial_worker, data.get('port', 'COM3'), data.get('baud', 115200))
    emit('status', {'msg': f'采集已启动 → {outdir}'})
    emit('capture_status', {
        'hit': 0, 'count': count, 'remaining': count,
        'outdir': str(outdir), 'note': note, 'defect_mm': defect_mm,
        'running': True,
    })


@socketio.on('stop_serial')
def on_stop_serial():
    serial_state['stop_flag'] = True


@socketio.on('analyze_file')
def on_analyze_file(data):
    """加载并分析 CSV 文件 (Verilog 算法 + 自动标峰)"""
    filepath = data.get('path', '')
    win_start = data.get('win_start', None)
    win_end = data.get('win_end', None)
    if not filepath or not os.path.isfile(filepath):
        emit('analysis_result', {'error': f'文件不存在: {filepath}'})
        return
    result = analyze_waveform_file(filepath, win_start, win_end)
    if result is None:
        emit('analysis_result', {'error': '分析失败'})
    else:
        emit('analysis_result', result)


@socketio.on('set_analysis_window')
def on_set_analysis_window(data):
    """前端调节判定窗口 [win_start, win_end]"""
    ws = data.get('win_start')
    we = data.get('win_end')
    if ws is not None:
        serial_state['win_start'] = int(ws)
    if we is not None:
        serial_state['win_end'] = int(we)


# ============================================================================
# 串口后台线程
# ============================================================================

def serial_worker(port, baud):
    try:
        import serial
    except ImportError:
        socketio.emit('error', {'msg': '需要 pyserial: pip install pyserial'})
        serial_state['running'] = False
        return

    def _open(p, b):
        try:
            return serial.Serial(p, b, timeout=0.05)
        except Exception:
            if p.upper().startswith("COM"):
                try:
                    return serial.Serial(f"\\\\.\\{p}", b, timeout=0.05)
                except Exception:
                    pass
            raise

    try:
        ser = _open(port, baud)
    except Exception as e:
        socketio.emit('error', {'msg': f'无法打开串口 {port}: {e}'})
        serial_state['running'] = False
        return

    serial_state['running'] = True
    W = get_Wq()
    capture = serial_state['capture']
    truth = {'rod_len': None, 'defect_mm': serial_state['defect_mm']}

    buf = bytearray()
    hit = 0
    dropped = 0
    last_rx = time.monotonic()
    IDLE_TIMEOUT = 0.5

    STATE_MAP = {0: "INVALID", 1: "NORMAL", 2: "DEFECT"}
    CONF_MAP = {0: "NONE", 1: "LOW", 2: "HIGH"}

    try:
        while not serial_state['stop_flag']:
            chunk = ser.read(4096)
            now = time.monotonic()
            if chunk:
                buf += chunk
                last_rx = now
            elif buf and (now - last_rx) > IDLE_TIMEOUT:
                dropped += len(buf)
                buf.clear()
                continue

            while buf:
                frame, consumed = parse_frame(bytes(buf))
                if frame is not None:
                    hit += 1
                    serial_state['hit_count'] = hit

                    # ── PC 判别 ──
                    wf = np.array(frame["samples"], dtype=np.int64)
                    rv = classify(W, wf)
                    pc_info = {
                        'prediction': rv['prediction'],
                        'class_name': rv['class_name'],
                        'class_id': rv['class_id'],
                        'distance_mm': rv['distance_mm'],
                        'confidence': rv['confidence'],
                        'valid': rv['valid'],
                    }
                    if rv['valid']:
                        pc_info['peak'] = rv['peak']
                        pc_info['imp'] = rv['imp']
                        pc_info['scores'] = [int(s) for s in rv.get('scores', [])]
                        pc_info['margin'] = rv.get('margin', 0)
                        if rv.get('T0'):
                            pc_info['T0'] = rv['T0']

                    # ── FPGA 信息 (可能为 None) ──
                    fpga_info = None
                    match = None
                    if frame['fpga']:
                        f = frame['fpga']
                        fpga_state = STATE_MAP.get(f['state'], '?')
                        fpga_conf = CONF_MAP.get(f['confidence'], '?')
                        fpga_info = {
                            'state': fpga_state,
                            'confidence': fpga_conf,
                            'impact_index': f['impact_index'],
                            # PC 重算冲击峰 (FPGA 导出值在部分固件下恒为 0, 原始样点才可靠)
                            'impact_recalc': recalc_impact_index(frame['samples']),
                            'defect_index': f['defect_index'],
                            'bottom_index': f['bottom_index'],
                            'distance_mm': f['distance_mm'],
                        }
                        # 一致性
                        match = '✓'
                        if not ((rv['prediction'] == 'GOOD' and fpga_state == 'NORMAL') or
                                (rv['prediction'] == 'DEFECT' and fpga_state == 'DEFECT') or
                                (rv['prediction'] == 'INVALID' and fpga_state == 'INVALID')):
                            match = '✗'

                    # ── 保存 CSV (仅采集模式) ──
                    if capture and serial_state['outdir']:
                        # 构造兼容 save_frame 的 frame dict
                        save_dict = {
                            'n': frame['n'], 'samples': frame['samples'],
                            'result_ready': 1 if frame['fpga'] else 0,
                            'state': frame['fpga']['state'] if frame['fpga'] else 0,
                            'confidence': frame['fpga']['confidence'] if frame['fpga'] else 0,
                            'impact_index': frame['fpga']['impact_index'] if frame['fpga'] else 0,
                            'defect_index': frame['fpga']['defect_index'] if frame['fpga'] else 0,
                            'bottom_index': frame['fpga']['bottom_index'] if frame['fpga'] else 0,
                            'distance_mm': frame['fpga']['distance_mm'] if frame['fpga'] else 0,
                            'threshold': 0,
                        }
                        save_frame(save_dict, hit, serial_state['outdir'],
                                   serial_state['note'], truth)

                    # ── Verilog 算法分析 (PC端) ──
                    vlg = analyze_samples(frame['samples'],
                                          win_start=serial_state.get('win_start'),
                                          win_end=serial_state.get('win_end'))

                    # ── 推送到浏览器 ──
                    socketio.emit('frame', {
                        'hit': hit, 'n_pts': frame['n'],
                        'waveform': frame['samples'],
                        'fpga': fpga_info,
                        'pc': pc_info,
                        'match': match,
                        'capture': capture,
                        'timestamp': datetime.now().strftime('%H:%M:%S'),
                        'vlg': vlg,  # Verilog 算法分析结果
                    })

                    # ── 采集模式: 推送状态更新 (还差N次等) ──
                    if capture:
                        cnt = serial_state['count']
                        socketio.emit('capture_status', {
                            'hit': hit,
                            'count': cnt,
                            'remaining': (cnt - hit) if cnt else None,
                            'outdir': serial_state['outdir'],
                            'note': serial_state['note'],
                            'defect_mm': serial_state['defect_mm'],
                            'running': True,
                            'last_state': fpga_info['state'] if fpga_info else '?',
                            'last_dist': fpga_info['distance_mm'] if fpga_info else None,
                            'last_conf': fpga_info['confidence'] if fpga_info else '?',
                        })

                    del buf[:consumed]

                    if capture and serial_state['count'] and hit >= serial_state['count']:
                        serial_state['stop_flag'] = True
                        socketio.emit('status', {'msg': f'采集完成: {hit} 帧'})
                        break

                elif consumed > 0:
                    dropped += consumed
                    del buf[:consumed]
                else:
                    break

    except Exception as e:
        socketio.emit('error', {'msg': f'串口线程异常: {e}'})
    finally:
        ser.close()
        serial_state['running'] = False
        # 清理采集状态: 避免上一轮留下的 capture/stop_flag 残留,
        # 否则下次 on_start_capture 会误判"旧采集仍在运行"或漏停
        if capture:
            serial_state['capture'] = False
            serial_state['stop_flag'] = False
        outdir_msg = f'  数据目录: {serial_state["outdir"]}' if capture and serial_state['outdir'] else ''
        socketio.emit('status', {
            'msg': f'已断开  |  {hit} 帧, 丢弃 {dropped} 字节{outdir_msg}',
            'serial_running': False,
        })
        if capture:
            socketio.emit('capture_status', {
                'hit': hit, 'count': serial_state['count'],
                'remaining': 0, 'outdir': serial_state['outdir'],
                'note': serial_state['note'], 'defect_mm': serial_state['defect_mm'],
                'running': False,
            })


# ============================================================================
# HTML 页面
# ============================================================================

HTML_PAGE = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>基桩动测仪</title>
<script src="https://cdn.plot.ly/plotly-3.0.0.min.js"></script>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Microsoft YaHei','SimHei',sans-serif;background:#1a1a2e;color:#eee;min-height:100vh}
header{background:#16213e;padding:10px 24px;display:flex;align-items:center;justify-content:space-between}
header h1{font-size:18px;color:#e94560}
.tabs{display:flex;background:#0f3460;padding:0 24px}
.tab{padding:10px 20px;cursor:pointer;border-bottom:3px solid transparent;transition:.2s}
.tab:hover{background:#16213e}
.tab.active{border-bottom-color:#e94560;color:#e94560}
.content{padding:16px;max-width:1200px;margin:0 auto}
.panel{display:none}
.panel.active{display:block}
.card{background:#16213e;border-radius:8px;padding:14px;margin-bottom:12px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
input,select,button{padding:7px 12px;border-radius:5px;border:1px solid #444;background:#0f3460;color:#eee;font-size:13px}
button{cursor:pointer;background:#e94560;border:none;font-weight:bold}
button:hover{opacity:.85}
button.secondary{background:#0f3460;border:1px solid #555}
button:disabled{opacity:.4}
label{font-size:12px;color:#aaa}
.plot-container{width:100%;height:400px}
.flex-2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.result-badge{display:inline-block;padding:2px 8px;border-radius:3px;font-weight:bold;font-size:12px}
.badge-good{background:#1b5e20;color:#81c784}
.badge-defect{background:#b71c1c;color:#ef9a9a}
.badge-invalid{background:#555;color:#ccc}
.match-ok{color:#4caf50}
.match-fail{color:#f44336}
.measure-bar{display:flex;align-items:center;gap:10px;margin:8px 0}
#toast{position:fixed;bottom:16px;right:16px;padding:10px 16px;border-radius:6px;display:none;z-index:999;font-size:13px}
.toast-info{background:#0f3460;border:1px solid #448aff}
.toast-error{background:#3e1010;border:1px solid #e94560}
.fpga-na{color:#666;font-style:italic}
.dir-check-row{display:flex;align-items:center;padding:3px 8px;border-bottom:1px solid #222;font-size:12px}
.dir-check-row:hover{background:#0f3460}
.dir-check-row input[type=checkbox]{margin-right:8px;accent-color:#e94560}
.tag-good{background:#1b5e20;color:#81c784;padding:1px 6px;border-radius:3px;font-size:10px;font-weight:bold}
.tag-defect{background:#b71c1c;color:#ef9a9a;padding:1px 6px;border-radius:3px;font-size:10px;font-weight:bold}
.mx-cell{text-align:center;padding:8px 14px;font-size:20px;font-weight:bold}
.mx-diag{color:#4caf50}
.mx-off{color:#f44336}
.mx-label{font-size:11px;color:#888;display:block;text-align:center;margin-top:2px}
.stat-num{font-size:22px;font-weight:bold;margin:2px 0}
.stat-good{color:#4caf50}
.stat-warn{color:#ffab40}
.stat-info{color:#448aff}
/* 采集保存 — 波形+侧边栏 */
.capture-layout{display:flex;gap:12px}
.capture-plot{flex:1;min-width:0}
.capture-sidebar{width:280px;flex-shrink:0;background:#0f1a2e;border-radius:8px;padding:14px;font-size:12px;display:flex;flex-direction:column;gap:12px}
.cap-section-title{color:#888;font-size:11px;text-transform:uppercase;letter-spacing:1px;border-bottom:1px solid #222;padding-bottom:4px;margin-bottom:2px}
.cap-info-row{display:flex;justify-content:space-between;align-items:center;padding:2px 0}
.cap-info-label{color:#888;font-size:11px}
.cap-info-value{color:#eee;font-weight:bold;font-size:12px;text-align:right;max-width:160px;overflow:hidden;text-overflow:ellipsis}
.cap-countdown{font-size:24px;font-weight:bold;text-align:center;padding:8px;border-radius:6px;margin:4px 0}
.cap-countdown.active{color:#ffab40;background:#1a1200}
.cap-countdown.done{color:#4caf50;background:#0a1a0a}
.cap-progress-bar{height:6px;background:#222;border-radius:3px;overflow:hidden}
.cap-progress-fill{height:100%;background:#ffab40;transition:width .3s}
.cap-last-hit{font-size:11px;color:#aaa;line-height:1.6}
.cap-last-hit .state-DEFECT{color:#f44336}
.cap-last-hit .state-NORMAL{color:#4caf50}
.cap-last-hit .state-INVALID{color:#888}
/* 目录浏览弹窗 */
.browse-overlay{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,.7);z-index:1000;display:flex;align-items:center;justify-content:center}
.browse-dialog{background:#1a2a3e;border-radius:10px;width:560px;max-height:70vh;display:flex;flex-direction:column;box-shadow:0 8px 32px rgba(0,0,0,.6)}
.browse-header{padding:14px 18px;border-bottom:1px solid #333;display:flex;justify-content:space-between;align-items:center}
.browse-header h3{margin:0;font-size:16px}
.browse-path-bar{display:flex;align-items:center;gap:6px;padding:10px 18px;background:#0f1a2e;font-size:12px;border-bottom:1px solid #222}
.browse-path-bar span{color:#448aff;cursor:pointer}
.browse-path-bar span:hover{text-decoration:underline}
.browse-list{flex:1;overflow-y:auto;padding:8px}
.browse-item{display:flex;align-items:center;gap:8px;padding:8px 12px;cursor:pointer;border-radius:5px;font-size:13px;transition:background .15s}
.browse-item:hover{background:#0f3460}
.browse-item.drive{border-bottom:1px solid #222}
.browse-item .icon{font-size:18px}
.browse-footer{padding:12px 18px;border-top:1px solid #333;display:flex;justify-content:space-between}
/* 自动编号预览 */
.folder-preview{font-size:11px;color:#ffab40;font-family:Consolas,monospace;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
</style>
</head>
<body>
<header>
  <h1>基桩动测仪</h1>
  <span id="connStatus" style="font-size:12px;color:#888">连接中...</span>
</header>

<div class="tabs">
  <div class="tab active" onclick="switchTab('serial')">📡 串口监听</div>
  <div class="tab" onclick="switchTab('capture')">💾 采集保存</div>
  <div class="tab" onclick="switchTab('import')">📂 文件导入</div>
  <div class="tab" onclick="switchTab('error')">📊 误差分析</div>
</div>

<div class="content">

<!-- ====== Tab: 串口监听 ====== -->
<div id="panel-serial" class="panel active">
  <div class="card">
    <div class="row">
      <select id="serPort" style="width:180px"></select>
      <label>波特率: <input id="serBaud" type="number" value="115200" style="width:90px"></label>
      <button id="btnSerConn" onclick="toggleSerial()">▶ 连接</button>
      <span id="serStatus" style="color:#aaa;font-size:12px">未连接</span>
      <span id="serHitCount" style="font-weight:bold;font-size:13px"></span>
    </div>
    <div class="measure-bar">
      <label>波速: <input id="serVelocity" type="number" value="2200" style="width:70px"> m/s</label>
      <label>测距方法: <select id="serMethod" onchange="onMethodChange('ser')"><option value="b2">B2反射位</option><option value="falling">下降段宽度</option></select></label>
      <label>系数: <input id="serCoeff" type="number" value="14.67" step="0.01" style="width:70px" oninput="recalcSerDist()"> <span id="serCoeffUnit">mm/样点(B2)</span></label>
      <label>判定阈值: <input id="serThresh" type="number" value="8" min="1" style="width:60px" oninput="recalcSerDist()"> 次翻转</label>
      <label>分析窗口: <input id="serWinStart" type="number" value="135" min="0" max="500" style="width:64px" oninput="onSerWindowChange()"> ~ <input id="serWinEnd" type="number" value="390" min="0" max="512" style="width:64px" oninput="onSerWindowChange()"><button class="secondary" onclick="resetWindow('ser')" title="恢复窗口默认" style="font-size:11px;padding:2px 5px;margin-left:4px">↺</button></label>
      <button class="secondary" onclick="startMeasure('ser')">📏 测距</button>
      <button class="secondary" onclick="resetMeasure('ser')">🔄 复位</button>
      <span id="serMeasureInfo" style="color:#ffab40;font-size:12px"></span>
    </div>
  </div>
  <div class="card"><div id="serPlot" class="plot-container"></div></div>
  <div id="serInfo"></div>
</div>

<!-- ====== Tab: 采集保存 ====== -->
<div id="panel-capture" class="panel">
  <div class="card">
    <!-- 第1行: 串口 + 按钮 -->
    <div class="row">
      <select id="capPort" style="width:180px"></select>
      <label>波特率: <input id="capBaud" type="number" value="115200" style="width:90px"></label>
      <button id="btnCapConn" onclick="toggleCapture()">▶ 开始采集</button>
      <button class="secondary" id="btnCapStop" onclick="stopCapture()" style="display:none">⏹ 停止</button>
      <span id="capStatus" style="color:#aaa;font-size:12px">未开始</span>
    </div>
    <!-- 第2行: 保存路径 + 自动编号预览 -->
    <div class="row" style="margin-top:8px">
      <label style="white-space:nowrap">保存路径:</label>
      <input id="capSavePath" style="flex:1;min-width:300px;font-size:11px;font-family:Consolas,monospace" oninput="updateCapFolderPreview()">
      <button class="secondary" onclick="browseSavePath()" style="white-space:nowrap">📁 浏览...</button>
      <span class="folder-preview" id="capFolderPreview" style="margin-left:8px"></span>
    </div>
    <!-- 第3行: 采集信息 (匹配原始工具提示语) -->
    <div class="row" style="margin-top:6px">
      <label style="white-space:nowrap">这次采什么?</label>
      <input id="capNote" placeholder="如: 缺陷棒 缺陷30mm 中敲" style="width:180px" oninput="updateCapFolderPreview()">
      <label style="white-space:nowrap">计划敲几次?</label>
      <input id="capCount" type="number" placeholder="回车=不限" style="width:80px" min="1">
      <label style="white-space:nowrap">缺陷真实位置 mm?</label>
      <input id="capDefectMm" type="number" placeholder="正常棒直接回车" style="width:100px">
    </div>
    <div class="measure-bar">
      <label>波速: <input id="capVelocity" type="number" value="2200" style="width:70px"> m/s</label>
      <button class="secondary" onclick="startMeasure('cap')">📏 测距</button>
      <button class="secondary" onclick="resetMeasure('cap')">🔄 复位</button>
      <span id="capMeasureInfo" style="color:#ffab40;font-size:12px"></span>
    </div>
  </div>

  <!-- 波形图 + 右侧状态面板 -->
  <div class="capture-layout">
    <div class="capture-plot card" style="padding:0"><div id="capPlot" class="plot-container"></div></div>
    <div class="capture-sidebar" id="capSidebar">
      <div class="cap-section-title">📋 采集信息</div>
      <div class="cap-info-row"><span class="cap-info-label">说明</span><span class="cap-info-value" id="capSiNote">-</span></div>
      <div class="cap-info-row"><span class="cap-info-label">目标次数</span><span class="cap-info-value" id="capSiCount">不限</span></div>
      <div class="cap-info-row"><span class="cap-info-label">缺陷位置</span><span class="cap-info-value" id="capSiDefect">未设置</span></div>
      <div class="cap-info-row"><span class="cap-info-label">保存目录</span><span class="cap-info-value" id="capSiDir" style="font-size:10px">-</span></div>

      <div class="cap-section-title">📊 采集进度</div>
      <div>已采集: <b id="capSiHit" style="font-size:20px;color:#448aff">0</b> 帧</div>
      <div id="capCountdown" class="cap-countdown" style="display:none"></div>
      <div class="cap-progress-bar"><div class="cap-progress-fill" id="capProgressFill" style="width:0%"></div></div>

      <div class="cap-section-title">🔍 最近一帧</div>
      <div class="cap-last-hit" id="capLastHit">等待中...</div>
    </div>
  </div>
  <div id="capInfo"></div>
</div>

<!-- 目录浏览弹窗 -->
<div id="browseOverlay" class="browse-overlay" style="display:none" onclick="if(event.target===this)closeBrowseModal()">
  <div class="browse-dialog">
    <div class="browse-header">
      <h3>📁 选择保存目录</h3>
      <button class="secondary" onclick="closeBrowseModal()" style="font-size:18px;padding:2px 8px">✕</button>
    </div>
    <div class="browse-path-bar" id="browsePathBar"></div>
    <div class="browse-list" id="browseList"></div>
    <div class="browse-footer">
      <button class="secondary" onclick="navigateBrowse(null)">🖥 驱动器列表</button>
      <button onclick="selectBrowseDir()">✅ 选择此目录</button>
    </div>
  </div>
</div>

<!-- ====== Tab: 文件导入 ====== -->
<div id="panel-import" class="panel">
  <div class="card">
    <!-- 第1行: 数据目录路径 + 浏览 -->
    <div class="row">
      <label style="white-space:nowrap">数据目录:</label>
      <input id="impDataPath" style="flex:1;min-width:280px;font-size:11px;font-family:Consolas,monospace" onchange="onImpPathChange()">
      <button class="secondary" onclick="browseImportPath()" style="white-space:nowrap">📁 浏览...</button>
      <span style="color:#888;font-size:11px" id="impPathInfo"></span>
    </div>
    <!-- 第2行: 子目录 + 文件选择 + 分析按钮 -->
    <div class="row" style="margin-top:8px">
      <select id="impDir" style="width:300px" onchange="loadFileList()">
        <option value="">-- 选择子目录 (可选) --</option>
      </select>
      <select id="impFile" style="width:260px">
        <option value="">-- 选择 CSV 文件 --</option>
      </select>
      <button id="btnAnalyze" onclick="analyzeFile()">🔍 加载分析</button>
      <button class="secondary" onclick="prevFile()" title="上一个CSV文件" style="font-size:12px;padding:4px 8px">◀ 上一个</button>
      <button class="secondary" onclick="nextFile()" title="下一个CSV文件" style="font-size:12px;padding:4px 8px">下一个 ▶</button>
      <button class="secondary" onclick="copyPath('impFile')" title="复制当前选中CSV的完整路径" style="font-size:12px;padding:4px 8px">📋</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>测距方法: <select id="impMethod" onchange="onMethodChange('imp')"><option value="b2">B2反射位</option><option value="falling">下降段宽度</option></select></label>
      <label>系数: <input id="impCoeff" type="number" value="14.67" step="0.01" style="width:70px" oninput="recalcImportDist()"> <span id="impCoeffUnit">mm/样点(B2)</span></label>
      <label>判定阈值: <input id="impThresh" type="number" value="8" min="1" style="width:60px" oninput="recalcImportDist()"> 次翻转</label>
      <label>分析窗口: <input id="impWinStart" type="number" value="135" min="0" max="500" style="width:64px" oninput="onImpWindowChange()"> ~ <input id="impWinEnd" type="number" value="390" min="0" max="512" style="width:64px" oninput="onImpWindowChange()"><button class="secondary" onclick="resetWindow('imp')" title="恢复窗口默认" style="font-size:11px;padding:2px 5px;margin-left:4px">↺</button></label>
      <span style="color:#aaa;font-size:11px">(默认 120~512)</span>
    </div>
    <div class="measure-bar">
      <button class="secondary" onclick="startMeasure('imp')">📏 测距</button>
      <button class="secondary" onclick="resetMeasure('imp')">🔄 复位</button>
      <span id="impMeasureInfo" style="color:#ffab40;font-size:12px"></span>
    </div>
  </div>
  <div class="card"><div id="impPlot" class="plot-container"></div></div>
  <div id="impResult"></div>
</div>

<!-- 文件导入 — 目录浏览弹窗 -->
<div id="impBrowseOverlay" class="browse-overlay" style="display:none" onclick="if(event.target===this)closeImpBrowse()">
  <div class="browse-dialog">
    <div class="browse-header">
      <h3>📁 选择数据目录</h3>
      <button class="secondary" onclick="closeImpBrowse()" style="font-size:18px;padding:2px 8px">✕</button>
    </div>
    <div class="browse-path-bar" id="impBrowsePathBar"></div>
    <div class="browse-list" id="impBrowseList"></div>
    <div class="browse-footer">
      <button class="secondary" onclick="navigateImpBrowse(null)">🖥 驱动器列表</button>
      <button onclick="selectImpBrowseDir()">✅ 选择此目录</button>
    </div>
  </div>
</div>
</div>

<!-- ====== Tab: 误差分析 ====== -->
<div id="panel-error" class="panel">
  <div class="card">
    <!-- 第1行: 数据根目录 + 浏览 -->
    <div class="row" style="align-items:center">
      <label style="white-space:nowrap">数据根目录:</label>
      <input id="errDataPath" style="flex:1;min-width:300px;font-size:11px;font-family:Consolas,monospace" onchange="onErrPathChange()">
      <button class="secondary" onclick="copyPath('errDataPath')" title="复制当前数据根目录路径" style="font-size:12px;padding:4px 8px">📋</button>
      <button class="secondary" onclick="browseErrorPath()" style="white-space:nowrap">📁 浏览...</button>
      <span style="color:#888;font-size:11px" id="errPathInfo"></span>
    </div>
    <!-- 第2行: 阈值 + 距离系数 + 全选/取消 + 批量分析 -->
    <div class="row" style="margin-top:8px;align-items:center">
      <label>判定阈值: <input id="errThresh" type="number" value="8" min="1" style="width:60px"> 次翻转</label>
      <label>测距方法: <select id="errMethod" onchange="onMethodChange('err')"><option value="b2">B2反射位</option><option value="falling">下降段宽度</option></select></label>
      <label>距离系数: <input id="errCoeff" type="number" value="14.67" step="0.01" style="width:65px"> <span id="errCoeffUnit">mm/样点(B2)</span></label>
      <button class="secondary" onclick="errSelectAll()">☑ 全选</button>
      <button class="secondary" onclick="errDeselectAll()">☐ 取消</button>
      <button id="btnErrRun" onclick="runBatchAnalyze()" style="font-size:14px;padding:10px 24px">🚀 批量分析</button>
      <span id="errProgress" style="color:#ffab40;font-size:12px;margin-left:8px"></span>
    </div>
    <div id="errDirList" style="max-height:340px;overflow-y:auto;margin-top:8px;border:1px solid #333;border-radius:4px;padding:6px">
      <span style="color:#888">加载中...</span>
    </div>
  </div>

  <div id="errSummary" class="card" style="display:none"></div>

  <div class="flex-2">
    <div class="card"><div id="errScatter" class="plot-container"></div></div>
    <div class="card"><div id="errHist" class="plot-container"></div></div>
  </div>

  <div id="errDetails" class="card" style="display:none"></div>

  <!-- 缺陷距离误差分析 (仅坏棒) -->
  <div id="distSection" style="display:none">
    <h3 style="margin:16px 0 8px;color:#ffab40;border-bottom:1px solid #333;padding-bottom:4px">
      📏 缺陷距离测量误差
    </h3>
    <div id="distSummary" class="card"></div>
    <div class="flex-2">
      <div class="card"><div id="distScatter" class="plot-container"></div></div>
      <div class="card"><div id="distHist" class="plot-container"></div></div>
    </div>
  </div>
</div>

<!-- 误差分析 — 目录浏览弹窗 -->
<div id="errBrowseOverlay" class="browse-overlay" style="display:none" onclick="if(event.target===this)closeErrBrowse()">
  <div class="browse-dialog">
    <div class="browse-header">
      <h3>📁 选择数据根目录</h3>
      <button class="secondary" onclick="closeErrBrowse()" style="font-size:18px;padding:2px 8px">✕</button>
    </div>
    <div class="browse-path-bar" id="errBrowsePathBar"></div>
    <div class="browse-list" id="errBrowseList"></div>
    <div class="browse-footer">
      <button class="secondary" onclick="navigateErrBrowse(null)">🖥 驱动器列表</button>
      <button onclick="selectErrBrowseDir()">✅ 选择此目录</button>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
const socket = io();
const SAMPLE_US = 10.42, DEFAULT_V = 2200;
let measureState = {};
let lastImpData = null;  // 缓存最近一次文件分析结果, 用于系数联动
let lastImpPath = '';    // 当前分析的文件完整路径, 用于复制
let lastSerData = null;  // 缓存最近一次串口分析结果, 用于系数联动

// ── 测距方法管理 ──
const DIST_METHODS = {
  b2:      {coeff: 14.67, label: 'B2反射位',  desc: '冲击峰后第2边沿 - 冲击峰'},
  falling: {coeff: 15,    label: '下降段宽度', desc: 'e4~e7区域 qiudao=1 周期数'},
};

function getActiveMethod(ctx) {
  const sel = document.getElementById(ctx + 'Method');
  return (sel && sel.value) ? sel.value : 'b2';
}

function getDistValue(data, method) {
  // 返回指定方法的测距特征值 (样点数)
  if (method === 'falling') return data.falling_width || 0;
  return data.reflection_pos || 0;  // b2 (默认)
}

function getDistLabel(method) {
  return (DIST_METHODS[method] || DIST_METHODS['b2']).label;
}

function onMethodChange(ctx) {
  const method = getActiveMethod(ctx);
  const info = DIST_METHODS[method] || DIST_METHODS['b2'];

  // 更新系数默认值
  const coeffEl = document.getElementById(ctx + 'Coeff');
  if (coeffEl) coeffEl.value = info.coeff;

  // 更新系数标签
  const unitEl = document.getElementById(ctx + 'CoeffUnit');
  if (unitEl) unitEl.textContent = 'mm/样点(' + info.label + ')';

  // 触发重新计算
  if (ctx === 'imp') recalcImportDist();
  else if (ctx === 'ser') recalcSerDist();
  else if (ctx === 'err' && window._lastErrData) renderDistAnalysis(window._lastErrData);
}

// ── SocketIO ──
socket.on('connect', () => {
  document.getElementById('connStatus').textContent = '● 已连接';
  document.getElementById('connStatus').style.color = '#4caf50';
});
socket.on('disconnect', () => {
  document.getElementById('connStatus').textContent = '● 断开';
  document.getElementById('connStatus').style.color = '#f44336';
});
socket.on('status', d => toast(d.msg, 'info'));
socket.on('error', d => toast(d.msg, 'error'));

socket.on('frame', data => {
  const ctx = data.capture ? 'cap' : 'ser';
  updatePlot(ctx, data);
  updateInfo(ctx, data);

  // ── 串口监听 Tab: Verilog 实时分析 ──
  if (!data.capture && data.vlg && !data.vlg.error) {
    lastSerData = {
      vlg: data.vlg, fpga: data.fpga,
      hit: data.hit, n_pts: data.n_pts, match: data.match,
    };
    // 同步窗口输入框显示实际使用的窗口
    if (data.vlg.analysis_win) {
      document.getElementById('serWinStart').value = data.vlg.analysis_win.start;
      document.getElementById('serWinEnd').value = data.vlg.analysis_win.end;
    }
    updateVlgInfo('ser', data.vlg, data.fpga, data.match);
  }

  if (data.capture) {
    document.getElementById('capStatus').textContent = `采集中: ${data.hit} 帧 | ${data.timestamp}`;
  } else {
    document.getElementById('serHitCount').textContent = `#${data.hit}`;
    document.getElementById('serStatus').textContent = `监听中 | ${data.timestamp}`;
  }
});

// ── Tab 切换 ──
function switchTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  const idx = name==='serial'?1:name==='capture'?2:name==='import'?3:4;
  document.querySelector(`.tab:nth-child(${idx})`).classList.add('active');
  document.getElementById(`panel-${name}`).classList.add('active');
  refreshPorts();
  if (name === 'import') loadDirs();
  if (name === 'error') loadErrDirs();
}

// ── 串口列表 ──
async function refreshPorts() {
  try {
    const r = await fetch('/api/ports');
    const d = await r.json();
    const opts = d.ports.map(p => `<option value="${p.device}">${p.device} - ${p.description}</option>`).join('');
    ['serPort','capPort'].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.innerHTML = opts || '<option>无可用串口</option>';
    });
    if (d.error) console.warn('端口扫描警告:', d.error);
  } catch(e) {
    console.error('获取串口列表失败:', e);
    ['serPort','capPort'].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.innerHTML = '<option value="">获取失败, 请刷新页面重试</option>';
    });
  }
}

// 页面加载时执行一次, 之后切 Tab 也刷新
refreshPorts();

// ── 串口连接 ──
function toggleSerial() {
  const btn = document.getElementById('btnSerConn');
  if (btn.textContent.includes('连接')) {
    const port = document.getElementById('serPort').value;
    const baud = parseInt(document.getElementById('serBaud').value) || 115200;
    if (!port) { toast('请选择串口', 'error'); return; }
    socket.emit('start_serial', {port, baud});
    btn.textContent = '⏹ 断开'; btn.style.background = '#555';
    document.getElementById('serStatus').textContent = '连接中...';
  } else {
    socket.emit('stop_serial');
    btn.textContent = '▶ 连接'; btn.style.background = '#e94560';
    document.getElementById('serStatus').textContent = '未连接';
    document.getElementById('serHitCount').textContent = '';
  }
}

function toggleCapture() {
  const btn = document.getElementById('btnCapConn');
  if (btn.textContent.includes('开始')) {
    const port = document.getElementById('capPort').value;
    const baud = parseInt(document.getElementById('capBaud').value) || 115200;
    const note = document.getElementById('capNote').value || '';
    const c = document.getElementById('capCount').value;
    const count = c ? parseInt(c) : null;
    const d = document.getElementById('capDefectMm').value;
    const defect_mm = d ? parseInt(d) : null;
    const outdir_base = document.getElementById('capSavePath').value || '';
    if (!port) { toast('请选择串口', 'error'); return; }
    if (!socket.connected) { toast('与服务器连接已断开, 请刷新页面重试', 'error'); return; }

    // 初始化侧边栏
    document.getElementById('capSiNote').textContent = note || '(未填写)';
    document.getElementById('capSiCount').textContent = count || '不限';
    document.getElementById('capSiDefect').textContent = defect_mm ? defect_mm+' mm' : '未设置';
    document.getElementById('capSiHit').textContent = '0';
    document.getElementById('capCountdown').style.display = count ? 'block' : 'none';
    if (count) {
      document.getElementById('capCountdown').className = 'cap-countdown active';
      document.getElementById('capCountdown').textContent = '还差 '+count+' 次';
    }
    document.getElementById('capProgressFill').style.width = '0%';
    document.getElementById('capLastHit').textContent = '等待敲击...';

    socket.emit('start_capture', {port, baud, note, count, defect_mm, outdir_base});
    btn.style.display = 'none';
    document.getElementById('btnCapStop').style.display = '';
    document.getElementById('capStatus').textContent = '采集中...';
    document.getElementById('capStatus').style.color = '#ffab40';
  }
}
function stopCapture() {
  socket.emit('stop_serial');
  document.getElementById('btnCapConn').style.display = '';
  document.getElementById('btnCapStop').style.display = 'none';
  document.getElementById('btnCapConn').textContent = '▶ 开始采集';
  document.getElementById('btnCapConn').style.background = '#e94560';
  document.getElementById('capStatus').textContent = '已停止';
  document.getElementById('capStatus').style.color = '#aaa';
}

// ── 采集保存 — 侧边栏更新 ──
socket.on('capture_status', data => {
  document.getElementById('capSiHit').textContent = data.hit;
  document.getElementById('capSiDir').textContent = data.outdir || '-';
  document.getElementById('capSiNote').textContent = data.note || '(未填写)';
  document.getElementById('capSiCount').textContent = data.count || '不限';
  document.getElementById('capSiDefect').textContent = data.defect_mm ? data.defect_mm+' mm' : '未设置';

  // 还差 N 次
  const cd = document.getElementById('capCountdown');
  if (data.count && data.running) {
    cd.style.display = 'block';
    const rem = data.remaining;
    if (rem > 0) {
      cd.className = 'cap-countdown active';
      cd.textContent = '还差 ' + rem + ' 次';
    } else {
      cd.className = 'cap-countdown done';
      cd.textContent = '✅ 采集完成!';
    }
    const pct = Math.min(100, Math.round((data.hit / data.count) * 100));
    document.getElementById('capProgressFill').style.width = pct + '%';
  } else if (!data.running && data.count) {
    cd.style.display = 'block';
    cd.className = 'cap-countdown done';
    cd.textContent = '✅ 完成 (' + data.hit + ' 帧)';
    document.getElementById('capProgressFill').style.width = '100%';
  }

  // 最近一帧
  if (data.last_state) {
    const cls = 'state-' + data.last_state;
    document.getElementById('capLastHit').innerHTML =
      '判定: <b class="'+cls+'">' + data.last_state + '</b><br>' +
      '距离: <b>' + (data.last_dist != null ? data.last_dist + ' mm' : '-') + '</b><br>' +
      '置信度: ' + (data.last_conf || '-');
  }

  if (!data.running) {
    document.getElementById('btnCapConn').style.display = '';
    document.getElementById('btnCapStop').style.display = 'none';
    document.getElementById('btnCapConn').textContent = '▶ 开始采集';
    document.getElementById('btnCapConn').style.background = '#e94560';
    document.getElementById('capStatus').textContent = '已停止';
    document.getElementById('capStatus').style.color = '#aaa';
  }
});

// ── 目录浏览弹窗 ──
let browseCurrentPath = '';

async function browseSavePath() {
  document.getElementById('browseOverlay').style.display = 'flex';
  await navigateBrowse('');
}

async function navigateBrowse(path) {
  try {
    const r = await fetch('/api/browse_path', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({path: path || ''}),
    });
    const d = await r.json();
    if (d.error) { toast(d.error, 'error'); return; }

    browseCurrentPath = d.current || '';

    // 路径面包屑
    let barHtml = '';
    if (d.drives && d.drives.length > 0) {
      barHtml = '<span style="color:#888">此电脑</span>';
    } else {
      barHtml = '<span style="color:#888">📁</span> ';
      if (d.parent != null) {
        barHtml += '<span onclick="navigateBrowse(\'' + d.parent.replace(/\\/g,'\\\\') + '\')">⬆ 上级</span> » ';
      }
      barHtml += '<span style="color:#eee">' + (d.current || '') + '</span>';
    }
    document.getElementById('browsePathBar').innerHTML = barHtml;

    // 目录列表
    let listHtml = '';
    if (d.drives && d.drives.length > 0) {
      d.drives.forEach(drv => {
        listHtml += '<div class="browse-item drive" ondblclick="navigateBrowse(\'' + drv.path.replace(/\\/g,'\\\\') + '\')" onclick="browseCurrentPath=\'' + drv.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">💽</span><span>' + drv.name + '</span></div>';
      });
    }
    if (d.subdirs && d.subdirs.length > 0) {
      d.subdirs.forEach(sd => {
        listHtml += '<div class="browse-item" ondblclick="navigateBrowse(\'' + sd.path.replace(/\\/g,'\\\\') + '\')" onclick="browseCurrentPath=\'' + sd.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">📁</span><span>' + sd.name + '</span></div>';
      });
    }
    if (!listHtml) {
      listHtml = '<div style="color:#888;padding:20px;text-align:center">此目录下没有子文件夹</div>';
    }
    document.getElementById('browseList').innerHTML = listHtml;
  } catch(e) {
    toast('浏览目录失败: '+e, 'error');
  }
}

function selectBrowseDir() {
  if (browseCurrentPath) {
    document.getElementById('capSavePath').value = browseCurrentPath;
    updateCapFolderPreview();
  }
  closeBrowseModal();
}

function closeBrowseModal() {
  document.getElementById('browseOverlay').style.display = 'none';
}

// ── 自动编号预览 ──
function updateCapFolderPreview() {
  const path = document.getElementById('capSavePath').value || '';
  const note = document.getElementById('capNote').value || '';
  const preview = document.getElementById('capFolderPreview');
  if (!path || !note) {
    preview.textContent = path ? '(请填写采集说明)' : '';
    return;
  }
  // 显示格式预览: XX_HHMMSS_说明/
  const now = new Date();
  const hhmmss = String(now.getHours()).padStart(2,'0') +
                 String(now.getMinutes()).padStart(2,'0') +
                 String(now.getSeconds()).padStart(2,'0');
  const safe = note.replace(/[\\/:*?"<>| ]/g, '_').substring(0, 40) || 'unnamed';
  preview.textContent = '→ 自动编号: ' + path.replace(/\\/g,'/').replace(/\/$/,'') + '/??_' + hhmmss + '_' + safe + '/';
}

// ── 文件导入 ──
async function loadDirs() {
  const path = document.getElementById('impDataPath').value || '';
  try {
    const url = '/api/data_dirs' + (path ? '?path=' + encodeURIComponent(path) : '');
    const r = await fetch(url);
    const d = await r.json();
    const sel = document.getElementById('impDir');
    sel.innerHTML = '<option value="">-- 选择子目录 (可选) --</option>' +
      d.dirs.map(dd => `<option value="${dd.path}">${dd.name} (${dd.n_files}个CSV)</option>`).join('');
    if (d.current_path) {
      document.getElementById('impPathInfo').textContent = d.dirs.length + ' 个子目录';
    }
  } catch(e) { toast('加载目录失败: '+e, 'error'); }
}

// ── 文件导入 — 路径变更时重新加载子目录 ──
function onImpPathChange() {
  document.getElementById('impFile').innerHTML = '<option value="">-- 选择 CSV 文件 --</option>';
  loadDirs();
}

async function loadFileList() {
  const dir = document.getElementById('impDir').value;
  const sel = document.getElementById('impFile');
  sel.innerHTML = '<option value="">-- 选择 CSV 文件 --</option>';
  if (!dir) {
    // 如果没选子目录, 尝试直接列出根目录下的 CSV
    const rootPath = document.getElementById('impDataPath').value;
    if (!rootPath) return;
    try {
      const r = await fetch('/api/data_files?dir=' + encodeURIComponent(rootPath));
      const d = await r.json();
      sel.innerHTML = d.files.map(f => `<option value="${f.path}">${f.name}</option>`).join('');
    } catch(e) {}
    return;
  }
  try {
    const r = await fetch('/api/data_files?dir=' + encodeURIComponent(dir));
    const d = await r.json();
    sel.innerHTML = d.files.map(f => `<option value="${f.path}">${f.name}</option>`).join('');
  } catch(e) { toast('加载文件列表失败: '+e, 'error'); }
}

// ── 文件导入 — 目录浏览弹窗 ──
let impBrowseCurrentPath = '';

async function browseImportPath() {
  document.getElementById('impBrowseOverlay').style.display = 'flex';
  await navigateImpBrowse('');
}

async function navigateImpBrowse(path) {
  try {
    const r = await fetch('/api/browse_path', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({path: path || ''}),
    });
    const d = await r.json();
    if (d.error) { toast(d.error, 'error'); return; }

    impBrowseCurrentPath = d.current || '';

    let barHtml = '';
    if (d.drives && d.drives.length > 0) {
      barHtml = '<span style="color:#888">此电脑</span>';
    } else {
      barHtml = '<span style="color:#888">📁</span> ';
      if (d.parent != null) {
        barHtml += '<span onclick="navigateImpBrowse(\'' + d.parent.replace(/\\/g,'\\\\') + '\')">⬆ 上级</span> » ';
      }
      barHtml += '<span style="color:#eee">' + (d.current || '') + '</span>';
    }
    document.getElementById('impBrowsePathBar').innerHTML = barHtml;

    let listHtml = '';
    if (d.drives && d.drives.length > 0) {
      d.drives.forEach(drv => {
        listHtml += '<div class="browse-item drive" ondblclick="navigateImpBrowse(\'' + drv.path.replace(/\\/g,'\\\\') + '\')" onclick="impBrowseCurrentPath=\'' + drv.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">💽</span><span>' + drv.name + '</span></div>';
      });
    }
    if (d.subdirs && d.subdirs.length > 0) {
      d.subdirs.forEach(sd => {
        listHtml += '<div class="browse-item" ondblclick="navigateImpBrowse(\'' + sd.path.replace(/\\/g,'\\\\') + '\')" onclick="impBrowseCurrentPath=\'' + sd.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">📁</span><span>' + sd.name + '</span></div>';
      });
    }
    if (!listHtml) {
      listHtml = '<div style="color:#888;padding:20px;text-align:center">此目录下没有子文件夹</div>';
    }
    document.getElementById('impBrowseList').innerHTML = listHtml;
  } catch(e) {
    toast('浏览目录失败: '+e, 'error');
  }
}

function selectImpBrowseDir() {
  if (impBrowseCurrentPath) {
    document.getElementById('impDataPath').value = impBrowseCurrentPath;
    onImpPathChange();
  }
  closeImpBrowse();
}

function closeImpBrowse() {
  document.getElementById('impBrowseOverlay').style.display = 'none';
}

// ── 文件导入: 上一个/下一个文件导航 ──
function _getImpFileList() {
  const sel = document.getElementById('impFile');
  const files = [];
  for (let i = 0; i < sel.options.length; i++) {
    if (sel.options[i].value) files.push({name: sel.options[i].text, path: sel.options[i].value});
  }
  return files;
}

function navigateFile(delta) {
  const files = _getImpFileList();
  if (files.length === 0) { toast('文件列表为空', 'error'); return; }
  const curPath = document.getElementById('impFile').value;
  let idx = files.findIndex(f => f.path === curPath);
  if (idx < 0) idx = 0;
  else idx = (idx + delta + files.length) % files.length;
  // 选中并触发分析
  document.getElementById('impFile').value = files[idx].path;
  analyzeFile();
}

function prevFile() { navigateFile(-1); }
function nextFile() { navigateFile(1); }

function analyzeFile() {
  const path = document.getElementById('impFile').value;
  if (!path) { toast('请选择 CSV 文件', 'error'); return; }
  lastImpPath = path;
  document.getElementById('btnAnalyze').disabled = true;
  document.getElementById('btnAnalyze').textContent = '分析中...';
  const ws = parseInt(document.getElementById('impWinStart').value) || 120;
  const we = parseInt(document.getElementById('impWinEnd').value) || 512;
  socket.emit('analyze_file', {path: path, win_start: ws, win_end: we});
}

socket.on('analysis_result', data => {
  document.getElementById('btnAnalyze').disabled = false;
  document.getElementById('btnAnalyze').textContent = '🔍 加载分析';

  if (data.error) {
    toast(data.error, 'error');
    return;
  }

  // ── 同步窗口输入框显示实际使用的窗口 ──
  if (data.analysis_win) {
    document.getElementById('impWinStart').value = data.analysis_win.start;
    document.getElementById('impWinEnd').value = data.analysis_win.end;
  }

  // ── 缓存原始数据 (不含系数相关结果) ──
  lastImpData = {
    n: data.n, note: data.note, total_edges: data.total_edges,
    win_edges: data.win_edges,
    e4: data.e4, e5: data.e5, e6: data.e6, e7: data.e7,
    falling_width: data.falling_width,
    falling_start: data.falling_start, falling_end: data.falling_end,
    reflection_pos: data.reflection_pos,
    e1_after: data.e1_after, e2_after: data.e2_after, e3_after: data.e3_after,
    impact_idx: data.impact_idx,
    defect_mm: data.defect_mm,
    samples: data.samples,
    marked_points: data.marked_points,
    edges: data.edges,
    analysis_win: data.analysis_win,
    region_start: data.region_start, region_end: data.region_end,
  };

  const divId = 'impPlot';
  const wf = data.samples, x = wf.map((_,i)=>i);

  // ── 构建 shapes (不依赖系数) ──
  const shapes = [];
  if (data.analysis_win) {
    shapes.push({type:'rect', x0:data.analysis_win.start, x1:data.analysis_win.end,
      yref:'paper', y0:0, y1:1, line:{color:'#666',width:1,dash:'dot'}, fillcolor:'rgba(255,255,255,0.02)'});
  }
  (data.edges||[]).forEach(e => {
    const isKey = (e.num >= 4 && e.num <= 7);
    shapes.push({type:'line', x0:e.index, x1:e.index, yref:'paper', y0:0, y1:1,
      line:{color: isKey?'#ff9800':'#555', width: isKey?2:0.5, dash: isKey?'solid':'dot'}});
  });
  shapes.push({type:'line', x0:data.impact_idx, x1:data.impact_idx, yref:'paper', y0:0, y1:1,
    line:{color:'red', width:2, dash:'dash'}});
  if (data.region_start != null && data.region_end != null) {
    shapes.push({type:'rect', x0:data.region_start, x1:data.region_end,
      yref:'paper', y0:0, y1:1, line:{color:'#ff9800',width:2},
      fillcolor:'rgba(255,152,0,0.08)'});
  }
  // ── 下降段起止标注: 青虚线(起点) + 紫线(终点) ──
  if (data.falling_start != null) {
    shapes.push({type:'line', x0:data.falling_start, x1:data.falling_start, yref:'paper', y0:0, y1:1,
      line:{color:'#00e5ff', width:2.5, dash:'dash'}});
  }
  if (data.falling_end != null) {
    shapes.push({type:'line', x0:data.falling_end, x1:data.falling_end, yref:'paper', y0:0, y1:1,
      line:{color:'#e040fb', width:2.5, dash:'dash'}});
  }

  const markTraces = [];
  if (data.marked_points && data.marked_points.length > 0) {
    data.marked_points.forEach(p => {
      markTraces.push({
        x: [p.index], y: [p.value],
        type: 'scatter', mode: 'markers+text',
        marker: {color: p.type==='peak'?'#00e5ff':'#ff4081', size: 12, symbol: p.type==='peak'?'triangle-down':'triangle-up'},
        text: [p.type==='peak'?'波峰':'波谷'], textposition: 'top center',
        textfont: {color: p.type==='peak'?'#00e5ff':'#ff4081', size: 11},
        showlegend: false,
      });
    });
  }

  const allTraces = [{x, y:wf, type:'scatter', mode:'lines', line:{color:'#448aff',width:1.2}, name:'waveform'}].concat(markTraces);

  Plotly.react(divId, allTraces, {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:30,b:40,l:50,r:20},
    xaxis:{title:'Sample', gridcolor:'#333', range:[0, wf.length]},
    yaxis:{title:'ADC', gridcolor:'#333', zerolinecolor:'#555'},
    shapes,
    title: {text: '', font: {size: 10, color: '#aaa'}},
    showlegend: false,
  }, {responsive:true});

  // 测距 handler 已在 initEmptyPlot 中注册, Plotly.react 不会销毁事件绑定,
  // 无需重新注册 (重复注册会导致一次点击触发两次 handler, 使 pointA==pointB, 距离永远为0)

  // ── 用当前系数生成信息卡和标题 ──
  recalcImportDist();
});

// ── 系数联动: 修改系数后重新计算所有距离 ──
function recalcImportDist() {
  if (!lastImpData) return;
  const d = lastImpData;
  const coeff = parseFloat(document.getElementById('impCoeff').value) || 14.67;
  const divId = 'impPlot';

  // ── 当前选定的测距方法 ──
  const method = getActiveMethod('imp');
  const distVal = getDistValue(d, method);
  const distLabel = getDistLabel(method);
  const verilogDist = Math.round(distVal * coeff);

  // ── 参考: 敲击峰→波峰距离 (辅助) ──
  let peakDist = null, peakMethod = '';
  const imp = d.impact_idx;
  if (d.marked_points && d.marked_points.length > 0) {
    const peakPt = d.marked_points.find(p => p.type === 'peak');
    if (peakPt) {
      const ds = Math.abs(peakPt.index - imp);
      peakDist = Math.round(ds * coeff);
      peakMethod = `敲击峰→波峰 Δ=${ds}样点×${coeff}`;
    }
  }

  // 判定阈值
  const thresh = parseInt(document.getElementById('impThresh').value) || 8;
  const winEdges = d.win_edges || 0;
  const isDefect = winEdges >= thresh;
  const verdictColor = isDefect ? '#f44336' : '#4caf50';
  const verdictText = isDefect ? '坏桩(缺陷)' : '好桩(正常)';

  // ── 更新标题 ──
  let title = `${d.n}pt | 判定:${verdictText} | 窗口翻转:${winEdges}次(阈值${thresh})`;
  if (distVal > 0) {
    title += ` | ${distLabel}距离=${verilogDist}mm (${distVal}样点×${coeff})`;
    if (d.defect_mm != null) {
      const err = verilogDist - d.defect_mm;
      const sign = err >= 0 ? '+' : '';
      const pct = (err / d.defect_mm * 100).toFixed(1);
      title += ` | 真值=${d.defect_mm}mm | 误差=${sign}${err}mm (${sign}${pct}%)`;
    }
  }
  Plotly.relayout(divId, {title: {text: title, font: {size: 10, color: '#aaa'}}});

  // 更新信息卡
  let html = '<div class="card">';
  // ── 文件路径 (可复制) ──
  if (lastImpPath) {
    html += '<div style="display:flex;align-items:center;gap:8px;margin-bottom:10px;padding:6px 10px;background:#0f1a2e;border-radius:4px">';
    html += '<span style="color:#888;font-size:11px;white-space:nowrap">📄 当前文件:</span>';
    html += '<code style="flex:1;font-size:11px;color:#ffab40;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="' + lastImpPath.replace(/"/g,'&quot;') + '">' + lastImpPath + '</code>';
    html += '<button class="secondary" onclick="copyPath(&quot;' + lastImpPath.replace(/\\/g,'\\\\').replace(/"/g,'\\"') + '&quot;)" title="复制完整路径" style="font-size:11px;padding:3px 8px;white-space:nowrap">📋 复制</button>';
    html += '</div>';
  }
  html += '<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px">';
  html += `<div><strong>文件信息</strong><br>样点数: ${d.n}<br>`;
  if (d.note) html += `说明: ${d.note}<br>`;
  html += `翻转总次数: ${d.total_edges}<br>`;
  html += `窗口内翻转: <b>${winEdges}</b> 次<br>`;
  html += `判定: <b style="font-size:16px;color:${verdictColor}">${verdictText}</b> (阈值=${thresh})<br></div>`;

  // ── 中列: 选定方法测距 ──
  html += `<div><strong>📏 ${distLabel}测距 (系数=${coeff}mm/样点)</strong><br>`;
  html += `冲击峰索引: <b>${d.impact_idx}</b><br>`;
  if (method === 'b2') {
    html += `冲击后e1: <b style="color:#00e5ff">${d.e1_after != null ? d.e1_after : 'N/A'}</b><br>`;
    html += `冲击后e2: <b style="color:#e040fb">${d.e2_after != null ? d.e2_after : 'N/A'}</b><br>`;
  }
  html += `${distLabel}: <b>${distVal}</b> 样点<br>`;
  html += `${distLabel}距离: <b style="font-size:18px;color:#ffab40">${verilogDist} mm</b> (${distVal}×${coeff})<br>`;
  if (d.defect_mm != null) {
    const errV = verilogDist - d.defect_mm;
    const signV = errV >= 0 ? '+' : '';
    const pctV = (errV / d.defect_mm * 100);
    const colorV = Math.abs(errV) < 50 ? '#4caf50' : Math.abs(errV) < 100 ? '#ffab40' : '#f44336';
    html += `真值: <b>${d.defect_mm} mm</b><br>`;
    html += `误差: <b style="color:${colorV}">${signV}${errV} mm (${signV}${pctV.toFixed(1)}%)</b>`;
  }
  html += '</div>';

  // ── 右列: 辅助参考 ──
  html += `<div><strong>📐 辅助参考</strong><br>`;
  html += `第4次翻转: 样点 ${d.e4||'N/A'}<br>`;
  html += `第5次翻转: 样点 ${d.e5||'N/A'}<br>`;
  html += `第6次翻转(e6): 样点 ${d.e6||'N/A'}<br>`;
  html += `第7次翻转(e7): 样点 ${d.e7||'N/A'}<br>`;
  if (peakDist != null) {
    html += `敲击峰→波峰: <b>${peakDist} mm</b><br>`;
    html += `<span style="font-size:10px;color:#888">(${peakMethod})</span>`;
  } else {
    html += '<span style="color:#888;font-size:11px">未检测到有效峰/谷</span>';
  }
  html += '</div></div>';

  if (d.marked_points && d.marked_points.length > 0) {
    html += '<div style="margin-top:8px"><strong>自动标记的波峰/波谷:</strong> ';
    html += d.marked_points.map(p =>
      `<span style="color:${p.type==='peak'?'#00e5ff':'#ff4081'}">${p.type==='peak'?'▲波峰':'▼波谷'}@${p.index} (${p.value.toFixed(0)})</span>`
    ).join(' &nbsp;|&nbsp; ');
    html += '</div>';
  }

  html += '<div style="margin-top:6px;font-size:11px;color:#888">';
  html += '<span style="color:#ff9800">▬</span> 橙线=第4~6翻转 &nbsp;';
  html += '<span style="color:#00e5ff">---</span> 青虚线=冲击后e1 &nbsp;';
  html += '<span style="color:#e040fb">---</span> 紫线=冲击后e2 &nbsp;';
  html += '<span style="color:#00e5ff">▼</span> 青三角=波峰 &nbsp;';
  html += '<span style="color:#ff4081">▲</span> 粉三角=波谷 &nbsp;';
  html += '<span style="color:red">---</span> 红线=敲击峰';
  html += '</div></div>';
  document.getElementById('impResult').innerHTML = html;
  recalcMeasureDist('imp');  // 联动手动测距结果
}

// ── 串口监听: Verilog 分析信息卡 (随系数联动) ──
function updateVlgInfo(ctx, vlg, fpga, match) {
  if (!vlg || vlg.error) return;
  const coeff = parseFloat(document.getElementById(ctx + 'Coeff').value) || 14.67;
  const thresh = parseInt(document.getElementById(ctx + 'Thresh').value) || 8;
  const winEdges = vlg.win_edges || 0;
  const isDefect = winEdges >= thresh;
  const verdictColor = isDefect ? '#f44336' : '#4caf50';
  const verdictText = isDefect ? '坏桩(缺陷)' : '好桩(正常)';
  const methodV = getActiveMethod(ctx);
  const distValV = getDistValue(vlg, methodV);
  const distLabelV = getDistLabel(methodV);
  const verilogDist = Math.round(distValV * coeff);

  // ── FPGA 距离 (串口回传) ──
  const fpgaDist = (fpga && fpga.distance_mm != null) ? fpga.distance_mm : null;
  const fpgaState = fpga ? fpga.state : null;
  const fpgaConf  = fpga ? fpga.confidence : null;

  // ── FPGA vs PC 对比 ──
  let compareHtml = '';
  if (fpgaDist != null) {
    const diff = verilogDist - fpgaDist;
    const diffSign = diff >= 0 ? '+' : '';
    const diffColor = Math.abs(diff) <= 2 ? '#4caf50' : Math.abs(diff) <= 5 ? '#ffab40' : '#f44336';
    const matchIcon = match === '✓' ? '<span style="color:#4caf50;font-size:18px">✓</span>' :
                      match === '✗' ? '<span style="color:#f44336;font-size:18px">✗</span>' : '';
    compareHtml = `
      <div style="margin-top:8px;padding:8px 10px;background:#0f1a2e;border-radius:6px;border-left:3px solid ${diffColor}">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px">
          <span style="font-size:11px;color:#888">FPGA vs PC 对比</span>
          ${matchIcon}
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;text-align:center">
          <div>
            <span style="font-size:11px;color:#888">FPGA 回传</span><br>
            <b style="font-size:20px;color:#448aff">${fpgaDist}</b> <span style="font-size:12px;color:#aaa">mm</span>
          </div>
          <div>
            <span style="font-size:11px;color:#888">PC Verilog</span><br>
            <b style="font-size:20px;color:#ffab40">${verilogDist}</b> <span style="font-size:12px;color:#aaa">mm</span>
          </div>
          <div>
            <span style="font-size:11px;color:#888">偏差</span><br>
            <b style="font-size:20px;color:${diffColor}">${diffSign}${diff}</b> <span style="font-size:12px;color:#aaa">mm</span>
          </div>
        </div>
        <div style="font-size:10px;color:#666;margin-top:3px">
          FPGA判定: <b style="color:${fpgaState==='DEFECT'?'#f44336':fpgaState==='NORMAL'?'#4caf50':'#888'}">${fpgaState||'?'}</b> | 置信度: ${fpgaConf||'?'} |
          PC ${distLabelV}: ${distValV}样点×${coeff}
        </div>
      </div>`;
  }

  let html = '<div class="card">';
  html += '<div style="display:grid;grid-template-columns:1fr 1fr;gap:10px">';
  html += `<div><strong>Verilog 算法分析</strong><br>`;
  html += `窗口内翻转: <b style="color:${verdictColor}">${winEdges}</b> 次<br>`;
  html += `判定: <b style="font-size:16px;color:${verdictColor}">${verdictText}</b> (阈值=${thresh})<br>`;
  html += `翻转总次数: ${vlg.total_edges}<br>`;
  html += `e4=${vlg.e4||'?'} e5=${vlg.e5||'?'} e6=${vlg.e6||'?'} e7=${vlg.e7||'?'}<br>`;
  html += `冲击后e1: <b style="color:#00e5ff">${vlg.e1_after||'?'}</b> e2: <b style="color:#e040fb">${vlg.e2_after||'?'}</b> e3: ${vlg.e3_after||'?'}</div>`;
  html += `<div><strong>📏 ${distLabelV}测距 (系数=<span style="color:#ffab40">${coeff}</span>)</strong><br>`;
  html += `${distLabelV}: <b>${distValV}</b> 样点<br>`;
  html += `距离: <b style="font-size:18px;color:#ffab40">${verilogDist} mm</b><br>`;
  html += `<span style="font-size:11px;color:#888">${distValV}样点 × ${coeff}mm/样点</span>`;
  if (!fpgaDist) {
    html += '<br><span style="font-size:11px;color:#666">(无FPGA结果段, 仅PC分析)</span>';
  }
  html += '</div></div>';

  // ── FPGA vs PC 对比栏 (有 FPGA 数据时显示) ──
  html += compareHtml;

  if (vlg.marked_points && vlg.marked_points.length > 0) {
    html += '<div style="margin-top:4px;font-size:12px">';
    html += vlg.marked_points.map(p =>
      `<span style="color:${p.type==='peak'?'#00e5ff':'#ff4081'}">${p.type==='peak'?'▲':'▼'}@${p.index}</span>`
    ).join(' &nbsp;');
    html += '</div>';
  }
  html += '</div>';
  document.getElementById(ctx + 'Info').innerHTML = html;
}

function recalcSerDist() {
  if (!lastSerData || !lastSerData.vlg) return;
  updateVlgInfo('ser', lastSerData.vlg, lastSerData.fpga, lastSerData.match);

  // 也更新波形图标题
  const vlg = lastSerData.vlg;
  const d = lastSerData;
  const coeff = parseFloat(document.getElementById('serCoeff').value) || 14.67;
  const thresh = parseInt(document.getElementById('serThresh').value) || 8;
  const winEdges = vlg.win_edges || 0;
  const isDefect = winEdges >= thresh;
  const verdictText = isDefect ? '坏桩' : '好桩';
  const methodS = getActiveMethod('ser');
  const distValS = getDistValue(vlg, methodS);
  const distLabelS = getDistLabel(methodS);
  const verilogDist = Math.round(distValS * coeff);
  let title = `[#${d.hit}] ${d.n_pts}pt | 判定:${verdictText} | 翻转:${winEdges}次(阈值${thresh})`;
  if (distValS > 0) title += ` | ${distLabelS}距离=${verilogDist}mm (${distValS}样点×${coeff})`;
  Plotly.relayout('serPlot', {title: {text: title, font: {size: 10, color: '#aaa'}}});
  recalcMeasureDist('ser');  // 联动手动测距结果
}

// ── 分析窗口联动 ──
let _impWindowTimer = null;

function resetWindow(ctx) {
  document.getElementById(ctx + 'WinStart').value = 135;
  document.getElementById(ctx + 'WinEnd').value = 390;
  if (ctx === 'imp') onImpWindowChange();
  else onSerWindowChange();
}

function onImpWindowChange() {
  // 防抖: 300ms 内不再输入才触发重新分析
  if (_impWindowTimer) clearTimeout(_impWindowTimer);
  _impWindowTimer = setTimeout(() => {
    if (!lastImpPath) return;
    document.getElementById('btnAnalyze').disabled = true;
    document.getElementById('btnAnalyze').textContent = '分析中...';
    const ws = parseInt(document.getElementById('impWinStart').value) || 120;
    const we = parseInt(document.getElementById('impWinEnd').value) || 512;
    socket.emit('analyze_file', {path: lastImpPath, win_start: ws, win_end: we});
  }, 300);
}

function onSerWindowChange() {
  // 发送窗口参数到后端, 后续帧将使用新窗口
  const ws = parseInt(document.getElementById('serWinStart').value) || 120;
  const we = parseInt(document.getElementById('serWinEnd').value) || 512;
  socket.emit('set_analysis_window', {win_start: ws, win_end: we});
  toast(`分析窗口已更新为 [${ws}, ${we}]，下一帧生效`, 'info');
}

// ── 初始化空图 ──
function initEmptyPlot(divId) {
  Plotly.newPlot(divId, [{x:[],y:[],type:'scatter',mode:'lines',line:{color:'#448aff',width:1.2}}], {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:10,b:40,l:50,r:20},
    xaxis:{title:'Sample', gridcolor:'#333'},
    yaxis:{title:'ADC', gridcolor:'#333', zerolinecolor:'#555'},
  }, {responsive:true});
  const ctx = divId === 'serPlot' ? 'ser' : divId === 'capPlot' ? 'cap' : 'imp';
  setupPlotMeasure(divId, ctx);
}

// ── 更新波形图 ──
function updatePlot(ctx, data) {
  const divId = ctx + 'Plot';
  const wf = data.waveform, x = wf.map((_,i)=>i);
  const shapes = [];

  if (data.fpga) {
    const impShow = (data.fpga.impact_recalc !== undefined) ? data.fpga.impact_recalc : data.fpga.impact_index;
    shapes.push({type:'line',x0:impShow,x1:impShow,yref:'paper',y0:0,y1:1,line:{color:'red',dash:'dash',width:1}});
    shapes.push({type:'line',x0:data.fpga.defect_index,x1:data.fpga.defect_index,yref:'paper',y0:0,y1:1,line:{color:'orange',dash:'dash',width:1}});
    shapes.push({type:'line',x0:data.fpga.bottom_index,x1:data.fpga.bottom_index,yref:'paper',y0:0,y1:1,line:{color:'green',dash:'dash',width:1}});
  }

  // ── Verilog 分析叠加 (串口监听 Tab) ──
  const markTraces = [];
  if (ctx === 'ser' && data.vlg && !data.vlg.error) {
    const vlg = data.vlg;
    // 分析窗口
    if (vlg.analysis_win) {
      shapes.push({type:'rect', x0:vlg.analysis_win.start, x1:vlg.analysis_win.end,
        yref:'paper', y0:0, y1:1, line:{color:'#666',width:1,dash:'dot'}, fillcolor:'rgba(255,255,255,0.02)'});
    }
    // 翻转位置
    (vlg.edges||[]).forEach(e => {
      const isKey = (e.num >= 4 && e.num <= 7);
      shapes.push({type:'line', x0:e.index, x1:e.index, yref:'paper', y0:0, y1:1,
        line:{color: isKey?'#ff9800':'#555', width: isKey?2:0.5, dash: isKey?'solid':'dot'}});
    });
    // 敲击峰
    shapes.push({type:'line', x0:vlg.impact_idx, x1:vlg.impact_idx, yref:'paper', y0:0, y1:1,
      line:{color:'red', width:2, dash:'dash'}});
    // 第4~6翻转区域
    if (vlg.region_start != null && vlg.region_end != null) {
      shapes.push({type:'rect', x0:vlg.region_start, x1:vlg.region_end,
        yref:'paper', y0:0, y1:1, line:{color:'#ff9800',width:2}, fillcolor:'rgba(255,152,0,0.08)'});
    }
    // ── 下降段起止标注 ──
    if (vlg.falling_start != null) {
      shapes.push({type:'line', x0:vlg.falling_start, x1:vlg.falling_start, yref:'paper', y0:0, y1:1,
        line:{color:'#00e5ff', width:2.5, dash:'dash'}});
    }
    if (vlg.falling_end != null) {
      shapes.push({type:'line', x0:vlg.falling_end, x1:vlg.falling_end, yref:'paper', y0:0, y1:1,
        line:{color:'#e040fb', width:2.5, dash:'dash'}});
    }
    // 标记点
    (vlg.marked_points||[]).forEach(p => {
      markTraces.push({
        x: [p.index], y: [p.value],
        type: 'scatter', mode: 'markers+text',
        marker: {color: p.type==='peak'?'#00e5ff':'#ff4081', size: 12, symbol: p.type==='peak'?'triangle-down':'triangle-up'},
        text: [p.type==='peak'?'波峰':'波谷'], textposition: 'top center',
        textfont: {color: p.type==='peak'?'#00e5ff':'#ff4081', size: 11},
        showlegend: false,
      });
    });
  }

  let title = `[#${data.hit}] ${data.n_pts}pt | PC: ${data.pc.prediction}/${data.pc.class_name} dist=${data.pc.distance_mm}mm`;
  if (data.fpga) {
    title += ` | FPGA: ${data.fpga.state} dist=${data.fpga.distance_mm}mm ${data.match}`;
  } else {
    title += ' | FPGA: (无结果段)';
  }

  const allTraces = [{x, y: wf, type:'scatter', mode:'lines', line:{color:'#448aff',width:1.2}, name:'waveform'}].concat(markTraces);

  Plotly.react(divId, allTraces, {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:30,b:40,l:50,r:20},
    xaxis:{title:'Sample', gridcolor:'#333', range:[0, wf.length]},
    yaxis:{title:'ADC', gridcolor:'#333', zerolinecolor:'#555'},
    shapes,
    title: {text: title, font: {size: 10, color: '#aaa'}}
  }, {responsive:true});
  setupPlotMeasure(divId, ctx);
}

// ── 判别信息卡 ──
function updateInfo(ctx, data) {
  const div = document.getElementById(ctx + 'Info');
  const pc = data.pc;
  let badge = '';
  if (pc.prediction === 'GOOD') badge = '<span class="result-badge badge-good">GOOD</span>';
  else if (pc.prediction === 'DEFECT') badge = '<span class="result-badge badge-defect">DEFECT</span>';
  else badge = '<span class="result-badge badge-invalid">' + pc.prediction + '</span>';

  let fpgaHtml = '';
  if (data.fpga) {
    const m = data.match === '✓' ? 'match-ok' : 'match-fail';
    const impFpga = data.fpga.impact_index;
    const impShow = (data.fpga.impact_recalc !== undefined) ? data.fpga.impact_recalc : impFpga;
    fpgaHtml = `<div><strong>FPGA 判别</strong><br>
      ${data.fpga.state} | dist=${data.fpga.distance_mm}mm | conf=${data.fpga.confidence}<br>
      imp=${impFpga}${impShow !== impFpga ? `→${impShow}` : ''} def=${data.fpga.defect_index} bot=${data.fpga.bottom_index}</div>
      <span style="font-size:20px" class="${m}">${data.match}</span>`;
  } else {
    fpgaHtml = '<div class="fpga-na"><strong>FPGA 判别</strong><br>(串口仅发送原始ADC, 无结果段)</div>';
  }

  div.innerHTML = `<div class="card"><div class="flex-2">
    <div><strong>PC 判别</strong><br>${badge} ${pc.class_name} | dist=${pc.distance_mm}mm | conf=${pc.confidence}</div>
    ${fpgaHtml}
  </div></div>`;
}

// ── 测距功能 ──
function getVelocity(ctx) {
  const el = document.getElementById(ctx + 'Velocity');
  return el ? (parseFloat(el.value) || DEFAULT_V) : DEFAULT_V;
}

function setupPlotMeasure(divId, ctx) {
  const el = document.getElementById(divId);
  if (!el || el._measureSetup) return;
  el._measureSetup = true;
  el.removeAllListeners && el.removeAllListeners('plotly_click');  // 防止重复绑定
  if (!measureState[ctx]) measureState[ctx] = {active:false, pointA:null, pointB:null};

  el.on('plotly_click', function(data) {
    const ms = measureState[ctx];
    if (!ms || !ms.active) return;
    if (!data.points || !data.points.length) return;
    const idx = Math.round(data.points[0].x);
    const d = document.getElementById(divId);

    if (ms.pointA === null) {
      ms.pointA = idx;
      Plotly.relayout(d, {shapes: [{type:'line',x0:idx,x1:idx,yref:'paper',y0:0,y1:1,line:{color:'cyan',dash:'dot',width:2}}]});
      document.getElementById(ctx + 'MeasureInfo').textContent = `A=${idx} | 点击第二个点...`;
    } else {
      ms.pointB = idx; ms.active = false;
      const dIdx = Math.abs(idx - ms.pointA);
      // 优先用系数 (文件导入Tab), 否则用波速 (串口/采集Tab)
      const coeffEl = document.getElementById(ctx + 'Coeff');
      let distMm, formula;
      if (coeffEl) {
        const coeff = parseFloat(coeffEl.value) || 15;
        distMm = Math.round(coeff * dIdx);
        formula = `Δ=${dIdx}样点 × ${coeff}mm/样点`;
      } else {
        const v = getVelocity(ctx);
        distMm = Math.round(v * dIdx * SAMPLE_US * 1e-6 / 2 * 1000);
        formula = `Δ=${dIdx}样点 Δt=${(dIdx*SAMPLE_US).toFixed(1)}μs (v=${v}m/s)`;
      }
      Plotly.relayout(d, {shapes: [
        {type:'line',x0:ms.pointA,x1:ms.pointA,yref:'paper',y0:0,y1:1,line:{color:'cyan',dash:'dot',width:2}},
        {type:'line',x0:idx,x1:idx,yref:'paper',y0:0,y1:1,line:{color:'magenta',dash:'dot',width:2}},
      ]});
      document.getElementById(ctx + 'MeasureInfo').innerHTML =
        `A=${ms.pointA} B=${idx} ${formula} <b>距离≈${distMm}mm</b>`;
    }
  });
}

function startMeasure(ctx) {
  const ms = measureState[ctx]; if (!ms) return;
  ms.active = true; ms.pointA = null; ms.pointB = null;
  Plotly.relayout(document.getElementById(ctx + 'Plot'), {shapes: []});
  document.getElementById(ctx + 'MeasureInfo').textContent = '测距中: 点击第一个点...';
}

// ── 系数联动: 重算已完成的测距结果 ──
function recalcMeasureDist(ctx) {
  const ms = measureState[ctx];
  if (!ms || ms.pointA === null || ms.pointB === null || ms.active) return;
  const dIdx = Math.abs(ms.pointB - ms.pointA);
  const coeffEl = document.getElementById(ctx + 'Coeff');
  let distMm, formula;
  if (coeffEl) {
    const coeff = parseFloat(coeffEl.value) || 15;
    distMm = Math.round(coeff * dIdx);
    formula = `Δ=${dIdx}样点 × ${coeff}mm/样点`;
  } else {
    const v = getVelocity(ctx);
    distMm = Math.round(v * dIdx * SAMPLE_US * 1e-6 / 2 * 1000);
    formula = `Δ=${dIdx}样点 Δt=${(dIdx*SAMPLE_US).toFixed(1)}μs (v=${v}m/s)`;
  }
  document.getElementById(ctx + 'MeasureInfo').innerHTML =
    `A=${ms.pointA} B=${ms.pointB} ${formula} <b>距离≈${distMm}mm</b>`;
}

function resetMeasure(ctx) { startMeasure(ctx); }

// ── 复制路径 ──
function copyPath(source) {
  let path = '';
  if (source === 'impFile') {
    path = document.getElementById('impFile').value;
  } else if (source === 'impLoaded') {
    path = lastImpPath;
  } else if (source === 'errDataPath') {
    path = document.getElementById('errDataPath').value;
  } else if (typeof source === 'string' && source.length > 0) {
    path = source;
  }
  if (!path) { toast('没有可复制的路径', 'error'); return; }
  if (navigator.clipboard) {
    navigator.clipboard.writeText(path).then(() => toast('已复制: ' + path, 'info'));
  } else {
    // fallback
    const ta = document.createElement('textarea');
    ta.value = path; ta.style.position = 'fixed'; ta.style.left = '-9999px';
    document.body.appendChild(ta); ta.select();
    document.execCommand('copy'); document.body.removeChild(ta);
    toast('已复制: ' + path, 'info');
  }
}

// ── Toast ──
function toast(msg, type) {
  const el = document.getElementById('toast');
  el.textContent = msg; el.className = 'toast-' + (type||'info');
  el.style.display = 'block';
  clearTimeout(el._timer);
  el._timer = setTimeout(() => el.style.display = 'none', 4000);
}

// ══════════════════════════════════════════════════════════════════════════════
// 误差分析 Tab
// ══════════════════════════════════════════════════════════════════════════════

async function loadErrDirs() {
  const container = document.getElementById('errDirList');
  const path = document.getElementById('errDataPath').value || '';
  try {
    const url = '/api/data_dirs' + (path ? '?path=' + encodeURIComponent(path) : '');
    const r = await fetch(url);
    const d = await r.json();
    if (d.error) { container.innerHTML = `<span style="color:#f44336">${d.error}</span>`; return; }
    if (d.current_path) {
      document.getElementById('errPathInfo').textContent = d.dirs.length + ' 个子目录';
    }
    let html = '';
    d.dirs.forEach(dd => {
      let gt = 'UNKNOWN', tagCls = '';
      if (dd.name.includes('好棒')) { gt = 'GOOD'; tagCls = 'tag-good'; }
      else if (dd.name.includes('坏棒') || dd.name.includes('坏帮')) { gt = 'DEFECT'; tagCls = 'tag-defect'; }
      html += `<label class="dir-check-row">
        <input type="checkbox" value="${dd.path.replace(/"/g,'&quot;')}" data-gt="${gt}" checked>
        <span style="flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${dd.name}">${dd.name}</span>
        <span class="${tagCls}">${gt}</span>
        <span style="color:#888;margin-left:8px;font-size:11px">${dd.n_files} 文件</span>
      </label>`;
    });
    container.innerHTML = html || '<span style="color:#888">无可用目录 (该路径下没有包含CSV的子目录)</span>';
  } catch(e) {
    container.innerHTML = `<span style="color:#f44336">加载失败: ${e}</span>`;
  }
}

// ── 误差分析 — 路径变更时重新加载 ──
function onErrPathChange() {
  loadErrDirs();
}

// ── 误差分析 — 目录浏览弹窗 ──
let errBrowseCurrentPath = '';

async function browseErrorPath() {
  document.getElementById('errBrowseOverlay').style.display = 'flex';
  await navigateErrBrowse('');
}

async function navigateErrBrowse(path) {
  try {
    const r = await fetch('/api/browse_path', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({path: path || ''}),
    });
    const d = await r.json();
    if (d.error) { toast(d.error, 'error'); return; }

    errBrowseCurrentPath = d.current || '';

    let barHtml = '';
    if (d.drives && d.drives.length > 0) {
      barHtml = '<span style="color:#888">此电脑</span>';
    } else {
      barHtml = '<span style="color:#888">📁</span> ';
      if (d.parent != null) {
        barHtml += '<span onclick="navigateErrBrowse(\'' + d.parent.replace(/\\/g,'\\\\') + '\')">⬆ 上级</span> » ';
      }
      barHtml += '<span style="color:#eee">' + (d.current || '') + '</span>';
    }
    document.getElementById('errBrowsePathBar').innerHTML = barHtml;

    let listHtml = '';
    if (d.drives && d.drives.length > 0) {
      d.drives.forEach(drv => {
        listHtml += '<div class="browse-item drive" ondblclick="navigateErrBrowse(\'' + drv.path.replace(/\\/g,'\\\\') + '\')" onclick="errBrowseCurrentPath=\'' + drv.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">💽</span><span>' + drv.name + '</span></div>';
      });
    }
    if (d.subdirs && d.subdirs.length > 0) {
      d.subdirs.forEach(sd => {
        listHtml += '<div class="browse-item" ondblclick="navigateErrBrowse(\'' + sd.path.replace(/\\/g,'\\\\') + '\')" onclick="errBrowseCurrentPath=\'' + sd.path.replace(/\\/g,'\\\\') + '\'">' +
          '<span class="icon">📁</span><span>' + sd.name + '</span></div>';
      });
    }
    if (!listHtml) {
      listHtml = '<div style="color:#888;padding:20px;text-align:center">此目录下没有子文件夹</div>';
    }
    document.getElementById('errBrowseList').innerHTML = listHtml;
  } catch(e) {
    toast('浏览目录失败: '+e, 'error');
  }
}

function selectErrBrowseDir() {
  if (errBrowseCurrentPath) {
    document.getElementById('errDataPath').value = errBrowseCurrentPath;
    onErrPathChange();
  }
  closeErrBrowse();
}

function closeErrBrowse() {
  document.getElementById('errBrowseOverlay').style.display = 'none';
}

function errSelectAll() {
  document.querySelectorAll('#errDirList input[type=checkbox]').forEach(cb => cb.checked = true);
}
function errDeselectAll() {
  document.querySelectorAll('#errDirList input[type=checkbox]').forEach(cb => cb.checked = false);
}

async function runBatchAnalyze() {
  const checkboxes = document.querySelectorAll('#errDirList input[type=checkbox]:checked');
  if (checkboxes.length === 0) { toast('请至少选择一个目录', 'error'); return; }
  const dirs = Array.from(checkboxes).map(cb => cb.value);
  const threshold = parseInt(document.getElementById('errThresh').value) || 8;

  const btn = document.getElementById('btnErrRun');
  btn.disabled = true; btn.textContent = '⏳ 分析中...';
  document.getElementById('errProgress').textContent = `正在分析 ${dirs.length} 个目录...`;
  document.getElementById('errSummary').style.display = 'none';
  document.getElementById('errDetails').style.display = 'none';

  try {
    const t0 = performance.now();
    const r = await fetch('/api/batch_analyze', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({dirs, threshold}),
    });
    const data = await r.json();
    const elapsed = ((performance.now() - t0) / 1000).toFixed(1);

    if (data.error) { toast(data.error, 'error'); return; }

    document.getElementById('errProgress').textContent =
      `✅ 完成! ${data.total_files} 文件, 耗时 ${elapsed}s`;

    renderErrSummary(data);
    renderErrScatter(data);
    renderErrHistogram(data);
    renderErrDetails(data);
    renderDistAnalysis(data);
  } catch(e) {
    toast('分析失败: ' + e, 'error');
  } finally {
    btn.disabled = false; btn.textContent = '🚀 批量分析';
  }
}

// ── 结果渲染 ──

function renderErrSummary(data) {
  const s = data.summary, th = data.threshold;
  const pct = v => (v * 100).toFixed(1);

  let html = `<div style="display:flex;gap:24px;flex-wrap:wrap;align-items:flex-start">`;

  // 混淆矩阵
  html += `<div>
    <strong>混淆矩阵 (阈值=${th}次翻转)</strong>
    <table style="border-collapse:collapse;margin-top:8px;font-size:13px">
      <tr><td></td><td style="padding:4px 10px;color:#888">预测GOOD</td><td style="padding:4px 10px;color:#888">预测DEFECT</td><td style="padding:4px 10px;color:#888;font-size:11px">合计</td></tr>
      <tr>
        <td style="padding:4px 10px;color:#888">实际GOOD</td>
        <td class="mx-cell mx-diag">${s.TN}<span class="mx-label">TN ✓</span></td>
        <td class="mx-cell mx-off">${s.FP}<span class="mx-label">FP ✗</span></td>
        <td class="mx-cell" style="color:#ccc">${s.TN+s.FP}</td>
      </tr>
      <tr>
        <td style="padding:4px 10px;color:#888">实际DEFECT</td>
        <td class="mx-cell mx-off">${s.FN}<span class="mx-label">FN ✗</span></td>
        <td class="mx-cell mx-diag">${s.TP}<span class="mx-label">TP ✓</span></td>
        <td class="mx-cell" style="color:#ccc">${s.FN+s.TP}</td>
      </tr>
      <tr>
        <td style="padding:4px 10px;color:#888">合计</td>
        <td class="mx-cell" style="color:#ccc">${s.TN+s.FN}</td>
        <td class="mx-cell" style="color:#ccc">${s.FP+s.TP}</td>
        <td class="mx-cell" style="color:#448aff">${s.total}</td>
      </tr>
    </table>
  </div>`;

  // 全局统计
  html += `<div>
    <strong>全局统计</strong>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px">
      <div>总文件数<br><span class="stat-num" style="color:#eee">${s.total}</span></div>
      <div>正确数<br><span class="stat-num stat-good">${s.TP+s.TN}</span></div>
      <div>准确率<br><span class="stat-num stat-good">${pct(s.accuracy)}%</span></div>
      <div>精确率<br><span class="stat-num stat-info">${pct(s.precision)}%</span></div>
      <div>召回率<br><span class="stat-num stat-warn">${pct(s.recall)}%</span></div>
      <div>F1 得分<br><span class="stat-num" style="color:#e94560">${pct(s.f1)}%</span></div>
    </div>
  </div>`;

  html += `</div>`;
  document.getElementById('errSummary').innerHTML = html;
  document.getElementById('errSummary').style.display = 'block';
}

function renderErrScatter(data) {
  const goodX=[], goodY=[], goodT=[];
  const defectX=[], defectY=[], defectT=[];

  // 按目录分组: GOOD 先, DEFECT 后
  const sorted = [...data.per_file].sort((a,b) => {
    if (a.truth !== b.truth) return a.truth === 'GOOD' ? -1 : 1;
    return a.dir.localeCompare(b.dir);
  });

  let idx = 0;
  let goodFolderEnd = 0, currentTruth = 'GOOD';
  const folderBounds = []; // 目录边界标注

  sorted.forEach((f,i) => {
    idx++;
    const label = `${f.dir}<br>${f.file}<br>win_edges=${f.win_edges} | ${f.correct ? '✓': '✗'}`;
    if (f.truth === 'GOOD') {
      goodX.push(idx); goodY.push(f.win_edges); goodT.push(label);
    } else {
      defectX.push(idx); defectY.push(f.win_edges); defectT.push(label);
    }
  });

  const traces = [
    {x:goodX, y:goodY, type:'scatter', mode:'markers',
     marker:{color:'#4caf50', size:3, opacity:.5},
     text:goodT, hoverinfo:'text', name:'实际 GOOD (好桩)'},
    {x:defectX, y:defectY, type:'scatter', mode:'markers',
     marker:{color:'#f44336', size:3, opacity:.5},
     text:defectT, hoverinfo:'text', name:'实际 DEFECT (坏桩)'},
  ];

  const layout = {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:40,b:40,l:50,r:20},
    xaxis:{title:'样本序号 (GOOD→DEFECT 按目录分组)', gridcolor:'#333'},
    yaxis:{title:'窗口内翻转次数 (win_edges)', gridcolor:'#333'},
    shapes:[{type:'line', x0:0, x1:idx, y0:data.threshold, y1:data.threshold,
              line:{color:'#ffab40', width:2, dash:'dash'}}],
    title:{text:`分类分布 (阈值=${data.threshold}, N=${sorted.length})`, font:{size:12, color:'#aaa'}},
    legend:{x:.01, y:.99},
  };
  Plotly.newPlot('errScatter', traces, layout, {responsive:true});
}

function renderErrHistogram(data) {
  const goodEdges=[], defectEdges=[];
  data.per_file.forEach(f => {
    if (f.truth === 'GOOD') goodEdges.push(f.win_edges);
    else defectEdges.push(f.win_edges);
  });

  const traces = [
    {x:goodEdges, type:'histogram', name:'实际 GOOD',
     marker:{color:'#4caf50', opacity:.5},
     xbins:{start:0, end:50, size:1}},
    {x:defectEdges, type:'histogram', name:'实际 DEFECT',
     marker:{color:'#f44336', opacity:.5},
     xbins:{start:0, end:50, size:1}},
  ];

  const layout = {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:40,b:40,l:50,r:20},
    xaxis:{title:'窗口内翻转次数 (win_edges)', gridcolor:'#333'},
    yaxis:{title:'文件数量', gridcolor:'#333'},
    barmode:'overlay',
    shapes:[{type:'line', x0:data.threshold, x1:data.threshold, yref:'paper', y0:0, y1:1,
              line:{color:'#ffab40', width:2, dash:'dash'}}],
    title:{text:`win_edges 分布直方图 (阈值=${data.threshold})`, font:{size:12, color:'#aaa'}},
    legend:{x:.7, y:.99},
  };
  Plotly.newPlot('errHist', traces, layout, {responsive:true});
}

function renderErrDetails(data) {
  const folders = data.directories;
  const pct = v => (v * 100).toFixed(1);

  let html = `<strong>各目录详细结果</strong>
    <table style="width:100%;border-collapse:collapse;margin-top:8px;font-size:12px">
    <tr style="color:#888;border-bottom:2px solid #333">
      <th style="text-align:left;padding:4px">目录</th>
      <th style="text-align:center;padding:4px">真值</th>
      <th style="text-align:center;padding:4px">文件数</th>
      <th style="text-align:center;padding:4px">TN</th>
      <th style="text-align:center;padding:4px">FP</th>
      <th style="text-align:center;padding:4px">FN</th>
      <th style="text-align:center;padding:4px">TP</th>
      <th style="text-align:center;padding:4px">准确率</th>
    </tr>`;

  folders.forEach(f => {
    const accPct = pct(f.accuracy);
    const c = f.accuracy >= .9 ? '#4caf50' : f.accuracy >= .7 ? '#ffab40' : '#f44336';
    const gtTag = f.truth === 'GOOD'
      ? '<span class="tag-good">GOOD</span>'
      : '<span class="tag-defect">DEFECT</span>';
    html += `<tr style="border-bottom:1px solid #222">
      <td style="text-align:left;padding:4px;max-width:280px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${f.name}">${f.name}</td>
      <td style="text-align:center;padding:4px">${gtTag}</td>
      <td style="text-align:center;padding:4px;color:#eee">${f.n_files}</td>
      <td style="text-align:center;padding:4px;color:#4caf50">${f.TN}</td>
      <td style="text-align:center;padding:4px;color:#f44336">${f.FP}</td>
      <td style="text-align:center;padding:4px;color:#ff9800">${f.FN}</td>
      <td style="text-align:center;padding:4px;color:#4caf50">${f.TP}</td>
      <td style="text-align:center;padding:4px"><b style="color:${c}">${accPct}%</b></td>
      <td style="text-align:center;padding:4px"><button class="secondary" onclick="copyPath('${f.path.replace(/\\/g,'\\\\').replace(/'/g,"\\'")}')" title="复制目录路径" style="font-size:10px;padding:2px 6px">📋</button></td>
    </tr>`;
  });

  html += '</table>';
  document.getElementById('errDetails').innerHTML = html;
  document.getElementById('errDetails').style.display = 'block';
}

// ── 缺陷距离误差分析 ──
function renderDistAnalysis(data) {
  window._lastErrData = data;  // 缓存，便于切换方法后重新渲染
  const distData = data.distance_data || [];
  const coeff = parseFloat(document.getElementById('errCoeff').value) || 15;
  const method = getActiveMethod('err');
  const distLabel = getDistLabel(method);

  if (distData.length === 0) {
    document.getElementById('distSection').style.display = 'none';
    return;
  }
  document.getElementById('distSection').style.display = 'block';

  // 计算距离和误差 (用选定方法)
  const points = distData.map(d => {
    const dv = getDistValue(d, method);
    return {
      ...d,
      measured_mm: Math.round(dv * coeff),
      error_mm: Math.round(dv * coeff) - d.defect_true_mm,
    };
  });

  // ── 汇总统计 ──
  const errors = points.map(p => p.error_mm);
  const absErrors = errors.map(e => Math.abs(e));
  const mae = (absErrors.reduce((a,b) => a+b, 0) / errors.length).toFixed(1);
  const rmse = Math.sqrt(errors.reduce((a,b) => a + b*b, 0) / errors.length).toFixed(1);
  const within10 = (absErrors.filter(e => e <= 10).length / errors.length * 100).toFixed(1);
  const within20 = (absErrors.filter(e => e <= 20).length / errors.length * 100).toFixed(1);
  const within50 = (absErrors.filter(e => e <= 50).length / errors.length * 100).toFixed(1);
  const meanErr = (errors.reduce((a,b) => a+b, 0) / errors.length).toFixed(1);
  const maxErr = Math.max(...absErrors);

  // 按真值分组
  const byTruth = {};
  points.forEach(p => {
    const key = p.defect_true_mm;
    if (!byTruth[key]) byTruth[key] = [];
    byTruth[key].push(p);
  });

  let summaryHtml = '<div style="display:flex;gap:20px;flex-wrap:wrap;align-items:flex-start">';
  summaryHtml += `<div><strong>距离误差统计 (${distLabel}法, 系数=${coeff}mm/样点)</strong>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px 16px;margin-top:8px">
      <div>样本数: <b style="color:#eee">${points.length}</b></div>
      <div>平均误差: <b style="color:${Math.abs(parseFloat(meanErr))<20?'#4caf50':'#f44336'}">${parseFloat(meanErr)>0?'+':''}${meanErr} mm</b></div>
      <div>MAE: <b style="color:#ffab40">${mae} mm</b></div>
      <div>RMSE: <b style="color:#ffab40">${rmse} mm</b></div>
      <div>最大误差: <b style="color:#f44336">${maxErr} mm</b></div>
      <div>≤10mm: <b style="color:#4caf50">${within10}%</b></div>
      <div>≤20mm: <b style="color:#ffab40">${within20}%</b></div>
      <div>≤50mm: <b style="color:#e94560">${within50}%</b></div>
    </div></div>`;

  // 按真值分组的统计
  summaryHtml += '<div><strong>按缺陷位置分组</strong><table style="font-size:12px;border-collapse:collapse;margin-top:4px">';
  summaryHtml += '<tr style="color:#888;border-bottom:1px solid #333"><th style="text-align:left;padding:2px 8px">真值</th><th style="text-align:right;padding:2px 8px">N</th><th style="text-align:right;padding:2px 8px">均值</th><th style="text-align:right;padding:2px 8px">MAE</th><th style="text-align:right;padding:2px 8px">≤20mm</th></tr>';
  Object.keys(byTruth).sort((a,b) => +a - +b).forEach(key => {
    const pts = byTruth[key];
    const es = pts.map(p => p.error_mm);
    const gMae = (es.reduce((a,b) => a + Math.abs(b), 0) / es.length).toFixed(1);
    const gMean = (es.reduce((a,b) => a+b, 0) / es.length).toFixed(1);
    const g20 = (es.filter(e => Math.abs(e) <= 20).length / es.length * 100).toFixed(1);
    const dist = pts[0].measured_mm;
    summaryHtml += `<tr style="border-bottom:1px solid #222">
      <td style="padding:2px 8px;color:#ffab40">${key}mm</td>
      <td style="text-align:right;padding:2px 8px;color:#ccc">${pts.length}</td>
      <td style="text-align:right;padding:2px 8px;color:${Math.abs(parseFloat(gMean))<20?'#4caf50':'#f44336'}">${parseFloat(gMean)>0?'+':''}${gMean}</td>
      <td style="text-align:right;padding:2px 8px;color:#ffab40">${gMae}</td>
      <td style="text-align:right;padding:2px 8px;color:${parseFloat(g20)>80?'#4caf50':'#ffab40'}">${g20}%</td>
    </tr>`;
  });
  summaryHtml += '</table></div></div>';
  document.getElementById('distSummary').innerHTML = summaryHtml;

  // ── 散点图: 测量值 vs 真值 ──
  const scatterTraces = [];
  Object.keys(byTruth).sort((a,b) => +a - +b).forEach((key, i) => {
    const pts = byTruth[key];
    const colors = ['#448aff','#ffab40','#4caf50','#e94560','#00e5ff'];
    scatterTraces.push({
      x: pts.map(p => p.defect_true_mm),
      y: pts.map(p => p.measured_mm),
      type: 'scatter', mode: 'markers',
      marker: {color: colors[i % colors.length], size: 4, opacity: .5},
      name: `真值=${key}mm`,
      text: pts.map(p => `${p.dir}<br>${p.file}<br>真值=${p.defect_true_mm}mm 测量=${p.measured_mm}mm 误差=${p.error_mm>0?'+':''}${p.error_mm}mm (${distLabel}×${coeff})`),
      hoverinfo: 'text',
    });
  });
  // 理想线 y=x
  const allTrue = points.map(p => p.defect_true_mm);
  const minT = Math.min(...allTrue) - 50;
  const maxT = Math.max(...allTrue) + 50;
  scatterTraces.push({
    x: [minT, maxT], y: [minT, maxT],
    type: 'scatter', mode: 'lines',
    line: {color: '#666', width: 1, dash: 'dash'},
    name: '理想 y=x', hoverinfo: 'none',
  });

  Plotly.newPlot('distScatter', scatterTraces, {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:40,b:50,l:60,r:20},
    xaxis:{title:'真实缺陷位置 (mm)', gridcolor:'#333'},
    yaxis:{title:`${distLabel} 测量距离 (mm, 系数=${coeff})`, gridcolor:'#333'},
    title:{text:`缺陷距离 [${distLabel}]: 测量值 vs 真值 (N=${points.length})`, font:{size:12, color:'#aaa'}},
    legend:{x:.01, y:.99},
  }, {responsive:true});

  // ── 直方图: 误差分布 ──
  Plotly.newPlot('distHist', [{
    x: errors,
    type: 'histogram',
    marker: {color: '#ffab40', opacity: .6,
             line: {color: '#ffab40', width: 1}},
    xbins: {size: 5},
    name: '距离误差',
  }], {
    paper_bgcolor:'#16213e', plot_bgcolor:'#1a1a2e',
    font:{color:'#ccc'}, margin:{t:40,b:40,l:50,r:20},
    xaxis:{title:'测量误差 (mm)', gridcolor:'#333'},
    yaxis:{title:'文件数量', gridcolor:'#333'},
    shapes: [
      {type:'line', x0:0, x1:0, yref:'paper', y0:0, y1:1, line:{color:'#4caf50', width:1.5, dash:'dash'}},
      {type:'line', x0:parseFloat(meanErr), x1:parseFloat(meanErr), yref:'paper', y0:0, y1:1, line:{color:'#fff', width:1, dash:'dot'}},
    ],
    title:{text:`距离误差分布 (MAE=${mae}mm, RMSE=${rmse}mm)`, font:{size:12, color:'#aaa'}},
  }, {responsive:true});
}

// ══════════════════════════════════════════════════════════════════════════════

// ── 初始化 ──
window.onload = function() {
  refreshPorts();
  initEmptyPlot('serPlot');
  initEmptyPlot('capPlot');
  initEmptyPlot('impPlot');
  initEmptyPlot('errScatter');
  initEmptyPlot('errHist');
  // 初始化默认路径 (采集/导入/误差分析共用)
  fetch('/api/default_save_dir').then(r=>r.json()).then(d=>{
    if (d.path) {
      // 采集保存
      document.getElementById('capSavePath').value = d.path;
      updateCapFolderPreview();
      // 文件导入
      document.getElementById('impDataPath').value = d.path;
      loadDirs();
      // 误差分析
      document.getElementById('errDataPath').value = d.path;
      loadErrDirs();
    }
  }).catch(()=>{});
};
</script>
</body>
</html>'''


# ============================================================================
# 主入口
# ============================================================================
def main():
    ap = argparse.ArgumentParser(description='基桩动测仪 Web 界面')
    ap.add_argument('--port', type=int, default=5000, help='HTTP 端口 (默认 5000)')
    ap.add_argument('--host', default='0.0.0.0', help='监听地址')
    args = ap.parse_args()

    try:
        get_Wq()
        print(f"LDA 权重加载完毕: shape={Wq.shape}")
    except Exception as e:
        print(f"警告: 权重加载失败 ({e})")

    print(f"\n  基桩动测仪 Web 界面")
    print(f"  http://localhost:{args.port}")
    print(f"  按 Ctrl+C 停止\n")

    socketio.run(app, host=args.host, port=args.port, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
