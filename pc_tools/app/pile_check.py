#!/usr/bin/env python3
"""
pile_check.py — 基桩动测仪 PC端判别脚本
=========================================
基于 FPGA defect_ac_classifier.v 的同款算法 (LDA 自相关分类 + 800mm 二次判别)

用法:
  python pile_check.py --test          # 批量测试所有数据，打印统计
  python pile_check.py --file <CSV>    # 测试单个CSV文件
  python pile_check.py --serial COM3   # 从串口实时读取+判别+波形显示
  python pile_check.py --serial COM3 --no-plot  # 纯文本模式(无波形图)
  python pile_check.py --capture COM3  # 串口采集保存 (储存到God3.0/data/)
  python pile_check.py --help

算法:
  1. 512点帧内找 |x| 峰值位置 imp (主冲击)
  2. 有效性: peak<300000 或 imp>=112 或 imp+400>512 → INVALID
  3. s[i]=x[imp+16+i]>>sh, 整数自相关 r[k] k=0..135
  4. LDA 5类打分 → argmax → 类别 (good/d200/d307/d695/d800)
  5. 若判good但基波周期T0>=103 → 改判d800

性能 (9000有效样本, 偶训奇测, 窗口冲击检测):
  好棒判好: 95.8%  |  坏棒判坏: 96.1%  |  缺陷漏判为好的: 3.9%  |  定位正确: 80.6%
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # app/ -> pc_tools/

from capture.uart_frame_parser import run_serial

import os, sys, csv, json, struct, time, argparse
import numpy as np
from pathlib import Path
from datetime import datetime
from collections import defaultdict, Counter

# ============================================================================
# Constants
# ============================================================================
FS = 95880.0
SAMPLE_PERIOD_US = 10.42
NYLON_V_MS = 2200       # 纵波在尼龙棒中典型波速 m/s
SKIP = 16
SEGLEN = 384
NLAG = 136
LAG0 = 8
IMP_WIN = 112
MIN_PEAK = 300000
MARGIN_Q = 82
QW = 12
D800_LAG_LO, D800_LAG_HI, D800_T_TH = 90, 110, 103

CLS_NAMES = ["good", "d200", "d307", "d695", "d800"]
CLS_DIST = [0, 200, 307, 695, 800]

# 组号→类别映射 (与 fpga_model.py 一致)
GRP_TO_CLS = {
    5:0,6:0,7:0,8:0,      3:0,                       # good
    21:1,22:1,                                        # d200
    13:2,14:2,15:2,16:2,  1:2,                       # d307
    9:3,10:3,11:3,12:3,   4:3,                       # d695
    17:4,18:4,19:4,20:4,                              # d800
}

PROJ_ROOT = Path(__file__).parent.parent.parent  # app/ -> pc_tools/ -> project root
DATA_DIR = PROJ_ROOT / "God3.0" / "data"
WEIGHTS_PATH = PROJ_ROOT / "God3.0" / "rtl" / "defect_lda_weights.mem"


# ============================================================================
# Weight Loading
# ============================================================================
def load_weights():
    """Load Q12 LDA weights from .mem file."""
    with open(WEIGHTS_PATH) as f:
        lines = [l.strip() for l in f if l.strip()]
    Wq = np.zeros((129, 5), dtype=np.int16)
    for cls in range(5):
        for idx in range(129):
            addr = cls * 256 + idx
            v = int(lines[addr], 16)
            Wq[idx, cls] = v - 65536 if v > 32767 else v
    return Wq


# ============================================================================
# Data I/O
# ============================================================================
def load_csv(filepath):
    """Load waveform from CSV (output of UART capture tool)."""
    vals = []
    hdr = {}
    with open(filepath, encoding='utf-8-sig', errors='replace') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            if line.startswith('#'):
                p = line[1:].split(',', 1)
                if len(p) == 2: hdr[p[0].strip()] = p[1].strip()
                continue
            if line.startswith('index'): continue
            a = line.split(',')
            if len(a) >= 2:
                try: vals.append(int(a[1]))
                except: pass
    return np.array(vals, dtype=np.int64), hdr


def parse_uart_frame(data):
    """Parse binary UART frame (wave_uart_export.v format).
    Returns (waveform, n_points, fpga_results_dict) or None."""
    if len(data) < 7:
        return None, "too short"

    # Find sync
    idx = 0
    while idx < len(data) - 1:
        if data[idx] == 0xAA and data[idx+1] == 0x55:
            break
        idx += 1
    if idx >= len(data) - 1: return None, "no sync"
    data = data[idx:]

    if len(data) < 4: return None, "truncated"

    len_sel = data[2]
    n_pts = {0:64,1:128,2:256,3:512}.get(len_sel, 256)
    exp_len = 4 + 3*n_pts + 1
    if len(data) < exp_len: return None, f"short data: {len(data)}<{exp_len}"

    wf = np.zeros(n_pts, dtype=np.int64)
    pos = 3
    for i in range(n_pts):
        if pos+2 >= len(data): break
        v = (data[pos]<<16) | (data[pos+1]<<8) | data[pos+2]
        wf[i] = v - 0x1000000 if v & 0x800000 else v
        pos += 3

    # Optional result segment
    fpga = None
    if pos+13 <= len(data) and data[pos] == 0x5A:
        pos += 1
        ver = data[pos]; pos+=1; stat = data[pos]; pos+=1
        idxh = data[pos]; pos+=1
        ipl = data[pos]; pos+=1; dfl = data[pos]; pos+=1; btl = data[pos]; pos+=1
        dist = (data[pos]<<8)|data[pos+1]; pos+=2
        thr = (data[pos]<<16)|(data[pos+1]<<8)|data[pos+2]; pos+=3
        fpga = {
            'state': {0:'INVALID',1:'NORMAL',2:'DEFECT'}.get(stat&3,'?'),
            'confidence': {0:'NONE',1:'LOW',2:'HIGH'}.get((stat>>2)&3,'?'),
            'impact_index': ((idxh&1)<<8)|ipl,
            'defect_index': ((idxh&2)<<7)|dfl,
            'bottom_index': ((idxh&4)<<6)|btl,
            'distance_mm': dist, 'threshold': thr
        }

    return (wf, n_pts, fpga), None


# ============================================================================
# Core Algorithm (bit-accurate with fpga_model.py)
# ============================================================================
def fx_feature(waveform):
    """Extract autocorrelation features r[0..135]. Returns None if INVALID.
    冲击峰检测: 在帧前段[0,IMP_WIN)内搜索, 确保捕获初始锤击而非强缺陷反射。
    例如d800铁头桩的缺陷反射可达撞击峰1.5倍, 全局搜索会误判为DOUBLE_HIT。"""
    x = np.array(waveform[:512], dtype=np.int64) if len(waveform)>=512 else np.array(waveform, dtype=np.int64)
    n = len(x)
    ax = np.abs(x)
    search_end = min(IMP_WIN, n)
    peak = int(ax[:search_end].max())
    if peak < MIN_PEAK: return None, 'LOW_PEAK'
    imp = int(np.argmax(ax[:search_end]))
    if imp+SKIP+SEGLEN > n: return None, 'TOO_SHORT'

    seg = x[imp+SKIP:imp+SKIP+SEGLEN].astype(np.int64)
    sh = max(0, peak.bit_length()-15)
    s = seg >> sh
    r = np.zeros(NLAG, dtype=np.int64)
    for k in range(NLAG):
        r[k] = int(np.dot(s[:SEGLEN-k], s[k:]))
    if r[0] <= 0: return None, 'R0_ZERO'
    return r, peak, imp, sh


def fx_predict(Wq, r):
    """LDA prediction. Returns (pred_class, margin, rs0, scores list)."""
    r0 = int(r[0])
    rsh = max(0, r0.bit_length()-30)
    rs0 = r0 >> rsh
    scores = []
    for c in range(5):
        acc = 0
        for k in range(LAG0, NLAG):
            rk = int(r[k])
            rsk = rk>>rsh if rk>=0 else -((-rk)>>rsh)
            acc += int(Wq[k-LAG0, c]) * rsk
        acc += int(Wq[128, c]) * rs0
        scores.append(acc)
    order = np.argsort(scores)[::-1]
    return int(order[0]), scores[order[0]]-scores[order[1]], rs0, scores


def classify(Wq, waveform):
    """Full classification pipeline. Returns result dict."""
    rv = {'prediction':'INVALID', 'class_id': -1, 'class_name': 'INVALID',
          'distance_mm': 0, 'confidence': 'NONE', 'valid': False}

    ft = fx_feature(waveform)
    if ft is None or ft[0] is None:
        if ft and ft[1] == 'LOW_PEAK':
            rv['prediction'] = 'NOISE'
        return rv

    r, peak, imp, sh = ft
    rv['valid'] = True
    rv['peak'] = peak; rv['imp'] = imp

    pred, margin, rs0, scores = fx_predict(Wq, r)

    # 800mm secondary
    if pred == 0:
        T0 = int(np.argmax(r[90:111])) + 90
        rv['T0'] = T0
        if T0 >= D800_T_TH:
            pred = 4
            rv['d800_override'] = True

    rv['class_id'] = pred
    rv['class_name'] = CLS_NAMES[pred]
    rv['distance_mm'] = CLS_DIST[pred]
    rv['prediction'] = 'GOOD' if pred==0 else 'DEFECT'
    rv['confidence'] = 'LOW' if margin < MARGIN_Q*rs0 else 'HIGH'
    if rv.get('d800_override'): rv['confidence'] = 'LOW'
    rv['margin'] = margin; rv['scores'] = [int(s) for s in scores]
    return rv


# ============================================================================
# Batch Testing
# ============================================================================
def test_all(Wq, limit=None, only_new=True):
    """Test all CSV files, return (results_list, stats_dict)."""
    results = []
    stats = defaultdict(lambda: {'total':0,'correct':0,'wrong':0,'valid':0,'invalid':0,
                                  'tp':0,'fp':0,'tn':0,'fn':0,'pos_errors':[]})

    for dp in sorted(DATA_DIR.iterdir()):
        if not dp.is_dir(): continue
        try: gid = int(dp.name.split('_')[0])
        except: continue

        cls_id = GRP_TO_CLS.get(gid)
        if cls_id is None: continue

        # Skip old 256-point batch for new-only mode
        if only_new and gid <= 4: continue

        is_good = (cls_id == 0)
        true_dist = CLS_DIST[cls_id]
        gt_label = 'GOOD' if is_good else 'DEFECT'

        files = sorted(dp.glob('hit_*.csv'))
        if limit: files = files[:limit]

        for fp in files:
            try:
                wf, hdr = load_csv(fp)
                if len(wf) < 100: continue
            except: continue

            rv = classify(Wq, wf)

            entry = {
                'file': str(fp.relative_to(DATA_DIR)), 'group': gid,
                'true_cls': cls_id, 'pred_cls': rv['class_id'],
                'valid': rv['valid'], 'prediction': rv['prediction'],
                'confidence': rv['confidence'],
                'distance_mm': rv['distance_mm'],
                'd800_override': rv.get('d800_override', False),
            }

            stats[gt_label]['total'] += 1
            if not rv['valid']:
                stats[gt_label]['invalid'] += 1
                # INVALID on a real hit = wrong for good, acceptable for actual noise
                entry['correct'] = False
                stats[gt_label]['wrong'] += 1
            else:
                stats[gt_label]['valid'] += 1
                pred_is_good = (rv['class_id'] == 0)
                correct = (pred_is_good == is_good)
                entry['correct'] = correct

                if correct:
                    stats[gt_label]['correct'] += 1
                    if is_good: stats[gt_label]['tn'] += 1
                    else:
                        stats[gt_label]['tp'] += 1
                        if rv['class_id'] == cls_id:
                            stats[gt_label]['pos_errors'].append(0)
                        else:
                            stats[gt_label]['pos_errors'].append(
                                abs(rv['distance_mm']-true_dist))
                else:
                    stats[gt_label]['wrong'] += 1
                    if is_good: stats[gt_label]['fp'] += 1
                    else: stats[gt_label]['fn'] += 1

            results.append(entry)

    return results, stats


def print_report(results, stats):
    """Print formatted statistics."""
    print("\n" + "="*70)
    print("  桩完整性检测 - 测试报告")
    print("="*70)

    tv = sum(s['valid'] for s in stats.values())
    ti = sum(s['invalid'] for s in stats.values())
    tc = sum(s['correct'] for s in stats.values())
    tw = sum(s['wrong'] for s in stats.values())
    tt = tv + ti

    print(f"  总样本: {tt}  (有效: {tv} = {tv/max(tt,1)*100:.1f}%, "
          f"无效: {ti} = {ti/max(tt,1)*100:.1f}%)")
    print(f"  有效帧正确率: {tc}/{tv} = {tc/max(tv,1)*100:.1f}%")

    # GOOD stats
    gs = stats['GOOD']
    print(f"\n  GOOD (好桩):")
    print(f"    {gs['total']} 文件, 有效 {gs['valid']}, 无效 {gs['invalid']}")
    if gs['valid'] > 0:
        print(f"    判好(TN): {gs['tn']}/{gs['valid']} = {gs['tn']/gs['valid']*100:.1f}%")
        print(f"    误判为坏(FP): {gs['fp']}/{gs['valid']} = {gs['fp']/max(gs['valid'],1)*100:.1f}%")

    # DEFECT stats
    ds = stats['DEFECT']
    print(f"\n  DEFECT (坏桩):")
    print(f"    {ds['total']} 文件, 有效 {ds['valid']}, 无效 {ds['invalid']}")
    if ds['valid'] > 0:
        print(f"    检坏(TP): {ds['tp']}/{ds['valid']} = {ds['tp']/ds['valid']*100:.1f}%")
        print(f"    漏判(FN): {ds['fn']}/{ds['valid']} = {ds['fn']/max(ds['valid'],1)*100:.1f}%")
        pe = np.array(ds['pos_errors']) if ds['pos_errors'] else np.array([0])
        print(f"    定位中位误差: {np.median(pe):.0f}mm  |  平均: {pe.mean():.0f}mm")

    # 5-class confusion matrix (valid only)
    valid_results = [r for r in results if r['valid']]
    conf = np.zeros((5,5), dtype=int)
    for r in valid_results:
        if 0 <= r['true_cls'] < 5 and 0 <= r['pred_cls'] < 5:
            conf[r['true_cls'], r['pred_cls']] += 1

    print(f"\n  混淆矩阵 (真\\判)  n={len(valid_results)}:")
    print(f"  {'':>8}", end="")
    for n in CLS_NAMES: print(f"{n:>8}", end="")
    print()
    for i, n in enumerate(CLS_NAMES):
        print(f"  {n:>8}", end="")
        for j in range(5): print(f"{conf[i,j]:>8}", end="")
        print()

    # Per-class metrics
    print(f"\n  {'类':<8}{'精确率':<10}{'召回率':<10}{'F1':<10}")
    for i, n in enumerate(CLS_NAMES):
        tp = conf[i,i]; pp = conf[:,i].sum(); ap = conf[i,:].sum()
        prec = tp/max(pp,1); rec = tp/max(ap,1)
        f1 = 2*prec*rec/max(prec+rec,1e-10)
        print(f"  {n:<8}{prec:<10.3f}{rec:<10.3f}{f1:<10.3f}")

    # 800mm override stats
    d8 = [r for r in results if r.get('d800_override')]
    if d8:
        print(f"\n  800mm二次判别: 触发 {len(d8)} 次")


# ============================================================================
# Waveform Plot (matplotlib, 串口模式可选开启)
# ============================================================================
_WF_PLOT = None  # 全局绘图状态

def ensure_matplotlib():
    """确保 matplotlib 已安装，否则提示并返回 False"""
    try:
        import matplotlib
        # 修复中文乱码 (必须在创建 figure 前设置)
        import matplotlib.pyplot as plt
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun']
        plt.rcParams['axes.unicode_minus'] = False
        return True
    except ImportError:
        print("\n⚠ matplotlib 未安装，无法显示波形图。")
        print("  安装: pip install matplotlib")
        print("  或使用 --no-plot 模式 (纯文本判别)\n")
        return False

def setup_waveform_plot():
    """初始化波形显示窗口 + 测距按钮。返回 (fig, ax, line, markers_dict)"""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, TextBox

    # 修复中文乱码
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun']
    plt.rcParams['axes.unicode_minus'] = False

    plt.ion()
    fig, ax = plt.subplots(figsize=(14, 6))
    plt.subplots_adjust(bottom=0.13)  # 给按钮留空间

    ax.set_xlabel("Sample Index")
    ax.set_ylabel("ADC Value (24-bit signed)")
    ax.set_title("Pile Check — 等待波形...  |  测距: 点'测距'→左键单击两点→显示距离")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='gray', linewidth=0.5)
    line, = ax.plot([], [], 'b-', linewidth=0.8)
    markers = {
        'impact': ax.axvline(x=0, color='r', linestyle='--', linewidth=1.0, alpha=0.8, label='Impact'),
        'defect': ax.axvline(x=0, color='orange', linestyle='--', linewidth=1.0, alpha=0.8, label='Defect'),
        'bottom': ax.axvline(x=0, color='green', linestyle='--', linewidth=1.0, alpha=0.8, label='Bottom'),
    }
    ax.legend(loc='upper right')
    ax.set_xlim(0, 512)

    # ── 测距功能 ──
    measure = {
        'active': False,
        'point_a': None,    # (idx, val) or None
        'point_b': None,
        'marker_a': None,   # axvline artist
        'marker_b': None,
        'text': None,       # text artist (axes coords)
        'velocity': NYLON_V_MS,  # 当前波速 m/s
    }

    def _calc_and_display():
        """根据当前选点和波速, 计算并更新距离显示。返回 None 如果选点不全。"""
        if measure['point_a'] is None or measure['point_b'] is None:
            return
        idx_a = measure['point_a'][0]
        idx_b = measure['point_b'][0]
        d_idx = abs(idx_b - idx_a)
        v = measure['velocity']
        dt_sec = d_idx * SAMPLE_PERIOD_US * 1e-6
        dist_m = v * dt_sec / 2
        dist_mm = dist_m * 1000

        if measure['text']:
            measure['text'].remove()
        measure['text'] = ax.text(
            0.02, 0.92,
            f"A={idx_a}  B={idx_b}  Δ={d_idx}样点  Δt={dt_sec*1e6:.1f}μs  "
            f"距离≈{dist_mm:.0f}mm  (v={v}m/s, ÷2往返)",
            transform=ax.transAxes, fontsize=9,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9, ec='gray')
        )
        ax.figure.canvas.draw_idle()

    def _on_click(event):
        """鼠标点击: 测距模式下选点"""
        if not measure['active']:
            return
        if event.inaxes != ax:
            return
        if event.xdata is None:
            return
        idx = int(round(event.xdata))

        if measure['point_a'] is None:
            measure['point_a'] = (idx, event.ydata)
            if measure['marker_a']:
                measure['marker_a'].remove()
            measure['marker_a'] = ax.axvline(
                x=idx, color='cyan', linestyle=':', linewidth=1.5, alpha=0.9)
            ax.figure.canvas.draw_idle()
        else:
            measure['point_b'] = (idx, event.ydata)
            if measure['marker_b']:
                measure['marker_b'].remove()
            measure['marker_b'] = ax.axvline(
                x=idx, color='magenta', linestyle=':', linewidth=1.5, alpha=0.9)
            measure['active'] = False  # 选完两点自动退出测距模式
            _calc_and_display()

    def _on_velocity_change(text):
        """波速 TextBox 回车回调: 解析新波速, 实时更新距离"""
        try:
            v = float(text.strip())
            if v > 0:
                measure['velocity'] = v
                # 已有选点时自动重算距离
                if measure['point_a'] is not None and measure['point_b'] is not None:
                    _calc_and_display()
        except ValueError:
            pass  # 非法输入忽略, 保持原值

    def _on_measure_click(event):
        """测距按钮: 清除旧选点, 进入测距模式"""
        measure['active'] = True
        measure['point_a'] = None
        measure['point_b'] = None
        for k in ('marker_a', 'marker_b', 'text'):
            if measure[k]:
                measure[k].remove()
                measure[k] = None
        ax.set_title(ax.get_title().replace(
            "| 测距中: 点击第一个点...", "").replace(
            "| 测距中: 点击第二个点...", "") + " | 测距中: 点击第一个点...")
        ax.figure.canvas.draw_idle()

    def _on_reset_click(event):
        """复位按钮: 等同于重新开始测距"""
        _on_measure_click(event)

    # ── 底部控件: 波速输入 + 测距/复位按钮 ──
    # 波速标签
    ax_vlabel = plt.axes([0.36, 0.02, 0.10, 0.04])
    ax_vlabel.set_axis_off()
    vlabel = ax_vlabel.text(0.5, 0.5, '波速:', transform=ax_vlabel.transAxes,
                            fontsize=9, ha='center', va='center')
    # 波速输入框
    ax_vbox = plt.axes([0.46, 0.025, 0.08, 0.035])
    v_textbox = TextBox(ax_vbox, '', initial=str(NYLON_V_MS))
    v_textbox.set_val(str(NYLON_V_MS))
    v_textbox.on_submit(_on_velocity_change)
    # m/s 单位标签
    ax_unit = plt.axes([0.54, 0.02, 0.05, 0.04])
    ax_unit.set_axis_off()
    ulabel = ax_unit.text(0, 0.5, 'm/s', transform=ax_unit.transAxes,
                          fontsize=9, va='center')

    # 测距/复位按钮
    ax_btn_m = plt.axes([0.70, 0.02, 0.08, 0.04])
    ax_btn_r = plt.axes([0.79, 0.02, 0.08, 0.04])
    btn_measure = Button(ax_btn_m, '测距')
    btn_reset = Button(ax_btn_r, '复位')
    btn_measure.on_clicked(_on_measure_click)
    btn_reset.on_clicked(_on_reset_click)

    fig.canvas.mpl_connect('button_press_event', _on_click)

    # 把控件引用挂在 fig 上, 防止被 GC
    fig._pile_widgets = (btn_measure, btn_reset, v_textbox)
    fig._pile_measure = measure

    return fig, ax, line, markers

def update_waveform_plot(fig, ax, line, markers, waveform, fpga_result, pc_result, frame_count):
    """用最新帧更新波形图"""
    import matplotlib.pyplot as plt
    n = len(waveform)
    x = list(range(n))
    line.set_data(x, waveform[:n])
    ax.set_xlim(0, n)

    # Y 轴自适应
    peak = max(abs(min(waveform)), abs(max(waveform))) or 1
    ax.set_ylim(-peak * 1.15, peak * 1.15)

    # 清除旧标记位置
    markers['impact'].set_xdata([0])
    markers['defect'].set_xdata([0])
    markers['bottom'].set_xdata([0])

    # FPGA 标记线
    if fpga_result:
        markers['impact'].set_xdata([fpga_result.get('impact_index', 0)])
        markers['defect'].set_xdata([fpga_result.get('defect_index', 0)])
        markers['bottom'].set_xdata([fpga_result.get('bottom_index', 0)])

    # 标题: PC 判别结果 + FPGA 对比
    pc_label = f"{pc_result['prediction']}/{pc_result['class_name']} dist={pc_result['distance_mm']}mm"
    if pc_result['valid']:
        pc_label += f" conf={pc_result['confidence']}"
    if pc_result.get('T0'):
        pc_label += f" T0={pc_result['T0']}"

    fpga_label = ""
    if fpga_result:
        fpga_label = (f" | FPGA: {fpga_result['state']} "
                      f"dist={fpga_result['distance_mm']}mm "
                      f"conf={fpga_result['confidence']}")
        # 一致性
        pc_is_good = pc_result['prediction'] == 'GOOD'
        fpga_is_normal = fpga_result['state'] == 'NORMAL'
        if (pc_is_good and fpga_is_normal) or (not pc_is_good and fpga_result['state'] == 'DEFECT'):
            fpga_label += " ✓"
        elif pc_result['prediction'] == 'INVALID' and fpga_result['state'] == 'INVALID':
            fpga_label += " ✓"
        else:
            fpga_label += " ✗"

    title = f"[#{frame_count}] {n}pt | PC: {pc_label}{fpga_label}"

    # 追加测距状态提示
    m = getattr(fig, '_pile_measure', None)
    if m and m['active']:
        if m['point_a'] is None:
            title += " | 测距中: 点击第一个点..."
        else:
            title += " | 测距中: 点击第二个点..."

    ax.set_title(title, fontsize=9)
    plt.pause(0.01)


# ============================================================================
# Serial Mode
# ============================================================================
def serial_mode(Wq, port, baud=115200, show_plot=False):
    """Real-time classification from serial port.
    show_plot=True: 同时显示 matplotlib 波形图 (需 pip install matplotlib)"""
    import serial as pyserial
    import serial.tools.list_ports

    # 智能处理串口号: "20" → "COM20", "com20" → "COM20"
    port = port.strip()
    if port.isdigit():
        port = f"COM{port}"
    elif port.upper().startswith("COM") and port[3:].isdigit():
        port = port.upper()

    # 列出可用串口
    print("\n可用串口:")
    available = list(pyserial.tools.list_ports.comports())
    if available:
        for p in available:
            print(f"  {p.device} - {p.description}")
    else:
        print("  (未找到任何串口)")

    # Windows COM≥10 需要 \\.\ 前缀
    try:
        ser = pyserial.Serial(port, baud, timeout=0.01 if show_plot else 2)
    except Exception:
        if port.upper().startswith("COM"):
            long_port = f"\\\\.\\{port}"
            try:
                ser = pyserial.Serial(long_port, baud, timeout=0.01 if show_plot else 2)
                port = long_port
            except Exception as e2:
                print(f"\n无法打开串口 {port}: {e2}")
                print("请检查串口号是否正确, 或是否被其他程序占用")
                return
        else:
            print(f"\n无法打开串口 {port}")
            return

    # ── 波形显示初始化 ──
    wf_plot = None
    if show_plot:
        if ensure_matplotlib():
            import matplotlib.pyplot as plt
            fig, ax, line, markers = setup_waveform_plot()
            plt.show(block=False)
            plt.pause(0.2)
            wf_plot = (fig, ax, line, markers)
        else:
            show_plot = False  # 降级为纯文本模式

    plot_mode = "波形图 + 文本" if show_plot else "纯文本"
    print(f"监听 {port} @ {baud}... 模式: {plot_mode}  (Ctrl+C 停止)")

    frame_count = 0
    buf = b""
    try:
        while True:
            if show_plot and wf_plot:
                fig = wf_plot[0]
                if not plt.fignum_exists(fig.number):
                    print("\n波形窗口已关闭, 停止监听.")
                    break
                chunk = ser.read(4096)
            else:
                chunk = ser.read(1024)

            if chunk:
                buf += chunk
                parsed, err = parse_uart_frame(buf)
                if parsed is not None:
                    buf = buf[buf.find(b'\xAA\x55')+1:]
                    wf, npts, fpga = parsed
                    rv = classify(Wq, wf)
                    frame_count += 1

                    # ── 文本输出 ──
                    print(f"\n--- [{frame_count}] {time.strftime('%H:%M:%S')} | {npts}点 ---")
                    print(f"  PC判别:  {rv['prediction']:<8} {rv['class_name']:<6} "
                          f"dist={rv['distance_mm']:>4}mm  conf={rv['confidence']}")
                    if fpga:
                        print(f"  FPGA判:  {fpga['state']:<8} "
                              f"dist={fpga['distance_mm']}mm  conf={fpga['confidence']}")
                        match = ('✓' if (rv['prediction']=='GOOD' and fpga['state']=='NORMAL')
                                       or (rv['prediction']=='DEFECT' and fpga['state']=='DEFECT')
                                       or (rv['prediction']=='INVALID' and fpga['state']=='INVALID')
                                       else '✗ 不一致!')
                        print(f"  比对: {match}")

                    # ── 波形图更新 ──
                    if wf_plot:
                        fig, ax, line, markers = wf_plot
                        update_waveform_plot(fig, ax, line, markers, wf, fpga, rv, frame_count)

                elif len(buf) > 8192:
                    buf = buf[-4096:]

            # 驱动 GUI 事件循环
            if show_plot and wf_plot:
                import matplotlib.pyplot as plt
                plt.pause(0.01)

    except KeyboardInterrupt:
        print("\n停止")
    finally:
        ser.close()
        if wf_plot:
            import matplotlib.pyplot as plt
            plt.close(wf_plot[0])
            print(f"共接收 {frame_count} 帧")


# ============================================================================
# Capture Mode — 调用 uart_frame_parser.run_serial() + PC判别 + 可选波形
# ============================================================================
def capture_mode(Wq, port, baud=115200, show_plot=False,
                 count=None, note=None, defect_mm=None):
    """采集模式: 调用现有 run_serial() 保存CSV, 注入 on_frame 回调做 PC判别+波形。
    参数 count/note/defect_mm 若为 None 则交互提问 (CLI可预填)。"""
    import serial.tools.list_ports

    # ── 串口号规范化 ──
    port = port.strip()
    if port.isdigit():
        port = f"COM{port}"
    elif port.upper().startswith("COM") and port[3:].isdigit():
        port = port.upper()

    # 列出可用串口
    print("\n可用串口:")
    available = list(serial.tools.list_ports.comports())
    if available:
        for p in available:
            print(f"  {p.device} - {p.description}")
    else:
        print("  (未找到任何串口)")

    # ── 交互提问 (仅当参数未通过CLI预填时) ──
    if note is None:
        note = input("\n本组说明 (如: 缺陷棒800mm 中敲): ").strip()
    if count is None:
        c_str = input("计划敲几次? (回车=不限, Ctrl+C停止): ").strip()
        count = int(c_str) if c_str else None
    if defect_mm is None:
        d_str = input("缺陷真实距离 mm? (正常棒回车跳过): ").strip()
        defect_mm = int(d_str) if d_str else None

    truth = {"rod_len": None, "defect_mm": defect_mm}

    # ── 创建输出目录 ──
    def _sanitize(text, maxlen=40):
        bad = '\\/:*?"<>| '
        return "".join(("_" if c in bad else c) for c in text.strip())[:maxlen] or "unnamed"

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    seq = 1 + sum(1 for d in DATA_DIR.iterdir() if d.is_dir())
    sess = f"{seq:02d}_{datetime.now().strftime('%H%M%S')}_{_sanitize(note)}"
    outdir = DATA_DIR / sess
    outdir.mkdir(parents=True)
    print(f"\n数据目录: {outdir}")

    # ── 波形显示初始化 ──
    wf_plot = None
    if show_plot:
        if ensure_matplotlib():
            import matplotlib.pyplot as plt
            fig, ax, line, markers = setup_waveform_plot()
            plt.show(block=False)
            plt.pause(0.2)
            wf_plot = (fig, ax, line, markers)
        else:
            show_plot = False

    plot_mode = "波形图 + 保存" if show_plot else "纯文本 + 保存"
    print(f"模式: {plot_mode}")

    # ── 回调: 每收到一帧 → PC判别 + 波形更新 ──
    def on_frame(frame, hit):
        wf = np.array(frame["samples"], dtype=np.int64)
        rv = classify(Wq, wf)

        # FPGA 状态
        fpga_state = {0:"INVALID",1:"NORMAL",2:"DEFECT"}.get(frame["state"],"?")
        fpga_conf = {0:"NONE",1:"LOW",2:"HIGH"}.get(frame["confidence"],"?")

        # 文本输出
        print(f"  PC判别:  {rv['prediction']:<8} {rv['class_name']:<6} "
              f"dist={rv['distance_mm']:>4}mm  conf={rv['confidence']}")
        match = ('✓' if (rv['prediction']=='GOOD' and fpga_state=='NORMAL')
                       or (rv['prediction']=='DEFECT' and fpga_state=='DEFECT')
                       or (rv['prediction']=='INVALID' and fpga_state=='INVALID')
                       else '✗ 不一致!')
        print(f"  比对: {match}")

        # 波形图更新
        if wf_plot:
            fig, ax, line, markers = wf_plot
            fpga_info = {
                'state': fpga_state,
                'confidence': fpga_conf,
                'impact_index': frame['impact_index'],
                'defect_index': frame['defect_index'],
                'bottom_index': frame['bottom_index'],
                'distance_mm': frame['distance_mm'],
            }
            update_waveform_plot(fig, ax, line, markers, wf, fpga_info, rv, hit)
            import matplotlib.pyplot as plt
            plt.pause(0.01)

        return True  # 继续采集

    # ── 回调: 检查波形窗口是否关闭 ──
    def check_continue():
        if wf_plot:
            import matplotlib.pyplot as plt
            return plt.fignum_exists(wf_plot[0].number)
        return True

    # ── 调用现有 run_serial() ──
    run_serial(port, baud, str(outdir), note, truth, count,
               on_frame=on_frame, check_continue=check_continue)

    if wf_plot:
        import matplotlib.pyplot as plt
        plt.close(wf_plot[0])


# ============================================================================
def interactive_menu(Wq):
    """交互式菜单，双击运行时不闪退。"""
    while True:
        print("\n" + "=" * 50)
        print("  基桩动测仪 - PC 判别工具")
        print("=" * 50)
        print("  [1] 批量测试 (快速, 每目录100文件)")
        print("  [2] 全量测试 (慢, 所有文件)")
        print("  [3] 测试单个 CSV 文件")
        print("  [4] 串口实时监听 (判别+波形)")
        print("  [5] 串口采集保存 (储存到data/)")
        print("  [6] 显示帮助")
        print("  [0] 退出")
        print("-" * 50)
        choice = input("  请选择 [1]: ").strip() or "1"

        if choice == "1":
            limit = input("  每目录文件数 [100]: ").strip()
            limit = int(limit) if limit else 100
            print("\n测试中...")
            res, st = test_all(Wq, limit=limit, only_new=True)
            print_report(res, st)

        elif choice == "2":
            print("\n全量测试中 (较慢, 请耐心等待)...")
            res, st = test_all(Wq, limit=None, only_new=True)
            print_report(res, st)

        elif choice == "3":
            path = input("  CSV文件路径: ").strip()
            if not path:
                print("  未输入路径")
                continue
            fp = Path(path)
            if not fp.exists():
                print(f"  文件不存在: {fp}")
                continue
            wf, hdr = load_csv(fp)
            rv = classify(Wq, wf)
            print(f"\n  文件: {fp}")
            print(f"  点数: {len(wf)}")
            print(f"  有效: {'是' if rv['valid'] else '否'}")
            print(f"  判别: {rv['prediction']}")
            print(f"  类别: {rv['class_name']} (id={rv['class_id']})")
            print(f"  距离: {rv['distance_mm']} mm")
            print(f"  置信: {rv['confidence']}")
            if rv['valid']:
                print(f"  冲击峰: {rv['peak']} @ idx={rv['imp']}")
                print(f"  打分: {rv.get('scores', [])}")
                if rv.get('T0'): print(f"  T0: {rv['T0']}")

        elif choice == "4":
            # 先列出可用串口
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    print("\n  可用串口:")
                    for i, p in enumerate(ports):
                        print(f"    [{i}] {p.device} - {p.description}")
                    sel = input("  选择序号或直接输入串口号 (如 COM20): ").strip()
                    if sel.isdigit() and int(sel) < len(ports):
                        port = ports[int(sel)].device
                    elif sel:
                        port = sel
                    else:
                        port = ports[0].device if ports else "COM3"
                else:
                    port = input("  未检测到串口, 请手动输入串口号 (如 COM20): ").strip()
                    if not port:
                        print("  未输入串口号")
                        continue
            except ImportError:
                port = input("  串口号 [COM3]: ").strip() or "COM3"

            baud_str = input("  波特率 [115200]: ").strip()
            baud = int(baud_str) if baud_str else 115200

            # 是否启用波形图
            show_plot = False
            plot_choice = input("  显示波形图? (y=是/n=否, 需matplotlib) [y]: ").strip().lower()
            if plot_choice in ('', 'y', 'yes', '是'):
                show_plot = True

            serial_mode(Wq, port, baud, show_plot=show_plot)

        elif choice == "5":
            # 采集保存模式 — 复用串口选择逻辑
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    print("\n  可用串口:")
                    for i, p in enumerate(ports):
                        print(f"    [{i}] {p.device} - {p.description}")
                    sel = input("  选择序号或直接输入串口号 (如 COM20): ").strip()
                    if sel.isdigit() and int(sel) < len(ports):
                        port = ports[int(sel)].device
                    elif sel:
                        port = sel
                    else:
                        port = ports[0].device if ports else "COM3"
                else:
                    port = input("  未检测到串口, 请手动输入串口号 (如 COM20): ").strip()
                    if not port:
                        print("  未输入串口号")
                        continue
            except ImportError:
                port = input("  串口号 [COM3]: ").strip() or "COM3"

            baud_str = input("  波特率 [115200]: ").strip()
            baud = int(baud_str) if baud_str else 115200

            show_plot = False
            plot_choice = input("  显示波形图? (y=是/n=否, 需matplotlib) [y]: ").strip().lower()
            if plot_choice in ('', 'y', 'yes', '是'):
                show_plot = True

            capture_mode(Wq, port, baud, show_plot=show_plot)

        elif choice == "6":
            print("""
  用法说明:
    python pile_check.py                  → 交互菜单 (当前)
    python pile_check.py --test           → 命令行批量测试
    python pile_check.py --file xxx       → 测试单个文件
    python pile_check.py --serial COM3    → 串口监听 (判别+波形)
    python pile_check.py --capture COM3   → 串口采集保存到data/

  采集模式:
    接收FPGA串口数据, 保存为CSV到 God3.0/data/<序号>_时间_说明/
    可选设置敲击次数, 收够自动停止; 不限次数则Ctrl+C手动结束.
            """)

        elif choice == "0":
            print("  再见!")
            break

        else:
            print(f"  无效选项: {choice}")

        input("\n  按回车键继续...")


def main():
    ap = argparse.ArgumentParser(description='基桩动测仪 PC端判别')
    ap.add_argument('--test', action='store_true', help='批量测试所有数据')
    ap.add_argument('--file', type=str, help='测试单个CSV文件')
    ap.add_argument('--serial', type=str, help='串口设备 (如 COM3)')
    ap.add_argument('--capture', type=str, help='串口采集保存模式 (如 COM20)')
    ap.add_argument('--count', type=int, default=None, help='采集模式下计划敲击次数')
    ap.add_argument('--note', type=str, default=None, help='采集模式下本组说明')
    ap.add_argument('--defect-mm', type=int, default=None, help='缺陷真实距离 mm')
    ap.add_argument('--baud', type=int, default=115200, help='波特率')
    ap.add_argument('--no-plot', action='store_true', help='串口模式禁用波形图 (纯文本)')
    ap.add_argument('--limit', type=int, default=None, help='每目录最多测试文件数')
    ap.add_argument('--old', action='store_true', help='包含旧批次256点数据')
    ap.add_argument('--output', type=str, help='JSON结果输出路径')
    args = ap.parse_args()

    # 检查权重文件
    if not WEIGHTS_PATH.exists():
        print(f"错误: 权重文件不存在 {WEIGHTS_PATH}")
        print("请先运行: cd God3.0 && python tools/fpga_model.py")
        try:
            input("按回车键退出...")
        except EOFError:
            pass
        sys.exit(1)

    Wq = load_weights()
    print(f"LDA权重加载完毕: shape={Wq.shape}")

    # 有命令行参数时走命令行模式
    has_args = args.test or args.file or args.serial or args.capture

    if args.test:
        print("批量测试中...")
        res, st = test_all(Wq, limit=args.limit, only_new=not args.old)
        print_report(res, st)
        if args.output:
            with open(args.output, 'w') as f:
                json.dump({k:{kk:vv for kk,vv in v.items() if kk!='pos_errors'}
                           for k,v in st.items()}, f, indent=2)
            print(f"结果已保存: {args.output}")

    elif args.file:
        fp = Path(args.file)
        if not fp.exists():
            print(f"文件不存在: {fp}")
            try:
                input("按回车键退出...")
            except EOFError:
                pass
            sys.exit(1)
        wf, hdr = load_csv(fp)
        rv = classify(Wq, wf)
        print(f"\n文件: {fp}")
        print(f"点数: {len(wf)}")
        print(f"有效: {'是' if rv['valid'] else '否'}")
        print(f"判别: {rv['prediction']}")
        print(f"类别: {rv['class_name']} (id={rv['class_id']})")
        print(f"距离: {rv['distance_mm']} mm")
        print(f"置信: {rv['confidence']}")
        if rv['valid']:
            print(f"冲击峰: {rv['peak']} @ idx={rv['imp']}")
            print(f"打分: {rv.get('scores', [])}")
            print(f"Margin: {rv.get('margin', 0)}")
            if rv.get('T0'): print(f"T0: {rv['T0']}")

    elif args.serial:
        serial_mode(Wq, args.serial, args.baud, show_plot=not args.no_plot)

    elif args.capture:
        capture_mode(Wq, args.capture, args.baud,
                     show_plot=not args.no_plot,
                     count=args.count, note=args.note,
                     defect_mm=args.defect_mm)

    elif not has_args:
        # 无参数 → 交互菜单
        interactive_menu(Wq)

    else:
        ap.print_help()

    # 命令行模式下也暂停，防止闪退 (仅交互终端下有效)
    if has_args:
        try:
            input("\n按回车键退出...")
        except EOFError:
            pass


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"\n程序出错: {e}")
        import traceback
        traceback.print_exc()
        try:
            input("\n按回车键退出...")
        except EOFError:
            pass
