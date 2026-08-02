#!/usr/bin/env python3
"""
基桩动测仪 - 桩完整性检测算法分析与验证
============================================
采样周期: 10.42us/点
纵波在尼龙棒中速度: ~2000-2600 m/s
桩长: ~1000mm

方法:
1. 锤击后采集256点波形
2. 自相关分析找基波周期 → 判断好/坏
3. 对坏桩: 找缺陷反射峰 → 计算缺陷位置

Author: AI Analysis
Date: 2026-07-30
"""

import os
import sys
import csv
import re
import json
from collections import defaultdict
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# Try importing optional libraries
try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False
    print("WARNING: numpy not available, using pure Python (slow)")

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except ImportError:
    HAVE_PLOT = False
    print("WARNING: matplotlib not available, skipping plots")

# ============================================================================
# Constants
# ============================================================================
SAMPLE_PERIOD_US = 10.42  # us per sample
NYLON_V_MIN = 2000  # m/s
NYLON_V_MAX = 2600  # m/s
NYLON_V_TYPICAL = 2200  # m/s typical

# Data directory
DATA_DIR = Path(r"D:\FPGA\dian_sai\Project\God3.11\God3.0\data")

# Output directory for plots
PLOT_DIR = Path(r"D:\FPGA\dian_sai\Project\God3.11\analysis_plots")
PLOT_DIR.mkdir(exist_ok=True)

# ============================================================================
# Data Loading
# ============================================================================
def parse_csv_header(filepath):
    """Parse the header metadata from a CSV file."""
    info = {
        'note': '',
        'rod_len_mm': None,
        'defect_true_mm': 0,
        'err_mm': None,
        'points': 256,
        'state': 'UNKNOWN',
        'confidence': 'UNKNOWN',
        'impact_index': 0,
        'defect_index': 0,
        'bottom_index': 0,
        'distance_mm': 0,
        'threshold': 0,
        'peak_abs': 0,
    }
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        for line in f:
            line = line.strip()
            if line.startswith('# note,'):
                info['note'] = line.replace('# note,', '').strip()
            elif line.startswith('# rod_len_mm,'):
                val = line.replace('# rod_len_mm,', '').strip()
                info['rod_len_mm'] = int(val) if val else None
            elif line.startswith('# defect_true_mm,'):
                val = line.replace('# defect_true_mm,', '').strip()
                info['defect_true_mm'] = int(val) if val else 0
            elif line.startswith('# err_mm,'):
                val = line.replace('# err_mm,', '').strip()
                info['err_mm'] = int(val) if val else None
            elif line.startswith('# points,'):
                val = line.replace('# points,', '').strip()
                info['points'] = int(val) if val else 256
            elif line.startswith('# state,'):
                info['state'] = line.replace('# state,', '').strip()
            elif line.startswith('# confidence,'):
                info['confidence'] = line.replace('# confidence,', '').strip()
            elif line.startswith('# impact_index,'):
                val = line.replace('# impact_index,', '').strip()
                info['impact_index'] = int(val) if val else 0
            elif line.startswith('# defect_index,'):
                val = line.replace('# defect_index,', '').strip()
                info['defect_index'] = int(val) if val else 0
            elif line.startswith('# bottom_index,'):
                val = line.replace('# bottom_index,', '').strip()
                info['bottom_index'] = int(val) if val else 0
            elif line.startswith('# distance_mm,'):
                val = line.replace('# distance_mm,', '').strip()
                info['distance_mm'] = int(val) if val else 0
            elif line.startswith('# threshold,'):
                val = line.replace('# threshold,', '').strip()
                info['threshold'] = int(val) if val else 0
            elif line.startswith('# peak_abs,'):
                val = line.replace('# peak_abs,', '').strip()
                info['peak_abs'] = int(val) if val else 0
            elif line.startswith('index,raw24'):
                break
    return info


def load_csv_waveform(filepath):
    """Load only the waveform data (index, raw24) from a CSV file."""
    data = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        in_data = False
        for line in f:
            line = line.strip()
            if line == 'index,raw24':
                in_data = True
                continue
            if in_data and line:
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        data.append(int(parts[1]))
                    except ValueError:
                        continue
    return data


def get_category_from_dirname(dirname):
    """Extract category info from directory name."""
    # Format: XX_HHMMSS_category_details
    parts = dirname.split('_', 2)
    if len(parts) >= 3:
        return parts[2]
    return dirname


def classify_ground_truth(dirname):
    """Determine ground truth from directory name."""
    d = dirname.lower()
    if '正常' in d or '好棒' in d:
        return 'GOOD', 0
    elif '环境干扰' in d or '未敲' in d or '踩地面' in d:
        return 'NOISE', 0
    elif '缺陷' in d or '坏棒' in d or '坏帮' in d:
        # Try to extract defect position
        # Pattern: _距离_ or just number at end
        # Check for common patterns
        if '305' in d or '307' in d:
            return 'DEFECT', 305 if '305' in d else 307
        elif '695' in d:
            return 'DEFECT', 695
        elif '800' in d:
            return 'DEFECT', 800
        elif '200' in d:
            return 'DEFECT', 200
        else:
            # Try to find defect position in the name
            match = re.search(r'_(\d{3})(?:mm|$)', d)
            if match:
                return 'DEFECT', int(match.group(1))
            return 'DEFECT', None
    return 'UNKNOWN', 0


# ============================================================================
# Signal Processing (Pure Python fallback)
# ============================================================================
if HAVE_NUMPY:
    def compute_autocorrelation(signal, max_lag=None):
        """Compute normalized autocorrelation of a signal (numpy)."""
        n = len(signal)
        if max_lag is None:
            max_lag = n
        signal = np.array(signal, dtype=float)
        signal = signal - np.mean(signal)
        result = np.correlate(signal, signal, mode='full')
        result = result[n-1:n-1+max_lag]
        # Normalize by r[0]
        if result[0] != 0:
            result = result / result[0]
        return result

    def compute_envelope(signal, window=4):
        """Compute moving average envelope of signal."""
        sig = np.abs(np.array(signal, dtype=float))
        kernel = np.ones(window) / window
        return np.convolve(sig, kernel, mode='same')

    def find_peaks(signal, min_height=None, min_distance=5, max_peaks=10):
        """Find peaks in signal (simple implementation)."""
        sig = np.array(signal, dtype=float)
        n = len(sig)
        peaks = []
        if min_height is None:
            min_height = np.mean(sig) + 0.1 * (np.max(sig) - np.mean(sig))

        for i in range(1, n - 1):
            if sig[i] > sig[i-1] and sig[i] > sig[i+1] and sig[i] >= min_height:
                if not peaks or (i - peaks[-1][0]) >= min_distance:
                    peaks.append((i, sig[i]))
                elif sig[i] > peaks[-1][1]:
                    peaks[-1] = (i, sig[i])

        # Sort by amplitude, return top N
        peaks.sort(key=lambda x: x[1], reverse=True)
        peaks = peaks[:max_peaks]
        # Sort by position
        peaks.sort(key=lambda x: x[0])
        return peaks

    def parabolic_interp(signal, peak_idx):
        """Parabolic interpolation for sub-sample peak position."""
        if peak_idx <= 0 or peak_idx >= len(signal) - 1:
            return peak_idx, signal[peak_idx]
        y0, y1, y2 = signal[peak_idx-1], signal[peak_idx], signal[peak_idx+1]
        denom = y0 - 2*y1 + y2
        if abs(denom) < 1e-10:
            return peak_idx, y1
        offset = (y0 - y2) / (2 * denom)
        peak_pos = peak_idx + offset
        peak_val = y1 - (y0 - y2) * offset / 4
        return peak_pos, peak_val
else:
    # Pure Python fallbacks - not as optimized but functional
    def compute_autocorrelation(signal, max_lag=None):
        n = len(signal)
        if max_lag is None:
            max_lag = n
        mean = sum(signal) / n
        centered = [x - mean for x in signal]
        # Compute autocorrelation directly
        result = []
        for lag in range(max_lag):
            s = 0
            for i in range(n - lag):
                s += centered[i] * centered[i + lag]
            result.append(s)
        if result[0] != 0:
            result = [r / result[0] for r in result]
        return result

    def compute_envelope(signal, window=4):
        abs_sig = [abs(x) for x in signal]
        n = len(abs_sig)
        half = window // 2
        result = []
        for i in range(n):
            start = max(0, i - half)
            end = min(n, i + half + 1)
            result.append(sum(abs_sig[start:end]) / (end - start))
        return result

    def find_peaks(signal, min_height=None, min_distance=5, max_peaks=10):
        sig = list(signal)
        n = len(sig)
        if min_height is None:
            min_height = sum(sig)/n + 0.1 * (max(sig) - sum(sig)/n)

        peaks = []
        for i in range(1, n - 1):
            if sig[i] > sig[i-1] and sig[i] > sig[i+1] and sig[i] >= min_height:
                if not peaks or (i - peaks[-1][0]) >= min_distance:
                    peaks.append((i, sig[i]))
                elif sig[i] > peaks[-1][1]:
                    peaks[-1] = (i, sig[i])

        peaks.sort(key=lambda x: x[1], reverse=True)
        peaks = peaks[:max_peaks]
        peaks.sort(key=lambda x: x[0])
        return peaks


# ============================================================================
# Pile Analysis Algorithm
# ============================================================================
class PileAnalyzer:
    """
    Pile integrity analyzer using autocorrelation method.

    Physics:
    - Impact generates stress wave traveling at velocity v
    - Wave reflects from impedance changes (defects, bottom)
    - Round trip time to position x: t = 2x/v
    - For good pile: only bottom reflection visible
    - For defect pile: additional early reflection from defect

    Method inspired by the FPGA defect_ac_classifier.v:
    1. Find impact peak position
    2. Compute autocorrelation of post-impact segment
    3. Find fundamental period T0 (bottom reflection)
    4. Check for additional peaks at shorter lags (defect reflections)
    5. Classify: GOOD if only bottom peak, DEFECT if additional peaks
    6. Calculate defect position from lag ratio
    """

    def __init__(self, sample_period_us=10.42, velocity_range=(2000, 2600)):
        self.dt = sample_period_us
        self.v_min, self.v_max = velocity_range

    def analyze(self, waveform, rod_len_mm=None):
        """
        Analyze a single waveform.

        Returns dict with:
            - prediction: 'GOOD', 'DEFECT', 'NOISE', 'INVALID'
            - confidence: 'HIGH', 'LOW'
            - defect_distance_mm: estimated defect position
            - bottom_lag: autocorrelation lag of bottom reflection
            - defect_lag: autocorrelation lag of defect reflection (if any)
            - T0: fundamental period in samples
            - details: additional debug info
        """
        n = len(waveform)
        if n < 100:
            return {'prediction': 'INVALID', 'confidence': 'NONE',
                    'defect_distance_mm': 0, 'bottom_lag': 0, 'defect_lag': 0,
                    'T0': 0, 'details': 'Too few samples'}

        # 1. Find impact peak
        abs_wave = [abs(x) for x in waveform]
        impact_idx = abs_wave.index(max(abs_wave[:min(60, n)]))  # impact within first 60 samples

        # Check if there's actually a significant impact
        peak_abs = abs_wave[impact_idx]
        if peak_abs < 50000:  # Very weak signal → noise or no hit
            return {'prediction': 'NOISE', 'confidence': 'HIGH',
                    'defect_distance_mm': 0, 'bottom_lag': 0, 'defect_lag': 0,
                    'T0': 0, 'impact_idx': impact_idx, 'peak_abs': peak_abs,
                    'details': f'Weak signal: peak={peak_abs}'}

        # 2. Extract post-impact segment (skip the impact ringing, ~16 samples)
        skip = 16
        seg_start = impact_idx + skip
        if seg_start + 200 > n:
            return {'prediction': 'INVALID', 'confidence': 'NONE',
                    'defect_distance_mm': 0, 'bottom_lag': 0, 'defect_lag': 0,
                    'T0': 0, 'impact_idx': impact_idx, 'peak_abs': peak_abs,
                    'details': 'Segment too short after impact'}

        segment = waveform[seg_start:min(seg_start + 300, n)]

        # 3. Compute autocorrelation (lags 8 to ~150)
        ac = compute_autocorrelation(segment, max_lag=min(150, len(segment)))

        # 4. Find peaks in autocorrelation
        # The first significant peak in ac corresponds to the fundamental period
        # (bottom reflection for good piles, or defect reflection)

        # Normalize by r[0]
        if ac[0] != 0:
            ac_norm = [x / ac[0] if ac[0] != 0 else 0 for x in ac]
        else:
            ac_norm = ac[:]

        # Find bottom peak: look in lag range corresponding to rod length
        # For 1000mm rod, v=2200m/s: T0 ≈ 2*1m/2200 / 10.42e-6 ≈ 87 samples
        # Search range: lag 60 to 130
        lag_lo = 50
        lag_hi = min(140, len(ac_norm) - 1)

        # Find all local maxima in autocorrelation
        ac_peaks = []
        for i in range(lag_lo, lag_hi):
            if ac_norm[i] > ac_norm[i-1] and ac_norm[i] > ac_norm[i+1]:
                if ac_norm[i] > 0.05:  # minimum correlation threshold
                    ac_peaks.append((i, ac_norm[i]))

        # Sort by amplitude
        ac_peaks.sort(key=lambda x: x[1], reverse=True)

        if not ac_peaks:
            # Try a wider range
            for i in range(20, min(150, len(ac_norm) - 1)):
                if i > 0 and i < len(ac_norm) - 1:
                    if ac_norm[i] > ac_norm[i-1] and ac_norm[i] > ac_norm[i+1]:
                        if ac_norm[i] > 0.03:
                            ac_peaks.append((i, ac_norm[i]))
            ac_peaks.sort(key=lambda x: x[1], reverse=True)

        if not ac_peaks:
            return {'prediction': 'INVALID', 'confidence': 'NONE',
                    'defect_distance_mm': 0, 'bottom_lag': 0, 'defect_lag': 0,
                    'T0': 0, 'impact_idx': impact_idx, 'peak_abs': peak_abs,
                    'ac_peaks': ac_peaks, 'ac': ac_norm[:140],
                    'details': 'No autocorrelation peaks found'}

        # The strongest peak should be the fundamental period (bottom reflection)
        bottom_peak = ac_peaks[0]
        bottom_lag = bottom_peak[0]
        bottom_corr = bottom_peak[1]

        # 5. Determine if there's a defect peak
        # Look for significant peaks at lags shorter than bottom_lag - margin
        defect_margin = 8  # minimum lag separation from bottom
        defect_lag = 0
        defect_corr = 0
        is_defect = False

        # Check peaks before bottom_lag
        for lag, corr in ac_peaks[1:]:
            if lag < bottom_lag - defect_margin and corr > 0.08:
                # This is a potential defect reflection
                # Verify it's not a harmonic (lag should not be exactly T0/2)
                if not (abs(2 * lag - bottom_lag) <= 4 or
                        abs(3 * lag - bottom_lag) <= 5):
                    if corr > defect_corr:
                        defect_lag = lag
                        defect_corr = corr
                        is_defect = True

        # Also scan low lags for early reflections (defects at 200-300mm)
        if not is_defect:
            for i in range(15, min(bottom_lag - defect_margin, len(ac_norm))):
                if i > 0 and i < len(ac_norm) - 1:
                    if ac_norm[i] > ac_norm[i-1] and ac_norm[i] > ac_norm[i+1]:
                        if ac_norm[i] > 0.12:  # higher threshold for early lags
                            # Check this is not just noise
                            if ac_norm[i] > defect_corr:
                                # Additional check: is this at a "reasonable" fraction?
                                ratio = i / bottom_lag
                                if 0.15 < ratio < 0.85:
                                    defect_lag = i
                                    defect_corr = ac_norm[i]
                                    is_defect = True

        # 6. Classify
        if is_defect and defect_lag > 0:
            prediction = 'DEFECT'
            # Calculate defect distance
            # d_defect / L = t_defect / t_bottom
            # d_defect = L * defect_lag / bottom_lag
            if rod_len_mm and rod_len_mm > 0:
                # w/o rod_len, estimate using velocity
                defect_dist = rod_len_mm * defect_lag / bottom_lag
            else:
                # Use typical velocity: dist = v * lag * dt / 2
                defect_dist = NYLON_V_TYPICAL * defect_lag * self.dt * 1e-6 / 2 * 1000  # mm

            confidence = 'HIGH' if defect_corr > 0.15 else 'LOW'
        else:
            prediction = 'GOOD'
            defect_dist = 0
            confidence = 'HIGH' if bottom_corr > 0.3 else 'LOW'

        # 7. Calculate velocity estimate
        if rod_len_mm and rod_len_mm > 0 and bottom_lag > 0:
            # v = 2*L / (bottom_lag * dt)
            velocity = 2 * rod_len_mm / 1000 / (bottom_lag * self.dt * 1e-6)
        else:
            velocity = 0

        return {
            'prediction': prediction,
            'confidence': confidence,
            'defect_distance_mm': round(defect_dist),
            'bottom_lag': bottom_lag,
            'defect_lag': defect_lag,
            'T0': bottom_lag,
            'velocity_ms': round(velocity),
            'impact_idx': impact_idx,
            'peak_abs': peak_abs,
            'ac_peaks': ac_peaks[:5],
            'bottom_corr': bottom_corr,
            'defect_corr': defect_corr,
            'ac_norm': ac_norm[:140] if HAVE_PLOT else None,
            'details': ''
        }


# ============================================================================
# Batch Processing & Evaluation
# ============================================================================
def process_all_data(analyzer, limit_per_dir=None):
    """Process all data files and collect statistics."""
    results = []
    stats = defaultdict(lambda: {'total': 0, 'correct': 0, 'wrong': 0,
                                  'errors': []})

    for dirpath in sorted(DATA_DIR.iterdir()):
        if not dirpath.is_dir():
            continue

        dirname = dirpath.name
        ground_truth, true_defect_pos = classify_ground_truth(dirname)

        csv_files = sorted(dirpath.glob('hit_*.csv'))
        if limit_per_dir:
            csv_files = csv_files[:limit_per_dir]

        for csv_file in csv_files:
            try:
                info = parse_csv_header(csv_file)
                waveform = load_csv_waveform(csv_file)

                if len(waveform) < 100:
                    continue

                rod_len = info.get('rod_len_mm') or 1000
                result = analyzer.analyze(waveform, rod_len_mm=rod_len)

                # Determine if correct
                pred = result['prediction']
                if ground_truth == 'NOISE':
                    is_correct = (pred in ('NOISE', 'INVALID'))
                elif ground_truth == 'GOOD':
                    is_correct = (pred == 'GOOD')
                elif ground_truth == 'DEFECT':
                    # DEFECT if prediction is DEFECT
                    is_correct = (pred == 'DEFECT')
                else:
                    is_correct = True

                stats[ground_truth]['total'] += 1
                if is_correct:
                    stats[ground_truth]['correct'] += 1
                else:
                    stats[ground_truth]['wrong'] += 1

                # For defect cases, check distance accuracy
                dist_error = None
                if ground_truth == 'DEFECT' and pred == 'DEFECT' and true_defect_pos:
                    dist_error = abs(result['defect_distance_mm'] - true_defect_pos)

                results.append({
                    'file': str(csv_file.relative_to(DATA_DIR)),
                    'ground_truth': ground_truth,
                    'true_defect_mm': true_defect_pos,
                    'prediction': pred,
                    'defect_distance_mm': result['defect_distance_mm'],
                    'dist_error_mm': dist_error,
                    'bottom_lag': result['bottom_lag'],
                    'defect_lag': result['defect_lag'],
                    'T0': result['T0'],
                    'confidence': result['confidence'],
                    'velocity_ms': result['velocity_ms'],
                })

            except Exception as e:
                stats[ground_truth]['errors'].append(f"{csv_file.name}: {e}")
                continue

    return results, stats


# ============================================================================
# Visualization
# ============================================================================
def plot_examples(analyzer):
    """Plot example waveforms from each category."""
    if not HAVE_PLOT:
        print("Skipping plots (matplotlib not available)")
        return

    categories = {
        'GOOD': [
            ('03_233534_正常棒_总长1016mm_中敲', 'Normal rod, 1016mm'),
            ('05_120614_好棒_总长1000mm_轻轻敲_塑料头', 'Good rod, light plastic'),
            ('07_124142_好棒_总长1000mm_轻轻敲_铁头', 'Good rod, light iron'),
        ],
        'DEFECT_305': [
            ('01_233055_缺陷棒_总长1000mm_距敲击端305mm_中敲', 'Defect at 305mm'),
            ('13_160206_坏帮_总长1002_轻轻敲_塑料头_307', 'Defect 307mm, light plastic'),
        ],
        'DEFECT_695': [
            ('04_233806_缺陷棒_总长1000mm_距敲击端695mm_中敲', 'Defect at 695mm'),
            ('09_134941_坏棒_总长1002mm_轻轻敲_塑料头_695', 'Defect 695mm, light plastic'),
        ],
        'DEFECT_800': [
            ('17_171034_坏棒_总长1000_轻轻敲_塑料头_800', 'Defect 800mm, light plastic'),
            ('19_173006_坏棒_总长_1000_轻轻敲_铁头_800', 'Defect 800mm, light iron'),
        ],
        'DEFECT_200': [
            ('21_175223_坏棒_总长1000mm_塑料头_200', 'Defect 200mm, plastic'),
            ('22_175528_坏棒_总长_1000mm_铁头_200', 'Defect 200mm, iron'),
        ],
        'NOISE': [
            ('02_233205_环境干扰_踩地面_未敲棒', 'Noise, no hit'),
        ],
    }

    fig, axes = plt.subplots(len(categories), 2, figsize=(16, 4 * len(categories)))
    if len(categories) == 1:
        axes = axes.reshape(1, -1)

    for row_idx, (cat_name, dirs) in enumerate(categories.items()):
        for dir_name, label in dirs:
            # Find matching directory
            matched = None
            for d in DATA_DIR.iterdir():
                if d.is_dir() and dir_name[:20] in d.name:
                    matched = d
                    break

            if matched is None:
                continue

            csv_files = sorted(matched.glob('hit_*.csv'))
            if not csv_files:
                continue

            # Use first file
            csv_file = csv_files[0]
            waveform = load_csv_waveform(csv_file)
            rod_len = 1000  # default

            try:
                info = parse_csv_header(csv_file)
                rod_len = info.get('rod_len_mm') or 1000
            except:
                pass

            result = analyzer.analyze(waveform, rod_len_mm=rod_len)

            ax1 = axes[row_idx][0]
            ax2 = axes[row_idx][1]

            # Plot waveform
            times = [i * SAMPLE_PERIOD_US for i in range(len(waveform))]
            ax1.plot(times, waveform, 'b-', linewidth=0.5, alpha=0.8)
            ax1.axvline(x=result['impact_idx'] * SAMPLE_PERIOD_US, color='r',
                       linestyle='--', alpha=0.5, label=f"Impact@{result['impact_idx']}")
            if result['bottom_lag'] > 0:
                bottom_sample = result['impact_idx'] + 16 + result['bottom_lag']
                if bottom_sample < len(waveform):
                    ax1.axvline(x=bottom_sample * SAMPLE_PERIOD_US, color='g',
                               linestyle='--', alpha=0.7,
                               label=f"Bottom lag={result['bottom_lag']}")
            if result['defect_lag'] > 0:
                defect_sample = result['impact_idx'] + 16 + result['defect_lag']
                if defect_sample < len(waveform):
                    ax1.axvline(x=defect_sample * SAMPLE_PERIOD_US, color='orange',
                               linestyle='--', alpha=0.7,
                               label=f"Defect lag={result['defect_lag']}")

            ax1.set_title(f"{cat_name}: {label}\n"
                         f"Pred: {result['prediction']} "
                         f"DefDist: {result['defect_distance_mm']}mm "
                         f"T0: {result['T0']}")
            ax1.set_xlabel('Time (us)')
            ax1.set_ylabel('ADC Value')
            ax1.legend(fontsize=6, loc='upper right')
            ax1.grid(True, alpha=0.3)

            # Plot autocorrelation
            if result.get('ac_norm'):
                ac = result['ac_norm']
                lags = list(range(len(ac)))
                ax2.plot(lags, ac, 'b-', linewidth=0.8)
                ax2.axhline(y=0, color='gray', alpha=0.5)

                if result['bottom_lag'] > 0 and result['bottom_lag'] < len(ac):
                    ax2.axvline(x=result['bottom_lag'], color='g',
                               linestyle='--', alpha=0.7,
                               label=f"Bottom: {result['bottom_lag']}")
                    ax2.plot(result['bottom_lag'], ac[result['bottom_lag']],
                            'go', markersize=8)

                if result['defect_lag'] > 0 and result['defect_lag'] < len(ac):
                    ax2.axvline(x=result['defect_lag'], color='orange',
                               linestyle='--', alpha=0.7,
                               label=f"Defect: {result['defect_lag']}")
                    ax2.plot(result['defect_lag'], ac[result['defect_lag']],
                            'o', color='orange', markersize=8)

                ax2.set_title(f"Autocorrelation (T0={result['T0']}, v≈{result['velocity_ms']}m/s)")
                ax2.set_xlabel('Lag (samples)')
                ax2.set_ylabel('Correlation')
                ax2.legend(fontsize=6)
                ax2.grid(True, alpha=0.3)

            break  # Only plot first file per category

    plt.tight_layout()
    plt.savefig(PLOT_DIR / 'waveform_examples.png', dpi=150)
    plt.close()
    print(f"Saved waveform examples to {PLOT_DIR / 'waveform_examples.png'}")


def plot_summary(results, stats):
    """Plot summary statistics."""
    if not HAVE_PLOT:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1. Confusion matrix-style bar chart
    ax1 = axes[0][0]
    categories = sorted(stats.keys())
    totals = [stats[c]['total'] for c in categories]
    corrects = [stats[c]['correct'] for c in categories]
    wrongs = [stats[c]['wrong'] for c in categories]

    x = range(len(categories))
    width = 0.35
    ax1.bar(x, totals, width, label='Total', color='lightgray')
    ax1.bar(x, corrects, width, label='Correct', color='green', alpha=0.7)
    ax1.bar([i + width for i in x], wrongs, width, label='Wrong', color='red', alpha=0.7)
    ax1.set_xticks([i + width/2 for i in x])
    ax1.set_xticklabels(categories, rotation=45, ha='right', fontsize=8)
    ax1.set_title('Classification Results by Category')
    ax1.set_ylabel('Count')
    ax1.legend()

    # 2. Accuracy by category
    ax2 = axes[0][1]
    accuracies = []
    acc_labels = []
    for c in categories:
        if stats[c]['total'] > 0:
            acc = stats[c]['correct'] / stats[c]['total'] * 100
            accuracies.append(acc)
            acc_labels.append(c)
    ax2.bar(range(len(acc_labels)), accuracies, color='steelblue')
    ax2.set_xticks(range(len(acc_labels)))
    ax2.set_xticklabels(acc_labels, rotation=45, ha='right', fontsize=8)
    ax2.set_title('Accuracy by Category')
    ax2.set_ylabel('Accuracy (%)')
    ax2.set_ylim(0, 105)
    for i, acc in enumerate(accuracies):
        ax2.text(i, acc + 1, f'{acc:.1f}%', ha='center', fontsize=8)

    # 3. Defect distance error distribution
    ax3 = axes[1][0]
    defect_results = [r for r in results
                     if r['ground_truth'] == 'DEFECT'
                     and r['prediction'] == 'DEFECT'
                     and r['dist_error_mm'] is not None]
    if defect_results:
        errors = [r['dist_error_mm'] for r in defect_results]
        ax3.hist(errors, bins=30, color='orange', alpha=0.7, edgecolor='black')
        ax3.axvline(x=np.mean(errors), color='red', linestyle='--',
                   label=f'Mean: {np.mean(errors):.0f}mm')
        ax3.set_title(f'Defect Distance Error Distribution (n={len(defect_results)})')
        ax3.set_xlabel('Absolute Error (mm)')
        ax3.set_ylabel('Count')
        ax3.legend()

    # 4. T0 (fundamental period) distribution by class
    ax4 = axes[1][1]
    colors_map = {'GOOD': 'green', 'DEFECT': 'blue', 'NOISE': 'red'}
    for gt in ['GOOD', 'DEFECT', 'NOISE']:
        t0_vals = [r['T0'] for r in results
                   if r['ground_truth'] == gt and r['T0'] > 0]
        if t0_vals:
            ax4.hist(t0_vals, bins=30, alpha=0.5, label=f'{gt} (n={len(t0_vals)})',
                    color=colors_map.get(gt, 'gray'))

    ax4.set_title('Fundamental Period (T0) Distribution')
    ax4.set_xlabel('T0 (samples)')
    ax4.set_ylabel('Count')
    ax4.legend()

    plt.tight_layout()
    plt.savefig(PLOT_DIR / 'summary_stats.png', dpi=150)
    plt.close()
    print(f"Saved summary stats to {PLOT_DIR / 'summary_stats.png'}")


# ============================================================================
# Detailed T0 analysis for GOOD vs DEFECT (especially 800mm)
# ============================================================================
def analyze_T0_patterns(analyzer, limit_per_dir=50):
    """Deep dive into T0 (fundamental period) patterns for GOOD vs 800mm defect."""
    print("\n" + "="*80)
    print("T0 (Fundamental Period) Analysis for GOOD vs DEFECT discrimination")
    print("="*80)

    good_T0s = []
    defect_T0s = defaultdict(list)  # defect_pos -> [T0s]

    for dirpath in sorted(DATA_DIR.iterdir()):
        if not dirpath.is_dir():
            continue

        dirname = dirpath.name
        gt, true_defect = classify_ground_truth(dirname)

        csv_files = sorted(dirpath.glob('hit_*.csv'))
        if limit_per_dir:
            csv_files = csv_files[:limit_per_dir]

        for csv_file in csv_files:
            try:
                waveform = load_csv_waveform(csv_file)
                if len(waveform) < 100:
                    continue
                rod_len = 1000
                result = analyzer.analyze(waveform, rod_len_mm=rod_len)

                if result['T0'] > 0 and result['prediction'] != 'NOISE':
                    if gt == 'GOOD':
                        good_T0s.append((result['T0'], csv_file.name))
                    elif gt == 'DEFECT':
                        defect_T0s[true_defect].append((result['T0'], csv_file.name))
            except:
                continue

    print(f"\nGOOD rods: {len(good_T0s)} samples")
    if good_T0s:
        good_vals = [t[0] for t in good_T0s]
        print(f"  T0: min={min(good_vals)}, max={max(good_vals)}, "
              f"mean={sum(good_vals)/len(good_vals):.1f}, "
              f"median={sorted(good_vals)[len(good_vals)//2]}")

    for def_pos, vals in sorted(defect_T0s.items()):
        print(f"\nDEFECT at {def_pos}mm: {len(vals)} samples")
        if vals:
            t0_vals = [t[0] for t in vals]
            print(f"  T0: min={min(t0_vals)}, max={max(t0_vals)}, "
                  f"mean={sum(t0_vals)/len(t0_vals):.1f}, "
                  f"median={sorted(t0_vals)[len(t0_vals)//2]}")

    # Check if T0 alone can separate GOOD from 800mm defect
    if 800 in defect_T0s and good_T0s:
        good_set = set(t[0] for t in good_T0s)
        d800_set = set(t[0] for t in defect_T0s[800])

        good_max = max(t[0] for t in good_T0s)
        d800_min = min(t[0] for t in defect_T0s[800])

        overlap = good_set & d800_set
        print(f"\n800mm DEFECT vs GOOD overlap analysis:")
        print(f"  Good T0 max: {good_max}")
        print(f"  800mm T0 min: {d800_min}")
        print(f"  Overlap count: {len(overlap)}")
        print(f"  Separable at T0={max(good_max, d800_min)}: "
              f"{'Yes' if good_max < d800_min else 'No (overlap)'}")

    return good_T0s, defect_T0s


# ============================================================================
# Main
# ============================================================================
def main():
    print("="*80)
    print("基桩动测仪 - 桩完整性检测算法分析")
    print("="*80)
    print(f"数据路径: {DATA_DIR}")
    print(f"采样周期: {SAMPLE_PERIOD_US} us/sample")
    print(f"纵波速度范围: {NYLON_V_MIN} ~ {NYLON_V_MAX} m/s")
    print()

    analyzer = PileAnalyzer(sample_period_us=SAMPLE_PERIOD_US,
                            velocity_range=(NYLON_V_MIN, NYLON_V_MAX))

    # Process a subset first for quick analysis
    print("Processing data (limited to 50 files per category for speed)...")
    results, stats = process_all_data(analyzer, limit_per_dir=50)

    # Print statistics
    print("\n" + "="*80)
    print("分类统计 Results Summary")
    print("="*80)
    total_correct = 0
    total_all = 0
    for category in ['GOOD', 'DEFECT', 'NOISE']:
        s = stats[category]
        total_correct += s['correct']
        total_all += s['total']
        acc = s['correct'] / s['total'] * 100 if s['total'] > 0 else 0
        print(f"\n{category}:")
        print(f"  Total: {s['total']}")
        print(f"  Correct: {s['correct']}")
        print(f"  Wrong: {s['wrong']}")
        print(f"  Accuracy: {acc:.1f}%")
        if s['errors']:
            print(f"  Errors: {s['errors'][:5]}")

    overall_acc = total_correct / total_all * 100 if total_all > 0 else 0
    print(f"\nOverall Accuracy: {total_correct}/{total_all} = {overall_acc:.1f}%")

    # Analyze defect distance accuracy
    print("\n" + "="*80)
    print("缺陷位置估计 Defect Distance Accuracy")
    print("="*80)
    defect_by_pos = defaultdict(list)
    for r in results:
        if r['ground_truth'] == 'DEFECT' and r['prediction'] == 'DEFECT':
            defect_by_pos[r['true_defect_mm']].append(r)

    for pos in sorted(defect_by_pos.keys()):
        items = defect_by_pos[pos]
        if not items:
            continue
        errors = [it['dist_error_mm'] for it in items
                 if it['dist_error_mm'] is not None]
        preds = [it['defect_distance_mm'] for it in items]
        if errors:
            print(f"\nDefect at {pos}mm ({len(items)} samples):")
            print(f"  Mean predicted: {sum(preds)/len(preds):.0f}mm")
            print(f"  Mean error: {sum(errors)/len(errors):.0f}mm")
            print(f"  Median error: {sorted(errors)[len(errors)//2]:.0f}mm")
            print(f"  Min error: {min(errors):.0f}mm")
            print(f"  Max error: {max(errors):.0f}mm")

    # Plot examples
    print("\nGenerating plots...")
    plot_examples(analyzer)
    plot_summary(results, stats)

    # Deep T0 analysis
    analyze_T0_patterns(analyzer, limit_per_dir=50)

    # Save results to JSON for further analysis
    output_json = Path(r"D:\FPGA\dian_sai\Project\God3.11\analysis_results.json")
    with open(output_json, 'w', encoding='utf-8') as f:
        json.dump({
            'stats': {k: {'total': v['total'], 'correct': v['correct'],
                         'wrong': v['wrong']} for k, v in stats.items()},
            'overall_accuracy': overall_acc,
            'results': results[:100]  # Save first 100 results
        }, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to {output_json}")

    print("\n分析完成!")


if __name__ == '__main__':
    main()
