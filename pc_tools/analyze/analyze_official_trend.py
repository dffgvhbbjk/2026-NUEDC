# -*- coding: utf-8 -*-
"""验证官方“五点单调趋势/边沿计数”对 305/695 mm 的辅助价值。

只读取 data 下 hit_*.csv，不写工程参数。前 50 帧选阈值，后 50 帧独立验证。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
from statistics import median

DATA = "data"
TRIG = 32


def load_hit(path):
    values = []
    with open(path, encoding="utf-8-sig") as src:
        for row in csv.reader(src):
            if row and row[0].isdigit():
                values.append(int(row[1]))
    base = sum(values[:24]) // 24
    return [v - base for v in values]


def envelope4(values):
    hist = [0, 0, 0, 0]
    total = 0
    out = []
    for value in values:
        mag = abs(value)
        total += mag - hist[-1]
        hist = [mag] + hist[:-1]
        out.append(total // 4)
    return out


def trend_edges(values, lo, hi, descending=True):
    """官方 qiudao_panjue 的等价形式，并统计真假翻转边沿。"""
    flag_prev = False
    edges = 0
    active = 0
    for i in range(max(lo, 4), min(hi + 1, len(values))):
        window = values[i - 4:i + 1]
        if descending:
            flag = all(window[j] < window[j - 1] for j in range(1, 5))
        else:
            flag = all(window[j] > window[j - 1] for j in range(1, 5))
        if flag:
            active += 1
        if flag != flag_prev:
            edges += 1
        flag_prev = flag
    return edges, active


def sign_changes(values, lo, hi):
    count = 0
    last = values[lo]
    for value in values[lo + 1:hi + 1]:
        if (last < 0 <= value) or (last >= 0 > value):
            count += 1
        last = value
    return count


def frame_features(raw):
    rel_raw = raw[TRIG:]
    env = envelope4(raw)[TRIG:]
    result = {}
    for name, values in (("raw", rel_raw), ("abs", list(map(abs, rel_raw))),
                         ("env", env)):
        for win, lo, hi in (("near", 20, 45), ("far", 55, 85),
                            ("all", 20, 110)):
            down_e, down_a = trend_edges(values, lo, hi, True)
            up_e, up_a = trend_edges(values, lo, hi, False)
            result[f"{name}_{win}_edges"] = down_e + up_e
            result[f"{name}_{win}_active"] = down_a + up_a
    result["raw_near_zc"] = sign_changes(rel_raw, 20, 45)
    result["raw_far_zc"] = sign_changes(rel_raw, 55, 85)
    frame_peak = max(env)
    near_peak = max(env[20:41])
    mid_peak = max(env[55:86])
    ref_peak = max(env[95:121])
    result["normal_class"] = int(mid_peak * 100 > frame_peak * 66)
    result["amplitude_far"] = int(
        near_peak * 100 > frame_peak * 82
        or mid_peak * 100 > frame_peak * 60
        or ref_peak * 100 > frame_peak * 55)
    result["far_votes"] = sum((
        near_peak * 100 > frame_peak * 82,
        mid_peak * 100 > frame_peak * 60,
        ref_peak * 100 > frame_peak * 55))
    result["far_near_vote"] = int(near_peak * 100 > frame_peak * 82)
    result["far_mid_vote"] = int(mid_peak * 100 > frame_peak * 60)
    result["far_ref_vote"] = int(ref_peak * 100 > frame_peak * 55)
    return result


def label_from_dir(name):
    if "305mm" in name:
        return 305
    if "695mm" in name:
        return 695
    return None


def main():
    frames = {305: [], 695: []}
    for group in sorted(os.listdir(DATA)):
        label = label_from_dir(group)
        if label is None:
            continue
        folder = os.path.join(DATA, group)
        for name in sorted(os.listdir(folder)):
            if name.startswith("hit_") and name.endswith(".csv"):
                frames[label].append(frame_features(load_hit(
                    os.path.join(folder, name))))

    keys = sorted(frames[305][0])
    print("frames:", {k: len(v) for k, v in frames.items()})
    print("\n单特征最佳阈值（前半训练，后半验证）:")
    scored = []
    for key in keys:
        train = {k: [r[key] for r in rows[:50]] for k, rows in frames.items()}
        test = {k: [r[key] for r in rows[50:]] for k, rows in frames.items()}
        candidates = sorted(set(train[305] + train[695]))
        for direction in (1, -1):
            for threshold in candidates:
                def pred(v):
                    return 695 if direction * v >= direction * threshold else 305
                train_ok = sum(pred(v) == label for label in (305, 695)
                               for v in train[label])
                test_ok = sum(pred(v) == label for label in (305, 695)
                              for v in test[label])
                scored.append((test_ok, train_ok, key, direction, threshold))
    for test_ok, train_ok, key, direction, threshold in sorted(
            scored, reverse=True)[:15]:
        op = ">=" if direction == 1 else "<="
        vals305 = [r[key] for r in frames[305]]
        vals695 = [r[key] for r in frames[695]]
        print("%-22s 695 if %s %2d  train=%3d/100 test=%3d/100  "
              "median 305/695=%s/%s range=%s/%s" % (
                  key, op, threshold, train_ok, test_ok,
                  median(vals305), median(vals695),
                  (min(vals305), max(vals305)),
                  (min(vals695), max(vals695))))

    # 当前幅值远端判据必须先成立，再由趋势证据确认；搜索这种“辅助否决”组合。
    print("\n当前幅值判据 + 官方趋势辅助确认（仅统计非 normal 帧）:")
    combo = []
    for key in keys:
        if key in ("normal_class", "amplitude_far"):
            continue
        train_rows = {
            k: [r for r in rows[:50] if not r["normal_class"]]
            for k, rows in frames.items()}
        test_rows = {
            k: [r for r in rows[50:] if not r["normal_class"]]
            for k, rows in frames.items()}
        candidates = sorted(set(r[key] for label in (305, 695)
                                for r in train_rows[label]))
        for direction in (1, -1):
            for threshold in candidates:
                def pred(row):
                    trend_ok = direction * row[key] >= direction * threshold
                    return 695 if row["amplitude_far"] and trend_ok else 305
                train_ok = sum(pred(r) == label for label in (305, 695)
                               for r in train_rows[label])
                test_ok = sum(pred(r) == label for label in (305, 695)
                              for r in test_rows[label])
                train_n = sum(len(v) for v in train_rows.values())
                test_n = sum(len(v) for v in test_rows.values())
                combo.append((test_ok / test_n, train_ok / train_n,
                              test_ok, test_n, train_ok, train_n,
                              key, direction, threshold))
    for _, _, test_ok, test_n, train_ok, train_n, key, direction, threshold in \
            sorted(combo, reverse=True)[:15]:
        op = ">=" if direction == 1 else "<="
        print("%-22s amp_far AND %s %2d  train=%3d/%-3d test=%3d/%-3d" %
              (key, op, threshold, train_ok, train_n, test_ok, test_n))

    print("\n针对“305误报695”的取舍（全200帧，已排除 normal）:")
    rows305 = [r for r in frames[305] if not r["normal_class"]]
    rows695 = [r for r in frames[695] if not r["normal_class"]]
    print("gate                         305误报       695识别")
    base_fp = sum(r["amplitude_far"] for r in rows305)
    base_tp = sum(r["amplitude_far"] for r in rows695)
    print("仅幅值                     %3d/%-3d      %3d/%-3d" %
          (base_fp, len(rows305), base_tp, len(rows695)))
    for threshold in range(20, 29):
        fp = sum(r["amplitude_far"] and r["env_far_active"] >= threshold
                 for r in rows305)
        tp = sum(r["amplitude_far"] and r["env_far_active"] >= threshold
                 for r in rows695)
        print("幅值 AND 趋势持续 >= %-2d    %3d/%-3d      %3d/%-3d" %
              (threshold, fp, len(rows305), tp, len(rows695)))
    for votes in (1, 2, 3):
        fp = sum(r["far_votes"] >= votes for r in rows305)
        tp = sum(r["far_votes"] >= votes for r in rows695)
        print("幅值条件至少命中 %d 项       %3d/%-3d      %3d/%-3d" %
              (votes, fp, len(rows305), tp, len(rows695)))
    for key in ("far_near_vote", "far_mid_vote", "far_ref_vote"):
        fp = sum(r[key] for r in rows305)
        tp = sum(r[key] for r in rows695)
        print("%-24s %3d/%-3d      %3d/%-3d" %
              (key, fp, len(rows305), tp, len(rows695)))
    print("305误报帧特征:",
          [(r["far_near_vote"], r["far_mid_vote"], r["far_ref_vote"],
            r["env_far_active"], r["env_far_edges"])
           for r in rows305 if r["amplitude_far"]])
    print("695仅MID命中帧特征:",
          [(r["env_far_active"], r["env_far_edges"])
           for r in rows695
           if r["far_mid_vote"] and not r["far_near_vote"]
           and not r["far_ref_vote"]])


if __name__ == "__main__":
    main()
