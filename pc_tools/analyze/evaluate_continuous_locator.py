# -*- coding: utf-8 -*-
"""Evaluate continuous reflection candidates on the captured waveform frames.

This is a software-side diagnostic model, not an RTL bit-accurate model.  It
uses the same trigger-relative first-difference template as defect_analyzer.v
and reports local correlation peaks without forcing them into 305/695 windows.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

from __future__ import annotations

import csv
from collections import Counter
from pathlib import Path
from statistics import median


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
TRIGGER_INDEX = 32
TEMPLATE_POINTS = 12
SEARCH_LO = 20
SEARCH_HI = 118
BOTTOM_LO = 97
BOTTOM_HI = 113


def load_frame(path: Path) -> tuple[list[int], dict[str, str]]:
    samples: list[int] = []
    meta: dict[str, str] = {}
    with path.open(encoding="utf-8-sig", newline="") as src:
        for row in csv.reader(src):
            if not row:
                continue
            if row[0].startswith("#"):
                key = row[0][1:].strip()
                meta[key] = row[1].strip() if len(row) > 1 else ""
            elif row[0].isdigit():
                samples.append(int(row[1]))
    return samples, meta


def preprocess(raw: list[int]) -> list[int]:
    baseline = sum(raw[:24]) // 24
    # defect_analyzer listens to wave_trigger.corrected_data before the
    # optional three-point display smoother.
    return [value - baseline for value in raw]


def correlations(
    smooth: list[int],
) -> tuple[list[int], list[int], list[int], list[int]]:
    post = smooth[TRIGGER_INDEX:]
    # RTL stores diff_mem[0] on the first sample after the trigger.
    diff = [
        post[index] - post[index - 1] for index in range(1, len(post))
    ]
    template = diff[:TEMPLATE_POINTS]
    scores = [0] * len(diff)
    signed_scores = [0] * len(diff)
    energies = [0] * len(diff)
    template_energy = sum(abs(value) for value in template)
    for lag in range(SEARCH_LO, min(SEARCH_HI + 1, len(diff) - TEMPLATE_POINTS)):
        window = diff[lag : lag + TEMPLATE_POINTS]
        signed_scores[lag] = sum(a * b for a, b in zip(template, window))
        scores[lag] = abs(signed_scores[lag])
        energies[lag] = sum(abs(value) for value in window)
    return scores, signed_scores, energies, [template_energy] * len(diff)


def envelope4(values: list[int]) -> list[int]:
    history = [0, 0, 0, 0]
    total = 0
    result = []
    for value in values:
        magnitude = abs(value)
        total += magnitude - history[-1]
        history = [magnitude] + history[:-1]
        result.append(total >> 2)
    return result


def rtl_packet_candidates(corrected: list[int], trigger_threshold: int):
    env = envelope4(corrected)
    frame_peak = env[TRIGGER_INDEX]
    near_peak = mid_peak = ref_peak = 0
    packet_mask: list[int] = []
    active = False
    width = fall_count = 0
    packet_peak = packet_peak_index = 0
    previous_env = env[TRIGGER_INDEX]
    bottom_peak = 0
    bottom_index = 105

    for raw_index in range(TRIGGER_INDEX + 1, len(corrected)):
        index = raw_index - TRIGGER_INDEX
        current = env[raw_index]
        threshold = max(30000, trigger_threshold, frame_peak >> 4)
        if current > frame_peak:
            frame_peak = current
        if 20 <= index <= 40:
            near_peak = max(near_peak, current)
        if 55 <= index <= 85:
            mid_peak = max(mid_peak, current)
        if 95 <= index <= 120:
            ref_peak = max(ref_peak, current)
        if 97 <= index <= 113 and current > bottom_peak:
            bottom_peak = current
            bottom_index = index

        if index < 20:
            active = False
            width = fall_count = 0
        elif not active:
            if current > threshold and current > previous_env:
                active = True
                width = 1
                fall_count = 0
                packet_peak = current
                packet_peak_index = index
        else:
            width = min(width + 1, 63)
            if current > packet_peak:
                packet_peak = current
                packet_peak_index = index
                fall_count = 0
            elif current < previous_env:
                if (fall_count >= 1 or current <= threshold) and width >= 3:
                    if len(packet_mask) < 8:
                        packet_mask.append(packet_peak_index)
                    active = False
                    width = fall_count = 0
                else:
                    fall_count += 1
            else:
                fall_count = 0
        previous_env = current
    if active and width >= 4:
        if len(packet_mask) < 8:
            packet_mask.append(packet_peak_index)
    return (
        packet_mask,
        env,
        frame_peak,
        near_peak,
        mid_peak,
        ref_peak,
        bottom_peak,
        bottom_index,
    )


def rtl_like_result(raw: list[int], trigger_threshold: int = 20000):
    corrected = preprocess(raw)
    (
        packet_mask,
        _,
        frame_peak,
        near_peak,
        mid_peak,
        ref_peak,
        bottom_peak,
        bottom_index,
    ) = rtl_packet_candidates(corrected, trigger_threshold)
    scores, _, _, _ = correlations(corrected)
    candidate_lags = [
        lag for lag in packet_mask if 20 <= lag <= 89 and lag < len(scores)
    ]
    ranked = sorted(candidate_lags, key=lambda lag: scores[lag], reverse=True)
    bottom_peaks = local_peaks(scores, 97, 113)
    if bottom_peaks:
        bottom_lag = max(bottom_peaks, key=lambda lag: scores[lag])
    else:
        bottom_lag = bottom_index
    bottom_valid = bottom_peak >= 500000
    normal = mid_peak * 100 > frame_peak * 66
    late_evidence = (
        near_peak * 100 > frame_peak * 82
        or ref_peak * 100 > frame_peak * 55
    )

    chosen = ranked[0] if ranked else 0
    if len(ranked) >= 2 and bottom_valid:
        first, second = ranked[:2]
        if abs((first + second) - bottom_lag) <= 12:
            early, late = sorted((first, second))
            chosen = late if late_evidence else early

    strong = frame_peak >= 500000 and max(abs(value) for value in corrected) < 8300000
    if not strong:
        state = "INVALID"
    elif normal:
        state = "NORMAL"
    elif chosen:
        state = "DEFECT"
    else:
        state = "INVALID"
    distance = (
        chosen * 1000 // bottom_lag
        if state == "DEFECT" and bottom_valid and bottom_lag
        else 0
    )
    return state, chosen, bottom_lag if bottom_valid else 0, distance, packet_mask


def local_peaks(values: list[int], lo: int, hi: int) -> list[int]:
    return [
        index
        for index in range(max(lo, 1), min(hi, len(values) - 2) + 1)
        if values[index] > values[index - 1]
        and values[index] >= values[index + 1]
    ]


def select_candidate(
    scores: list[int],
    signed_scores: list[int],
    energies: list[int],
    peaks: list[int],
    bottom: int,
    mode: str,
) -> int:
    candidates = [
        index
        for index in peaks
        if SEARCH_LO <= index and index + 8 < bottom
    ]
    if mode.startswith("phase_"):
        candidates = [
            index
            for index in candidates
            if (
                (2 * index < bottom and signed_scores[index] < 0)
                or (2 * index >= bottom and signed_scores[index] >= 0)
            )
        ]
        mode = mode[6:]
    if not candidates:
        return 0
    if not candidates:
        return 0

    def feature(index: int) -> float:
        left = min(scores[max(SEARCH_LO, index - 12) : index] or [0])
        right = min(scores[index + 1 : min(bottom, index + 13)] or [0])
        prominence = max(scores[index] - max(left, right), 0)
        energy = max(energies[index], 1)
        if mode == "raw":
            return float(scores[index])
        if mode == "norm":
            return scores[index] / energy
        if mode == "sqrt_norm":
            return (scores[index] * scores[index]) / energy
        if mode == "three_quarter_norm":
            return (scores[index] ** 4) / (energy ** 3)
        if mode == "prom":
            return float(prominence)
        if mode == "norm_prom":
            return prominence / energy
        if mode == "late_norm_prom":
            return (prominence / energy) * index
        raise ValueError(mode)

    return max(candidates, key=feature)


def classify_group(name: str) -> tuple[str, int]:
    if "305mm" in name:
        return "DEFECT", 305
    if "695mm" in name:
        return "DEFECT", 695
    if "正常棒" in name:
        return "NORMAL", 0
    return "INTERFERENCE", 0


def summarize() -> None:
    locator_errors: dict[str, list[int]] = {
        mode: []
        for mode in (
            "raw",
            "phase_raw",
            "sqrt_norm",
            "three_quarter_norm",
            "norm",
            "phase_norm",
            "prom",
            "norm_prom",
            "late_norm_prom",
        )
    }
    pair_records: dict[int, list[tuple[float, int, int, int]]] = {
        305: [],
        695: [],
    }
    for folder in sorted(path for path in DATA.iterdir() if path.is_dir()):
        kind, truth = classify_group(folder.name)
        group_locator_errors: dict[str, list[int]] = {
            mode: []
            for mode in locator_errors
        }
        best_all: list[int] = []
        best_bottom: list[int] = []
        first_half: list[int] = []
        peak_rank_counts: Counter[int] = Counter()
        expected_signs: Counter[str] = Counter()
        early_signs: Counter[str] = Counter()
        rows = []
        for path in sorted(folder.glob("hit_*.csv")):
            raw, _ = load_frame(path)
            smooth = preprocess(raw)
            scores, signed_scores, energies, template_energies = correlations(
                smooth
            )
            peaks = local_peaks(scores, SEARCH_LO, SEARCH_HI)
            if not peaks:
                continue
            ranked = sorted(peaks, key=lambda index: scores[index], reverse=True)
            bottom_peaks = [
                index for index in peaks if BOTTOM_LO <= index <= BOTTOM_HI
            ]
            bottom = (
                max(bottom_peaks, key=lambda index: scores[index])
                if bottom_peaks
                else 0
            )
            before_bottom = [
                index
                for index in peaks
                if index + 8 < (bottom if bottom else BOTTOM_HI)
            ]
            best_all.append(ranked[0])
            best_bottom.append(bottom)
            if before_bottom:
                first_half.append(before_bottom[0])
            expected = (
                round((bottom if bottom else 105) * truth / 1000)
                if truth
                else 0
            )
            if expected and bottom:
                for mode in locator_errors:
                    selected = select_candidate(
                        scores,
                        signed_scores,
                        energies,
                        peaks,
                        bottom,
                        mode,
                    )
                    error = selected - expected
                    locator_errors[mode].append(error)
                    group_locator_errors[mode].append(error)
                usable = [
                    index
                    for index in peaks
                    if SEARCH_LO <= index and index + 8 < bottom
                ]
                near = [
                    index
                    for index in usable
                    if 2 * index < bottom and signed_scores[index] < 0
                ]
                far = [
                    index
                    for index in usable
                    if 2 * index >= bottom and signed_scores[index] >= 0
                ]
                if near and far:
                    near_index = max(near, key=lambda index: scores[index])
                    far_index = max(
                        far,
                        key=lambda index: scores[index] / max(energies[index], 1),
                    )
                    near_quality = scores[near_index] / max(
                        energies[near_index], 1
                    )
                    far_quality = scores[far_index] / max(
                        energies[far_index], 1
                    )
                    pair_records[truth].append(
                        (
                            far_quality / max(near_quality, 1.0),
                            near_index,
                            far_index,
                            bottom,
                        )
                    )
            nearest_rank = 0
            if expected:
                nearest = min(peaks, key=lambda index: abs(index - expected))
                nearest_rank = ranked.index(nearest) + 1
                peak_rank_counts[nearest_rank] += 1
                expected_signs[
                    "+" if signed_scores[nearest] >= 0 else "-"
                ] += 1
                early = min(
                    (index for index in peaks if index < bottom - 8),
                    default=0,
                )
                if early:
                    early_signs[
                        "+" if signed_scores[early] >= 0 else "-"
                    ] += 1
            rows.append(
                (
                    path.name,
                    ranked[:6],
                    [scores[index] for index in ranked[:6]],
                    ["+" if signed_scores[index] >= 0 else "-" for index in ranked[:6]],
                    [energies[index] for index in ranked[:6]],
                    [
                        (
                            sum(
                                abs(value)
                                for value in smooth[
                                    TRIGGER_INDEX + max(0, index - 8) :
                                    TRIGGER_INDEX + index
                                ]
                            ),
                            sum(
                                abs(value)
                                for value in smooth[
                                    TRIGGER_INDEX + index :
                                    TRIGGER_INDEX + index + TEMPLATE_POINTS
                                ]
                            ),
                        )
                        for index in ranked[:6]
                    ],
                    template_energies[0],
                    bottom,
                    expected,
                    nearest_rank,
                )
            )

        print("=" * 90)
        print(folder.name, "frames=", len(rows), "kind=", kind, "truth=", truth)
        if rows:
            print(
                "best_all median/range:",
                median(best_all),
                (min(best_all), max(best_all)),
            )
            print(
                "bottom median/range:",
                median(best_bottom),
                (min(best_bottom), max(best_bottom)),
            )
            print(
                "earliest pre-bottom median/range:",
                median(first_half) if first_half else "-",
                (min(first_half), max(first_half)) if first_half else "-",
            )
            if truth:
                print("expected-peak correlation rank:", dict(peak_rank_counts))
                print(
                    "expected sign:",
                    dict(expected_signs),
                    "earliest sign:",
                    dict(early_signs),
                )
                for mode, errors in group_locator_errors.items():
                    print(
                        f"{mode:16s} median={median(errors):6.1f} "
                        f"|e|<=6 {sum(abs(error) <= 6 for error in errors)}/"
                        f"{len(errors)}"
                    )
            for row in rows[:3]:
                print(row)
    print("=" * 90)
    print("Continuous locator error over all defect frames (samples):")
    for mode, errors in locator_errors.items():
        within_6 = sum(abs(error) <= 6 for error in errors)
        within_12 = sum(abs(error) <= 12 for error in errors)
        print(
            f"{mode:16s} median={median(errors):6.1f} "
            f"range=({min(errors):+d},{max(errors):+d}) "
            f"|e|<=6 {within_6}/{len(errors)} "
            f"|e|<=12 {within_12}/{len(errors)}"
        )
    print("=" * 90)
    print("Complementary candidate quality ratio (far_norm / near_norm):")
    for truth, records in pair_records.items():
        ratios = [row[0] for row in records]
        print(
            truth,
            "n=",
            len(records),
            "median=",
            round(median(ratios), 3),
            "range=",
            (round(min(ratios), 3), round(max(ratios), 3)),
        )
    train = {
        truth: records[: len(records) // 2]
        for truth, records in pair_records.items()
    }
    test = {
        truth: records[len(records) // 2 :]
        for truth, records in pair_records.items()
    }
    thresholds = sorted(
        {row[0] for records in train.values() for row in records}
    )
    best = None
    for threshold in thresholds:
        train_ok = sum(
            ((row[0] >= threshold) == (truth == 695))
            for truth, records in train.items()
            for row in records
        )
        candidate = (train_ok, threshold)
        if best is None or candidate > best:
            best = candidate
    if best:
        _, threshold = best
        for split_name, split in (("train", train), ("test", test)):
            ok = sum(
                ((row[0] >= threshold) == (truth == 695))
                for truth, records in split.items()
                for row in records
            )
            total = sum(len(records) for records in split.values())
            print(
                f"ratio >= {threshold:.4f} selects second half: "
                f"{split_name} {ok}/{total}"
            )


def summarize_rtl_like() -> None:
    print("=" * 90)
    print("RTL-like end-to-end replay:")
    for folder in sorted(path for path in DATA.iterdir() if path.is_dir()):
        kind, truth = classify_group(folder.name)
        states: Counter[str] = Counter()
        distances: list[int] = []
        lags: list[int] = []
        bottoms: list[int] = []
        candidate_counts: list[int] = []
        for path in sorted(folder.glob("hit_*.csv")):
            raw, meta = load_frame(path)
            threshold = int(meta.get("threshold") or 20000)
            state, lag, bottom, distance, packet_mask = rtl_like_result(
                raw, threshold
            )
            states[state] += 1
            candidate_counts.append(len(packet_mask))
            if lag:
                lags.append(lag)
            if bottom:
                bottoms.append(bottom)
            if distance:
                distances.append(distance)
        print(
            folder.name,
            "states=",
            dict(states),
            "lag_med=",
            median(lags) if lags else "-",
            "bottom_med=",
            median(bottoms) if bottoms else "-",
            "distance_med=",
            median(distances) if distances else "-",
            "truth=",
            truth,
            "packet_count_med=",
            median(candidate_counts) if candidate_counts else "-",
        )


if __name__ == "__main__":
    summarize()
    summarize_rtl_like()
