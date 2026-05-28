#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Summarize safety_throttle_debug_logger CSV output.

If an episode column is present, one row is emitted per episode. Otherwise the
whole CSV is treated as one run with episode=0.
"""

import argparse
import csv
import math
import os
from collections import defaultdict
from pathlib import Path


SUMMARY_COLUMNS = [
    "episode",
    "safety_pass_ratio",
    "safety_slow_ratio",
    "safety_stop_ratio",
    "mean_linear_ratio",
    "mean_angular_ratio",
    "angular_intervention_ratio",
    "reverse_request_ratio",
    "reverse_allowed_ratio",
    "max_continuous_slow_duration",
    "max_continuous_stop_duration",
    "front_center_min_mean",
    "left_turn_min_mean",
    "right_turn_min_mean",
    "rear_min_mean",
]


def to_float(value):
    if value is None:
        return math.nan
    s = str(value).strip()
    if s == "":
        return math.nan
    try:
        return float(s)
    except Exception:
        return math.nan


def to_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def finite_mean(rows, key):
    vals = [to_float(row.get(key)) for row in rows]
    vals = [v for v in vals if math.isfinite(v)]
    if not vals:
        return math.nan
    return sum(vals) / float(len(vals))


def max_finite(rows, key):
    vals = [to_float(row.get(key)) for row in rows]
    vals = [v for v in vals if math.isfinite(v)]
    if not vals:
        return 0.0
    return max(vals)


def ratio(rows, predicate):
    if not rows:
        return math.nan
    return sum(1 for row in rows if predicate(row)) / float(len(rows))


def front_state(row):
    if str(row.get("safety_mode", "")).strip() == "passthrough":
        return "pass"
    return str(row.get("front_state", row.get("state", ""))).strip()


def summarize_episode(episode_id, rows):
    return {
        "episode": episode_id,
        "safety_pass_ratio": ratio(rows, lambda r: front_state(r) == "pass"),
        "safety_slow_ratio": ratio(rows, lambda r: front_state(r) == "slow"),
        "safety_stop_ratio": ratio(rows, lambda r: front_state(r) == "stop"),
        "mean_linear_ratio": finite_mean(rows, "linear_ratio"),
        "mean_angular_ratio": finite_mean(rows, "angular_ratio"),
        "angular_intervention_ratio": ratio(rows, lambda r: to_bool(r.get("angular_intervened", False))),
        "reverse_request_ratio": ratio(rows, lambda r: to_bool(r.get("reverse_requested", False))),
        "reverse_allowed_ratio": ratio(rows, lambda r: to_bool(r.get("reverse_allowed", False))),
        "max_continuous_slow_duration": max_finite(rows, "continuous_slow_duration"),
        "max_continuous_stop_duration": max_finite(rows, "continuous_stop_duration"),
        "front_center_min_mean": finite_mean(rows, "front_center_min"),
        "left_turn_min_mean": finite_mean(rows, "left_turn_min"),
        "right_turn_min_mean": finite_mean(rows, "right_turn_min"),
        "rear_min_mean": finite_mean(rows, "rear_min"),
    }


def main():
    parser = argparse.ArgumentParser(description="Summarize safety throttle debug CSV.")
    parser.add_argument("--in", dest="in_csv", required=True, help="Input safety_throttle_debug.csv")
    parser.add_argument("--out", dest="out_csv", default=None, help="Output summary CSV")
    parser.add_argument("--episode-col", default="episode", help="Episode column name if present")
    args = parser.parse_args()

    in_csv = Path(os.path.expanduser(args.in_csv)).resolve()
    out_csv = Path(os.path.expanduser(args.out_csv)).resolve() if args.out_csv else in_csv.with_name("safety_throttle_summary.csv")

    groups = defaultdict(list)
    with open(in_csv, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        has_episode = reader.fieldnames is not None and args.episode_col in reader.fieldnames
        for row in reader:
            episode = row.get(args.episode_col, "0") if has_episode else "0"
            groups[episode].append(row)

    summaries = [summarize_episode(ep, rows) for ep, rows in sorted(groups.items(), key=lambda item: str(item[0]))]
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in summaries:
            writer.writerow(row)

    print("[OK] wrote %s" % out_csv)


if __name__ == "__main__":
    main()
