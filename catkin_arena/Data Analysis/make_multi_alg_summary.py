#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_multi_alg_summary.py
一次性处理多个算法的 metrics.csv，返回每个算法一行的 across-episode mean（首列为算法名）。
支持命令行参数 --files 和可选 --names（用逗号分隔）。
"""
import ast
import math
import numpy as np
import pandas as pd
from collections import Counter, defaultdict
import os
import argparse

# 默认输出文件名（可用 --out_csv/--out_xlsx 覆盖）
DEFAULT_OUTPUT_CSV = "summary_multi_algs.csv"
DEFAULT_OUTPUT_XLSX = "summary_multi_algs.xlsx"

# 可能包含列表/数组的列名（用于 converters）
LIST_COLS = [
    "curvature", "normalized_curvature", "roughness", "path_length_values",
    "acceleration", "jerk", "velocity", "path", "action_type", "time",
    "cmd_vel", "collisions",

    "collision_event_indices",
    "collision_event_context",
    "min_scan",
    "clearance",
    "near_collision_event_indices",
    "warning_event_indices",
    "danger_event_indices",
    "critical_event_indices",
]

SAFETY_SUMMARY_COLS = {
    "min_clearance",
    "mean_clearance",
    "clearance_p05",
    "near_collision_frame_count",
    "near_collision_event_count",
    "near_collision_duration_ratio",
    "warning_frame_count",
    "warning_event_count",
    "warning_duration_ratio",
    "danger_frame_count",
    "danger_event_count",
    "danger_duration_ratio",
    "critical_frame_count",
    "critical_event_count",
    "critical_duration_ratio",
    "collision_event_count",
    "collision_approach_speed",
    "collision_pre_warning_frames",
}

def parse_list_cell(cell):
    """把 CSV 中看起来像列表的单元格解析为 Python list（鲁棒）"""
    if cell is None:
        return []
    if isinstance(cell, float) and np.isnan(cell):
        return []
    if isinstance(cell, (list, tuple, np.ndarray)):
        return list(cell)
    s = str(cell).strip()
    if s == "" or s.lower() == "nan":
        return []
    try:
        val = ast.literal_eval(s)
        if isinstance(val, (int, float, str)):
            return [val]
        return list(val)
    except Exception:
        try:
            inner = s
            if inner.startswith("[") and inner.endswith("]"):
                inner = inner[1:-1]
            parts = [p.strip().strip("'\"") for p in inner.split(",") if p.strip() != ""]
            return parts
        except Exception:
            return []

def to_seconds(v):
    """启发式把 time_diff 转为秒（支持纳秒/毫秒/秒的情况）"""
    try:
        v = float(v)
    except Exception:
        return np.nan
    if v > 1e12:
        return v/1e9
    if v > 1e9:
        return v/1e9
    if v > 1e6:
        return v/1e3
    return v

def cmd_stats_episode(cmd_list):
    """假设 cmd_list 中每项是 [vx, vy, omega] 或字符串表示"""
    linear_mags = []
    angs = []
    for e in cmd_list:
        try:
            if isinstance(e, str):
                e = parse_list_cell(e)
            if isinstance(e, (list, tuple, np.ndarray)):
                el = list(e)
            else:
                el = [e]
            if len(el) >= 2:
                lx = float(el[0]); ly = float(el[1])
                linear_mags.append(math.hypot(lx, ly))
            if len(el) >= 3:
                angs.append(abs(float(el[2])))
        except Exception:
            continue
    return (np.nan if not linear_mags else float(np.mean(linear_mags)),
            np.nan if not angs else float(np.mean(angs)))

def action_time_ratios_episode(actions, times, total_duration_s):
    acts = [str(a).upper() for a in actions]
    if len(acts) == 0:
        return {}
    if len(times) == len(acts) and len(times) >= 2:
        try:
            tnums = [float(t) for t in times]
            if max(tnums) > 1e9:
                tsecs = [t/1e9 for t in tnums]
            elif max(tnums) > 1e6:
                tsecs = [t/1e3 for t in tnums]
            else:
                tsecs = tnums
            diffs = np.diff(tsecs).tolist()
            last_piece = max(0.0, total_duration_s - tsecs[-1]) if (not np.isnan(total_duration_s)) else 0.0
            durations = diffs + [last_piece]
            dmap = defaultdict(float)
            for a, d in zip(acts, durations):
                dmap[a] += max(0.0, d)
            tot = sum(dmap.values())
            if tot > 0:
                return {k: v/tot for k,v in dmap.items()}
        except Exception:
            pass
    cnt = Counter(acts)
    totcnt = sum(cnt.values())
    return {k: v/totcnt for k,v in cnt.items()}

def g(row, col):
    """安全地获取单元格并保证返回 Python list（不会返回 ndarray）"""
    if col not in row:
        return []
    v = row[col]
    if v is None:
        return []
    if isinstance(v, float) and np.isnan(v):
        return []
    if isinstance(v, (list, tuple, np.ndarray)):
        return list(v)
    if isinstance(v, str):
        return parse_list_cell(v)
    return [v]

def scalar_float(row, col, default=np.nan):
    if col not in row:
        return default
    try:
        val = row.get(col, default)
        if pd.isna(val):
            return default
        return float(val)
    except Exception:
        return default

def process_metrics_df(df):
    """
    输入：pandas DataFrame（metrics.csv 已经用 converters 解析列表列）
    输出：字典 (单行) 包含 across-episode 的 mean（列名后缀为 _mean）
    """
    per_ep_stats = []
    for _, row in df.iterrows():
        velocity = [float(x) for x in g(row, "velocity")] if len(g(row,"velocity"))>0 else []
        acceleration = [float(x) for x in g(row, "acceleration")] if len(g(row,"acceleration"))>0 else []
        jerk = [float(x) for x in g(row, "jerk")] if len(g(row,"jerk"))>0 else []
        curvature = [float(x) for x in g(row, "curvature")] if len(g(row,"curvature"))>0 else []
        roughness = [float(x) for x in g(row, "roughness")] if len(g(row,"roughness"))>0 else []
        path_len = row.get("path_length", np.nan)
        time_diff_raw = row.get("time_diff", np.nan)
        duration_s = to_seconds(time_diff_raw)
        collisions = g(row, "collisions")
        action_type = g(row, "action_type")
        time_list = g(row, "time")
        cmd_vel = g(row, "cmd_vel")
        result = row.get("result", "")
        success = 1.0 if (isinstance(result, str) and "GOAL" in result.upper()) else 0.0

        ep_avg_vel = np.nan
        try:
            if (not pd.isna(path_len)) and (not np.isnan(duration_s)) and duration_s > 0:
                ep_avg_vel = float(path_len) / float(duration_s)
            elif len(velocity) > 0:
                ep_avg_vel = float(np.mean(velocity))
        except Exception:
            ep_avg_vel = np.nan

        ep_dict = {
            "episode": row.get("episode", None),
            "path_length": np.nan if pd.isna(path_len) else float(path_len),
            "duration_s": duration_s,
            "avg_speed_by_episode": ep_avg_vel,
            "velocity_mean": np.nan if len(velocity)==0 else float(np.mean(velocity)),
            "velocity_max": np.nan if len(velocity)==0 else float(np.max(velocity)),
            "accel_mean": np.nan if len(acceleration)==0 else float(np.mean(acceleration)),
            "jerk_mean": np.nan if len(jerk)==0 else float(np.mean(jerk)),
            "curvature_mean": np.nan if len(curvature)==0 else float(np.mean(curvature)),
            "curvature_std": np.nan if len(curvature)==0 else float(np.std(curvature)),
            "roughness_mean": np.nan if len(roughness)==0 else float(np.mean(roughness)),
            "collision_amount": (int(row.get("collision_amount")) if (str(row.get("collision_amount", "")).isdigit()) else (len(collisions) if len(collisions)>0 else 0)),
            "success": success,
            "path_point_count": len(g(row, "path")),
            "min_clearance": scalar_float(row, "min_clearance"),
            "mean_clearance": scalar_float(row, "mean_clearance"),
            "clearance_p05": scalar_float(row, "clearance_p05"),
            "near_collision_frame_count": scalar_float(row, "near_collision_frame_count", 0.0),
            "near_collision_event_count": scalar_float(row, "near_collision_event_count", 0.0),
            "near_collision_duration_ratio": scalar_float(row, "near_collision_duration_ratio"),
            "warning_frame_count": scalar_float(row, "warning_frame_count", 0.0),
            "warning_event_count": scalar_float(row, "warning_event_count", 0.0),
            "warning_duration_ratio": scalar_float(row, "warning_duration_ratio"),
            "danger_frame_count": scalar_float(row, "danger_frame_count", 0.0),
            "danger_event_count": scalar_float(row, "danger_event_count", 0.0),
            "danger_duration_ratio": scalar_float(row, "danger_duration_ratio"),
            "critical_frame_count": scalar_float(row, "critical_frame_count", 0.0),
            "critical_event_count": scalar_float(row, "critical_event_count", 0.0),
            "critical_duration_ratio": scalar_float(row, "critical_duration_ratio"),
            "collision_event_count": scalar_float(row, "collision_event_count", 0.0),
            "collision_approach_speed": scalar_float(row, "collision_approach_speed"),
            "collision_pre_warning_frames": scalar_float(row, "collision_pre_warning_frames", 0.0),
        }

        mlin, mang = cmd_stats_episode(cmd_vel)
        ep_dict["cmd_linear_mean"] = mlin
        ep_dict["cmd_angular_mean"] = mang

        ar = action_time_ratios_episode(action_type, time_list, duration_s)
        ep_dict["action_move_ratio"] = ar.get("MOVE", ar.get("MOV", 0.0))
        ep_dict["action_stop_ratio"] = ar.get("STOP", 0.0)
        ep_dict["action_rotate_ratio"] = ar.get("ROTATE", 0.0)

        per_ep_stats.append(ep_dict)

    ep_df = pd.DataFrame(per_ep_stats)

    # across-episode mean/std
    cols_to_mean = [c for c in ep_df.columns if c not in ("episode",)]
    result_dict = {}
    for c in cols_to_mean:
        try:
            values = ep_df[c].values
            # 计算 mean/std 时忽略 NaN
            result_dict[c + "_mean"] = float(np.nanmean(values)) if len(pd.Series(values).dropna())>0 else np.nan
            result_dict[c + "_std"] = float(np.nanstd(values)) if len(pd.Series(values).dropna())>1 else np.nan
        except Exception:
            result_dict[c + "_mean"] = np.nan
            result_dict[c + "_std"] = np.nan

    one_row = {
        k: v for k, v in result_dict.items()
        if k.endswith("_mean") or k[:-4] in SAFETY_SUMMARY_COLS
    }
    return one_row

def infer_name_from_path(p):
    base = os.path.basename(p)
    name, _ = os.path.splitext(base)
    return name

def main():
    parser = argparse.ArgumentParser(description="对多个算法的 metrics.csv 做 across-episode mean，输出每算法一行的表格。")
    parser.add_argument('--files', required=True, help="逗号分隔的 metrics.csv 文件路径列表（至少 1 个），例如 /a/metrics.csv,/b/metrics.csv")
    parser.add_argument('--names', default=None, help="可选，逗号分隔的算法名列表，数量应与 files 一致；若不提供将用文件名代替")
    parser.add_argument('--out_csv', default=DEFAULT_OUTPUT_CSV)
    parser.add_argument('--out_xlsx', default=DEFAULT_OUTPUT_XLSX)
    args = parser.parse_args()

    file_list = [p.strip() for p in args.files.split(",") if p.strip()!='']
    if len(file_list) == 0:
        print("未提供任何文件。")
        return

    name_list = None
    if args.names:
        name_list = [n.strip() for n in args.names.split(",")]
        if len(name_list) != len(file_list):
            print("警告：--names 提供的数量与 --files 不匹配，将使用文件名替代缺失项。")
            # pad or trim
            if len(name_list) < len(file_list):
                name_list += [infer_name_from_path(p) for p in file_list[len(name_list):]]
            else:
                name_list = name_list[:len(file_list)]
    else:
        name_list = [infer_name_from_path(p) for p in file_list]

    converters = {c: parse_list_cell for c in LIST_COLS}

    rows = []
    all_columns = set()
    for fpath, algname in zip(file_list, name_list):
        if not os.path.exists(fpath):
            print(f"文件不存在：{fpath} ，跳过。")
            continue
        try:
            df = pd.read_csv(fpath, converters=converters, low_memory=False)
        except Exception as e:
            print(f"读取 CSV 失败：{fpath} ，错误：{e} ，跳过。")
            continue
        one_row = process_metrics_df(df)
        # 插入算法名
        one_row_with_name = {"algorithm": algname}
        one_row_with_name.update(one_row)
        rows.append(one_row_with_name)
        all_columns.update(one_row_with_name.keys())

    if len(rows) == 0:
        print("没有成功处理任何文件。")
        return

    # 确保列顺序：先 algorithm，再按字母排序其余列（或者你可自定义列顺序）
    cols = ["algorithm"] + sorted([c for c in all_columns if c != "algorithm"])
    out_df = pd.DataFrame(rows, columns=cols)

    # 保存
    out_df.to_csv(args.out_csv, index=False)
    try:
        out_df.to_excel(args.out_xlsx, index=False)
    except Exception:
        pass

    pd.set_option('display.float_format', lambda x: '%.6g' % x)
    print("===== 各算法 across-episode mean（每行一个算法） =====")
    print(out_df)
    print(f"\n已保存：{args.out_csv} , {args.out_xlsx}（若 openpyxl 未安装可能只保存了 CSV）")

if __name__ == "__main__":
    main()
