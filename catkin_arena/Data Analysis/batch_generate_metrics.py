#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_generate_metrics_fixed.py

批量遍历子文件夹，读取仿真产生的 CSV（episode.csv, scan.csv, odom.csv, cmd_vel.csv, start_goal.csv, params.yaml）
对每个 episode 做分析并在对应文件夹生成 metrics.csv。

改进：
- 在读取阶段用 parse_to_float_list / parse_odom_cell 将字符串解析为数值列表 / 字典，兼容类似 "[2.662" 等异常格式
- 可选 --debug 打印每个 episode 的 path length（默认关）
- 出错时跳过该文件夹并继续处理其他文件夹
"""

from pathlib import Path
import argparse
import ast
import json
import os
from collections import defaultdict, Counter

import numpy as np
import pandas as pd
import yaml
import rospkg

# 必需文件
REQUIRED_FILES = [
    "cmd_vel.csv", "episode.csv", "odom.csv", "params.yaml", "scan.csv", "start_goal.csv"
]

# ---------- 辅助解析函数 ----------
def parse_to_float_list(cell):
    """把 csv 单元解析为 float 列表，容错各种字符串格式（比如 '[2.662', \"[ '2.662', 3 ]\" 等）"""
    if cell is None:
        return []
    # 如果已经是 list/tuple/ndarray，尝试把元素转 float
    if isinstance(cell, (list, tuple, np.ndarray)):
        out = []
        for e in cell:
            try:
                out.append(float(e))
            except Exception:
                s = str(e).strip().lstrip("[").rstrip("]").strip().strip("'\"")
                try:
                    out.append(float(s))
                except Exception:
                    # 跳过无法解析的元素
                    continue
        return out

    s = str(cell).strip()
    if s == "" or s.lower() in ("nan", "none"):
        return []

    # 尝试 JSON 解析（把单引号替换成双引号）
    try:
        if (s.startswith("[") and s.endswith("]")) or (s.startswith("{") and s.endswith("}")):
            val = json.loads(s.replace("'", '"'))
            if isinstance(val, list):
                return [float(x) for x in val]
    except Exception:
        pass

    # 尝试 ast.literal_eval（可以处理单引号）
    try:
        val = ast.literal_eval(s)
        if isinstance(val, (list, tuple, np.ndarray)):
            out = []
            for x in val:
                try:
                    out.append(float(x))
                except Exception:
                    try:
                        out.append(float(str(x).strip().lstrip("[").rstrip("]").strip()))
                    except Exception:
                        continue
            return out
        if isinstance(val, (int, float)):
            return [float(val)]
    except Exception:
        pass

    # 兜底：按逗号分割并清理方括号/引号
    parts = [p.strip().strip("[]'\"") for p in s.split(",") if p.strip() != ""]
    out = []
    for p in parts:
        try:
            out.append(float(p))
        except Exception:
            q = p.lstrip("[").rstrip("]")
            try:
                out.append(float(q))
            except Exception:
                continue
    return out

def parse_odom_cell(cell):
    """
    解析 odom.csv 中 data 单元，预期结构类似 {"position":[x,y,theta], "velocity":[vx,vy,vtheta]}
    返回字典；若解析失败返回空 dict
    """
    if cell is None:
        return {}
    if isinstance(cell, dict):
        # ensure numeric lists
        out = {}
        for k, v in cell.items():
            if isinstance(v, (list, tuple, np.ndarray)):
                out[k] = [float(x) for x in v]
            else:
                out[k] = v
        return out
    s = str(cell).strip()
    if s == "" or s.lower() in ("nan", "none"):
        return {}
    # 尝试 json
    try:
        val = json.loads(s.replace("'", '"'))
        if isinstance(val, dict):
            # 转成纯数字列表
            for kk, vv in val.items():
                if isinstance(vv, (list, tuple, np.ndarray)):
                    val[kk] = [float(x) for x in vv]
            return val
    except Exception:
        pass
    # 尝试 ast.literal_eval
    try:
        val = ast.literal_eval(s)
        if isinstance(val, dict):
            for kk, vv in val.items():
                if isinstance(vv, (list, tuple, np.ndarray)):
                    val[kk] = [float(x) for x in vv]
            return val
    except Exception:
        pass
    return {}

# ---------- 常量 / 枚举 ----------
class Action:
    STOP = "STOP"
    ROTATE = "ROTATE"
    MOVE = "MOVE"

class DoneReason:
    TIMEOUT = "TIMEOUT"
    GOAL_REACHED = "GOAL_REACHED"
    COLLISION = "COLLISION"

class Config:
    TIMEOUT_TRESHOLD = 180e9
    MAX_COLLISIONS = 3
    WARNING_CLEARANCE = 0.20
    DANGER_CLEARANCE = 0.10
    CRITICAL_CLEARANCE = 0.05
    PRE_COLLISION_WINDOWS_S = (1.0, 2.0)

# ---------- Metrics 核心实现 ----------
class MetricsGenerator:
    def __init__(self, folder: Path, debug=False):
        self.folder = folder
        self.debug = debug
        self.robot_params = self.get_robot_params(folder)
        # 读取并解析所有 CSV
        self.load_csvs()
        # 合并并分析
        self.generate_metrics()

    def load_csvs(self):
        f = self.folder
        try:
            self.episode = pd.read_csv(f / "episode.csv", low_memory=False)
        except Exception as e:
            raise RuntimeError(f"read episode.csv failed: {e}")

        # scan.csv -> laserscan（每 cell -> list[float]）
        try:
            scan_df = pd.read_csv(f / "scan.csv", low_memory=False)
            if "data" in scan_df.columns:
                scan_df = scan_df.rename(columns={"data": "laserscan"})
            if "laserscan" not in scan_df.columns:
                raise RuntimeError("scan.csv missing data/laserscan column")
            scan_df["laserscan"] = scan_df["laserscan"].apply(parse_to_float_list)
            self.scan = scan_df
        except Exception as e:
            raise RuntimeError(f"read scan.csv failed: {e}")

        # odom.csv -> odom (dict per row)
        try:
            odom_df = pd.read_csv(f / "odom.csv", low_memory=False)
            if "data" in odom_df.columns:
                odom_df = odom_df.rename(columns={"data": "odom"})
            if "odom" not in odom_df.columns:
                raise RuntimeError("odom.csv missing data/odom column")
            odom_df["odom"] = odom_df["odom"].apply(parse_odom_cell)
            self.odom = odom_df
        except Exception as e:
            raise RuntimeError(f"read odom.csv failed: {e}")

        # cmd_vel.csv -> cmd_vel (list per row)
        try:
            cmd_df = pd.read_csv(f / "cmd_vel.csv", low_memory=False)
            if "data" in cmd_df.columns:
                cmd_df = cmd_df.rename(columns={"data": "cmd_vel"})
            if "cmd_vel" not in cmd_df.columns:
                raise RuntimeError("cmd_vel.csv missing data/cmd_vel column")
            # parse each cell to list of floats (or empty list)
            cmd_df["cmd_vel"] = cmd_df["cmd_vel"].apply(parse_to_float_list)
            self.cmd_vel = cmd_df
        except Exception as e:
            raise RuntimeError(f"read cmd_vel.csv failed: {e}")

        # start_goal.csv -> start, goal (list per row)
        try:
            sg_df = pd.read_csv(f / "start_goal.csv", low_memory=False)
            for col in ("start", "goal"):
                if col not in sg_df.columns:
                    raise RuntimeError("start_goal.csv missing start/goal columns")
                sg_df[col] = sg_df[col].apply(parse_to_float_list)
            self.start_goal = sg_df
        except Exception as e:
            raise RuntimeError(f"read start_goal.csv failed: {e}")

        # 合并数据（与原脚本使用 pandas.concat join="inner" 一致）
        try:
            frames = [self.episode, self.scan, self.odom, self.cmd_vel, self.start_goal]
            data = pd.concat(frames, axis=1, join="inner")
            data = data.loc[:, ~data.columns.duplicated()].copy()
            self.data = data
        except Exception as e:
            raise RuntimeError(f"concat data failed: {e}")

    def generate_metrics(self):
        episode_data = {}
        i = 0
        while True:
            current_episode = self.data[self.data["episode"] == i]
            # 原脚本以 len <= 5 作为终止条件
            if len(current_episode) <= 5:
                break
            episode_data[i] = self.analyze_episode(current_episode, i)
            i += 1

        if len(episode_data) == 0:
            out_df = pd.DataFrame()
            out_df.to_csv(self.folder / "metrics.csv", index=False)
            print(f"[WARN] no episodes found in {self.folder}, wrote empty metrics.csv")
            return

        data = pd.DataFrame(episode_data).transpose().set_index("episode")
        data.to_csv(self.folder / "metrics.csv")
        print(f"[OK] Wrote metrics.csv for {self.folder}")

    def analyze_episode(self, episode: pd.DataFrame, index: int):
        positions = []
        velocities = []

        for od in episode["odom"]:
            if isinstance(od, dict):
                pos = od.get("position", [0.0, 0.0, 0.0])
                vel = od.get("velocity", [0.0, 0.0, 0.0])
            else:
                pos = [0.0, 0.0, 0.0]
                vel = [0.0, 0.0, 0.0]
            positions.append(np.array(pos, dtype=float))
            velocities.append(np.array(vel, dtype=float))

        curvature, normalized_curvature = self.get_curvature(np.array(positions))
        roughness = self.get_roughness(np.array(positions))

        vel_absolute = self.get_velocity_abs(velocities)
        acceleration = self.get_acceleration(vel_absolute)
        jerk = self.get_jerk(vel_absolute)

        # episode["laserscan"] 已在读取阶段解析为 list[float]
        laser_scans_raw = episode["laserscan"].tolist()
        # 保证传入 get_collisions 的是 list-of-lists 或 numpy array
        clean_scans = []
        for s in laser_scans_raw:
            if isinstance(s, (list, tuple, np.ndarray)):
                row = []
                for e in s:
                    try:
                        row.append(float(e))
                    except Exception:
                        try:
                            row.append(float(str(e).strip().strip("[]'\"")))
                        except Exception:
                            continue
                clean_scans.append(row)
            else:
                # 尝试解析字符串
                clean_scans.append(parse_to_float_list(s))

        robot_radius = float(self.robot_params.get("robot_radius", 0.3))

        collisions, collision_amount = self.get_collisions(
            np.array(clean_scans, dtype=object),
            robot_radius
        )
        min_scan_series = self.get_min_scan_series(clean_scans)
        clearance_series = self.get_clearance_series(min_scan_series, robot_radius)
        risk_metrics = self.get_risk_metrics(clearance_series)

        path_length, path_length_per_step = self.get_path_length(positions)

        time = int(list(episode["time"])[-1] - list(episode["time"])[0])

        start_position = self.get_mean_position(episode, "start")
        goal_position = self.get_mean_position(episode, "goal")

        action_types = list(self.get_action_type(episode["cmd_vel"]))

        collision_event_indices = self.get_collision_event_indices(collisions)

        collision_event_context = self.get_collision_event_context(
            collision_event_indices,
            positions,
            clean_scans,
            clearance_series,
            vel_absolute,
            acceleration,
            action_types,
            list(episode["time"]),
            start_position,
            goal_position,
        )

        phase_counter = Counter([c["phase"] for c in collision_event_context])
        action_counter = Counter([c["action"] for c in collision_event_context])

        first_collision_progress = (
            collision_event_context[0]["progress"]
            if len(collision_event_context) > 0 else np.nan
        )

        first_collision_dist_to_goal = (
            collision_event_context[0]["dist_to_goal"]
            if len(collision_event_context) > 0 else np.nan
        )
        approach_speeds = [
            c.get("collision_approach_speed", np.nan)
            for c in collision_event_context
            if np.isfinite(c.get("collision_approach_speed", np.nan))
        ]
        pre_warning_frames = [
            c.get("collision_pre_warning_frames", 0)
            for c in collision_event_context
            if np.isfinite(c.get("collision_pre_warning_frames", 0))
        ]

        if self.debug:
            print(f"[DEBUG] {self.folder} episode {index} PATH LENGTH {path_length}")

        return {
            "curvature": MetricsGenerator.round_values(curvature),
            "normalized_curvature": MetricsGenerator.round_values(normalized_curvature),
            "roughness": MetricsGenerator.round_values(roughness),
            "path_length_values": MetricsGenerator.round_values(path_length_per_step),
            "path_length": path_length,
            "acceleration": MetricsGenerator.round_values(acceleration),
            "jerk": MetricsGenerator.round_values(jerk),
            "velocity": MetricsGenerator.round_values(vel_absolute),
            "min_scan": MetricsGenerator.round_values(min_scan_series),
            "clearance": MetricsGenerator.round_values(clearance_series),
            "collision_amount": collision_amount,
            "collisions": list(collisions),
            "path": [list(p) for p in positions],
            "angle_over_length": self.get_angle_over_length(path_length, positions),
            "action_type": action_types,
            "time_diff": time,
            "time": list(map(int, episode["time"].tolist())),
            "episode": index,
            "result": self.get_success(time, collision_amount),
            "cmd_vel": list(map(list, episode["cmd_vel"].to_list())),
            "goal": goal_position,
            "start": start_position,
            "collision_event_indices": collision_event_indices,
            "collision_event_context": collision_event_context,
            "near_collision_event_indices": risk_metrics["near_collision_event_indices"],
            "warning_event_indices": risk_metrics["warning_event_indices"],
            "danger_event_indices": risk_metrics["danger_event_indices"],
            "critical_event_indices": risk_metrics["critical_event_indices"],

            "collision_event_count": len(collision_event_indices),
            "min_clearance": risk_metrics["min_clearance"],
            "mean_clearance": risk_metrics["mean_clearance"],
            "clearance_p05": risk_metrics["clearance_p05"],
            "near_collision_frame_count": risk_metrics["near_collision_frame_count"],
            "near_collision_event_count": risk_metrics["near_collision_event_count"],
            "near_collision_duration_ratio": risk_metrics["near_collision_duration_ratio"],
            "warning_frame_count": risk_metrics["warning_frame_count"],
            "warning_event_count": risk_metrics["warning_event_count"],
            "warning_duration_ratio": risk_metrics["warning_duration_ratio"],
            "danger_frame_count": risk_metrics["danger_frame_count"],
            "danger_event_count": risk_metrics["danger_event_count"],
            "danger_duration_ratio": risk_metrics["danger_duration_ratio"],
            "critical_frame_count": risk_metrics["critical_frame_count"],
            "critical_event_count": risk_metrics["critical_event_count"],
            "critical_duration_ratio": risk_metrics["critical_duration_ratio"],
            "collision_approach_speed": (
                np.nan if len(approach_speeds) == 0
                else float(np.mean(approach_speeds))
            ),
            "collision_pre_warning_frames": (
                0 if len(pre_warning_frames) == 0
                else int(np.max(pre_warning_frames))
            ),

            "collision_start_count": int(phase_counter.get("start", 0)),
            "collision_middle_count": int(phase_counter.get("middle", 0)),
            "collision_near_goal_count": int(phase_counter.get("near_goal", 0)),

            "collision_move_count": int(action_counter.get(Action.MOVE, 0)),
            "collision_rotate_count": int(action_counter.get(Action.ROTATE, 0)),
            "collision_stop_count": int(action_counter.get(Action.STOP, 0)),

            "first_collision_progress": first_collision_progress,
            "first_collision_dist_to_goal": first_collision_dist_to_goal,
        }

    def get_mean_position(self, episode, key):
        positions = episode[key].to_list()
        counter = {}
        for p in positions:
            key_s = ":".join([str(x) for x in p])
            counter[key_s] = counter.get(key_s, 0) + 1
        sorted_positions = dict(sorted(counter.items(), key=lambda x: x))
        first = list(sorted_positions.keys())[0]
        return [float(r) for r in first.split(":")]

    def get_position_for_collision(self, collisions, positions):
        for i, collision in enumerate(collisions):
            collisions[i][2] = positions[collision[0]]
        return collisions

    def get_angle_over_length(self, path_length, positions):
        total_yaw = 0.0
        for i in range(len(positions)-1):
            yaw_val = positions[i][2]
            next_yaw = positions[i+1][2]
            total_yaw += abs(next_yaw - yaw_val)
        if path_length == 0:
            return 0.0
        return total_yaw / path_length

    def get_success(self, time, collisions):
        if time >= Config.TIMEOUT_TRESHOLD:
            return DoneReason.TIMEOUT
        if collisions >= Config.MAX_COLLISIONS:
            return DoneReason.COLLISION
        return DoneReason.GOAL_REACHED

    def get_path_length(self, positions):
        path_length = 0.0
        path_length_per_step = []
        for i in range(len(positions)-1):
            step = float(np.linalg.norm(positions[i] - positions[i+1]))
            path_length_per_step.append(step)
            path_length += step
        return path_length, path_length_per_step

    def get_collisions(self, laser_scans, lower_bound):
        collisions = []
        collisions_marker = []
        for i, scan in enumerate(laser_scans):
            try:
                arr = np.array(scan, dtype=float)
                is_collision = np.any(arr <= lower_bound)
            except Exception:
                is_collision = False
            collisions_marker.append(int(is_collision))
            if is_collision:
                collisions.append(i)

        collision_amount = 1 if len(collisions_marker) > 0 and collisions_marker[0] == 1 else 0
        for i in range(1, len(collisions_marker)):
            prev = collisions_marker[i-1]
            cur = collisions_marker[i]
            if cur - prev > 0:
                collision_amount += 1

        return collisions, collision_amount

    def get_min_scan_series(self, scans):
        min_values = []
        for scan in scans:
            try:
                arr = np.array(scan, dtype=float)
                arr = arr[np.isfinite(arr)]
                min_values.append(float(np.min(arr)) if arr.size > 0 else np.nan)
            except Exception:
                min_values.append(np.nan)
        return min_values

    def get_clearance_series(self, min_scan_series, robot_radius):
        clearances = []
        for min_scan in min_scan_series:
            try:
                clearances.append(float(min_scan) - float(robot_radius))
            except Exception:
                clearances.append(np.nan)
        return clearances

    def get_risk_event_indices(self, clearance_series, threshold):
        events = []
        prev_risky = False
        for idx, clearance in enumerate(clearance_series):
            risky = bool(np.isfinite(clearance) and clearance <= threshold)
            if risky and not prev_risky:
                events.append(idx)
            prev_risky = risky
        return events

    def get_risk_metrics(self, clearance_series):
        valid = np.array([c for c in clearance_series if np.isfinite(c)], dtype=float)
        total_frames = len(clearance_series)

        def count_frames(threshold):
            return int(sum(1 for c in clearance_series if np.isfinite(c) and c <= threshold))

        warning_frames = count_frames(Config.WARNING_CLEARANCE)
        danger_frames = count_frames(Config.DANGER_CLEARANCE)
        critical_frames = count_frames(Config.CRITICAL_CLEARANCE)

        warning_events = self.get_risk_event_indices(clearance_series, Config.WARNING_CLEARANCE)
        danger_events = self.get_risk_event_indices(clearance_series, Config.DANGER_CLEARANCE)
        critical_events = self.get_risk_event_indices(clearance_series, Config.CRITICAL_CLEARANCE)

        return {
            "min_clearance": float(np.min(valid)) if valid.size > 0 else np.nan,
            "mean_clearance": float(np.mean(valid)) if valid.size > 0 else np.nan,
            "clearance_p05": float(np.percentile(valid, 5)) if valid.size > 0 else np.nan,
            "near_collision_frame_count": warning_frames,
            "near_collision_event_count": len(warning_events),
            "near_collision_duration_ratio": float(warning_frames / total_frames) if total_frames > 0 else np.nan,
            "near_collision_event_indices": warning_events,
            "warning_frame_count": warning_frames,
            "warning_event_count": len(warning_events),
            "warning_duration_ratio": float(warning_frames / total_frames) if total_frames > 0 else np.nan,
            "warning_event_indices": warning_events,
            "danger_frame_count": danger_frames,
            "danger_event_count": len(danger_events),
            "danger_duration_ratio": float(danger_frames / total_frames) if total_frames > 0 else np.nan,
            "danger_event_indices": danger_events,
            "critical_frame_count": critical_frames,
            "critical_event_count": len(critical_events),
            "critical_duration_ratio": float(critical_frames / total_frames) if total_frames > 0 else np.nan,
            "critical_event_indices": critical_events,
        }

    def get_action_type(self, actions):
        action_type = []
        for action in actions:
            try:
                a = list(action)
            except Exception:
                a = [0,0,0]
            if sum(a) == 0:
                action_type.append(Action.STOP)
            elif a[0] == 0 and a[1] == 0:
                action_type.append(Action.ROTATE)
            else:
                action_type.append(Action.MOVE)
        return action_type

    def get_curvature(self, positions):
        curvature_list = []
        normalized_list = []
        for i in range(len(positions)-2):
            first = positions[i]
            second = positions[i+1]
            third = positions[i+2]
            curv, norm = MetricsGenerator.calc_curvature(first, second, third)
            curvature_list.append(curv)
            normalized_list.append(norm)
        return curvature_list, normalized_list

    def get_roughness(self, positions):
        roughness_list = []
        for i in range(len(positions)-2):
            first = positions[i]
            second = positions[i+1]
            third = positions[i+2]
            roughness_list.append(MetricsGenerator.calc_roughness(first, second, third))
        return roughness_list

    def get_velocity_abs(self, velocities):
        return [float((i**2 + j**2)**0.5) for i,j,z in velocities]

    def get_acceleration(self, vel_abs):
        return [vel_abs[i+1] - vel_abs[i] for i in range(len(vel_abs)-1)]

    def get_jerk(self, vel_abs):
        jerk_list = []
        for i in range(len(vel_abs)-2):
            jerk_list.append(MetricsGenerator.calc_jerk(vel_abs[i], vel_abs[i+1], vel_abs[i+2]))
        return jerk_list
    
    def get_collision_event_indices(self, collision_indices):
        """
        Convert all collision frames into rising-edge collision events.

        Example:
            [10, 11, 12, 50, 51] -> [10, 50]
        """
        events = []
        prev = None

        for idx in collision_indices:
            idx = int(idx)
            if prev is None or idx > prev + 1:
                events.append(idx)
            prev = idx

        return events
    
    def get_collision_event_context(
        self,
        event_indices,
        positions,
        scans,
        clearance_series,
        velocities,
        accelerations,
        action_types,
        times,
        start_position,
        goal_position,
    ):
        contexts = []

        start_xy = np.array(start_position[:2], dtype=float)
        goal_xy = np.array(goal_position[:2], dtype=float)
        total_dist = float(np.linalg.norm(goal_xy - start_xy))
        total_dist = max(total_dist, 1e-6)

        for idx in event_indices:
            if idx < 0 or idx >= len(positions):
                continue

            pos = np.array(positions[idx], dtype=float)
            xy = pos[:2]

            dist_to_goal = float(np.linalg.norm(goal_xy - xy))
            progress = float(np.clip(1.0 - dist_to_goal / total_dist, 0.0, 1.0))

            if progress < 0.25:
                phase = "start"
            elif progress < 0.75:
                phase = "middle"
            else:
                phase = "near_goal"

            try:
                scan_arr = np.array(scans[idx], dtype=float)
                min_scan = float(np.nanmin(scan_arr)) if scan_arr.size > 0 else np.nan
            except Exception:
                min_scan = np.nan

            try:
                action = str(action_types[idx])
            except Exception:
                action = "UNKNOWN"

            try:
                t = int(times[idx])
            except Exception:
                t = -1

            pre_windows = self.get_pre_collision_window_stats(
                idx,
                times,
                clearance_series,
                velocities,
                accelerations,
                action_types,
            )

            context = {
                "idx": int(idx),
                "time": t,
                "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                "dist_to_goal": dist_to_goal,
                "progress": progress,
                "phase": phase,
                "min_scan": min_scan,
                "clearance": float(clearance_series[idx]) if idx < len(clearance_series) and np.isfinite(clearance_series[idx]) else np.nan,
                "action": action,
            }
            context.update(pre_windows)
            contexts.append(context)

        return contexts

    def get_pre_collision_window_stats(
        self,
        idx,
        times,
        clearance_series,
        velocities,
        accelerations,
        action_types,
    ):
        stats = {}
        time_s = self.normalize_times_to_seconds(times)
        end_t = time_s[idx] if idx < len(time_s) else np.nan

        for window_s in Config.PRE_COLLISION_WINDOWS_S:
            label = f"{int(window_s)}s"
            indices = self.get_window_indices(idx, time_s, end_t, window_s)
            clearances = [clearance_series[i] for i in indices if i < len(clearance_series) and np.isfinite(clearance_series[i])]
            vels = [velocities[i] for i in indices if i < len(velocities) and np.isfinite(velocities[i])]
            accels = [abs(accelerations[i]) for i in indices if i < len(accelerations) and np.isfinite(accelerations[i])]
            actions = [str(action_types[i]) for i in indices if i < len(action_types)]
            action_counts = Counter(actions)
            action_total = sum(action_counts.values())

            stats[f"pre_{label}_min_clearance"] = float(np.min(clearances)) if len(clearances) > 0 else np.nan
            stats[f"pre_{label}_mean_speed"] = float(np.mean(vels)) if len(vels) > 0 else np.nan
            stats[f"pre_{label}_max_abs_accel"] = float(np.max(accels)) if len(accels) > 0 else np.nan
            stats[f"pre_{label}_move_ratio"] = (
                float(action_counts.get(Action.MOVE, 0) / action_total) if action_total > 0 else np.nan
            )
            stats[f"pre_{label}_rotate_ratio"] = (
                float(action_counts.get(Action.ROTATE, 0) / action_total) if action_total > 0 else np.nan
            )
            stats[f"pre_{label}_stop_ratio"] = (
                float(action_counts.get(Action.STOP, 0) / action_total) if action_total > 0 else np.nan
            )

        two_s_key = "pre_2s_mean_speed"
        one_s_key = "pre_1s_mean_speed"
        stats["collision_approach_speed"] = stats.get(
            two_s_key,
            stats.get(one_s_key, np.nan)
        )
        stats["collision_pre_warning_frames"] = self.count_contiguous_risk_before(
            idx, clearance_series, Config.WARNING_CLEARANCE
        )
        stats["collision_pre_danger_frames"] = self.count_contiguous_risk_before(
            idx, clearance_series, Config.DANGER_CLEARANCE
        )
        return stats

    def get_window_indices(self, idx, time_s, end_t, window_s):
        if idx < 0:
            return []
        if not np.isfinite(end_t):
            return list(range(max(0, idx - 1), idx + 1))
        start_t = end_t - window_s
        return [
            i for i in range(0, idx + 1)
            if i < len(time_s) and np.isfinite(time_s[i]) and start_t <= time_s[i] <= end_t
        ]

    def count_contiguous_risk_before(self, idx, clearance_series, threshold):
        count = 0
        for i in range(idx, -1, -1):
            if i >= len(clearance_series):
                continue
            clearance = clearance_series[i]
            if not np.isfinite(clearance) or clearance > threshold:
                break
            count += 1
        return count

    @staticmethod
    def normalize_times_to_seconds(times):
        out = []
        for t in times:
            try:
                out.append(float(t))
            except Exception:
                out.append(np.nan)
        finite = [t for t in out if np.isfinite(t)]
        if len(finite) == 0:
            return out
        scale = 1.0
        max_t = max(finite)
        if max_t > 1e12:
            scale = 1e9
        elif max_t > 1e9:
            scale = 1e9
        elif max_t > 1e6:
            scale = 1e3
        return [t / scale if np.isfinite(t) else np.nan for t in out]

    @staticmethod
    def calc_curvature(first, second, third):
        triangle_area = MetricsGenerator.calc_triangle_area(first, second, third)
        divisor = (
            np.abs(np.linalg.norm(first - second)) *
            np.abs(np.linalg.norm(second - third)) *
            np.abs(np.linalg.norm(third - first))
        )
        if divisor == 0:
            return 0, 0
        curvature = 4 * triangle_area / divisor
        normalized = curvature * (np.abs(np.linalg.norm(first - second)) + np.abs(np.linalg.norm(second - third)))
        return curvature, normalized

    @staticmethod
    def calc_triangle_area(first, second, third):
        return 0.5 * np.abs(
            first[0] * (second[1] - third[1]) +
            second[0] * (third[1] - first[1]) +
            third[0] * (first[1] - second[1])
        )

    @staticmethod
    def calc_roughness(first, second, third):
        denom = np.abs(np.linalg.norm(third - first))
        if denom == 0:
            return 0.0
        triangle_area = MetricsGenerator.calc_triangle_area(first, second, third)
        return 2 * triangle_area / (denom ** 2)

    @staticmethod
    def calc_jerk(first, second, third):
        a1 = second - first
        a2 = third - second
        return float(np.abs(a2 - a1))

    @staticmethod
    def round_values(values, digits=3):
        return [round(float(v), digits) for v in values]

    @staticmethod
    def get_robot_params(dir_path: Path):
        params_file = dir_path / "params.yaml"
        try:
            with open(params_file, "r", encoding="utf-8") as f:
                content = yaml.safe_load(f)
            model = content.get("model", None)
            if model is None:
                print(f"[WARN] no model in {params_file}, defaulting robot_radius to 0.3")
                return {"robot_radius": 0.3}
            rp = rospkg.RosPack()
            base = os.path.join(rp.get_path("simulator_setup"), "robot", model, "model_params.yaml")
            with open(base, "r", encoding="utf-8") as ff:
                mp = yaml.safe_load(ff)
            return mp
        except Exception as e:
            print(f"[WARN] failed to read robot params for {dir_path}: {e}")
            return {"robot_radius": 0.3}

# ---------- 批量处理 ----------
def find_candidate_folders(root: Path, pattern: str = None):
    for p in root.rglob("*"):
        if p.is_dir():
            files = set([x.name for x in p.iterdir() if x.is_file()])
            if all(req in files for req in REQUIRED_FILES):
                if pattern is None or pattern in p.name:
                    yield p

def main():
    parser = argparse.ArgumentParser(description="Batch generate metrics.csv in subfolders")
    parser.add_argument("--root", "-r", default=".", help="root folder to search (default current dir)")
    parser.add_argument("--pattern", "-p", default=None, help="only process folders whose name contains this substring")
    parser.add_argument("--debug", action="store_true", help="print per-episode debug info (path length)")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    if not root.exists():
        print("root not found:", root)
        return

    folders = list(find_candidate_folders(root, pattern=args.pattern))
    if not folders:
        print("No candidate folders with required files found under", root)
        return

    print(f"Found {len(folders)} folders to process.")
    for folder in folders:
        print("Processing:", folder)
        try:
            MetricsGenerator(folder, debug=args.debug)
        except Exception as e:
            print(f"[ERROR] processing {folder}: {e}")

if __name__ == "__main__":
    main()
