#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sderi_decision_debug_logger.py

Subscribe to /sderi/decision_debug, parse JSON String messages,
and write them into a clean CSV file.

Usage:
    rosrun zjr_planner sderi_decision_debug_logger.py \
        --out ~/catkin_arena/sderi_decision_debug.csv

The topic is expected to be std_msgs/String, where msg.data is JSON.
"""

import os
import csv
import json
import argparse
from pathlib import Path

import rospy
from std_msgs.msg import String


DEFAULT_COLUMNS = [
    "stamp",

    "pose_x",
    "pose_y",
    "pose_yaw",

    "subgoal_x",
    "subgoal_y",

    "eri_rule",
    "band_idx",

    "t_replan",
    "H_k",

    "current_target_x_before",
    "current_target_y_before",

    "final_goal_x",
    "final_goal_y",

    "dist_to_subgoal",
    "dist_to_final_goal",
]


class SDERIDecisionDebugLogger:
    def __init__(self, out_csv: str, topic: str = "/sderi/decision_debug"):
        self.out_csv = Path(os.path.expanduser(out_csv)).resolve()
        self.topic = topic

        self.out_csv.parent.mkdir(parents=True, exist_ok=True)

        self.f = open(self.out_csv, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.f, fieldnames=DEFAULT_COLUMNS)
        self.writer.writeheader()
        self.f.flush()

        self.count = 0

        self.sub = rospy.Subscriber(
            self.topic,
            String,
            self._cb,
            queue_size=100
        )

        rospy.loginfo("[decision-debug-logger] Listening to %s", self.topic)
        rospy.loginfo("[decision-debug-logger] Writing to %s", str(self.out_csv))

    def _cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[decision-debug-logger] Failed to parse JSON: %s", str(e))
            return

        row = {}
        for col in DEFAULT_COLUMNS:
            val = payload.get(col, "")

            # Keep CSV clean.
            if val is None:
                val = ""
            row[col] = val

        try:
            self.writer.writerow(row)
            self.f.flush()
            self.count += 1

            if self.count % 10 == 0:
                rospy.loginfo(
                    "[decision-debug-logger] wrote %d rows to %s",
                    self.count,
                    str(self.out_csv)
                )

        except Exception as e:
            rospy.logwarn("[decision-debug-logger] Failed to write row: %s", str(e))

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
