#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Subscribe to /sderi/safety_throttle_debug and write a CSV log.
"""

import argparse
import csv
import json
import os
from pathlib import Path

import rospy
from std_msgs.msg import String


DEFAULT_COLUMNS = [
    "stamp",
    "state",
    "front_min",
    "scan_stamp",
    "eri_rule",
    "d_stop_eff",
    "d_slow_eff",
    "throttle_ratio",
    "nominal_linear_x",
    "nominal_angular_z",
    "safe_linear_x",
    "safe_angular_z",
]


class SafetyThrottleDebugLogger(object):
    def __init__(self, out_csv: str, topic: str = "/sderi/safety_throttle_debug"):
        self.out_csv = Path(os.path.expanduser(out_csv)).resolve()
        self.topic = topic
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)

        self.f = open(self.out_csv, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.f, fieldnames=DEFAULT_COLUMNS)
        self.writer.writeheader()
        self.f.flush()

        self.count = 0
        self.sub = rospy.Subscriber(self.topic, String, self._cb, queue_size=100)

        rospy.loginfo("[safety-throttle-logger] Listening to %s", self.topic)
        rospy.loginfo("[safety-throttle-logger] Writing to %s", str(self.out_csv))

    def _cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[safety-throttle-logger] Failed to parse JSON: %s", str(e))
            return

        row = {}
        for col in DEFAULT_COLUMNS:
            val = payload.get(col, "")
            row[col] = "" if val is None else val

        try:
            self.writer.writerow(row)
            self.f.flush()
            self.count += 1
            if self.count % 50 == 0:
                rospy.loginfo(
                    "[safety-throttle-logger] wrote %d rows to %s",
                    self.count,
                    str(self.out_csv)
                )
        except Exception as e:
            rospy.logwarn("[safety-throttle-logger] Failed to write row: %s", str(e))

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="~/catkin_arena/safety_throttle_debug.csv")
    parser.add_argument("--topic", default="/sderi/safety_throttle_debug")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("safety_throttle_debug_logger", anonymous=False)
    logger = SafetyThrottleDebugLogger(args.out, args.topic)
    rospy.on_shutdown(logger.close)
    rospy.spin()


if __name__ == "__main__":
    main()
