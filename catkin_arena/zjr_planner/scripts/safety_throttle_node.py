#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ERI-conditioned safety throttle.

This node sits after the local planner and before the final /cmd_vel topic:

    planner raw cmd_vel -> safety_throttle_node -> /cmd_vel

It keeps the nominal angular command and only scales forward linear speed when
the front laser sector is close to obstacles.
"""

from __future__ import annotations

import copy
import json
import math
from typing import Iterable, Optional, Tuple

try:
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError:  # Allows pure throttle math to be checked without ROS.
    rospy = None
    Twist = None
    LaserScan = None
    String = None


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def effective_thresholds(eri: float,
                         d_stop: float,
                         d_slow: float,
                         eri_stop_gain: float,
                         eri_slow_gain: float) -> Tuple[float, float]:
    eri = clamp(float(eri), 0.0, 1.0)
    d_stop_eff = max(0.0, float(d_stop) + eri * float(eri_stop_gain))
    d_slow_eff = max(d_stop_eff, float(d_slow) + eri * float(eri_slow_gain))
    return d_stop_eff, d_slow_eff


def throttle_ratio(front_min: Optional[float],
                   eri: float,
                   d_stop: float,
                   d_slow: float,
                   eri_stop_gain: float,
                   eri_slow_gain: float,
                   max_linear_scale: float) -> Tuple[float, str, float, float]:
    d_stop_eff, d_slow_eff = effective_thresholds(
        eri, d_stop, d_slow, eri_stop_gain, eri_slow_gain
    )
    max_scale = clamp(float(max_linear_scale), 0.0, 1.0)

    if front_min is None or not math.isfinite(float(front_min)):
        return 0.0, "no_scan", d_stop_eff, d_slow_eff

    front = float(front_min)
    if front <= d_stop_eff:
        return 0.0, "stop", d_stop_eff, d_slow_eff
    if front < d_slow_eff:
        denom = max(1e-6, d_slow_eff - d_stop_eff)
        ratio = (front - d_stop_eff) / denom
        return clamp(ratio, 0.0, max_scale), "slow", d_stop_eff, d_slow_eff
    return max_scale, "pass", d_stop_eff, d_slow_eff


def compute_front_min(ranges: Iterable[float],
                      angle_min: float,
                      angle_increment: float,
                      front_angle_deg: float,
                      range_min: float = 0.0,
                      range_max: float = float("inf")) -> Optional[float]:
    if angle_increment == 0:
        return None

    half_angle = math.radians(max(0.0, float(front_angle_deg))) / 2.0
    vals = []
    for i, raw in enumerate(ranges):
        try:
            dist = float(raw)
        except Exception:
            continue
        if not math.isfinite(dist):
            continue
        if dist < range_min or dist > range_max:
            continue

        angle = float(angle_min) + i * float(angle_increment)
        if abs(angle) <= half_angle:
            vals.append(dist)

    if not vals:
        return None
    return min(vals)


def build_safe_twist(nominal,
                     ratio: float,
                     keep_reverse: bool = False,
                     angular_limit: float = 0.0):
    safe = copy.deepcopy(nominal)
    linear_x = float(nominal.linear.x)

    if linear_x >= 0.0 or not keep_reverse:
        safe.linear.x = linear_x * float(ratio)

    if angular_limit and angular_limit > 0.0:
        safe.angular.z = clamp(float(safe.angular.z), -float(angular_limit), float(angular_limit))

    return safe


class SafetyThrottleNode(object):
    def __init__(self):
        if rospy is None:
            raise RuntimeError("safety_throttle_node requires rospy at runtime")

        self.enabled = as_bool(rospy.get_param("~enabled", True))
        self.cmd_vel_in = rospy.get_param("~cmd_vel_in", "/sderi/cmd_vel_raw")
        self.cmd_vel_out = rospy.get_param("~cmd_vel_out", "/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.decision_debug_topic = rospy.get_param("~decision_debug_topic", "/sderi/decision_debug")
        self.debug_topic = rospy.get_param("~debug_topic", "/sderi/safety_throttle_debug")

        self.front_angle_deg = float(rospy.get_param("~front_angle_deg", 60.0))
        self.d_stop = float(rospy.get_param("~d_stop", 0.35))
        self.d_slow = float(rospy.get_param("~d_slow", 0.90))
        self.eri_stop_gain = float(rospy.get_param("~eri_stop_gain", 0.20))
        self.eri_slow_gain = float(rospy.get_param("~eri_slow_gain", 0.40))
        self.max_linear_scale = float(rospy.get_param("~max_linear_scale", 1.0))
        self.keep_reverse = as_bool(rospy.get_param("~keep_reverse", False))
        self.angular_limit = float(rospy.get_param("~angular_limit", 0.0))

        self.latest_front_min = None
        self.latest_scan_stamp = None
        self.latest_eri = 0.0

        self.pub_cmd = rospy.Publisher(self.cmd_vel_out, Twist, queue_size=10)
        self.pub_debug = rospy.Publisher(self.debug_topic, String, queue_size=20)

        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        self.sub_decision = rospy.Subscriber(self.decision_debug_topic, String, self._decision_cb, queue_size=20)
        self.sub_cmd = rospy.Subscriber(self.cmd_vel_in, Twist, self._cmd_cb, queue_size=10)

        rospy.loginfo(
            "[safety-throttle] enabled=%s in=%s out=%s scan=%s front=%.1fdeg "
            "d_stop=%.3f d_slow=%.3f eri_gains=(%.3f, %.3f)",
            str(self.enabled), self.cmd_vel_in, self.cmd_vel_out, self.scan_topic,
            self.front_angle_deg, self.d_stop, self.d_slow,
            self.eri_stop_gain, self.eri_slow_gain
        )

    def _scan_cb(self, msg):
        self.latest_front_min = compute_front_min(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.front_angle_deg,
            msg.range_min,
            msg.range_max,
        )
        self.latest_scan_stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

    def _decision_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.latest_eri = clamp(float(payload.get("eri_rule", 0.0)), 0.0, 1.0)
        except Exception:
            return

    def _cmd_cb(self, msg):
        if not self.enabled:
            self.pub_cmd.publish(msg)
            return

        ratio, state, d_stop_eff, d_slow_eff = throttle_ratio(
            self.latest_front_min,
            self.latest_eri,
            self.d_stop,
            self.d_slow,
            self.eri_stop_gain,
            self.eri_slow_gain,
            self.max_linear_scale,
        )
        safe = build_safe_twist(msg, ratio, self.keep_reverse, self.angular_limit)
        self.pub_cmd.publish(safe)

        try:
            payload = {
                "stamp": float(rospy.Time.now().to_sec()),
                "state": state,
                "front_min": float(self.latest_front_min) if self.latest_front_min is not None else None,
                "scan_stamp": float(self.latest_scan_stamp) if self.latest_scan_stamp is not None else None,
                "eri_rule": float(self.latest_eri),
                "d_stop_eff": float(d_stop_eff),
                "d_slow_eff": float(d_slow_eff),
                "throttle_ratio": float(ratio),
                "nominal_linear_x": float(msg.linear.x),
                "nominal_angular_z": float(msg.angular.z),
                "safe_linear_x": float(safe.linear.x),
                "safe_angular_z": float(safe.angular.z),
            }
            self.pub_debug.publish(json.dumps(payload))
        except Exception as e:
            rospy.logwarn("[safety-throttle] Failed to publish debug payload: %s", str(e))


def main():
    rospy.init_node("safety_throttle_node", anonymous=False)
    SafetyThrottleNode()
    rospy.spin()


if __name__ == "__main__":
    main()
