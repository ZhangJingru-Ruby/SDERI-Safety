#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Direction-aware execution safety governor.

This node sits after the local planner and before the final /cmd_vel topic:

    planner raw cmd_vel -> safety_throttle_node -> /cmd_vel

Supported modes:
    passthrough  : publish the nominal Twist unchanged
    linear_only  : throttle forward linear velocity from front clearance only
    turn_aware   : throttle forward velocity and direction-aware angular velocity
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
except ImportError:  # Allows pure governor math to be checked without ROS.
    rospy = None
    Twist = None
    LaserScan = None
    String = None


VALID_SAFETY_MODES = ("passthrough", "linear_only", "turn_aware")


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def clip_abs(value: float, max_abs: float) -> float:
    limit = abs(float(max_abs))
    return clamp(float(value), -limit, limit)


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _normalize_deg(angle_deg: float) -> float:
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def _angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    angle = _normalize_deg(angle_deg)
    start = _normalize_deg(start_deg)
    end = _normalize_deg(end_deg)
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


def min_valid_range_in_sector(scan, angle_start_deg: float, angle_end_deg: float) -> Optional[float]:
    """Return the minimum finite, in-range LaserScan distance within an angle sector."""
    if scan is None:
        return None
    angle_increment = float(getattr(scan, "angle_increment", 0.0))
    if angle_increment == 0.0:
        return None

    ranges = getattr(scan, "ranges", None)
    if ranges is None:
        return None

    range_min = float(getattr(scan, "range_min", 0.0))
    range_max = float(getattr(scan, "range_max", float("inf")))
    angle_min = float(getattr(scan, "angle_min", 0.0))

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

        angle_deg = math.degrees(angle_min + i * angle_increment)
        if _angle_in_sector(angle_deg, angle_start_deg, angle_end_deg):
            vals.append(dist)

    if not vals:
        return None
    return min(vals)


def compute_front_min(ranges: Iterable[float],
                      angle_min: float,
                      angle_increment: float,
                      front_angle_deg: float,
                      range_min: float = 0.0,
                      range_max: float = float("inf")) -> Optional[float]:
    class _Scan(object):
        pass

    scan = _Scan()
    scan.ranges = ranges
    scan.angle_min = angle_min
    scan.angle_increment = angle_increment
    scan.range_min = range_min
    scan.range_max = range_max
    half = max(0.0, float(front_angle_deg)) / 2.0
    return min_valid_range_in_sector(scan, -half, half)


def distance_scale(clearance: Optional[float], d_stop: float, d_slow: float) -> Tuple[float, str]:
    d_stop = float(d_stop)
    d_slow = float(d_slow)
    if d_slow <= d_stop:
        raise ValueError("distance_scale requires d_slow > d_stop")
    if clearance is None or not math.isfinite(float(clearance)):
        return 0.0, "no_scan"

    dist = float(clearance)
    if dist <= d_stop:
        return 0.0, "stop"
    if dist < d_slow:
        return (dist - d_stop) / (d_slow - d_stop), "slow"
    return 1.0, "pass"


def build_safe_twist(nominal,
                     ratio: float,
                     keep_reverse: bool = False,
                     angular_limit: float = 0.0):
    safe = copy.deepcopy(nominal)
    linear_x = float(nominal.linear.x)

    if linear_x >= 0.0 or not keep_reverse:
        safe.linear.x = linear_x * float(ratio)

    if angular_limit and angular_limit > 0.0:
        safe.angular.z = clip_abs(float(safe.angular.z), float(angular_limit))

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

        self.safety_mode = str(rospy.get_param("~safety_mode", "linear_only")).strip()
        if self.safety_mode not in VALID_SAFETY_MODES:
            raise ValueError("~safety_mode must be one of %s, got %r" % (VALID_SAFETY_MODES, self.safety_mode))

        self.allow_safe_reverse = as_bool(rospy.get_param("~allow_safe_reverse", False))

        # Legacy params are still accepted so older launch invocations keep their thresholds.
        legacy_d_stop = float(rospy.get_param("~d_stop", 0.35))
        legacy_d_slow = float(rospy.get_param("~d_slow", 0.90))
        self.d_stop_forward = float(rospy.get_param("~d_stop_forward", legacy_d_stop))
        self.d_slow_forward = float(rospy.get_param("~d_slow_forward", legacy_d_slow))

        self.d_stop_turn = float(rospy.get_param("~d_stop_turn", 0.25))
        self.d_slow_turn = float(rospy.get_param("~d_slow_turn", 0.45))
        self.omega_front_slow_cap = float(rospy.get_param("~omega_front_slow_cap", 0.40))
        self.omega_front_stop_cap = float(rospy.get_param("~omega_front_stop_cap", 0.20))

        self.max_reverse_speed = abs(float(rospy.get_param("~max_reverse_speed", 0.20)))
        self.d_stop_reverse = float(rospy.get_param("~d_stop_reverse", 0.30))
        self.d_slow_reverse = float(rospy.get_param("~d_slow_reverse", 0.50))

        self.front_center_start_deg = float(rospy.get_param("~front_center_start_deg", -30.0))
        self.front_center_end_deg = float(rospy.get_param("~front_center_end_deg", 30.0))
        self.left_turn_start_deg = float(rospy.get_param("~left_turn_start_deg", 15.0))
        self.left_turn_end_deg = float(rospy.get_param("~left_turn_end_deg", 100.0))
        self.right_turn_start_deg = float(rospy.get_param("~right_turn_start_deg", -100.0))
        self.right_turn_end_deg = float(rospy.get_param("~right_turn_end_deg", -15.0))
        self.rear_start_deg = float(rospy.get_param("~rear_start_deg", 150.0))
        self.rear_end_deg = float(rospy.get_param("~rear_end_deg", -150.0))

        self.eri_stop_gain = float(rospy.get_param("~eri_stop_gain", 0.0))
        self.eri_slow_gain = float(rospy.get_param("~eri_slow_gain", 0.0))

        self._validate_distance_params()

        self.latest_scan = None
        self.latest_scan_stamp = None
        self.latest_front_center_min = None
        self.latest_left_turn_min = None
        self.latest_right_turn_min = None
        self.latest_rear_min = None
        self.latest_eri = 0.0
        self.slow_since = None
        self.stop_since = None

        self.pub_cmd = rospy.Publisher(self.cmd_vel_out, Twist, queue_size=10)
        self.pub_debug = rospy.Publisher(self.debug_topic, String, queue_size=20)

        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        self.sub_decision = rospy.Subscriber(self.decision_debug_topic, String, self._decision_cb, queue_size=20)
        self.sub_cmd = rospy.Subscriber(self.cmd_vel_in, Twist, self._cmd_cb, queue_size=10)

        rospy.loginfo(
            "[safety-throttle] enabled=%s mode=%s allow_reverse=%s in=%s out=%s scan=%s "
            "forward=(%.3f, %.3f) turn=(%.3f, %.3f) reverse=(%.3f, %.3f) eri_gains=(%.3f, %.3f)",
            str(self.enabled), self.safety_mode, str(self.allow_safe_reverse),
            self.cmd_vel_in, self.cmd_vel_out, self.scan_topic,
            self.d_stop_forward, self.d_slow_forward,
            self.d_stop_turn, self.d_slow_turn,
            self.d_stop_reverse, self.d_slow_reverse,
            self.eri_stop_gain, self.eri_slow_gain
        )

    def _validate_distance_params(self):
        checks = [
            ("forward", self.d_stop_forward, self.d_slow_forward),
            ("turn", self.d_stop_turn, self.d_slow_turn),
            ("reverse", self.d_stop_reverse, self.d_slow_reverse),
        ]
        for name, d_stop, d_slow in checks:
            if d_slow <= d_stop:
                msg = "~d_slow_%s must be > ~d_stop_%s, got %.3f <= %.3f" % (
                    name, name, d_slow, d_stop
                )
                rospy.logerr("[safety-throttle] %s", msg)
                raise ValueError(msg)

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.latest_front_center_min = min_valid_range_in_sector(
            msg, self.front_center_start_deg, self.front_center_end_deg
        )
        self.latest_left_turn_min = min_valid_range_in_sector(
            msg, self.left_turn_start_deg, self.left_turn_end_deg
        )
        self.latest_right_turn_min = min_valid_range_in_sector(
            msg, self.right_turn_start_deg, self.right_turn_end_deg
        )
        self.latest_rear_min = min_valid_range_in_sector(
            msg, self.rear_start_deg, self.rear_end_deg
        )
        self.latest_scan_stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

    def _decision_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.latest_eri = clamp(float(payload.get("eri_rule", 0.0)), 0.0, 1.0)
        except Exception:
            return

    def _front_durations(self, front_state: str, now: float) -> Tuple[float, float]:
        if front_state == "slow":
            if self.slow_since is None:
                self.slow_since = now
            self.stop_since = None
        elif front_state == "stop":
            if self.stop_since is None:
                self.stop_since = now
            self.slow_since = None
        else:
            self.slow_since = None
            self.stop_since = None

        slow_duration = 0.0 if self.slow_since is None else max(0.0, now - self.slow_since)
        stop_duration = 0.0 if self.stop_since is None else max(0.0, now - self.stop_since)
        return slow_duration, stop_duration

    def _append_reason(self, reasons, state: str, prefix: str):
        if state == "no_scan":
            reasons.append("no_scan")
        elif state in ("slow", "stop"):
            reasons.append("%s_%s" % (prefix, state))

    def _select_reason(self, reasons):
        priority = [
            "no_scan",
            "front_stop",
            "left_turn_stop",
            "right_turn_stop",
            "rear_stop",
            "front_slow",
            "left_turn_slow",
            "right_turn_slow",
            "rear_slow",
        ]
        for item in priority:
            if item in reasons:
                return item
        return "none"

    def _cmd_cb(self, msg):
        if not self.enabled:
            self.pub_cmd.publish(msg)
            return

        now = float(rospy.Time.now().to_sec())
        safe = copy.deepcopy(msg)
        nominal_linear_x = float(msg.linear.x)
        nominal_angular_z = float(msg.angular.z)

        front_center_min = self.latest_front_center_min
        left_turn_min = self.latest_left_turn_min
        right_turn_min = self.latest_right_turn_min
        rear_min = self.latest_rear_min

        linear_ratio = 1.0
        angular_ratio = 1.0
        reverse_ratio = 1.0
        front_state = "inactive"
        turn_state = "inactive"
        reverse_state = "inactive"
        turn_direction = "none"
        reverse_requested = nominal_linear_x < 0.0
        reverse_allowed = False
        reasons = []

        if self.safety_mode == "passthrough":
            safe.linear.x = nominal_linear_x
            safe.angular.z = nominal_angular_z
            reverse_allowed = reverse_requested
        else:
            linear_ratio, front_state = distance_scale(
                front_center_min, self.d_stop_forward, self.d_slow_forward
            )
            self._append_reason(reasons, front_state, "front")

            if nominal_linear_x > 0.0:
                safe.linear.x = nominal_linear_x * linear_ratio
            elif nominal_linear_x < 0.0:
                if self.allow_safe_reverse:
                    reverse_ratio, reverse_state = distance_scale(
                        rear_min, self.d_stop_reverse, self.d_slow_reverse
                    )
                    clipped_reverse = max(nominal_linear_x, -self.max_reverse_speed)
                    safe.linear.x = clipped_reverse * reverse_ratio
                    reverse_allowed = reverse_ratio > 0.0
                    self._append_reason(reasons, reverse_state, "rear")
                else:
                    reverse_ratio = 0.0
                    reverse_state = "blocked"
                    safe.linear.x = 0.0
                    reverse_allowed = False
            else:
                safe.linear.x = 0.0

            if self.safety_mode == "turn_aware":
                if nominal_angular_z > 0.0:
                    turn_direction = "left"
                    angular_ratio, turn_state = distance_scale(
                        left_turn_min, self.d_stop_turn, self.d_slow_turn
                    )
                    candidate_angular_z = nominal_angular_z * angular_ratio
                    self._append_reason(reasons, turn_state, "left_turn")
                elif nominal_angular_z < 0.0:
                    turn_direction = "right"
                    angular_ratio, turn_state = distance_scale(
                        right_turn_min, self.d_stop_turn, self.d_slow_turn
                    )
                    candidate_angular_z = nominal_angular_z * angular_ratio
                    self._append_reason(reasons, turn_state, "right_turn")
                else:
                    angular_ratio = 1.0
                    turn_state = "inactive"
                    candidate_angular_z = 0.0

                if front_state == "stop":
                    safe.angular.z = clip_abs(candidate_angular_z, self.omega_front_stop_cap)
                elif front_state == "slow":
                    safe.angular.z = clip_abs(candidate_angular_z, self.omega_front_slow_cap)
                else:
                    safe.angular.z = candidate_angular_z
            else:
                safe.angular.z = nominal_angular_z

        continuous_slow_duration, continuous_stop_duration = self._front_durations(front_state, now)
        scan_age = None if self.latest_scan_stamp is None else max(0.0, now - float(self.latest_scan_stamp))

        linear_intervened = abs(float(safe.linear.x) - nominal_linear_x) > 1e-6
        angular_intervened = abs(float(safe.angular.z) - nominal_angular_z) > 1e-6
        intervention_reason = self._select_reason(reasons)

        self.pub_cmd.publish(safe)

        try:
            payload = {
                "stamp": now,
                "state": front_state,
                "front_min": float(front_center_min) if front_center_min is not None else None,
                "scan_stamp": float(self.latest_scan_stamp) if self.latest_scan_stamp is not None else None,
                "eri_rule": float(self.latest_eri),
                "d_stop_eff": float(self.d_stop_forward),
                "d_slow_eff": float(self.d_slow_forward),
                "throttle_ratio": float(linear_ratio),
                "nominal_linear_x": nominal_linear_x,
                "nominal_angular_z": nominal_angular_z,
                "safe_linear_x": float(safe.linear.x),
                "safe_angular_z": float(safe.angular.z),
                "safety_mode": self.safety_mode,
                "front_center_min": float(front_center_min) if front_center_min is not None else None,
                "left_turn_min": float(left_turn_min) if left_turn_min is not None else None,
                "right_turn_min": float(right_turn_min) if right_turn_min is not None else None,
                "rear_min": float(rear_min) if rear_min is not None else None,
                "linear_ratio": float(linear_ratio),
                "angular_ratio": float(angular_ratio),
                "reverse_ratio": float(reverse_ratio),
                "front_state": front_state,
                "turn_state": turn_state,
                "reverse_state": reverse_state,
                "linear_intervened": bool(linear_intervened),
                "angular_intervened": bool(angular_intervened),
                "reverse_requested": bool(reverse_requested),
                "reverse_allowed": bool(reverse_allowed),
                "turn_direction": turn_direction,
                "intervention_reason": intervention_reason,
                "continuous_slow_duration": float(continuous_slow_duration),
                "continuous_stop_duration": float(continuous_stop_duration),
                "scan_age": float(scan_age) if scan_age is not None else None,
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
