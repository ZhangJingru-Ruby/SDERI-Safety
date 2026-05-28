#!/usr/bin/env python3

import os
import csv
import time
import json
import rospy
import rospkg
import yaml
from datetime import datetime
from std_msgs.msg import Int16, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion
import math


SAFETY_DEBUG_COLUMNS = [
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
    "safety_mode",
    "front_center_min",
    "left_turn_min",
    "right_turn_min",
    "rear_min",
    "linear_ratio",
    "angular_ratio",
    "reverse_ratio",
    "front_state",
    "turn_state",
    "reverse_state",
    "linear_intervened",
    "angular_intervened",
    "reverse_requested",
    "reverse_allowed",
    "turn_direction",
    "intervention_reason",
    "continuous_slow_duration",
    "continuous_stop_duration",
    "scan_age",
]

class DataCollector:
    def __init__(self, topic_name, label, msg_type):
        self.full_topic_name = topic_name
        self.data = None
        self.subscriber = rospy.Subscriber(topic_name, msg_type, self.callback)
        self.label = label

    def callback(self, msg):
        if self.label == "scan":
            self.data = [msg.range_max if math.isnan(val) else round(val, 3) for val in msg.ranges]
        elif self.label == "odom":
            pose3d = msg.pose.pose
            twist = msg.twist.twist
            self.data = {
                "position": [
                    round(pose3d.position.x, 3),
                    round(pose3d.position.y, 3),
                    round(euler_from_quaternion([
                        pose3d.orientation.x,
                        pose3d.orientation.y,
                        pose3d.orientation.z,
                        pose3d.orientation.w
                    ])[2], 3)
                ],
                "velocity": [
                    round(twist.linear.x, 3),
                    round(twist.linear.y, 3),
                    round(twist.angular.z, 3)
                ]
            }
        elif self.label == "cmd_vel":
            self.data = [
                round(msg.linear.x, 3),
                round(msg.linear.y, 3),
                round(msg.angular.z, 3)
            ]

    def get_data(self):
        return self.full_topic_name, self.data


class Recorder:
    def __init__(self):
        self.dir = rospkg.RosPack().get_path("arena-evaluation")
        self.result_dir = os.path.join(
            self.dir, "data",
            datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        ) + "_" + rospy.get_namespace().replace("/", "")

        os.makedirs(self.result_dir, exist_ok=True)

        self.write_params()

        # 只订阅有数据的三个 topic
        self.data_collectors = [
            DataCollector("/scan", "scan", LaserScan),
            DataCollector("/odom", "odom", Odometry),
            DataCollector("/cmd_vel", "cmd_vel", Twist)
        ]

        for dc in self.data_collectors:
            self.write_data(dc.label, ["time", "data"], mode="w")

        self.write_data("episode", ["time", "episode"], mode="w")
        self.write_data("start_goal", ["episode", "start", "goal"], mode="w")
        self.write_data("safety_throttle_debug", ["time", "episode"] + SAFETY_DEBUG_COLUMNS, mode="w")

        self.current_episode = 0
        self.config = self.read_config()
        self.current_time = None
        self.latest_safety_debug = None

        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.scenario_reset_sub = rospy.Subscriber("/scenario_reset", Int16, self.scenario_reset_callback)
        self.safety_debug_topic = rospy.get_param("~safety_throttle_debug_topic", "/sderi/safety_throttle_debug")
        self.safety_debug_sub = rospy.Subscriber(
            self.safety_debug_topic,
            String,
            self.safety_debug_callback,
            queue_size=100,
        )

    def scenario_reset_callback(self, data: Int16):
        self.current_episode = data.data

    def safety_debug_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest_safety_debug = payload
        except Exception:
            return

    def clock_callback(self, clock: Clock):
        current_simulation_time = clock.clock.secs * 1e9 + clock.clock.nsecs
        if self.current_time is None:
            self.current_time = current_simulation_time

        time_diff = (current_simulation_time - self.current_time) / 1e6  # ms
        if time_diff < self.config["record_frequency"]:
            return
        self.current_time = current_simulation_time

        for collector in self.data_collectors:
            topic_name, data = collector.get_data()
            if data is not None:
                self.write_data(collector.label, [self.current_time, data])

        self.write_data("episode", [self.current_time, self.current_episode])
        self.write_data("start_goal", [
            self.current_episode,
            rospy.get_param(rospy.get_namespace() + "start", [0, 0, 0]),
            rospy.get_param(rospy.get_namespace() + "goal", [0, 0, 0])
        ])

        if self.latest_safety_debug is not None:
            row = [self.current_time, self.current_episode]
            for col in SAFETY_DEBUG_COLUMNS:
                value = self.latest_safety_debug.get(col, "")
                row.append("" if value is None else value)
            self.write_data("safety_throttle_debug", row)

    def read_config(self):
        with open(os.path.join(self.dir, "data_recorder_config.yaml")) as f:
            return yaml.safe_load(f)

    def write_data(self, file_name, data, mode="a"):
        with open(os.path.join(self.result_dir, f"{file_name}.csv"), mode, newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def write_params(self):
        with open(os.path.join(self.result_dir, "params.yaml"), "w") as file:
            yaml.dump({
                "model": rospy.get_param(os.path.join(rospy.get_namespace(), "model"), ""),
                "map_file": rospy.get_param("/map_file", ""),
                "scenario_file": rospy.get_param("/scenario_file", ""),
                "local_planner": rospy.get_param(rospy.get_namespace() + "local_planner", ""),
                "agent_name": rospy.get_param(rospy.get_namespace() + "agent_name", ""),
                "namespace": rospy.get_namespace().replace("/", "")
            }, file)


if __name__ == "__main__":
    rospy.init_node("data_recorder", anonymous=True)
    time.sleep(2)  # 等待其它节点启动
    Recorder()
    rospy.spin()
