#!/usr/bin/env python3
# monitor_and_override.py
"""
使用 multiprocessing 将 pipeline 放到子进程中执行的完整实现。

行为简述：
- 每 check_period 秒检查一次：机器人是否到达 current_target（<= dist_thresh）
  或者自目标分配已超过 time_thresh（T>1s）。
- 若满足则通过 processing_lock 进入 processing_worker（主进程），此时暂停周期判断。
- processing_worker 会在独立的子进程中执行 pipeline（RRT+merge+RuleBand），
  子进程返回 (x_sub, y_sub) -> 主进程发送短时 override（override_sub_dur），
  并将 current_target 更新为该子目标；释放锁，恢复周期检查。
- pipeline 在子进程内部以“无 rospy”方式运行（避免在子进程中初始化 ROS）。
"""

import os
import time
import math
import subprocess
import warnings
import argparse
import json
import sys
import threading
import multiprocessing
from std_msgs.msg import String

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

# pipeline modules (确保这些模块在你的环境可 import)
# 注意：子进程也需要这些模块可 import（且运行环境与主进程相同或兼容）
from pointcloud_generate import generate_pointcloud_grid
from merge_data import merge_start_and_paths
from ruleband_api import RuleBandAPI

# action / message imports
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_srvs.srv import Empty

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")


# ------------------ 子进程用的 pipeline（不依赖 rospy，使用 print） ------------------
# 这些函数在子进程里被 import/调用，不能使用 rospy.loginfo 等 ROS API。


def find_rrt_script_no_ros():
    candidates = [
        "~/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "/home/robot/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "./rrt_path_generate.py",
    ]
    for p in candidates:
        p_exp = os.path.expanduser(p)
        if os.path.exists(p_exp):
            return os.path.abspath(p_exp)
    return None


def run_rrt_once_no_ros(goal_x, goal_y, out_file=None,
                        num_paths=3, step=0.3, max_iters=4000,
                        start_x=None, start_y=None):
    """
    Launch rrt_path_generate.py in a subprocess.

    Important:
    - If start_x/start_y are provided, pass them to RRT via --start.
    - This keeps RRT path, pointcloud local map, and merge start aligned.
    """
    rrt_script = find_rrt_script_no_ros()
    if rrt_script is None:
        raise FileNotFoundError("Cannot find rrt_path_generate.py; set correct path in find_rrt_script_no_ros()")

    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    cmd = [
        "python3", rrt_script,
        "--goal", str(float(goal_x)), str(float(goal_y)),
        "--num_paths", str(int(num_paths)),
        "--step", str(float(step)),
        "--max_iters", str(int(max_iters))
    ]

    if start_x is not None and start_y is not None:
        cmd.extend(["--start", str(float(start_x)), str(float(start_y))])

    if out_file:
        cmd.extend(["--out", out_file])

    print("[pipeline-child] Running RRT:", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)

    if res.stdout and res.stdout.strip():
        print("[pipeline-child] RRT stdout:", res.stdout.strip())
    if res.stderr and res.stderr.strip():
        print("[pipeline-child] RRT stderr:", res.stderr.strip())

    if res.returncode != 0:
        raise RuntimeError(
            "RRT script failed (rc=%d)\nstdout:\n%s\nstderr:\n%s"
            % (res.returncode, res.stdout, res.stderr)
        )

    return out_file


def pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y):
    print("[pipeline-child] Saving pointcloud...")
    saved_file = generate_pointcloud_grid(index=5)
    print("[pipeline-child] Pointcloud saved to:", saved_file)

    out_paths = os.path.expanduser(
        "~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json"
    )

    run_rrt_once_no_ros(
        goal_x,
        goal_y,
        out_file=out_paths,
        start_x=start_x,
        start_y=start_y
    )

    merged_out = os.path.expanduser(
        "~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/example_data.json"
    )

    merged_file = merge_start_and_paths(
        (float(start_x), float(start_y)),
        goal=(float(goal_x), float(goal_y)),
        out_path=merged_out,
        data_start_path=saved_file,
        paths_path=out_paths
    )

    api = RuleBandAPI(device="cpu")
    res = api.predict_from_file(merged_file, debug=True)

    # Backward-compatible parsing
    if isinstance(res, (list, tuple)):
        x_sub0 = res[0]
        y_sub0 = res[1]
        eri_rule = res[2] if len(res) > 2 else None
        band_idx = res[3] if len(res) > 3 else None
    else:
        x_sub0, y_sub0 = res
        eri_rule = None
        band_idx = None

    x_sub = float(x_sub0)
    y_sub = float(y_sub0)

    print("[pipeline-child] Sampled subgoal -> (%.3f, %.3f), eri_rule=%s, band_idx=%s" %
        (x_sub, y_sub, str(eri_rule), str(band_idx)))

    return x_sub, y_sub, eri_rule, band_idx


def pipeline_child_entry(start_x, start_y, goal_x, goal_y, q: multiprocessing.Queue):
    """
    子进程入口：显式使用独立 child ROS node 名，避免抢父节点名。
    """
    try:
        import sys
        sys.argv = [sys.argv[0]]

        try:
            import rospy
            if not rospy.core.is_initialized():
                rospy.init_node(
                    "sderi_pipeline_child",
                    anonymous=True,
                    disable_signals=True,
                    argv=[]
                )
                print("[pipeline-child] Child ROS node up as:", rospy.get_name())
        except Exception as e:
            print("[pipeline-child] Child rospy init skipped/failed:", e)

        x, y, eri_rule, band_idx = pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y)
        q.put(("ok", float(x), float(y), eri_rule, band_idx))
    except Exception as e:
        import traceback
        tb = traceback.format_exc()
        q.put(("err", str(e) + "\n" + tb))


# ------------------ 主进程中的 ROS 交互函数 ------------------


def get_robot_pose(timeout=5.0):
    """从 /odom 读取机器人位姿 (x,y,yaw)。超时返回 None。"""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /odom (%.1fs)" % timeout)
        return None
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(x), float(y), float(yaw)


def send_action_goal_and_cancel(x, y, yaw=0.0, duration=0.5, wait_server=3.0):
    """
    使用 move_base action 发送 goal 并在 duration 秒后 cancel_all_goals。
    返回 True 表示 action server 存在并已发送/取消，False 表示 action server 不可用。
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server (%.1fs)..." % wait_server)
    if not client.wait_for_server(rospy.Duration(wait_server)):
        rospy.logwarn("move_base action server not available after %.1fs" % wait_server)
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo("Sending action goal to move_base: x=%.3f y=%.3f yaw=%.3f" % (x, y, yaw))
    client.send_goal(goal)

    start = time.time()
    try:
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    rospy.loginfo("Cancelling move_base goal(s) after %.3fs" % duration)
    try:
        client.cancel_all_goals()
    except Exception as e:
        rospy.logwarn("cancel_all_goals() raised: %s" % e)

    rospy.sleep(0.12)
    return True


def fallback_publish_goal_once(x, y, yaw=0.0, topic="/move_base_simple/goal", wait_conn=1.0):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=False)
    start_t = time.time()
    while pub.get_num_connections() == 0 and time.time() - start_t < wait_conn and not rospy.is_shutdown():
        rospy.sleep(0.02)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.pose = Pose(Point(float(x), float(y), 0.0), Quaternion(q[0], q[1], q[2], q[3]))
    pub.publish(goal)
    rospy.loginfo("Published fallback pose to %s: x=%.3f y=%.3f" % (topic, x, y))
    return True


# ------------------ MonitorNode（主逻辑） ------------------

class MonitorNode(object):
    def __init__(self,
                 check_period=0.2,
                 dist_thresh=0.4,
                 time_thresh=1.0,
                 override_sub_dur=0.5,
                 final_goal=(1.73, -9.57),
                 pipeline_timeout=30.0):
        # parameters
        self.check_period = float(check_period)
        self.dist_thresh = float(dist_thresh)
        self.time_thresh = float(time_thresh)
        self.override_sub_dur = float(override_sub_dur)
        self.final_goal = final_goal
        self.pipeline_timeout = float(pipeline_timeout)

        # runtime state
        self.current_target = (float(self.final_goal[0]), float(self.final_goal[1]))
        self.target_assigned_time = rospy.get_time()
        self.processing_lock = threading.Lock()  # 在处理时阻止周期判断
        self.timer = None

        # latest robot pose from /odom
        self.latest_pose = None   # (x,y,yaw)
        self.latest_pose_time = 0.0

        # subscribe odom to keep pose up-to-date
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)

        # decision debug publisher
        self.pub_decision_debug = rospy.Publisher(
            "/sderi/decision_debug",
            String,
            queue_size=10
        )

        # start periodic timer
        self.timer = rospy.Timer(rospy.Duration(self.check_period), self._timer_cb)

        # Adaptive replan params
        self.t_min = float(rospy.get_param("~t_min", 0.30))
        self.t_max = float(rospy.get_param("~t_max", 1.20))
        self.gamma = float(rospy.get_param("~gamma", 2.0))
        self.control_dt = float(rospy.get_param("~control_dt", 0.05))

        rospy.loginfo(
            "Adaptive replan params: t_min=%.3f t_max=%.3f gamma=%.3f control_dt=%.3f",
            self.t_min, self.t_max, self.gamma, self.control_dt
        )

        rospy.loginfo("MonitorNode initialized: check_period=%.3fs dist_thresh=%.3fm time_thresh=%.3fs final_goal=(%.3f,%.3f) pipeline_timeout=%.1fs",
                      self.check_period, self.dist_thresh, self.time_thresh, self.final_goal[0], self.final_goal[1], self.pipeline_timeout)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_pose = (float(p.position.x), float(p.position.y), float(yaw))
        self.latest_pose_time = rospy.get_time()

    def _distance_to_target(self, pose, target):
        dx = pose[0] - target[0]
        dy = pose[1] - target[1]
        return math.hypot(dx, dy)
    
    def _compute_t_replan(self, eri_rule):
        """
        Paper-style adaptive replanning time:

            t_replan = t_min + (t_max - t_min) * (1 - xi)^gamma

        Larger xi -> shorter commitment.
        Smaller xi -> longer commitment.
        """
        try:
            xi = float(eri_rule)
            if not math.isfinite(xi):
                xi = 0.0
        except Exception:
            xi = 0.0

        xi = max(0.0, min(1.0, xi))

        t_replan = self.t_min + (self.t_max - self.t_min) * ((1.0 - xi) ** self.gamma)
        t_replan = max(self.t_min, min(self.t_max, t_replan))

        h_min = int(math.ceil(self.t_min / self.control_dt))
        h_max = int(math.floor(self.t_max / self.control_dt))
        h_k = int(math.floor(t_replan / self.control_dt))
        h_k = max(h_min, min(h_max, h_k))

        return t_replan, h_k, xi

    def _timer_cb(self, event):
        # 跳过若已有处理在进行
        if self.processing_lock.locked():
            return

        if not self.latest_pose:
            rospy.logdebug("No odom yet; skipping check")
            return

        dist = self._distance_to_target(self.latest_pose, self.current_target)
        elapsed = rospy.get_time() - self.target_assigned_time if self.target_assigned_time else float("inf")

        rospy.logdebug("Check target dist=%.3f (thresh=%.3f), elapsed=%.3f (thresh=%.3f)",
                       dist, self.dist_thresh, elapsed, self.time_thresh)

        trigger = False
        if dist <= self.dist_thresh:
            rospy.loginfo("Within distance threshold: dist=%.3f <= %.3f -> trigger processing", dist, self.dist_thresh)
            trigger = True
        elif elapsed >= self.time_thresh:
            rospy.loginfo("Target elapsed time exceeded: elapsed=%.3f >= %.3f -> trigger processing", elapsed, self.time_thresh)
            trigger = True

        if not trigger:
            return

        got = self.processing_lock.acquire(False)
        if not got:
            return

        # 进入处理，重置计时标志（用 0 表示处理中）
        self.target_assigned_time = 0.0

        # 启动处理线程（线程内部会启动子进程来执行 pipeline）
        t = threading.Thread(target=self._processing_worker, args=(), daemon=True)
        t.start()

    def _processing_worker(self):
        """
        Worker 在主进程线程中执行（以便与 rospy/actionlib 交互）。
        它会：
          - 启动子进程运行 pipeline（隔离 CPU/GPU 工作）
          - 等待子进程返回 (x_sub, y_sub) 或超时
          - 以 action 或 fallback 发布子目标并短时保持 override_sub_dur
          - 更新 current_target = (x_sub,y_sub)，并设置 target_assigned_time = now
          - 释放 processing_lock（恢复周期检查）
        """
        try:
            rospy.loginfo("Processing worker started (launch pipeline in child process).")

            # 读取最近缓存的位姿（如果没有就 wait）
            if self.latest_pose:
                cur_x, cur_y, cur_yaw = self.latest_pose
            else:
                pose = get_robot_pose(timeout=3.0)
                if pose is None:
                    rospy.logerr("Cannot read odom for processing; aborting.")
                    return
                cur_x, cur_y, cur_yaw = pose

            rospy.loginfo("Current pose: x=%.3f y=%.3f yaw=%.3f" % (cur_x, cur_y, cur_yaw))

            # 使用 multiprocessing spawn 启动子进程来执行 pipeline
            ctx = multiprocessing.get_context('spawn')
            q = ctx.Queue()
            p = ctx.Process(target=pipeline_child_entry,
                            args=(cur_x, cur_y, self.final_goal[0], self.final_goal[1], q))
            p.start()

            rospy.loginfo("Pipeline child process started (pid=%s), waiting up to %.1fs..." % (str(p.pid), self.pipeline_timeout))

            got_result = False
            x_sub = y_sub = None
            eri_rule = None
            band_idx = None
            try:
                # 等待子进程通过队列返回数据（阻塞，带总超时）
                # 使用循环尝试从队列读取（以便捕获 queue.Empty）
                start_wait = time.time()
                timeout = float(self.pipeline_timeout)
                while time.time() - start_wait < timeout:
                    try:
                        if not q.empty():
                            tup = q.get_nowait()
                            if tup and isinstance(tup, (list, tuple)) and len(tup) >= 1:
                                tag = tup[0]
                                if tag == "ok":
                                    _, x_sub, y_sub = tup[:3]
                                    eri_rule = tup[3] if len(tup) > 3 else None
                                    band_idx = tup[4] if len(tup) > 4 else None
                                    got_result = True
                                    break
                                else:
                                    # err
                                    rospy.logerr("Pipeline child reported error: %s" % (tup[1] if len(tup) > 1 else "<no msg>"))
                                    break
                        # 若队列为空，短 sleep
                        time.sleep(0.1)
                    except Exception:
                        time.sleep(0.05)
                # 若没有从队列拿到但子进程已退出，也检查一下 exitcode 和 stdout/stderr not available here
                if not got_result:
                    # 尝试从队列再读一次（非阻塞）
                    try:
                        if not q.empty():
                            tup = q.get_nowait()
                            if tup and tup[0] == "ok":
                                _, x_sub, y_sub = tup[:3]
                                eri_rule = tup[3] if len(tup) > 3 else None
                                band_idx = tup[4] if len(tup) > 4 else None
                                got_result = True
                    except Exception:
                        pass
            finally:
                # 如果子进程还在运行且我们决定超时或已获得结果，确保结束子进程
                if p.is_alive():
                    # 如果已经拿到结果，优雅 join；否则 terminate
                    if got_result:
                        p.join(timeout=1.0)
                        if p.is_alive():
                            try:
                                p.terminate()
                            except Exception:
                                pass
                            p.join(timeout=1.0)
                    else:
                        rospy.logwarn("Pipeline child did not return in time (%.1fs). Terminating child." % self.pipeline_timeout)
                        try:
                            p.terminate()
                        except Exception as e:
                            rospy.logwarn("Error terminating pipeline child: %s" % e)
                        p.join(timeout=1.0)

            if not got_result or x_sub is None or y_sub is None:
                rospy.logerr("Pipeline failed or timed out; aborting processing worker.")
                return

            t_replan, h_k, xi = self._compute_t_replan(eri_rule)

            rospy.loginfo(
                "Pipeline produced subgoal: (%.3f, %.3f), eri_rule=%.3f, band_idx=%s, t_replan=%.3fs, H_k=%d",
                x_sub, y_sub, xi, str(band_idx), t_replan, h_k
            )

            # Publish structured decision debug info.
            # This is useful for offline analysis:
            # collision moment ↔ active subgoal / ERI / band / replan time.
            try:
                decision_payload = {
                    "stamp": float(rospy.Time.now().to_sec()),

                    "pose_x": float(cur_x),
                    "pose_y": float(cur_y),
                    "pose_yaw": float(cur_yaw),

                    "subgoal_x": float(x_sub),
                    "subgoal_y": float(y_sub),

                    "eri_rule": float(xi),
                    "band_idx": int(band_idx) if band_idx is not None else -1,

                    "t_replan": float(t_replan),
                    "H_k": int(h_k),

                    "current_target_x_before": float(self.current_target[0]),
                    "current_target_y_before": float(self.current_target[1]),

                    "final_goal_x": float(self.final_goal[0]),
                    "final_goal_y": float(self.final_goal[1]),

                    "dist_to_subgoal": float(math.hypot(float(x_sub) - float(cur_x),
                                                        float(y_sub) - float(cur_y))),

                    "dist_to_final_goal": float(math.hypot(float(self.final_goal[0]) - float(cur_x),
                                                        float(self.final_goal[1]) - float(cur_y))),
                }

                self.pub_decision_debug.publish(json.dumps(decision_payload))
            except Exception as e:
                rospy.logwarn("Failed to publish /sderi/decision_debug: %s", str(e))

            # 发送子目标，并按 ERI 自适应保持时间
            ok = send_action_goal_and_cancel(
                x_sub,
                y_sub,
                yaw=0.0,
                duration=t_replan,
                wait_server=3.0
            )
            if not ok:
                rospy.logwarn("Action server unavailable; fallback publish for subgoal.")
                fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)
                rospy.sleep(self.override_sub_dur)

            rospy.sleep(0.05)

            # 清理 costmaps（如果服务存在）
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
                clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_costmaps()
                rospy.loginfo("Called /move_base/clear_costmaps after override.")
            except Exception:
                pass

            # 更新 current_target 并重置计时
            self.current_target = (float(x_sub), float(y_sub))
            self.target_assigned_time = rospy.get_time()
            rospy.loginfo("Updated current_target to (%.3f, %.3f). Reset target_assigned_time." % (x_sub, y_sub))

            rospy.loginfo("Processing worker finished successfully.")

        finally:
            if self.processing_lock.locked():
                self.processing_lock.release()


# ------------------ scenario util ------------------

def read_final_goal_from_scenario(scenario_file):
    if not scenario_file:
        return None
    try:
        path = os.path.expanduser(scenario_file)
        with open(path, "r") as f:
            data = json.load(f)
        if isinstance(data, dict) and "robot_goal" in data:
            g = data["robot_goal"]
            if isinstance(g, (list, tuple)) and len(g) >= 2:
                return float(g[0]), float(g[1])
    except Exception as e:
        rospy.logwarn("Failed to read scenario_file '%s': %s" % (scenario_file, e))
    return None


# ------------------ main / CLI ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check_period", type=float, default=None, help="period (s) to check distance (default 0.2)")
    parser.add_argument("--dist_thresh", type=float, default=None, help="distance threshold (m), default 0.4")
    parser.add_argument("--time_thresh", type=float, default=None, help="time threshold (s) default 1.0")
    parser.add_argument("--override_sub_dur", type=float, default=None, help="seconds to hold override subgoal (default 0.5)")
    parser.add_argument("--pipeline_timeout", type=float, default=None, help="max seconds to wait for pipeline child (default 30)")
    parser.add_argument("--goal_x", type=float, default=None, help="final goal x override")
    parser.add_argument("--goal_y", type=float, default=None, help="final goal y override")
    parser.add_argument("--scenario_file", type=str, default=None, help="optional scenario json file")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("monitor_and_override_node", anonymous=True)

    check_period = float(rospy.get_param("~check_period", args.check_period if args.check_period is not None else 0.2))
    dist_thresh = float(rospy.get_param("~dist_thresh", args.dist_thresh if args.dist_thresh is not None else 0.4))
    time_thresh = float(rospy.get_param("~time_thresh", args.time_thresh if args.time_thresh is not None else 1.0))
    override_sub_dur = float(rospy.get_param("~override_sub_dur", args.override_sub_dur if args.override_sub_dur is not None else 0.5))
    pipeline_timeout = float(rospy.get_param("~pipeline_timeout", args.pipeline_timeout if args.pipeline_timeout is not None else 30.0))

    final_goal_x = rospy.get_param("~final_goal_x", args.goal_x if args.goal_x is not None else None)
    final_goal_y = rospy.get_param("~final_goal_y", args.goal_y if args.goal_y is not None else None)
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file if args.scenario_file is not None else None)

    if scenario_file:
        sgoal = read_final_goal_from_scenario(scenario_file)
        if sgoal:
            final_goal_x, final_goal_y = sgoal
            rospy.loginfo("Using robot_goal from scenario file: (%.3f, %.3f)" % (final_goal_x, final_goal_y))
        else:
            rospy.logwarn("scenario_file provided but robot_goal not found; using params/CLI for final goal")

    if final_goal_x is None or final_goal_y is None:
        rospy.logwarn("final_goal not fully specified; using default (1.73,-9.57)")
        final_goal = (1.73, -9.57)
    else:
        final_goal = (float(final_goal_x), float(final_goal_y))

    node = MonitorNode(check_period=check_period,
                       dist_thresh=dist_thresh,
                       time_thresh=time_thresh,
                       override_sub_dur=override_sub_dur,
                       final_goal=final_goal,
                       pipeline_timeout=pipeline_timeout)

    rospy.spin()
