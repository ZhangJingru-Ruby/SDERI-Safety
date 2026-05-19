## 3. 当前工程环境与总体架构

### 3.1 当前环境

- **OS**：Ubuntu 20.04
- **ROS**：ROS 1 / Noetic 生态
- **机器人模型**：Jackal
- **仿真环境**：Gazebo + Arena-rosnav-3D
- **常用 world**：`small_warehouse`

### 3.2 顶层启动链

当前主入口是：

```
start_arena_gazebo.launch
→ move_base_rosnav.launch
→ move_base
→ spacial_horizon_node
→ drl_local_planner
→ /cmd_vel
→ /velocity_redirect
→ /jackal_velocity_controller/cmd_vel
→ Gazebo / Jackal
```

其中：

- `move_base` 提供 costmap、goal 管理与传统导航基建
- `spacial_horizon_node` 负责高层局部目标与重规划节奏
- `drl_local_planner` 是当前真正的低层执行器
- `/velocity_redirect` 是控制指令重定向节点
- Jackal 通过 `/jackal_velocity_controller/cmd_vel` 执行控制

---

## 4. 当前系统的双层结构

### 4.1 上层：Subgoal Agent

负责：

- 在全局路径约束下生成安全子目标
- 根据 ERI 调度子目标分布与重规划时机
- 向下层发布 `/subgoal` 与 `/globalPlan`

### 4.2 下层：Motion Agent

负责：

- 接收 `/subgoal`
- 结合局部感知与机器人状态进行短时动作决策
- 输出 `/cmd_vel`
- 在高层设置的时间尺度内持续跟踪该 subgoal

当前下层 Motion Agent 由 `drl_local_planner` 承担，且实际是一个 **goal-conditioned local controller**。

---

## 5. 当前底层控制器的精确接口定义

### 5.1 节点入口

当前底层执行器节点为：

```
Node name: /drl_local_planner
Package: arena_local_planner_drl
Entry file: drl_agent_node.py
```

`move_base_rosnav.launch` 通过该节点启动 DRL local planner。

### 5.2 当前底层控制器的内部框架

当前底层控制器是一个**部署壳 + encoder backend** 的插件式结构：

```
drl_agent_node.py
→ BaseDRLAgent
→ ObservationCollector
→ EncoderFactory
→ RosnavEncoder (当前 Jackal backend)
→ PPO policy
→ encoder.get_action()
→ publish_action()
```

### 5.3 当前底层控制器的 ROS 输入

`/drl_local_planner` 当前订阅：

- `/scan` : `sensor_msgs/LaserScan`
- `/odom` : `nav_msgs/Odometry`
- `/subgoal` : `geometry_msgs/PoseStamped`
- `/globalPlan` : `nav_msgs/Path`
- `/tf`
- `/tf_static`
- `/clock`

同时依赖若干参数：

- `model`
- `agent_name`
- `trainings_environment`
- `network_type`
- `/action_frequency`
- `/train_mode`
- `/bool_goal_reached`
- `/robot_action_rate`
- `radius`
- `laser_beams`
- `laser_range`

### 5.4 当前底层控制器的观测定义

`ObservationCollector` 明确给出了当前 ROSNAV planner 的 merged observation：

```
merged_obs = [laser_scan, rho, theta]
```

其中：

- `laser_scan`：处理过 NaN/Inf 之后的激光数组
- `rho`：机器人到当前 `/subgoal` 的距离
- `theta`：当前 `/subgoal` 在机器人坐标系中的相对角度

此外，`obs_dict` 还保存了：

- `global_plan`
- `robot_pose`
- `subgoal`
- `robot_vel`
- `last_action`

但**当前 Jackal PPO policy 实际只使用了 `merged_obs + last_action`**。

因为 `hyperparameters.json` 中 `actions_in_observationspace = true`，而 `RosnavEncoder.get_observation()` 会把 `last_action` 拼到 merged observation 后面。

### 5.5 当前 policy 的实际输入维度

对于 Jackal，`rosnav_rosnav.py` 的注释表明激光束数通常为 **720**。因此当前 Jackal policy 的实际输入大概率为：

```
720 laser + rho + theta + 3D last_action = 725 维
```

即：

```
policy_input = [scan(720), rho, theta, last_action(3)]
```

### 5.6 当前动作定义

当前 Jackal 的动作空间由 `default_settings_jackal.yaml` 给出：

- `linear_range = [0, 2.0]`
- `angular_range = [-4.0, 4.0]`

当前策略输出为二维连续动作：

```
[v, w]
```

再由 `RosnavEncoder.get_action()` 映射成：

```
[v, 0, w]
```

最终通过 `publish_action()`转为：

- `Twist.linear.x = v`
- `Twist.linear.y = 0`
- `Twist.angular.z = w`

### 5.7 当前后端算法

当前 Jackal local planner 的模型后端不是 TensorFlow，而是 **Stable-Baselines3 的 PPO policy**，通过 `best_model.zip` 加载；如果开启归一化，还会额外加载 `vec_normalize.pkl`。当前 jackal 超参数配置还表明：

- `discrete_action_space = false`
- `normalize = false`
- `actions_in_observationspace = true`

因此，当前底层控制器可被精确定义为：

> **一个基于激光与相对子目标的、带 last_action 的连续动作 PPO 局部控制器。**
> 

---

## 6. 当前高层与低层之间的关键耦合接口

现在已经能明确，高层和低层最关键的握手接口就是：

```
/subgoal
```

`ObservationCollector` 会订阅 `/subgoal`，将其转换为 `Pose2D`，再根据当前机器人位姿计算 `rho, theta`。

**底层控制器并不直接追踪全局 goal，而是追踪高层下发的局部 subgoal**。

因此后续集成 TD3 时可以**保留现有 `/subgoal` 接口，不要破坏高层-低层之间已经跑通的握手方式。**

同时，`spacial_horizon_node` 在 `plan_fsm_param.yaml` 中被配置为：

- `use_drl = true`
- `subgoal_pub_period = 0.5`

说明当前系统本来就是：

- **高层以较低频率发布 subgoal**
- **低层以较高频率持续执行该 subgoal**

这种“高层慢、低层快”的时间尺度分离，也与开题中的 HRL/SMDP 建模完全一致。