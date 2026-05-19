好！我们现在不考虑重新训练 Motion Agent / TD3，只考虑：**PPO 继续当司机，我们在它后面加一个安全副驾驶。** 🚗🐾

你的系统现在本来就是：

```
Subgoal Agent → /subgoal
Motion Agent / drl_local_planner → /cmd_vel
velocity_redirect → Jackal controller
```

也就是说，最自然的插入点就是 `/cmd_vel` 后面，或者继续放在 `drl_local_planner` 内部做 action filter。你的底层文档也明确说，当前下层 Motion Agent 接收 `/subgoal`，结合局部感知输出 `/cmd_vel`，并且 `drl_local_planner` 是真正的底层执行器。

毕业论文底层项目框架文档

另外，你的小论文里已经承认 safe-region mask 只是 support-level safety prior，不保证执行阶段完全无碰撞，执行安全仍然依赖 local controller。

root

所以我们要补的是：

```
PPO: 我想这么开
Safety layer: 等等，这样会撞，慢点/换个近似安全速度/停下
```

* * *

# 总体结论

我把三个方案讲成幼儿园版本：

| 路线 | 幼稚解释 | 适合你吗 |
| --- | --- | --- |
| 1. Collision Monitor / Safety Throttle | 给机器人装“刹车片” | 最适合立刻做 |
| 2. Velocity Obstacle / DWA / velocity-space shield | 给机器人一个“小速度筛子” | 最适合替代当前 CBF |
| 3. Predictive Safety Filter / MPC Shield | 让机器人每步先“脑内演练未来 1 秒” | 最高级，但工程最重 |

如果我现在拍板：

```
第一优先级：Safety Throttle
第二优先级：Velocity-space predictive shield
第三优先级：轻量 MPC / Predictive Safety Filter
```

不要一上来搞第三个。第三个像给小车装飞控大脑，帅，但容易炸厨房 🧯。

* * *

# 方案 1：Collision Monitor / Safety Throttle

## 它到底在做什么？

超级幼稚版：

> 机器人前面放一个“透明泡泡”。  
> 泡泡里没人：正常开。  
> 泡泡里有人：慢点。  
> 人贴脸了：停车。

它不替 PPO 想路线，不替 PPO 转弯。它只做一件事：

```
危险近了 → 降低 v
特别危险 → v = 0
```

也就是：

```
PPO 输出: v=2.0, w=0.5
Safety throttle 看到前面太近:
输出: v=0.6, w=0.5
```

它通常不碰角速度 `w`，只管线速度 `v`。这对你很重要，因为你现在的 CBF gate 大问题之一就是改方向改太多，和 PPO 打架。

* * *

## 有没有人经常这么做？

**非常经常。** 这是工程机器人里很常见的安全层思想。

Nav2 里就有官方 `Collision Monitor`。它的设计目标就是作为额外安全层，直接根据传感器数据做 collision avoidance，并且绕过 costmap 和 trajectory planner，在 emergency-stop 层面监控潜在碰撞。官方文档明确说它支持 stop、slowdown、limit 等模型。[docs.nav2.org+1](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html?utm_source=chatgpt.com)

更关键的是，Nav2 的 `Collision Monitor` 设计上就是放在 controller 后面，当作 `cmd_vel` 过滤器：如果没有触发安全区域，就用原 controller 的 `cmd_vel`；如果触发了，就缩放速度或停车。[ROS Documentation](https://docs.ros.org/en/humble/p/nav2_collision_monitor/?utm_source=chatgpt.com)

这和你现在的系统超级像：

```
Nav2:
Controller Server cmd_vel → Collision Monitor → robot

你的系统:
drl_local_planner cmd_vel → Safety Throttle → velocity_redirect / Jackal
```

* * *

## 相关文献/系统在什么场景？

严格说，Collision Monitor 更像工程系统，不是一篇单独算法论文。但它背后对应的是工业移动机器人里很常见的 protective zone / safety field / emergency stop 思想。

典型场景：

```
仓储机器人
服务机器人
医院/商场移动机器人
室内动态人群
低速或中速移动底盘
```

Nav2 官方教程还专门展示了如何在 Nav2 stack 中配置 Collision Monitor，包括普通 polygon zone 和 velocity polygon。[docs.nav2.org](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html?utm_source=chatgpt.com)

* * *

## 它怎么 integrate 到导航框架里？

别人一般这么接：

```
Global planner → Local controller → Collision Monitor → robot base
```

它不是 planner，不生成路径，只是最后一层安全监督。

你这里可以接成：

```
drl_local_planner
  publishes /cmd_vel_raw

safety_throttle_node
  subscribes /cmd_vel_raw, /scan, /odom, /sderi/eri_act
  publishes /cmd_vel

velocity_redirect
  subscribes /cmd_vel
  publishes /jackal_velocity_controller/cmd_vel
```

如果不想大改 launch，也可以在 `drl_agent_node.py` 里做：

```
nominal_action = PPO action
safe_action = throttle(nominal_action, scan, eri)
publish_action(safe_action)
```

但我更喜欢外置 node，因为它更干净，也更像 Nav2 Collision Monitor 的工程结构。

* * *

## 优点

1. **最容易做出来**  
    不需要 QP，不需要优化器，不需要预测人。
2. **最不破坏 PPO**  
    PPO 还负责“往哪转”，安全层只说“慢点”。
3. **很容易降低 AC**  
    如果你现在 AC 高是因为速度太快，那么 throttle 很可能立刻有效。
4. **很好 debug**  
    你能直接看：前方距离多少，限速多少，是否停车。

* * *

## 缺点

1. **可能变笨**  
    它只会刹车，不会聪明绕路。
2. **可能降低效率**  
    AC 下降了，但 AT 可能上升。
3. **可能被困住**  
    人一直在前面，它可能一直等。
4. **理论上不够华丽**  
    它不是 CBF，不是 MPC，不是大魔法。但工程上非常香。

* * *

## 适配你项目的最小版本

```Python
front_min = min(scan in front_sector)

if front_min < d_stop:
    v_safe = 0.0
elif front_min < d_slow:
    ratio = (front_min - d_stop) / (d_slow - d_stop)
    v_safe = ratio * v_nom
else:
    v_safe = v_nom

w_safe = w_nom
```

ERI 可以这样接：

```
ERI 高 → d_stop / d_slow 变大，提前刹车
ERI 低 → d_stop / d_slow 变小，少干预
```

这就是最朴素的：

```
ERI-conditioned Safety Throttle
```

我认为这是你最该优先做的方案。

* * *

# 方案 2：Velocity Obstacle / DWA / Velocity-space Shield

## 它到底在做什么？

超级幼稚版：

> PPO 说：“我想用这个速度开。”  
> 小裁判说：“等一下，我把附近一圈速度都试一下。”  
> 撞墙的速度扔掉。  
> 不撞的速度留下。  
> 在不撞的速度里，选一个最像 PPO 原本想法的。

它不是直接说“慢点”，而是问：

```
如果我现在用这个 v,w 开 1 秒，会不会撞？
```

所以它比 Safety Throttle 聪明一点。

* * *

## 它和 DWA 是什么关系？

DWA，全名 Dynamic Window Approach，很经典。它的核心思想是：

```
不要直接在位置空间里找路
而是在速度空间里找安全速度
```

DWA 会考虑机器人动力学限制，然后在可达的速度窗口里搜索平移速度和旋转速度。CMU 的介绍里说，DWA 直接从机器人运动动力学推导，并且在实验中安全控制了 RHINO 机器人，以最高 95 cm/s 在 populated and dynamic environments 中移动。[Robotics Institute Publications](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance?utm_source=chatgpt.com)

这说明 DWA 本来就是给移动机器人局部避障用的，而且很老牌，很常见。

* * *

## 它和 Velocity Obstacle 是什么关系？

Velocity Obstacle，简称 VO，也是在速度空间里想问题。

幼稚版：

> 未来会撞的速度，是“坏速度”。  
> 机器人只要别选坏速度，就能避开动态障碍。

Fiorini 和 Shiller 的 VO 论文就是为动态环境中的机器人运动规划提出的，方法根据机器人和障碍物的当前位置与速度，在速度空间中选择避让动作，避开静态和移动障碍物。[Sage Journals+1](https://journals.sagepub.com/doi/10.1177/027836499801700706?utm_source=chatgpt.com)

RVO，Reciprocal Velocity Obstacle，是 VO 的多智能体版本。它假设大家都愿意互相让一点路。van den Berg 等人的 RVO 论文处理的是动态环境中包含静态和移动障碍物的 real-time multi-agent navigation，并且每个 agent 独立导航、不需要显式通信。[gamma.cs.unc.edu+1](https://gamma.cs.unc.edu/RVO/icra2008.pdf?utm_source=chatgpt.com)

* * *

## 有没有人经常这么做？

**非常经常。**

DWA 是经典 ROS navigation stack 的代表性 local planner 思想之一。VO/RVO/ORCA 是多机器人、人群仿真、动态避障里非常常见的一族方法。VO 类方法的综述还专门比较了 VO、RVO、HRVO、ORCA 在多智能体 collision avoidance 里的表现。[White Rose Research Online](https://eprints.whiterose.ac.uk/id/eprint/141072/1/Manuscript.pdf?utm_source=chatgpt.com)

* * *

## 相关文献都是怎么 integrate 到导航框架里的？

### DWA 的典型接法

```
Global planner gives path/goal
DWA local planner samples v,w
DWA simulates short arcs
DWA scores: obstacle distance + goal direction + speed
DWA publishes cmd_vel
```

也就是说，DWA 通常是直接当 local planner。

### VO/RVO 的典型接法

```
Robot has goal direction
Estimate obstacle positions/velocities
Build forbidden velocity set
Choose safe velocity outside forbidden set
Publish cmd_vel
```

它更像一个动态避障模块，尤其适合多 agent 场景。

### 你这里不应该直接替换 PPO

因为你要求不重新训练，也不想换 Motion Agent。所以我们不把 DWA/VO 当完整 local planner，而是把它改成：

```
PPO action shield
```

也就是：

```
PPO gives nominal [v_nom, w_nom]
sample nearby [v, w]
rollout each candidate for 0.5s / 1.0s / 1.5s
reject collision candidates
choose closest safe candidate to [v_nom, w_nom]
```

这个比当前 CBF 更适合 Jackal，因为它直接用差速车模型往前滚一下，不是假装机器人是单积分点。

* * *

## 它怎么 integrate 到你的系统？

最推荐结构：

```
drl_local_planner/PPO
    ↓ nominal_action [v_nom, w_nom]

velocity_space_shield
    input: scan, odom, nominal_action, optional ERI
    sample candidate velocities
    rollout short horizon
    choose closest safe action

safe_action
    ↓
/cmd_vel
```

伪代码：

```Python
candidates = sample_around(v_nom, w_nom)

safe = []
for v, w in candidates:
    traj = rollout_unicycle(v, w, horizon=1.0, dt=0.1)
    if footprint_clear(traj, scan_points, d_safe):
        safe.append((v, w))

if safe:
    v_safe, w_safe = closest_to_nominal(safe, v_nom, w_nom)
else:
    v_safe, w_safe = 0.0, 0.0
```

ERI 可以调：

```
ERI 高 → horizon 更长 / d_safe 更大 / candidates 更保守
ERI 低 → horizon 更短 / 更接近 PPO
```

例如：

```
horizon = 0.6 + 0.6 * ERI
d_safe = 0.35 + 0.15 * ERI
```

* * *

## 优点

1. **比 throttle 聪明**  
    它不只会停车，还会找一个“差不多但更安全”的速度。
2. **比 CBF 更贴近 Jackal**  
    直接 rollout 差速模型，不用单积分近似。
3. **不需要重训 PPO**  
    只是在 PPO 后面筛动作。
4. **和论文很好讲**  
    你可以说它是 “nominal-action-preserving velocity-space safety shield”。

* * *

## 缺点

1. **计算比 throttle 多**  
    但如果候选速度数量控制在几十个，10Hz 应该能扛住。
2. **需要 footprint collision checking**  
    不能只看一个点，Jackal 有身体宽度。
3. **对动态障碍速度估计有限**  
    如果只用单帧 scan，它更多是短时几何预测；如果要真正 VO，需要估计障碍速度。
4. **可能和 PPO 目标轻微冲突**  
    但比 CBF/circulation 少很多，因为它选择“最接近 PPO 的安全动作”。

* * *

## 我对它的评价

这是我最看好的主替代方案。

它不像 throttle 那么笨，也不像 MPC 那么重。它像一个小筛子，先把“会撞的速度”筛掉，再把最像 PPO 的那个还给机器人。很适合你现在的需求：**不重训，不替换 Motion Agent，只增强执行安全。**

* * *

# 方案 3：Predictive Safety Filter / MPC Shield

## 它到底在做什么？

超级幼稚版：

> PPO 说：“我要这么开！”  
> 安全滤波器说：“我先在脑子里演一下未来 1 秒。”  
> 如果不会撞：放行。  
> 如果会撞：改一点点。  
> 如果怎么改都不安全：停车。

它比方案 2 更高级，因为它不是简单采样，而是解一个小优化问题：

```
尽量接近 PPO 的动作
同时未来几步都不能撞
```

* * *

## 有没有人这么做？

**有，而且这是 learning-based control safety 里很正统的一条线。**

Wabersich 和 Zeilinger 的 Predictive Safety Filter 思想就是：学习控制器可能不懂显式约束，所以在它后面加一个预测安全滤波器。这个滤波器接收 proposed control input，然后判断能不能安全施加；如果不能，就修改控制输入。论文明确说这个 filter 能让任何 RL 算法以 “out-of-the-box” 的方式接入受约束系统。[arXiv+1](https://arxiv.org/abs/1812.05506?utm_source=chatgpt.com)

后来也有人把 Predictive Safety Filter 和 RL 结合到具体导航控制场景里。比如 marine navigation 里的 RL + PSF，安全滤波器接收主控制器可能不安全的 action，通过优化得到一个最小扰动动作，满足物理和安全约束。[arXiv+1](https://arxiv.org/html/2312.01855v2?utm_source=chatgpt.com)

还有 Conformal Predictive Safety Filter，它更偏动态人群：先预测其他 agents 的轨迹，再用 conformal prediction 给预测加不确定性区间，最后学习一个安全滤波器，使它尽量跟随 RL controller，但避开不确定区域。这个工作明确针对 robot navigation around pedestrians 这类安全关键场景。[arXiv+1](https://arxiv.org/abs/2306.02551?utm_source=chatgpt.com)

* * *

## 它一般在什么场景？

常见场景：

```
学习控制器后处理
自动驾驶/赛车
无人船/海事导航
移动机器人动态避障
行人环境中的 RL controller safety
```

比如 autonomous racing 里的 predictive safety filter 会和任意潜在不安全的控制信号配对，判断 desired control input 是否安全，必要时提供 alternate input，让车保持在赛道边界内。[arXiv](https://arxiv.org/abs/2102.11907?utm_source=chatgpt.com)

动态环境中的 conformal MPC/PSF 方向则会把未来障碍轨迹预测和不确定性区域放进 MPC 里。Lindemann 等人的 conformal prediction + MPC 框架就是用动态环境轨迹预测和预测区域来做安全规划。[arXiv+1](https://arxiv.org/abs/2210.10254?utm_source=chatgpt.com)

* * *

## 它怎么 integrate 到导航框架？

别人一般这么接：

```
RL / nominal controller
    ↓ proposed action
Predictive Safety Filter
    ↓ safe or minimally modified action
robot dynamics
```

它的核心是“模块化”：主控制器不用知道安全约束的全部细节，安全滤波器在后面负责检查和修正。相关文献也强调这种 modular separation，允许使用 arbitrary control policies。[arXiv+1](https://arxiv.org/html/2312.01855v2?utm_source=chatgpt.com)

你这里可以接成：

```
PPO nominal action
    ↓
MPC shield
    state: robot pose/velocity
    input: v,w
    model: differential drive / unicycle
    horizon: 5-10 steps
    constraints: footprint does not overlap scan obstacles
    objective: stay close to PPO action
    ↓
safe cmd_vel
```

目标函数幼稚版：

```
不要离 PPO 太远
不要撞
不要转得太疯
尽量还朝 subgoal 前进
```

数学味一点：

```
minimize:
  distance_to_nominal_action
  + collision_risk
  + action_smoothness_penalty
  - progress_to_subgoal

subject to:
  v_min <= v <= v_max
  w_min <= w <= w_max
  footprint_clearance >= d_safe
```

* * *

## 优点

1. **理论最强**  
    它最像真正的 safety filter。
2. **最小干预很清楚**  
    “PPO 能用就用，不能用才改”。
3. **可以处理未来 horizon**  
    比当前单步 CBF 更合理。
4. **和相关文献贴得很近**  
    尤其是 learning controller + safety filter 这个框架。

* * *

## 缺点

1. **工程最重**  
    要优化器，要模型，要 footprint collision constraint。
2. **实时性风险最大**  
    你的控制频率大概 10Hz 左右，优化器如果慢，就会影响闭环。
3. **调参复杂**  
    horizon、cost 权重、约束松弛、fallback 都要调。
4. **不一定比 velocity sampling 更快见效**  
    它更高级，但不是最快落地。

* * *

## 我对它的评价

它很适合当“第二阶段升级版”，但不适合作为今晚实验之后立刻开干的第一方案。

它像一位严肃律师，每一步都要审查合同。靠谱，但慢。你现在更需要先给机器人装刹车片和小筛子，而不是先请律师团 📜。

* * *

# 三个方案放在一起看

| 维度 | Safety Throttle | Velocity-space Shield | MPC / PSF |
| --- | --- | --- | --- |
| 是否重训 | 不需要 | 不需要 | 不需要 |
| 是否替换 PPO | 不替换 | 不替换 | 不替换 |
| 实现难度 | 低 | 中 | 高 |
| 实时风险 | 低 | 中 | 高 |
| 能否绕障 | 弱 | 中等 | 强 |
| 能否解释 | 强 | 强 | 很强 |
| 文献支撑 | 工程强 | 经典算法强 | 理论强 |
| 最适合当前项目 | 很适合 | 最适合作主替代 | 适合后续升级 |

* * *

# 推荐你接下来调研的文献清单

## A. Safety Throttle / Collision Monitor

### 1. Nav2 Collision Monitor 官方文档

它不是论文，但非常值得看。重点看它的四种动作模型：

```
stop
slowdown
limit
approach
```

官方文档说 Collision Monitor 是额外安全层，直接用 sensor data，绕过 costmap 和 trajectory planner，防止潜在碰撞。[docs.nav2.org](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html?utm_source=chatgpt.com)

你要学的是它的工程结构，而不是完全搬 ROS2 包。

### 2. Nav2 Collision Monitor design

重点看它的 integration：

```
Controller cmd_vel → Collision Monitor → filtered cmd_vel
```

官方 ROS docs 明确说它作为 controller 输出 `cmd_vel` 的 filter。[ROS Documentation](https://docs.ros.org/en/humble/p/nav2_collision_monitor/?utm_source=chatgpt.com)

这几乎就是你系统的模板。

* * *

## B. Velocity-space / DWA / VO

### 1. Fox, Burgard, Thrun, “The Dynamic Window Approach to Collision Avoidance”, 1997

场景：移动机器人局部避障，动态/有人环境。  
做法：在速度空间中搜索可执行速度，考虑机器人动力学。  
集成：作为 local planner，直接输出速度命令。  
文献要点：DWA 直接从机器人运动动力学推导，在 RHINO 机器人 populated and dynamic environments 中实验验证。[Robotics Institute Publications+1](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance?utm_source=chatgpt.com)

### 2. Fiorini and Shiller, “Motion Planning in Dynamic Environments Using Velocity Obstacles”, 1998

场景：动态环境，静态和移动障碍物。  
做法：在速度空间标出会导致碰撞的速度，避开这些速度。  
集成：作为动态避障模块选择 avoidance maneuver。  
文献要点：根据机器人和障碍物当前位置与速度，在速度空间中选择避让动作。[Sage Journals](https://journals.sagepub.com/doi/10.1177/027836499801700706?utm_source=chatgpt.com)

### 3. van den Berg et al., “Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation”, 2008

场景：多智能体、人群、无通信实时避障。  
做法：每个 agent 都承担一半避让责任，避免大家互相躲到一起振荡。  
集成：每个 agent 独立运行本地避障。  
文献要点：处理包含静态和移动障碍物的 real-time multi-agent navigation，每个 agent 独立导航且无需显式通信。[gamma.cs.unc.edu+1](https://gamma.cs.unc.edu/RVO/icra2008.pdf?utm_source=chatgpt.com)

* * *

## C. Predictive Safety Filter / MPC Shield

### 1. Wabersich and Zeilinger, “A Predictive Safety Filter for Learning-Based Control of Constrained Nonlinear Dynamical Systems”

场景：学习控制器 + 有约束非线性系统。  
做法：学习控制器给 action，safety filter 判断是否安全，不安全就修改。  
集成：放在任意 RL controller 后面。  
文献要点：filter 接收 proposed control input，判断能否安全施加，必要时修改；可让任意 RL 算法 out-of-the-box 接入受约束系统。[arXiv+1](https://arxiv.org/abs/1812.05506?utm_source=chatgpt.com)

### 2. Strawn, Ayanian, Lindemann, “Conformal Predictive Safety Filter for RL Controllers in Dynamic Environments”

场景：动态 agents，尤其是 RL navigation around pedestrians。  
做法：预测其他 agents 轨迹，用 conformal prediction 构造不确定性区间，再让 filter 避开这些区域。  
集成：RL controller 后处理 safety filter。  
文献要点：框架模块化，在 collision avoidance gym 中减少碰撞，且不做过度保守预测。[arXiv+1](https://arxiv.org/abs/2306.02551?utm_source=chatgpt.com)

### 3. Lindemann et al., “Safe Planning in Dynamic Environments using Conformal Prediction”

场景：未知动态环境，动态障碍预测不确定。  
做法：MPC 使用轨迹预测和不确定性 prediction regions。  
集成：prediction module + conformal uncertainty + MPC planner。  
文献要点：设计 MPC，使用动态环境轨迹预测和预测区域来提供概率安全保证。[arXiv+1](https://arxiv.org/abs/2210.10254?utm_source=chatgpt.com)

* * *

# 那么，具体到你的系统，怎么选？

我建议你把路线拆成两步。

## 第一步：先做 Safety Throttle

目标不是聪明，而是确认：

```
只限制速度，AC 能不能下降？
```

如果能下降，说明你现在最大问题就是高速执行风险。这个结果很有价值。

最小集成：

```
PPO action [v,w]
scan front_min
ERI adjusts thresholds
output [v_safe,w]
```

推荐实验组：

```
PPO baseline
fixed throttle
ERI-throttle
aggressive throttle
conservative throttle
```

指标看：

```
AC 是否下降
SR 是否保持
AT 是否恶化太多
stop_ratio 是否过高
```

* * *

## 第二步：做 Velocity-space Shield

如果 throttle 有用，但太笨，就升级到 velocity-space shield。

目标：

```
不是只会停，而是能从候选速度里选一个更安全但接近 PPO 的动作
```

最小集成：

```
sample v: [0, 0.4, 0.8, 1.2, 1.6, 2.0]
sample w around w_nom: [w_nom-1, w_nom-0.5, w_nom, w_nom+0.5, w_nom+1]
rollout 1 second
choose closest safe candidate
```

这就足够做第一版了。

* * *

# 最后给你一个非常清楚的判断

## 不推荐现在主攻 MPC / PSF

不是因为它不好，而是因为它现在太重。你当前最需要的是快速验证执行层安全增强能不能让 AC 下降。MPC/PSF 适合在你已经有 throttle 或 velocity-space shield 的正结果之后再升级。

## 推荐主线

```
主线 1：ERI-conditioned Safety Throttle
主线 2：Nominal-action-preserving Velocity-space Shield
备用高级线：Lightweight MPC / Predictive Safety Filter
```

更像小朋友语言就是：

```
先装刹车片。
刹车片有效但太笨，再装速度筛子。
速度筛子还不够，再请 MPC 律师。
```

我最推荐你下一步真的实现的是：

```
ERI-conditioned velocity-space shield
```

它既保留 ERI，又不重训 Motion Agent，还比当前 CBF/circulation 更贴合 Jackal 的真实运动。