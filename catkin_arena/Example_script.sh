#!/bin/bash
# 文件: run_6steps_venv.sh
# 说明: 确保在 rosnav 虚拟环境中顺序执行 roslaunch
#       如果是 mpc 规划器则运行 80 分钟；否则运行 60 分钟。

# 日志目录
LOGDIR="$HOME/run_schedule_logs"
mkdir -p "$LOGDIR"

# 虚拟环境路径（根据你的实际位置修改）
VENV_PATH="/home/robot/python_env/rosnav/bin/python"

# 时间戳函数
timestamp() {
  date +"%Y%m%d_%H%M%S"
}

# 杀掉上一个 roslaunch 相关进程
kill_roslaunch() {
  echo "停止所有 roslaunch 相关进程..."
  pkill -f "roslaunch arena_bringup start_arena_gazebo.launch"
  pkill -f "gazebo"
  pkill -f "roscore"
  pkill -f "rosmaster"
  sleep 8
}

# 检查并激活虚拟环境
activate_venv() {
  if [ -f "$VENV_PATH/bin/activate" ]; then
    echo "激活 rosnav 虚拟环境..."
    source "$VENV_PATH/bin/activate"
    echo "当前 Python: $(which python)"
    echo "当前 Python 版本: $(python --version)"
    return 0
  else
    echo "错误: 找不到 rosnav 虚拟环境"
    echo "虚拟环境路径: $VENV_PATH"
    echo "请检查路径是否正确或重新创建虚拟环境:"
    echo "  python3 -m venv $VENV_PATH"
    return 1
  fi
}

# 配置 ROS 环境
setup_ros() {
  echo "配置 ROS 环境..."
  source /opt/ros/noetic/setup.bash
  if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source "$HOME/catkin_ws/devel/setup.bash"
  elif [ -f "$HOME/catkin_arena/devel/setup.bash" ]; then
    source "$HOME/catkin_arena/devel/setup.bash"
  fi
  echo "当前 roslaunch: $(which roslaunch)"
}

# 执行单个任务
run_step() {
  local planner="$1"
  local world="$2"
  local scenario="$3"
  local ts=$(timestamp)
  local logfile="${LOGDIR}/run_${world}_${planner}_${ts}.log"
  
  # 根据 planner 决定运行时长（秒）
  if [ "$planner" = "mpc" ]; then
    runtime=$((140*60))   #
    minutes=140
  else
    runtime=$((110*60))   #
    minutes=110
  fi

  echo "========================================"
  echo "执行任务: $planner in $world with $scenario"
  echo "预计运行时长: ${minutes} 分钟"
  echo "========================================"
  echo "=== 启动 roslaunch，日志: $logfile ==="
  
  # 在虚拟环境中执行 roslaunch
  roslaunch arena_bringup start_arena_gazebo.launch \
    local_planner:=$planner \
    world:=$world \
    task_mode:=scenario \
    scenario_file:=$scenario \
    record_data:=true > "$logfile" 2>&1 &
  
  ROS_PID=$!
  
  # 等待一段时间确保 roslaunch 启动
  sleep 15
  
  # 检查进程是否还在运行
  if ! ps -p $ROS_PID > /dev/null 2>&1; then
    echo "错误: roslaunch 进程启动失败，请检查日志文件: $logfile"
    return 1
  fi
  
  echo "roslaunch 启动成功 (PID=$ROS_PID)，开始运行 ${minutes} 分钟..."
  
  # 运行指定时长（由 runtime 决定）
  sleep "$runtime"
  
  # 停止上一步
  echo "=== ${minutes} 分钟结束，停止 roslaunch (PID=$ROS_PID) ==="
  kill_roslaunch
  
  # 等待清理完成
  sleep 10
  
  return 0
}

# 循环步骤列表
STEPS=(
  "rosnav aws_house aws_house_obs10.json"
  "rosnav hospital hospital_obs10.json"
  "mpc aws_house aws_house_obs10.json"
  "mpc hospital hospital_obs10.json"
  "rosnav aws_house aws_house_obs10.json"
  "rosnav hospital hospital_obs10.json"
)

# 主程序
echo "开始执行 schedule（共 ${#STEPS[@]} 步）。每步时长：mpc -> 80 分钟，其它 -> 60 分钟。日志目录：$LOGDIR"

# 激活虚拟环境
if ! activate_venv; then
  exit 1
fi

# 配置 ROS 环境
setup_ros

# 检查 roslaunch 是否可用
if ! command -v roslaunch &> /dev/null; then
  echo "错误: roslaunch 命令未找到，请检查 ROS 环境配置"
  exit 1
fi

# 执行所有任务
for step in "${STEPS[@]}"; do
  planner=$(echo $step | awk '{print $1}')
  world=$(echo $step | awk '{print $2}')
  scenario=$(echo $step | awk '{print $3}')
  
  if ! run_step "$planner" "$world" "$scenario"; then
    echo "任务执行失败，退出脚本"
    exit 1
  fi
  
  echo "任务完成，等待 10 秒后开始下一个任务..."
  sleep 10
done

echo "=== 所有步骤完成 ==="

