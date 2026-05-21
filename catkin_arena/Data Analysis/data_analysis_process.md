测试算法、评估获取结果数据流程
1、确定参数配置文件
在/home/robot/catkin_arena/src/arena-rosnav-3D/simulator_setup/scenarios目录下
有对应json文件，在launch文件中指定并启动
涉及机器人初始位置、行人路径、行人数量、行人速度、运动批次、世界选择等设置。
注意：修改世界的时候要同时修改机器人起始点和终点
2、启动环境
启动单个环境（适用于启动subgoal和rrt_supply）
workon rosnav
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=rosnav world:=small_warehouse task_mode:=scenario scenario_file:=small_warehouse_obs10_v0.2.json record_data:=true
#第二个终端
workon rosnav
roslaunch zjr_planner action_server.launch
多个环境线程启动（不适用于启动subgoal和rrt_supply，因为这两个需要单独开launch文件）
cd /home/robot/catkin_arena/src
./run_6steps.sh
注意要修改run_6steps.sh文件才能发挥作用：
[Image]
只修改这个位置就行，一次能跑多种条件下的仿真（一次最好不要超过6组，不然会出现电脑死机的情况）
3、记录数据
在主launch文件中通过添加语句来启动数据记录节点data_recorder_node
核心：data_recorder_node.py
在目录/home/robot/catkin_arena/src/forks/arena-evaluation/scripts下
记录的数据保存在data里，以时间戳命名，如"15-09-2025_20-50-14_"
在目录/home/robot/catkin_arena/src/forks/arena-evaluation/data下
4、处理数据
注意！数据需要全部放在目录/home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理下
4.1、给数据文件重命名
eg：data下的15-09-2025_20-50-14_重命名为dwa_aws_house_aws_house_obs10_v0.2
格式：算法_世界_json文件名
4.2、批量处理当前目录下所有原始数据，生成成功率、路径长度、平均用时等信息
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 batch_generate_metrics.py
4.3、批量裁剪metrics数据，使其变为100行，方便统计
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 metrics_change.py
4.4、批量修改param.yaml文件，如果使用了subgoal就需要使用这一步将rosnav重命名为subgoal，没有就不需要！
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 params_change.py
4.5、合并数据，方便对比和做柱状图以及风琴图
注意要以算法分类，统计同一场景下五种算法
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 make_multi_alg_summary.py
4.6、合并更多的数据，方便对比和做柱状图以及风琴图
注意要以环境和算法分类，统计不同场景下不同算法结果，要先运行4.5
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 make_multi_alg_summary.py  

5、如果 json 环境跑到约30个 episode 会卡死：分段跑满100个 episode
结论：数据分析脚本本身不是必须一次跑完100个 episode。
batch_generate_metrics.py 会按实际 episode 逐个生成 metrics.csv；100 的要求主要来自 metrics_change.py 保留前100行，目的是让不同算法/场景的统计口径一致。

推荐做法：每次只跑25个完整 episode，自动重启环境，固定跑4次，最后把4个 record_data:=true 生成的原始数据目录合并成一个连续的100 episode目录，再进入原有4.2之后的处理流程。

5.1、自动分段运行并合并
workon rosnav
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理

# 先把本目录下的 run_100_episodes_chunked.sh 和 merge_episode_chunks.py 放到 数据处理 目录，
# 或者在下面命令中使用它们在机器上的真实绝对路径。

# 脚本路径在本项目 Data Analysis 目录下；按实际算法/世界/json 修改这三个变量
PLANNER=rosnav \
WORLD=small_warehouse \
SCENARIO_FILE=small_warehouse_obs10_v0.2.json \
bash "/home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理/run_100_episodes_chunked.sh"

脚本逻辑：
- 每个 chunk 启动 roslaunch arena_bringup start_arena_gazebo.launch，并使用 record_data:=true；
- 每次启动前会放一个临时 marker，只监控本轮新生成的时间戳目录，避免误抓上一轮目录；
- 当 episode id 到达 CHUNK_EPISODES（默认25）后停止并重启；
- 固定重复 RUN_COUNT 次（默认4次），参考 run_6steps/Example_script 的“启动-等待-关闭-再启动”循环逻辑；
- 默认排除每个 chunk 里最高编号 episode，因为它通常是刚开始的未完成 episode；
- 4次结束后自动调用 merge_episode_chunks.py，把4个 chunk 合成 数据处理/算法_世界_json_MM-DD-YYYY_HH-MM-SS_merged100。

5.2、如果已经手动跑出了多个 chunk，可以只合并
python3 merge_episode_chunks.py \
  /home/robot/catkin_arena/src/forks/arena-evaluation/data/第1段时间戳目录 \
  /home/robot/catkin_arena/src/forks/arena-evaluation/data/第2段时间戳目录 \
  /home/robot/catkin_arena/src/forks/arena-evaluation/data/第3段时间戳目录 \
  /home/robot/catkin_arena/src/forks/arena-evaluation/data/第4段时间戳目录 \
  --output /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理/rosnav_small_warehouse_small_warehouse_obs10_v0.2_merged100 \
  --target-episodes 100 \
  --force

5.3、合并后继续原流程
cd /home/robot/catkin_arena/src/forks/arena-evaluation/data/数据处理
python3 batch_generate_metrics.py --root . --pattern merged100
python3 metrics_change.py --keep-rows 100 --require-rows

之后按需要运行：
python3 params_change.py
python3 make_multi_alg_summary.py --files 路径1/metrics.csv,路径2/metrics.csv --names 算法1,算法2
