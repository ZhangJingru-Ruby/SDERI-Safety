#!/usr/bin/env bash
set -euo pipefail

# Run arena-rosnav in episode chunks, then merge the raw recorder folders.
# This is meant for environments that hang around episode 30: run 25 episodes,
# restart Gazebo/ROS, repeat, and merge the chunks into a 100-episode folder.

PLANNER="${PLANNER:-rosnav}"
WORLD="${WORLD:-small_warehouse}"
SCENARIO_FILE="${SCENARIO_FILE:-small_warehouse_obs10_v0.2.json}"
TARGET_EPISODES="${TARGET_EPISODES:-100}"
CHUNK_EPISODES="${CHUNK_EPISODES:-25}"
MAX_CHUNKS="${MAX_CHUNKS:-8}"
STALE_SECONDS="${STALE_SECONDS:-900}"
POLL_SECONDS="${POLL_SECONDS:-10}"
LOGDIR="${LOGDIR:-$HOME/run_schedule_logs}"
VENV_DIR="${VENV_DIR:-/home/robot/python_env/rosnav}"
CATKIN_SETUP="${CATKIN_SETUP:-$HOME/catkin_arena/devel/setup.bash}"
DATA_ROOT="${DATA_ROOT:-}"
PROCESS_ROOT="${PROCESS_ROOT:-}"
OUTPUT_NAME="${OUTPUT_NAME:-${PLANNER}_${WORLD}_${SCENARIO_FILE%.*}_merged100}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<USAGE
Usage:
  PLANNER=rosnav WORLD=small_warehouse SCENARIO_FILE=small_warehouse_obs10_v0.2.json \\
  bash "$0"

Environment variables:
  TARGET_EPISODES   default: 100
  CHUNK_EPISODES    default: 25. Stop when episode id reaches this value; merge uses 0..CHUNK_EPISODES-1.
  MAX_CHUNKS        default: 8
  STALE_SECONDS     default: 900. Restart if episode.csv stops changing.
  DATA_ROOT         default: rospack find arena-evaluation/data
  PROCESS_ROOT      default: DATA_ROOT/数据处理
  OUTPUT_NAME       default: planner_world_scenario_merged100
USAGE
}

timestamp() {
  date +"%Y%m%d_%H%M%S"
}

setup_env() {
  if [ -f "$VENV_DIR/bin/activate" ]; then
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"
  else
    echo "[WARN] virtualenv not found: $VENV_DIR"
  fi

  if [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  fi

  if [ -f "$CATKIN_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$CATKIN_SETUP"
  fi

  if [ -z "$DATA_ROOT" ]; then
    DATA_ROOT="$(rospack find arena-evaluation)/data"
  fi
  if [ -z "$PROCESS_ROOT" ]; then
    PROCESS_ROOT="$DATA_ROOT/数据处理"
  fi
  mkdir -p "$DATA_ROOT" "$PROCESS_ROOT"
}

kill_arena() {
  echo "[STOP] stopping arena/gazebo/ros processes" >&2
  pkill -f "roslaunch arena_bringup start_arena_gazebo.launch" || true
  pkill -f "gazebo" || true
  pkill -f "gzserver" || true
  pkill -f "gzclient" || true
  pkill -f "rosmaster" || true
  pkill -f "roscore" || true
  sleep 8
}

latest_data_dir() {
  find "$DATA_ROOT" -mindepth 1 -maxdepth 1 -type d ! -name "数据处理" -print0 \
    | xargs -0 ls -td 2>/dev/null \
    | head -n 1
}

max_episode_seen() {
  local episode_csv="$1"
  if [ ! -s "$episode_csv" ]; then
    echo "-1"
    return
  fi
  awk -F',' 'NR > 1 && $2 ~ /^[0-9]+$/ { if ($2 > max) max=$2 } END { if (max == "") print -1; else print max }' "$episode_csv"
}

wait_for_chunk() {
  local chunk_index="$1"
  local pid="$2"
  local data_dir=""
  local last_size=0
  local last_change
  local max_ep=-1

  last_change="$(date +%s)"
  while kill -0 "$pid" >/dev/null 2>&1; do
    if [ -z "$data_dir" ]; then
      data_dir="$(latest_data_dir || true)"
      if [ -n "$data_dir" ]; then
        echo "[CHUNK $chunk_index] recorder folder: $data_dir" >&2
      fi
    fi

    if [ -n "$data_dir" ] && [ -f "$data_dir/episode.csv" ]; then
      local size
      size="$(wc -c < "$data_dir/episode.csv")"
      if [ "$size" != "$last_size" ]; then
        last_size="$size"
        last_change="$(date +%s)"
      fi

      max_ep="$(max_episode_seen "$data_dir/episode.csv")"
      echo "[CHUNK $chunk_index] max episode seen: $max_ep / stop at $CHUNK_EPISODES" >&2
      if [ "$max_ep" -ge "$CHUNK_EPISODES" ]; then
        echo "$data_dir"
        return 0
      fi
    fi

    local now
    now="$(date +%s)"
    if [ $((now - last_change)) -ge "$STALE_SECONDS" ]; then
      echo "[WARN] episode.csv stale for ${STALE_SECONDS}s; ending chunk $chunk_index" >&2
      echo "$data_dir"
      return 0
    fi

    sleep "$POLL_SECONDS"
  done

  echo "[WARN] roslaunch exited before chunk target" >&2
  echo "$data_dir"
}

run_chunk() {
  local chunk_index="$1"
  local logfile="$LOGDIR/run_${PLANNER}_${WORLD}_${SCENARIO_FILE}_chunk${chunk_index}_$(timestamp).log"

  echo "[START] chunk $chunk_index: $PLANNER $WORLD $SCENARIO_FILE" >&2
  roslaunch arena_bringup start_arena_gazebo.launch \
    local_planner:="$PLANNER" \
    world:="$WORLD" \
    task_mode:=scenario \
    scenario_file:="$SCENARIO_FILE" \
    record_data:=true > "$logfile" 2>&1 &

  local pid="$!"
  sleep 20
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    echo "[ERROR] roslaunch failed. Log: $logfile" >&2
    return 1
  fi

  local data_dir
  data_dir="$(wait_for_chunk "$chunk_index" "$pid" | tail -n 1)"
  kill_arena

  if [ -z "$data_dir" ] || [ ! -d "$data_dir" ]; then
    echo "[ERROR] no recorder folder found for chunk $chunk_index" >&2
    return 1
  fi

  echo "$data_dir"
}

main() {
  if [ "${1:-}" = "--help" ]; then
    usage
    exit 0
  fi

  setup_env
  mkdir -p "$LOGDIR"
  command -v roslaunch >/dev/null
  command -v python3 >/dev/null

  echo "[CONFIG] planner=$PLANNER world=$WORLD scenario=$SCENARIO_FILE"
  echo "[CONFIG] target=$TARGET_EPISODES chunk=$CHUNK_EPISODES data=$DATA_ROOT process=$PROCESS_ROOT"

  declare -a chunks=()
  for idx in $(seq 1 "$MAX_CHUNKS"); do
    chunk_dir="$(run_chunk "$idx")"
    chunks+=("$chunk_dir")

    merged_preview="$PROCESS_ROOT/${OUTPUT_NAME}_preview"
    python3 "$SCRIPT_DIR/merge_episode_chunks.py" "${chunks[@]}" \
      --output "$merged_preview" \
      --target-episodes "$TARGET_EPISODES" \
      --force >/tmp/merge_episode_chunks_preview.log

    merged_count="$(awk -F',' 'NR > 1 && $2 ~ /^[0-9]+$/ { seen[$2]=1 } END { print length(seen) }' "$merged_preview/episode.csv")"
    echo "[PROGRESS] merged usable episodes: $merged_count / $TARGET_EPISODES"
    rm -rf "$merged_preview"

    if [ "$merged_count" -ge "$TARGET_EPISODES" ]; then
      break
    fi
  done

  output_dir="$PROCESS_ROOT/$OUTPUT_NAME"
  python3 "$SCRIPT_DIR/merge_episode_chunks.py" "${chunks[@]}" \
    --output "$output_dir" \
    --target-episodes "$TARGET_EPISODES" \
    --force

  echo "[OK] merged raw data: $output_dir"
  echo "[NEXT] cd \"$PROCESS_ROOT\""
  echo "[NEXT] python3 batch_generate_metrics.py --root . --pattern \"$OUTPUT_NAME\""
  echo "[NEXT] python3 metrics_change.py"
}

main "$@"
