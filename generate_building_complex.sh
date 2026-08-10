#!/usr/bin/env bash
# =============================================================================
# 脚本名称: generate_building_complex.sh
# 功能描述: 生成多层建筑竞赛场景（世界文件、危险物真值等），
#          并启动门/电梯控制服务（不包含机器人及 Gazebo 仿真）
# 启动：    INCLUDE_ROOF=false ./generate_building_complex.sh
# =============================================================================
set -euo pipefail  # 严格模式：任何命令失败/未定义变量/管道错误均立即退出

# =============================================================================
# 1. 确定工作空间目录（脚本所在目录）
# =============================================================================
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# =============================================================================
# 2. 可配置参数（均支持通过环境变量覆盖，提供默认值）
# =============================================================================

# ---- 随机种子（空表示不固定种子） ----
SEED="${SEED:-}"

# ---- 建筑结构参数 ----
FLOOR_COUNT="${FLOOR_COUNT:-2}"              # 楼层数量，默认 3 层
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"      # 每层房间数，默认 4 间
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"     # 建筑宽度（米），默认 20m
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"   # 建筑长度（米），默认 36m

# ---- 危险物/干扰物数量（格式: min:max） ----
DANGER_COUNT="${DANGER_COUNT:-3:6}"          # 危险物品数量范围
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"  # 干扰物数量范围

# ---- 建筑生成器类型（competition 或 lightweight） ----
BUILDING_GENERATOR="${BUILDING_GENERATOR:-competition}"

# ---- lightweight 生成器专用参数 ----
INCLUDE_ROOF="${INCLUDE_ROOF:-false}"            # 是否包含屋顶
INCLUDE_ELEVATOR="${INCLUDE_ELEVATOR:-true}"      # 是否包含电梯
INCLUDE_STAIRS="${INCLUDE_STAIRS:-true}"          # 是否包含楼梯
INCLUDE_FURNITURE="${INCLUDE_FURNITURE:-true}"   # 是否包含家具
FLOOR_HEIGHT="${FLOOR_HEIGHT:-2.6}"              # 每层高度（米）
WALL_HEIGHT="${WALL_HEIGHT:-2.35}"               # 墙体高度（米）
CORRIDOR_WIDTH="${CORRIDOR_WIDTH:-1.6}"          # 走廊宽度（米）
LOBBY_DEPTH="${LOBBY_DEPTH:-4.2}"               # 大厅深度（米）

# ---- 门/电梯控制服务 ----
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}" # 是否启动门/电梯控制服务（默认启动）
ROBOT_MODEL="${ROBOT_MODEL:-tb3_mid360}"              # 机器人 Gazebo 模型名（电梯同步移动用）

# =============================================================================
# 3. 加载 ROS 环境
# =============================================================================
echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash         # ROS Noetic 主环境
source "$WORKSPACE_DIR/devel/setup.bash"  # 当前工作空间的编译产物

# =============================================================================
# 4. 定位关键目录
# =============================================================================
BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"   # 建筑障碍物包路径
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"           # 场景输出目录
RESULTS_DIR="$WORKSPACE_DIR/results"                           # 结果输出目录
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

# =============================================================================
# 5. 生成竞赛场景（调用 Python 脚本）
# =============================================================================
echo "Generating competition scene..."

# 根据 BUILDING_GENERATOR 类型选择不同的生成脚本
GENERATOR_SCRIPT="generate_competition_scene.py"
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_SCRIPT="generate_lightweight_competition_scene.py"
elif [ "$BUILDING_GENERATOR" = "competition" ]; then
  if [ "$INCLUDE_ROOF" = "false" ] || [ "$INCLUDE_ROOF" = "0" ]; then
    GENERATOR_SCRIPT="generate_competition_scene_no_roof.py"
  fi
fi

# 构建通用参数列表
GENERATOR_ARGS=(
  --output-dir "$SCENE_OUTPUT_DIR"
  --results-dir "$RESULTS_DIR"
  --floor-count "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width "$BUILDING_WIDTH"
  --length "$BUILDING_LENGTH"
  --danger-count "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
)

# lightweight 生成器额外支持建筑细节参数
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_ARGS+=(
    --floor-height "$FLOOR_HEIGHT"
    --wall-height "$WALL_HEIGHT"
    --corridor-width "$CORRIDOR_WIDTH"
    --lobby-depth "$LOBBY_DEPTH"
  )
  # 处理布尔型参数（屋顶、电梯、楼梯、家具）
  if [ "$INCLUDE_ROOF" = "true" ] || [ "$INCLUDE_ROOF" = "1" ]; then
    GENERATOR_ARGS+=(--include-roof)
  else
    GENERATOR_ARGS+=(--no-roof)
  fi
  if [ "$INCLUDE_ELEVATOR" = "true" ] || [ "$INCLUDE_ELEVATOR" = "1" ]; then
    GENERATOR_ARGS+=(--include-elevator)
  else
    GENERATOR_ARGS+=(--no-elevator)
  fi
  if [ "$INCLUDE_STAIRS" = "true" ] || [ "$INCLUDE_STAIRS" = "1" ]; then
    GENERATOR_ARGS+=(--include-stairs)
  else
    GENERATOR_ARGS+=(--no-stairs)
  fi
  if [ "$INCLUDE_FURNITURE" = "true" ] || [ "$INCLUDE_FURNITURE" = "1" ]; then
    GENERATOR_ARGS+=(--include-furniture)
  else
    GENERATOR_ARGS+=(--no-furniture)
  fi
fi

# competition 去屋顶模式：传递 --no-roof 参数
if [ "$BUILDING_GENERATOR" = "competition" ] && [ "$GENERATOR_SCRIPT" = "generate_competition_scene_no_roof.py" ]; then
  GENERATOR_ARGS+=(--no-roof)
fi

# 如果指定了随机种子，添加到参数中
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi

# 执行场景生成脚本，标准输出重定向到 manifest 文件
python3 "$BUILDING_OBSTACLES_DIR/scripts/$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

# =============================================================================
# 6. 打印生成结果
# =============================================================================
echo "=========================================="
echo "Competition scene generation complete"
echo "  World:   $SCENE_OUTPUT_DIR/competition_scene.world"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="

# =============================================================================
# 7. 可选：启动建筑门/电梯控制服务
# =============================================================================
if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    --robot-model "$ROBOT_MODEL" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="