#!/usr/bin/env bash
# =============================================================================
# generate_building.sh
# 
# 功能：仅生成竞赛建筑场景，不启动 Gazebo、机器人、控制器等任何仿真组件。
# 
# 用法：
#   # 使用默认参数（competition 模式，3层，每层4房间）
#   ./generate_building.sh
#
#   # 使用 lightweight 轻量模式（低链接数，适合性能敏感场景）
#   BUILDING_GENERATOR=lightweight ./generate_building.sh
#
#   # 自定义楼层和房间数
#   FLOOR_COUNT=5 ROOMS_PER_FLOOR=6 ./generate_building.sh
#
#   # 固定随机种子以复现场景
#   SEED=42 ./generate_building.sh
#
#   # 组合使用
#   BUILDING_GENERATOR=lightweight SEED=123 FLOOR_COUNT=2 ROOMS_PER_FLOOR=3 ./generate_building.sh
#
# 可配置的环境变量（均有默认值，按需覆盖）：
#   BUILDING_GENERATOR  - 生成器类型: competition(默认) | lightweight
#   FLOOR_COUNT         - 楼层数: 整数或 min:max 范围 (默认: 3)
#   ROOMS_PER_FLOOR     - 每层房间数: 整数或 min:max 范围 (默认: 4)
#   BUILDING_WIDTH      - 建筑宽度/米 (默认: 20.0)
#   BUILDING_LENGTH     - 建筑长度/米 (默认: 36.0)
#   DANGER_COUNT        - 危险源数量: 整数或 min:max 范围 (默认: 3:6)
#   DISTRACTOR_COUNT    - 干扰源数量: 整数或 min:max 范围 (默认: 4:8)
#   SEED                - 随机种子 (默认: 随机)
#
#   --- 以下仅 lightweight 模式生效 ---
#   INCLUDE_ROOF        - 是否生成屋顶 (默认: false)
#   INCLUDE_ELEVATOR    - 是否生成电梯 (默认: true)
#   INCLUDE_STAIRS      - 是否生成楼梯 (默认: true)
#   INCLUDE_FURNITURE   - 是否生成家具 (默认: false)
#   FLOOR_HEIGHT        - 层高/米 (默认: 2.6)
#   WALL_HEIGHT         - 墙高/米 (默认: 2.35)
#   CORRIDOR_WIDTH      - 走廊宽度/米 (默认: 1.6)
#   LOBBY_DEPTH         - 大厅深度/米 (默认: 4.2)
#
#   --- 机器人起始位姿 ---
#   ROBOT_X / ROBOT_Y / ROBOT_Z / ROBOT_YAW  (默认: 0.0 / -2.2 / 0.6 / 1.5708)
#
# 输出文件（位于 generated_building/ 和 results/）：
#   generated_building/competition_scene.world  - 竞赛世界文件（Gazebo 直接加载）
#   generated_building/world.sdf                - 建筑 SDF 模型
#   generated_building/model.sdf                - 建筑静态模型
#   generated_building/scene_manifest.json      - 场景清单（记录所有生成参数）
#   generated_building/building_config.json     - 建筑配置
#   generated_building/door_config.yaml         - 门控配置文件
#   generated_building/elevator_config.yaml     - 电梯配置文件
#   generated_building/layout_metadata.json     - 布局元数据
#   generated_building/generation_checks.json   - 生成校验报告
#   generated_building/danger_truth.json        - 危险源真值
#   results/danger_truth.json                   - 危险源真值（供裁判使用）
# =============================================================================

set -euo pipefail

# ---- 1. 确定工作空间目录 ------------------------------------------------
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# ---- 2. 可配置参数（均支持通过环境变量覆盖）-------------------------------
SEED="${SEED:-}"                              # 随机种子，空=随机
FLOOR_COUNT="${FLOOR_COUNT:-1}"               # 楼层数
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-2}"       # 每层房间数
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"      # 建筑宽度
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"    # 建筑长度
DANGER_COUNT="${DANGER_COUNT:-3:6}"           # 危险源数量
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"   # 干扰源数量
BUILDING_GENERATOR="${BUILDING_GENERATOR:-competition}"  # 生成器: competition | lightweight

# lightweight 专属参数
INCLUDE_ROOF="${INCLUDE_ROOF:-false}"
INCLUDE_ELEVATOR="${INCLUDE_ELEVATOR:-false}"
INCLUDE_STAIRS="${INCLUDE_STAIRS:-true}"
INCLUDE_FURNITURE="${INCLUDE_FURNITURE:-false}"
FLOOR_HEIGHT="${FLOOR_HEIGHT:-2.6}"
WALL_HEIGHT="${WALL_HEIGHT:-2.35}"
CORRIDOR_WIDTH="${CORRIDOR_WIDTH:-1.6}"
LOBBY_DEPTH="${LOBBY_DEPTH:-4.2}"

# 机器人起始位姿
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--2.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

# ---- 3. Source ROS 环境 -------------------------------------------------
echo "==> Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

# ---- 4. 确定路径 --------------------------------------------------------
BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR"

# ---- 5. 选择生成器脚本 ---------------------------------------------------
# 根据 BUILDING_GENERATOR 环境变量切换：
#   competition → 完整版生成器（原始复杂模型）
#   lightweight → 轻量版生成器（低链接数 box 模型，性能更好）
GENERATOR_SCRIPT="generate_competition_scene.py"
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_SCRIPT="generate_lightweight_competition_scene.py"
fi

# ---- 6. 构建生成器参数 ---------------------------------------------------
# 基础参数（两种模式通用）
GENERATOR_ARGS=(
  --output-dir      "$SCENE_OUTPUT_DIR"
  --results-dir     "$RESULTS_DIR"
  --floor-count     "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width           "$BUILDING_WIDTH"
  --length          "$BUILDING_LENGTH"
  --danger-count    "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
  --robot-x         "$ROBOT_X"
  --robot-y         "$ROBOT_Y"
  --robot-z         "$ROBOT_Z"
  --robot-yaw       "$ROBOT_YAW"
)

# lightweight 模式额外参数（建筑细节控制）
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_ARGS+=(
    --floor-height   "$FLOOR_HEIGHT"
    --wall-height    "$WALL_HEIGHT"
    --corridor-width "$CORRIDOR_WIDTH"
    --lobby-depth    "$LOBBY_DEPTH"
  )

  # 布尔开关：处理 true/1 都视为启用
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

# 固定随机种子（如指定则传入）
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi

# ---- 7. 执行场景生成 ----------------------------------------------------
echo "=========================================="
echo "  Generator : $BUILDING_GENERATOR ($GENERATOR_SCRIPT)"
echo "  Floors    : $FLOOR_COUNT"
echo "  Rooms/Flr : $ROOMS_PER_FLOOR"
echo "  Size      : ${BUILDING_WIDTH}m x ${BUILDING_LENGTH}m"
echo "  Dangers   : $DANGER_COUNT"
echo "  Distract. : $DISTRACTOR_COUNT"
echo "  Seed      : ${SEED:-<random>}"
echo "  Robot     : ($ROBOT_X, $ROBOT_Y, $ROBOT_Z) yaw=$ROBOT_YAW"
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  echo "  Roof      : $INCLUDE_ROOF"
  echo "  Elevator  : $INCLUDE_ELEVATOR"
  echo "  Stairs    : $INCLUDE_STAIRS"
  echo "  Furniture : $INCLUDE_FURNITURE"
fi
echo "=========================================="

python3 "$BUILDING_OBSTACLES_DIR/scripts/$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

# ---- 8. 输出结果汇总 ----------------------------------------------------
echo ""
echo "=========================================="
echo " Building generation complete."
echo "=========================================="
echo "  World     : $SCENE_OUTPUT_DIR/competition_scene.world"
echo "  Truth     : $RESULTS_DIR/danger_truth.json"
echo "  Doors     : $SCENE_OUTPUT_DIR/door_config.yaml"
echo "  Elevator  : $SCENE_OUTPUT_DIR/elevator_config.yaml"
echo "  Manifest  : $SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Config    : $SCENE_OUTPUT_DIR/building_config.json"
echo "=========================================="
