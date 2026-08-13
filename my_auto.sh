#!/usr/bin/env bash
# =============================================================================
# my_auto.sh —— 建筑生成 + 仿真一键启动脚本
#
# 功能:
#   1. 生成或加载多楼层建筑 + 危险源/干扰物世界文件 (generated_building/)
#      - 默认重新生成; 或 LOAD_EXISTING=1 加载已有建筑(不重新生成)
#      - 或 ASK_BUILDING=1 运行时交互选择 重新生成 / 加载已有
#   2. 启动 Gazebo + 机器人 + 传感器 + 导航接口
#   3. 启动建筑门/电梯控制服务与机器人控制器
#
# 常用环境变量(均可覆盖默认值):
#   建筑: FLOOR_COUNT / ROOMS_PER_FLOOR / BUILDING_WIDTH / BUILDING_LENGTH
#          DANGER_COUNT / DISTRACTOR_COUNT / SEED
#   仿真: GUI / PAUSED / AUTO_UNPAUSE / START_CONTROLLER / START_BUILDING_CONTROL
#   物理: GAZEBO_PHYSICS_MAX_STEP_SIZE / GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE
#          GAZEBO_PHYSICS_ODE_ITERS / GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL
#   出生: ROBOT_X / ROBOT_Y / ROBOT_Z / ROBOT_YAW
#
# 示例:
#   ./my_auto.sh                                # 默认 2层 每层4间
#   FLOOR_COUNT=3 SEED=42 ./my_auto.sh          # 指定楼层与随机种子
#   START_CONTROLLER=0 ./my_auto.sh             # 不启动控制器, 仅开仿真

##  方式1：强制加载已有建筑（跳过重新生成）
#   LOAD_EXISTING=1 ./my_auto.sh

#   方式2：运行时交互选择（在终端运行，检测到已有建筑会询问）
#   ASK_BUILDING=1 ./my_auto.sh
#   提示: 检测到已有建筑(...)。重新生成 [r] / 加载已有 [l]? (默认 r):
#   输入 l 即加载已有, 直接回车则重新生成

#   方式3：默认行为（重新生成，与之前完全一致）
#   ./my_auto.sh
# =============================================================================
set -euo pipefail

# 切换到脚本所在目录, 保证相对路径始终正确
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# 将 1/true/yes/on 等值统一转换为 "true"/"false"，用于解析布尔环境变量
as_ros_bool() {
  case "$1" in
    1|true|TRUE|True|yes|YES|on|ON) printf "true" ;;
    0|false|FALSE|False|no|NO|off|OFF) printf "false" ;;
    *) printf "%s" "$1" ;;
  esac
}

# ───────── 建筑布局参数 ─────────
SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-2}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"

# ───────── 仿真启动控制 ─────────
GUI="${GUI:-true}"
PAUSED="${PAUSED:-false}"
AUTO_UNPAUSE="$(as_ros_bool "${AUTO_UNPAUSE:-1}")"
AUTO_UNPAUSE_DELAY="${AUTO_UNPAUSE_DELAY:-6}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"

# ───────── 传感器与数据开关 ─────────
ENABLE_SENSOR_DATA_DEFAULT="${ENABLE_SENSORS:-1}"
ENABLE_SENSOR_DATA="$(as_ros_bool "${ENABLE_SENSOR_DATA:-$ENABLE_SENSOR_DATA_DEFAULT}")"
ENABLE_LIVOX="$(as_ros_bool "${ENABLE_LIVOX:-$ENABLE_SENSOR_DATA}")"
ENABLE_LIVOX_IMU="$(as_ros_bool "${ENABLE_LIVOX_IMU:-$ENABLE_LIVOX}")"
ENABLE_REALSENSE_INPUT="${ENABLE_REALSENSE:-${ENABLE_DEPTH_CAMERA:-$ENABLE_SENSOR_DATA}}"
ENABLE_REALSENSE="$(as_ros_bool "$ENABLE_REALSENSE_INPUT")"
ENABLE_FRONT_CAMERA="$(as_ros_bool "${ENABLE_FRONT_CAMERA:-0}")"
ENABLE_REFEREE_ODOM="$(as_ros_bool "${ENABLE_REFEREE_ODOM:-1}")"
ENABLE_GROUND_TRUTH="$(as_ros_bool "${ENABLE_GROUND_TRUTH:-1}")"
ENABLE_FOOT_CONTACT_SENSOR="$(as_ros_bool "${ENABLE_FOOT_CONTACT_SENSOR:-0}")"
ENABLE_FOOT_FORCE_VISUAL="$(as_ros_bool "${ENABLE_FOOT_FORCE_VISUAL:-0}")"
ENABLE_JOY_NODE="$(as_ros_bool "${ENABLE_JOY_NODE:-0}")"
ENABLE_POINTCLOUD_CONVERTER="$(as_ros_bool "${ENABLE_POINTCLOUD_CONVERTER:-$ENABLE_LIVOX}")"
POINTCLOUD_USE_GROUND_TRUTH_ODOM="$(as_ros_bool "${POINTCLOUD_USE_GROUND_TRUTH_ODOM:-1}")"
WRITE_GENERATED_TRUTH_COPY="$(as_ros_bool "${WRITE_GENERATED_TRUTH_COPY:-1}")"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
UNITREE_LOG_WAIT_WARNINGS="$(as_ros_bool "${UNITREE_LOG_WAIT_WARNINGS:-0}")"
ROBOT_SPAWN_TIMEOUT="${ROBOT_SPAWN_TIMEOUT:-120}"
CONTROLLER_SPAWNER_TIMEOUT="${CONTROLLER_SPAWNER_TIMEOUT:-120}"

# ───────── Gazebo 物理参数（影响机器人运动稳定性） ─────────
# 注意: contact_max_correcting_vel 过大或 update_rate 过高会导致机器人打滑/抖动
GAZEBO_PHYSICS_MAX_STEP_SIZE="${GAZEBO_PHYSICS_MAX_STEP_SIZE:-0.002}"
GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE:-500}"
GAZEBO_PHYSICS_ODE_ITERS="${GAZEBO_PHYSICS_ODE_ITERS:-40}"
GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL="${GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL:-5.0}"

# ───────── 机器人出生位姿 ─────────
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:-0.4}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

# ───────── 建筑来源控制 ─────────
# LOAD_EXISTING=1 : 强制加载已生成的建筑, 不重新生成
# ASK_BUILDING=1  : 运行时交互询问 重新生成 / 加载已有 (需在终端运行)
LOAD_EXISTING="$(as_ros_bool "${LOAD_EXISTING:-0}")"
ASK_BUILDING="$(as_ros_bool "${ASK_BUILDING:-0}")"

# 后台定时自动取消 Gazebo 暂停:
# 当 AUTO_UNPAUSE=true 时, 延迟 delay 秒后调用 /gazebo/unpause_physics
schedule_unpause_physics() {
  if [ "$AUTO_UNPAUSE" != "true" ]; then
    return
  fi

  (
    sleep "$AUTO_UNPAUSE_DELAY"
    for _ in $(seq 1 40); do
      if rosservice list 2>/dev/null | grep -q '^/gazebo/unpause_physics$'; then
        rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true
        exit 0
      fi
      sleep 0.25
    done
  ) &
}

# 等待机器人模型 spawn 成功:
# 轮询 /gazebo/get_model_state 服务并检查启动日志, 超时或失败时输出日志并退出
wait_for_robot_spawn() {
  local timeout="$ROBOT_SPAWN_TIMEOUT"
  local deadline=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "roslaunch exited during startup. Last log lines:" >&2
      tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
      exit 1
    fi
    if timeout 1s rosservice call /gazebo/get_model_state "{model_name: 'a1_gazebo', relative_entity_name: 'world'}" 2>/dev/null | grep -q "success: True"; then
      return
    fi
    if grep -a -q "Successfully spawned entity" "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>/dev/null; then
      return
    fi
    if grep -a -E -q "Spawn service failed|Service call failed" "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>/dev/null; then
      echo "Robot spawn failed. Last log lines:" >&2
      tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
      exit 1
    fi
    sleep 0.2
  done

  echo "Timed out waiting for robot spawn. Last log lines:" >&2
  tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
  exit 1
}

# ───────── 1. 清理残留的仿真/控制器/摇杆进程 ─────────
echo "Terminating previous Gazebo, launch, controller, and optional joystick processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true

# ───────── 2. 加载 ROS Noetic 与工作空间环境 ─────────
echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE_DIR/devel/setup.bash. Run catkin_make in this workspace before starting the simulation." >&2
  exit 1
fi
source "$WORKSPACE_DIR/devel/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$WORKSPACE_DIR/devel:${CMAKE_PREFIX_PATH:-}"
export PYTHONPATH="$WORKSPACE_DIR/src/building_generator_classic:$WORKSPACE_DIR/src/building_generator_core:${PYTHONPATH:-}"

# ───────── 关键脚本与目录路径 ─────────
GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
BUILDING_CONTROL_SCRIPT="$WORKSPACE_DIR/src/building_generator_classic/scripts/building_generator_classic_control"
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

# ───────── 3. 建筑来源: 重新生成 或 加载已有 ─────────
# 返回 "generate"(重新生成) 或 "load"(加载已有)
choose_building_source() {
  if [ "$LOAD_EXISTING" = "true" ]; then
    printf "load"
  elif [ "$ASK_BUILDING" = "true" ] && [ -t 0 ]; then
    # 交互式询问: 仅当存在已有建筑且有终端输入时提示
    if [ -f "$SCENE_OUTPUT_DIR/competition_scene.world" ]; then
      read -r -p "检测到已有建筑($SCENE_OUTPUT_DIR)。重新生成 [r] / 加载已有 [l]? (默认 r): " _ans
      case "$_ans" in
        l|L|load|加载) printf "load" ;;
        *) printf "generate" ;;
      esac
    else
      printf "generate"
    fi
  else
    printf "generate"
  fi
}

BUILD_SOURCE="$(choose_building_source)"

if [ "$BUILD_SOURCE" = "load" ]; then
  # 加载已有建筑, 跳过重新生成; 校验加载所需的关键文件是否齐全
  for _f in competition_scene.world door_config.yaml elevator_config.yaml danger_truth.json; do
    if [ ! -f "$SCENE_OUTPUT_DIR/$_f" ]; then
      echo "已有建筑缺少文件: $SCENE_OUTPUT_DIR/$_f" >&2
      echo "请先重新生成一次建筑, 或清除 LOAD_EXISTING/ASK_BUILDING 后运行。" >&2
      exit 1
    fi
  done
  echo "加载已有建筑: $SCENE_OUTPUT_DIR"
else
  # 重新生成建筑 + 世界文件
  # 调用生成器脚本, 输出 world / 门 / 电梯 / 危险真值等文件
  echo "Generating competition scene..."
  GENERATOR_ARGS=(
    --output-dir "$SCENE_OUTPUT_DIR"
    --results-dir "$RESULTS_DIR"
    --floor-count "$FLOOR_COUNT"
    --rooms-per-floor "$ROOMS_PER_FLOOR"
    --width "$BUILDING_WIDTH"
    --length "$BUILDING_LENGTH"
    --danger-count "$DANGER_COUNT"
    --distractor-count "$DISTRACTOR_COUNT"
    --robot-x "$ROBOT_X"
    --robot-y "$ROBOT_Y"
    --robot-z "$ROBOT_Z"
    --robot-yaw "$ROBOT_YAW"
  )
  if [ -n "$SEED" ]; then
    GENERATOR_ARGS+=(--seed "$SEED")
  fi
  GENERATOR_ARGS+=(--physics-max-step-size "$GAZEBO_PHYSICS_MAX_STEP_SIZE")
  GENERATOR_ARGS+=(--physics-real-time-update-rate "$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE")
  GENERATOR_ARGS+=(--physics-ode-iters "$GAZEBO_PHYSICS_ODE_ITERS")
  GENERATOR_ARGS+=(--physics-contact-max-correcting-vel "$GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL")
  if [ "$WRITE_GENERATED_TRUTH_COPY" = "false" ]; then
    GENERATOR_ARGS+=(--no-generated-truth-copy)
  fi
  python3 "$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
    > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"
fi

# ───────── 4. 导出供 roslaunch 使用的环境变量 ─────────
export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export UNITREE_LOG_WAIT_WARNINGS
export CONTROLLER_SPAWNER_TIMEOUT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

# ───────── 5. 打印生成结果摘要 ─────────
echo "=========================================="
echo "Competition scene is ready"
echo "  Building source: $BUILD_SOURCE (generate=重新生成 / load=加载已有)"
echo "  Workspace: $WORKSPACE_DIR"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "  Robot pose: x=$ROBOT_X y=$ROBOT_Y z=$ROBOT_Z yaw=$ROBOT_YAW"
echo "  Sensor data default: $ENABLE_SENSOR_DATA"
echo "  Livox lidar: $ENABLE_LIVOX"
echo "  Livox IMU: $ENABLE_LIVOX_IMU"
echo "  RealSense depth camera: $ENABLE_REALSENSE"
echo "  Front RGB camera: $ENABLE_FRONT_CAMERA"
echo "  PointCloud2 converter: $ENABLE_POINTCLOUD_CONVERTER"
echo "  Ground truth topics: $ENABLE_GROUND_TRUTH"
echo "  Referee odom: $ENABLE_REFEREE_ODOM"
echo "  Foot contact sensors: $ENABLE_FOOT_CONTACT_SENSOR"
echo "  Foot force visual: $ENABLE_FOOT_FORCE_VISUAL"
echo "  Gazebo starts paused: $PAUSED"
echo "  Auto unpause: $AUTO_UNPAUSE after ${AUTO_UNPAUSE_DELAY}s"
echo "  Unitree wait warnings: $UNITREE_LOG_WAIT_WARNINGS"
echo "  Robot spawn timeout: ${ROBOT_SPAWN_TIMEOUT}s"
echo "  Controller spawner timeout: ${CONTROLLER_SPAWNER_TIMEOUT}s"
echo "  Gazebo physics: max_step=$GAZEBO_PHYSICS_MAX_STEP_SIZE update_rate=$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE ode_iters=$GAZEBO_PHYSICS_ODE_ITERS"
echo "  Gazebo plugin path: $GAZEBO_PLUGIN_PATH"
echo "=========================================="

# ───────── 6. 可选: 启动虚拟摇杆 (需要 uinput 权限) ─────────
if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

# ───────── 7. 启动 Gazebo + 机器人 + 传感器 ─────────
# 后台运行 roslaunch, 记录 PID 与日志, 等待机器人成功 spawn
echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  controller_spawner_timeout:="$CONTROLLER_SPAWNER_TIMEOUT" \
  enable_sensor_data:="$ENABLE_SENSOR_DATA" \
  enable_livox:="$ENABLE_LIVOX" \
  enable_livox_imu:="$ENABLE_LIVOX_IMU" \
  enable_realsense:="$ENABLE_REALSENSE" \
  enable_front_camera:="$ENABLE_FRONT_CAMERA" \
  enable_referee_odom:="$ENABLE_REFEREE_ODOM" \
  enable_ground_truth:="$ENABLE_GROUND_TRUTH" \
  enable_foot_contact_sensor:="$ENABLE_FOOT_CONTACT_SENSOR" \
  enable_foot_force_visual:="$ENABLE_FOOT_FORCE_VISUAL" \
  enable_joy_node:="$ENABLE_JOY_NODE" \
  enable_pointcloud_converter:="$ENABLE_POINTCLOUD_CONVERTER" \
  pointcloud_use_ground_truth_odom:="$POINTCLOUD_USE_GROUND_TRUTH_ODOM" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
wait_for_robot_spawn

# ───────── 8. 启动建筑门/电梯控制服务 ─────────
if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  python3 "$BUILDING_CONTROL_SCRIPT" \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

# ───────── 9. 启动机器人控制器 (junior_ctrl) ─────────
# 前台模式可接收键盘输入切换状态; 后台模式用于无人值守运行
if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 2 = stand, 4 = RL keyboard walk, 6 = RL /cmd_vel mode, 8 = reset."
    echo "In RL keyboard walk mode: W/S = forward/back, A/D = left/right, J/L = turn, Space = stop."
    schedule_unpause_physics
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl" || true
    echo "junior_ctrl exited; keeping Gazebo running for inspection. Press Ctrl-C to stop this script."
    wait "$LAUNCH_PID"
    exit 0
  else
    echo "Starting junior_ctrl controller in the background. Keyboard state switching may not be available."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl" \
      > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
    schedule_unpause_physics
  fi
else
  schedule_unpause_physics
fi

echo "Simulation startup command completed."
echo "Controller mode remains governed by unitree_guide keyboard/joy input. Mode 4 uses RL with keyboard axes; mode 6 keeps the original RL /cmd_vel logic."
wait "$LAUNCH_PID"
