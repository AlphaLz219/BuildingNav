#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-true}"
PAUSED="${PAUSED:-true}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
ROBOT_MODEL="${ROBOT_MODEL:-a1}"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--2.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"
BUILDING_GENERATOR="${BUILDING_GENERATOR:-competition}"
INCLUDE_ROOF="${INCLUDE_ROOF:-false}"
INCLUDE_ELEVATOR="${INCLUDE_ELEVATOR:-true}"
INCLUDE_STAIRS="${INCLUDE_STAIRS:-true}"
INCLUDE_FURNITURE="${INCLUDE_FURNITURE:-false}"
FLOOR_HEIGHT="${FLOOR_HEIGHT:-2.6}"
WALL_HEIGHT="${WALL_HEIGHT:-2.35}"
CORRIDOR_WIDTH="${CORRIDOR_WIDTH:-1.6}"
LOBBY_DEPTH="${LOBBY_DEPTH:-4.2}"

echo "Terminating previous Gazebo, launch, controller, and optional joystick processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true

echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"
UNITREE_GAZEBO_MODELS="$(rospack find unitree_gazebo)/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

echo "Generating competition scene..."
GENERATOR_SCRIPT="generate_competition_scene.py"
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_SCRIPT="generate_lightweight_competition_scene.py"
fi
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
if [ "$BUILDING_GENERATOR" = "lightweight" ]; then
  GENERATOR_ARGS+=(
    --floor-height "$FLOOR_HEIGHT"
    --wall-height "$WALL_HEIGHT"
    --corridor-width "$CORRIDOR_WIDTH"
    --lobby-depth "$LOBBY_DEPTH"
  )
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
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi
python3 "$BUILDING_OBSTACLES_DIR/scripts/$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"

echo "=========================================="
echo "Competition scene is ready"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

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
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
sleep 6

if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    --robot-model "$ROBOT_MODEL" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 2 = stand, 6 = RL mode."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
  else
    echo "Starting junior_ctrl controller in the background. Keyboard state switching may not be available."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl" \
      > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
  fi
fi

echo "Simulation startup command completed."
echo "Controller mode remains governed by unitree_guide keyboard/joy input; publish geometry_msgs/Twist to /cmd_vel after RL mode is enabled."
