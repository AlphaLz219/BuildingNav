#!/usr/bin/env bash
# =============================================================================
# generate_building.sh —— 纯建筑生成脚本（不启动 Gazebo / 控制器 / 机器人）
#
# 功能:
#   1. 预设模式 (preset)：small / medium / large / tall / custom
#   2. 批量生成 (batch)：同参数不同种子，一次性生成多栋建筑
#   3. 预览模式 (dry-run)：只打印参数，不实际生成
#   4. 列表模式 (list)：列出已生成的建筑目录及其摘要
#   5. 清理模式 (clean)：归档或删除旧的生成目录
#   6. 自定义输出目录 (--output-dir)
#   7. 屋顶开关 (--roof / --no-roof)：按需选择是否需要屋顶
#
# 用法:
#   ./generate_building.sh                        # 交互式菜单
#   ./generate_building.sh preset small           # 小型建筑
#   ./generate_building.sh preset medium no-roof  # 开顶式中型建筑
#   ./generate_building.sh preset large           # 大型建筑
#   ./generate_building.sh preset tall            # 高层建筑
#   ./generate_building.sh batch 5                # 批量生成 5 栋
#   ./generate_building.sh dry-run preset large   # 预览大建筑参数
#   ./generate_building.sh list                   # 列出已生成建筑
#   ./generate_building.sh clean                  # 清理旧建筑
#   ./generate_building.sh custom --floor-count 2 --rooms 5 --width 25 --length 40 --no-roof
# =============================================================================

set -euo pipefail

# ───────── 工作目录 ─────────
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# ───────── 路径 ─────────
GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
DEFAULT_OUTPUT_BASE="$WORKSPACE_DIR/generated_buildings"   # 批量/预设输出根目录
DEFAULT_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"     # 单次默认目录
RESULTS_DIR="$WORKSPACE_DIR/results"

# ───────── 颜色 ─────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }
header(){ echo -e "\n${BLUE}${BOLD}━━━ $* ━━━${NC}\n"; }

# 解析屋顶参数: 接受 no-roof / false / 0 / off / 无 作为"无屋顶"
parse_roof_arg() {
  case "${1:-true}" in
    no-roof|false|0|off|OFF|无|no) printf "false" ;;
    *) printf "true" ;;
  esac
}

# ───────── 预设定义 ─────────
declare -A PRESET_FLOORS PRESET_ROOMS PRESET_WIDTH PRESET_LENGTH PRESET_DANGER PRESET_DISTRACTOR PRESET_DESC

PRESET_FLOORS[small]="2";    PRESET_ROOMS[small]="3";    PRESET_WIDTH[small]="18";   PRESET_LENGTH[small]="24"
PRESET_DANGER[small]="2:4";  PRESET_DISTRACTOR[small]="3:5"; PRESET_DESC[small]="小型双层 - 2层 每层3间 18x24m"

PRESET_FLOORS[medium]="3";   PRESET_ROOMS[medium]="4";    PRESET_WIDTH[medium]="20";   PRESET_LENGTH[medium]="36"
PRESET_DANGER[medium]="3:6"; PRESET_DISTRACTOR[medium]="4:8"; PRESET_DESC[medium]="中型三层 - 3层 每层4间 20x36m"

PRESET_FLOORS[large]="4";    PRESET_ROOMS[large]="5";     PRESET_WIDTH[large]="28";    PRESET_LENGTH[large]="48"
PRESET_DANGER[large]="5:8";  PRESET_DISTRACTOR[large]="6:10"; PRESET_DESC[large]="大型四层 - 4层 每层5间 28x48m"

PRESET_FLOORS[tall]="6";     PRESET_ROOMS[tall]="3";      PRESET_WIDTH[tall]="18";     PRESET_LENGTH[tall]="30"
PRESET_DANGER[tall]="4:7";   PRESET_DISTRACTOR[tall]="5:9";  PRESET_DESC[tall]="高层六层 - 6层 每层3间 18x30m"

# ───────── ROS 环境 ─────────
source_ros_env() {
  if [ ! -f /opt/ros/noetic/setup.bash ]; then
    err "未找到 /opt/ros/noetic/setup.bash，请确认 ROS Noetic 已安装。"
    exit 1
  fi
  source /opt/ros/noetic/setup.bash

  if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    err "未找到 $WORKSPACE_DIR/devel/setup.bash。请先执行 catkin_make。"
    exit 1
  fi
  source "$WORKSPACE_DIR/devel/setup.bash"
  export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
  export CMAKE_PREFIX_PATH="$WORKSPACE_DIR/devel:${CMAKE_PREFIX_PATH:-}"
  export PYTHONPATH="$WORKSPACE_DIR/src/building_generator_classic:$WORKSPACE_DIR/src/building_generator_core:${PYTHONPATH:-}"
}

# ───────── 生成单栋建筑 ─────────
generate_one() {
  local output_dir="$1"; shift
  local floor_count="$1"; shift
  local rooms_per_floor="$1"; shift
  local width="$1"; shift
  local length="$1"; shift
  local danger_count="$1"; shift
  local distractor_count="$1"; shift
  local seed="${1:-}"; shift
  local include_roof="${1:-true}"

  mkdir -p "$output_dir" "$RESULTS_DIR"

  local args=(
    --output-dir "$output_dir"
    --results-dir "$RESULTS_DIR"
    --floor-count "$floor_count"
    --rooms-per-floor "$rooms_per_floor"
    --width "$width"
    --length "$length"
    --danger-count "$danger_count"
    --distractor-count "$distractor_count"
  )

  if [ -n "$seed" ]; then
    args+=(--seed "$seed")
  fi
  if [ "$include_roof" = "false" ]; then
    args+=(--no-roof)
  fi

  info "生成建筑: $output_dir"
  info "  参数: 楼层=$floor_count 房间/层=$rooms_per_floor 尺寸=${width}x${length}m"
  info "  危险源=$danger_count  干扰物=$distractor_count"
  [ -n "$seed" ] && info "  种子=$seed"
  if [ "$include_roof" = "false" ]; then
    info "  屋顶=无 (开顶式)"
  else
    info "  屋顶=有"
  fi

  python3 "$GENERATOR_SCRIPT" "${args[@]}" \
    > "$output_dir/scene_manifest.stdout.json" 2>&1

  if [ -f "$output_dir/scene_manifest.json" ]; then
    local danger_actual distractor_actual
    danger_actual=$(python3 -c "import json; d=json.load(open('$output_dir/scene_manifest.json')); print(d.get('danger_count','?'))" 2>/dev/null || echo "?")
    distractor_actual=$(python3 -c "import json; d=json.load(open('$output_dir/scene_manifest.json')); print(d.get('distractor_count','?'))" 2>/dev/null || echo "?")
    info "  ✓ 生成完成! 危险源=$danger_actual  干扰物=$distractor_actual"
  else
    err "  ✗ 生成失败，请查看日志: $output_dir/scene_manifest.stdout.json"
    return 1
  fi
}

# ───────── 预设模式 ─────────
cmd_preset() {
  local preset="$1"
  local include_roof; include_roof="$(parse_roof_arg "${2:-true}")"
  if [ -z "${PRESET_FLOORS[$preset]:-}" ]; then
    err "未知预设: $preset"
    echo "可用预设: small medium large tall"
    exit 1
  fi

  header "预设模式: $preset —— ${PRESET_DESC[$preset]}"
  local dir="$DEFAULT_OUTPUT_DIR"
  generate_one "$dir" \
    "${PRESET_FLOORS[$preset]}" \
    "${PRESET_ROOMS[$preset]}" \
    "${PRESET_WIDTH[$preset]}" \
    "${PRESET_LENGTH[$preset]}" \
    "${PRESET_DANGER[$preset]}" \
    "${PRESET_DISTRACTOR[$preset]}" \
    "" \
    "$include_roof"

  echo ""
  info "建筑已生成到: $dir"
  info "场景描述: $dir/scene_manifest.json"
  info "世界文件: $dir/competition_scene.world"
  info "危险真值: $RESULTS_DIR/danger_truth.json"
}

# ───────── 批量生成 ─────────
cmd_batch() {
  local count="${1:-3}"
  if ! [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
    err "批量数量必须为正整数: $count"
    exit 1
  fi

  header "批量生成模式: 共 $count 栋建筑"
  local preset="${2:-medium}"
  local include_roof; include_roof="$(parse_roof_arg "${3:-true}")"
  if [ -z "${PRESET_FLOORS[$preset]:-}" ]; then
    preset="medium"
  fi

  info "使用预设: $preset —— ${PRESET_DESC[$preset]}"
  [ "$include_roof" = "false" ] && info "屋顶: 无 (开顶式)"
  local timestamp; timestamp="$(date +%Y%m%d_%H%M%S)"
  local batch_dir="$DEFAULT_OUTPUT_BASE/batch_${timestamp}"
  mkdir -p "$batch_dir"

  local floor_count="${PRESET_FLOORS[$preset]}"
  local rooms="${PRESET_ROOMS[$preset]}"
  local width="${PRESET_WIDTH[$preset]}"
  local length="${PRESET_LENGTH[$preset]}"
  local danger="${PRESET_DANGER[$preset]}"
  local distractor="${PRESET_DISTRACTOR[$preset]}"

  local success=0 fail=0
  for i in $(seq 1 "$count"); do
    local seed="$(( RANDOM * 32768 + RANDOM ))"
    local dir="$batch_dir/building_$(printf '%03d' "$i")"

    if generate_one "$dir" "$floor_count" "$rooms" "$width" "$length" "$danger" "$distractor" "$seed" "$include_roof"; then
      success=$((success + 1))
    else
      fail=$((fail + 1))
    fi
  done

  # 生成汇总
  cat > "$batch_dir/_batch_summary.json" <<EOF
{
  "batch_timestamp": "$timestamp",
  "preset": "$preset",
  "total": $count,
  "success": $success,
  "failed": $fail,
  "params": {
    "floor_count": "$floor_count",
    "rooms_per_floor": "$rooms",
    "width": "$width",
    "length": "$length",
    "danger_count": "$danger",
    "distractor_count": "$distractor"
  }
}
EOF

  header "批量生成完成: 成功 $success / $fail 失败"
  info "输出目录: $batch_dir"
  info "汇总文件: $batch_dir/_batch_summary.json"
}

# ───────── 自定义模式 ─────────
cmd_custom() {
  local floor_count="3"
  local rooms="4"
  local width="20"
  local length="36"
  local danger="3:6"
  local distractor="4:8"
  local output_dir="$DEFAULT_OUTPUT_DIR"
  local seed=""
  local include_roof="true"

  while [ $# -gt 0 ]; do
    case "$1" in
      --floor-count)   floor_count="$2"; shift 2 ;;
      --rooms)         rooms="$2"; shift 2 ;;
      --width)         width="$2"; shift 2 ;;
      --length)        length="$2"; shift 2 ;;
      --danger-count)  danger="$2"; shift 2 ;;
      --distractor-count) distractor="$2"; shift 2 ;;
      --output-dir)    output_dir="$2"; shift 2 ;;
      --seed)          seed="$2"; shift 2 ;;
      --roof)          include_roof="true"; shift ;;
      --no-roof)       include_roof="false"; shift ;;
      *) err "未知参数: $1"; exit 1 ;;
    esac
  done

  header "自定义模式"
  generate_one "$output_dir" "$floor_count" "$rooms" "$width" "$length" "$danger" "$distractor" "$seed" "$include_roof"
  echo ""
  info "建筑已生成到: $output_dir"
}

# ───────── 预览/dry-run ─────────
cmd_dry_run() {
  local preset="${1:-medium}"
  local include_roof; include_roof="$(parse_roof_arg "${2:-true}")"
  if [ -z "${PRESET_FLOORS[$preset]:-}" ]; then
    err "未知预设: $preset"
    exit 1
  fi

  header "预览模式 (dry-run): $preset"
  printf "  %-25s %s\n" "描述:"       "${PRESET_DESC[$preset]}"
  printf "  %-25s %s\n" "楼层数:"     "${PRESET_FLOORS[$preset]}"
  printf "  %-25s %s\n" "每层房间数:" "${PRESET_ROOMS[$preset]}"
  printf "  %-25s %s m\n" "建筑宽度:" "${PRESET_WIDTH[$preset]}"
  printf "  %-25s %s m\n" "建筑长度:" "${PRESET_LENGTH[$preset]}"
  printf "  %-25s %s\n" "危险源数量:" "${PRESET_DANGER[$preset]}"
  printf "  %-25s %s\n" "干扰物数量:" "${PRESET_DISTRACTOR[$preset]}"
  if [ "$include_roof" = "false" ]; then
    local roof_text="无 (开顶式)"
  else
    local roof_text="有"
  fi
  printf "  %-25s %s\n" "屋顶:"       "$roof_text"
  printf "  %-25s %s\n" "输出目录:"   "$DEFAULT_OUTPUT_DIR"
  printf "  %-25s %s\n" "结果目录:"   "$RESULTS_DIR"
  echo ""
  info "以上为预览，未实际生成。去掉 dry-run 即可执行生成。"
}

# ───────── 列表模式 ─────────
cmd_list() {
  header "已生成的建筑目录"

  local found=0
  # 检查默认目录
  if [ -f "$DEFAULT_OUTPUT_DIR/scene_manifest.json" ]; then
    found=1
    echo -e "  ${CYAN}$DEFAULT_OUTPUT_DIR${NC}"
    python3 -c "
import json
m=json.load(open('$DEFAULT_OUTPUT_DIR/scene_manifest.json'))
print(f\"    种子={m.get('seed','?')}  楼层={m.get('building_config','') and '见config' or '?'}  危险源={m.get('danger_count','?')}  干扰物={m.get('distractor_count','?')}\")
" 2>/dev/null || echo "    (无法读取摘要)"
  fi

  # 检查批量输出目录
  if [ -d "$DEFAULT_OUTPUT_BASE" ]; then
    for batch_dir in "$DEFAULT_OUTPUT_BASE"/*/; do
      [ -d "$batch_dir" ] || continue
      local batch_name; batch_name="$(basename "$batch_dir")"
      found=1
      local count; count="$(find "$batch_dir" -maxdepth 1 -type d -name 'building_*' | wc -l)"
      echo -e "  ${CYAN}$batch_dir${NC}  (${count} 栋)"
    done
  fi

  if [ "$found" -eq 0 ]; then
    info "暂无已生成的建筑。运行 ./generate_building.sh preset medium 开始生成。"
  fi
}

# ───────── 清理模式 ─────────
cmd_clean() {
  header "清理模式"

  local targets=()
  # 默认单次生成目录
  if [ -d "$DEFAULT_OUTPUT_DIR" ]; then
    targets+=("$DEFAULT_OUTPUT_DIR")
  fi
  # 批量输出目录
  if [ -d "$DEFAULT_OUTPUT_BASE" ]; then
    for d in "$DEFAULT_OUTPUT_BASE"/*/; do
      [ -d "$d" ] && targets+=("${d%/}")
    done
  fi

  if [ ${#targets[@]} -eq 0 ]; then
    info "没有需要清理的目录。"
    return
  fi

  echo "以下目录将被清理:"
  for t in "${targets[@]}"; do
    local size; size="$(du -sh "$t" 2>/dev/null | cut -f1)"
    echo "  - $t  ($size)"
  done
  echo ""

  local archive_dir="$WORKSPACE_DIR/archived_buildings_$(date +%Y%m%d_%H%M%S)"
  read -r -p "归档(archive)还是删除(delete)? [a/D] " choice
  case "${choice:-d}" in
    a|A|archive)
      mkdir -p "$archive_dir"
      for t in "${targets[@]}"; do
        mv "$t" "$archive_dir/"
      done
      # 同时移动 results 中的危险真值
      [ -f "$RESULTS_DIR/danger_truth.json" ] && mv "$RESULTS_DIR/danger_truth.json" "$archive_dir/" 2>/dev/null || true
      info "已归档到: $archive_dir"
      ;;
    *)
      for t in "${targets[@]}"; do
        rm -rf "$t"
      done
      [ -f "$RESULTS_DIR/danger_truth.json" ] && rm -f "$RESULTS_DIR/danger_truth.json"
      info "已删除所有生成目录。"
      ;;
  esac
}

# ───────── 交互式菜单 ─────────
interactive_menu() {
  header "建筑生成工具 —— 交互模式"
  echo "  1) 小型建筑 (small)"
  echo "  2) 中型建筑 (medium)"
  echo "  3) 大型建筑 (large)"
  echo "  4) 高层建筑 (tall)"
  echo "  5) 自定义参数"
  echo "  6) 批量生成"
  echo "  7) 预览 (dry-run)"
  echo "  8) 列出已生成建筑"
  echo "  9) 清理旧建筑"
  echo "  0) 退出"
  echo ""
  read -r -p "请选择 [0-9]: " choice

  case "${choice:-0}" in
    1) cmd_preset small ;;
    2) cmd_preset medium ;;
    3) cmd_preset large ;;
    4) cmd_preset tall ;;
    5)
      read -r -p "楼层数 [3]: " fc; fc="${fc:-3}"
      read -r -p "每层房间数 [4]: " rpf; rpf="${rpf:-4}"
      read -r -p "宽度(m) [20]: " w; w="${w:-20}"
      read -r -p "长度(m) [36]: " l; l="${l:-36}"
      read -r -p "危险源数量 [3:6]: " dc; dc="${dc:-3:6}"
      read -r -p "干扰物数量 [4:8]: " dtc; dtc="${dtc:-4:8}"
      read -r -p "输出目录 [$DEFAULT_OUTPUT_DIR]: " od; od="${od:-$DEFAULT_OUTPUT_DIR}"
      read -r -p "种子(留空随机): " seed
      read -r -p "是否需要屋顶 [y/N]: " roof_ans
      local roof_opt="true"
      case "$roof_ans" in
        y|Y|yes|YES) roof_opt="true" ;;
        *) roof_opt="false" ;;
      esac
      generate_one "$od" "$fc" "$rpf" "$w" "$l" "$dc" "$dtc" "$seed" "$roof_opt"
      ;;
    6)
      read -r -p "批量数量 [3]: " cnt; cnt="${cnt:-3}"
      read -r -p "预设 [medium]: " preset; preset="${preset:-medium}"
      read -r -p "是否需要屋顶 [y/N]: " roof_ans
      local roof_opt="true"
      case "$roof_ans" in
        y|Y|yes|YES) roof_opt="true" ;;
        *) roof_opt="false" ;;
      esac
      cmd_batch "$cnt" "$preset" "$roof_opt"
      ;;
    7)
      read -r -p "预设 [medium]: " preset; preset="${preset:-medium}"
      cmd_dry_run "$preset"
      ;;
    8) cmd_list ;;
    9) cmd_clean ;;
    0) info "退出。"; exit 0 ;;
    *) err "无效选项。"; exit 1 ;;
  esac
}

# ───────── 显示预设表 ─────────
show_presets() {
  echo ""
  echo -e "${BOLD}可用预设:${NC}"
  printf "  %-10s %-45s %s\n" "名称" "描述" "参数"
  printf "  %-10s %-45s %s\n" "----------" "---------------------------------------------" "--------------------------"
  for p in small medium large tall; do
    printf "  ${CYAN}%-10s${NC} %-45s 楼层=%s 房间/层=%s %sx%sm\n" \
      "$p" "${PRESET_DESC[$p]}" "${PRESET_FLOORS[$p]}" "${PRESET_ROOMS[$p]}" \
      "${PRESET_WIDTH[$p]}" "${PRESET_LENGTH[$p]}"
  done
  echo ""
}

# ───────── 帮助 ─────────
show_help() {
  echo "用法: $0 [命令] [参数...]"
  echo ""
  echo "命令:"
  echo "  preset <名称> [roof]  使用预设生成 (small|medium|large|tall)"
  echo "  batch <数量> [预设] [roof] 批量生成多栋建筑 (默认 medium)"
  echo "  custom [选项]       自定义参数生成"
  echo "  dry-run <预设> [roof] 预览预设参数，不实际生成"
  echo "  list                列出已生成建筑"
  echo "  clean               归档或删除旧的生成目录"
  echo "  help                显示此帮助"
  echo ""
  echo "custom 选项:"
  echo "  --floor-count N     楼层数 (默认 3)"
  echo "  --rooms N           每层房间数 (默认 4)"
  echo "  --width N           建筑宽度 米 (默认 20)"
  echo "  --length N          建筑长度 米 (默认 36)"
  echo "  --danger-count N:M  危险源数量/范围 (默认 3:6)"
  echo "  --distractor-count N:M 干扰物数量/范围 (默认 4:8)"
  echo "  --output-dir PATH   输出目录 (默认 generated_building)"
  echo "  --seed N            随机种子 (默认随机)"
  echo "  --roof / --no-roof  是否需要屋顶 (默认有; --no-roof 生成开顶式建筑)"
  echo ""
  echo "示例:"
  echo "  $0 preset medium"
  echo "  $0 preset medium no-roof        # 开顶式中型建筑"
  echo "  $0 batch 10 tall no-roof"
  echo "  $0 custom --floor-count 5 --rooms 6 --width 30 --length 50 --no-roof"
  echo "  $0 dry-run large"
}

# ═════════════════════════════════════════════════════════════════════
# 主入口
# ═════════════════════════════════════════════════════════════════════

main() {
  source_ros_env
  show_presets

  if [ $# -eq 0 ]; then
    interactive_menu
    exit 0
  fi

  local cmd="$1"; shift

  case "$cmd" in
    preset)
      cmd_preset "${1:-medium}" "${2:-true}"
      ;;
    batch)
      cmd_batch "${1:-3}" "${2:-medium}" "${3:-true}"
      ;;
    custom)
      cmd_custom "$@"
      ;;
    dry-run|preview)
      cmd_dry_run "${1:-medium}" "${2:-true}"
      ;;
    list|ls)
      cmd_list
      ;;
    clean|clear)
      cmd_clean
      ;;
    help|--help|-h)
      show_help
      ;;
    *)
      err "未知命令: $cmd"
      show_help
      exit 1
      ;;
  esac
}

main "$@"
