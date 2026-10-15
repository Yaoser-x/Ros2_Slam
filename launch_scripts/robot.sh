#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-/home/robot/ros2_ws}"

if [ -z "${ROBOT_EFFECTIVE_PARAMS:-}" ] && [ -x "${ROS_WS}/bin/robot" ]; then
    echo "[DEPRECATED] launch_scripts/robot.sh 将在 v0.5.0 删除，请改用 bin/robot。" >&2
    exec "${ROS_WS}/bin/robot" "$@"
fi

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

LIDAR_HEALTH_SCRIPT="${SCRIPT_DIR}/check_lidar_health.sh"
DETECT_LIDAR_PORT_SCRIPT="${SCRIPT_DIR}/detect_lidar_port.sh"
DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
STOP_ALL_SCRIPT="${SCRIPT_DIR}/stop_all.sh"
SAVE_MAP_SCRIPT="${SCRIPT_DIR}/save_map.sh"
CHECK_SYSTEM_SCRIPT="${SCRIPT_DIR}/check_system.sh"
CHECK_CHASSIS_SCRIPT="${SCRIPT_DIR}/check_chassis.sh"
CHECK_TF_SCRIPT="${SCRIPT_DIR}/check_tf.sh"
CHECK_MAPPING_SCRIPT="${SCRIPT_DIR}/check_mapping_pipeline.sh"
CHECK_NAVIGATION_SCRIPT="${SCRIPT_DIR}/check_navigation_pipeline.sh"
KEYBOARD_CONTROL_SCRIPT="${SCRIPT_DIR}/keyboard_control.sh"
BASE_TEST_SCRIPT="${SCRIPT_DIR}/test_base_cmd.sh"

DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
DEFAULT_SLAM_PARAMS="${ROBOT_SLAM_PARAMS_QUALITY:-${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping.yaml}"
PRECISION_SLAM_PARAMS="${ROBOT_SLAM_PARAMS_PRECISION:-}"
FAST_SLAM_PARAMS="${ROBOT_SLAM_PARAMS_FAST:-}"
DEFAULT_NAV2_MAPPING_PARAMS="${ROS_WS}/src/robot_bringup/config/nav2_mapping_params.yaml"
DEFAULT_NAV2_BT_XML="${ROS_WS}/src/robot_bringup/behavior_trees/navigate_to_pose_recovery.xml"
DEFAULT_LIDAR_PARAMS="${ROBOT_LIDAR_PARAMS:-${ROS_WS}/src/robot_sensing/config/ydlidar_x2.yaml}"
DEFAULT_LIDAR_FALLBACK_PARAMS="${ROS_WS}/src/vendor/ydlidar_ros2_driver/params/X2.yaml"
DEFAULT_LIDAR_RVIZ="${ROS_WS}/src/robot_bringup/rviz/lidar_mapping.rviz"
DEFAULT_SYSTEM_RVIZ="${ROS_WS}/src/robot_bringup/rviz/system.rviz"
DEFAULT_LIDAR_TF_YAW_RAD="1.570796326795"
DEFAULT_BASE_PORT="/dev/serial0"

TMP_FILES=()

cleanup() {
    local file
    for file in "${TMP_FILES[@]:-}"; do
        if [ -n "$file" ] && [ -f "$file" ]; then
            rm -f "$file"
        fi
    done
}

trap cleanup EXIT

register_tmp_file() {
    TMP_FILES+=("$1")
}

main_usage() {
    cat <<'EOF'
统一入口: ./bin/robot <command> [args]

命令:
  mapping       启动建图
  navigation    启动导航
  sensor        启动单一传感器
  base          启动 STM32 底盘桥接
  base-test     只测试上位机 /cmd_vel/test 控制底盘
  full          启动全量观测系统（雷达 + 相机 + 底盘 + 建图）
  check         执行健康检查（lidar 或 mapping）
  save-map      保存当前地图
  doctor        执行系统诊断
  teleop        启动键盘遥控
  stop          停止当前 ROS2 会话
  help          查看帮助

示例:
  ./bin/robot mapping lidar --manual --real-base
  ./bin/robot save-map my_map
  ./bin/robot navigation --real-base --ekf-base
  ./bin/robot navigation /home/robot/ros2_maps/my_map.yaml --real-base
  ./bin/robot base-test --rotate-only
  ./bin/robot base-test --calibrate-angular
  ./bin/robot sensor lidar
  ./bin/robot check lidar
  ./bin/robot doctor
EOF
}

mapping_usage() {
    cat <<'EOF'
用法: ./bin/robot mapping [lidar] [quality|precision|fast] [--manual] [--skip-lidar-check] [--rviz|--no-rviz] [--real-base|--fake-base] [--ekf-base] [--base-port PORT] [--lidar-port PATH] [--lidar-yaw-rad RAD|--lidar-yaw-deg DEG] [--lidar-reversion|--no-lidar-reversion] [--lidar-inverted|--no-lidar-inverted]

说明:
  --manual 为正式建图入口；旧 --auto/frontier_explorer 已移到 experiments/legacy，不属于默认平台启动链。
  可用 AUTO_MAPPING_GOAL_CLEARANCE_RADIUS / AUTO_MAPPING_GOAL_UNKNOWN_CLEARANCE_RADIUS 临时放宽或收紧贴墙距离。
  lidar 源建图时可用 --lidar-reversion / --lidar-inverted / --lidar-yaw-* 做运行时覆盖。
EOF
}

navigation_usage() {
    cat <<'EOF'
用法: ./bin/robot navigation [map.yaml] [--real-base|--fake-base] [--ekf-base] [--lidar-port PATH] [--lidar-yaw-rad RAD|--lidar-yaw-deg DEG] [--lidar-reversion|--no-lidar-reversion] [--lidar-inverted|--no-lidar-inverted] [--rviz|--no-rviz] [--skip-lidar-check] [--base-port PORT] [--localization-only|--nav2-only]

说明:
  map.yaml 省略时默认使用 /home/robot/ros2_maps/latest.yaml。
  默认 headless；需要 RViz 时追加 --rviz，再用 2D Pose Estimate 设置当前位置。
  默认正式流程是单阶段 navigation；只有定位未就绪或现场异常时，才回退到下面两个模式。
  --localization-only 只启动雷达、底盘、AMCL 和 RViz，先完成初始定位。
  --nav2-only         在初始定位完成后，单独启动 Nav2 规划和控制节点；会先检查 AMCL 是否就绪。
  若 2D Pose Estimate 箭头与点云朝向不一致，优先测试 --lidar-reversion / --lidar-inverted，再考虑 lidar_tf_yaw。
EOF
}

check_usage() {
    cat <<'EOF'
用法:
  ./robot.sh check lidar [雷达参数文件或 --lidar-port PATH]
  ./robot.sh check mapping [--allow-headless] [--min-scan-hz N]
  ./robot.sh check navigation
  ./robot.sh check chassis
  ./robot.sh check tf
EOF
}

resolve_base_port() {
    local port="$1"

    if [ "$port" = "auto" ]; then
        if [ -n "${ROBOT_EFFECTIVE_PARAMS:-}" ]; then
            log_error "compiled base_port cannot be auto; set hardware.base_port or --base-port" >&2
            return 1
        fi
        log_warn "base_port=auto 已废弃；当前底盘默认使用 Raspberry Pi GPIO UART ${DEFAULT_BASE_PORT}" >&2
        printf '%s\n' "${ROBOT_BASE_PORT_HINT:-$DEFAULT_BASE_PORT}"
        return 0
    fi

    printf '%s\n' "$port"
}

resolve_base_port_avoiding_lidar() {
    local mode_label="$1"
    local requested_base_port="$2"
    local lidar_port="$3"
    local base_port_explicit="$4"
    local resolved_base=""
    local resolved_lidar=""
    local resolved_base_port=""

    if [ -z "$lidar_port" ]; then
        resolve_base_port "$requested_base_port"
        return 0
    fi

    resolved_lidar="$(resolve_port "$lidar_port")"

    if [ "$requested_base_port" != "auto" ]; then
        resolved_base="$(resolve_port "$requested_base_port")"
        if [ "$resolved_base" = "$resolved_lidar" ]; then
            if [ -n "${ROBOT_EFFECTIVE_PARAMS:-}" ]; then
                log_error "compiled base and lidar ports conflict; recompile with --base-port" >&2
                return 1
            fi
            if [ "$base_port_explicit" = true ]; then
                log_error "✗ 底盘串口和雷达串口相同: ${requested_base_port}"
                log_error "  雷达已经占用 ${lidar_port}，请用 --base-port 指定另一个 STM32 串口，或交换 USB 后重试。"
                exit 1
            fi
            log_warn "检测到默认底盘串口 ${requested_base_port} 与雷达串口 ${lidar_port} 冲突，改用 GPIO UART ${DEFAULT_BASE_PORT}" >&2
            requested_base_port="$DEFAULT_BASE_PORT"
        fi
    fi

    resolved_base_port="$(resolve_base_port "$requested_base_port")"
    if [ ! -e "$resolved_base_port" ]; then
        log_error "✗ 底盘串口不存在: ${resolved_base_port}"
        log_error "  请确认 Raspberry Pi GPIO UART 已启用，/dev/serial0 存在，或用 --base-port 指定实际底盘串口。"
        exit 1
    fi

    resolved_base="$(resolve_port "$resolved_base_port")"
    if [ "$resolved_base" = "$resolved_lidar" ]; then
        log_error "✗ 底盘串口和雷达串口仍然相同: ${resolved_base_port}"
        log_error "  请确认底盘使用 /dev/serial0，或用 --base-port 指定不与雷达冲突的串口。"
        exit 1
    fi

    printf '%s\n' "$resolved_base_port"
}

prepare_lidar_params() {
    local params_file="$1"
    local configured_port=""

    if [ -z "$params_file" ] || [ ! -f "$params_file" ]; then
        log_error "✗ 雷达参数文件不存在: ${params_file:-<empty>}"
        exit 1
    fi

    configured_port="$(get_yaml_key_value "$params_file" "port" || true)"
    if [ -z "$configured_port" ]; then
        log_error "✗ 雷达参数文件缺少 port: ${params_file}"
        exit 1
    fi

    printf '%s\n' "$params_file"
}

prepare_lidar_params_with_runtime_overrides() {
    local params_file="$1"
    local reversion_override="${2:-}"
    local inverted_override="${3:-}"
    local port_override="${4:-}"
    local tmp_file=""

    if [ -n "${ROBOT_EFFECTIVE_CONFIG:-}" ]; then
        if { [ -n "$reversion_override" ] && [ "$(get_yaml_key_value "$params_file" reversion)" != "$reversion_override" ]; } \
            || { [ -n "$inverted_override" ] && [ "$(get_yaml_key_value "$params_file" inverted)" != "$inverted_override" ]; } \
            || { [ -n "$port_override" ] && [ "$(get_yaml_key_value "$params_file" port)" != "$port_override" ]; }; then
            log_error "雷达覆写与已编译配置不一致，请通过 bin/robot 重新编译。"
            return 1
        fi
        printf '%s\n' "$params_file"
        return 0
    fi

    if [ -z "$reversion_override" ] && [ -z "$inverted_override" ] && [ -z "$port_override" ]; then
        printf '%s\n' "$params_file"
        return 0
    fi

    tmp_file="$(mktemp /tmp/robot_lidar_params.XXXXXX.yaml)"
    cp "$params_file" "$tmp_file"
    register_tmp_file "$tmp_file"

    if [ -n "$reversion_override" ]; then
        render_yaml_key_value "$tmp_file" "reversion" "$reversion_override" "$tmp_file.tmp"
        mv "$tmp_file.tmp" "$tmp_file"
    fi
    if [ -n "$inverted_override" ]; then
        render_yaml_key_value "$tmp_file" "inverted" "$inverted_override" "$tmp_file.tmp"
        mv "$tmp_file.tmp" "$tmp_file"
    fi
    if [ -n "$port_override" ]; then
        render_yaml_key_value "$tmp_file" "port" "$port_override" "$tmp_file.tmp"
        mv "$tmp_file.tmp" "$tmp_file"
    fi

    printf '%s\n' "$tmp_file"
}

resolve_lidar_params_device() {
    local params_file="$1"
    local explicit_port_override="${2:-}"
    local configured_port=""
    local detected_port=""

    configured_port="$(get_yaml_key_value "$params_file" "port" || true)"
    if [ -z "$configured_port" ]; then
        log_error "✗ 雷达参数文件缺少 port: ${params_file}"
        exit 1
    fi

    if [ -e "$configured_port" ]; then
        printf '%s\n' "$params_file"
        return 0
    fi

    if [ -n "${ROBOT_EFFECTIVE_CONFIG:-}" ]; then
        log_error "已编译雷达设备不存在: $configured_port；请使用 --lidar-port 重新编译。"
        return 1
    fi

    if [ -n "$explicit_port_override" ]; then
        log_error "✗ 指定的雷达设备不存在: ${configured_port}"
        exit 1
    fi

    if [ -f "$DETECT_LIDAR_PORT_SCRIPT" ]; then
        detected_port="$(bash "$DETECT_LIDAR_PORT_SCRIPT" "$configured_port" 2>/dev/null || true)"
    fi

    if [ -n "$detected_port" ] && [ -e "$detected_port" ]; then
        log_warn "未找到 ${configured_port}，自动使用检测到的雷达串口: ${detected_port}" >&2
        prepare_lidar_params_with_runtime_overrides "$params_file" "" "" "$detected_port"
        return 0
    fi

    log_error "✗ 未找到雷达设备"
    log_error "  已检查: ${configured_port}"
    log_error "  可用时建议建立 /dev/ydlidar，或启动时传 --lidar-port /dev/ttyUSBx"
    exit 1
}

ensure_lidar_port_access() {
    local lidar_port="$1"

    if [ -r "$lidar_port" ] && [ -w "$lidar_port" ]; then
        return 0
    fi

    log_error "✗ 当前用户没有雷达串口读写权限: ${lidar_port}"
    log_error "  当前用户组: $(id -nG)"
    log_error "  设备权限: $(ls -l "$lidar_port")"
    log_error "  请在树莓派本机执行一次:"
    log_error "    sudo /home/robot/ros2_ws/launch_scripts/setup_lidar_permissions.sh"
    log_error "  临时测试也可执行:"
    log_error "    sudo chmod 666 ${lidar_port}"
    log_error "  加入 dialout 后需要重新登录，或重启终端/系统。"
    exit 1
}

resolve_default_lidar_params() {
    if [ -f "$DEFAULT_LIDAR_PARAMS" ]; then
        printf '%s\n' "$DEFAULT_LIDAR_PARAMS"
        return 0
    fi
    if [ -f "$DEFAULT_LIDAR_FALLBACK_PARAMS" ]; then
        printf '%s\n' "$DEFAULT_LIDAR_FALLBACK_PARAMS"
        return 0
    fi

    log_error "✗ 未找到默认雷达参数文件"
    log_error "  已检查: ${DEFAULT_LIDAR_PARAMS}"
    log_error "  已检查: ${DEFAULT_LIDAR_FALLBACK_PARAMS}"
    exit 1
}

require_existing_file() {
    local file_path="$1"
    local label="$2"

    if [ -z "$file_path" ] || [ ! -f "$file_path" ]; then
        log_error "✗ ${label}不存在: ${file_path:-<empty>}"
        exit 1
    fi
}

choose_slam_params() {
    local profile="$1"

    case "$profile" in
        quality)
            printf '%s\n' "$DEFAULT_SLAM_PARAMS"
            ;;
        precision)
            printf '%s\n' "$PRECISION_SLAM_PARAMS"
            ;;
        fast)
            printf '%s\n' "$FAST_SLAM_PARAMS"
            ;;
        *)
            log_error "✗ 未知 SLAM 参数档位: $profile"
            exit 1
            ;;
    esac
}

deg_to_rad() {
    awk -v deg="$1" 'BEGIN { printf "%.12f\n", deg * atan2(0, -1) / 180.0 }'
}

stop_existing_sessions() {
    if [ -f "$STOP_ALL_SCRIPT" ]; then
        log_info "清理旧 ROS2 会话..."
        bash "$STOP_ALL_SCRIPT" >/dev/null 2>&1 || true
        sleep 1
    fi
}

ensure_localization_ready_for_nav2() {
    local amcl_pose_status=0

    if ! ros2_node_exists "/map_server"; then
        log_error "✗ 未检测到 /map_server。请先运行 ./robot.sh navigation --localization-only"
        exit 1
    fi
    if ! ros2_node_exists "/amcl"; then
        log_error "✗ 未检测到 /amcl。请先运行 ./robot.sh navigation --localization-only"
        exit 1
    fi
    if ! ros2_lifecycle_is_active "/map_server"; then
        log_error "✗ /map_server 当前不在 active 状态，不能直接启动 --nav2-only"
        exit 1
    fi
    if ! ros2_lifecycle_is_active "/amcl"; then
        log_error "✗ /amcl 当前不在 active 状态，不能直接启动 --nav2-only"
        exit 1
    fi

    if ros2_wait_for_topic_message "/amcl_pose" 8; then
        amcl_pose_status=0
    else
        amcl_pose_status=$?
    fi

    case "$amcl_pose_status" in
        0)
            log_success "检测到 /amcl_pose，定位链已就绪。"
            ;;
        2)
            log_warn "⚠ 系统缺少 timeout，跳过 /amcl_pose 消息等待；继续按节点状态启动 Nav2。"
            ;;
        *)
            log_error "✗ 8 秒内未收到 /amcl_pose。请先在 RViz 执行 2D Pose Estimate 并确认定位稳定。"
            exit 1
            ;;
    esac
}

run_mapping() {
    local mapping_source="lidar"
    local slam_profile="auto"
    local skip_lidar_check=false
    local use_rviz=false
    local base_mode="${ROBOT_BASE_MODE:-none}"
    local base_port="${BASE_PORT:-$DEFAULT_BASE_PORT}"
    local has_source=false
    local has_profile=false
    local slam_params=""
    local lidar_params=""
    local rviz_config="$DEFAULT_SYSTEM_RVIZ"
    local use_base_arg="false"
    local base_fusion_mode="${ROBOT_BASE_FUSION_MODE:-wheel}"
    local auto_mapping_drive="false"
    local auto_mapping_nav2_params="${AUTO_MAPPING_NAV2_PARAMS:-$DEFAULT_NAV2_MAPPING_PARAMS}"
    local auto_mapping_max_duration_sec="${AUTO_MAPPING_MAX_DURATION_SEC:-180.0}"
    local auto_mapping_frontier_min_distance="${AUTO_MAPPING_FRONTIER_MIN_DISTANCE:-0.70}"
    local auto_mapping_frontier_max_distance="${AUTO_MAPPING_FRONTIER_MAX_DISTANCE:-2.5}"
    local auto_mapping_min_frontier_cluster_cells="${AUTO_MAPPING_MIN_FRONTIER_CLUSTER_CELLS:-10}"
    local auto_mapping_goal_approach_offset="${AUTO_MAPPING_GOAL_APPROACH_OFFSET:-0.55}"
    local auto_mapping_goal_approach_max_offset="${AUTO_MAPPING_GOAL_APPROACH_MAX_OFFSET:-0.90}"
    local auto_mapping_goal_approach_step="${AUTO_MAPPING_GOAL_APPROACH_STEP:-0.05}"
    local auto_mapping_goal_clearance_radius="${AUTO_MAPPING_GOAL_CLEARANCE_RADIUS:-0.38}"
    local auto_mapping_goal_unknown_clearance_radius="${AUTO_MAPPING_GOAL_UNKNOWN_CLEARANCE_RADIUS:-0.35}"
    local auto_mapping_linear_speed="${AUTO_MAPPING_LINEAR_SPEED:-0.10}"
    local auto_mapping_turn_speed="${AUTO_MAPPING_TURN_SPEED:-1.65}"
    local auto_mapping_emergency_stop_distance="${AUTO_MAPPING_EMERGENCY_STOP_DISTANCE:-0.25}"
    local auto_mapping_front_stop_distance="${AUTO_MAPPING_FRONT_STOP_DISTANCE:-0.40}"
    local auto_mapping_front_resume_distance="${AUTO_MAPPING_FRONT_RESUME_DISTANCE:-0.48}"
    local auto_mapping_front_slow_distance="${AUTO_MAPPING_FRONT_SLOW_DISTANCE:-0.80}"
    local auto_mapping_side_emergency_stop_distance="${AUTO_MAPPING_SIDE_EMERGENCY_STOP_DISTANCE:-0.12}"
    local auto_mapping_side_stop_distance="${AUTO_MAPPING_SIDE_STOP_DISTANCE:-0.10}"
    local auto_mapping_side_resume_distance="${AUTO_MAPPING_SIDE_RESUME_DISTANCE:-0.18}"
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local lidar_reversion_override=""
    local lidar_inverted_override=""
    local lidar_port_override=""
    local lidar_port=""
    local base_port_explicit=false
    local arg=""

    while [ $# -gt 0 ]; do
        arg="$1"
        case "$arg" in
            camera|lidar)
                if [ "$has_source" = true ]; then
                    log_error "✗ 重复指定建图源: $arg"
                    mapping_usage
                    exit 1
                fi
                mapping_source="$arg"
                has_source=true
                shift
                ;;
            quality|precision|fast)
                if [ "$has_profile" = true ]; then
                    log_error "✗ 重复指定参数档位: $arg"
                    mapping_usage
                    exit 1
                fi
                slam_profile="$arg"
                has_profile=true
                shift
                ;;
            --manual)
                auto_mapping_drive="false"
                shift
                ;;
            --auto|--auto-drive)
                auto_mapping_drive="true"
                shift
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --rviz)
                use_rviz=true
                shift
                ;;
            --real-base)
                base_mode="real"
                use_base_arg="true"
                shift
                ;;
            --fake-base)
                base_mode="fake"
                use_base_arg="true"
                shift
                ;;
            --ekf-base)
                base_fusion_mode="ekf"
                shift
                ;;
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    mapping_usage
                    exit 1
                fi
                base_port="$2"
                base_port_explicit=true
                shift 2
                ;;
            --lidar-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-port 需要一个串口路径"
                    mapping_usage
                    exit 1
                fi
                lidar_port_override="$2"
                shift 2
                ;;
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    mapping_usage
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    mapping_usage
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
                ;;
            --lidar-reversion)
                lidar_reversion_override="true"
                shift
                ;;
            --no-lidar-reversion)
                lidar_reversion_override="false"
                shift
                ;;
            --lidar-inverted)
                lidar_inverted_override="true"
                shift
                ;;
            --no-lidar-inverted)
                lidar_inverted_override="false"
                shift
                ;;
            --auto-drive-duration)
                if [ $# -lt 2 ]; then
                    log_error "✗ --auto-drive-duration 需要秒数"
                    mapping_usage
                    exit 1
                fi
                auto_mapping_max_duration_sec="$2"
                shift 2
                ;;
            -h|--help)
                mapping_usage
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $arg"
                mapping_usage
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "建图模式启动"

    if [ "$slam_profile" = "auto" ]; then
        if [ "$mapping_source" = "lidar" ]; then
            slam_profile="quality"
        else
            slam_profile="fast"
        fi
    fi
    slam_params="$(choose_slam_params "$slam_profile")"
    require_existing_file "$slam_params" "SLAM 参数文件"

    if [ "$mapping_source" = "lidar" ]; then
        rviz_config="$DEFAULT_LIDAR_RVIZ"
        lidar_params="$(resolve_default_lidar_params)"
        lidar_params="$(prepare_lidar_params "$lidar_params")"
        lidar_params="$(prepare_lidar_params_with_runtime_overrides "$lidar_params" "$lidar_reversion_override" "$lidar_inverted_override" "$lidar_port_override")"
        lidar_params="$(resolve_lidar_params_device "$lidar_params" "$lidar_port_override")"
        lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
        ensure_lidar_port_access "$lidar_port"
        export ROBOT_LIDAR_PORT_HINT="$lidar_port"
        if [ "$skip_lidar_check" = false ] && [ -f "$LIDAR_HEALTH_SCRIPT" ]; then
            log_info "执行雷达数据健康检查..."
            bash "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
        fi
    fi

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port_avoiding_lidar "建图模式" "$base_port" "$lidar_port" "$base_port_explicit")"
        export ROBOT_BASE_PORT_HINT="$base_port"
    fi

    if [ "$auto_mapping_drive" = "true" ]; then
        require_existing_file "$auto_mapping_nav2_params" "自动建图 Nav2 参数文件"
    fi
    if [ "$use_rviz" = true ]; then
        require_existing_file "$rviz_config" "RViz 配置文件"
    fi

    log_info "系统组件:"
    if [ "$mapping_source" = "camera" ]; then
        echo "  • Astra Pro 深度相机 -> depthimage_to_laserscan"
    else
        echo "  • YDLIDAR X2 激光雷达"
    fi
    case "$base_mode" in
        real)
            echo "  • STM32 底盘里程计 (${base_port})"
            ;;
        fake)
            echo "  • 虚拟底盘里程计"
            ;;
        *)
            echo "  • 底盘里程计已禁用"
            ;;
    esac
    echo "  • SLAM Toolbox 建图引擎"
    if [ "$use_rviz" = true ]; then
        echo "  • RViz2 可视化界面"
    else
        echo "  • RViz2 已禁用"
    fi
    echo

    stop_existing_sessions

    log_success "正在启动统一建图系统..."
    log_info "入口: robot_bringup/system.launch.py"
    log_info "建图源: ${mapping_source}"
    log_info "参数档位: ${slam_profile}"
    log_info "SLAM 参数: ${slam_params}"
    if [ "$mapping_source" = "lidar" ]; then
        log_info "雷达参数: ${lidar_params}"
        log_info "雷达串口: ${lidar_port}"
        log_info "雷达 yaw 外参(rad): ${lidar_tf_yaw}"
        if [ -n "$lidar_reversion_override" ]; then
            log_info "雷达 reversion 覆写: ${lidar_reversion_override}"
        fi
        if [ -n "$lidar_inverted_override" ]; then
            log_info "雷达 inverted 覆写: ${lidar_inverted_override}"
        fi
    fi
    log_info "底盘模式: ${base_mode}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘融合模式: ${base_fusion_mode}"
    fi
    if [ "$auto_mapping_drive" = "true" ]; then
        log_warn "自动建图巡航已启用: 请保持人工急停可用，不要让 PS2/ESP 同时抢控制"
        log_info "自动探索: Nav2 frontier, ${auto_mapping_max_duration_sec}s"
        log_info "frontier 安全点: dist=${auto_mapping_frontier_min_distance}-${auto_mapping_frontier_max_distance}m approach=${auto_mapping_goal_approach_offset}-${auto_mapping_goal_approach_max_offset}m clear=${auto_mapping_goal_clearance_radius}m unknown_clear=${auto_mapping_goal_unknown_clearance_radius}m"
        log_info "Nav2 建图参数: ${auto_mapping_nav2_params}"
    fi
    if [ "$use_rviz" = true ]; then
        log_info "RViz 配置: ${rviz_config}"
    else
        log_info "RViz: headless"
    fi
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=mapping \
        mapping_source:=${mapping_source} \
        base_mode:=${base_mode} \
        use_base:=${use_base_arg} \
        base_port:=${base_port} \
        base_fusion_mode:=${base_fusion_mode} \
        auto_mapping_drive:=${auto_mapping_drive} \
        auto_mapping_nav2_params_file:=${auto_mapping_nav2_params} \
        auto_mapping_max_duration_sec:=${auto_mapping_max_duration_sec} \
        auto_mapping_frontier_min_distance:=${auto_mapping_frontier_min_distance} \
        auto_mapping_frontier_max_distance:=${auto_mapping_frontier_max_distance} \
        auto_mapping_min_frontier_cluster_cells:=${auto_mapping_min_frontier_cluster_cells} \
        auto_mapping_goal_approach_offset:=${auto_mapping_goal_approach_offset} \
        auto_mapping_goal_approach_max_offset:=${auto_mapping_goal_approach_max_offset} \
        auto_mapping_goal_approach_step:=${auto_mapping_goal_approach_step} \
        auto_mapping_goal_clearance_radius:=${auto_mapping_goal_clearance_radius} \
        auto_mapping_goal_unknown_clearance_radius:=${auto_mapping_goal_unknown_clearance_radius} \
        auto_mapping_linear_speed:=${auto_mapping_linear_speed} \
        auto_mapping_turn_speed:=${auto_mapping_turn_speed} \
        auto_mapping_emergency_stop_distance:=${auto_mapping_emergency_stop_distance} \
        auto_mapping_front_stop_distance:=${auto_mapping_front_stop_distance} \
        auto_mapping_front_resume_distance:=${auto_mapping_front_resume_distance} \
        auto_mapping_front_slow_distance:=${auto_mapping_front_slow_distance} \
        auto_mapping_side_emergency_stop_distance:=${auto_mapping_side_emergency_stop_distance} \
        auto_mapping_side_stop_distance:=${auto_mapping_side_stop_distance} \
        auto_mapping_side_resume_distance:=${auto_mapping_side_resume_distance} \
        use_rviz:=${use_rviz} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
        lidar_params_file:=${lidar_params:-$DEFAULT_LIDAR_PARAMS} \
        rviz_config:=${rviz_config} \
        slam_params_file:=${slam_params}
}

run_navigation() {
    local map_file="$DEFAULT_MAP"
    local base_mode="${ROBOT_BASE_MODE:-real}"
    local base_port="${BASE_PORT:-$DEFAULT_BASE_PORT}"
    local base_fusion_mode="${ROBOT_BASE_FUSION_MODE:-wheel}"
    local use_rviz=false
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""
    local rviz_config="$DEFAULT_SYSTEM_RVIZ"
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local lidar_reversion_override=""
    local lidar_inverted_override=""
    local lidar_port_override=""
    local base_port_explicit=false
    local localization_only=false
    local nav2_only=false
    local nav2_params_file="${ROBOT_NAV2_PARAMS:-${ROS_WS}/src/robot_bringup/config/nav2_params_robot.yaml}"
    local nav2_bt_xml_file="${DEFAULT_NAV2_BT_XML}"
    local map_file_explicit=false
    local arg=""

    while [ $# -gt 0 ]; do
        arg="$1"
        case "$arg" in
            --fake-base)
                base_mode="fake"
                shift
                ;;
            --real-base)
                base_mode="real"
                shift
                ;;
            --ekf-base)
                base_fusion_mode="ekf"
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --rviz)
                use_rviz=true
                shift
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --localization-only)
                localization_only=true
                shift
                ;;
            --nav2-only)
                nav2_only=true
                shift
                ;;
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    navigation_usage
                    exit 1
                fi
                base_port="$2"
                base_port_explicit=true
                shift 2
                ;;
            --lidar-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-port 需要一个串口路径"
                    navigation_usage
                    exit 1
                fi
                lidar_port_override="$2"
                shift 2
                ;;
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    navigation_usage
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    navigation_usage
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
                ;;
            --lidar-reversion)
                lidar_reversion_override="true"
                shift
                ;;
            --no-lidar-reversion)
                lidar_reversion_override="false"
                shift
                ;;
            --lidar-inverted)
                lidar_inverted_override="true"
                shift
                ;;
            --no-lidar-inverted)
                lidar_inverted_override="false"
                shift
                ;;
            -h|--help)
                navigation_usage
                exit 0
                ;;
            *)
                if [[ "$arg" == --* ]]; then
                    log_error "✗ 未知参数: $arg"
                    navigation_usage
                    exit 1
                fi
                if [ "$map_file_explicit" = true ]; then
                    log_error "✗ 重复指定地图文件: $arg"
                    navigation_usage
                    exit 1
                fi
                map_file="$arg"
                map_file_explicit=true
                shift
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    if [ "$localization_only" = true ] && [ "$nav2_only" = true ]; then
        log_error "✗ --localization-only 和 --nav2-only 不能同时使用"
        navigation_usage
        exit 1
    fi

    if [ "$nav2_only" = true ]; then
        print_header "Nav2 规划控制启动"
        require_existing_file "$nav2_params_file" "Nav2 参数文件"
        require_existing_file "$nav2_bt_xml_file" "Nav2 行为树文件"
        log_info "前提: 已经另一个终端运行 ./robot.sh navigation --localization-only"
        log_info "前提: RViz 已用 2D Pose Estimate 完成初始定位，并且 map->base_link TF 可用"
        log_info "Nav2 参数: ${nav2_params_file}"
        log_info "Nav2 BT: ${nav2_bt_xml_file}"
        echo
        ensure_localization_ready_for_nav2
        ros2 launch robot_bringup nav2.launch.py \
            use_sim_time:=false \
            params_file:=${nav2_params_file} \
            default_bt_xml_filename:=${nav2_bt_xml_file} \
            autostart:=true
        return
    fi

    print_header "导航模式启动"

    require_existing_file "$map_file" "地图文件"
    require_existing_file "$nav2_params_file" "Nav2 参数文件"
    require_existing_file "$nav2_bt_xml_file" "Nav2 行为树文件"
    if [ "$use_rviz" = true ]; then
        require_existing_file "$rviz_config" "RViz 配置文件"
    fi

    lidar_params="$(resolve_default_lidar_params)"
    lidar_params="$(prepare_lidar_params "$lidar_params")"
    lidar_params="$(prepare_lidar_params_with_runtime_overrides "$lidar_params" "$lidar_reversion_override" "$lidar_inverted_override" "$lidar_port_override")"
    lidar_params="$(resolve_lidar_params_device "$lidar_params" "$lidar_port_override")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    ensure_lidar_port_access "$lidar_port"
    export ROBOT_LIDAR_PORT_HINT="$lidar_port"

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port_avoiding_lidar "导航模式" "$base_port" "$lidar_port" "$base_port_explicit")"
        export ROBOT_BASE_PORT_HINT="$base_port"
    fi

    if [ "$skip_lidar_check" = false ] && [ -f "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        bash "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    if [ "$localization_only" = true ]; then
        log_success "正在启动定位阶段（雷达 + 底盘 + AMCL + RViz，不启动 Nav2）..."
    else
        log_success "正在启动统一导航系统..."
    fi
    local odom_source_label="bridge odom"
    if [ "$base_fusion_mode" = "ekf" ]; then
        odom_source_label="ekf odom"
    fi
    log_info "地图文件: ${map_file}"
    log_info "雷达参数: ${lidar_params}"
    log_info "雷达串口: ${lidar_port}"
    log_info "雷达 yaw 外参(rad): ${lidar_tf_yaw}"
    log_info "Nav2 BT: ${nav2_bt_xml_file}"
    if [ -n "$lidar_reversion_override" ]; then
        log_info "雷达 reversion 覆写: ${lidar_reversion_override}"
    fi
    if [ -n "$lidar_inverted_override" ]; then
        log_info "雷达 inverted 覆写: ${lidar_inverted_override}"
    fi
    log_info "底盘模式: ${base_mode}"
    log_info "导航 odom 来源: ${odom_source_label}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘融合模式: ${base_fusion_mode}"
    fi
    if [ "$localization_only" = false ] && [ -f "$CHECK_NAVIGATION_SCRIPT" ]; then
        log_info "若 RViz 点 2D Goal Pose 后没有 /plan 或车不动，可另开终端执行: ./robot.sh check navigation"
    fi
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=navigation \
        use_lidar:=true \
        base_mode:=${base_mode} \
        base_fusion_mode:=${base_fusion_mode} \
        use_base:=true \
        use_rviz:=${use_rviz} \
        map_file:=${map_file} \
        base_port:=${base_port} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
        lidar_params_file:=${lidar_params} \
        nav2_bt_xml_file:=${nav2_bt_xml_file} \
        rviz_config:=${rviz_config} \
        nav2_start:=$( [ "$localization_only" = true ] && printf 'false' || printf 'true' )
}

run_sensor() {
    local sensor="${1:-}"
    local lidar_params=""
    local lidar_port_override=""
    local lidar_port=""

    if [ -z "$sensor" ]; then
        log_error "✗ 缺少传感器类型: lidar 或 camera"
        exit 1
    fi
    shift || true

    ensure_workspace_built
    setup_ros_env

    case "$sensor" in
        lidar)
            print_header "激光雷达传感器启动"
            while [ $# -gt 0 ]; do
                case "$1" in
                    --lidar-port|--port)
                        if [ $# -lt 2 ]; then
                            log_error "✗ --lidar-port 需要一个串口路径"
                            exit 1
                        fi
                        lidar_port_override="$2"
                        shift 2
                        ;;
                    -h|--help)
                        echo "用法: ./robot.sh sensor lidar [--lidar-port PATH]"
                        exit 0
                        ;;
                    *)
                        log_error "✗ 未知参数: $1"
                        echo "用法: ./robot.sh sensor lidar [--lidar-port PATH]"
                        exit 1
                        ;;
                esac
            done
            lidar_params="$(resolve_default_lidar_params)"
            lidar_params="$(prepare_lidar_params "$lidar_params")"
            lidar_params="$(prepare_lidar_params_with_runtime_overrides "$lidar_params" "" "" "$lidar_port_override")"
            lidar_params="$(resolve_lidar_params_device "$lidar_params" "$lidar_port_override")"
            lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
            ensure_lidar_port_access "$lidar_port"
            log_info "雷达串口: ${lidar_port}"
            log_info "雷达参数: ${lidar_params}"
            ros2 launch robot_sensing lidar.launch.py \
                use_lidar:=true \
                lidar_params_file:=${lidar_params}
            ;;
        camera)
            log_error "✗ 摄像头传感器已移除（不再使用 Astra Pro）"
            log_error "  如需添加新摄像头，请创建对应的 launch 文件和配置"
            exit 1
            ;;
        *)
            log_error "✗ 未知传感器类型: $sensor"
            exit 1
            ;;
    esac
}

run_base() {
    local base_port="${BASE_PORT:-$DEFAULT_BASE_PORT}"

    while [ $# -gt 0 ]; do
        case "$1" in
            --fake-base|--real-base)
                shift
                ;;
            --base-port|--port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --port 需要一个串口路径"
                    exit 1
                fi
                base_port="$2"
                shift 2
                ;;
            -h|--help)
                echo "用法: ./robot.sh base [--port /dev/serial0]"
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $1"
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "底盘桥接启动"
    if [ "${ROBOT_BASE_MODE:-real}" = real ]; then
        base_port="$(resolve_base_port "$base_port")"
    fi
    log_info "底盘串口: ${base_port}"
    ros2 launch robot_bringup base_only.launch.py \
        effective_params_file:="${ROBOT_EFFECTIVE_PARAMS}" \
        base_mode:="${ROBOT_BASE_MODE:-real}"

}

run_system() {
    local base_port="${BASE_PORT:-$DEFAULT_BASE_PORT}"
    local use_rviz=false
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local base_port_explicit=false
    local rviz_config="$DEFAULT_SYSTEM_RVIZ"

    while [ $# -gt 0 ]; do
        case "$1" in
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    exit 1
                fi
                base_port="$2"
                base_port_explicit=true
                shift 2
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --rviz)
                use_rviz=true
                shift
                ;;
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
                ;;
            -h|--help)
                echo "用法: ./robot.sh system [--base-port PORT] [--skip-lidar-check] [--rviz|--no-rviz]"
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $1"
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "完整系统启动"
    if [ "$use_rviz" = true ]; then
        require_existing_file "$rviz_config" "RViz 配置文件"
    fi

    lidar_params="$(resolve_default_lidar_params)"
    lidar_params="$(prepare_lidar_params "$lidar_params")"
    lidar_params="$(resolve_lidar_params_device "$lidar_params" "")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    ensure_lidar_port_access "$lidar_port"
    export ROBOT_LIDAR_PORT_HINT="$lidar_port"
    base_port="$(resolve_base_port_avoiding_lidar "完整系统" "$base_port" "$lidar_port" "$base_port_explicit")"
    export ROBOT_BASE_PORT_HINT="$base_port"
    if [ "$skip_lidar_check" = false ] && [ -f "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        bash "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    log_info "系统组件:"
    echo "  • YDLIDAR X2 激光雷达 (${lidar_port})"
    echo "  • 雷达 yaw 外参(rad): ${lidar_tf_yaw}"
    echo "  • STM32 串口桥接 (${base_port})"
    echo "  • SLAM Toolbox"
    if [ "$use_rviz" = true ]; then
        echo "  • RViz2"
    fi
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=mapping \
        mapping_source:=lidar \
        use_lidar:=true \
        base_mode:=real \
        use_base:=true \
        use_rviz:=${use_rviz} \
        base_port:=${base_port} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
        lidar_params_file:=${lidar_params} \
        rviz_config:=${rviz_config}
}

run_passthrough_script() {
    local script_path="$1"
    shift
    exec bash "$script_path" "$@"
}

run_check() {
    local target="${1:-}"
    shift || true

    case "$target" in
        lidar)
            run_passthrough_script "$LIDAR_HEALTH_SCRIPT" "$@"
            ;;
        chassis)
            run_passthrough_script "$CHECK_CHASSIS_SCRIPT" "$@"
            ;;
        tf)
            run_passthrough_script "$CHECK_TF_SCRIPT" "$@"
            ;;
        mapping)
            run_passthrough_script "$CHECK_MAPPING_SCRIPT" "$@"
            ;;
        navigation)
            run_passthrough_script "$CHECK_NAVIGATION_SCRIPT" "$@"
            ;;
        -h|--help|"")
            check_usage
            exit 0
            ;;
        *)
            log_error "✗ 未知检查目标: ${target}"
            check_usage
            exit 1
            ;;
    esac
}

COMMAND="${1:-help}"
if [ $# -gt 0 ]; then
    shift
fi

case "$COMMAND" in
    mapping)
        run_mapping "$@"
        ;;
    navigation|nav)
        run_navigation "$@"
        ;;
    sensor|sensors)
        run_sensor "$@"
        ;;
    base|bridge)
        run_base "$@"
        ;;
    base-test|test-base)
        run_passthrough_script "$BASE_TEST_SCRIPT" "$@"
        ;;
    full|system)
        run_system "$@"
        ;;
    check)
        run_check "$@"
        ;;
    save-map)
        run_passthrough_script "$SAVE_MAP_SCRIPT" "$@"
        ;;
    check-lidar)
        run_passthrough_script "$LIDAR_HEALTH_SCRIPT" "$@"
        ;;
    doctor|check-system)
        run_passthrough_script "$CHECK_SYSTEM_SCRIPT" "$@"
        ;;
    teleop)
        run_passthrough_script "$KEYBOARD_CONTROL_SCRIPT" "$@"
        ;;
    stop)
        run_passthrough_script "$STOP_ALL_SCRIPT" "$@"
        ;;
    help|-h|--help)
        main_usage
        ;;
    *)
        log_error "✗ 未知命令: ${COMMAND}"
        main_usage
        exit 1
        ;;
esac
