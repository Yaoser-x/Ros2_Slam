#!/bin/bash
# ============================================
# 雷达健康检查脚本（用于建图前验收）
# 重点检查：
# 1) 串口是否可用
# 2) /scan 发布频率是否正常
# 3) 驱动日志是否出现 Check Sum / Failed to get scan
# ============================================

set -o pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/robot/ros2_ws"
PARAM_FILE="$ROS_WS/src/robot_sensing/config/ydlidar_x2.yaml"
LIDAR_PORT_OVERRIDE=""
NODE_EXE="$ROS_WS/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DETECT_LIDAR_PORT_SCRIPT="$SCRIPT_DIR/detect_lidar_port.sh"
MIN_SCAN_HZ=6.0
CHECK_SECONDS=6
MAX_STARTUP_CHECKSUM_ERRORS=2
MAX_RANGE_SIZE_SPAN=8
MAX_RANGE_SIZE_REL_SPAN=0.03

LOG_PREFIX="/tmp/lidar_health_$(date +%Y%m%d_%H%M%S)_$$"
DRIVER_LOG="${LOG_PREFIX}_driver.log"
HZ_LOG="${LOG_PREFIX}_scan_hz.log"

NODE_PID=""
TMP_PARAM_FILE=""

usage() {
    cat <<'EOF'
用法: check_lidar_health.sh [参数文件] [--lidar-port PATH]

默认参数文件: /home/robot/ros2_ws/src/robot_sensing/config/ydlidar_x2.yaml
默认雷达设备: 参数文件中的 port，正式部署建议使用 /dev/ydlidar
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar-port|--port)
            if [ $# -lt 2 ]; then
                echo -e "${RED}✗ --lidar-port 需要一个串口路径${NC}"
                usage
                exit 1
            fi
            LIDAR_PORT_OVERRIDE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            PARAM_FILE="$1"
            shift
            ;;
    esac
done

cleanup() {
    if [ -n "$NODE_PID" ] && kill -0 "$NODE_PID" 2>/dev/null; then
        kill "$NODE_PID" >/dev/null 2>&1 || true
        wait "$NODE_PID" 2>/dev/null || true
    fi
    if [ -n "$TMP_PARAM_FILE" ] && [ -f "$TMP_PARAM_FILE" ]; then
        rm -f "$TMP_PARAM_FILE"
    fi
}
trap cleanup EXIT INT TERM

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    雷达健康检查${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

if [ ! -f "$PARAM_FILE" ]; then
    echo -e "${RED}✗ 参数文件不存在: $PARAM_FILE${NC}"
    exit 1
fi

CONFIG_PORT="$(awk '$1 == "port:" {print $2; exit}' "$PARAM_FILE")"
LIDAR_PORT="${LIDAR_PORT_OVERRIDE:-$CONFIG_PORT}"
if [ -z "$LIDAR_PORT_OVERRIDE" ] && [ -x "$DETECT_LIDAR_PORT_SCRIPT" ]; then
    DETECTED_PORT="$($DETECT_LIDAR_PORT_SCRIPT "$CONFIG_PORT" 2>/dev/null || true)"
    if [ -n "$DETECTED_PORT" ]; then
        LIDAR_PORT="$DETECTED_PORT"
    fi
fi

if [ -z "$LIDAR_PORT" ] || [ ! -e "$LIDAR_PORT" ]; then
    echo -e "${RED}✗ 未找到雷达设备${NC}"
    exit 1
fi

if [ ! -r "$LIDAR_PORT" ] || [ ! -w "$LIDAR_PORT" ]; then
    echo -e "${RED}✗ 当前用户没有雷达串口读写权限: $LIDAR_PORT${NC}"
    echo -e "${YELLOW}当前用户组: $(id -nG)${NC}"
    echo -e "${YELLOW}设备权限: $(ls -l "$LIDAR_PORT")${NC}"
    echo -e "${YELLOW}请在树莓派本机执行一次:${NC}"
    echo "  sudo usermod -aG dialout robot"
    echo "  sudo chmod 666 $LIDAR_PORT"
    echo -e "${YELLOW}长期建议执行:${NC}"
    echo "  /home/robot/ros2_ws/launch_scripts/setup_lidar_permissions.sh"
    echo -e "${YELLOW}加入 dialout 后需要重新登录，或重启终端/系统。${NC}"
    exit 1
fi

if [ "$LIDAR_PORT" != "$CONFIG_PORT" ]; then
    TMP_PARAM_FILE="$(mktemp /tmp/ydlidar_health_XXXX.yaml)"
    sed -E "s|^([[:space:]]*port:).*|\\1 ${LIDAR_PORT}|" "$PARAM_FILE" > "$TMP_PARAM_FILE"
    PARAM_FILE="$TMP_PARAM_FILE"
fi

if fuser "$LIDAR_PORT" >/dev/null 2>&1; then
    echo -e "${RED}✗ ${LIDAR_PORT} 正被其它进程占用${NC}"
    echo -e "${YELLOW}请先执行: /home/robot/ros2_ws/launch_scripts/stop_all.sh${NC}"
    echo -e "${YELLOW}或手动检查: fuser -v ${LIDAR_PORT}${NC}"
    exit 1
fi

echo -e "${YELLOW}启动 ydlidar 驱动进行采样...${NC}"
echo -e "${YELLOW}参数文件: $PARAM_FILE${NC}"
echo -e "${YELLOW}雷达串口: $LIDAR_PORT${NC}"
if [ -x "$NODE_EXE" ]; then
    "$NODE_EXE" --ros-args --params-file "$PARAM_FILE" >"$DRIVER_LOG" 2>&1 &
else
    ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
        --ros-args --params-file "$PARAM_FILE" \
        >"$DRIVER_LOG" 2>&1 &
fi
NODE_PID=$!

sleep 3
if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo -e "${RED}✗ ydlidar 驱动未正常启动${NC}"
    echo -e "${YELLOW}日志: $DRIVER_LOG${NC}"
    tail -n 40 "$DRIVER_LOG" || true
    exit 1
fi

echo -e "${YELLOW}采样 /scan 频率（${CHECK_SECONDS}s）...${NC}"
python3 - "$CHECK_SECONDS" >"$HZ_LOG" 2>&1 <<'PY'
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

rclpy.init()
node = rclpy.create_node("lidar_health_rate_check")

state = {
    "count": 0,
    "first_t": None,
    "last_t": None,
    "len_hist": {},
    "last_stamp_ns": None,
    "scan_metadata_errors": 0,
    "valid_range_count": 0,
}

def cb(msg):
    t = time.monotonic()
    if state["first_t"] is None:
        state["first_t"] = t
    state["last_t"] = t
    state["count"] += 1
    n = len(msg.ranges)
    state["len_hist"][n] = state["len_hist"].get(n, 0) + 1
    stamp = msg.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    stamp_valid = stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
    if state["last_stamp_ns"] is not None and stamp_ns <= state["last_stamp_ns"]:
        stamp_valid = False
    state["last_stamp_ns"] = stamp_ns
    # X2 scans at multiple Hz. A nonfinite or >1s period is not a usable scan;
    # zero time_increment is allowed when individual sample timing is unknown.
    timing_valid = (
        math.isfinite(msg.scan_time)
        and math.isfinite(msg.time_increment)
        and 0.0 < msg.scan_time <= 1.0
        and 0.0 <= msg.time_increment <= msg.scan_time
    )
    state["scan_metadata_errors"] += int(not stamp_valid or not timing_valid)
    state["valid_range_count"] += sum(
        math.isfinite(value) and msg.range_min <= value <= msg.range_max for value in msg.ranges
    )

sub = node.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
start = time.monotonic()
while time.monotonic() - start < duration:
    rclpy.spin_once(node, timeout_sec=0.2)

count = state["count"]
if count <= 1 or state["first_t"] is None or state["last_t"] is None:
    hz = 0.0
else:
    dt = state["last_t"] - state["first_t"]
    hz = ((count - 1) / dt) if dt > 0 else 0.0

print(f"COUNT={count}")
print(f"HZ={hz:.3f}")
print(f"SCAN_METADATA_ERRORS={state['scan_metadata_errors']}")
print(f"VALID_RANGE_COUNT={state['valid_range_count']}")
print(f"UNIQUE_RANGE_SIZE={len(state['len_hist'])}")
if state["len_hist"]:
    sizes = sorted(state["len_hist"])
    min_size = sizes[0]
    max_size = sizes[-1]
    mean_size = sum(k * v for k, v in state["len_hist"].items()) / max(count, 1)
    rel_span = ((max_size - min_size) / mean_size) if mean_size > 0 else 0.0
    parts = [f"{k}:{v}" for k, v in sorted(state["len_hist"].items())]
    print(f"RANGE_SIZE_STATS={','.join(parts)}")
    print(f"RANGE_SIZE_MIN={min_size}")
    print(f"RANGE_SIZE_MAX={max_size}")
    print(f"RANGE_SIZE_SPAN={max_size - min_size}")
    print(f"RANGE_SIZE_REL_SPAN={rel_span:.6f}")

node.destroy_node()
rclpy.shutdown()
PY

# 采样完成后立即回收驱动，避免后续启动建图时串口冲突。
cleanup
NODE_PID=""

CHECKSUM_COUNT=$(grep -Eic "Check Sum|CheckSum|checksum" "$DRIVER_LOG" 2>/dev/null || true)
if [ -z "$CHECKSUM_COUNT" ]; then
    CHECKSUM_COUNT=0
fi
SCAN_FAIL_COUNT=$(grep -Eic "Failed to get scan" "$DRIVER_LOG" 2>/dev/null || true)
if [ -z "$SCAN_FAIL_COUNT" ]; then
    SCAN_FAIL_COUNT=0
fi
SCAN_COUNT=$(awk -F= '/^COUNT=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
AVG_SCAN_HZ=$(awk -F= '/^HZ=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
SCAN_METADATA_ERRORS=$(awk -F= '/^SCAN_METADATA_ERRORS=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
VALID_RANGE_COUNT=$(awk -F= '/^VALID_RANGE_COUNT=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
UNIQUE_RANGE_SIZE=$(awk -F= '/^UNIQUE_RANGE_SIZE=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
RANGE_SIZE_STATS=$(awk -F= '/^RANGE_SIZE_STATS=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
RANGE_SIZE_MIN=$(awk -F= '/^RANGE_SIZE_MIN=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
RANGE_SIZE_MAX=$(awk -F= '/^RANGE_SIZE_MAX=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
RANGE_SIZE_SPAN=$(awk -F= '/^RANGE_SIZE_SPAN=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')
RANGE_SIZE_REL_SPAN=$(awk -F= '/^RANGE_SIZE_REL_SPAN=/{print $2}' "$HZ_LOG" 2>/dev/null | tail -n 1 | tr -d '[:space:]')

echo ""
echo -e "${BLUE}检查结果:${NC}"
echo "  日志文件: $DRIVER_LOG"
echo "  频率文件: $HZ_LOG"
echo "  CheckSum错误数: $CHECKSUM_COUNT"
echo "  Failed to get scan次数: $SCAN_FAIL_COUNT"
echo "  扫描时间字段错误数: ${SCAN_METADATA_ERRORS:-未获取到}"
echo "  量程内有效距离数: ${VALID_RANGE_COUNT:-未获取到}"
if [ -n "${SCAN_COUNT:-}" ]; then
    echo "  /scan消息数: ${SCAN_COUNT}"
fi
if [ -n "${AVG_SCAN_HZ:-}" ]; then
    echo "  /scan平均频率: ${AVG_SCAN_HZ} Hz"
else
    echo "  /scan平均频率: 未获取到"
fi
if [ -n "${UNIQUE_RANGE_SIZE:-}" ]; then
    echo "  /scan点数档位: ${UNIQUE_RANGE_SIZE}"
fi
if [ -n "${RANGE_SIZE_STATS:-}" ]; then
    echo "  点数统计: ${RANGE_SIZE_STATS}"
fi
if [ -n "${RANGE_SIZE_SPAN:-}" ]; then
    echo "  点数跨度: ${RANGE_SIZE_SPAN} (${RANGE_SIZE_MIN:-?}..${RANGE_SIZE_MAX:-?})"
fi

if [ "$SCAN_FAIL_COUNT" -gt 0 ]; then
    echo ""
    echo -e "${RED}✗ 未通过: 驱动未能持续获取扫描帧 (Failed to get scan)${NC}"
    echo -e "${YELLOW}建议排查:${NC}"
    echo "  1) 更换 USB 线，避免劣质延长线/HUB"
    echo "  2) 检查供电稳定性（雷达单独供电更稳）"
    echo "  3) 确认雷达型号与参数文件一致（当前默认 X2.yaml）"
    exit 1
fi

if [ -z "${SCAN_METADATA_ERRORS:-}" ] || [ "$SCAN_METADATA_ERRORS" != "0" ]; then
    echo -e "${RED}✗ 未通过: 扫描时间戳不递增、扫描周期异常，或未取得时间字段检查结果${NC}"
    echo -e "${YELLOW}采样日志: $HZ_LOG${NC}"
    exit 1
fi

if [ -z "${VALID_RANGE_COUNT:-}" ] || [ "$VALID_RANGE_COUNT" -eq 0 ]; then
    echo -e "${RED}✗ 未通过: 未取得量程内有效距离${NC}"
    exit 1
fi

if [ -z "${SCAN_COUNT:-}" ] || [ "${SCAN_COUNT:-0}" -lt 2 ]; then
    echo ""
    echo -e "${RED}✗ 未通过: /scan 数据不足（消息数 < 2）${NC}"
    echo -e "${YELLOW}请检查驱动日志: $DRIVER_LOG${NC}"
    echo -e "${YELLOW}频率采样输出: $HZ_LOG${NC}"
    tail -n 40 "$HZ_LOG" 2>/dev/null || true
    exit 1
fi

if [ -z "${AVG_SCAN_HZ:-}" ]; then
    echo ""
    echo -e "${RED}✗ 未通过: 未解析到 /scan 平均频率${NC}"
    echo -e "${YELLOW}频率采样输出: $HZ_LOG${NC}"
    tail -n 40 "$HZ_LOG" 2>/dev/null || true
    exit 1
fi

if ! awk "BEGIN {exit !($AVG_SCAN_HZ >= $MIN_SCAN_HZ)}"; then
    echo ""
    echo -e "${RED}✗ 未通过: /scan 频率过低 (${AVG_SCAN_HZ} Hz < ${MIN_SCAN_HZ} Hz)${NC}"
    echo -e "${YELLOW}请检查 USB 连接、CPU负载和雷达状态${NC}"
    exit 1
fi

if [ -z "${UNIQUE_RANGE_SIZE:-}" ]; then
    echo ""
    echo -e "${RED}✗ 未通过: 未解析到 /scan 点数稳定性结果${NC}"
    echo -e "${YELLOW}频率采样输出: $HZ_LOG${NC}"
    tail -n 40 "$HZ_LOG" 2>/dev/null || true
    exit 1
fi

if [ -z "${RANGE_SIZE_SPAN:-}" ] || [ -z "${RANGE_SIZE_REL_SPAN:-}" ]; then
    echo ""
    echo -e "${RED}✗ 未通过: 未解析到 /scan 点数波动范围${NC}"
    echo -e "${YELLOW}频率采样输出: $HZ_LOG${NC}"
    tail -n 40 "$HZ_LOG" 2>/dev/null || true
    exit 1
fi

if ! awk "BEGIN {exit !(($RANGE_SIZE_SPAN <= $MAX_RANGE_SIZE_SPAN) || ($RANGE_SIZE_REL_SPAN <= $MAX_RANGE_SIZE_REL_SPAN))}"; then
    echo ""
    echo -e "${RED}✗ 未通过: /scan 单帧点数波动过大（跨度=${RANGE_SIZE_SPAN}, 相对跨度=${RANGE_SIZE_REL_SPAN}）${NC}"
    echo -e "${YELLOW}当前统计: ${RANGE_SIZE_STATS}${NC}"
    echo -e "${YELLOW}建议排查雷达转速、USB 串口稳定性和驱动参数；不要为了通过检查而裁剪有效扫描点。${NC}"
    exit 1
fi

if [ "$CHECKSUM_COUNT" -gt "$MAX_STARTUP_CHECKSUM_ERRORS" ]; then
    echo ""
    echo -e "${RED}✗ 未通过: CheckSum 错误过多 (${CHECKSUM_COUNT} > ${MAX_STARTUP_CHECKSUM_ERRORS})${NC}"
    echo -e "${YELLOW}建议排查 USB 连接、供电稳定性和 X2 参数文件；不要在错误持续出现时跳过检查。${NC}"
    exit 1
fi

if [ "$CHECKSUM_COUNT" -gt 0 ]; then
    echo -e "${YELLOW}提示: 启动期出现 ${CHECKSUM_COUNT} 次 CheckSum，同步后 /scan 质量达标，允许继续建图。${NC}"
fi

if [ "${UNIQUE_RANGE_SIZE:-0}" -gt 1 ]; then
    echo -e "${YELLOW}提示: /scan 点数存在轻微正常波动（档位数=${UNIQUE_RANGE_SIZE}, 跨度=${RANGE_SIZE_SPAN}），不会阻止建图。${NC}"
fi

echo ""
echo -e "${GREEN}✓ 通过: 雷达数据质量可用于建图${NC}"
exit 0
