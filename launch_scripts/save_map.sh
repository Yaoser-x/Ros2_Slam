#!/bin/bash
# ============================================
# 保存SLAM地图脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"
PAD_MAP_SCRIPT="$SCRIPT_DIR/pad_nav_map.py"
MAP_PADDING_METERS="${MAP_PADDING_METERS:-1.0}"
MAP_PADDING_VALUE="${MAP_PADDING_VALUE:-205}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}    保存SLAM地图${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source $ROS_WS/install/setup.bash

# 获取地图名称
MAP_NAME=${1:-robot_map_$(date +%Y%m%d_%H%M%S)}
MAP_DIR="$HOME/ros2_maps"
MAP_BASENAME="$MAP_DIR/$MAP_NAME"
LATEST_BASENAME="$MAP_DIR/latest"

# 创建地图保存目录
mkdir -p "$MAP_DIR"

echo -e "${YELLOW}地图信息:${NC}"
echo "  名称: $MAP_NAME"
echo "  保存路径: $MAP_BASENAME"
echo ""

# 检查 /map 是否存在
if ! ros2 topic list | grep -q '^/map$'; then
    echo -e "${RED}✗ 未检测到 /map 话题${NC}"
    echo -e "${YELLOW}请先运行建图系统:${NC}"
    echo "  ./robot.sh mapping camera"
    echo "  或"
    echo "  ./robot.sh mapping lidar --real-base"
    exit 1
fi

# 在保存前确认 /map 确实有消息可读，避免 map_saver 直接超时
echo -e "${YELLOW}检查 /map 是否有可用数据(最长等待15秒)...${NC}"
if ! timeout 15s ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable >/tmp/map_probe.txt 2>/tmp/map_probe.err; then
    echo -e "${RED}✗ /map 在15秒内没有可读消息${NC}"
    echo -e "${YELLOW}请保持建图进程运行，并推动机器人移动后重试。${NC}"
    exit 1
fi

echo -e "${GREEN}正在保存地图(/map -> yaml/pgm)...${NC}"
SAVE_RC=1
for TRY in 1 2 3; do
    echo "  尝试 $TRY/3 ..."
    ros2 run nav2_map_server map_saver_cli \
        -t /map \
        -f "$MAP_BASENAME" \
        --free 0.196 --occ 0.65 \
        --ros-args \
        -p save_map_timeout:=15.0 \
        -p map_subscribe_transient_local:=true
    SAVE_RC=$?
    if [ $SAVE_RC -eq 0 ]; then
        break
    fi
    sleep 1
done

if [ $SAVE_RC -ne 0 ]; then
    echo -e "${RED}✗ map_saver_cli 执行失败 (exit code: $SAVE_RC)${NC}"
    echo -e "${YELLOW}请确认建图进程仍在运行，且机器人已移动一小段距离让SLAM先生成地图。${NC}"
    exit 1
fi

# map_saver_cli 返回后短暂等待文件刷新
for _ in 1 2 3 4 5; do
    if [ -f "${MAP_BASENAME}.yaml" ] && [ -f "${MAP_BASENAME}.pgm" ]; then
        break
    fi
    sleep 0.3
done

if [ -f "${MAP_BASENAME}.yaml" ] && [ -f "${MAP_BASENAME}.pgm" ]; then
    if [ ! -s "${MAP_BASENAME}.yaml" ] || [ ! -s "${MAP_BASENAME}.pgm" ]; then
        echo ""
        echo -e "${RED}✗ 地图文件存在但大小为0，判定保存失败${NC}"
        exit 1
    fi

    if [ -f "$PAD_MAP_SCRIPT" ]; then
        echo ""
        echo -e "${YELLOW}为 Nav2 规划添加地图边界缓冲 (${MAP_PADDING_METERS}m)...${NC}"
        if ! python3 "$PAD_MAP_SCRIPT" "${MAP_BASENAME}.yaml" \
            --padding-m "$MAP_PADDING_METERS" \
            --unknown-value "$MAP_PADDING_VALUE"; then
            echo -e "${RED}✗ 地图边界缓冲处理失败${NC}"
            exit 1
        fi
    fi

    if [ "$MAP_NAME" != "latest" ]; then
        rm -f "${LATEST_BASENAME}.yaml" "${LATEST_BASENAME}.pgm"
        ln -s "$(basename "${MAP_BASENAME}.yaml")" "${LATEST_BASENAME}.yaml"
        ln -s "$(basename "${MAP_BASENAME}.pgm")" "${LATEST_BASENAME}.pgm"
    fi

    echo ""
    echo -e "${GREEN}✓ 地图保存成功！${NC}"
    echo ""
    echo -e "${YELLOW}地图文件:${NC}"
    if [ "$MAP_NAME" != "latest" ]; then
        ls -lh "${MAP_BASENAME}.yaml" "${MAP_BASENAME}.pgm" "${LATEST_BASENAME}.yaml" "${LATEST_BASENAME}.pgm"
    else
        ls -lh "${MAP_BASENAME}.yaml" "${MAP_BASENAME}.pgm"
    fi
    echo ""
    echo -e "${YELLOW}加载地图进行导航:${NC}"
    echo "  cd $SCRIPT_DIR"
    echo "  ./robot.sh navigation ${MAP_BASENAME}.yaml --real-base --ekf-base"
    echo ""
    if [ "$MAP_NAME" != "latest" ]; then
        echo -e "${YELLOW}默认地图别名已更新:${NC}"
        echo "  ${LATEST_BASENAME}.yaml -> ${MAP_NAME}.yaml"
        echo "  以后也可以直接运行: ./robot.sh navigation --real-base --ekf-base"
        echo ""
    fi
    echo -e "${YELLOW}RViz 操作:${NC}"
    echo "  1. 用 2D Pose Estimate 在地图上设置机器人当前实际位置和朝向"
    echo "  2. 雷达点云与地图墙体重合后，用 2D Goal Pose 点目标位置"
else
    echo ""
    echo -e "${RED}✗ 未找到生成的地图文件(.yaml/.pgm)${NC}"
    echo -e "${YELLOW}请确认 /map 有数据后重试。${NC}"
    exit 1
fi
