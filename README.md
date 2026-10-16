# ROS2 机器人上位机工程

本仓库是 Raspberry Pi 4B 上位机工作区，目标是在 Ubuntu 22.04 64-bit + ROS2 Humble 下收口 YDLIDAR X2、STM32 两轮差速底盘、SLAM Toolbox 和 Nav2 主链路。

## 能力矩阵

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| 雷达建图 | 已支持 | `YDLIDAR X2 -> /scan -> slam_toolbox` |
| 摄像头建图 | 后续扩展 | 摄像头默认不进入当前激光 SLAM 主链路 |
| 真实底盘导航 | 阻塞 | 固件候选 `ba670e6` 已冻结 upper v3；实读参数 CRC 已绑定，完整 UART HIL 和实车验收尚未完成 |
| 虚拟底盘联调 | 已支持 | `base_mode:=fake` |
| 串口设备隔离 | 已支持 | 雷达可检测；底盘默认固定 `/dev/serial0`，`base_port=auto` 已废弃 |
| 底盘 EKF 融合 | 实验开关 | 默认 wheel-only；正式 `/odom`/TF 始终由 `formal_odometry` 发布 |
| 激光运行时覆写 | 已支持 | `mapping` / `navigation` 支持 `--lidar-reversion`、`--lidar-inverted`、`--lidar-yaw-*` |

## 统一入口

- 用户和运维入口：`/home/robot/ros2_ws/bin/robot`
- 工程编排入口：`/home/robot/ros2_ws/src/robot_bringup/launch/system.launch.py`

`launch_scripts/robot.sh` 在 v0.5.x 只做兼容转发。`bin/robot` 会先编译、校验并记录 effective config；`system.launch.py` 只用于开发、调试和二次编排。

## 快速路径

```bash
cd /home/robot/ros2_ws
source /opt/ros/humble/setup.bash
./scripts/bootstrap/fetch_vendor.sh
sudo ./scripts/bootstrap/install_ydlidar_sdk.sh
./scripts/build/build_ros_ws.sh
source /home/robot/ros2_ws/install/setup.bash
```

```bash
cd /home/robot/ros2_ws
./bin/robot mapping lidar --manual --fake-base --rviz
./bin/robot save-map my_map
./bin/robot navigation /home/robot/ros2_maps/my_map.yaml --fake-base --rviz
```

上述命令只验证上位机拓扑。真实底盘只能在对应门禁通过后，
把 `--fake-base` 替换为 `--real-base --base-port /dev/serial0`。
标准导航流程只有一条：

1. 运行 `./bin/robot navigation /home/robot/ros2_maps/my_map.yaml --fake-base --rviz`；
   门禁通过后才使用 `--real-base`
2. 在 RViz 中先执行 `2D Pose Estimate`
3. 等激光与地图基本重合后，再用 `2D Goal Pose` 下发目标

两阶段导航 `--localization-only` / `--nav2-only` 只保留为定位未就绪或现场异常时的回退流程，不再作为默认主流程。

## 当前默认约定

- 底盘默认串口：`/dev/serial0`；可用编译配置或 `--base-port` 显式覆盖
- 雷达正式串口：`/dev/ydlidar`
- 雷达参数基线：`src/robot_config/config/components/lidar.yaml`；运行时只使用配置编译产物
- 默认激光手性修正：`inverted: true`
- 雷达 yaw、轮径、有效轮距、单轮最大周速均为 provisional，待实测；编码器/电机方向归固件参数管理
- 默认导航行为树：`src/robot_bringup/behavior_trees/navigate_to_pose_recovery.xml`
- 默认底盘命令时序：mux 250 ms、bridge 150 ms、keepalive 50 ms、固件 watchdog 200 ms
- 上位机配置事实源：`src/robot_config/config`；固件参数由下位机管理，跨机以参数 CRC/标定包校验
- Platform API 5 / Upper Protocol v3；固定下位机候选 `ba670e6`（本地联调候选，未打 tag），发布兼容仍为 false
- 当前证据状态：软件单测、PTY 和黄金向量可重放；rc2 机器报告仍为 `NOT_RUN`，真实 UART HIL、参数 CRC、车辆、标定、SLAM/Nav2 和长稳均为 pending

## 仓库分层

```text
ros2_ws
├── deps/                  # 第三方依赖锁定清单与补丁
├── deploy/                # 树莓派部署模板
├── docs/                  # 标准阶段文档与旧中文文档
├── launch_scripts/        # 统一运维入口与诊断脚本
├── scripts/               # bootstrap / build / verify 脚本
├── verification/          # 机器实验、HIL、报告、schema 与发布门
├── tools/                 # 现场诊断和标定工具
├── compatibility/         # 固件兼容性声明
├── PLATFORM_API_VERSION
├── src/
│   ├── robot_description
│   ├── robot_config
│   ├── robot_chassis_model
│   ├── robot_chassis_ops
│   ├── robot_sensing
│   ├── stm32_robot_bridge
│   ├── robot_state_estimation
│   ├── robot_control
│   ├── robot_supervision
│   ├── robot_navigation_guard
│   ├── robot_verification
│   ├── robot_bringup
│   └── vendor/            # vcs 自动拉取，Git 忽略
├── README.md
└── SYSTEM_OVERVIEW.md
```

运行数据不进仓库，树莓派上使用：

```text
/home/robot/robot_data/{maps,bags,reports,exports}
/home/robot/.config/slamrobot/robot.yaml
```

## 文档导航

- [文档总览](./docs/README.md)
- [项目范围](./docs/00-项目范围.md)
- [快速开始](./docs/01-快速开始.md)
- [系统架构](./docs/02-系统架构.md)
- [硬件接线与设备识别](./docs/03-硬件接线与设备识别.md)
- [Upper protocol v3](./docs/interfaces/upper-protocol-v3.md)
- [Platform API 5](./docs/interfaces/platform-api-5.md)
- [建图指南](./docs/04-建图指南.md)
- [建图测试流程](./docs/05-建图测试流程.md)
- [导航拆解调试](./docs/05b-导航拆解调试.md)
- [底盘与串口桥接](./docs/06-底盘与串口桥接.md)
- [运维与排障](./docs/07-运维与排障.md)
- [开发与二次复用](./docs/08-开发与二次复用.md)
- [迁移与兼容说明](./docs/09-迁移与兼容说明.md)
- [树莓派4B雷达迁移问题清单](./docs/10-树莓派4B雷达迁移问题清单.md)

## 设计原则

- 运维层只保留一个正式 CLI：`bin/robot`
- `start_*.sh` 继续存在，但只做兼容转发，不再维护独立逻辑
- 技术编排只保留一个核心入口：`src/robot_bringup/launch/system.launch.py`
- 第三方源码不提交；按 `deps/*.repos` 锁定 commit 后拉到忽略目录
- 对外平台接口固定为 `/scan`、`/odom`、`/tf`、`/tf_static` 和 `/cmd_vel/*` 候选输入
- 文档统一按“单阶段默认、两阶段回退”的导航语义描述
- 代码是真值来源，文档随代码更新
- 实车命令只描述操作路径，不代表 Gate B/C/D 已通过；兼容门关闭时禁止绕过
