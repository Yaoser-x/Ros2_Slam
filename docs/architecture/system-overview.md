# 当前系统架构

## 职权边界

Platform API 5 把 Host 意图、传输事实、物理控制和状态估计分开：

- `robot_control`：候选 `Twist` 的校验、优先级、lease、速度/加速度/jerk 限制和 Host rearm。
- `robot_supervision`：运动一致性风险、Host 降级倍率和 Host release 建议。
- `stm32_robot_bridge`：串口、framing、Upper v3 codec、HELLO、wire session、ACK、超时和状态适配。
- 固件：Host/PS2/ESP/Line/Debug 最终仲裁、物理运动许可、闭环控制和电机输出。
- `robot_state_estimation`：累计编码器里程计、IMU 字段质量、MCU 到 ROS 时间映射、可选融合、正式 odom 和动态 TF。
- `robot_sensing`：YDLidar 设备接入、scan 规范化和设备诊断。
- `robot_config`：手写配置、严格校验、编译产物和配置身份。
- `robot_bringup`：节点编排，不持有算法状态。
- `robot_verification`：Fake Base、PTY、Bag、HIL、标定和发布证据。

Bridge 不计算 odometry，不发布 TF，不执行 Host 子来源仲裁，也不作 Nav2 决策。
上位机 release 只撤回 Host 候选；整车是否物理停止必须由新鲜固件 STATUS 证明。

## 数据与控制链

```text
teleop/test/nav/research Twist
          |
          v
robot_control -- HostMotionCommand + HostControlState
          |
          +------------------------------+
          v                              v
stm32_robot_bridge                  Fake Base
          |                              |
          +---- common Platform API -----+
                     |
      WheelObservation / ImuObservation
      ChassisLinkState / FirmwareControlState / FirmwareInfo
                     |
      +--------------+------------------+
      v                                 v
robot_state_estimation            robot_supervision
  wheel/odom, imu/data               MotionSupervisionState
      |                                 |
      +-> optional internal EKF          +-> robot_control
      |
      v
formal_odometry -> /odom + odom->base_link
```

Fake Base 和真实 Bridge 是互斥 provider。两者发布相同 Platform API 状态，但都不得
发布 odometry 或 TF。

## 正式里程计与融合

默认 `fusion_mode: wheel`：`wheel_odometry` 生成 `wheel/odom`，
`formal_odometry` 在兼容门和当前 session 新鲜观测允许时转发为 `/odom` 并发布 TF。

显式 `fusion_mode: wheel_imu`：`robot_localization/ekf_node` 只发布内部
`odometry/filtered_internal`，仍由 `formal_odometry` 完成正式出口和 TF。EKF 目前是
实验路径；独立 Bag 对照、IMU 标定和降级证据通过前不得改成默认。

## 兼容门与发布状态

`platform_compatibility` 同时检查当前 wire session、HELLO 身份、协议/schema、
capability、硬件 revision、参数 CRC、轮布局和新鲜观测。真实平台任一事实缺失时，
正式 odom 和导航均 fail-closed；Fake Base 通过显式 `simulated` 身份进入 simulation
状态。

兼容配置中的候选固件仍为 `bc472cc874e930aaed6eb8e7de73b41a2563dd85`，tested commit
与正式配对尚未锁定。2026-10-15 的开发固件串口实测已读到 hardware revision
`0x00020000` 和参数 CRC `0x0D663E2A`；这些观测不自动修改正式兼容配置。
当前开发构建的 HELLO 身份仍为零，BUILD_IDENTITY capability 缺失，兼容门应保持关闭。
`release_compatible` 仍为 `false`，标定和落地车辆验收尚未完成。

真实 UART 阶段补充了两个此前源码／fake-base 回归未覆盖的问题：完整控制帧被 DMA
覆盖保护误判丢弃，以及 Bridge 安装后缺少 Python 路径环境 hook。下位机接收契约见
`lower/docs/repair-business-contracts.md`；上位机使用 `ament_python_install_package`
安装模块，回归须在清除源码 PYTHONPATH 后重新构建并 source 安装产物，验证实际导入。
Bridge 退出时不再访问可能已失效的 ROS 发布／日志上下文，释放过程抛出异常也必须关闭串口。
架空低速、10.5 秒同值命令续租、GET_INFO 幂等和串口停止发送后停车已有开发级证据；
它们不替代正式 ROS 运动许可、落地制动距离、SLAM/Nav2 和长稳验收。

## 后续实施顺序

| 阶段 | 当前边界与进入条件 |
| --- | --- |
| 1：导航可靠性 | 已实现本轮限速、Action、会话证据、许可时效、配置传播和 base-only 修复；ROS executor/运行图结果以实际报告为准。仍需锁定干净版本身份、实测参数 CRC、接线、真实 UART、标定、车辆及 SLAM/Nav2 验收，才能关闭本阶段。 |
| 2：平台化 | 保留现有业务包和上下位机独立仓库，按经过验证的需求增加 provider、能力和配置边界。每个新平台需要独立硬件身份、标定及 UART/车辆证据；Fake 不替代实机资格。 |
| 3：教学入口 | 在已验收平台上整理 fake/base-only 场景、状态观察和实拍操作。先交付一个可复现单元，再扩展课程；实机步骤保留兼容门、操作员就绪和安全准备要求。 |

Upper v4 若增加请求关联的操作结果，应另行定义请求身份、ACK/结果语义和跨版本
兼容，并通过双端黄金向量、主机测试和真实 UART 验证。当前修复仍使用 v3，未将
无关联 STATUS 包装成请求执行回执。

当前明确拒绝非空 namespace。开放多机器人前，需要统一参数 selector、相对话题、
Action 与 TF 的隔离规则，并在两个独立 fake 图及对应实机上验证无串线。
第二、三阶段尚未实施完成；本轮软件回归不能替代上述物理门禁。
