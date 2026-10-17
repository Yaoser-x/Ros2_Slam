# 上下位机运行职责契约

本契约描述 Platform API 5 / Upper Protocol v3 的生产入口。两个仓库独立维护，
协议帧、消息和 Action 类型保持不变；软件、静止、运动和整车证据分别判定。

| 状态或决策 | 唯一所有者 | 其他模块的边界 |
| --- | --- | --- |
| Host 候选优先级、意图寿命、整形和 Host 恢复 | `robot_control` | 唯一发布 `HostMotionCommand`；导航、遥控、测试只能提交候选 |
| UART、wire session、sequence、ACK、线级恢复和传输超时 | `stm32_robot_bridge` | 适配原始事实，不积分里程计、发布 TF、作运动一致性或导航决策 |
| Host 风险分级与降级建议 | `robot_supervision` | 建议缩放／释放 Host，不直接操作下位机电机或撤销其他物理来源 |
| 导航 Goal 准入、替换、取消及终止清理 | `robot_navigation_guard` | 后端目标未终止前不得接纳冲突目标；取消成功不等于物理停车 |
| 编码器积分、IMU 字段质量、时间转换和正式 odom/TF | `robot_state_estimation` | Bridge/Fake Base 只提供同一组原始 Platform API；正式出口由 `formal_odometry` 独占 |
| 物理来源槽、租约、rearm 和最终来源仲裁 | 下位机 `CommandManagement` | Communication 只验证协议身份并提交 intent；Host 不是全车最高物理权限 |
| 安全许可、能力和故障决策 | 下位机 `SafetyManagement` | 消费 App 组合的事实；不读取其他 Service 的隐藏状态或拥有协议 session |
| 闭环、方向阶段与正常运动输出决策 | 下位机 `MotionControl` | BSP 执行硬件操作；硬件 Break、复位和故障抑制只能独立撤权，不能生成运动意图 |
| 设备采样与硬件事实 | 下位机 BSP | 不拥有模式、仲裁、标定流程或运行参数策略 |
| 跨能力周期、维护锁和保存工作流 | 下位机 App | 按公开 DTO 主动收集事实、执行决策，不复制 Service 领域状态 |
| PS2／ESP 控制意图 | 下位机 Teleoperation／ESP 输入端 | 独立于上位机运行，但仍经过 Command、安全许可和 Motion 输出链 |
| 下位机参数写入及 Flash 模型 | 下位机 `ParameterManagement` | 上位机保留预期身份、CRC 和估计所需的已标定副本，不直接写 Flash 或改 CRC 放行 |

## 同名概念的边界

- Host release 只撤回 Host 候选；wire disable ACK、Host 恢复资格和下位机来源 rearm
  是不同事实。物理停车须由新鲜 STATUS 的目标、输出、轮速和连续编码器观测共同证明。
- Host 限速／jerk 是意图整形；下位机速度斜坡、物理限幅和闭环是执行约束，二者各自
  生效。不得用上位机参数绕过下位机限制。
- IMU 汇总 ERROR 不替代字段 invalid 位。时间无效可回退接收时间，仍按 sample count
  保序；gyro 无效不可作为零角速度参与监督。默认 wheel-only 不等于忽略 IMU 质量。
- `stm32_robot_bridge.imu_converter` 是历史辅助模块，当前 v3 生产入口不得导入。
  保留其兼容文件不赋予 Bridge 状态估计职责；新增估计逻辑只能进入估计包。

## 几何与参数同步

1. 使用外部距离／角度基准得到测量结果，记录方法、单位、不确定性和设备身份。
2. 经下位机公开参数入口校验、应用并按需要保存；读取生效值和参数 CRC，不以发送成功代替生效。
3. 把确认后的轮半径、有效轮距和计数约定同步到上位机标定配置，绑定实际 firmware identity、
   hardware revision 和 CRC，再编译有效配置及 hash。不得编辑编译产物或修改期望值掩盖不一致。
4. 使用轮观测、wheel-only 里程计及制动试验验证，保留变更前后配对清单。每次新烧录或参数
   保存均按其独立授权和备份流程执行；未完成实测时继续标为 provisional。

当前数值为轮半径 35 mm、轮距 178 mm、2464 count/rev。它们是当前生效配置，
不是已经完成落地标定的结论。下位机内部分层以其 `docs/architecture.md` 为准。
