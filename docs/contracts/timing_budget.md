# 时序预算

| 项目 | 当前配置 | 时钟/语义 |
| --- | ---: | --- |
| Host 发布周期 | 50 ms | monotonic timer |
| DDS deadline | 100 ms | QoS 诊断 |
| DDS lifespan | 120 ms | 队列准入 |
| Bridge command/ACK timeout | 150 ms | monotonic fail-closed |
| Firmware Host watchdog | 200 ms | 固件独立安全边界 |
| STATUS timeout | 250 ms | monotonic fail-closed |
| mux 来源 lease | 250 ms | monotonic |
| Guard 输入状态 / mux Guard 状态 lease | 250 ms | monotonic；失联撤销导航权限 |
| formal odometry compatibility lease | 250 ms | monotonic；失联关闭正式 odom/TF |
| gyro 最大样本年龄 | 200 ms | ROS stamp 准入后转换为 monotonic age |
| chassis operation deadline | 1 s | monotonic；包含队列响应与事实确认 |
| backend Goal 响应 deadline | 1 s | 公开 Goal 失败后仍保留未终止的后端所有权 |
| backend cancel 重试间隔 | 200 ms | 前一个 cancel Future 完成后才允许重试 |
| command header 最大年龄 | 100 ms | ROS stamp 准入检查 |
| wire keepalive | 50 ms | 相同 target 可复用 sequence |

跨参数不变量由配置编译器执行：`publish < deadline <= lifespan < mux lease`、
`keepalive < bridge timeout < firmware timeout`、`ack <= bridge timeout`。

Host/permission lease、supervision age、rearm quiet window、limiter `dt`、Bridge watchdog、
STATUS 和 ACK timeout 使用 monotonic time。ROS time 用于 header、Bag、观测映射和
命令入队年龄，`/clock` 跳变不得延长已接纳命令的安全 lease。每层停车时间必须在
UART HIL 分别测量，不能仅由配置值推断整车停车距离。

`ChassisLinkState.command_ack_age_ms` 表示本 wire session 最后一次合法、向前推进的
APPLIED ACK 的 monotonic 接收年龄；尚无该证据、跨 session 或断链时为
`AGE_INVALID`，已确认的 sequence `0` 有效。新目标和重复 ACK 不刷新此年龄。
150 ms ACK 预算用于等待未确认命令：首个未确认命令开始计时，forward APPLIED
可推进等待，追上当前目标即结束等待；恒速已确认的同序号 keepalive 不重新开窗。
因此 ACK 年龄本身可以超过 150 ms，不能单独用它推断停车或驱动许可。
