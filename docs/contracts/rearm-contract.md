# Rearm contract

fault、ESTOP、timeout、断线、固件拒绝、严重 supervision 或 reboot 都会关闭 Host 运动。故障前或故障期间的 disable 不能作为恢复证据。

Bridge 等待新无故障 STATUS，发送新 sequence 的 disable，并只接受同一 wire session、received/applied sequence 精确匹配、带 `SESSION_VALID|RECEIVED|APPLIED`、无 rejected 位且 reject reason 为零的 ACK。确认前只以相同 sequence 重发 disable。

`robot_control` 独立清空候选、等待完整 source quiet window、等待 `WIRE_REARM_READY`，再要求新的操作者/任务回调并生成新 `command_epoch`。Navigation Guard 必须收到旧后端 Goal 的终止结果，并拒绝旧 UUID，才允许新 Goal 的 Twist 成为 Host 意图。

Host disable 使用 volatile DDS，首帧可能早于 reader discovery。mux 在当前会话尚未
wire ready 时按 `rearm_disable_retry_sec` 重发，默认 50 ms；沿用同一禁用
`command_epoch` 并前进 Host sequence。发布成功不替代 ACK，retry 不缩短 quiet
window，也不产生 enable；wire ready 后仍等待新的操作者输入。Bridge 对同一 wire
disable sequence 的重发不得延长首次 ACK deadline。

Bridge、mux 和运动监督的进程退出由 Python SIGINT/SIGTERM 中断执行循环，再按顺序
清理节点及 ROS 上下文，避免 Humble 在消息转换期间异步销毁上下文。Bridge 在节点
销毁前完成有界串口关闭；主机写入完成与固件 APPLIED 证据仍分别记录。其他运行期
RuntimeError 继续抛出，不通过异常文本判断来忽略业务错误。

## 导航取消与许可时效

Action 等待使用原生 rclpy Future，steady timer 驱动状态检查和取消，不要求 asyncio
事件环或额外执行线程。撤权立即停止导航候选准入；mux 移除缓存 nav 候选，存在新鲜
人工候选时继续仲裁，否则发布 Host release。

公开 Goal 失败、取消请求被接受或后端 Future 异常，都不等于后端 Goal 已终止。
Guard 在终止证据到达前保留旧 Goal 所有权并拒绝新 Goal。每次最多保留一个未完成
cancel Future；响应完成后才按间隔重试。响应永久失联时保持关闭，不自动重发新 Goal。

Guard 分别检查 link、firmware、supervision、compatibility 和 Host 状态的单调时钟
lease，并校验会话一致性；默认 lease 为 250 ms。mux 也对 Guard 状态执行 250 ms
lease，Guard 消失时不允许后端持续发布的 Twist 保留导航权限。
`formal_odometry` 的 compatibility lease 默认同为 250 ms；恢复后需要当前会话的新
wheel evidence 才能重新发布正式 odom/TF。这些时限不代表实测物理停车时间。
# Bridge command decision boundary

Mux starts a new disabled Host epoch whenever a nonzero wire session changes,
including the first connection. Retries within that wire session retain the disabled
epoch. This prevents a disable observed before serial connect from being retried
forever after the Bridge retires its old Host generation. A new wire session also
clears candidates and requires the quiet window and fresh Host intent, even when its
first state message already reports wire readiness.

The Bridge ROS adapter never maps an arbitrary callback error directly to a vehicle
release. `BridgeCore` returns an explicit immutable decision. Idempotent duplicates,
retired sessions and out-of-order sequences are diagnosed without mutating the current
target. A conflicting duplicate or invalid value in the current active epoch clears the
target and requires rearm. STATUS/ACK timeout, firmware rejection, ESTOP, fault and
transport loss remain global release events.
