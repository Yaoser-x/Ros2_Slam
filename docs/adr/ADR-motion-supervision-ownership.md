# ADR：运动监督所有权

状态：Accepted；按 Platform API 5 术语修订。

四轮一致性、目标跟踪、非预期运动和 wheel/gyro yaw 差异不是传输事实，不能
由 Bridge 拥有。`robot_supervision` 订阅 Host 候选、`WheelObservation` 与质量
处理后的 IMU，发布 `MotionSupervisionState`；它只给出分级、缩放和释放 Host
候选的建议。下位机仍拥有物理安全；Nav2 Goal 取消由 `robot_navigation_guard` 负责。

证据：监督纯算法测试、Fake fault scenario 和 Topic 所有权测试。
# rc2 terminology and freshness

`robot_supervision` publishes deterministic `motion_consistency_risk` components and
Host degradation advice. The API-5 field `score` is retained for compatibility but is
not a slip probability. Stale gyro is excluded from yaw consistency; stale command is
excluded from target tracking and unexpected-motion classification; incomplete wheel
quality only affects components that remain observable. Threshold changes require a
labelled ROC/FPR/FNR report and independent validation.

## Gyro 字段准入

监督器检查 `sensor_msgs/Imu.angular_velocity_covariance[0]`；负值、非有限 covariance
或非有限 `angular_velocity.z` 都使 gyro 无效。默认只接受不早于 200 ms 的样本，
允许至多 20 ms 的未来时间误差。已接受样本把接收时的样本年龄计入 monotonic age，
后续 ROS 时钟跳变不得延长其有效期。

无效或过期 gyro 不作为零角速度参与 yaw 比较，而是排除 yaw 分量并报告
`REASON_IMU_UNAVAILABLE`。健康轮系继续按 wheel-only 路径监督；缺少 gyro 本身不
构成 CRITICAL。字段准入和 ROS executor 回归是软件证据，阈值可靠性仍需独立实测。
