# Upper protocol v3

本文档是上位机与 STM32 下位机之间的规范性线协议。v0.6.0-rc2 运行时只接受 v3；ROS Platform API 5 的语义升级不改变本线协议字节。`v1.0.0-beta4` 的 upper v2 固件不兼容。

## 帧格式

所有多字节整数和 IEEE-754 浮点数均为 little-endian。

```text
A5 5A | LEN:u8 | CMD:u8 | PAYLOAD:(LEN-1) | CRC8:u8
```

`LEN` 包含 CMD，范围为 1..100。CRC 覆盖 `LEN + CMD + PAYLOAD`，算法为 CRC-8/MAXIM，初值 0，多项式的反射形式为 `0x8C`。接收端必须在坏长度和 CRC 错误后重新同步，不得执行部分帧。

## 命令表

| CMD | 方向 | 名称 | payload 长度 |
| --- | --- | --- | ---: |
| `0x01` | 上→下 | SET_VELOCITY | 23 |
| `0x02` | 上→下 | ESTOP | 2 |
| `0x03` | 上→下 | LINE_CTRL | 2 |
| `0x04` | 上→下 | CLEAR_FAULT | 1 |
| `0x05` | 上→下 | GET_INFO | 1 |
| `0x80` | 下→上 | HELLO | 34 |
| `0x81` | 下→上 | STATUS | 92 |
| `0x82` | 下→上 | DIAGNOSTIC | 28 |
| `0x83` | 下→上 | IMU_STATUS | 99 |

所有 payload 的 offset 0 都是 `protocol_version=3`。版本或长度不匹配时必须拒绝整帧。

## 会话命令和确认

SET_VELOCITY：

| offset | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | protocol_version |
| 1 | f32 | linear_x_mps |
| 5 | f32 | angular_z_radps |
| 9 | u8 | enable |
| 10 | u8 | mode/source |
| 11 | u64 | session_id，必须非 0 |
| 19 | u32 | sequence |

新的上位机串口连接生成新的随机 `session_id`。同一 session 的 sequence 按模 `2^32` 比较，`0 < new-old < 2^31` 才是前进序列。相同 sequence 且相同目标仅作为 keepalive，不重复应用；倒退序列和旧 session 必须拒绝。disable 是有序命令，并清零运动目标。

STATUS 的尾部是命令确认：

| offset | 类型 | 字段 |
| ---: | --- | --- |
| 65 | u32 | status_sequence |
| 69 | u32 | sample_timestamp_ms |
| 73 | u64 | last_received_session_id |
| 81 | u32 | last_received_sequence |
| 85 | u32 | last_applied_sequence |
| 89 | u8 | last_reject_reason |
| 90 | u8 | side_consistency_flags |
| 91 | u8 | command_ack_flags |

ACK flags：`0x01 session_valid`、`0x02 received`、`0x04 applied`、`0x08 duplicate_keepalive`、`0x10 rejected`。reject reason：0 无、1 格式、2 版本、3 非有限值、4 mode、5 旧 session、6 乱序、7 fault、8 source 不允许。

固件必须在 200 ms 内因 UPPER 命令丢失进入安全停止。上位机每 50 ms keepalive，150 ms 内未收到 command ACK 或命令更新时主动 release；250 ms 未收到 STATUS 时 release 并要求新的 disable→enable 边沿。

Bridge 的串口线程保留每一帧 STATUS/ACK 的处理顺序，优先交给控制回调；高频 IMU 观测在尚未消费时保留最新样本，避免旧观测积压延迟安全状态。`serial_rx_coalesced_frames` 单独记录被新样本替代的帧数，与队列溢出丢帧计数区分。连接或 wire session 变化时丢弃待处理的旧 IMU 观测。WheelObservation 和 ImuObservation 的接收时间戳扣除队列驻留时间，使下游能发现延迟数据。控制回调仍受每次最多四个事件、约 4 ms 的批次预算约束；此策略不放宽命令、ACK 或状态超时。

## HELLO 与能力门

GET_INFO payload 仅为版本字节。固件响应 HELLO：

| offset | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | protocol_version |
| 1 | u8 | schema_version |
| 2 | u32 | capabilities |
| 6 | bytes[20] | firmware Git commit 原始 20 字节 |
| 26 | u32 | hardware_revision |
| 30 | u32 | parameter_crc32 |

v0.6.0-rc2 要求 capabilities `0x1f`：session ACK、STATUS 采样时间、IMU 字段质量、左右侧一致性、构建身份。任一缺失时 bridge 保持 fail-closed。

## STATUS 主体

| offset | 类型 | 字段 |
| ---: | --- | --- |
| 0..3 | 4×u8 | version、status_flags、control_source、motor_enabled_mask |
| 4 | u32 | error_flags |
| 8 | u32 | latched_error_flags |
| 12 | u16 | battery_mV |
| 14 | 4×i16 | motor_speed_mmps，顺序 LF/LR/RF/RR |
| 22 | 4×i32 | encoder_count，顺序 LF/LR/RF/RR |
| 38 | 4×u16 | motor_current_mA |
| 46 | 4×i16 | motor_target_mmps |
| 54 | 4×i16 | motor_output_permille |
| 62..64 | 3×u8 | speed_valid_mask、encoder_anomaly_mask、comm_health_flags |
| 65..91 | — | 时间、序列和 ACK，见上节 |

编码器必须是连续的有符号 32 位累计计数；重启或不物理可达的跳变通过 anomaly mask 报告。`status_sequence` 每个真实采样前进一次，重发不得伪造新样本。

## IMU_STATUS 和字段质量

布局保持为：version；3×f32 accel_g；3×f32 corrected gyro_dps；3×f32 Euler；4×f32 quaternion(w,x,y,z)；timestamp_ms、sensor_time、sample_count、quality_flags；7×u32 quality counters；status_flags；i8 temperature。

quality flags 按字段解释：bit0 gyro invalid、bit1 accel invalid、bit2 orientation invalid、bit3 timestamp invalid、bit4 gyro warning、bit5 accel warning、bit6 orientation warning、bit7 timestamp warning、bit8 fatal、bit9 sensor reset。gyro-only 模式不因 quaternion 无效丢弃有效 gyro；warning 数据继续发布但提高协方差。status flags 为 online `0x01`、calibrated `0x02`、error `0x04`、sensor-time-valid `0x08`。

## 下位机验收条件

合入兼容矩阵前，下位机必须提供固定 commit，并通过：黄金帧、重复/乱序/回绕/session 重放、NaN/Inf、fault/ESTOP、200 ms watchdog、STATUS/IMU 时间戳回绕和重启、CRC/短帧/粘包，以及树莓派 UART HIL。只有机器报告为 PASS 后才允许将 `compatibility/firmware.yaml` 的 `release_compatible` 改为 true。
