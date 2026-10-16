# 架空证据与数据分类

`bin/robot base-test --suspended-wheel` 使用正式 base-only 控制、兼容和监督入口。
schema 2 将低速起动能力、命令链及停车分别报告。前后、左右转、12 秒恒速仍必须有正式里程计
位移和双轮方向证据；每一阶段还需至少三条新 Host sequence 及三条新 MCU STATUS 证明目标已送达。

`creep` 是 0.01 m/s 短时能力观察：无明显运动时记 `NOT_DEMONSTRATED`，不据此承诺最低速度。
Host/MCU 命令送达和之后收敛到零仍是必过项；缺 odom、未送达目标、越界或反向反馈仍判失败。
只有此前证明了实际运动，才可能将该阶段 `physical_stop` 判为 PASS；否则明确
`NOT_APPLICABLE_NO_PROVEN_MOTION`。停车同时要求新 Host 零指令、MCU 目标及输出零、轮速 ≤5 mm/s，
连续三帧新鲜且同 session 的编码器静止，沿用 0.8 秒上限。

旧 schema 1 报告及其中 NO_MOTION 失败原样保留。schema 2 通过只代表当前架空软件链及所列阶段，
地面最低起动/稳定速度仍按双向分档、5 秒保持和三次重复测量；架空差速不能用作真实转角标定。

RC2 rosbag catalog 的顶层 `required_topics` 保留为完整采集集合，以兼容已有记录工具。
各 dataset 的 `topic_profile` 决定本阶段必须存在且非空的事实流；`event_topics` 继续采集，但无事件
或该能力未启用时允许为空或缺失。例如 wheel-only 静止不要求导航速度消息和 IMU 融合输出，建图
必须有 scan，导航还必须有 nav 命令与 guard。不能发虚构消息补齐空话题，也不能将基础事实流改为空。

`bag_08_navigation_validation` 表示一次完整 campaign 的数据，至少 600 秒；其中至少 50 次正常目标
试验由导航评估器检查十目标各五次，不能解释为必须记录 50 个十分钟 bag。故障、取消及替换试验
单独采集，不混入正常成功率。SLAM、Nav2 和 bag 评估器默认均使用 v0.6.0-rc2 契约；配置、地图、
标定、原始测量与源码身份仍须随报告保存。单独的统计评估通过不构成完整实机验收通过。
