# Verification

`tests/` 保存自动单元、接口、协议、集成和部署契约测试；本目录保存需要数据、硬件或
发布身份的可追溯验证。

```text
verification/
├── configs/
│   ├── benchmarks/     # Bag/风险数据集清单
│   ├── experiments/    # EKF、SLAM、Nav2 实验定义
│   ├── hil/            # UART HIL 场景与阈值
│   └── release/        # 发布所需报告集合
├── runners/            # 可重复执行器
├── reports/            # 机器可读结果；模板初始为 NOT_RUN
└── schemas/            # effective config 与结果格式
```

## 证据顺序

```text
Host unit + protocol golden + PTY
  -> real UART HIL
  -> wheels-off-ground
  -> low-speed vehicle
  -> geometry/IMU/covariance calibration
  -> wheel-only versus EKF
  -> SLAM independent validation
  -> Nav2 10 goals x 5 runs
  -> fault injection + long run
```

每份物理 PASS 报告必须绑定相同的 upper tested commit、firmware compatible commit、
hardware revision、parameter CRC、effective config SHA-256、artifact SHA-256 manifest
和 calibration version，并包含实际测量 metrics。runner 使用严格递增事件时间和原子
报告写入；缺字段、零 CRC、非法 hash 或非递增时间都 fail-closed。

软件门与物理门使用不同证据格式。`ros_ci`、`protocol_cross_test`、`pty_report` 必须全部
提供执行报告；它们记录两仓实际 HEAD、dirty 状态、源文件 SHA-256、执行命令、退出码、
日志 hash 和实际测试数量。dirty 工作树的软件测试可以通过，但发布门仍拒绝将它当成
对应 clean commit 的发布验收。前后源指纹变化、零用例、跳过或失败的测试均不能产生 PASS。

在已经安装项目现有 ROS Humble 与开发依赖的 Linux 上执行，输出放在外部目录或 ignored
build 目录；不会覆盖仓库里的 `NOT_RUN` 模板：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/verify/run_software_evidence.py \
  --firmware-root ../lower --output-dir /tmp/slamrobot-software-evidence
python3 scripts/verify/verify_release.py \
  --software-reports-dir /tmp/slamrobot-software-evidence
```

可通过 `--gates protocol_cross_test pty_report` 单独记录这两门；完整发布依然需要三个软件
报告。CI 的固件路径为 `firmware-contract`，测试使用 `FIRMWARE_CONTRACT_DIR` 指向实际
checkout。上下位机修复必须先通过各自正常提交/候选更新流程形成可测试的 commit pair，
不得把本地 dirty 修复伪写成冻结候选固件已经通过。软件报告的字段不充当 CRC、标定或
车辆证据。

CI 通过同一 runner 的 `--with-coverage` 选项保留原有六个模块的覆盖统计和 term/XML
输出，然后执行原有 `coverage json` 与模块阈值检查；不会重复运行旧的 colcon/pytest
步骤。日志、JUnit 和三个软件报告留在 ignored `build/software-evidence` 中。

UART HIL 的 `result`、`metrics`、`failures`、schema/release 和证据 hash 由 runner 计算，
metadata 禁止覆盖。当前物理停车场景只有 `wheel_speed_below_threshold_at` 事件名，尚未
定义实测速度阈值与接受规则；runner 将它记录为 `NOT_EVALUATED` 并阻断 PASS，不能据此
宣称轮速已通过物理停车验收。

当前 rc2 报告仍为 `NOT_RUN`，不得因本地 pytest、Fake Base 或 PTY 通过而改写为
物理 PASS。发布门运行方式：

```bash
python3 scripts/verify/verify_release.py
```

在物理证据齐全前它应返回非零。旧 `config/`、`reports/` 路径仅为 v0.6.x 只读
兼容入口，新结果不得写入旧目录。
