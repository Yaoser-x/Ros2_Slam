# Chassis operation result contract

`/chassis/operation` is the sole public operation endpoint. Bridge-private
`/stm32_bridge/wire_*` services report queue admission only.

| Result | Required evidence | ROS Action terminal |
| --- | --- | --- |
| `QUEUED` | wire queue accepted; no request-correlated applied result exists | succeed |
| `APPLIED` | a new same-session firmware fact confirms ESTOP or a fresh HELLO | succeed |
| `REJECTED` | local validation, link/session or wire adapter rejected it | abort |
| `CONDITION_NOT_CLEARED` | reserved for a protocol with request-correlated clear evidence; not emitted by v3 | abort |
| `TIMEOUT` | monotonic deadline expired before required evidence | abort |
| `CANCELLED` | caller stopped waiting | canceled |

Upper v3 returns `QUEUED` for successful `CLEAR_FAULT` and `LINE_CONTROL` admission.
A later fault-free STATUS is useful observation, but cannot prove that this specific
clear request was executed. `ESTOP_SET` may return `APPLIED` after a fresh ESTOP STATUS;
`GET_INFO` may do so after a new HELLO in the operation's captured wire session.
Neither result is a firmware request receipt.

The wrapper captures the wire session and STATUS/HELLO generation before dispatch.
STATUS confirmation must advance modulo 2^32 from an existing sequence baseline;
duplicate, regressed and other-session facts cannot confirm the operation. A session
change rejects the wait. The default combined queue/evidence deadline is one monotonic
second, driven by a steady timer and native rclpy Future.

`CANCELLED` stops the caller's wait. Cancellation and timeout cannot retract an admitted
frame or prove that the firmware stopped; inspect new firmware state separately.
