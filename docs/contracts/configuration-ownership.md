# Configuration ownership

Canonical upper configuration lives under `src/robot_config/config` and is compiled into
one effective configuration plus SHA-256 and per-artifact SHA-256 manifest. Validation
uses Draft 7, rejects unknown/duplicate/nonfinite data and enforces physical and timing
invariants. Geometry, frames, upper safety timing and upper estimator parameters are
upper-owned. Firmware control/PID/motor parameters are firmware-owned; the upper
calibration bundle stores only the expected firmware identity and parameter CRC.

Code defaults are development fallbacks, not release facts. A release requires a final
calibration bundle, clean upper commit, non-null real hardware identity and matching
effective-config hash. Runtime nodes may reject configuration but may not rewrite the
canonical bundle.

`bin/robot` resolves supported CLI/environment overrides before compilation. Ports,
provider mode, fusion mode, lidar handedness and yaw are part of the effective hash
and provenance. `launch-env.sh`, lidar, EKF, SLAM, Nav2 and extrinsic artifacts belong
to the same manifest; launch checks the artifact hashes and logical configuration
identity before composing nodes. It consumes canonical frames, topics and footprint.
Conflicting launch arguments or serial-port aliases fail instead of changing the
execution after compilation. Update source calibration, not a generated artifact.
Legacy `BASE_CMD_TIMEOUT` and `BASE_DRIVE_KEEPALIVE_SEC` enter the timing section
before validation and hashing. Unimplemented v3 bridge log-interval overrides fail
explicitly.

Nonempty namespace is rejected until every node, topic, Action and TF consumer is
qualified together. `BASE_WHEEL_TRACK_WIDTH`, `BASE_ODOM_ANGULAR_SCALE` and
`BASE_ODOM_ANGULAR_SIGN` are rejected; their replacement is canonical calibration.
`base_only` reuses the provider, compatibility gate, command mux, state estimation,
supervision and chassis-operation chain without lidar, SLAM or Nav2.
