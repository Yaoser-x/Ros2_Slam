from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_ekf_fuses_wheel_twist_and_imu_gyro_with_wheel_fallback():
    config = yaml.safe_load(
        (ROOT / "src" / "robot_state_estimation" / "config" / "ekf_base.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert config["frequency"] == 30.0
    assert config["sensor_timeout"] == 0.2
    assert config["two_d_mode"] is True
    assert config["publish_tf"] is False
    assert config["odom0"] == "wheel/odom"
    assert [index for index, enabled in enumerate(config["odom0_config"]) if enabled] == [6, 11]
    assert config["imu0"] == "imu/data"
    assert [index for index, enabled in enumerate(config["imu0_config"]) if enabled] == [11]
    assert config["imu0_remove_gravitational_acceleration"] is False
    assert len(config["process_noise_covariance"]) == 225
    assert len(config["initial_estimate_covariance"]) == 225
    assert all(isinstance(value, float) for value in config["process_noise_covariance"])
    assert all(isinstance(value, float) for value in config["initial_estimate_covariance"])

    wheel_only = yaml.safe_load(
        (ROOT / "src/robot_state_estimation/config/ekf_wheel.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert all(isinstance(value, float) for value in wheel_only["process_noise_covariance"])
    assert all(isinstance(value, float) for value in wheel_only["initial_estimate_covariance"])


def test_real_base_tf_has_one_odom_to_base_link_owner():
    system = (ROOT / "src" / "robot_bringup" / "launch" / "system.launch.py").read_text(
        encoding="utf-8"
    )
    bridge_launch = (
        ROOT / "src" / "stm32_robot_bridge" / "launch" / "stm32_bridge.launch.py"
    ).read_text(encoding="utf-8")
    ekf = (ROOT / "src" / "robot_state_estimation" / "config" / "ekf_base.yaml").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("base_fusion_mode", default_value="")' in system
    profile = yaml.safe_load(
        (ROOT / "src/robot_config/config/profiles/mapping.yaml").read_text(encoding="utf-8")
    )
    assert profile["runtime"]["fusion_mode"] == "wheel"
    state_launch = (
        ROOT / "src" / "robot_state_estimation" / "launch" / "state_estimation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'package="robot_localization"' in state_launch
    assert 'if fusion_mode == "wheel_imu"' in state_launch
    assert 'effective["platform"]["topics"]["wheel_odom"]' in state_launch
    assert "load_launch_config" in state_launch
    assert "wheel_odom_republisher" not in state_launch
    assert "bridge v3 never owns odom->base_link TF" in (
        ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert '"publish_tf": False' in (
        ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert "publish_tf: false" in ekf
    assert 'executable="formal_odometry_node.py"' in state_launch
    assert "TransformBroadcaster" in (
        ROOT / "src/robot_state_estimation/robot_state_estimation/formal_odometry_node.py"
    ).read_text(encoding="utf-8")
    formal = (
        ROOT / "src/robot_state_estimation/robot_state_estimation/formal_odometry_node.py"
    ).read_text(encoding="utf-8")
    assert "fresh_evidence_at" in formal
    assert "transport_session_id" in formal


def test_ekf_experiment_has_four_groups_and_keeps_wheel_default():
    experiment = yaml.safe_load(
        (ROOT / "verification/configs/experiments/ekf-comparison-v0.6.0-rc2.yaml").read_text()
    )

    assert set(experiment["groups"]) == {"A", "B", "C", "D"}
    assert experiment["promotion"]["default"] == "A"
    assert experiment["groups"]["A"]["base_fusion_mode"] == "wheel"
    assert experiment["status"].startswith("provisional")
