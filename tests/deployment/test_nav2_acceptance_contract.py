from pathlib import Path
import importlib.util

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "verification/configs/experiments/nav2-acceptance-v0.6.0-rc2.yaml"


def test_nav2_acceptance_requires_dynamics_and_fifty_repeatable_trials():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert config["vehicle_characterization"]["required_status"] == "PASS"
    assert len(config["vehicle_characterization"]["required_measurements"]) == 8
    assert len(config["targets"]) == 10
    assert config["repetitions_per_target"] == 5
    assert config["acceptance"]["total_trials"]["limit"] == 50
    assert config["acceptance"]["collision_count"]["limit"] == 0
    assert config["acceptance"]["communication_degradation_count"]["limit"] == 0
    assert config["fixed_map_sha256"] is None
    assert config["status"] == "dynamics_and_dataset_pending"


def test_current_navigation_gate_counts_and_requires_communication_evidence():
    spec = importlib.util.spec_from_file_location(
        "navigation_rc2_gate", ROOT / "tools/experiments/navigation_gate.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = module.load_contract(CONFIG)
    runs = [
        {
            "target_id": f"target_{i:02d}",
            "success": True,
            "position_error_m": 0.1,
            "yaw_error_rad": 0.2,
            "recovery_count": 0,
            "collision": False,
            "manual_intervention": False,
            "unexpected_motion": False,
            "communication_degradation": False,
        }
        for i in range(1, 11)
        for _ in range(5)
    ]
    assert module.evaluate_runs(config, runs).passed
    runs[0]["communication_degradation"] = True
    assert "communication_degradation_count" in module.evaluate_runs(config, runs).failed
    del runs[0]["communication_degradation"]
    assert module.evaluate_runs(config, runs).coverage_errors
