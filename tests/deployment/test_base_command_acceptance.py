import importlib.util
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[2]


def load_readiness():
    spec = importlib.util.spec_from_file_location(
        "base_readiness_test", ROOT / "launch_scripts/base_test_readiness.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_command(monkeypatch):
    for name in ("rclpy", "geometry_msgs", "nav_msgs"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    for name, symbol in (("geometry_msgs.msg", "Twist"), ("nav_msgs.msg", "Odometry")):
        module = ModuleType(name)
        setattr(module, symbol, object)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "base_command_test", ROOT / "launch_scripts/base_cmd_test_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def odom(stamp, x=0.0):
    return NS(
        header=NS(stamp=NS(sec=0, nanosec=stamp)),
        pose=NS(pose=NS(position=NS(x=x, y=0.0), orientation=NS(z=0.0, w=1.0))),
        twist=NS(twist=NS(linear=NS(x=0.0), angular=NS(z=0.0))),
    )


def ready_gate(mode="real"):
    gate = load_readiness().BaseReadiness(mode)
    messages = {
        "link": NS(
            wire_session_id=7,
            link_state=6,
            protocol_compatible=True,
            wire_rearm_required=False,
            status_age_ms=10,
        ),
        "compatibility": NS(
            state=2 if mode == "fake" else 1,
            observation_ready=True,
            wire_session_id=7,
            observation_session_id=7,
            permit_formal_odometry=True,
        ),
        "supervision": NS(
            level=1,
            release_host_candidate=False,
            command_scale=1.0,
            observation_session_id=7,
            observation_age_ms=10,
        ),
        "host": NS(rearm_required=True, control_state=3),
    }
    for name, message in messages.items():
        gate.observe(name, message, 1.0)
    gate.observe("odom", odom(10), 1.0)
    gate.observe("odom", odom(20), 1.0)
    return gate


def test_ready_requires_current_business_state_and_advancing_odometry():
    gate = ready_gate()
    assert gate.reason(1.1) == ""
    gate.messages["compatibility"].observation_session_id = 6
    assert "compatibility" in gate.reason(1.1)
    gate.messages["compatibility"].observation_session_id = 7
    gate.messages["link"].wire_rearm_required = True
    assert "wire ready" in gate.reason(1.1)
    gate.messages["link"].wire_rearm_required = False
    gate.messages["supervision"].release_host_candidate = True
    assert "supervision" in gate.reason(1.1)
    assert ready_gate("fake").reason(1.1) == ""
    frozen = ready_gate()
    for message in ("link", "compatibility", "supervision", "host"):
        frozen.observe(message, frozen.messages[message], 2.0)
    frozen.observe("odom", odom(20), 2.0)
    assert "odom" in frozen.reason(2.0)
    reconnected = ready_gate()
    new_link = NS(**vars(reconnected.messages["link"]))
    new_link.wire_session_id = 8
    reconnected.observe("link", new_link, 1.2)
    assert reconnected.odom_samples == 0
    assert "compatibility" in reconnected.reason(1.2)


@pytest.mark.parametrize(
    "control_state,rearm_required,ready",
    [
        (0, False, False),
        (0, True, False),
        (1, False, False),
        (1, True, False),
        (2, False, False),
        (2, True, False),
        (3, False, True),
        (3, True, True),
        (4, False, True),
        (4, True, False),
    ],
)
def test_readiness_host_state_waits_for_fresh_intent_without_circular_rearm(
    control_state, rearm_required, ready
):
    gate = ready_gate()
    gate.messages["host"].control_state = control_state
    gate.messages["host"].rearm_required = rearm_required
    reason = gate.reason(1.1)
    if ready:
        assert reason == ""
    else:
        assert "Host control" in reason


def test_readiness_freshness_comes_from_canonical_compiled_timing():
    module = load_readiness()
    assert (
        module.freshness_limit({"stm32_bridge": {"ros__parameters": {"status_timeout": 0.25}}})
        == 0.25
    )
    for value in (None, float("nan"), float("inf"), 0.0, -0.1):
        with pytest.raises(ValueError):
            module.freshness_limit({"stm32_bridge": {"ros__parameters": {"status_timeout": value}}})


def test_summary_requires_actual_motion_and_stop_evidence(monkeypatch):
    module = load_command(monkeypatch)
    phase = module.PhaseStats("rotate", 0.0, 0.35, 1.5)
    assert phase.execution_status() == "NO_ODOM"
    phase.observe(0, 0, 0, 0, 0)
    phase.observe(0, 0, 0, 0, 0)
    assert phase.execution_status() == "NO_MOTION"
    phase.observe(0, 0, -0.1, 0, -0.35)
    assert phase.execution_status() == "NO_MOTION"
    phase.observe(0, 0, 0.1, 0, 0.35)
    assert phase.execution_status() == "EXECUTED"
    assert "status=EXECUTED" in phase.summary_line()
    forward = module.PhaseStats("forward", 0.08, 0.0, 1.5)
    forward.observe(0, 0, 0, 0, 0)
    forward.observe(0.01, 0, 0, 0.08, 0)
    assert forward.execution_status() == "EXECUTED"
    stopped = module.PhaseStats("stop", 0.0, 0.0, 0.8)
    stopped.observe(0, 0, 0, 0.1, 0)
    stopped.observe(0.01, 0, 0, 0.1, 0)
    assert stopped.execution_status() == "NOT_STOPPED"
    stopped.observe(0.01, 0, 0, 0, 0)
    assert stopped.execution_status() == "STOPPED"
    before = stopped.sample_count
    stopped.observe(float("nan"), 0, 0, 0, 0)
    assert stopped.sample_count == before


def test_rotation_summary_keeps_multiple_turns(monkeypatch):
    module = load_command(monkeypatch)
    phase = module.PhaseStats("rotation", 0.0, 1.0, 8.0)
    for degrees in range(0, 451, 30):
        phase.observe(0, 0, module.wrap_angle(math.radians(degrees)), 0, 1.0)
    assert phase.yaw_delta_deg() == pytest.approx(450.0)


@pytest.mark.parametrize(
    "events,expected",
    [
        ([(0.6, 0.6, 0.0), (0.7, 0.7, 0.0)], 0),
        ([], 4),
        ([(0.6, 0.6, 0.0), (0.7, 0.6, 0.0)], 4),
        ([(0.6, -0.5, 0.0), (0.7, -0.4, 0.0)], 4),
        ([(0.6, 0.6, float("nan")), (0.7, 0.7, float("nan"))], 4),
    ],
    ids=("late_reader", "no_odom", "frozen_stamp", "stale_samples", "invalid_samples"),
)
def test_command_startup_waits_for_its_own_fresh_advancing_odometry(
    monkeypatch, events, expected, capsys
):
    # This deterministic host double checks startup ordering; it is not DDS/ROS evidence.
    module = load_command(monkeypatch)
    monkeypatch.delenv("ROBOT_EFFECTIVE_PARAMS", raising=False)
    clock = NS(now=0.0)
    pending = list(events)
    published = []
    phase_starts = []
    position = NS(x=0.0, yaw=0.0)
    publisher = NS(
        get_subscription_count=lambda: 1,
        publish=lambda msg: published.append((clock.now, msg.linear.x, msg.angular.z)),
    )

    class Node:
        callback = None
        destroyed = False

        def create_publisher(self, *args):
            return publisher

        def create_subscription(self, message_type, topic, callback, depth):
            assert topic == "/odom"
            self.callback = callback

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=int((100.0 + clock.now) * 1e9)))

        def destroy_node(self):
            self.destroyed = True

    node = Node()
    shutdown_calls = []

    def sample(stamp, x=0.0, yaw=0.0, vx=0.0, wz=0.0):
        message = odom(0, x)
        stamp_ns = int((100.0 + stamp) * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.pose.pose.orientation.z = math.sin(yaw / 2)
        message.pose.pose.orientation.w = math.cos(yaw / 2)
        message.twist.twist.linear.x = vx
        message.twist.twist.angular.z = wz
        return message

    def spin_once(current_node, timeout_sec):
        assert current_node is node
        clock.now += timeout_sec
        while pending and pending[0][0] <= clock.now + 1e-9:
            _, stamp, x = pending.pop(0)
            node.callback(sample(stamp, x))

    def publish_phase(current_node, current_publisher, vx, wz, duration, hz, label):
        phase_starts.append((label, clock.now))
        current_publisher.publish(NS(linear=NS(x=vx), angular=NS(z=wz)))
        for _ in range(2):
            clock.now += 0.05
            position.x += vx * 0.05
            position.yaw += wz * 0.05
            node.callback(sample(clock.now, position.x, position.yaw, vx, wz))

    def publish_cleanup_stop(current_node, current_publisher, hz, duration):
        current_publisher.publish(NS(linear=NS(x=0.0), angular=NS(z=0.0)))

    for name, function in (
        ("init", lambda: None),
        ("create_node", lambda name: node),
        ("ok", lambda: True),
        ("spin_once", spin_once),
        ("shutdown", lambda: shutdown_calls.append(True)),
    ):
        monkeypatch.setattr(module.rclpy, name, function, raising=False)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(module, "publish_for", publish_phase)
    monkeypatch.setattr(module, "publish_stop", publish_cleanup_stop)
    result = module.main(["--wait-subscriber-sec", "1.0", "--odom-settle-sec", "0"])
    assert result == expected
    assert node.destroyed and shutdown_calls == [True]
    if expected == 0:
        assert phase_starts[0][0] == "stop_before"
        assert phase_starts[0][1] >= 0.7
        assert all(at >= 0.7 for at, _, _ in published)
        assert any(vx != 0.0 or wz != 0.0 for _, vx, wz in published)
    else:
        assert not phase_starts
        assert clock.now == pytest.approx(1.0)
        assert published and all(vx == 0.0 and wz == 0.0 for _, vx, wz in published)
        assert "no fresh advancing /odom" in capsys.readouterr().err


def test_command_startup_freshness_uses_compiled_timing(monkeypatch, tmp_path):
    module = load_command(monkeypatch)
    parameters = tmp_path / "effective-params.yaml"
    parameters.write_text("stm32_bridge:\n  ros__parameters:\n    status_timeout: 0.15\n")
    monkeypatch.setenv("ROBOT_EFFECTIVE_PARAMS", str(parameters))
    assert module.odom_freshness_limit() == 0.15
    parameters.write_text("stm32_bridge:\n  ros__parameters:\n    status_timeout: .nan\n")
    with pytest.raises(ValueError):
        module.odom_freshness_limit()


def bash_path():
    if os.name == "nt":
        candidate = Path.home() / "scoop/apps/git/current/bin/bash.exe"
        if candidate.is_file():
            return str(candidate)
    return shutil.which("bash")


@pytest.mark.parametrize(
    "ready_exit,summary,expected,legacy_timing",
    [
        (
            0,
            "SUMMARY rotate: samples=5 status=EXECUTED\nSUMMARY stop: samples=5 status=STOPPED",
            0,
            None,
        ),
        (2, "", 1, None),
        (0, "SUMMARY rotate: samples=0 status=NO_ODOM", 1, None),
        (0, "", 1, None),
        (0, "", 2, "BASE_TEST_CMD_TIMEOUT"),
        (0, "", 2, "BASE_TEST_KEEPALIVE"),
        (0, "", 2, "NO_SETSID"),
    ],
)
def test_base_entry_uses_one_chain_and_requires_summary(
    tmp_path, ready_exit, summary, expected, legacy_timing
):
    bash = bash_path()
    if not bash:
        pytest.skip("bash unavailable for shell entry regression")
    scripts = tmp_path / "launch_scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(ROOT / "launch_scripts/test_base_cmd.sh", scripts / "test_base_cmd.sh")
    (scripts / "lib/common.sh").write_text(
        """
log_info() { echo "$*"; }
log_error() { echo "$*" >&2; }
log_success() { echo "SUCCESS $*"; }
print_header() { :; }
ensure_workspace_built() { :; }
setup_ros_env() { :; }
command() {
  if [ "${MISSING_SETSID:-0}" = 1 ] && [ "$1" = -v ] && [ "$2" = setsid ]; then return 1; fi
  builtin command "$@"
}
kill() {
  if [ "$1" = -0 ]; then
    case "${@: -1}" in -*) return 1 ;; *) return 0 ;; esac
  fi
  printf 'SIGNAL %s\\n' "$*" >> "$CALLS"
}
""",
        encoding="utf-8",
    )
    binaries = tmp_path / "bin"
    binaries.mkdir()
    (binaries / "setsid").write_text(
        """#!/bin/bash
printf 'SESSION pid=%s %s\\n' "$$" "$*" >> "$CALLS"
exec "$@"
""",
        encoding="utf-8",
    )
    (binaries / "ros2").write_text(
        """#!/bin/bash
printf '%s\\n' "$*" >> "$CALLS"
""",
        encoding="utf-8",
    )
    (binaries / "python3").write_text(
        """#!/bin/bash
printf '%s\\n' "$*" >> "$CALLS"
case "$1" in
 *base_test_readiness.py) echo 'READY test fixture'; exit "$READY_EXIT" ;;
 *base_cmd_test_node.py) printf '%s\\n' "$SUMMARY" ;;
esac
""",
        encoding="utf-8",
    )
    for executable in binaries.iterdir():
        executable.chmod(0o755)
    effective = tmp_path / "effective.yaml"
    effective.write_text("{}", encoding="utf-8")
    calls = tmp_path / "calls.log"
    environment = dict(
        os.environ,
        PATH=str(binaries) + os.pathsep + os.environ["PATH"],
        ROBOT_EFFECTIVE_PARAMS=str(effective),
        BASE_TEST_COUNTDOWN="0",
        BASE_TEST_STOP_FIRST="false",
        BASE_TEST_LOG_DIR=str(tmp_path / "logs"),
        CALLS=str(calls),
        READY_EXIT=str(ready_exit),
        SUMMARY=summary,
    )
    # Git Bash otherwise rewrites ROS absolute topic arguments into Windows paths.
    environment["MSYS_NO_PATHCONV"] = "1"
    for name in ("BASE_TEST_CMD_TIMEOUT", "BASE_TEST_KEEPALIVE"):
        environment.pop(name, None)
    if legacy_timing == "NO_SETSID":
        environment["MISSING_SETSID"] = "1"
    elif legacy_timing:
        environment[legacy_timing] = "0.2"
    result = subprocess.run(
        [bash, str(scripts / "test_base_cmd.sh"), "--fake-base"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == expected, result.stdout + result.stderr
    if legacy_timing:
        expected_message = "setsid is required" if legacy_timing == "NO_SETSID" else "unsupported"
        assert expected_message in result.stderr and not calls.exists()
        return
    called = calls.read_text(encoding="utf-8")
    assert called.count("\nlaunch robot_bringup base_only.launch.py") == 1
    session = next(line for line in called.splitlines() if line.startswith("SESSION pid="))
    launch_pid = session.split()[1].split("=", 1)[1]
    assert f"SIGNAL -INT -- {launch_pid}" in called
    assert f"SIGNAL -INT -- -{launch_pid}" not in called
    assert "base_mode:=fake" in called and "effective_params_file:=" in called
    assert "control.launch.py" not in called and "stm32_bridge.launch.py" not in called
    assert ("SUCCESS " in result.stdout) == (expected == 0)
    if ready_exit:
        assert "base_cmd_test_node.py" not in called


@pytest.mark.skipif(os.name == "nt", reason="real process-group lifecycle requires POSIX setsid")
def test_failed_readiness_cleans_only_its_private_launch_group(tmp_path):
    import time

    bash = shutil.which("bash")
    if not bash or not shutil.which("setsid"):
        pytest.skip("POSIX bash and setsid unavailable")
    scripts = tmp_path / "launch_scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(ROOT / "launch_scripts/test_base_cmd.sh", scripts / "test_base_cmd.sh")
    (scripts / "lib/common.sh").write_text(
        'log_info() { :; }; log_error() { echo "$*" >&2; }; log_success() { echo SUCCESS; }; '
        "print_header() { :; }; ensure_workspace_built() { :; }; setup_ros_env() { :; }\n",
        encoding="utf-8",
    )
    binaries = tmp_path / "bin"
    binaries.mkdir()
    (binaries / "ros2").write_text(
        f"#!{sys.executable}\n"
        """import os, signal, subprocess, sys, time
from pathlib import Path
if sys.argv[1] != "launch":
    raise SystemExit(0)
signal.signal(signal.SIGINT, signal.SIG_IGN)
child_ready = Path(os.environ["OWN_PIDS"] + ".child")
child = subprocess.Popen([sys.executable, "-c",
    "import signal,time; from pathlib import Path; "
    "signal.signal(signal.SIGINT,signal.SIG_IGN); "
    "Path(" + repr(str(child_ready)) + ").write_text('ready'); time.sleep(30)"])
def terminate_group_member(signum, frame):
    child.terminate()
    child.wait(timeout=2)
    raise SystemExit(0)
signal.signal(signal.SIGTERM, terminate_group_member)
while not child_ready.exists():
    time.sleep(0.01)
Path(os.environ["OWN_PIDS"]).write_text(str(os.getpid()) + " " + str(child.pid))
while True:
    time.sleep(0.05)
""",
        encoding="utf-8",
    )
    (binaries / "python3").write_text(
        """#!/bin/bash
for ((i=0; i<200; i++)); do
  if [ -s "$OWN_PIDS" ]; then echo 'NOT_READY fixture'; exit 2; fi
  sleep 0.01
done
exit 3
""",
        encoding="utf-8",
    )
    for executable in binaries.iterdir():
        executable.chmod(0o755)
    effective = tmp_path / "effective.yaml"
    effective.write_text("{}", encoding="utf-8")
    own_pids = tmp_path / "own_pids"
    environment = dict(
        os.environ,
        PATH=str(binaries) + os.pathsep + os.environ["PATH"],
        ROBOT_EFFECTIVE_PARAMS=str(effective),
        BASE_TEST_COUNTDOWN="0",
        BASE_TEST_STOP_FIRST="false",
        BASE_TEST_LOG_DIR=str(tmp_path / "logs"),
        OWN_PIDS=str(own_pids),
    )
    environment.pop("BASE_TEST_CMD_TIMEOUT", None)
    environment.pop("BASE_TEST_KEEPALIVE", None)
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True
    )
    try:
        started = time.monotonic()
        result = subprocess.run(
            [bash, str(scripts / "test_base_cmd.sh"), "--fake-base"],
            env=environment,
            capture_output=True,
            text=True,
            timeout=15,
        )
        assert result.returncode == 1, result.stdout + result.stderr
        assert time.monotonic() - started < 10, "cleanup must have bounded grace"
        assert unrelated.poll() is None, "cleanup must preserve other process groups"
        pids = [int(value) for value in own_pids.read_text().split()]
        assert len(pids) == 2
        for pid in pids:
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=3)
