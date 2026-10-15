from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests"))
from ros_adapter_helpers import ManualFuture, adapter_classes


def make_node(operation=1):
    clock = NS(value=10.0)
    action = NS(
        Goal=NS(OP_ESTOP_SET=1, OP_CLEAR_FAULT=2, OP_LINE_CONTROL=3, OP_GET_INFO=4),
        Result=NS(
            RESULT_QUEUED=1,
            RESULT_APPLIED=2,
            RESULT_REJECTED=3,
            RESULT_TIMEOUT=5,
            RESULT_CANCELLED=6,
        ),
    )
    classes = adapter_classes(
        ROOT / "src/robot_chassis_ops/robot_chassis_ops/chassis_ops_node.py",
        time=NS(monotonic=lambda: clock.value),
        ChassisOperation=action,
    )
    node = classes.ChassisOpsNode.__new__(classes.ChassisOpsNode)
    node.wire_session = 7
    node.state = NS(wire_session_id=7, status_sequence=100, estop_active=False)
    node.state_at = clock.value
    node.state_timeout = 0.25
    node.operation_timeout = 1.0
    node.info_generations = {7: 1}
    node.operations = []
    response, completion = ManualFuture(), ManualFuture()
    goal = NS(request=NS(operation=operation), is_cancel_requested=False)
    run = classes._OperationRun(goal, response, completion, 7, 100, 1, clock.value)
    node.operations.append(run)
    return node, clock, response, completion


def test_clear_fault_reports_queued_without_claiming_observed_state_is_its_result():
    node, _, response, completion = make_node(operation=2)
    response.set_result(NS(success=True))
    node._tick()
    assert completion.result()[0] == 1


def test_changed_session_cannot_confirm_estop():
    node, clock, response, completion = make_node()
    response.set_result(NS(success=True))
    node.wire_session = 8
    node.state = NS(wire_session_id=8, status_sequence=1, estop_active=True)
    node.state_at = clock.value
    node._tick()
    assert completion.result()[0] == 3


def test_duplicate_or_regressed_status_cannot_confirm_estop():
    for sequence in (100, 99):
        node, _, response, completion = make_node()
        response.set_result(NS(success=True))
        node.state.status_sequence = sequence
        node.state.estop_active = True
        node._tick()
        assert not completion.done()


def test_estop_applied_requires_fresh_forward_same_session_evidence():
    node, clock, response, completion = make_node()
    response.set_result(NS(success=True))
    node.state.status_sequence = 101
    node.state.estop_active = True
    node.state_at = clock.value - 0.01
    node._tick()
    assert not completion.done()
    node.state_at = clock.value
    node._tick()
    assert completion.result()[0] == 2


def test_get_info_requires_new_hello_in_operation_session():
    node, _, response, completion = make_node(operation=4)
    response.set_result(NS(success=True))
    node._info(NS(wire_session_id=8))
    node._tick()
    assert not completion.done()
    node._info(NS(wire_session_id=7))
    node._tick()
    assert completion.result()[0] == 2


def test_pending_service_can_timeout_or_cancel_without_asyncio():
    node, clock, _, completion = make_node()
    clock.value += 1.01
    node._tick()
    assert completion.result()[0] == 5
    node, _, _, completion = make_node()
    node.operations[0].goal.is_cancel_requested = True
    node._tick()
    assert completion.result()[0] == 6
