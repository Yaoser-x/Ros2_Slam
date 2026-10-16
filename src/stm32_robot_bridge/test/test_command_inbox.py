from dataclasses import replace
from threading import Thread

from stm32_robot_bridge.command_inbox import CommandInbox, ReceivedCommand


def command(sequence=1, **changes):
    return replace(ReceivedCommand(0.05, 0.0, True, 3, 7, sequence, 1.0, 1.01, 1.01), **changes)


def test_inbox_retains_disables_conflicting_duplicates_and_epochs_in_order():
    inbox = CommandInbox()
    commands = [command(), command(vx=-0.05), command(2, enable=False),
                command(3, session_id=8), command(2, vx=float("nan"))]
    for item in commands:
        inbox.put(item)
    assert [inbox.take()[1] for _ in commands] == commands
    assert inbox.take() == (False, None)


def test_overflow_discards_all_old_commands_and_preserves_failure_across_clear():
    inbox = CommandInbox(capacity=2)
    for seq in range(4):
        inbox.put(command(seq))
    inbox.clear()
    assert inbox.take() == (True, None)
    assert inbox.take() == (False, None)
    fresh_disable = command(5, enable=False)
    inbox.put(fresh_disable)
    assert inbox.take() == (False, fresh_disable)


def test_stop_drops_pending_and_racing_future_input():
    inbox = CommandInbox()
    producer = Thread(target=lambda: [inbox.put(command(i)) for i in range(1000)])
    producer.start()
    inbox.stop()
    producer.join()
    inbox.put(command(1001))
    # Overflow, if it raced with stop, remains visible but cannot contain motion.
    assert inbox.take()[1] is None
    assert inbox.take() == (False, None)
