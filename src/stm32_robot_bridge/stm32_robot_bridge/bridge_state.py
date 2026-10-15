"""ROS-independent STM32 bridge communication state machine."""

from enum import IntEnum


FAULT_STATUS_MASK = 0x03


class BridgeState(IntEnum):
    DISCONNECTED = 0
    WAIT_HELLO = 1
    WAIT_STATUS = 2
    FAULT_LATCHED = 3
    WAIT_FAULT_CLEAR = 4
    WAIT_POST_CLEAR_DISABLE_ACK = 5
    WIRE_REARM_READY = 6
    DRIVE_ACTIVE = 7


class BridgeStateMachine:
    def __init__(self) -> None:
        self.state = BridgeState.DISCONNECTED
        self.fault_present = False
        self.expected_disable_sequence: int | None = None
        self.generation = 0

    def _set_state(self, state: BridgeState) -> None:
        if self.state is not state:
            self.state = state
            self.generation = (self.generation + 1) & 0xFFFFFFFF

    @property
    def can_drive(self) -> bool:
        return (
            self.state in (BridgeState.WIRE_REARM_READY, BridgeState.DRIVE_ACTIVE)
            and not self.fault_present
        )

    def on_serial_opened(self) -> None:
        self._set_state(BridgeState.WAIT_HELLO)
        self.fault_present = False
        self.expected_disable_sequence = None

    def on_hello_compatible(self) -> None:
        if self.state is BridgeState.WAIT_HELLO:
            self._set_state(BridgeState.WAIT_STATUS)

    def on_hello_incompatible(self) -> None:
        self.expected_disable_sequence = None
        self._set_state(BridgeState.WAIT_HELLO)

    def on_valid_status(self, status_flags: int) -> None:
        has_fault = bool(int(status_flags) & FAULT_STATUS_MASK)
        if has_fault:
            self.fault_present = True
            self.expected_disable_sequence = None
            self._set_state(BridgeState.FAULT_LATCHED)
            return
        was_fault = self.fault_present or self.state in (
            BridgeState.FAULT_LATCHED,
            BridgeState.WAIT_FAULT_CLEAR,
        )
        self.fault_present = False
        if was_fault:
            self._set_state(BridgeState.WAIT_FAULT_CLEAR)

    def on_clear_fault_requested(self) -> None:
        if self.state is BridgeState.FAULT_LATCHED:
            self._set_state(BridgeState.WAIT_FAULT_CLEAR)

    def on_disable_sent(self, sequence: int) -> None:
        if not self.fault_present and self.state in (
            BridgeState.WAIT_STATUS,
            BridgeState.WAIT_FAULT_CLEAR,
            BridgeState.WIRE_REARM_READY,
            BridgeState.DRIVE_ACTIVE,
        ):
            self.expected_disable_sequence = int(sequence) & 0xFFFFFFFF
            self._set_state(BridgeState.WAIT_POST_CLEAR_DISABLE_ACK)

    def on_disable_applied(self, sequence: int) -> bool:
        if (
            self.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
            and self.expected_disable_sequence == (int(sequence) & 0xFFFFFFFF)
            and not self.fault_present
        ):
            self._set_state(BridgeState.WIRE_REARM_READY)
            self.expected_disable_sequence = None
            return True
        return False

    def on_drive_enabled(self) -> None:
        if self.can_drive:
            self._set_state(BridgeState.DRIVE_ACTIVE)

    def on_disable_command(self) -> None:
        if self.state is BridgeState.DRIVE_ACTIVE:
            self._set_state(BridgeState.WIRE_REARM_READY)

    def on_status_timeout(self) -> None:
        if self.state not in (BridgeState.DISCONNECTED, BridgeState.WAIT_HELLO):
            self._set_state(BridgeState.WAIT_STATUS)
            self.fault_present = False
            self.expected_disable_sequence = None

    def on_disconnected(self) -> None:
        self._set_state(BridgeState.DISCONNECTED)
        self.fault_present = False
        self.expected_disable_sequence = None
