"""ROS- and serial-independent bridge event core."""

from __future__ import annotations

import math
import secrets
import struct
from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Optional, Tuple

from .bridge_state import BridgeState, BridgeStateMachine
from .protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_REJECTED,
    ACK_SESSION_VALID,
    REQUIRED_CAPABILITIES,
    STATUS_FLAG_ESTOP,
    STATUS_FLAG_FAULT_STOP,
    HelloPayload,
    StatusPayload,
    sequence_is_forward,
)


REARM_TRANSPORT = 1 << 0
REARM_STATUS_TIMEOUT = 1 << 1
REARM_ACK_TIMEOUT = 1 << 2
REARM_FIRMWARE_REJECT = 1 << 3
REARM_ESTOP = 1 << 4
REARM_FAULT = 1 << 5
REARM_COMMAND_TIMEOUT = 1 << 6
REARM_PROTOCOL_INCOMPATIBLE = 1 << 9
REARM_HOST_REPLAY = 1 << 10

INCOMPAT_PROTOCOL = 1 << 0
INCOMPAT_SCHEMA = 1 << 1
INCOMPAT_CAPABILITIES = 1 << 2


class StatusDisposition(IntEnum):
    INVALID = 0
    NEW = 1
    DUPLICATE = 2
    OUT_OF_ORDER = 3


class CommandDisposition(IntEnum):
    ACCEPT_ENABLE = 1
    ACCEPT_DISABLE = 2
    IGNORE_DUPLICATE = 3
    REJECT_STALE = 4
    REJECT_OUT_OF_ORDER = 5
    REJECT_OLD_SESSION = 6
    REJECT_INVALID_VALUE = 7
    REQUIRE_REARM = 8
    GLOBAL_RELEASE = 9


@dataclass(frozen=True)
class FirmwareSnapshot:
    received_at: float
    status: StatusPayload


@dataclass(frozen=True)
class RuntimeSnapshot:
    state: BridgeState = BridgeState.DISCONNECTED
    wire_session_id: int = 0
    wire_sent_sequence: int = 0
    command_session_id: int = 0
    command_sequence: int = 0
    selected_source: int = 0
    target_vx: float = 0.0
    target_wz: float = 0.0
    firmware: Optional[FirmwareSnapshot] = None
    hello: Optional[HelloPayload] = None
    last_status_at: Optional[float] = None
    last_command_at: Optional[float] = None
    rearm_required: bool = False
    rearm_reason_flags: int = 0
    protocol_compatible: bool = False
    incompatibility_flags: int = 0
    reset_generation: int = 0


@dataclass(frozen=True)
class ValidatedCommand:
    vx: float
    wz: float
    enable: bool
    source: int
    session_id: int
    sequence: int


@dataclass(frozen=True)
class CommandDecision:
    disposition: CommandDisposition
    validated_command: Optional[ValidatedCommand]
    reason: str
    rearm_reason_flags: int


class BridgeCore:
    def __init__(
        self,
        *,
        hard_max_linear_mps: float,
        hard_max_angular_radps: float,
        command_timeout_sec: float,
        status_timeout_sec: float,
        max_command_age_sec: float,
        required_capabilities: int = REQUIRED_CAPABILITIES,
    ) -> None:
        self.hard_max_linear_mps = abs(float(hard_max_linear_mps))
        self.hard_max_angular_radps = abs(float(hard_max_angular_radps))
        self.command_timeout_sec = float(command_timeout_sec)
        self.status_timeout_sec = float(status_timeout_sec)
        self.max_command_age_sec = float(max_command_age_sec)
        self.required_capabilities = int(required_capabilities)
        self.machine = BridgeStateMachine()
        self.snapshot = RuntimeSnapshot()
        self._retired_host_epochs: deque[int] = deque(maxlen=16)
        self._host_disable_observed = False
        self._host_rearm_required = True
        self._last_host_command: Optional[ValidatedCommand] = None

    @property
    def state(self) -> BridgeState:
        return self.machine.state

    @property
    def can_drive(self) -> bool:
        return (
            self.machine.can_drive
            and self.snapshot.protocol_compatible
            and not self.snapshot.rearm_required
        )

    @property
    def state_generation(self) -> int:
        return self.machine.generation

    def on_connected(self, session_id: Optional[int] = None) -> RuntimeSnapshot:
        if self.snapshot.command_session_id:
            self._retired_host_epochs.append(self.snapshot.command_session_id)
        self.machine.on_serial_opened()
        wire_session = int(session_id if session_id is not None else secrets.randbits(64)) or 1
        # Every transport generation starts closed.  The wire-side release sent by
        # the adapter is necessary but is not an upper-layer enable edge; a fresh
        # HostMotionCommand(enable=false) must still be observed before motion.
        self.snapshot = RuntimeSnapshot(
            state=self.machine.state,
            wire_session_id=wire_session,
            rearm_required=True,
            rearm_reason_flags=REARM_TRANSPORT,
            reset_generation=(self.snapshot.reset_generation + 1) & 0xFFFFFFFF,
        )
        self._host_disable_observed = False
        self._host_rearm_required = True
        self._last_host_command = None
        return self.snapshot

    def record_wire_sequence(self, sequence: int) -> None:
        self.snapshot = replace(self.snapshot, wire_sent_sequence=int(sequence) & 0xFFFFFFFF)

    def on_clear_fault_requested(self) -> None:
        self.machine.on_clear_fault_requested()
        self.snapshot = replace(self.snapshot, state=self.machine.state)

    def on_disconnected(self) -> RuntimeSnapshot:
        self._host_disable_observed = False
        self._host_rearm_required = True
        self.machine.on_disconnected()
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            firmware=None,
            hello=None,
            last_status_at=None,
            target_vx=0.0,
            target_wz=0.0,
            last_command_at=None,
            rearm_required=True,
            rearm_reason_flags=REARM_TRANSPORT,
            protocol_compatible=False,
            incompatibility_flags=0,
        )
        return self.snapshot

    def on_hello(self, hello: HelloPayload) -> bool:
        flags = 0
        if hello.version != 3:
            flags |= INCOMPAT_PROTOCOL
        if hello.schema_version != 1:
            flags |= INCOMPAT_SCHEMA
        if (hello.capabilities & self.required_capabilities) != self.required_capabilities:
            flags |= INCOMPAT_CAPABILITIES
        compatible = flags == 0
        identity_changed = self.snapshot.hello is not None and self.snapshot.hello != hello
        if identity_changed:
            self.require_rearm(REARM_TRANSPORT)
            self.machine.on_status_timeout()
            self.snapshot = replace(self.snapshot, firmware=None, last_status_at=None)
        if compatible:
            self.machine.on_hello_compatible()
        else:
            self.require_rearm(REARM_PROTOCOL_INCOMPATIBLE)
            self.machine.on_hello_incompatible()
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            hello=hello,
            protocol_compatible=compatible,
            incompatibility_flags=flags,
        )
        return compatible

    def on_status(self, status: StatusPayload, now_sec: float) -> StatusDisposition:
        if (
            status.version != 3
            or self.snapshot.hello is None
            or not self.snapshot.protocol_compatible
        ):
            return StatusDisposition.INVALID
        previous = self.snapshot.firmware
        if previous is not None:
            if status.status_sequence == previous.status.status_sequence:
                return StatusDisposition.DUPLICATE
            if not sequence_is_forward(status.status_sequence, previous.status.status_sequence):
                return StatusDisposition.OUT_OF_ORDER
        self.machine.on_valid_status(status.status_flags)
        expected = self.machine.expected_disable_sequence
        required_ack = ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED
        ack_current = bool(
            expected is not None
            and status.last_received_session_id == self.snapshot.wire_session_id
            and status.last_received_sequence == expected
            and status.last_applied_sequence == expected
            and status.command_ack_flags & required_ack == required_ack
            and not status.command_ack_flags & ACK_REJECTED
            and status.last_reject_reason == 0
        )
        if expected is not None and ack_current and self.machine.on_disable_applied(expected):
            self.snapshot = replace(
                self.snapshot,
                rearm_required=self._host_rearm_required,
                rearm_reason_flags=(
                    self.snapshot.rearm_reason_flags if self._host_rearm_required else 0
                ),
            )
        if status.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP):
            reason = 0
            if status.status_flags & STATUS_FLAG_ESTOP:
                reason |= REARM_ESTOP
            if status.status_flags & STATUS_FLAG_FAULT_STOP:
                reason |= REARM_FAULT
            self.require_rearm(reason)
        firmware = FirmwareSnapshot(float(now_sec), status)
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            firmware=firmware,
            last_status_at=float(now_sec),
        )
        return StatusDisposition.NEW

    def require_rearm(self, reason_flags: int) -> None:
        self._host_rearm_required = True
        self._host_disable_observed = False
        self.snapshot = replace(
            self.snapshot,
            rearm_required=True,
            rearm_reason_flags=self.snapshot.rearm_reason_flags | int(reason_flags),
            target_vx=0.0,
            target_wz=0.0,
            last_command_at=None,
        )

    def accept_command(
        self,
        *,
        vx: float,
        wz: float,
        enable: bool,
        source: int,
        session_id: int,
        sequence: int,
        command_stamp_sec: float,
        now_ros_sec: float,
        now_monotonic: float,
        received_monotonic: Optional[float] = None,
    ) -> CommandDecision:
        # Header/ROS time bounds queue admission.  Once accepted, safety lease
        # freshness starts at monotonic callback receipt and is unaffected by
        # later /clock changes.
        received_at = now_monotonic if received_monotonic is None else received_monotonic
        values = (vx, wz, now_monotonic, received_at)
        if not all(math.isfinite(float(value)) for value in values):
            current = self.snapshot.command_session_id
            if current and int(session_id) == current:
                self.require_rearm(REARM_HOST_REPLAY)
                return self._decision(
                    CommandDisposition.GLOBAL_RELEASE,
                    None,
                    "current command contains non-finite value",
                )
            return self._decision(
                CommandDisposition.REJECT_INVALID_VALUE,
                None,
                "command contains non-finite value",
            )
        session_id = int(session_id) & 0xFFFFFFFFFFFFFFFF
        sequence = int(sequence) & 0xFFFFFFFF
        if session_id == 0:
            return self._decision(
                CommandDisposition.REJECT_INVALID_VALUE,
                None,
                "session_id must be non-zero",
            )
        current_epoch = self.snapshot.command_session_id
        if received_at > now_monotonic or now_monotonic - received_at > self.command_timeout_sec:
            return self._reject_freshness(
                session_id,
                current_epoch,
                CommandDisposition.REJECT_STALE,
                "command receipt lease expired before processing",
            )
        epoch_changed = current_epoch not in (0, session_id)
        if session_id in self._retired_host_epochs:
            return self._decision(
                CommandDisposition.REJECT_OLD_SESSION,
                None,
                "retired ROS command epoch",
            )
        normalized = ValidatedCommand(
            self._float32(max(-self.hard_max_linear_mps, min(self.hard_max_linear_mps, float(vx)))),
            self._float32(
                max(-self.hard_max_angular_radps, min(self.hard_max_angular_radps, float(wz)))
            ),
            bool(enable),
            int(source) & 0xFF,
            session_id,
            sequence,
        )
        if not normalized.enable:
            normalized = ValidatedCommand(0.0, 0.0, False, 0, session_id, sequence)
        if current_epoch == session_id:
            if sequence == self.snapshot.command_sequence:
                if normalized == self._last_host_command:
                    return self._decision(
                        CommandDisposition.IGNORE_DUPLICATE,
                        normalized,
                        "idempotent duplicate",
                    )
                self.require_rearm(REARM_HOST_REPLAY)
                return self._decision(
                    CommandDisposition.GLOBAL_RELEASE,
                    None,
                    "duplicate sequence has a different payload",
                )
            if not sequence_is_forward(sequence, self.snapshot.command_sequence):
                return self._decision(
                    CommandDisposition.REJECT_OUT_OF_ORDER,
                    None,
                    "out-of-order ROS command sequence",
                )
        try:
            command_stamp = float(command_stamp_sec)
            ros_now = float(now_ros_sec)
        except (TypeError, ValueError):
            command_stamp = ros_now = float("nan")
        if not math.isfinite(command_stamp) or not math.isfinite(ros_now):
            return self._reject_freshness(
                session_id,
                current_epoch,
                CommandDisposition.REJECT_INVALID_VALUE,
                "command timestamp is not finite",
            )
        # A zero/zero pair is retained for deterministic non-ROS core tests and
        # startup at ROS epoch zero.  Otherwise the header stamp is an
        # admission bound; the active lease itself remains monotonic-clock
        # based and therefore cannot be extended by /clock jumps.
        if command_stamp != 0.0 or ros_now != 0.0:
            age = ros_now - command_stamp
            if age > self.max_command_age_sec:
                return self._reject_freshness(
                    session_id,
                    current_epoch,
                    CommandDisposition.REJECT_STALE,
                    f"command is stale by {age:.6f} s",
                )
            if age < -self.max_command_age_sec:
                return self._reject_freshness(
                    session_id,
                    current_epoch,
                    CommandDisposition.REJECT_INVALID_VALUE,
                    f"command timestamp is in the future by {-age:.6f} s",
                )
        if current_epoch != session_id and enable and not self._host_disable_observed:
            self.require_rearm(REARM_HOST_REPLAY)
            return self._decision(
                CommandDisposition.GLOBAL_RELEASE,
                None,
                "new command epoch requires a preceding disable",
            )
        if enable and current_epoch == session_id and self._host_disable_observed:
            self.require_rearm(REARM_HOST_REPLAY)
            return self._decision(
                CommandDisposition.GLOBAL_RELEASE,
                None,
                "enable after disable requires a new command epoch",
            )
        if not enable:
            # A wire disable ACK only qualifies the wire link.  The host must
            # provide its own fresh disable after a safety event has ended.
            self._host_disable_observed = not self.machine.fault_present
            self._host_rearm_required = self.machine.fault_present
            self._last_host_command = normalized
            if epoch_changed:
                self._retired_host_epochs.append(current_epoch)
            rearm_required = self._host_rearm_required or not self.machine.can_drive
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                command_session_id=session_id,
                command_sequence=sequence,
                selected_source=0,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=rearm_required,
                rearm_reason_flags=(self.snapshot.rearm_reason_flags if rearm_required else 0),
            )
            return self._decision(
                CommandDisposition.ACCEPT_DISABLE,
                normalized,
                "disable accepted",
            )
        if self.snapshot.rearm_required:
            return self._decision(
                CommandDisposition.REQUIRE_REARM,
                None,
                "disable-enable rearm is required",
            )
        status_age = (
            None
            if self.snapshot.last_status_at is None
            else now_monotonic - self.snapshot.last_status_at
        )
        if status_age is None or status_age < 0.0 or status_age > self.status_timeout_sec:
            self.require_rearm(REARM_STATUS_TIMEOUT)
            return self._decision(
                CommandDisposition.GLOBAL_RELEASE,
                None,
                "STATUS is unavailable or stale",
            )
        if self.snapshot.hello is None or not self.machine.can_drive:
            return self._decision(
                CommandDisposition.REQUIRE_REARM,
                None,
                "bridge state does not permit drive",
            )
        self.machine.on_drive_enabled()
        if epoch_changed:
            self._retired_host_epochs.append(current_epoch)
        self._host_disable_observed = False
        self._last_host_command = normalized
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            command_session_id=session_id,
            command_sequence=sequence,
            selected_source=normalized.source,
            target_vx=normalized.vx,
            target_wz=normalized.wz,
            last_command_at=float(received_at),
        )
        return self._decision(
            CommandDisposition.ACCEPT_ENABLE,
            normalized,
            "enable accepted",
        )

    def _reject_freshness(
        self,
        session_id: int,
        current_epoch: int,
        inactive_disposition: CommandDisposition,
        reason: str,
    ) -> CommandDecision:
        if current_epoch and session_id == current_epoch:
            self.require_rearm(REARM_HOST_REPLAY)
            return self._decision(CommandDisposition.GLOBAL_RELEASE, None, reason)
        return self._decision(inactive_disposition, None, reason)

    def _decision(
        self,
        disposition: CommandDisposition,
        command: Optional[ValidatedCommand],
        reason: str,
    ) -> CommandDecision:
        return CommandDecision(
            disposition,
            command,
            reason,
            self.snapshot.rearm_reason_flags,
        )

    @staticmethod
    def _float32(value: float) -> float:
        return struct.unpack("<f", struct.pack("<f", float(value)))[0]

    def on_disable_sent(self, sequence: int) -> None:
        self.machine.on_disable_sent(sequence)
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            wire_sent_sequence=int(sequence) & 0xFFFFFFFF,
            target_vx=0.0,
            target_wz=0.0,
            last_command_at=None,
            rearm_required=(
                self.snapshot.rearm_required
                or self.machine.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
            ),
        )

    def tick(self, now_sec: float) -> Tuple[bool, str]:
        if (
            self.snapshot.last_status_at is not None
            and now_sec - self.snapshot.last_status_at > self.status_timeout_sec
        ):
            self.machine.on_status_timeout()
            self.require_rearm(REARM_STATUS_TIMEOUT)
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=True,
                rearm_reason_flags=self.snapshot.rearm_reason_flags | REARM_STATUS_TIMEOUT,
            )
            return True, "status_timeout"
        if (
            self.snapshot.last_command_at is not None
            and now_sec - self.snapshot.last_command_at > self.command_timeout_sec
        ):
            self.machine.on_disable_command()
            self.require_rearm(REARM_COMMAND_TIMEOUT)
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=True,
                rearm_reason_flags=self.snapshot.rearm_reason_flags | REARM_COMMAND_TIMEOUT,
            )
            return True, "command_timeout"
        return False, ""
