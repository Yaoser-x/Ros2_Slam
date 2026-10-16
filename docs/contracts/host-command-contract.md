# Host command contract

Nav2, teleoperation, test and explicitly configured research sources publish candidate
Twists. `robot_control` validates, leases, limits and selects those candidates, then
publishes one `HostMotionCommand` on `chassis/host_motion_command`.

`command_epoch` identifies a Host activation generation. A new epoch is legal only
after Host disable/release and rearm. Within an epoch, `sequence` must advance modulo
2^32. Duplicate or out-of-order ROS commands do not refresh a lease. ROS header time is
an admission bound: stale or implausibly future commands are rejected before mutation.
After admission, lease freshness uses callback receipt on the Host monotonic clock, so
later `/clock` jumps cannot extend motion intent.

The bridge captures receipt in a dedicated callback group and passes an immutable
command through a bounded FIFO to its serialized state owner. Queue residence never
renews that lease; an expired receipt cannot enable motion. Overflow revokes control
and discards the entire pending batch. Disables, epoch changes and conflicting
duplicates retain their order, and safety/session changes discard pending input.

An invalid non-selected candidate is removed and diagnosed without interrupting the
selected source. If the selected source becomes invalid, arbitration runs immediately:
a fresh fallback remains enabled through the existing motion limiter, and Host release is
published once only when no valid fallback exists. A callback exception is never itself
authority to stop the complete vehicle.

Candidate `Twist` subscriptions use Reliable/Volatile/KeepLast(1). The final
`HostMotionCommand` link uses matched Reliable/Volatile/KeepLast(1) QoS with a 100 ms
deadline and 120 ms lifespan. The 50 ms publication period and all leases use monotonic
time. `HostControlState` exposes candidate/selected age plus DDS deadline, liveliness and
queue-loss counters; these counters are diagnostic facts, not physical stop evidence.

Bridge always maps every Host subsource to Upper-v3 mode 2/`COMMAND_SOURCE_HOST`.
The float32 wire target and disabled-to-enabled edge advance the wire sequence.
Fresh Host commands with unchanged targets advance it at least once per second;
pure retransmissions retain their identity and cannot outlive the firmware's two-second
hard lifetime.
