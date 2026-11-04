# PAROL6 serial execution

## Operator sequence

Close Commander and other serial clients. Use one robot server in this ROS domain.
Rerunning the server task attaches a read-only terminal to an existing matching
server instead of opening another serial connection. Missing visual publishers
are restored. Ctrl-C in an attached terminal detaches it; `pixi run disable`
still disables the arm. When the launch owns a newly started server, Ctrl-C shuts
that server down. If the owned server exits, its visualization children exit too.
An attachment ends if the server disappears; rerunning then starts a new disabled
connection. If live telemetry still reports all six joints homed with valid counters,
`pixi run enable` restores holding without another homing cycle.

```sh
pixi run server-parol6-serial serial_port:=/dev/cu.usbmodemYOUR_DEVICE tool_isolated:=true
```

The backend opens that port exclusively, polls disabled, and waits in `Idle`.
With the arm supported, press and release E-stop once so the backend can verify
that it sees both input levels. Then, in another terminal:

```sh
pixi run home
```

This command waits for completed homing, not just command acceptance. Ctrl-C
requests disable. On completion the arm remains enabled and holding in `Ready`.

```sh
pixi run path-source
pixi run engage
```

The producer owns geometry, IK and the requested duration. For serial execution,
`pose`, `standby` and the first teleop move upload a quintic joint trajectory;
`path-source` uploads its complete timed joint path. The native serial loop starts
the trajectory on its first dispatch and evaluates position and its derivative
together at each write. Subsequent producer heartbeats renew ownership without
restarting the trajectory. Upload retries use a motion id and are idempotent.
Dense path knots do not increase the serial packet rate. Path interpolation and
requested durations are unchanged; there is no additional speed cap or retiming.

Live teleop uploads each completed IK target. The existing causal two-stage
smoothing now runs on that same native clock, retaining position and velocity
when a new target arrives. It needs no future IK samples and inserts no stops at
sample boundaries. Simulation continues using its sampled-position interface.
Teleop uses the ROS-free `robot_kinematics` library's Pinocchio frame-Jacobian
IK. Each target starts from the previous joint solution, uses damped Jacobian
updates and enforces URDF joint bounds. Traces include convergence, iteration
count and residual pose error. The old SciPy optimizer, posture anchor and
singularity-condition strategy settings have been removed. The commanded start
pose, four-second move and native following-error threshold are unchanged.
The serial loop uses an absolute 10 ms schedule; late wake-ups do not accumulate
or cause catch-up bursts. Completion still requires endpoint counter feedback
before requesting a hold. Reported angles are software step-count estimates.

After installing this version, stop old sources and restart the server to load
the native library and changed ROS message types:

```sh
pixi run server-parol6-serial tool_isolated:=true restart:=true
# In another terminal, when the controller retains its homed position:
pixi run enable
```

`pixi run pause` holds an already homed arm with motors enabled. Pausing during
homing must disable: stock command 255 would continue homing. `pixi run disable`,
E-stop, communication faults and shutdown disable motors. `pixi run reset` now
recovers software command faults in place and restores holding when the controller's
homed flags and counters remain valid. After a server restart, use `pixi run enable`.
E-stop during enabled operation, lost homing flags, invalid counters and communication failures invalidate
the position on the current connection; these still require homing. Releasing
E-stop never restarts motion automatically. Counters are not encoders: moving the
arm while disabled, or losing motor power while USB keeps the controller alive,
can invalidate physical alignment without changing the reported counters.
An E-stop input cycle while already disabled does not itself invalidate stored
homing. Enabling remains blocked until the live input is released. Command
rejections report their current reason separately from any latched fault.

## Command protocol and Commander comparison

Reviewed local sources:

- `testing/PAROL-commander-software/GUI/files/Serial_sender_good_latest.py`:
  `dummy_data`, `Pack_data`, `Unpack_data`, E-stop handling in the sender loop.
- `testing/PAROL6-firmware-reference/PAROL6 control board main software/src/`:
  `main.cpp` dispatch, `Pack_data`, `home_all`; `motor_init.cpp` gearing and
  standby positions; `structs.h` defaults.
- `testing/firmware_diagnostics/README.md`: documented stock telemetry limitations.

Firmware reference revision: `1e7fa6391440660a5c4ce2166e8a33e7e3d334cf`.

| Operation | Wire behavior |
| --- | --- |
| Poll disabled | 102 repeatedly |
| Begin homing | 101, then 100 once, then 255 while waiting |
| Enable retained position | Require live homed flags and valid counters; 101 then zero-velocity 255, without 100 |
| Homing complete | Require newly observed unhomed flags, then all six homed bits and expected standby counters stable over ten replies; continue enabled with zero-velocity 255 |
| Joint target | 156, six signed 24-bit big-endian counters and optional feed-forward speeds in steps/s |
| Normal hold | 255, current counters and zero velocities, matching Commander `dummy_data` |
| E-stop or fault | Latch fault and repeat 102; never auto-enable/reconnect |

Packets use 3 Mbaud, raw 8N1, DTR asserted and RTS cleared. Commands are 56 bytes
including header; responses are 60 bytes. The native worker polls at 100 Hz even
when Python is busy. E-stop is response IO bit 3, active low. The worker watches
it during homing, holding and motion. The echoed command is sent before firmware
dispatch; it is not a completion acknowledgement. Stock's CRC byte is a constant,
not a checksum. Homing flags/counters determine completion; neither proves actual
mechanical alignment without functioning sensors and mechanics.

OUT1/OUT2 and gripper fields remain zero. This implements Commander's arm-control
wire protocol; it does not implement its full GUI, gripper or output interface.
The explicit tool-isolation argument is required because outputs can change a tool.

## Command and feedback time

The feedback packet echoes the newly received opcode, but contains counters
sampled before that command is applied. The native worker records each packet's
trajectory evaluation time, write start/end and reply completion. It estimates
feedback sampling/command arrival at the exchange midpoint, and associates the
feedback with the previous transmitted curve. It evaluates that curve at the
corresponding point in its execution interval, accounting for the previous
packet's estimated delivery delay. It does not compare against a newly received
ROS goal, freeze the reference at the previous waypoint, or extrapolate the
reported counter to manufacture a current measurement.

The 0.03 rad following-error check runs on each returned sample against that
reference. Incoming goals still undergo ownership, sequence, bounds, continuity
and wire-range checks. Homing, E-stop and command expiry remain independent.

`/joint_states`, `/robot/reference` and `/robot/tracking_error` share the sample's
estimated capture stamp. Every native sample is published once, even when ROS
drains several samples together. Status remains at 20 Hz. The tracking topic now
uses `robot_server/msg/TrackingError`, retaining the `data` array and adding a
header, reference-valid flag and transport timing uncertainty. Reconnect Foxglove
after restarting; keep the plot path `/robot/tracking_error.data[:]` and select
**Header stamp** as its timestamp source, so ROS publication batches are not
plotted as if they were simultaneous measurements.

The firmware has no sample timestamp or applied-command sequence. Half the
host round-trip time is reported as **transport** timing uncertainty; firmware
snapshot/dispatch latency remains unknown and is explicitly identified in
`/robot/serial_telemetry`. This is host-side alignment, not exact hardware clock
synchronization. Exact synchronization needs firmware support. Feedback age,
packet timing and following error are reported separately. Fault traces include
every serial sample and its transmitted position/velocity and matched reference.

## Code ownership and failure handling

- `parol6_serial.cpp`: port setup, packet exchange, telemetry decoding, native
  motion states, E-stop/freshness/lease monitoring, and disable on IO failure.
- `serial_backend.py`: ROS-independent application ownership and operator commands.
- `robot_server_node/node.py`: message validation and command-result reporting.
- `robot_server_node/control.py`: operator request/acknowledgement/completion waits.
- Producers own geometry and IK; the server has no path planning.

Static configuration is validated before opening the port. Dynamic safety checks
remain at incoming command/telemetry boundaries: ownership, ordering, finite values,
joint limits, tracking error and E-stop cannot be checked just once at startup.
Expected native command rejection returns a result, not an exception. Low-level IO
exceptions are caught by the polling worker, which stays alive sending disable.
Unexpected ROS failures enter shutdown cleanup. Fault transitions are logged once.
The serial application keeps a bounded command/feedback history in memory and
writes it to `/private/tmp/parol6-server-*.json` on a fault. Teleop similarly saves
IK results and sent commands to `/private/tmp/parol6-teleop-*.json`. Fault logs
include the paths. Timestamps use the same host monotonic clock, allowing send
delays, receive gaps, feedback age and serial-cycle timing to be compared.
A local per-domain process lock prevents two updated servers owning `/robot` topics.

Motion ownership expires after 250 ms without a valid goal or motion heartbeat.
Live targets separately expire after 300 ms without a new target; heartbeats
cannot extend that target deadline. Homing and idle holding
require the server heartbeat within 250 ms. A native polling gap or missing reply
of 100 ms faults; homing has a 90-second limit. A physical E-stop report therefore
has USB/host polling latency. The firmware does not independently enforce E-stop
or communication loss: no host implementation can guarantee stopping after USB
loss, host death or OS suspension. Direct power isolation remains necessary.

## Verification scope

The native-timing and retained-position recovery changes were compiled but not
tested on hardware or in replay. The checks below describe the earlier implementation.

Native build and temporary pseudo-terminal firmware replay checked completed
homing with enabled hold, pause/resume without re-enable, E-stop in homing/Ready/
Running, producer and server lease expiry, malformed replies, missing replies,
invalid targets, and repeated disable after faults. The replay models reply-before-
dispatch ordering. No physical port was opened and no arm moved during this audit.
Physical operation and the installed firmware version remain unverified; the stock
protocol has no version query. Placeholder driver error bits are not live health.
