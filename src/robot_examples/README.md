# Example producers and operator commands

`robotctl` invokes explicit Robot Server services. It prints a queued receipt; inspect `/robot/result` for the native outcome.

`send_joint` advertises `example` and publishes a target only while selected. `--finite` sends one two-second native trajectory so admission and completion can be observed. Without that option it streams a position target.

`send_timed_targets` advertises `timed_example` and sends one raw timestamped chunk for native preparation. It does no fitting or interpolation.

`boundary_probe` freezes a valid envelope without sending it. The operator can reset or change selection before explicitly publishing that frozen envelope. `--duplicate` publishes once before replay.

All publishers use the same ROS messages and heartbeat rules. None enables or selects itself. The Robot Server package owns `robot_server_client`, which provides shared ROS envelope and source bookkeeping. Production producers do not depend on this examples package.
