# Recording Tools

This Python library writes canonical MCAP captures/events, reads previous recordings, checks alignment and exports train/validation datasets. It has no ROS or SDK dependency. The ROS package `src/robot_recorder` owns live subscriptions, recording sessions and episodes.

`python -m recording_tools.cli --run RUN --config CONFIG --alignment-only` inspects approved aligned segments without LeRobot or writing a dataset. Configuration contains `joints`, `period`, and a `cameras` mapping from model feature names to recorded camera names. JSON and YAML are accepted.

Remove `--alignment-only` and provide `--output DATASET` to export through the optional LeRobot 0.6.0 adapter in `environments/models`. Export requires a complete manifest, fixed simulation intervals and at least two independent approved episodes/recovery groups. Related recoveries stay in the same split. Joint input is the measured state at the frame time; action supervision is the accepted actuator reference at the following interval.

Original capture identities, acquisition timestamps, clock mappings, calibration and event IDs remain available. Legacy MessagePack and current canonical JSON recording envelopes are supported. Irregular live resampling and physical camera transport remain unqualified.
