# Keyboard teleoperation

Run `ros2 run robot_teleop keyboard` in an interactive terminal. Enable and select `keyboard` using `robotctl` in another terminal. Deadman begins off. Number keys select a joint; `j/k` adjust its target; `d` toggles deadman; space toggles clutch; `q` exits.

This node directly publishes the common command vocabulary and producer heartbeat. Robot Server translates deadman/clutch transitions into a native controlled pause. No relay, SDK controller or automatic selection is involved.
