# Commands on feature/python-api

| Task | Action |
| --- | --- |
| `pixi run server-parol6-serial` | Start the serial ROS node and owned API controller |
| `pixi run server-parol6-serial restart:=true` | Fully restart the serial server |
| `pixi run server-stop` | Stop the serial server and its API controller |
| `pixi run server` | Run the inactive simulation placeholder |
| `pixi run home` | Ask the API to home; wait for completion |
| `pixi run enable` / `disable` | Clear / latch the API protective stop |
| `pixi run pose` / `standby` | Timed joint moves through the API |
| `pixi run teleop` | Start SO101 → PAROL6 teleop, initially paused, with the follower model and TF. Use `visualize:=false` with a running standalone visualization. |
| `pixi run engage` / `disengage` | Activate / pause the current source |
| `pixi run path-source` | Generate the route, then stream Cartesian targets after engagement |
| `pixi run visualize-parol6` / `foxglove` | Start the separate visualization nodes / bridge |

The placeholder publishes no status and does not execute these commands. Recording and simulation-based replay/evaluation are deferred; the old implementation is on `main`.
