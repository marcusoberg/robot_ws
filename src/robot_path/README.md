# PAROL6 Cartesian path source

Start a Ready serial server and run `pixi run path-source`. The source starts its route at the measured TCP published in server status. Use `pixi run engage` to follow the line, triangle and circle, and `pixi run disengage` to pause.

The source publishes base-frame Cartesian targets on `/robot/pose_goal` with `position_only` set. This preserves the free tool orientation used by the tracing task. The server computes joint references from measured joint positions. Cartesian linear/angular rates in `path.toml` determine nominal path time. If the measured TCP lags, the route clock waits. A sustained stall reports the TCP error and pauses. Restart the source to generate a route from the new starting position.

`pixi run path-preview` previews Cartesian geometry and nominal timing using FK at the configured initial position. Reachability and joint motion are determined during tracking. The static route is published once on the transient-local `/path/markers` topic.
