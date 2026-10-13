# Ownership, timing and captures

Start the realtime robot as in tutorial 1. In another terminal:

```sh
ros2 run robot_teleop keyboard
```

Enable, then `robotctl select keyboard`. Number keys choose a joint; `d` toggles the deadman, `j/k` change its target, space toggles clutch, and `q` exits. Deadman begins off. Toggle it off or engage clutch and inspect the transition to HOLD. Heartbeats retain healthy ownership while demands pause.

Start a second producer and use `robotctl takeover example` to explicitly change ownership. Kill the selected producer and inspect braking/authority loss after its latest heartbeat lease expires (these producers request 300 ms). It must not reacquire itself.

To inspect raw timed targets:

```sh
ros2 run robot_examples send_timed_targets
# Another terminal:
ros2 run robot_examples robotctl enable
ros2 run robot_examples robotctl select timed_example
```

The example publishes positions and timestamps, without derivatives. Add `--replace` to propose one replacement after actual admission. Run a fresh selected producer with `--origin-age 5` to preserve an old chunk whose full four-second span ends before the future start; expect rejection rather than shifted predictions. In stepped mode explicitly advance enough intervals for that old origin to exist. Inspect the native fitting result and accepted schedule. Rejected proposals must leave the accepted path intact.

For explicit stepping, stop the launch and restart with `mode:=stepped`. Control requests remain queued until you explicitly call:

```sh
ros2 run robot_examples robotctl step --n 1
ros2 run robot_examples robotctl capture
ros2 run robot_examples robotctl reset
```

Enable → step → select → step is the stepped control sequence. Each step advances one configured action interval and returns a capture identity. Capture alone does not advance execution. Reset disables and invalidates the old timeline and sessions.

Run `pixi run smoke capture` to save a read-only live capture report. Run `pixi run smoke fitting` for native/SciPy fitting comparison. These commands are for you to run; migration does not execute them.

## Rejected frozen envelopes

Run this publisher in its own terminal, then enable and select `boundary_example`:

```sh
ros2 run robot_examples boundary_probe
```

Once it prints its frozen envelope, call `robotctl reset` in another terminal. Return to the publisher and press Enter. Expect a rejected old timeline and no restarted motion. Repeat after explicit takeover to check that the old native session is rejected. A paused stepped server also requires a step after each queued control action.

For duplicate request checks, run `boundary_probe --duplicate`, select it, and inspect its first result. Hold the robot, then press Enter. The repeated request may return its cached original result; it must never enqueue another native motion or acquire authority. Ctrl-C stops this publisher.

Record your observations in a new run directory, with topic logs if useful:

```sh
mkdir -p runs/my-ownership
ros2 topic echo /robot/result > runs/my-ownership/results.yaml
# Stop topic echo with Ctrl-C after the checks.
```
