from robot_data.recording import Recorder
from .simulation import SimulationEnv
from .task import PickPlaceSuccess
from .state import ServerState


class Application:
    def __init__(
        self, recordings, *, cameras=True, purpose="demonstrations", profile=None
    ):
        self.env = SimulationEnv(profile, cameras=cameras)
        self.recorder = Recorder(recordings, self.env.profile.spec, purpose=purpose)
        self.task = self.create_task()
        self.current = self.env.observation()
        self.profile = self.env.profile
        self.paced = False

    def start(self, paced):
        self.paced = paced
        if paced:
            self.env.native.start_paced()

    def poll(self):
        pass

    def state(self):
        snapshot = self.env.native.observe()
        status = snapshot.status
        return ServerState(
            mode=status.mode,
            instance=self.env.instance,
            episode=snapshot.episode,
            time_ns=snapshot.time,
            execution_time_ns=status.time,
            position=list(status.measured.position),
            velocity=list(status.measured.velocity),
            reference=list(status.reference),
            session=status.session,
            source=status.source,
            sequence=status.sequence,
            delivered_sequence=status.delivered_sequence,
            reason=status.reason,
            recording=bool(self.recorder.active),
            wall_lag_seconds=snapshot.wall_lag_seconds,
        )

    def submit(self, goal, session, sequence):
        if self.paced:
            self.env.native.submit(
                goal, session, sequence, self.profile.source_timeout_ns
            )
        else:
            self.step(goal, session=session, sequence=sequence)

    def activate(self, source):
        if not isinstance(source, str) or not source.strip():
            raise ValueError("A source identity is required")
        status = self.env.native.observe().status
        if status.session or status.mode == "Stopping":
            self.pause("source_switch")
        return self.env.activate(source)

    def step(self, goal, *, session=None, sequence=None):
        try:
            before = self.current
            after, snapshot = self.env.step(goal, session=session, sequence=sequence)
            self.recorder.append(before, goal, after, snapshot)
            self.current = after
            if self.task is not None:
                self.task.update(snapshot)
            return after, snapshot
        except Exception as exc:
            self.pause(f"action_source_error: {exc}")
            raise

    def pause(self, reason="paused"):
        # Stop first: draining recording I/O must never delay revoking control.
        status = self.env.pause(reason)
        self.recorder.interrupt(reason)
        self.current = self.env.observation()
        return status

    def command(self, command):
        if command == "record":
            return self.recorder.start(self.current)
        if command == "save":
            success = self.task.success if self.task is not None else False
            path = self.recorder.finish(complete=True, success=success)
            return {
                "episode": path,
                "success": success,
                "eligible_for_training": success,
            }
        if command == "discard":
            return self.recorder.finish(reason="operator_discard")
        if command == "reset":
            self.env.native.disable()
            self.recorder.interrupt("reset")
            self.current = self.env.reset()
            self.task = self.create_task()
            return "reset; activate a new session before submitting actions"
        if command == "pause":
            return self.pause().mode
        if command == "disable":
            self.env.native.disable()
            self.recorder.interrupt("disabled")
            return "Disabled"
        if command == "enable":
            self.env.native.enable()
            return "Ready"
        raise ValueError(f"Unknown application operation: {command}")

    def close(self):
        self.env.native.disable()
        self.recorder.close()
        self.env.close()

    def create_task(self):
        if self.env.profile.task == "pick_place":
            return PickPlaceSuccess(self.env.profile.cube_half_size)
        return None
