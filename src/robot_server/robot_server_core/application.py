from robot_data.recording import Recorder
from .simulation import SimulationEnv
from .task import PickPlaceSuccess

class Application:
    def __init__(self, recordings, *, cameras=True, purpose="demonstrations", profile=None):
        self.env = SimulationEnv(profile, cameras=cameras)
        self.recorder = Recorder(recordings,self.env.profile.spec,purpose=purpose)
        self.task = PickPlaceSuccess(self.env.profile.cube_half_size)
        self.current = self.env.observation()

    def activate(self, source):
        if not isinstance(source,str) or not source.strip():
            raise ValueError("A source identity is required")
        status = self.env.native.observe().status
        if status.session or status.mode == "Stopping":
            self.pause("source_switch")
        return self.env.activate(source)

    def step(self, goal, *, session=None, sequence=None):
        try:
            before = self.current
            after,snapshot = self.env.step(goal, session=session, sequence=sequence)
            self.recorder.append(before,goal,after,snapshot)
            self.current = after
            self.task.update(snapshot)
            return after,snapshot
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
            path=self.recorder.finish(complete=True,success=self.task.success)
            return {"episode":path,"success":self.task.success,"eligible_for_training":self.task.success}
        if command == "discard":
            return self.recorder.finish(reason="operator_discard")
        if command == "reset":
            self.env.native.disable()
            self.recorder.interrupt("reset")
            self.current = self.env.reset()
            self.task = PickPlaceSuccess(self.env.profile.cube_half_size)
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
