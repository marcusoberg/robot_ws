"""Own the upstream PAROL6 controller and its async client behind the ROS server."""

import asyncio
from concurrent.futures import Future
import threading
import time

import numpy as np
from parol6 import AsyncRobotClient, Robot
from parol6.config import HOME_ANGLES_DEG

from robot_core.config import robot_settings
from robot_core.validation import timed_move
from robot_core.base.command import CommandAction
from robot_core.data.commands import MoveJoint


class PythonApiController(CommandAction[MoveJoint]):
    def __init__(self, serial_port, port):
        self.hardware = robot_settings()['hardware']
        self.robot = Robot(host=self.hardware['api_host'], port=port, timeout=self.hardware['connect_timeout'])
        self.robot.start(com_port=serial_port, extra_env={'PAROL6_NOAUTOHOME': '1'})
        self.standby_degrees = HOME_ANGLES_DEG
        self.loop = asyncio.new_event_loop()
        self.ready = Future()
        self.snapshot = None
        self.failure = None
        self.port = port
        self.thread = threading.Thread(target=self.run, name='parol6-api')
        self.thread.start()
        try:
            self.ready.result()
        except Exception:
            self.thread.join()
            self.loop.close()
            self.robot.stop()
            raise

    def run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.client = AsyncRobotClient(host=self.hardware['api_host'], port=self.port)
        except Exception as error:
            self.ready.set_exception(error)
            return
        self.reader = self.loop.create_task(self.receive())
        self.ready.set_result(None)
        self.loop.run_forever()
        pending = asyncio.all_tasks(self.loop)
        for task in pending:
            task.cancel()
        self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        self.loop.close()

    async def receive(self):
        try:
            async for status in self.client.stream_status():
                self.snapshot = (status, time.monotonic())
        except Exception as error:
            self.failure = error

    def submit(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    async def _command(self, request, *, indexed=False):
        result = await request
        if (indexed and result < 0) or (not indexed and result != 1):
            raise RuntimeError(f"PAROL6 API rejected {request.__name__} (result {result})")
        return result

    def home(self):
        request = self.client.home(wait=True, timeout=self.hardware['home_timeout'])
        return self.submit(self._command(request, indexed=True))

    def execute(self, command: MoveJoint) -> Future[int]:
        timed_move(command)
        request = self.client.move_j(np.rad2deg(command.angles).tolist(), duration=command.duration,
                                     wait=True, timeout=command.duration + self.hardware['move_timeout_margin'])
        return self.submit(self._command(request, indexed=True))

    def servo(self, degrees):
        return self.submit(self._command(self.client.servo_j(degrees)))

    def stop(self):
        return self.submit(self._command(self.client.stop()))

    def disable(self):
        return self.submit(self._command(self.client.estop()))

    def enable(self):
        return self.submit(self._command(self.client.reset()))

    async def shutdown(self):
        try:
            await self.client.stop()
        finally:
            await self.client.close()
            await self.reader

    def close(self):
        try:
            self.submit(self.shutdown()).result(timeout=self.hardware['shutdown_timeout'])
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join()
            self.robot.stop()
