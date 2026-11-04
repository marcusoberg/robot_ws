"""Own the upstream PAROL6 controller and its async client behind the ROS server."""

import asyncio
from concurrent.futures import Future
import threading
import time

from parol6 import AsyncRobotClient, Robot
from parol6.config import HOME_ANGLES_DEG


class Parol6Controller:
    def __init__(self, serial_port, port):
        self.robot = Robot(host='127.0.0.1', port=port, timeout=30.)
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
            self.client = AsyncRobotClient(host='127.0.0.1', port=self.port)
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

    def home(self):
        return self.submit(self.client.home(wait=True, timeout=100.))

    def move(self, degrees, duration):
        return self.submit(self.client.move_j(degrees, duration=duration, wait=True, timeout=duration + 15.))

    def servo(self, degrees):
        return self.submit(self.client.servo_j(degrees))

    def stop(self):
        return self.submit(self.client.stop())

    def disable(self):
        return self.submit(self.client.estop())

    def enable(self):
        return self.submit(self.client.reset())

    async def shutdown(self):
        try:
            await self.client.stop()
        finally:
            await self.client.close()
            await self.reader

    def close(self):
        try:
            self.submit(self.shutdown()).result(timeout=5.)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join()
            self.robot.stop()
