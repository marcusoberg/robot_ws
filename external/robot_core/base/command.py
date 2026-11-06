"""Backend contract for submitting a robot action from Python."""

from abc import ABC, abstractmethod
from concurrent.futures import Future
from typing import Generic, TypeVar


Command = TypeVar("Command")


class CommandAction(ABC, Generic[Command]):
    @abstractmethod
    def execute(self, command: Command) -> Future[int]:
        """Submit a command. Its future completes with the backend command index."""
