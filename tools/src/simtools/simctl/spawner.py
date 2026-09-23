import queue
import threading
from collections.abc import Callable
from typing import Any


class Spawner:
    """Runs callables on one thread that lives as long as the backend."""

    def __init__(self) -> None:
        self._jobs: queue.Queue[tuple[Callable[[], Any], queue.Queue[tuple[bool, Any]]] | None] = (
            queue.Queue()
        )
        self._thread = threading.Thread(target=self._loop, name="spawner", daemon=True)
        self._thread.start()

    def run(self, job: Callable[[], Any]) -> Any:
        """Executes job on the spawner thread and returns its result; exceptions propagate."""
        reply: queue.Queue[tuple[bool, Any]] = queue.Queue()
        self._jobs.put((job, reply))
        ok, value = reply.get()
        if ok:
            return value
        raise value

    def close(self) -> None:
        self._jobs.put(None)
        self._thread.join(timeout=5.0)

    def _loop(self) -> None:
        while True:
            item = self._jobs.get()
            if item is None:
                return
            job, reply = item
            try:
                reply.put((True, job()))
            except BaseException as error:
                reply.put((False, error))
