from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque


class ObservationBuffer:
    def __init__(self, maxlen: int = 1024):
        self._items: Deque[dict] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._first_wall_time: float | None = None
        self._last_wall_time: float | None = None

    def push(self, obs: dict) -> None:
        now = time.time()
        with self._lock:
            if self._first_wall_time is None:
                self._first_wall_time = now
            self._last_wall_time = now
            self._items.append(obs)
            self._event.set()

    def wait_for_item(self, timeout_sec: float) -> bool:
        return self._event.wait(timeout=timeout_sec)

    def last(self) -> dict | None:
        with self._lock:
            if not self._items:
                return None
            return self._items[-1]

    def all(self) -> list[dict]:
        with self._lock:
            return list(self._items)

    def clear(self) -> None:
        with self._lock:
            self._items.clear()
            self._first_wall_time = None
            self._last_wall_time = None
            self._event.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)

    def stats(self) -> dict:
        with self._lock:
            count = len(self._items)
            first = self._first_wall_time
            last = self._last_wall_time
        duration = 0.0 if first is None or last is None else max(0.0, last - first)
        fps = 0.0 if duration <= 0.0 or count < 2 else (count - 1) / duration
        return {
            "count": count,
            "wall_duration_sec": duration,
            "fps": fps,
        }
