"""[공통] 스캔 PC의 모든 기다림과 한도가 쓰는 시계."""
import contextlib
import threading
import time


class Clock:
    """time_scale배 빠르게 가는 시계(가짜 에이전트 시험용). `hold()` 동안은 멈춘다."""

    def __init__(self, time_scale=1.0):
        self.time_scale = float(time_scale)
        self._t0 = time.monotonic()
        self._lock = threading.Lock()
        self._held = 0
        self._held_at = 0.0
        self._paused = 0.0

    def _raw(self):
        return (time.monotonic() - self._t0) * self.time_scale - self._paused

    def now(self):
        with self._lock:
            return self._held_at if self._held else self._raw()

    def sleep(self, s, cancel=None):
        end = self.now() + s
        while True:
            left = end - self.now()
            if left <= 0 or (cancel is not None and cancel.is_set()):
                return
            time.sleep(min(left / self.time_scale, 0.02))

    @contextlib.contextmanager
    def hold(self):
        with self._lock:
            if self._held == 0:
                self._held_at = self._raw()
            self._held += 1
        try:
            yield
        finally:
            with self._lock:
                self._held -= 1
                if self._held == 0:
                    self._paused += self._raw() - self._held_at
