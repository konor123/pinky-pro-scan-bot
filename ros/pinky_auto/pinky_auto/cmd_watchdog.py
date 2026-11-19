class CmdWatchdog:
    """Decides when the motors must be stopped because velocity commands ceased."""

    def __init__(self, timeout):
        if timeout <= 0:
            raise ValueError('timeout must be positive')
        self.timeout = timeout
        self._last_cmd = None
        self._stopped = True

    def feed(self, now):
        self._last_cmd = now
        self._stopped = False

    def should_stop(self, now):
        if self._stopped or self._last_cmd is None:
            return False
        return now - self._last_cmd > self.timeout

    def mark_stopped(self):
        self._stopped = True
