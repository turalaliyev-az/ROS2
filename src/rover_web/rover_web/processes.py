"""Start/stop the ROS launch processes behind each operating mode."""
import os
import signal
import subprocess
import threading
import time


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def _signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


class ProcessManager:

    def __init__(self, logs_dir, logger):
        self.logs_dir = logs_dir
        self.logger = logger
        self.procs = {}
        self._lock = threading.Lock()

    def start(self, name, cmd):
        with self._lock:
            self._stop_locked(name)
            log = open(os.path.join(self.logs_dir, f'{name}.log'), 'w')
            # Own session so a stop signal reaches the whole launch tree (launch
            # process, its nodes, the Nav2 container) and nothing else.
            self.procs[name] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True)
            log.close()
        self.logger.info(f'started {name}: {" ".join(cmd)}')

    def running(self, name):
        proc = self.procs.get(name)
        return proc is not None and proc.poll() is None

    def stop(self, name, timeout=15.0):
        with self._lock:
            self._stop_locked(name, timeout)

    def stop_all(self, timeout=8.0):
        """Stop every mode process at once (shutdown must fit launch's kill timeout)."""
        with self._lock:
            groups = []
            procs = list(self.procs.values())
            for proc in procs:
                proc.poll()
                pgid = proc.pid  # start_new_session: pgid == pid
                if _group_alive(pgid):
                    _signal_group(pgid, signal.SIGINT)
                    groups.append(pgid)
            self.procs.clear()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and any(_group_alive(g) for g in groups):
                for proc in procs:
                    proc.poll()  # reap exited launch processes so they don't count as alive
                time.sleep(0.1)
            for pgid in groups:
                _signal_group(pgid, signal.SIGKILL)

    def _stop_locked(self, name, timeout=15.0):
        proc = self.procs.pop(name, None)
        if proc is None:
            return
        proc.poll()
        pgid = proc.pid  # start_new_session: pgid == pid
        if not _group_alive(pgid):  # the launch may be gone while its nodes live on
            return
        # SIGINT first so nodes shut down cleanly; escalate only for a process
        # tree that hangs (the Nav2 container has been seen to).
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            _signal_group(pgid, sig)
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                proc.poll()  # reap the launch process so a zombie doesn't look alive
                if not _group_alive(pgid):
                    return
                time.sleep(0.1)
        self.logger.warning(f'{name} did not exit cleanly')
