# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Launching and stopping the nodes a phase needs.

Phase 3 needs a ``route_recorder``; Phases 5 and 6 need a ``route_follower``.
Originally the dashboard only *watched* for them and the operator started both
by hand, which produced two bad outcomes in practice:

* A teach pass that recorded nothing. Pressing Start on Phase 3 began
  evaluating gates but launched no recorder, so Stop had nothing to save. The
  drive was lost, silently, and the panel showed green predictions the whole
  way -- it was predicting what a recorder *would* have written.
* An autonomous run that followed the wrong route, because the follower was
  launched by hand against whatever path happened to be on disk.

So the dashboard manages them. The rule it follows is that **recording is
safe and driving is not**:

* the recorder is managed by default -- it only reads topics and writes a file
* the follower is opt-in, because launching it MOVES THE ROBOT. In the field
  that has to stay a deliberate act by someone holding a kill switch, so the
  alley and driveway profiles leave ``manage_follower`` false and the sim
  profile turns it on.

Children are spawned into their own process group and torn down by group.
Signalling ``ros2 launch`` alone does not reliably reach what it spawned, and
a surviving follower keeps publishing ``/cmd_vel``.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from typing import List, Optional


class ManagedProcess:
    """One child process group, started and stopped by the dashboard."""

    #: How long a straggler may linger after the process we signalled has
    #: itself exited. Short on purpose -- at that point it is an orphan.
    STRAGGLER_GRACE_S = 1.5

    def __init__(self, name: str, command: List[str], log_path: str,
                 logger=None) -> None:
        self.name = name
        self.command = list(command)
        self.log_path = log_path
        self._log = None
        self._proc: Optional[subprocess.Popen] = None
        self._logger = logger

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def returncode(self) -> Optional[int]:
        return None if self._proc is None else self._proc.poll()

    def start(self) -> bool:
        if self.running:
            return True
        try:
            self._log = open(self.log_path, 'w', encoding='utf-8')
            self._proc = subprocess.Popen(
                self.command, stdout=self._log, stderr=subprocess.STDOUT,
                start_new_session=True)
            return True
        except Exception as exc:                      # noqa: BLE001
            if self._logger:
                self._logger.error(f'could not start {self.name}: {exc}')
            self._close_log()
            self._proc = None
            return False

    def stop(self, timeout: float = 10.0) -> None:
        """SIGINT the whole group, then SIGKILL whatever is still there.

        Waiting on the direct child is not enough. ``ros2 launch`` exits as
        soon as it has asked its own children to go, and a backgrounded
        grandchild may ignore SIGINT entirely (POSIX gives background jobs of
        a non-interactive shell an ignored SIGINT). Both cases leave the
        group populated after ``wait()`` has happily returned -- and a
        surviving route_follower goes on publishing ``/cmd_vel`` to a robot
        that nothing is watching. So the group is polled until it is really
        empty, and killed hard if it is not.
        """
        if self._proc is None:
            self._close_log()
            return
        try:
            pgid = os.getpgid(self._proc.pid)
        except OSError:
            pgid = None

        if self._proc.poll() is None and pgid is not None:
            try:
                os.killpg(pgid, signal.SIGINT)
                self._proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
            except Exception:                         # noqa: BLE001
                pass

        # The thing we asked to stop has stopped. Anything still in the group
        # is an orphan -- `ros2 launch` exits before its children are all
        # gone, and a backgrounded grandchild may have ignored the SIGINT
        # outright. Give stragglers a short grace, then take them out: the
        # cost of waiting is a robot still being driven by a node the
        # dashboard believes it has already shut down.
        if pgid is not None:
            self._reap_group(pgid, grace=self.STRAGGLER_GRACE_S)

        self._proc = None
        self._close_log()

    def _reap_group(self, pgid: int, grace: float) -> None:
        """Wait briefly for the group to empty, then SIGKILL what remains."""
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if not self._group_alive(pgid):
                return
            time.sleep(0.05)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except OSError:
            return
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and self._group_alive(pgid):
            time.sleep(0.05)
        if self._group_alive(pgid) and self._logger:
            self._logger.error(
                f'{self.name}: process group {pgid} survived SIGKILL')

    @staticmethod
    def _group_alive(pgid: int) -> bool:
        """Signal 0 probes the group without touching it."""
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except OSError:
            return False

    def tail(self, lines: int = 12) -> str:
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as fh:
                return ''.join(fh.readlines()[-lines:])
        except OSError:
            return ''

    def _close_log(self) -> None:
        if self._log is not None:
            try:
                self._log.close()
            except OSError:
                pass
            self._log = None
