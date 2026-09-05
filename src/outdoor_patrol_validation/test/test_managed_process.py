# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Tests for the child-process manager behind Phase 3 and Phases 5/6."""

import os
import tempfile
import time

from outdoor_patrol_validation.managed_process import ManagedProcess


def _proc(command, name='child'):
    log = tempfile.NamedTemporaryFile(delete=False, suffix='.log')
    log.close()
    return ManagedProcess(name, command, log.name)


def test_starts_and_reports_running():
    p = _proc(['sleep', '30'])
    assert not p.running
    assert p.start()
    assert p.running
    p.stop()
    assert not p.running


def test_stop_is_idempotent_and_safe_before_start():
    p = _proc(['sleep', '30'])
    p.stop()                    # never started
    assert p.start()
    p.stop()
    p.stop()                    # already stopped
    assert not p.running


def test_stop_kills_the_whole_process_group():
    """A shell that spawns a child must not leave the child behind.

    This is the reason for start_new_session + killpg: signalling `ros2
    launch` alone does not reliably reach what it spawned, and a surviving
    route_follower keeps publishing /cmd_vel after the dashboard thinks it
    has stopped -- a robot driving with nothing watching it.
    """
    marker = tempfile.NamedTemporaryFile(delete=False, suffix='.marker')
    marker.close()
    os.unlink(marker.name)
    # The grandchild writes the marker only if it survives 3 s.
    p = _proc(['sh', '-c', f'(sleep 3; touch {marker.name}) & sleep 30'])
    assert p.start()
    time.sleep(0.5)
    p.stop()
    time.sleep(4)
    assert not os.path.exists(marker.name), 'grandchild survived the group kill'


def test_a_failed_command_reports_false_and_does_not_raise():
    p = _proc(['/nonexistent/definitely-not-a-binary'])
    assert p.start() is False
    assert not p.running


def test_returncode_is_visible_after_the_child_exits():
    p = _proc(['sh', '-c', 'exit 7'])
    assert p.start()
    for _ in range(100):
        if p.returncode is not None:
            break
        time.sleep(0.05)
    assert p.returncode == 7
    assert not p.running
    p.stop()


def test_tail_returns_child_output():
    p = _proc(['sh', '-c', 'echo hello-from-child; sleep 0.2'])
    assert p.start()
    time.sleep(0.8)
    assert 'hello-from-child' in p.tail()
    p.stop()


def test_tail_of_a_missing_log_is_empty_not_an_error():
    p = ManagedProcess('x', ['true'], '/nonexistent/dir/none.log')
    assert p.tail() == ''
