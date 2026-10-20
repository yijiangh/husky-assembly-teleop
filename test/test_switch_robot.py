"""The 'Switch to <robot>' buttons (HuskyMonitor.switch_robot and the restart in main).

Each test stands in for the monitor with a SimpleNamespace and calls the real
methods on it. Nothing restarts: os.execv and the 'ros2 daemon stop' call are
replaced. husky_monitor imports the ROS stack, so these tests skip when it is
not sourced.
"""

import sys
from types import SimpleNamespace

import pytest

husky_monitor = pytest.importorskip('husky_assembly_teleop.husky_monitor')
from husky_assembly_teleop.robot_registry import robot_by_name  # noqa: E402

HuskyMonitor = husky_monitor.HuskyMonitor


class _Log:
    """Collects what the monitor logs, by level."""

    def __init__(self):
        self.lines = []

    def info(self, text: str):
        self.lines.append(('info', text))

    def warn(self, text: str):
        self.lines.append(('warn', text))


def _monitor(connected: str = 'Cindy', *, tasks=(), arm_executing=(False, False)) -> SimpleNamespace:
    """A stand-in monitor connected to ``connected``, with the real task guard.

    Args:
        connected (str): The robot this run drives.
        tasks (Sequence): Queued tasks (non-empty = a step runs or waits).
        arm_executing (Sequence[bool]): The interface's per-arm moving flags.

    Returns:
        SimpleNamespace: The stand-in.
    """
    log = _Log()
    mon = SimpleNamespace(
        _switch_to=None, tasks=list(tasks), _running_task=None, _skeleton=object(),
        huskies=[SimpleNamespace(interface=SimpleNamespace(is_arm_executing=list(arm_executing)))],
        selected_robot_id=0, log=log)
    mon.get_logger = lambda: log
    mon._connected_robot = lambda: robot_by_name(connected)
    mon._refuse_while_tasks_run = lambda what, **kw: HuskyMonitor._refuse_while_tasks_run(mon, what, **kw)
    return mon


def test_switch_sets_the_flag_for_the_other_robot():
    """A click on another robot's button only sets the flag (update() acts on it)."""
    mon = _monitor('Cindy')
    HuskyMonitor.switch_robot(mon, 'Alice')
    assert mon._switch_to == robot_by_name('Alice')
    assert any('restarting the monitor on Alice (domain 84)' in t for _, t in mon.log.lines)


def test_current_robot_button_does_nothing():
    """'Switch to Cindy (current)' on Cindy only logs."""
    mon = _monitor('Cindy')
    HuskyMonitor.switch_robot(mon, 'Cindy')
    assert mon._switch_to is None
    assert mon.log.lines == [('info', '[Switch] already on Cindy (domain 86); nothing to do.')]


def test_refused_while_a_step_runs_or_waits():
    """A queued task (running or waiting for Confirm Exec) blocks the switch."""
    mon = _monitor('Cindy', tasks=[object()])
    HuskyMonitor.switch_robot(mon, 'Belle')
    assert mon._switch_to is None
    assert any(level == 'warn' and 'refused' in t for level, t in mon.log.lines)


def test_refused_while_an_arm_moves():
    """An arm that still moves (sent directly, not as a task) blocks the switch."""
    mon = _monitor('Alice', arm_executing=[True])
    HuskyMonitor.switch_robot(mon, 'Cindy')
    assert mon._switch_to is None
    assert any('an arm is still moving' in t for _, t in mon.log.lines)


def test_update_ends_the_spin_with_restart(monkeypatch):
    """With the flag set, update() raises RestartMonitor carrying the robot."""
    monkeypatch.setattr(husky_monitor._common, '_global_backend', None)
    mon = _monitor('Cindy')
    mon._switch_to = robot_by_name('Belle')
    with pytest.raises(husky_monitor.RestartMonitor) as raised:
        HuskyMonitor.update(mon)
    assert raised.value.spec.name == 'Belle'


def test_restart_runs_the_same_command_with_the_new_domain(monkeypatch):
    """_restart_monitor_for: the new domain in the env, the same python + argv execed."""
    calls = {}
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    monkeypatch.setattr(husky_monitor.subprocess, 'run', lambda cmd, **kw: calls.setdefault('run', cmd))
    monkeypatch.setattr(husky_monitor.os, 'execv',
                        lambda path, argv: calls.update(execv=(path, list(argv))))
    husky_monitor._restart_monitor_for(robot_by_name('Alice'))
    assert husky_monitor.os.environ['ROS_DOMAIN_ID'] == '84'
    assert calls['run'] == ['ros2', 'daemon', 'stop']
    assert calls['execv'] == (sys.executable, [sys.executable] + sys.argv)
