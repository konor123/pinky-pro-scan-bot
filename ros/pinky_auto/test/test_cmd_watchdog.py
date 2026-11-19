import pytest

from pinky_auto.cmd_watchdog import CmdWatchdog


def test_no_stop_before_first_command():
    assert not CmdWatchdog(0.5).should_stop(100.0)


def test_stop_after_timeout():
    wd = CmdWatchdog(0.5)
    wd.feed(10.0)
    assert not wd.should_stop(10.5)
    assert wd.should_stop(10.51)


def test_stop_reported_until_marked():
    wd = CmdWatchdog(0.5)
    wd.feed(10.0)
    assert wd.should_stop(11.0)
    assert wd.should_stop(11.1)
    wd.mark_stopped()
    assert not wd.should_stop(12.0)


def test_new_command_rearms():
    wd = CmdWatchdog(0.5)
    wd.feed(10.0)
    wd.mark_stopped()
    wd.feed(20.0)
    assert not wd.should_stop(20.4)
    assert wd.should_stop(20.6)


def test_steady_commands_never_stop():
    wd = CmdWatchdog(0.5)
    t = 0.0
    for _ in range(100):
        wd.feed(t)
        t += 0.05
        assert not wd.should_stop(t)


def test_invalid_timeout():
    with pytest.raises(ValueError):
        CmdWatchdog(0.0)
