"""A launch-argument mistake is a ConfigError whose message names the wrong value and, if one is close, the fix."""

import pytest

from husky_assembly_teleop.config import ConfigError, _parse_tools


def test_a_misspelt_tool_suggests_the_closest_and_lists_the_rest(monkeypatch):
    """Without colour the message is plain text."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "0")
    with pytest.raises(ConfigError) as error:
        _parse_tools(("alice:scaffolding_v2",))
    assert "unknown tool 'scaffolding_v2' in tools entry 'alice:scaffolding_v2'" in str(error.value)
    assert "did you mean   scaffolding_v3\n" in str(error.value)
    assert "calibration_marker" in str(error.value)


def test_a_tool_with_no_close_name_suggests_nothing_and_is_coloured_red(monkeypatch):
    """With colour forced on, the wrong name is bold red."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "1")
    with pytest.raises(ConfigError) as error:
        _parse_tools(("alice:probe",))
    assert "\033[1;31m'probe'\033[0m" in str(error.value)
    assert "did you mean" not in str(error.value)
