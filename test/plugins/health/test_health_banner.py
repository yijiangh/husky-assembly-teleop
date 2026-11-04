"""Health's banner: an overall chip, then one chip per robot and one for tracked objects, each in its worst colour."""

import re

from husky_assembly_teleop.plugins.health import render_banner, render_line
from husky_assembly_teleop.ui.style import BUSY, FAIL, INFO as BLUE, OK
from husky_assembly_teleop.world.checks import BAD, GOOD, INFO, PENDING, WARN, Check


def _chips(html: str) -> list[tuple[str, str, str]]:
    """Each chip as (text, colour, hover text)."""
    return re.findall(r'<span title="([^"]*)" style="[^"]*background:([^;]+);[^"]*">([^<]*)</span>', html)


def test_all_green_shows_one_green_chip_per_robot():
    """Nothing wrong: ALL GREEN, then a green chip per robot by its short serial, on one chip line."""
    html = render_banner({"a200-0804": [("base", [Check("ESTOP", GOOD)])],
                          "a200-0806": [("base", [Check("ESTOP", GOOD)])]}, [])
    assert html.count('class="chip-line"') == 1
    assert [(text, colour) for _title, colour, text in _chips(html)] == [("ALL GREEN", OK), ("0804", OK), ("0806", OK)]


def test_the_overall_chip_takes_the_worst_robot_and_each_robot_its_own():
    """One amber robot, one red: the overall chip is red, and its hover text lists what is not green, worst first."""
    html = render_banner({"a200-0804": [("base", [Check("battery", WARN, "low")])],
                          "a200-0806": [("", [Check("SOFT STOPPED", BAD, "Stop all")]),
                                        ("ur_arm", [Check("joints", GOOD), Check("sync", WARN, "old")])]},
                         [Check("probe", GOOD)])
    (overall_title, overall, label), alice, cindy, objects = _chips(html)
    assert (label, overall) == ("PROBLEMS", FAIL)
    assert overall_title.splitlines()[0] == "0806 SOFT STOPPED: Stop all"
    assert (alice[2], alice[1], alice[0]) == ("0804", BUSY, "base battery: low")
    assert (cindy[2], cindy[1]) == ("0806", FAIL)
    assert cindy[0].splitlines() == ["SOFT STOPPED: Stop all", "ur_arm sync: old"]
    assert (objects[2], objects[1], objects[0]) == ("objects", OK, "nothing amber or red")


def test_amber_without_red_is_check():
    """Only amber chips: the overall chip is amber."""
    title, colour, text = _chips(render_banner({"a200-0805": [("base", [Check("battery", WARN, "low")])]}, []))[0]
    assert (text, colour) == ("CHECK", BUSY) and title == "0805 base battery: low"


def test_a_simulated_part_is_blue_but_keeps_its_robot_green():
    """A sim chip is blue on its own line, yet its robot and the overall chip stay green."""
    lines = [("base", [Check("ESTOP", GOOD), Check("sim", INFO, "husky_sim")])]
    assert re.search(f'background:{BLUE};[^"]*">sim<', render_line(*lines[0]))
    (title, overall, label), (_robot_title, robot, _serial) = _chips(render_banner({"a200-0804": lines}, []))
    assert (label, overall, robot) == ("ALL GREEN", OK, OK)
    assert title == "nothing amber or red; facts: 0804 base sim", "the hover never claims every chip is green"


def test_grey_chips_are_named_on_an_all_green_banner():
    """Nothing amber or red, but a tcp not checked yet: the green chip's hover names it rather than say all green."""
    lines = [("ur_arm", [Check("signal", GOOD), Check("tcp", PENDING, "not checked yet")])]
    (title, overall, _label), (robot_title, _robot, _serial) = _chips(render_banner({"a200-0804": lines}, []))
    assert overall == OK and title == "nothing amber or red; not judged yet: 0804 ur_arm tcp"
    assert robot_title == "nothing amber or red; not judged yet: ur_arm tcp"
