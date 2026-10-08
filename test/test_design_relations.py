"""Tests for what follows from a state without being stored: the pose rule, mate status, allowed contacts."""

from dataclasses import replace
from pathlib import Path

import pytest
from design_fixtures import build_design

from bar_assembly_core.design import Design, Holder, State
from bar_assembly_core.design.relations import (ENGAGED, NOT_RELEVANT, OPEN, PENDING, allowed_contacts, is_present,
                                                mate_status, mount_offset, placement)
from bar_assembly_core.geometry import Pose, compose

G1_MATE, J1_MATE = ("ground/WG0", "joints/G1_ground"), ("joints/J1_female", "joints/J1_male")


@pytest.fixture
def design(tmp_path: Path) -> Design:
    """The fixture design."""
    return build_design(tmp_path)


def _start(design: Design, movement_id: str) -> State:
    """The start state of one movement, by id."""
    return next(movement.start for _, movement in design.movements() if movement.id == movement_id)


def test_pose_rule(design):
    """Built: design pose (even while held); attached and unbuilt: the first holder; staged: its pose, else design."""
    held_by_two = _start(design, "B1_R_M0_ungrasp")
    assert len(held_by_two.attached["bars/B1"]) == 2
    assert placement(design, held_by_two, "bars/B1") == design.bodies["bars/B1"].pose
    carried = _start(design, "B2_M1_grasp")
    assert placement(design, carried, "bars/B2") == carried.attached["bars/B2"][0]
    staged = _start(design, "B1_M0_mount")
    assert placement(design, staged, "bars/B2") == Pose((2.0, 0.0, 0.1))
    assert placement(design, replace(staged, poses={}), "bars/B2") == design.bodies["bars/B2"].pose


def test_halves_follow_their_bar(design):
    """A half is placed by its bar's placement times the mount offset, and is present with its bar."""
    offset = mount_offset(design, "joints/J1_male")
    carried = _start(design, "B2_M1_grasp")
    holder = carried.attached["bars/B2"][0]
    assert placement(design, carried, "joints/J1_male") == Holder(holder.to, compose(holder.grasp, offset))
    staged = _start(design, "B1_M0_mount")
    assert placement(design, staged, "joints/J1_male") == compose(Pose((2.0, 0.0, 0.1)), offset)
    assert not is_present(design, staged, "joints/G1_ground") and is_present(design, staged, "joints/J1_male")
    assert design.halves_of("bars/B1") == ("joints/G1_ground", "joints/J1_female")


def test_mate_status(design):
    """Open, pending while being joined, engaged once both sides are built, not relevant when neither is present."""
    assert mate_status(design, _start(design, "B1_M0_mount"), G1_MATE) == OPEN
    assert mate_status(design, _start(design, "B1_M2_insert"), G1_MATE) == PENDING
    assert mate_status(design, _start(design, "B1_H_M0_approach"), G1_MATE) == ENGAGED
    assert mate_status(design, _start(design, "B1_H_M0_approach"), J1_MATE) == OPEN
    assert mate_status(design, _start(design, "B2_M3_insert"), J1_MATE) == PENDING
    assert mate_status(design, _start(design, "B1_HR_M0_open"), J1_MATE) == ENGAGED
    empty = replace(_start(design, "B1_M0_mount"), present=frozenset({"ground/WG0"}), poses={})
    assert mate_status(design, empty, J1_MATE) == NOT_RELEVANT


def test_allowed_contacts_are_derived(design):
    """At B2's insert: tools with their body, halves with their bars, pending and engaged mates, wheels with ground.

    Nothing else: not a tool with the rest of the bar it holds.
    """
    contacts = allowed_contacts(design, _start(design, "B2_M3_insert"))
    assert contacts == {
        ("joints/J1_male", "tools/AT3L"), ("bars/B1", "tools/Grip"),
        ("bars/B1", "joints/G1_ground"), ("bars/B1", "joints/J1_female"), ("bars/B2", "joints/J1_male"),
        G1_MATE, J1_MATE,
        ("ground/WG0", "robots/alice/wheel_link"), ("ground/WG0", "robots/cindy/wheel_link"),
    }
    # * Before B1 is mounted: B1's halves are absent, the open J1 mate allows nothing, Alice (absent) has no wheels.
    assert allowed_contacts(design, _start(design, "B1_M0_mount")) == {
        ("bars/B2", "joints/J1_male"), ("ground/WG0", "robots/cindy/wheel_link")}
