"""`finished_moves`: what the executor finished in a run, read back from its log in order, records left untouched."""

from __future__ import annotations

import json
from pathlib import Path

from husky_assembly_teleop.shared.movement_exec_contract import DONE, DRY_RUN, FAILED
from husky_assembly_teleop.shared.runs.finished import FINISHED, finished_moves
from husky_assembly_teleop.shared.runs.runs import record_event


def executed(folder: Path, action: str, movement: str, primary: int, index: int, result: str = DONE,
             also: tuple[int, ...] = (), keys: bool = True, dry_run: bool = False) -> None:
    """A part's record and its `exec` event, as the executor writes them; `keys` False: an older event without them."""
    file = f"movements/{action}/{movement}.1.json"
    (folder / file).parent.mkdir(parents=True, exist_ok=True)
    (folder / file).write_text(json.dumps({"schema": "husky_run/1", "kind": "movement", "step": {"primary": primary},
                                           "part": {"index": index, "also": list(also)},
                                           "outcome": {"result": result, "dry_run": dry_run}}))
    data = {"status": "x", "result": result}
    if keys:
        data["keys"] = [[primary, each] for each in (index, *also)]
    record_event(folder, "exec", 1.0, action=action, movement=movement, data=data, file=file)


def forgot(folder: Path, action: str, done) -> None:
    """The executor's `forget` event: `done` the pairs, or None for the whole action."""
    record_event(folder, "forget", 1.0, action=action, data={"done": done})


def test_the_result_it_reads_is_the_executors():
    """The result string it counts is the executor's own."""
    assert FINISHED == DONE


def test_done_parts_count_failed_and_dry_ones_do_not(tmp_path):
    """New events carry their keys, older ones name their record; a folded part counts its other movements too."""
    executed(tmp_path, "B20_J", "B20_J_transfer", 1, 1)
    executed(tmp_path, "B20_J", "B20_J_place_bar", 2, 2, also=(3,), keys=False)
    executed(tmp_path, "B20_J", "B20_J_insert", 4, 4, result=FAILED)
    executed(tmp_path, "B20_J", "B20_J_servo", 1, -1, result=DRY_RUN, dry_run=True)
    executed(tmp_path, "B20_R", "B20_R_home", 2, 2, keys=False, dry_run=True)
    assert finished_moves(tmp_path) == {"B20_J": {(1, 1), (2, 2), (2, 3)}}


def test_a_forget_outdates_only_what_came_before_it(tmp_path):
    """Redo from a part: its pairs go; the whole action on a reopen; a part done again afterwards counts again."""
    executed(tmp_path, "B20_J", "B20_J_transfer", 1, 1)
    executed(tmp_path, "B20_J", "B20_J_insert", 4, 4)
    executed(tmp_path, "B20_R", "B20_R_home", 2, 2)
    forgot(tmp_path, "B20_J", [[4, 4]])
    forgot(tmp_path, "B20_R", None)
    assert finished_moves(tmp_path) == {"B20_J": {(1, 1)}}
    executed(tmp_path, "B20_J", "B20_J_insert", 4, 4)
    assert finished_moves(tmp_path) == {"B20_J": {(1, 1), (4, 4)}}
    assert (tmp_path / "movements/B20_R/B20_R_home.1.json").is_file(), "records are never moved"


def test_no_log_and_a_lost_record_read_as_nothing(tmp_path):
    """A run without a log yet, and an older event whose record is gone, give nothing finished."""
    assert finished_moves(tmp_path) == {}
    record_event(tmp_path, "exec", 1.0, action="B20_J", movement="B20_J_transfer", data={"result": DONE},
                 file="movements/B20_J/B20_J_transfer.9.json")
    assert finished_moves(tmp_path) == {}
