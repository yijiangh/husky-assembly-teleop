"""Deleting a run on a mock Drive: its folder and as-built rows go, other runs' rows and pages stay or are rebuilt."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from husky_assembly_teleop.shared.runs import delete_run as module
from husky_assembly_teleop.shared.runs.as_built import AS_BUILT_FOLDER
from husky_assembly_teleop.shared.runs.delete_run import Page, delete_run, plan_deletion, rebuild_page

PROBLEM, RUN, OTHER = "261103_SRD", "20261104_175016", "20261104_183353"


def as_built_file(drive: Path, day: str, rows: list[str], problem: str = PROBLEM) -> Path:
    """Write a day's as-built file with one row per run name given."""
    path = drive / AS_BUILT_FOLDER / f"{day}-asbuilt" / f"as_built_{problem}.jsonl"
    path.parent.mkdir(parents=True)
    path.write_text("".join(json.dumps({"run_id": run, "stamp": "x"}) + "\n" for run in rows))
    return path


def page_file(drive: Path, day: str, name: str) -> Path:
    """Write a day's page."""
    path = drive / AS_BUILT_FOLDER / f"{day}-asbuilt-result" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("<html>")
    return path


@pytest.fixture
def run_folder(tmp_path) -> Path:
    """A run on the mock Drive, with a record and a picture."""
    folder = tmp_path / "data_design_study" / PROBLEM / "runs" / RUN
    (folder / "servo" / "B20_J").mkdir(parents=True)
    (folder / "run.json").write_text("{}")
    (folder / "servo" / "B20_J" / "x.png").write_bytes(b"12345")
    return folder


def test_the_plan_says_what_would_go_and_touches_nothing(tmp_path, run_folder):
    """Rows are found in every day a run went on in; another process's file is not looked at."""
    first = as_built_file(tmp_path, "20261104", [RUN, OTHER, RUN])
    second = as_built_file(tmp_path, "20261105", [RUN])
    as_built_file(tmp_path, "20261103", [OTHER])
    (tmp_path / AS_BUILT_FOLDER / "20261105-asbuilt" / "as_built_other.jsonl").write_text(
        json.dumps({"run_id": RUN}) + "\n")
    page = page_file(tmp_path, "20261104", "accuracy_20261104.html")
    before = first.read_text()
    plan = plan_deletion(run_folder, tmp_path)
    assert (plan.file_count, plan.size) == (2, 7)
    assert plan.rows == {first: 2, second: 1}
    assert [listed.path for listed in plan.pages] == [page]
    assert first.read_text() == before and run_folder.is_dir()


def test_only_a_run_s_folder_can_be_planned(tmp_path, run_folder):
    """A folder that is not in `runs/`, or holds no run.json, is refused: this deletes for good."""
    with pytest.raises(ValueError):
        plan_deletion(run_folder.parent.parent, tmp_path)
    (run_folder / "run.json").unlink()
    with pytest.raises(ValueError):
        plan_deletion(run_folder, tmp_path)


def test_deleting_keeps_other_runs_rows_exactly_and_asks_to_rebuild(tmp_path, run_folder):
    """Other rows stay byte for byte, a line that can't be read too; the day's pages are to be built again."""
    path = as_built_file(tmp_path, "20261104", [RUN, OTHER])
    with open(path, "a") as file:
        file.write(f"broken {RUN}\n")
    accuracy = page_file(tmp_path, "20261104", "accuracy_20261104.html")
    page_file(tmp_path, "20261104", "incidents_20261104.html")
    done = delete_run(plan_deletion(run_folder, tmp_path))
    assert not run_folder.exists() and run_folder.parent.is_dir()
    assert path.read_text() == json.dumps({"run_id": OTHER, "stamp": "x"}) + f"\nbroken {RUN}\n"
    assert done.rows == 1 and [page.path for page in done.to_rebuild] == [accuracy]
    assert done.pages_removed == () and accuracy.exists()


def test_a_day_left_without_records_goes_with_its_pages_but_not_its_incidents(tmp_path, run_folder):
    """The file and its empty day folder go, the pages made from the records too; an incident page reads the logs."""
    path = as_built_file(tmp_path, "20261104", [RUN])
    accuracy = page_file(tmp_path, "20261104", "accuracy_20261104.html")
    incidents = page_file(tmp_path, "20261104", "incidents_20261104.html")
    done = delete_run(plan_deletion(run_folder, tmp_path))
    assert not path.parent.exists() and not accuracy.exists() and incidents.exists()
    assert done.pages_removed == (accuracy,) and done.to_rebuild == ()


def test_a_day_with_only_results_left_loses_its_empty_folders(tmp_path, run_folder):
    """With nothing else in it, the result folder goes as well."""
    as_built_file(tmp_path, "20261104", [RUN])
    accuracy = page_file(tmp_path, "20261104", "accuracy_20261104.html")
    delete_run(plan_deletion(run_folder, tmp_path))
    assert not accuracy.parent.exists()


def test_pages_stay_when_the_day_s_other_records_are_another_process_s(tmp_path, run_folder):
    """A page does not say which process it shows, so it is neither deleted nor overwritten: it is reported."""
    as_built_file(tmp_path, "20261104", [RUN])
    other = tmp_path / AS_BUILT_FOLDER / "20261104-asbuilt" / "as_built_other.jsonl"
    other.write_text(json.dumps({"run_id": "elsewhere"}) + "\n")
    accuracy = page_file(tmp_path, "20261104", "accuracy_20261104.html")
    done = delete_run(plan_deletion(run_folder, tmp_path))
    assert done.kept == (accuracy,) and done.to_rebuild == () and accuracy.exists() and other.exists()


def test_a_line_appended_during_the_rewrite_is_kept(tmp_path, run_folder, monkeypatch):
    """Another monitor appends while the rows are being taken out: the rewrite starts over with its line."""
    path = as_built_file(tmp_path, "20261104", [RUN, OTHER])
    write_bytes, appended = Path.write_bytes, []

    def write_then_append(self, data):
        write_bytes(self, data)
        if not appended:
            appended.append(True)
            with open(path, "a") as file:
                file.write(json.dumps({"run_id": "new"}) + "\n")

    monkeypatch.setattr(Path, "write_bytes", write_then_append)
    delete_run(plan_deletion(run_folder, tmp_path))
    assert [json.loads(line)["run_id"] for line in path.read_text().splitlines()] == [OTHER, "new"]
    assert not path.with_name(path.name + ".tmp").exists()


def test_a_failed_folder_delete_leaves_the_run_to_try_again(tmp_path, run_folder, monkeypatch):
    """The rows go first, so trying again finds nothing more to take out and only the folder is left to do."""
    path = as_built_file(tmp_path, "20261104", [RUN, OTHER])
    plan = plan_deletion(run_folder, tmp_path)

    def fail(_folder):
        raise OSError("busy")

    monkeypatch.setattr(module.shutil, "rmtree", fail)
    with pytest.raises(OSError):
        delete_run(plan)
    monkeypatch.undo()
    assert run_folder.is_dir()
    again = plan_deletion(run_folder, tmp_path)
    assert again.rows == {}
    delete_run(again)
    assert not run_folder.exists() and len(path.read_text().splitlines()) == 1


def test_rebuilding_tells_the_script_the_day_and_says_what_went_wrong(tmp_path, monkeypatch):
    """A script that is missing or fails is a line for the operator, not an exception."""
    process, page = tmp_path / PROBLEM, Page(tmp_path / "accuracy_20261104.html", "20261104", "missing.py")
    assert "no script missing.py" in rebuild_page(page, tmp_path, process)
    script = tmp_path / "ok.py"
    script.write_text("import sys\nprint(sys.argv[1:])\nsys.exit(0 if sys.argv[1] == '20261104' else 3)\n")
    monkeypatch.setattr(module, "SCRIPTS", tmp_path)
    assert rebuild_page(Page(page.path, "20261104", "ok.py"), tmp_path, process) == ""
    assert "accuracy_20261104.html not rebuilt" in rebuild_page(Page(page.path, "2", "ok.py"), tmp_path, process)
