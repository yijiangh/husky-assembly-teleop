"""The guided procedure: Back and Next only move, Done asks once more when unfinished, the overview, the procedure."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
import viser

from husky_assembly_teleop.shared.guide.both_pages import on_both
from husky_assembly_teleop.shared.guide.checklist import Checklist
from husky_assembly_teleop.shared.guide.panel import GuidePanel
from husky_assembly_teleop.shared.guide.procedure import Procedure
from husky_assembly_teleop.shared.guide.step import Step
from husky_assembly_teleop.ui.style import BUSY, FAIL, INFO, OK
from husky_assembly_teleop.world.checks import BAD, GOOD, PENDING, WARN, Check


class FakeStep(Step):
    """A step whose checks, readiness and expiry the test sets, and that counts entering and leaving."""

    def __init__(self, name: str):
        """Green, never entered."""
        self.name, self.instructions = name, f"do {name}"
        self.level, self.ready_level, self.entered, self.left, self.why = GOOD, GOOD, 0, 0, None

    def checks(self, ctx):
        """One check at `level`."""
        return [Check(f"{self.name} result", self.level, "")]

    def ready(self, ctx):
        """One check at `ready_level`."""
        return [Check(f"{self.name} ready", self.ready_level, "not now")]

    def expired(self, ctx):
        """`why`, set by the test."""
        return self.why

    def enter(self, ctx):
        """Count it."""
        self.entered += 1

    def leave(self, ctx):
        """Count it."""
        self.left += 1


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module."""
    server = viser.ViserServer(port=0, verbose=False)
    yield server
    server.stop()


def make_ctx() -> SimpleNamespace:
    """A plugin context whose intents run at once, whose log is kept, and whose clock the test sets (`ctx.time`)."""
    log = []

    def defer(_label, run, operator_only=False):
        async def callback(*_args):
            run()
        return callback

    def defer_value(_label, run, operator_only=False):
        async def callback(event):
            # * As the runtime: a change the code made itself comes without a client.
            if event.client is not None or not operator_only:
                run(event.target.value)
        return callback

    ctx = SimpleNamespace(defer=defer, defer_value=defer_value, log=log, log_info=log.append, log_warn=log.append,
                          time=0.0)
    ctx.now = lambda: ctx.time
    return ctx


def make_panel(server, count: int = 3, go_to: int = 2):
    """A panel of `count` fake steps whose Done goes to step `go_to`, keeping each Done's accept."""
    steps = [FakeStep(f"s{index}") for index in range(count)]
    dones = []

    def on_done(_ctx, accept):
        dones.append(accept)
        return go_to

    panel = GuidePanel("test", steps, Procedure(), on_done=on_done)
    ctx = make_ctx()
    panel.build(ctx, server.gui)
    return panel, steps, ctx, dones


def test_back_and_next_move_one_step_whatever_is_red(server):
    """Next and Back move one step, leaving and entering, red checks or not; Done is never asked."""
    panel, steps, ctx, dones = make_panel(server)
    steps[0].level = BAD
    panel.navigate(ctx, forward=True)
    panel.navigate(ctx, forward=True)
    assert panel.index == 2 and steps[0].left == 1 and steps[2].entered == 1
    panel.navigate(ctx, forward=True)
    assert panel.index == 2 and steps[2].entered == 1
    panel.navigate(ctx, forward=False)
    assert panel.index == 1 and dones == []
    panel.draw(ctx)
    assert panel._next_button.color is None and not panel._next_button.disabled


def test_done_leaves_a_finished_step_for_where_on_done_says(server):
    """A step whose state is finished: Done goes at once, without accepting."""
    panel, steps, ctx, dones = make_panel(server)
    panel.draw(ctx)
    assert panel._done_button.color == "green"
    panel.done(ctx)
    assert panel.index == 2 and dones == [False]


@pytest.mark.parametrize("level, label", [(BAD, "Accept as is"), (PENDING, "Skip")])
def test_done_asks_once_more_on_an_unfinished_step_then_goes_on(server, level, label):
    """A failing or missing result: the first Done turns into Accept as is or Skip; pressed in time, it goes on."""
    panel, steps, ctx, dones = make_panel(server)
    steps[0].level = level
    steps[0].state = lambda ctx: Check("none yet" if level == PENDING else "fails", level, "nothing yet")
    panel.draw(ctx)
    assert panel._done_button.color == "orange" and panel._done_button.label == "Done"
    panel.done(ctx)
    panel.draw(ctx)
    assert panel.index == 0 and dones == [] and f"{label} within 5 s goes on" in panel._notes_html.content
    assert panel._done_button.label == label
    ctx.time = 4.0
    panel.done(ctx)
    assert panel.index == 2 and dones == [True]


def test_done_asks_afresh_once_its_window_has_passed(server):
    """A second Done later than CONFIRM_SECONDS is a first one again; the button is Done meanwhile."""
    panel, steps, ctx, dones = make_panel(server)
    steps[0].level = BAD
    panel.done(ctx)
    ctx.time = 6.0
    panel.draw(ctx)
    assert panel._done_button.label == "Done" and "goes on" not in panel._notes_html.content
    panel.done(ctx)
    assert panel.index == 0 and dones == []
    ctx.time = 7.0
    panel.done(ctx)
    assert panel.index == 2 and dones == [True]


def test_done_forgets_its_question_once_the_step_is_finished(server):
    """Fixed between the presses, the step is left at the next Done without accepting it."""
    panel, steps, ctx, dones = make_panel(server)
    steps[0].level = BAD
    panel.done(ctx)
    steps[0].level = GOOD
    panel.draw(ctx)
    assert "goes on" not in panel._notes_html.content
    steps[0].level = BAD
    panel.done(ctx)
    assert panel.index == 0 and dones == []  # * asked afresh


def test_without_on_done_done_goes_to_the_next_step(server):
    """The generic default: the next step, the last one staying."""
    steps = [FakeStep(f"s{index}") for index in range(2)]
    panel, ctx = GuidePanel("test", steps, Procedure()), make_ctx()
    panel.build(ctx, server.gui)
    panel.done(ctx)
    panel.done(ctx)
    assert panel.index == 1


def test_ready_checks_always_show(server):
    """Green ready checks show too: chips change colour, never appear or vanish."""
    panel, steps, ctx, _ = make_panel(server)
    panel.draw(ctx)
    assert ">s0 ready<" in panel._status_html.content


def test_ready_checks_never_keep_the_operator_but_refuse_actions(server):
    """A step that cannot act now may still be left; its refusal names the red ready checks."""
    panel, steps, ctx, _ = make_panel(server)
    steps[0].ready_level = BAD
    assert steps[0].refusal(ctx) == "s0 ready: not now"
    panel.done(ctx)
    assert panel.index == 2


def test_nothing_moves_while_a_procedure_runs(server):
    """Back, Done and Next wait until the procedure ends or is cancelled."""
    panel, steps, ctx, dones = make_panel(server)
    panel.procedure._task = SimpleNamespace(done=lambda: False)
    panel.navigate(ctx, forward=True)
    panel.done(ctx)
    assert panel.index == 0 and dones == [] and "Cancel it first" in ctx.log[-1]


def test_the_overview_shows_every_step_state_on_its_own_line(server):
    """Results solid, preconditions dashed, each with its state and why; the current step ringed.

    The current step outlines the precondition steps it needs that have expired.
    """
    panel, steps, ctx, _ = make_panel(server)
    steps[0].precondition, steps[0].why = True, "ticked 61 min ago"
    steps[0].state = lambda ctx: Check("expired", WARN, "ticked 61 min ago")
    steps[1].level = BAD
    steps[2].needs = ("s0",)
    panel.navigate(ctx, forward=True)
    panel.navigate(ctx, forward=True)
    panel.draw(ctx)
    rows = panel._list_html.content.split('<div style="display:flex')[1:]
    assert len(rows) == 3
    assert f"1px dashed {BUSY}" in rows[0] and "outline:2px dotted" in rows[0] and "ticked 61 min ago" in rows[0]
    assert f"background:{FAIL}" in rows[1] and ">fails<" in rows[1] and "outline" not in rows[1]
    assert f"background:{OK}" in rows[2] and f"box-shadow:0 0 0 2px {INFO}" in rows[2] and ">passes<" in rows[2]
    steps[0].why = None
    panel.draw(ctx)
    assert "outline:2px dotted" not in panel._list_html.content


def test_the_overview_never_changes_height(server):
    """One line per step, whatever its state says: nothing above the buttons moves."""
    panel, steps, ctx, _ = make_panel(server)
    panel.draw(ctx)
    quiet = panel._list_html.content
    steps[1].level = BAD
    steps[1].checks = lambda ctx: [Check("result", BAD, "a long reason " * 20)]
    panel.draw(ctx)
    busy = panel._list_html.content
    assert busy.count("<div") == quiet.count("<div") and busy.count("white-space:nowrap;overflow:hidden") == 3


def make_async_ctx():
    """A context that runs procedures as tasks and checks waits once per loop turn."""
    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await asyncio.sleep(0)

    return SimpleNamespace(spawn=lambda _name, work: asyncio.get_event_loop().create_task(work),
                           wait_until=wait_until, log_info=lambda _m: None, log_warn=lambda _m: None)


def test_a_procedure_waits_for_go_and_keeps_how_it_ended():
    """Go answers the prompt; a failure becomes the outcome; Cancel ends it; one at a time."""
    procedure, ctx = Procedure(), make_async_ctx()

    async def scenario():
        async def work():
            await procedure.ask_go(ctx, "ready?", "start")
            raise RuntimeError("the arm refused")

        procedure.start(ctx, "take", work(), "take x")
        await asyncio.sleep(0)
        assert procedure.running and procedure.prompt == "ready?" and procedure.go_label == "start"
        procedure.start(ctx, "other", asyncio.sleep(0), "other")
        procedure.go()
        await procedure._task
        assert not procedure.running and procedure.outcome == Check("take failed", BAD, "the arm refused")

        procedure.start(ctx, "check", procedure.ask_go(ctx, "go?"), "check")
        await asyncio.sleep(0)
        procedure.cancel()
        with pytest.raises(asyncio.CancelledError):
            await procedure._task
        assert procedure.outcome == Check("check cancelled", WARN) and procedure.prompt == ""

    asyncio.run(scenario())


@pytest.mark.parametrize("level, color", [(GOOD, OK), (WARN, BUSY), (BAD, FAIL)])
def test_the_outcome_chip_takes_the_level_it_was_given(server, level, color):
    """A benign outcome ("new run") is green, a refusal red: the level is set where the outcome is, not guessed."""
    panel, _, ctx, _ = make_panel(server)
    panel.procedure.outcome = Check("new run", level, "its file starts with the first thing recorded")
    panel.draw(ctx)
    assert "new run" in panel._status_html.content and color in panel._status_html.content


def test_a_checklist_starts_unticked_and_ticks_by_label(server):
    """`tick` sets boxes by label, e.g. to show ticks kept from before."""
    checklist = Checklist((("Area clear", ""), ("E-stop at hand", "")))
    with server.gui.add_folder("checklist test"):
        checklist.build(make_ctx(), server.gui, "before you start", lambda *_args: None)
    assert [box.value for box in checklist.boxes] == [False, False]
    checklist.tick(["Area clear"], True)
    assert [box.value for box in checklist.boxes] == [True, False]
    checklist.tick(["Area clear", "E-stop at hand"], True)
    checklist.tick(["Area clear"], False)
    assert [box.value for box in checklist.boxes] == [False, True]


def test_chips_that_come_and_go_stay_on_the_status_line(server):
    """The status line is there with nothing on it, and holds what runs and the outcome.

    The Go question goes below the controls, so nothing above them changes height.
    """
    panel, _, ctx, _ = make_panel(server)
    panel.draw(ctx)
    quiet = panel._status_html.content
    assert 'class="chip-line"></div>' in quiet
    panel.procedure.doing, panel.procedure.prompt = "moving", "Move there? Go"
    panel.procedure.outcome = Check("take failed", BAD, "the arm refused")
    panel.draw(ctx, extra="<span>lease</span>")
    busy = panel._status_html.content
    assert busy.count("<div") == quiet.count("<div") and "moving" in busy and "take failed" in busy
    assert "Move there?" in panel._notes_html.content and "Move there?" not in busy


def test_both_pages_writes_each_page_only_what_changed():
    """Reads give the laptop's value; a write reaches both pages, content through the phone's transform."""
    class Handle:
        def __init__(self):
            self.content, self.writes = "", 0

        def __setattr__(self, name, value):
            if name == "content":
                object.__setattr__(self, "writes", getattr(self, "writes", 0) + 1)
            object.__setattr__(self, name, value)

    laptop, phone = Handle(), Handle()
    both = on_both(laptop, phone, lambda html: f"<p>{html}</p>")
    both.content = "a"
    both.content = "a"
    assert (laptop.content, phone.content, both.content) == ("a", "<p>a</p>", "a")
    assert (laptop.writes, phone.writes) == (1, 1)


def test_a_tick_on_one_page_ticks_the_other(server):
    """The checklist built on two pages keeps their boxes alike, and reports each operator tick once."""
    ticks, ctx = [], make_ctx()
    checklist = Checklist((("Area clear", ""), ("E-stop at hand", "")))
    with server.gui.add_folder("laptop"):
        checklist.build(ctx, server.gui, "before you start", lambda label, ticked: ticks.append((label, ticked)))
    with server.gui.add_folder("phone"):
        checklist.build(ctx, server.gui, "before you start", lambda label, ticked: ticks.append((label, ticked)))
    phone_box = checklist._pages[1][0]
    phone_box._impl.value = True
    for callback in phone_box._impl.update_cb:
        asyncio.run(callback(SimpleNamespace(client=object(), target=phone_box)))
    assert checklist.boxes[0].value and ticks == [("Area clear", True)]
    checklist.tick(["E-stop at hand"], True)
    assert checklist._pages[1][1].value and all(box.value for box in checklist.boxes)
