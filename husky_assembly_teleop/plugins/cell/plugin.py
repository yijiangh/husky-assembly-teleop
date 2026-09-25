"""
The cell plugin: owns the loaded design, the selected step, and the planner.

? What this plugin is for.
  The core knows about the live robots and provides infrastructure. It has no
  idea what a bar, a rack or an assembly step is. So the design, the position in
  it, and the planning session all live here.

! It owns a compas_fab session, not a pile of PyBullet bodies.
  A study of the old code settled this. The live planning path never calls
  PyBullet collision APIs itself -- `pairwise_collision` appears zero times.
  Instead `CfabSession` materializes the whole cell in one go through
  `planner.set_robot_cell(cell)`, and each movement is pushed as a declarative
  `RobotCellState` via `planner.set_robot_cell_state(state)`. compas_fab owns
  those body ids, and asserts that the cell and every state carry exactly the
  same rigid-body ids.

  Two consequences, both of which killed earlier drafts of this file:

  - Do not add and remove bodies per step. The authored `is_hidden` flag on a
    RigidBodyState is how "not an obstacle right now" is expressed, and the old
    code's own comment says switching movements that way "no longer re-adds and
    re-removes the built bars".
  - Do not compute the per-step layout. It is authored: each movement in a
    BarAction file carries its own `start_state` with frames, attachments,
    is_hidden and touch_links already set. This plugin's real job is to *patch*
    that authored state with live measurements and push it.

! Patching authored state with live measurement is the delicate part.
  The old code has a documented bug from getting it wrong: it wrote the live
  base pose into the planning state unconditionally, which teleported the
  planning robot to the origin whenever mocap had not produced a fix yet. Only
  tracked values may be copied in. That is the one piece of logic here worth
  testing properly.
"""

from __future__ import annotations

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from .assembly import Assembly


@register
class CellPlugin(HuskyPlugin):
    """Holds the design and the selected step, and drives the cfab planner."""

    name = "cell"

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded design. Replaced wholesale on load, never edited in place.
        self.assembly = Assembly()

        #: Which step is selected. The one piece of genuine state here, because
        #: it is an operator decision rather than a fact about anything.
        self.index = 0

        #: The compas_fab planning session: client, robot cell and planner.
        #: TODO port CfabSession. It owns its own PyBullet client -- deliberately
        #:      NOT ctx.scene's. The old code let the two share one client id and
        #:      then had to hide cfab's robot because it overlapped the live one.
        #:      Two clients, no sharing.
        self.session = None

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel. One widget set, mutated later, never rebuilt.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._step_slider = gui.add_slider("Step", min=0, max=0, step=1, initial_value=0)
            self._step_label = gui.add_text("Loaded", initial_value="no design", disabled=True)

        # ! Every viser callback goes through defer.
        #   viser fires this on a worker thread; defer turns it into an intent on
        #   this plugin's queue, so the work happens on the ROS thread where
        #   touching PyBullet is safe.
        self._step_slider.on_update(
            ctx.defer("select step", lambda: self.select_step(ctx, self._step_slider.value))
        )

        # TODO a file dialog for picking the BarAction file, wired the same way.

    def teardown(self, ctx: PluginContext) -> None:
        """Close the planning session, which owns its own PyBullet connection.

        Args:
            ctx: This plugin's context.
        """
        # TODO self.session.close() once CfabSession is ported.

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # Called from intents, so always on the ROS thread.

    def load(self, ctx: PluginContext, assembly: Assembly) -> None:
        """Replace the design and go back to the first step.

        Args:
            ctx: This plugin's context.
            assembly: The design to load.
        """
        self.assembly = assembly
        self.index = 0
        self._apply(ctx)

    def select_step(self, ctx: PluginContext, index: int) -> None:
        """Select a step and push its state to the planner.

        Args:
            ctx: This plugin's context.
            index: Step index, clamped to the loaded design.
        """
        if not self.assembly.steps:
            return
        self.index = max(0, min(index, len(self.assembly.steps) - 1))
        self._apply(ctx)

    # --- --- --- --- --- INTERNALS --- --- --- --- ---

    def _apply(self, ctx: PluginContext) -> None:
        """Push the current step's state to the planner and refresh the panel.

        Args:
            ctx: This plugin's context.
        """
        if not self.assembly.steps:
            self._step_label.value = "no design"
            return

        step = self.assembly.steps[self.index]
        self._step_slider.max = len(self.assembly.steps) - 1
        self._step_label.value = f"{step.step_id}: {step.element_name}"

        # TODO take this movement's authored start_state, patch it with the live
        #      base pose and arm configuration from ctx.world -- but ONLY where
        #      those are flagged tracked -- and push it with
        #      self.session.planner.set_robot_cell_state(state).
        # TODO mirror the same geometry into ctx.view so the browser shows it.
        #      Add one mesh node per element on first sight and assign transforms
        #      afterwards; viser is retained mode.
