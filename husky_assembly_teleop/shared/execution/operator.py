"""
The question the operator answers before anything moves: Confirm or Cancel, one prompt at a time.

Every prompt names its robot first, so the phone in the lab says which robot is about to move.

- A Confirm within PROMPT_ARM_S of the prompt opening is a double click: ignored. A Cancel always counts, and wins over
  a Confirm not yet taken.
- An answer reaches only the prompt open when it was given; a new prompt drops any earlier answer.
- ! Main thread only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext

#: A Confirm sooner than this after the prompt opened, seconds, is a double click: ignored.
PROMPT_ARM_S = 0.5


@dataclass(frozen=True)
class Prompt:
    """One question to the operator.

    Attributes:
        id: Tells this prompt from every other one.
        robot: The robot it is about, e.g. "alice (a200-0804)".
        text: The question.
        warn: Whether it is a warning (shown in orange, logged as a warning).
        opened_at: ROS time it was asked, seconds.
    """

    id: int
    robot: str
    text: str
    warn: bool
    opened_at: float

    @property
    def line(self) -> str:
        """str: The prompt as one line, the robot first."""
        return f"{self.robot}: {self.text}"


class PromptBox:
    """The open prompt, and the answers given to it. Main thread.

    Attributes:
        current: The prompt waiting for an answer, or None.
    """

    def __init__(self):
        """No prompt yet."""
        self.current: Prompt | None = None
        self._count = 0
        self._answers: dict[int, bool] = {}

    def ask(self, robot: str, text: str, now: float, warn: bool = False) -> Prompt:
        """Open a prompt, replacing any other; earlier answers are dropped.

        Args:
            robot: The robot it is about, e.g. "alice (a200-0804)".
            text: The question.
            now: ROS time, seconds.
            warn: Whether it is a warning.

        Returns:
            Prompt: The prompt; hand it to `wait`, or to `poll` each tick.
        """
        self._count += 1
        self.current = Prompt(self._count, robot, text, warn, now)
        self._answers.clear()
        return self.current

    def answer(self, ok: bool, now: float) -> str:
        """Answer the open prompt: Confirm (True) or Cancel (False).

        Args:
            ok: True to confirm.
            now: ROS time, seconds.

        Returns:
            str: "" if taken; else why it was ignored (no prompt, too soon after it opened, already confirmed).
        """
        prompt = self.current
        if prompt is None:
            return "nothing to answer"
        if ok and now - prompt.opened_at < PROMPT_ARM_S:
            return (f"Confirm ignored: {now - prompt.opened_at:.1f} s after the prompt opened, under "
                    f"{PROMPT_ARM_S} s (a double click?); read the prompt, then Confirm")
        if ok and prompt.id in self._answers:
            return "Confirm ignored: the prompt is already answered"
        self._answers[prompt.id] = ok
        return ""

    async def wait(self, ctx: PluginContext, prompt: Prompt) -> bool:
        """Wait, without a time limit, for the answer to `prompt`; then close it.

        Args:
            ctx: The plugin's context, for its ticks.
            prompt: The prompt `ask` opened.

        Returns:
            bool: True if confirmed, False if cancelled.
        """
        try:
            # ? No time limit: placing a bar by hand can take minutes.
            await ctx.wait_until(lambda: prompt.id in self._answers, description=f"an answer to {prompt.line!r}")
            return self._answers[prompt.id]
        finally:
            self.close(prompt)

    def poll(self, prompt: Prompt) -> bool | None:
        """The answer to `prompt` if one came, closing it then; None while it waits.

        Args:
            prompt: The prompt `ask` opened.

        Returns:
            bool | None: True if confirmed, False if cancelled, None if not answered yet.
        """
        answer = self._answers.get(prompt.id)
        if answer is not None:
            self.close(prompt)
        return answer

    def close(self, prompt: Prompt) -> None:
        """Close `prompt`, answered or not; its answer is dropped and a newer prompt stays open.

        Args:
            prompt: The prompt `ask` opened.
        """
        self._answers.pop(prompt.id, None)
        if self.current is prompt:
            self.current = None
