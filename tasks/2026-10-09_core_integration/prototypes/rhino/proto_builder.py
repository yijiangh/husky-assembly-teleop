"""
Prototype (scratchpad only): the builder RhinoDesignSource would need, as a state machine over the schedule.

Input: the per-bar records Rhino keeps in user text today (base, approach/assembled/retreat joints, tool0 frames,
which joint each tool is on, support keyframes). Here they are extracted from the converted 260814 design to stand in
for the document. Output: schema 2 actions in the agreed procedure (merged insert, form-B ungrasp, holds with two
holders on a built bar), with one running State threaded through the whole schedule. Then A and B checks.

Run with the Python 3.9 venv: py39_rhino/bin/python -I proto_builder.py <scratchpad>
"""
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop")
from bar_assembly_core.design import (Action, Design, Holder, Movement, RobotState, State, Target, ToolState,
                                      read, validate)
from bar_assembly_core.design.plan_check import check_plan

CINDY, L, R = "robots/cindy", "robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"
AT = {L: "tools/AT3L", R: "tools/AT3R"}
INSERT_DISTANCE, RETREAT_DISTANCE = 0.015, 0.05


# --- --- the "document": what Rhino stores per bar (extracted from the converted design) --- ---

def records(design: Design):
    """Per bar: the Rhino keyframe record; per held bar: the support record."""
    bars, holds = {}, {}
    for action_id in design.schedule:
        a = design.actions[action_id]
        m = a.movements
        if a.type == "bar_jointing":
            mount, transfer, insert = m[1], m[3], m[4]
            bars[a.bar] = dict(
                base=m[0].start.robots[CINDY].base, ground=a.ground,
                grasps={h.to: h.grasp for h in mount.target.attached[a.bar]},
                on={flange: m[2].start.tools[AT[flange]].on for flange in (L, R)},
                approach=transfer.target.joints.get(CINDY), approach_links=transfer.target.links,
                assembled=insert.target.joints.get(CINDY), assembled_links=insert.target.links,
                insert_line=insert.line)
        elif a.type == "bar_release":
            ungrasp, retreat, home = m
            bars[a.bar].update(retreat=retreat.target.joints.get(CINDY), retreat_links=retreat.target.links,
                               retreat_line=retreat.line, home=home.target.joints.get(CINDY))
        elif a.type == "bar_holding":
            # ? B12_H and B15_H lost their no-op "open" in the conversion: find movements by their parts.
            free = [x for x in m if x.path == "free"][0]
            line = [x for x in m if x.path == "linear"][0]
            m = (free, None, line, m[-1])
            flange = m[0].arms[0]
            holds[a.bar] = dict(robot=a.robot, flange=flange, tool=design.robots[a.robot].tools[flange.split("/")[-1]],
                                base=m[0].start.robots[a.robot].base, ground=a.ground,
                                supports_until=a.supports_until,
                                approach=m[0].target.joints.get(a.robot), approach_links=m[0].target.links,
                                held=m[2].target.joints.get(a.robot), held_links=m[2].target.links,
                                line=m[2].line,
                                grasp=[h for h in m[3].target.attached[a.bar] if h.to == flange][0].grasp)
        elif a.type == "bar_holding_release":
            holds[a.bar].update(retreat_line=m[1].line)
    return bars, holds


# --- --- the builder: one running state through the schedule --- ---

class Builder:
    def __init__(self, design: Design):
        self.design = design
        grounds = frozenset(b for b in design.bodies if b.startswith("ground/"))
        tools = {t: None for r in design.robots.values() for t in r.tools.values()}
        self.state = State(robots={r: None for r in design.robots}, present=grounds, tools=tools)
        self.movements = []

    def emit(self, id_, label, target=None, **parts):
        """Add a movement starting at the running state, then advance the state to its end."""
        self.movements.append(Movement(id=id_, label=label, start=self.state, target=target, **parts))
        s = self.state
        if target is not None:
            robots = dict(s.robots)
            for robot, joints in target.joints.items():
                robots[robot] = RobotState(robots[robot].base, dict(joints))
            for arm in parts.get("arms", ()):  # an arm motion without target joints ends unknown
                robot = "/".join(arm.split("/")[:2])
                if robot not in target.joints and robots[robot] is not None:
                    robots[robot] = RobotState(robots[robot].base, None)
            tools = {t: (replace(v, grip=target.tools[t]) if t in target.tools else v) for t, v in s.tools.items()}
            attached = s.attached if target.attached is None else target.attached
            built = s.built if target.built is None else target.built
            s = replace(s, robots=robots, tools=tools, attached=dict(attached), built=frozenset(built),
                        present=s.present | set(attached) | built)
        self.state = s

    def set(self, **changes):
        """Change the running state between movements (what `target` cannot say: `on`, robots arriving)."""
        self.state = replace(self.state, **changes)

    def on(self, **tool_on):
        tools = dict(self.state.tools)
        for tool, body in tool_on.items():
            tools[tool] = replace(tools[tool], on=body)
        self.set(tools=tools)

    # * Cindy: jointing (merged insert) and release (form B, no untighten).
    def jointing(self, bar, r):
        p = bar.split("/")[1]
        robots = dict(self.state.robots)
        robots[CINDY] = RobotState(r["base"], None)  # drove to this bar's base: joints unknown
        tools = dict(self.state.tools)
        for flange in (L, R):
            tools[AT[flange]] = tools[AT[flange]] or ToolState("open", None)
        self.set(robots=robots, tools=tools)
        both = (L, R)
        self.emit(f"{p}_J_load", "Free to the loading pose", arms=both, path="free", controller="position")
        self.emit(f"{p}_J_mount", "Operator mounts the bar", ends_on="operator",
                  target=Target(attached={**self.state.attached,
                                          bar: tuple(Holder(f, r["grasps"][f]) for f in both)}))
        self.on(**{AT[f]: r["on"][f] for f in both})
        self.emit(f"{p}_J_grasp", "Grasp", ends_on="tools", target=Target(tools={AT[f]: "closed" for f in both}))
        self.emit(f"{p}_J_transfer", "Transfer to the approach", arms=both, path="free", coupled=True,
                  controller="position",
                  target=Target(joints={CINDY: r["approach"]} if r["approach"] else {}, links=r["approach_links"]))
        self.emit(f"{p}_J_insert", "Insert", arms=both, path="linear", coupled=True, controller="compliant",
                  ends_on="tools", drives={AT[f]: "tighten" for f in both}, line=r["insert_line"],
                  target=Target(joints={CINDY: r["assembled"]} if r["assembled"] else {},
                                links=r["assembled_links"], built=self.state.built | {bar}))

    def release(self, bar, r):
        p = bar.split("/")[1]
        both = (L, R)
        others = tuple(h for h in self.state.attached[bar] if h.to not in both)
        attached = {k: v for k, v in self.state.attached.items() if k != bar}
        if others:
            attached[bar] = others
        self.emit(f"{p}_R_ungrasp", "Ungrasp (form B)", ends_on="tools",
                  target=Target(tools={AT[f]: "open" for f in both}, attached=attached))
        self.emit(f"{p}_R_retreat", "Careful retreat", arms=both, path="linear", controller="position",
                  line=r["retreat_line"],
                  target=Target(joints={CINDY: r["retreat"]} if r["retreat"] else {}, links=r["retreat_links"]))
        self.on(**{AT[f]: None for f in both})
        self.emit(f"{p}_R_home", "Home", arms=both, path="free", controller="position",
                  target=Target(joints={CINDY: r["home"]}))

    # * Support robot: hold (grasps a built bar Cindy still holds) and release.
    def holding(self, bar, h):
        p, robot, flange, tool = bar.split("/")[1], h["robot"], h["flange"], h["tool"]
        robots, tools = dict(self.state.robots), dict(self.state.tools)
        robots[robot] = RobotState(h["base"], None)
        tools[tool] = ToolState(None, None)  # arrives; grip unknown until opened
        self.set(robots=robots, tools=tools)
        self.emit(f"{p}_H_approach", "Free to the approach", arms=(flange,), path="free", controller="position",
                  target=Target(joints={robot: h["approach"]}, links=h["approach_links"]))
        self.emit(f"{p}_H_open", "Gripper opens", ends_on="tools", target=Target(tools={tool: "open"}))
        self.emit(f"{p}_H_to_grasp", "Linear onto the bar", arms=(flange,), path="linear", controller="position",
                  line=h["line"], target=Target(joints={robot: h["held"]}, links=h["held_links"]))
        self.on(**{tool: bar})
        attached = dict(self.state.attached)
        attached[bar] = attached.get(bar, ()) + (Holder(flange, h["grasp"]),)
        self.emit(f"{p}_H_close", "Gripper closes", ends_on="tools",
                  target=Target(tools={tool: "closed"}, attached=attached))

    def holding_release(self, bar, h):
        p, robot, flange, tool = bar.split("/")[1], h["robot"], h["flange"], h["tool"]
        attached = dict(self.state.attached)
        rest = tuple(x for x in attached[bar] if x.to != flange)
        attached = {k: v for k, v in attached.items() if k != bar}
        if rest:
            attached[bar] = rest
        self.emit(f"{p}_HR_open", "Gripper opens", ends_on="tools", target=Target(tools={tool: "open"},
                                                                                   attached=attached))
        self.emit(f"{p}_HR_retreat", "Linear retreat", arms=(flange,), path="linear", controller="position",
                  line=h["retreat_line"], target=Target(joints={robot: h["approach"]}, links=h["approach_links"]))
        robots, tools = dict(self.state.robots), dict(self.state.tools)
        robots[robot], tools[tool] = None, None  # drives away
        self.set(robots=robots, tools=tools)


def build(design: Design) -> Design:
    bars, holds = records(design)
    builder = Builder(design)
    actions, schedule = {}, []
    for action_id in design.schedule:  # the schedule Rhino's hold_schedule.build_action_schedule derives
        a = design.actions[action_id]
        kind = {"bar_jointing": "J", "bar_release": "R", "bar_holding": "H", "bar_holding_release": "HR"}[a.type]
        first = len(builder.movements)
        {"J": lambda: builder.jointing(a.bar, bars[a.bar]), "R": lambda: builder.release(a.bar, bars[a.bar]),
         "H": lambda: builder.holding(a.bar, holds[a.bar]),
         "HR": lambda: builder.holding_release(a.bar, holds[a.bar])}[kind]()
        new_id = f"{a.bar.split('/')[1]}_{kind}"
        actions[new_id] = Action(id=new_id, type=a.type, robot=a.robot, bar=a.bar,
                                 movements=tuple(builder.movements[first:]), ground=a.ground,
                                 supports_until=a.supports_until, label=a.label)
        schedule.append(new_id)
    return replace(design, actions=actions, schedule=tuple(schedule), folder=None)


if __name__ == "__main__":
    scratch = Path(sys.argv[1])
    for name in ("260814_RobArch_support_ik", "260920_RobArch_demo_revamp_backup"):
        source = read(scratch / "converted" / name)
        built = build(source)
        validate(built, check_robot_meshes=False)
        report = check_plan(built)
        n = sum(len(a.movements) for a in built.actions.values())
        print(f"{name}: {len(built.actions)} actions, {n} movements; A ok; B {len(report.errors)} errors, "
              f"{len(report.warnings)} warnings")
        for e in report.errors:
            print("   E", e[:170])
        kinds = {}
        for w in report.warnings:
            kinds.setdefault(w.split(":")[0], []).append(w)
        for k, v in kinds.items():
            print("   W", k, len(v), "e.g.", v[0][:150])
