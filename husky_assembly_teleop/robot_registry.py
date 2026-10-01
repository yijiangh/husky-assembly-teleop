"""One table describing the three huskies: Cindy (assembly), Alice and Belle (support).

Everything that differs between the robots -- ROS namespace and domain, mocap id,
one arm or two, which RobotCell file to plan in, joint / link / tool names, URDF
and SRDF files -- lives in ONE ``RobotSpec`` per robot, so the rest of the code
asks the spec instead of branching on ``dual_arm`` or a hard-coded serial.

* The names match the Rhino exporter VERBATIM: ``Action.robot_id`` is
* ``dual-arm_husky_Cindy`` / ``single-arm_husky_Alice`` / ``single-arm_husky_Belle``,
* each robot appears in the other robots' cells as the tool ``ObstacleRobot<Name>``,
* and ``Movement.target_ee_frames`` is keyed ``left``/``right`` (Cindy) or ``arm``.

! Keep this module import-light (standard library + the package's own
! ``DATA_DIRECTORY`` only). It is read by UI code, tests and scripts that must not
! start PyBullet or compas_fab just to learn a robot's name. That is also why the
! joint names and URDF/SRDF paths are written out here instead of being imported
! from ``utils.py`` (imports pybullet at load time) or ``cfab_session.py``
! (imports pybullet_planning and compas_fab at load time).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping, Tuple, Union

from husky_assembly_teleop import DATA_DIRECTORY

# * ------------------------------------------------------------- shared names
ROLE_ASSEMBLY = 'assembly'
ROLE_SUPPORT = 'support'

# The six UR5e joints, shoulder to wrist. A robot's joint names are these with
# its arm prefix in front: '' for a single-arm husky, 'left_'/'right_' for Cindy.
_UR_JOINT_SUFFIXES = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)

# Same strings as cfab_session.py's HUSKY_DUAL_URDF_PATH / HUSKY_DUAL_SRDF_PATH /
# HUSKY_SINGLE_URDF_PATHS / HUSKY_SINGLE_SRDF_PATHS (see the module note for why
# they are not imported from there).
_DUAL_URDF_PATH = os.path.join(
    DATA_DIRECTORY,
    'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/'
    'husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf')
_DUAL_SRDF_PATH = os.path.join(
    DATA_DIRECTORY,
    'husky_urdf/mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf')
_SINGLE_URDF_DIR = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/urdf')
_SINGLE_SRDF_DIR = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config')


@dataclass(frozen=True)
class RobotSpec:
    """Everything the monitor needs to know about one husky.

    Per-side tuples (``planning_groups``, ``side_keys``, ``arm_joint_names``,
    ``flange_links``, ``arm_base_links``, ``tool_names``) are all in the SAME side
    order, so index ``i`` of each describes the same arm.

    Attributes:
        name (str): Short name, e.g. ``'Cindy'`` (the ActionSchedule's robot key).
        robot_id (str): ``Action.robot_id`` in the exported JSON, e.g.
            ``'dual-arm_husky_Cindy'``.
        role (str): ``'assembly'`` or ``'support'``.
        namespace (str): ROS namespace, e.g. ``'/a200_0806'``.
        domain_id (str): ROS_DOMAIN_ID (a string, like environment variables).
        mocap_id (int): Mocap rigid-body id of the robot base.
        dual_arm (bool): True for Cindy.
        cell_file (str): The robot's own RobotCell file in the design problem.
        planning_groups (tuple): compas_fab planning group per side.
        side_keys (tuple): Keys of ``Movement.target_ee_frames``, one per side.
        arm_joint_names (tuple): One 6-tuple of joint names per side.
        flange_links (tuple): The tool0 link per side.
        arm_base_links (tuple): The arm base link per side.
        tool_names (tuple): The tool(s) this robot carries in its own cell.
        obstacle_tool_name (str): How this robot appears in the OTHER robots' cells.
        gripper_kind (str): ``'scaffolding'`` (Cindy's screw tools) or ``'robotiq'``.
        ee_types (tuple): Default end-effector meshes for the PyBullet viewer
            (``husky_world.init``).
        connect_gripper (bool): Whether ``husky_world.init`` connects the gripper.
        urdf_path (str): Calibrated URDF.
        srdf_path (str): SRDF holding the planning groups.
        rb_prefix (str): Prefix of the built bars' rigid-body names in this
            robot's cell: ``''`` (``bar_B3``) or ``'env_'`` (``env_bar_B3``).
    """

    name: str
    robot_id: str
    role: str
    namespace: str
    domain_id: str
    mocap_id: int
    dual_arm: bool
    cell_file: str
    planning_groups: tuple
    side_keys: tuple
    arm_joint_names: tuple
    flange_links: tuple
    arm_base_links: tuple
    tool_names: tuple
    obstacle_tool_name: str
    gripper_kind: str
    ee_types: tuple
    connect_gripper: bool
    urdf_path: str
    srdf_path: str
    rb_prefix: str

    @property
    def serial(self) -> str:
        """The husky's serial number, e.g. ``'0806'`` for ``'/a200_0806'``.

        Returns:
            str: The part of the namespace after the last underscore.
        """
        return self.namespace.rsplit('_', 1)[-1]

    @property
    def all_arm_joint_names(self) -> list:
        """Every arm joint of the robot in side order (12 for Cindy, 6 otherwise).

        Returns:
            list: Flat list of joint names.
        """
        return [name for side_names in self.arm_joint_names for name in side_names]

    @property
    def n_arms(self) -> int:
        """How many arms the robot has.

        Returns:
            int: 2 for Cindy, 1 for a support robot.
        """
        return len(self.side_keys)

    def side_of_link(self, link: str) -> str:
        """Which side a flange (tool0) or arm base link belongs to.

        Args:
            link (str): e.g. ``'left_ur_arm_tool0'`` or ``'ur_arm_base_link'``.

        Returns:
            str: The side key, e.g. ``'left'`` or ``'arm'``.

        Raises:
            KeyError: The link is not one of this robot's flange or arm base links.
        """
        by_link = {}
        for side, flange, base in zip(self.side_keys, self.flange_links, self.arm_base_links):
            by_link[flange] = side
            by_link[base] = side
        if link not in by_link:
            raise KeyError(f"{link!r} is not a flange or arm base link of {self.name}; "
                           f"valid: {sorted(by_link)}")
        return by_link[link]

    def group_for_side(self, side: str) -> str:
        """The compas_fab planning group that drives one side's arm.

        Args:
            side (str): A key of ``side_keys``.

        Returns:
            str: e.g. ``'base_left_arm_manipulator'`` or ``'manipulator'``.
        """
        return self.planning_groups[self._side_index(side)]

    def flange_for_side(self, side: str) -> str:
        """The tool0 link of one side's arm.

        Args:
            side (str): A key of ``side_keys``.

        Returns:
            str: e.g. ``'right_ur_arm_tool0'`` or ``'ur_arm_tool0'``.
        """
        return self.flange_links[self._side_index(side)]

    def base_calibration_filename(self, convention: str = 'rotated') -> str:
        """File name of the robot's base (mocap) calibration, as husky_world.init reads it.

        Args:
            convention (str): The monitor's MOCAP_AXIS_CONVENTION. ``'rhino'`` picks
                the ``_rhino``-tagged file; anything else the untagged one.

        Returns:
            str: e.g. ``'calibrated_transformation_0806.json'`` or
            ``'calibrated_transformation_0806_rhino.json'``.
        """
        suffix = '_rhino' if convention == 'rhino' else ''
        return f'calibrated_transformation_{self.serial}{suffix}.json'

    def _side_index(self, side: str) -> int:
        """Position of a side key in the per-side tuples.

        Args:
            side (str): A key of ``side_keys``.

        Returns:
            int: Its index.

        Raises:
            KeyError: The side does not exist on this robot.
        """
        if side not in self.side_keys:
            raise KeyError(f"{self.name} has no side {side!r}; valid: {list(self.side_keys)}")
        return self.side_keys.index(side)


def _arm_joint_names(prefix: str) -> Tuple[str, ...]:
    """The six UR5e joint names of one arm.

    Args:
        prefix (str): ``''``, ``'left_'`` or ``'right_'``.

    Returns:
        tuple: Six joint names, shoulder to wrist.
    """
    return tuple(f'{prefix}ur_arm_{suffix}' for suffix in _UR_JOINT_SUFFIXES)


def _assembly_robot(name: str, namespace: str, domain_id: str, mocap_id: int) -> RobotSpec:
    """Spec of the dual-arm assembly husky.

    Args:
        name (str): Short name, e.g. ``'Cindy'``.
        namespace (str): ROS namespace.
        domain_id (str): ROS_DOMAIN_ID.
        mocap_id (int): Mocap rigid-body id.

    Returns:
        RobotSpec: The robot's spec.
    """
    prefixes = ('left_', 'right_')
    return RobotSpec(
        name=name,
        robot_id=f'dual-arm_husky_{name}',
        role=ROLE_ASSEMBLY,
        namespace=namespace,
        domain_id=domain_id,
        mocap_id=mocap_id,
        dual_arm=True,
        cell_file='RobotCell.json',
        planning_groups=('base_left_arm_manipulator', 'base_right_arm_manipulator'),
        side_keys=('left', 'right'),
        arm_joint_names=tuple(_arm_joint_names(p) for p in prefixes),
        flange_links=tuple(f'{p}ur_arm_tool0' for p in prefixes),
        arm_base_links=tuple(f'{p}ur_arm_base_link' for p in prefixes),
        tool_names=('AT3L', 'AT3R'),
        obstacle_tool_name=f'ObstacleRobot{name}',
        gripper_kind='scaffolding',
        ee_types=('assembly_tool_v3_left', 'assembly_tool_v3_right'),
        connect_gripper=False,
        urdf_path=_DUAL_URDF_PATH,
        srdf_path=_DUAL_SRDF_PATH,
        rb_prefix='',
    )


def _support_robot(name: str, namespace: str, domain_id: str, mocap_id: int,
                   srdf_file: str) -> RobotSpec:
    """Spec of a single-arm support husky.

    Args:
        name (str): Short name, e.g. ``'Alice'``.
        namespace (str): ROS namespace.
        domain_id (str): ROS_DOMAIN_ID.
        mocap_id (int): Mocap rigid-body id.
        srdf_file (str): SRDF file name inside mt_husky_moveit_config/config.

    Returns:
        RobotSpec: The robot's spec.
    """
    return RobotSpec(
        name=name,
        robot_id=f'single-arm_husky_{name}',
        role=ROLE_SUPPORT,
        namespace=namespace,
        domain_id=domain_id,
        mocap_id=mocap_id,
        dual_arm=False,
        cell_file=f'RobotCell_{name}.json',
        # ? The SRDF also defines 'base_arm_manipulator' (same 6 joints), but the
        # ? design cells attach SupportGripper to 'manipulator', so plan with that.
        planning_groups=('manipulator',),
        side_keys=('arm',),
        arm_joint_names=(_arm_joint_names(''),),
        flange_links=('ur_arm_tool0',),
        arm_base_links=('ur_arm_base_link',),
        tool_names=('SupportGripper',),
        obstacle_tool_name=f'ObstacleRobot{name}',
        gripper_kind='robotiq',
        ee_types=('robotiq_gripper',),
        connect_gripper=True,
        urdf_path=os.path.join(_SINGLE_URDF_DIR, f'husky_ur5_e_no_base_joint_{name}_Calibrated.urdf'),
        srdf_path=os.path.join(_SINGLE_SRDF_DIR, srdf_file),
        rb_prefix='env_',
    )


# * ------------------------------------------------------------- the table
# Namespaces, domains and mocap ids as in husky_world.init's ROBOT_CONFIGS.
# Insertion order (Cindy, Alice, Belle) is the stable order other_robots() uses.
ROBOTS = {
    spec.name: spec for spec in (
        _assembly_robot('Cindy', '/a200_0806', '86', 1860),
        _support_robot('Alice', '/a200_0804', '84', 1840, 'husky.srdf'),
        _support_robot('Belle', '/a200_0805', '85', 1850, 'belle.srdf'),
    )
}

# The robot used when ROS_DOMAIN_ID is missing or unknown (as husky_world.init).
DEFAULT_DOMAIN_ID = '86'


# * ------------------------------------------------------------- lookups
def _lookup(field: str, value) -> RobotSpec:
    """Find the one robot whose ``field`` equals ``value``.

    Args:
        field (str): A RobotSpec attribute or property name.
        value: The value to match.

    Returns:
        RobotSpec: The matching robot.

    Raises:
        KeyError: No robot matches; the message lists the valid values.
    """
    for spec in ROBOTS.values():
        if getattr(spec, field) == value:
            return spec
    valid = [getattr(spec, field) for spec in ROBOTS.values()]
    raise KeyError(f"no robot with {field}={value!r}; valid: {valid}")


def robot_by_name(name: str) -> RobotSpec:
    """Look a robot up by its short name.

    Args:
        name (str): e.g. ``'Alice'``.

    Returns:
        RobotSpec: The robot.
    """
    return _lookup('name', name)


def robot_by_id(robot_id: str) -> RobotSpec:
    """Look a robot up by the exporter's ``Action.robot_id``.

    Args:
        robot_id (str): e.g. ``'single-arm_husky_Alice'``.

    Returns:
        RobotSpec: The robot.
    """
    return _lookup('robot_id', robot_id)


def robot_by_namespace(namespace: str) -> RobotSpec:
    """Look a robot up by its ROS namespace.

    Args:
        namespace (str): e.g. ``'/a200_0804'``.

    Returns:
        RobotSpec: The robot.
    """
    return _lookup('namespace', namespace)


def robot_by_domain_id(domain_id: Union[int, str]) -> RobotSpec:
    """Look a robot up by its ROS_DOMAIN_ID.

    Args:
        domain_id (int | str): e.g. ``85`` or ``'85'``.

    Returns:
        RobotSpec: The robot.
    """
    return _lookup('domain_id', str(domain_id).strip())


def robot_by_serial(serial: str) -> RobotSpec:
    """Look a robot up by its serial number.

    Args:
        serial (str): e.g. ``'0806'``.

    Returns:
        RobotSpec: The robot.
    """
    return _lookup('serial', serial)


def robot_from_env(default_domain_id: str = DEFAULT_DOMAIN_ID,
                   environ: Mapping = os.environ) -> Tuple[RobotSpec, bool]:
    """The robot this monitor run is connected to, from ROS_DOMAIN_ID.

    Args:
        default_domain_id (str): Domain used when ROS_DOMAIN_ID is missing or
            names no known robot.
        environ (Mapping): Where to read ROS_DOMAIN_ID (the process environment
            by default; tests pass a plain dict).

    Returns:
        tuple: ``(spec, was_default)``; ``was_default`` is True when the default
        robot was used because ROS_DOMAIN_ID was missing or unknown, so the
        caller can warn.
    """
    domain_id = environ.get('ROS_DOMAIN_ID')
    if domain_id is not None:
        try:
            return robot_by_domain_id(domain_id), False
        except KeyError:
            pass  # an unknown domain falls back to the default robot below
    return robot_by_domain_id(default_domain_id), True


def other_robots(name: str) -> list:
    """Every robot except ``name``, in the table's stable order.

    Args:
        name (str): The robot to leave out, e.g. ``'Cindy'``.

    Returns:
        list: RobotSpecs.
    """
    robot_by_name(name)  # fail loudly on a typo instead of returning all three
    return [spec for spec in ROBOTS.values() if spec.name != name]


def robot_name_from_id(robot_id: str) -> str:
    """Short robot name from the exporter's ``Action.robot_id``.

    Args:
        robot_id (str): e.g. ``'single-arm_husky_Alice'``.

    Returns:
        str: e.g. ``'Alice'``.
    """
    return robot_by_id(robot_id).name
