"""Client wrapper for the scaffolding_tool_driver running on the husky onboard PC.

One ScaffoldingToolClient per arm. Exposes per-motor tighten/loosen/stop +
config get/set + a cached last-known status from the driver's status topic.

Defensive imports: if scaffolding_tool_msgs is not built yet, the client
becomes a no-op stub so the monitor still launches.
"""

from typing import Optional

try:
    from scaffolding_tool_msgs.msg import ToolStatus
    from scaffolding_tool_msgs.srv import ToolCommand, ToolConfig
    _MSGS_AVAILABLE = True
except ImportError:
    ToolStatus = None
    ToolCommand = None
    ToolConfig = None
    _MSGS_AVAILABLE = False


# Mirror constants for callers (so they don't have to import scaffolding_tool_msgs)
VERB_PING = 0
VERB_VERSION = 1
VERB_STATUS = 2
VERB_TIGHTEN = 3
VERB_LOOSEN = 4
VERB_STOP = 5
VERB_RESET_CONFIG = 6

MOTOR_NONE = 0
MOTOR_M1 = 1
MOTOR_M2 = 2
MOTOR_BOTH = 3

OP_SET = 0
OP_GET = 1


def _motor_enum(motor) -> int:
    if isinstance(motor, int):
        return motor
    s = str(motor).upper()
    return {"M1": MOTOR_M1, "M2": MOTOR_M2, "BOTH": MOTOR_BOTH, "NONE": MOTOR_NONE}.get(s, MOTOR_NONE)


class ScaffoldingToolClient:
    """One per arm. Spins under the parent rclpy node's executor."""

    def __init__(self, node, robot_namespace: str, arm_label: str):
        self._node = node
        self._arm_label = arm_label
        self._available = _MSGS_AVAILABLE
        self.status: Optional["ToolStatus"] = None

        if not self._available:
            node.get_logger().warn(
                f"scaffolding_tool_msgs not available — {arm_label} client is a no-op stub")
            return

        ns = f"{robot_namespace.rstrip('/')}/{arm_label}/scaffolding_tool_driver"
        self._cmd_cli = node.create_client(ToolCommand, f"{ns}/command")
        self._cfg_cli = node.create_client(ToolConfig, f"{ns}/config")
        self._sub_status = node.create_subscription(
            ToolStatus, f"{ns}/status", self._on_status, 10)
        node.get_logger().info(f"ScaffoldingToolClient ready: {ns}")

    def _on_status(self, msg):
        self.status = msg

    # ---------- async fire-and-forget commands ---------------------------------

    def _send_cmd(self, verb: int, motor=MOTOR_NONE):
        if not self._available:
            return None
        if not self._cmd_cli.service_is_ready():
            self._node.get_logger().warn(
                f"[{self._arm_label}] command service not ready")
            return None
        req = ToolCommand.Request()
        req.verb = verb
        req.motor = _motor_enum(motor)
        return self._cmd_cli.call_async(req)

    def tighten(self, motor=MOTOR_M1):
        return self._send_cmd(VERB_TIGHTEN, motor)

    def loosen(self, motor=MOTOR_M1):
        return self._send_cmd(VERB_LOOSEN, motor)

    def stop(self, motor=MOTOR_NONE):
        # STOP firmware verb stops both motors regardless of motor arg
        return self._send_cmd(VERB_STOP, motor)

    def ping(self):
        return self._send_cmd(VERB_PING, MOTOR_NONE)

    def request_status_now(self):
        return self._send_cmd(VERB_STATUS, MOTOR_NONE)

    def reset_config(self):
        return self._send_cmd(VERB_RESET_CONFIG, MOTOR_NONE)

    # ---------- config get/set -------------------------------------------------

    def set_param(self, param: str, motor, values):
        if not self._available or not self._cfg_cli.service_is_ready():
            return None
        req = ToolConfig.Request()
        req.op = OP_SET
        req.param = param
        req.motor = _motor_enum(motor)
        req.values = list(values)
        return self._cfg_cli.call_async(req)

    def get_param(self, param: str, motor):
        if not self._available or not self._cfg_cli.service_is_ready():
            return None
        req = ToolConfig.Request()
        req.op = OP_GET
        req.param = param
        req.motor = _motor_enum(motor)
        req.values = []
        return self._cfg_cli.call_async(req)

    # ---------- pretty-printed status for GUI labels --------------------------

    @staticmethod
    def state_str(state_enum: int) -> str:
        # Mirrors ToolStatus.STATE_* constants
        return {0: "UNK", 1: "IDLE", 2: "TIGHT", 3: "LOOSE", 4: "STALL"}.get(state_enum, "?")

    def status_summary(self) -> str:
        s = self.status
        if s is None:
            return f"{self._arm_label}: no data"
        link = "OK" if s.link_ok else "DOWN"
        return (f"{self._arm_label}[{link}] "
                f"M1={self.state_str(s.m1_state)} M2={self.state_str(s.m2_state)} "
                f"cur={s.current_ma}mA pwm={s.pwm_pct}%")
