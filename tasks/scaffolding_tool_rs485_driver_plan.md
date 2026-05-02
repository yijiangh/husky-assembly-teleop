# Scaffolding Tool — RS485 ROS2 Driver + Monitor Client

## Context

Replace the current IO-toggle gripper/screw control (UR `SetIO` flipping `PIN_TOOL_DOUT0/1`) with a real driver for Victor's `scaffolding_tool_controller` board, mounted on each UR flange and wired to the UR tool RS485 pins.

- Three PCs: monitor (PyBullet GUI, this dev box) ↔ husky onboard (ROS2 stack, exposes UR tool RS485 as a tty) ↔ UR controller (wired to husky onboard). Monitor↔husky is wifi only → all comms via ROS2.
- Firmware protocol: ASCII line, LF-terminated, 115200 8N1, half-duplex, single tool per RS485 segment, no addressing, 100 ms response timeout. Verbs: `PING / VERSION / STATUS / TIGHTEN M1|M2 / LOOSEN M1|M2 / STOP / SET … / GET … / RESET CONFIG`. STATUS reply: `OK <m1_state> <m2_state> <current_mA> <pwm_pct>`. States: IDLE/TIGHTENING/LOOSENING/STALLED. Errors: `ERR <1-5> <msg>`.
- Old test script `supporting_materials/scripts/rs485_huksy_side_test.py` — verb names there are stale, but serial open/send/recv pattern is reusable.

## Locked-in choices (already confirmed)
1. **Dual**: one tool per arm → two driver-node instances on husky onboard, separate namespaces (`/a200_0804/left_tool`, `/a200_0804/right_tool`).
2. **Repo**: new git repo `ros2_ws/src/scaffolding_tool/` containing two pkgs (`scaffolding_tool_msgs` ament_cmake + `scaffolding_tool_driver` ament_python). Same nested layout as `crl_husky_msgs/crl_husky_msgs/`.
3. **Cmd surface**: `ToolCommand.srv` for verbs + `ToolStatus.msg` topic @ ~10 Hz. `ToolConfig.srv` for firmware SET/GET (NOT ROS param callbacks — see Design note D2).
4. **Serial path**: default `/tmp/ttyUR`, configurable as launch param.

## Open hardware question (flag, don't block)
Protocol is "single tool per RS485 segment, no addressing". Two arms = **two physical RS485 buses → two tty devices** on husky onboard (e.g. `/tmp/ttyUR_L`, `/tmp/ttyUR_R`). Confirm with hardware integrator before field test. Driver must refuse to open a port already locked (use `serial.Serial(..., exclusive=True)`).

---

## New repo structure

```
ros2_ws/src/scaffolding_tool/                    # new git repo
├── .gitignore  LICENSE  README.md
├── scaffolding_tool_msgs/                       # ament_cmake (clone crl_husky_msgs/crl_husky_msgs/ as template)
│   ├── CMakeLists.txt  package.xml
│   ├── msg/ToolStatus.msg
│   └── srv/{ToolCommand.srv, ToolConfig.srv}
└── scaffolding_tool_driver/                     # ament_python
    ├── package.xml  setup.py  setup.cfg  resource/scaffolding_tool_driver
    ├── scaffolding_tool_driver/
    │   ├── __init__.py
    │   ├── protocol.py          # pure parser/encoder, no ROS
    │   ├── serial_link.py       # threaded pyserial wrapper, no ROS
    │   ├── driver_node.py       # rclpy Node
    │   ├── fake_firmware.py     # mock for socat tests
    │   └── main.py              # console_script
    ├── launch/{single_tool.launch.py, dual_tool.launch.py}
    ├── config/default_params.yaml
    └── test/{test_protocol.py, test_serial_link_socat.py, test_driver_node.py,
              test_copyright.py, test_flake8.py, test_pep257.py}
```

## Interfaces (`scaffolding_tool_msgs`)

**`msg/ToolStatus.msg`**:
```
std_msgs/Header header
uint8 STATE_UNKNOWN=0  STATE_IDLE=1  STATE_TIGHTENING=2  STATE_LOOSENING=3  STATE_STALLED=4
uint8  m1_state
uint8  m2_state
uint16 current_ma
uint8  pwm_pct
bool   link_ok               # last STATUS round-trip ok
uint32 consecutive_errors
string last_error
```

**`srv/ToolCommand.srv`** (every verb whose payload is just `<verb> [M1|M2]`):
```
uint8 VERB_PING=0  VERB_VERSION=1  VERB_STATUS=2
uint8 VERB_TIGHTEN=3  VERB_LOOSEN=4  VERB_STOP=5  VERB_RESET_CONFIG=6
uint8 MOTOR_NONE=0  MOTOR_M1=1  MOTOR_M2=2  MOTOR_BOTH=3   # BOTH = driver fans out
uint8 verb
uint8 motor
---
bool   success            # firmware replied OK
string raw_reply          # full line — useful for VERSION text
uint8  err_code           # 0 if ok, else 1..5
string err_msg
```

**`srv/ToolConfig.srv`** (firmware SET/GET; replaces ROS param callback for hw config):
```
uint8 OP_SET=0  OP_GET=1
uint8 op
string  param            # firmware name e.g. "MAX_CURRENT", "STALL_TIME"
uint8   motor            # MOTOR_M1 / MOTOR_M2
int32[] values           # SET payload, ignored on GET
---
bool    success
int32[] values           # GET reply parsed; empty on SET
string  raw_reply
uint8   err_code
string  err_msg
```

`CMakeLists.txt` mirrors `crl_husky_msgs/crl_husky_msgs/CMakeLists.txt`; `rosidl_generate_interfaces` lists the three files with `DEPENDENCIES std_msgs builtin_interfaces`. `package.xml`: `<depend>std_msgs</depend>`, `<depend>builtin_interfaces</depend>`, `rosidl_default_*` boilerplate.

## Driver (`scaffolding_tool_driver`)

### Threading + executor model (D1)
- **`SerialLink` owns a `threading.Lock` (bus mutex)** — every write+readline pair acquires it. This is the single source of truth for half-duplex correctness.
- Reads happen on a dedicated **IO worker thread** doing blocking `readline()`. Writes go via `SerialLink.request(line, timeout) -> ToolReply` which: lock → flush input → write → wait `threading.Event` set by worker on next line → return parsed reply or raise `ToolTimeout`.
- Driver node uses **`MultiThreadedExecutor` (4 threads) + single `ReentrantCallbackGroup`** for status timer + service callbacks. The bus mutex inside `SerialLink` serialises hardware access; the executor just keeps the Python-level callbacks from blocking each other.
- IO thread auto-reconnects on `serial.SerialException` with exp backoff capped at 5 s.
- Worker logs WARN if a line arrives without a pending request (defensive — protocol says no unsolicited output).

### `protocol.py` (pure, no ROS, no serial)
- `@dataclass ToolReply`: `ok`, `m1_state`, `m2_state`, `current_ma`, `pwm_pct`, `err_code`, `err_msg`, `raw`, `tokens`.
- `STATE_MAP = {"IDLE":1, "TIGHTENING":2, "LOOSENING":3, "STALLED":4}` (matches `ToolStatus.msg` enum).
- `encode_command(verb, motor=None, args=None) -> bytes` → `b"TIGHTEN M1\n"`, `b"SET MAX_CURRENT M2 900\n"`. Always `\n` terminated.
- `parse_reply(line:str) -> ToolReply`: strip CR/LF; tokenise; first token `OK` or `ERR`; STATUS path parses 4 trailing tokens; otherwise tokens kept raw for VERSION etc. Empty/malformed → `ok=False, err_code=0xFF, err_msg="malformed"`.

### `serial_link.py` (no ROS; pyserial)
- ctor: `port`, `baud=115200`, `read_timeout=0.1`, `command_timeout=0.5`, `logger=print`.
- `connect()` opens `serial.Serial(port=…, exclusive=True, timeout=read_timeout)`. `disconnect()`, `is_connected`.
- `request(verb, motor=None, args=None, timeout=None) -> ToolReply` — composes via `protocol.encode_command`, runs the lock/write/wait dance, returns parsed reply, raises on timeout or transport error.
- Internal `_io_loop` thread: while `_alive`: try `readline`; if non-empty parse + put on `_pending_reply` queue + set the request `Event`; on `SerialException` → close + sleep + reconnect.

### `driver_node.py`
- `class ScaffoldingToolDriver(Node)`:
  - **Params** (`declare_parameter`): `port` (str, `/tmp/ttyUR`), `baud` (int, 115200), `status_rate_hz` (float, 10.0), `command_timeout_s` (float, 0.5), `connect_on_start` (bool, true), `fail_fast_on_startup` (bool, false), `tool_label` (str, `tool` — for log prefix only).
  - Construct `SerialLink`. If `fail_fast_on_startup` and connect fails → raise (kills proc); else schedule async reconnect.
  - **Service `~/command`** (`ToolCommand`): map verb enum → ASCII → `serial_link.request(...)`. For `MOTOR_BOTH`, fan out two calls; success only if both OK.
  - **Service `~/config`** (`ToolConfig`): SET → `f"SET {param} M{motor} {' '.join(values)}"`; GET → `f"GET {param} M{motor}"`. Parse trailing ints into `values[]`.
  - **Publisher `~/status`** (`ToolStatus`, depth 10) on a `Timer(1/status_rate_hz)`. Each tick: `serial_link.request("STATUS")`; on timeout → publish `link_ok=False`, increment `consecutive_errors`, fill `last_error`. On success → map states via `STATE_MAP`, publish `link_ok=True`.
  - Param callback (`add_on_set_parameters_callback`) handles only soft params (status_rate retunes the timer; command_timeout adjusts the SerialLink). **Does NOT push to firmware.**

### `main.py`
```python
def main():
    rclpy.init()
    node = ScaffoldingToolDriver()
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(node)
    try: ex.spin()
    finally: node.shutdown(); node.destroy_node(); rclpy.shutdown()
```

### `fake_firmware.py` (mock for socat tests)
- pyserial only, no ROS. Args: `--port`, `--baud`, `--stall-after-s` (default 2.0), optional `--inject-error-every`, `--latency-ms`.
- Loop `readline → handle → write reply`. Maintains `m1_state`, `m2_state`, mock `current_ma` (random 200–500), `pwm_pct`, in-memory `params` dict.
- TIGHTEN/LOOSEN flips state and arms a soft timer that flips to STALLED after `stall_after_s`. STOP zeroes both. STATUS prints `OK <s1> <s2> <cur> <pwm>`.

### `launch/dual_tool.launch.py`
Two `Node` actions, namespaces `/a200_0804/left_tool` + `/a200_0804/right_tool`, with port params `/tmp/ttyUR_L` + `/tmp/ttyUR_R` defaults. Comment: "Defaults assume two physical RS485 buses on husky onboard. Override `port` if hardware uses a different path."

### `setup.py` entry points
```
'scaffolding_tool_driver = scaffolding_tool_driver.main:main',
'scaffolding_tool_fake_firmware = scaffolding_tool_driver.fake_firmware:main',
```

### `package.xml` deps
`rclpy`, `std_msgs`, `scaffolding_tool_msgs`, `python3-serial` (rosdep key — NOT venv pip; this lets clean apt-install on husky onboard).

---

## Client-side integration (`husky_assembly_teleop`)

### New file: `husky_assembly_teleop/scaffolding_tool_client.py`
Keep separate from `husky_robot.py` (which is already 700+ lines mixing odom/arms/grippers). Localising the new msgs import here also keeps teleop runnable when the new pkg isn't built.

```python
class ScaffoldingToolClient:
    def __init__(self, node, robot_namespace, arm_label):
        ns = f"{robot_namespace}/{arm_label}"   # e.g. /a200_0804/left_tool
        self.cmd_cli  = node.create_client(ToolCommand, f"{ns}/command")
        self.cfg_cli  = node.create_client(ToolConfig,  f"{ns}/config")
        self.status   = None                    # last ToolStatus
        self.sub      = node.create_subscription(ToolStatus, f"{ns}/status", self._on_status, 10)

    def tighten(self, motor='M1'): ...   # async: returns rclpy Future like toggle_gripper did
    def loosen (self, motor='M1'): ...
    def stop   (self, motor='M1'): ...   # accepts 'BOTH'
    def ping   (self):              ...
    def request_status_now(self):   ...
    def set_param(self, name, motor, values): ...
    def get_param(self, name, motor):         ...
```

### `husky_robot.py` changes (`HuskyRobotInterface`)
- `__init__` (around line 175-188 where `setio_clients` is built): add
  ```python
  self.tool_clients = []
  if dual_arm:
      self.tool_clients.append(ScaffoldingToolClient(node, name, 'left_tool'))
      self.tool_clients.append(ScaffoldingToolClient(node, name, 'right_tool'))
  else:
      self.tool_clients.append(ScaffoldingToolClient(node, name, 'tool'))
  ```
- Add convenience methods on the class:
  - `tighten_tool(self, arm_index, motor='M1')`
  - `loosen_tool (self, arm_index, motor='M1')`
  - `stop_tool   (self, arm_index, motor='BOTH')`
  - `tool_status (self, arm_index) -> ToolStatus|None`
  Each delegates to `self.tool_clients[arm_index]`.
- **Deprecate** existing `toggle_gripper` (line 394) and `toggle_screw` (line 418): keep methods, route to no-op + one-time logger warning. Don't delete — git history + flag-back-on safety. Drop the `setio_clients` creation only after a real-hardware test pass (mark with `# TODO remove after RS485 driver field-tested`).

### `husky_monitor.py` button block (lines 1497-1511)
Replace the four toggle buttons with per-arm grid:

```python
if not self.FAKE_HARDWARE and not self.CALIBRATION:
    self.dump_sep_sliders.append(Slider("----------Scaffolding Tool (Left)", lambda: None))
    iface = lambda: self.huskies[self.selected_robot_id].interface
    for m in ('M1','M2'):
        self.buttons.append(Button(f'L {m} Tighten', lambda m=m: iface().tighten_tool(0, m)))
        self.buttons.append(Button(f'L {m} Loosen',  lambda m=m: iface().loosen_tool (0, m)))
        self.buttons.append(Button(f'L {m} Stop',    lambda m=m: iface().stop_tool   (0, m)))
    self.buttons.append(Button('L Ping',       lambda: iface().tool_clients[0].ping()))
    self.buttons.append(Button('L Reset Cfg',  lambda: iface().tool_clients[0].cmd_send(VERB_RESET_CONFIG)))

    if self.huskies[self.selected_robot_id].dual_arm:
        self.dump_sep_sliders.append(Slider("----------Scaffolding Tool (Right)", lambda: None))
        # same block with index 1, label 'R …'
```

Add a status readout via the slider-as-label idiom (already used elsewhere): each tick (`button update loop`, `husky_monitor.py:1824`), set the slider name from `iface().tool_clients[i].status` (e.g. `f"L M1: {state_str}  cur={cur}mA"`).

### Keyboard rebinding (`husky_monitor.py:1789-1801`)
Currently `1` toggles both grippers, `2` toggles both screws. New mapping (preserves muscle memory; adds **panic-stop**):

| key | action |
|---|---|
| `1`        | tighten M1 on both arms |
| `!` (Shift+1) | loosen M1 on both arms |
| `2`        | tighten M2 on both arms |
| `@` (Shift+2) | loosen M2 on both arms |
| `s` or backtick | **stop ALL motors on both arms** (panic) |

Panic-stop is **required** — TIGHTEN runs until firmware-detected stall, no DOUT-equivalent kill. Today there's no global stop; this is a regression to fix, not a feature.

---

## Workspace integration

- **Add submodule**: workspace `.gitmodules` already tracks `crl_husky_msgs`. Add `scaffolding_tool` the same way once you've pushed the new repo.
- **Build deps**: `apt install python3-serial` on monitor (and husky onboard at deployment). Add nothing to `husky-assembly-teleop/requirements.txt`.
- **husky-assembly-teleop/package.xml**: add `<exec_depend>scaffolding_tool_msgs</exec_depend>`.

---

## Critical files to modify (existing) / create

**Create (new repo)**
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_msgs/{CMakeLists.txt, package.xml, msg/ToolStatus.msg, srv/ToolCommand.srv, srv/ToolConfig.srv}`
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_driver/{package.xml, setup.py, setup.cfg, resource/…}`
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_driver/scaffolding_tool_driver/{__init__.py, protocol.py, serial_link.py, driver_node.py, fake_firmware.py, main.py}`
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_driver/launch/{single_tool.launch.py, dual_tool.launch.py}`
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_driver/config/default_params.yaml`
- `ros2_ws/src/scaffolding_tool/scaffolding_tool_driver/test/{test_protocol.py, test_serial_link_socat.py, test_driver_node.py}`

**Create (in husky-assembly-teleop)**
- `husky_assembly_teleop/scaffolding_tool_client.py`

**Modify**
- `husky_assembly_teleop/husky_robot.py` lines 175-188 (add `tool_clients`), add 4 convenience methods, deprecate `toggle_gripper`/`toggle_screw` at lines 394-440.
- `husky_assembly_teleop/husky_monitor.py` lines 1497-1511 (button block) and 1789-1801 (keyboard).
- `husky-assembly-teleop/package.xml` add `scaffolding_tool_msgs` exec_depend.

**Reuse (existing patterns)**
- Serial open/recv pattern from `supporting_materials/scripts/rs485_huksy_side_test.py` lines 9-30.
- Pkg layout from `crl_husky_msgs/crl_husky_msgs/{CMakeLists.txt, package.xml}`.
- `Button` / `Slider` classes at `husky_assembly_teleop/common.py:579-601`.
- Per-arm fan-out pattern from `husky_robot.py:175-188` (`setio_clients` list construction).

---

## Design notes (rationale, terse)

- **D1 Threading**: half-duplex bus + ROS callbacks → bus mutex inside SerialLink + `MultiThreadedExecutor`+`ReentrantCallbackGroup` in node. Single-threaded executor would let one slow service call starve the status publisher.
- **D2 ToolConfig.srv vs param callback for SET**: param callbacks must return fast and have no clean way to surface firmware ERR codes; they also fire on yaml load before serial is connected. Use explicit srv. ROS params reserved for driver-side config (port, baud, status rate, timeouts).
- **D3 Service vs Action for TIGHTEN-until-stall**: service ACKs receipt only; STALLED transition is observed via the status topic (≤100 ms latency at 10 Hz). Acceptable for human button-driven v1. For future scripted TAMP `await tighten_until_stall(M1)`, an Action layer can be added on top — keep enum surface stable so it's additive.
- **D4 Half-duplex correctness**: protocol has zero unsolicited firmware output; every read is the response to the last write. Bus mutex + worker-thread readline is sufficient. Log a WARN if an unsolicited line ever arrives (future-proof).
- **D5 Startup**: keep retrying by default; `fail_fast_on_startup` opt-in for CI/integration. Status topic publishes `link_ok=False` until connected — useful liveness signal for the monitor.
- **D6 Two physical buses**: only safe configuration with current protocol (no addressing). Driver opens its port with `exclusive=True` to surface conflicts loudly.
- **D7 `/tmp/ttyUR` default**: dev-acceptable; for production deploy a udev rule under `/dev/ttyUR_{L,R}`. Note in README.

---

## Verification

All commands from `/home/yijiangh/Code/ros2_ws`. Confirmed available: `socat` at `/usr/bin/socat`, ROS humble at `/opt/ros/humble`, venv at `ros2_ws/venv` with `rclpy`. **`pyserial` not yet installed** — add via `pip install pyserial==3.5` in the venv (and `apt install python3-serial` on husky onboard at deploy).

### Build
```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
pip install pyserial==3.5
colcon build --symlink-install --packages-select scaffolding_tool_msgs scaffolding_tool_driver
colcon build --symlink-install     # full ws to confirm nothing else broke
source install/setup.bash
ros2 interface show scaffolding_tool_msgs/msg/ToolStatus
ros2 interface show scaffolding_tool_msgs/srv/ToolCommand
ros2 interface show scaffolding_tool_msgs/srv/ToolConfig
```

### Local mock end-to-end (no real husky)

**term 1** — pty pair:
```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyTOOL_HOST pty,raw,echo=0,link=/tmp/ttyTOOL_FW
```

**term 2** — fake firmware:
```bash
source /opt/ros/humble/setup.bash; source venv/bin/activate; source install/setup.bash
ros2 run scaffolding_tool_driver scaffolding_tool_fake_firmware --port /tmp/ttyTOOL_FW --stall-after-s 2.0
```

**term 3** — driver in left_tool namespace:
```bash
source /opt/ros/humble/setup.bash; source venv/bin/activate; source install/setup.bash
ros2 run scaffolding_tool_driver scaffolding_tool_driver \
  --ros-args -r __ns:=/a200_0804/left_tool \
             -p port:=/tmp/ttyTOOL_HOST -p status_rate_hz:=10.0
```

**term 4** — observe status:
```bash
ros2 topic echo /a200_0804/left_tool/status
# expect ~10 Hz, link_ok=true, m1_state=1 (IDLE)
```

**term 5** — round-trip service calls:
```bash
ros2 service call /a200_0804/left_tool/command scaffolding_tool_msgs/srv/ToolCommand "{verb: 0, motor: 0}"   # PING
ros2 service call /a200_0804/left_tool/command scaffolding_tool_msgs/srv/ToolCommand "{verb: 3, motor: 1}"   # TIGHTEN M1
# topic should show STATE_TIGHTENING (2) then STATE_STALLED (4) ~2s later
ros2 service call /a200_0804/left_tool/command scaffolding_tool_msgs/srv/ToolCommand "{verb: 5, motor: 1}"   # STOP M1
ros2 service call /a200_0804/left_tool/config  scaffolding_tool_msgs/srv/ToolConfig  "{op: 0, param: 'MAX_CURRENT', motor: 1, values: [900]}"
ros2 service call /a200_0804/left_tool/config  scaffolding_tool_msgs/srv/ToolConfig  "{op: 1, param: 'MAX_CURRENT', motor: 1, values: []}"
```

**Disconnect/reconnect**: Ctrl-C term 2, observe `link_ok=False` on status topic within ~1 s; restart fake firmware → `link_ok=True` returns.

### Unit + integration tests
```bash
# Pure parser, fast
pytest src/scaffolding_tool/scaffolding_tool_driver/test/test_protocol.py -v

# Socat-based serial integration (no rclpy)
pytest src/scaffolding_tool/scaffolding_tool_driver/test/test_serial_link_socat.py -v

# Full workspace tests including rclpy node
colcon test --packages-select scaffolding_tool_msgs scaffolding_tool_driver
colcon test-result --verbose
```

### Monitor smoke test (after driver runs locally)
With driver running in `/a200_0804/left_tool` and fake firmware up:
```bash
ros2 run husky_assembly_teleop husky_monitor      # PyBullet GUI
# click L M1 Tighten → status slider should flip to TIGHTENING → STALLED
# press 's' → status returns to IDLE
```

---

## Implementation order (suggested for `implementer` subagent)

1. Create the new repo skeleton + `scaffolding_tool_msgs` interfaces. `colcon build` → `ros2 interface show` passes.
2. `protocol.py` + `test_protocol.py`. `pytest` passes.
3. `serial_link.py` + `fake_firmware.py` + `test_serial_link_socat.py`. socat round-trip works.
4. `driver_node.py` + `main.py` + launch files + `test_driver_node.py`. End-to-end mock works.
5. Client side: `scaffolding_tool_client.py`, hook into `HuskyRobotInterface`, deprecate `toggle_gripper`/`toggle_screw`, replace button block + keyboard. Smoke test in PyBullet against the local mock.
6. README in new repo with deployment notes (rosdep `python3-serial`, udev rule template, two-bus assumption, panic-stop key).

Per memory `feedback_subagent_workflow.md`: drive each step via planner→implementer→reviewer with the venv-based test commands above.
