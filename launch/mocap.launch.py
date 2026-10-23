"""The mocap relay: OptiTrack to ROS, one per ROS domain, on a control PC wired to the mocap network.

Moved here from crl-husky (`crl_husky mocap.launch.py`), which keeps the messages and the robots' config.

    ros2 launch husky_assembly_teleop mocap.launch.py                           # Motive PC at 192.168.0.28
    ros2 launch husky_assembly_teleop mocap.launch.py server_ip:=192.168.0.117  # another Motive PC

The same in a simulation's own domain: there the relay also publishes simulated robots and a fake Motive's bodies
(node.py), in place of Motive's with the same id, so it keeps running while simulations come and go.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detect_client_ip_from_server(server_ip: str):
    """Best-effort IPv4 source address detection for the route to server_ip."""
    import subprocess

    try:
        cmd = ["ip", "-4", "route", "get", server_ip]
        output = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT).strip()
        # Typical output contains: "src <ip>"
        tokens = output.split()
        if "src" in tokens:
            src_idx = tokens.index("src")
            if src_idx + 1 < len(tokens):
                return tokens[src_idx + 1]
    except Exception:
        return None

    return None


def _build_node(context):
    server_ip = LaunchConfiguration("server_ip").perform(context)
    client_ip = LaunchConfiguration("client_ip").perform(context).strip()
    publish_rate_hz = float(LaunchConfiguration("publish_rate_hz").perform(context))
    topic_prefix = LaunchConfiguration("topic_prefix").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    stale_timeout = float(LaunchConfiguration("streaming_stale_timeout_sec").perform(context))
    marker_error_valid_threshold = float(LaunchConfiguration("marker_error_valid_threshold").perform(context))
    use_multicast = LaunchConfiguration("use_multicast").perform(context).lower() in ("1", "true", "yes")
    multicast_address = LaunchConfiguration("multicast_address").perform(context)
    frame_timeout_sec = float(LaunchConfiguration("frame_timeout_sec").perform(context))
    reconnect_min_interval_sec = float(LaunchConfiguration("reconnect_min_interval_sec").perform(context))
    auto_detect = LaunchConfiguration("auto_detect_client_ip").perform(context).lower() in ("1", "true", "yes")
    drive_root = LaunchConfiguration("drive_root").perform(context)
    use_drive = LaunchConfiguration("use_drive").perform(context).lower() in ("1", "true", "yes")
    watch_period_sec = float(LaunchConfiguration("watch_period_sec").perform(context))

    resolved_client_ip = client_ip
    if auto_detect and not resolved_client_ip:
        detected_ip = _detect_client_ip_from_server(server_ip)
        if detected_ip:
            resolved_client_ip = detected_ip
        else:
            raise RuntimeError(
                "auto_detect_client_ip=true but no client IP could be resolved. "
                "Set client_ip explicitly."
            )

    if not resolved_client_ip:
        raise RuntimeError("client_ip is empty. Set client_ip or enable auto_detect_client_ip.")

    relay_node = Node(
        package="husky_assembly_teleop",
        executable="husky_mocap_relay",
        output="screen",
        # Launch pipes the output, so ROS would not colour warnings and errors; force it unless set already.
        additional_env={"RCUTILS_COLORIZED_OUTPUT": os.environ.get("RCUTILS_COLORIZED_OUTPUT", "1")},
        parameters=[
            {
                "client_ip": resolved_client_ip,
                "server_ip": server_ip,
                "publish_rate_hz": publish_rate_hz,
                "topic_prefix": topic_prefix,
                "frame_id": frame_id,
                "streaming_stale_timeout_sec": stale_timeout,
                "marker_error_valid_threshold": marker_error_valid_threshold,
                "use_multicast": use_multicast,
                "multicast_address": multicast_address,
                "frame_timeout_sec": frame_timeout_sec,
                "reconnect_min_interval_sec": reconnect_min_interval_sec,
                "drive_root": drive_root,
                "use_drive": use_drive,
                "watch_period_sec": watch_period_sec,
            }
        ],
    )

    return [relay_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "server_ip",
                default_value="192.168.0.28",
                description="Mocap/NatNet server IP (Motive host)",
            ),
            DeclareLaunchArgument(
                "client_ip",
                default_value="",
                description="Control-PC IPv4 to bind NatNet sockets (empty + auto_detect=true to detect)",
            ),
            DeclareLaunchArgument(
                "auto_detect_client_ip",
                default_value="true",
                description="Auto-detect client IP from route to server_ip when client_ip is empty",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="50.0",
                description="ROS publish rate for relayed mocap poses",
            ),
            DeclareLaunchArgument(
                "topic_prefix",
                default_value="/mocap/rigid_body",
                description="Topic prefix for per-ID pose topics",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="mocap_world",
                description="frame_id used in PoseStamped messages",
            ),
            DeclareLaunchArgument(
                "streaming_stale_timeout_sec",
                default_value="1.0",
                description="ID is considered streaming if seen within this many seconds",
            ),
            DeclareLaunchArgument(
                "marker_error_valid_threshold",
                default_value="1.0",
                description="Pose validity requires marker_error <= this threshold",
            ),
            DeclareLaunchArgument(
                "use_multicast",
                default_value="false",
                description="NatNet transport mode (false=unicast, true=multicast)",
            ),
            DeclareLaunchArgument(
                "multicast_address",
                default_value="239.255.42.99",
                description="NatNet multicast group address when use_multicast=true",
            ),
            DeclareLaunchArgument(
                "frame_timeout_sec",
                default_value="2.0",
                description="Reconnect if no NatNet frames are received for this duration",
            ),
            DeclareLaunchArgument(
                "reconnect_min_interval_sec",
                default_value="2.0",
                description="Minimum time between reconnect attempts",
            ),
            DeclareLaunchArgument(
                "drive_root",
                default_value="",
                description="Local copy of the project's Drive folder; empty uses HUSKY_DRIVE_ROOT",
            ),
            DeclareLaunchArgument(
                "use_drive",
                default_value="true",
                description="Prefer the newest calibration session on the Drive over crl-husky's default",
            ),
            DeclareLaunchArgument(
                "watch_period_sec",
                default_value="30.0",
                description="Seconds between looks for newer calibrations on the Drive (logged, never applied); 0: off",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
