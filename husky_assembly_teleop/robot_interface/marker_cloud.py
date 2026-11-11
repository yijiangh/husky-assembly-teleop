"""
The relay's marker cloud on the wire: every marker the cameras saw in one frame, as a PointCloud2.

`cloud_message` builds it on the relay, `read_cloud` turns it back into a `MarkerCloud` on the monitor; the fields
are x, y, z, size, residual (float32, metres) and id (uint32, NatNet's).
"""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

from .mocap_state import MarkerCloud

#: The relay's marker cloud: every frame's visible markers, in the lab frame (header.frame_id says if raw).
CLOUD_TOPIC = "/mocap_relay/markers"
#: One point's bytes, little endian.
POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("size", "<f4"), ("residual", "<f4"), ("id", "<u4")])
_FIELDS = [PointField(name=name, offset=POINT.fields[name][1],
                      datatype=PointField.UINT32 if name == "id" else PointField.FLOAT32, count=1)
           for name in POINT.names]


def cloud_message(positions: np.ndarray, sizes: np.ndarray, residuals: np.ndarray, ids: np.ndarray,
                  capture_time_ns: int, frame_id: str) -> PointCloud2:
    """One frame's markers as a PointCloud2 stamped with when the cameras captured them.

    Args:
        positions: Shape (n, 3), metres.
        sizes: Diameters, metres, shape (n,).
        residuals: Motive's residuals, metres, shape (n,).
        ids: NatNet's marker ids, shape (n,).
        capture_time_ns: Capture time, ns.
        frame_id: The frame the positions are in.
    """
    points = np.zeros(len(ids), dtype=POINT)
    positions = np.asarray(positions, dtype=float).reshape(-1, 3)
    points["x"], points["y"], points["z"] = positions.T
    points["size"], points["residual"], points["id"] = sizes, residuals, ids
    message = PointCloud2(height=1, width=len(points), fields=_FIELDS, is_bigendian=False, point_step=POINT.itemsize,
                          row_step=POINT.itemsize * len(points), is_dense=True)
    message.header.stamp = Time(sec=capture_time_ns // 1_000_000_000, nanosec=capture_time_ns % 1_000_000_000)
    message.header.frame_id = frame_id
    message.data = points.tobytes()
    return message


def read_cloud(message: PointCloud2, now: float) -> MarkerCloud:
    """The cloud a relay message carries, arrived at ROS time `now`.

    Raises:
        ValueError: If the message is not laid out as `cloud_message` makes it.
    """
    if message.point_step != POINT.itemsize or [field.name for field in message.fields] != list(POINT.names):
        raise ValueError(f"not the relay's marker cloud (fields {[field.name for field in message.fields]})")
    points = np.frombuffer(bytes(message.data), dtype=POINT, count=message.width * message.height)
    stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
    return MarkerCloud(positions=np.column_stack([points["x"], points["y"], points["z"]]).astype(float),
                       sizes=points["size"].astype(float), residuals=points["residual"].astype(float),
                       ids=points["id"].astype(np.int64), capture_time=stamp if stamp > 0.0 else now,
                       arrival_time=now)
