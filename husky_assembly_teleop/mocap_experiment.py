import argparse
import csv
import json
import math
import os
import re
import shutil
from collections import defaultdict
from datetime import datetime

import numpy as np

from husky_assembly_teleop import DATA_DIRECTORY

EXPERIMENT_DATA_DIR = os.path.join(DATA_DIRECTORY, "mocap_experiments")
CONFIG_TEMPLATE_PATH = os.path.join(EXPERIMENT_DATA_DIR, "_template", "config.yaml")
DEFAULT_CONFIG_PATH = os.path.join(EXPERIMENT_DATA_DIR, "config.yaml")

SCHEMA_VERSION = 1

METRIC_SPECS = (
    ("distance_mm", "Distance Drift", "Distance from first frame (mm)"),
    ("angle_x_deg", "X-Axis Angular Drift", "Angle to first frame X axis (deg)"),
    ("angle_y_deg", "Y-Axis Angular Drift", "Angle to first frame Y axis (deg)"),
    ("angle_z_deg", "Z-Axis Angular Drift", "Angle to first frame Z axis (deg)"),
    ("marker_error", "Marker Error", "OptiTrack marker error"),
    ("tracking_invalid_flag", "Invalid Tracking Flag", "1 = invalid tracking, 0 = valid tracking"),
)


class InvalidTakeDataError(RuntimeError):
    """Raised when a take contains clearly invalid frozen rigid-body tracking data."""


def _default_config():
    return {
        "experiment": {
            "name": "mocap_cover_study",
            "session_name": "default_session",
            "operator": "",
            "duration_sec": 20.0,
            "notes": "",
        },
        "take": {
            "take_id": "",
            "workspace_position": "center",
            "marker_configuration": "markers_4",
            "camera_configuration": "all_cameras",
            "cover_configuration": "no_cover",
            "trial_index": 1,
            "notes": "",
            "reference_images": {
                "overview": "",
                "workspace": "",
                "markers": "",
                "camera": "",
                "cover": "",
                "extra": [],
            },
        },
        "tags": {
            "marker_count": 4,
            "added_marker_count": 0,
            "disabled_camera_count": 0,
            "deck_motion": "none",
            "robot_stationary": True,
        },
        "capture": {
            "workspace_webcam": {
                "enabled": True,
                "device_index": 0,
                "warmup_frames": 8,
                "role": "workspace",
                "timelapse_enabled": True,
                "timelapse_interval_sec": 0.5,
                "timelapse_video_fps": 2.0,
            }
        },
        "report": {
            "group_by": [
                "take.workspace_position",
                "take.marker_configuration",
                "take.camera_configuration",
                "take.cover_configuration",
            ]
        },
    }


def ensure_default_config(config_path=DEFAULT_CONFIG_PATH):
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    if os.path.exists(config_path):
        return config_path

    if os.path.exists(CONFIG_TEMPLATE_PATH):
        shutil.copyfile(CONFIG_TEMPLATE_PATH, config_path)
        return config_path

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            f"Cannot create default MoCap experiment config at {config_path}: missing PyYAML"
        ) from exc

    with open(config_path, "w") as f:
        yaml.safe_dump(_default_config(), f, sort_keys=False)
    return config_path


def load_experiment_config(config_path=DEFAULT_CONFIG_PATH):
    config_path = ensure_default_config(config_path)

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            f"Cannot load MoCap experiment config from {config_path}: missing PyYAML"
        ) from exc

    with open(config_path, "r") as f:
        config = yaml.safe_load(f) or {}

    defaults = _default_config()
    for section_name, section_defaults in defaults.items():
        section = config.get(section_name)
        if not isinstance(section, dict):
            section = {}
        merged_section = dict(section_defaults)
        merged_section.update(section)
        config[section_name] = merged_section

    config["experiment"]["duration_sec"] = float(config["experiment"].get("duration_sec", 20.0))
    config["take"]["trial_index"] = int(config["take"].get("trial_index", 1))
    config["take"]["take_id"] = str(config["take"].get("take_id", "") or "")
    default_reference_images = defaults["take"]["reference_images"]
    reference_images = config["take"].get("reference_images")
    if not isinstance(reference_images, dict):
        reference_images = {}
    merged_reference_images = dict(default_reference_images)
    merged_reference_images.update(reference_images)
    extra_images = merged_reference_images.get("extra")
    if extra_images is None:
        merged_reference_images["extra"] = []
    elif isinstance(extra_images, (list, tuple)):
        merged_reference_images["extra"] = [str(path) for path in extra_images if str(path).strip()]
    elif str(extra_images).strip():
        merged_reference_images["extra"] = [str(extra_images)]
    else:
        merged_reference_images["extra"] = []
    config["take"]["reference_images"] = merged_reference_images
    capture_defaults = defaults["capture"]["workspace_webcam"]
    capture_config = config.get("capture")
    if not isinstance(capture_config, dict):
        capture_config = {}
    workspace_webcam = capture_config.get("workspace_webcam")
    if not isinstance(workspace_webcam, dict):
        workspace_webcam = {}
    merged_workspace_webcam = dict(capture_defaults)
    merged_workspace_webcam.update(workspace_webcam)
    merged_workspace_webcam["enabled"] = bool(merged_workspace_webcam.get("enabled", True))
    merged_workspace_webcam["device_index"] = int(merged_workspace_webcam.get("device_index", 0))
    merged_workspace_webcam["warmup_frames"] = max(0, int(merged_workspace_webcam.get("warmup_frames", 8)))
    merged_workspace_webcam["role"] = str(merged_workspace_webcam.get("role", "workspace") or "workspace")
    merged_workspace_webcam["timelapse_enabled"] = bool(merged_workspace_webcam.get("timelapse_enabled", True))
    merged_workspace_webcam["timelapse_interval_sec"] = max(0.05, float(merged_workspace_webcam.get("timelapse_interval_sec", 0.5)))
    merged_workspace_webcam["timelapse_video_fps"] = max(0.1, float(merged_workspace_webcam.get("timelapse_video_fps", 2.0)))
    config["capture"] = {"workspace_webcam": merged_workspace_webcam}
    group_by = config["report"].get("group_by") or defaults["report"]["group_by"]
    config["report"]["group_by"] = [str(key) for key in group_by]
    return config_path, config


def sanitize_slug(value):
    text = str(value).strip().lower()
    allowed = []
    for char in text:
        if char.isalnum():
            allowed.append(char)
        elif char in ("-", "_"):
            allowed.append(char)
        elif char in (" ", ".", "/"):
            allowed.append("-")
    slug = "".join(allowed).strip("-_")
    return slug or "unnamed"


def format_take_label(config):
    take = config.get("take", {})
    label_parts = [
        take.get("take_id", ""),
        take.get("workspace_position", ""),
        take.get("marker_configuration", ""),
        take.get("camera_configuration", ""),
        take.get("cover_configuration", ""),
        f"take{take.get('trial_index', 1)}",
    ]
    return " | ".join(str(part) for part in label_parts if str(part).strip())


def prepare_take_output(config, recorded_at=None):
    recorded_at = recorded_at or datetime.now()
    date_folder = recorded_at.strftime("%Y%m%d")
    session_name = sanitize_slug(
        config.get("experiment", {}).get("session_name")
        or config.get("experiment", {}).get("name")
        or "session"
    )
    session_dir = os.path.join(EXPERIMENT_DATA_DIR, date_folder, session_name)
    takes_dir = os.path.join(session_dir, "takes")
    analysis_dir = os.path.join(session_dir, "analysis")
    os.makedirs(takes_dir, exist_ok=True)
    os.makedirs(analysis_dir, exist_ok=True)

    take_label = format_take_label(config)
    filename = f"{recorded_at.strftime('%Y%m%d_%H%M%S')}_{sanitize_slug(take_label)}.json"
    take_stem = os.path.splitext(filename)[0]
    return {
        "recorded_at": recorded_at,
        "date_folder": date_folder,
        "session_dir": session_dir,
        "takes_dir": takes_dir,
        "analysis_dir": analysis_dir,
        "take_path": os.path.join(takes_dir, filename),
        "manifest_path": os.path.join(session_dir, "manifest.json"),
        "take_label": take_label,
        "take_stem": take_stem,
        "reference_media_dir": os.path.join(session_dir, "reference_media", take_stem),
        "photo_library_dir": os.path.join(session_dir, "photo_library"),
    }


def _resolve_asset_path(config_path, asset_path):
    if not asset_path:
        return None
    if os.path.isabs(asset_path):
        return os.path.abspath(asset_path)
    return os.path.abspath(os.path.join(os.path.dirname(config_path), asset_path))


def _copy_reference_images(config, config_path, output_paths):
    reference_images = config.get("take", {}).get("reference_images", {})
    copied_assets = []
    os.makedirs(output_paths["reference_media_dir"], exist_ok=True)

    for role, raw_value in reference_images.items():
        asset_values = raw_value if role == "extra" and isinstance(raw_value, list) else [raw_value]
        for asset_index, asset_value in enumerate(asset_values):
            if not str(asset_value).strip():
                continue

            resolved_path = _resolve_asset_path(config_path, str(asset_value))
            if not resolved_path or not os.path.exists(resolved_path):
                copied_assets.append(
                    {
                        "role": role,
                        "path_in_config": str(asset_value),
                        "status": "missing",
                    }
                )
                continue

            basename = os.path.basename(resolved_path)
            target_name = basename if role == "extra" else f"{role}_{asset_index + 1}{os.path.splitext(basename)[1]}"
            target_path = os.path.join(output_paths["reference_media_dir"], target_name)
            shutil.copy2(resolved_path, target_path)
            copied_assets.append(
                {
                    "role": role,
                    "path_in_config": str(asset_value),
                    "status": "copied",
                    "source_path": resolved_path,
                    "session_relative_path": os.path.relpath(target_path, output_paths["session_dir"]),
                    "filename": os.path.basename(target_path),
                }
            )

    return copied_assets


def get_take_association_key(config, output_paths):
    take_id = str(config.get("take", {}).get("take_id", "") or "").strip()
    return sanitize_slug(take_id) if take_id else output_paths["take_stem"]


def validate_target_rigid_body_motion(frames, target_rigid_body, take_label=None):
    samples = []
    for frame in frames:
        rigid_body = frame.get("rigid_bodies", {}).get(target_rigid_body)
        if rigid_body is None:
            continue
        position = tuple(float(value) for value in rigid_body.get("position_m", []))
        quaternion = tuple(float(value) for value in rigid_body.get("quaternion_xyzw", []))
        if len(position) != 3 or len(quaternion) != 4:
            continue
        samples.append((position, quaternion))

    if len(samples) <= 1:
        return

    unique_positions = {position for position, _ in samples}
    unique_quaternions = {quaternion for _, quaternion in samples}
    if len(unique_positions) == 1 and len(unique_quaternions) == 1:
        take_context = f" for take '{take_label}'" if take_label else ""
        raise InvalidTakeDataError(
            "Detected frozen MoCap data"
            f"{take_context}: target rigid body '{target_rigid_body}'"
            f" has one unique position and one unique quaternion across {len(samples)} frames."
        )


def capture_workspace_webcam_image(config, output_paths):
    webcam_config = config.get("capture", {}).get("workspace_webcam", {})
    if not webcam_config.get("enabled", True):
        return None

    try:
        import cv2
    except ImportError:
        return {
            "role": webcam_config.get("role", "workspace"),
            "status": "capture_failed",
            "reason": "opencv_not_available",
        }

    device_index = int(webcam_config.get("device_index", 0))
    warmup_frames = int(webcam_config.get("warmup_frames", 8))
    role = str(webcam_config.get("role", "workspace") or "workspace")

    os.makedirs(output_paths["photo_library_dir"], exist_ok=True)
    association_key = get_take_association_key(config, output_paths)
    filename = f"{association_key}__{sanitize_slug(role)}.jpg"
    target_path = os.path.join(output_paths["photo_library_dir"], filename)

    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        return {
            "role": role,
            "status": "capture_failed",
            "reason": f"cannot_open_device_{device_index}",
        }

    frame = None
    try:
        for _ in range(max(1, warmup_frames)):
            ok, current = cap.read()
            if ok:
                frame = current
        if frame is None:
            ok, current = cap.read()
            if ok:
                frame = current
        if frame is None:
            return {
                "role": role,
                "status": "capture_failed",
                "reason": "no_frame_returned",
            }

        ok = cv2.imwrite(target_path, frame)
        if not ok:
            return {
                "role": role,
                "status": "capture_failed",
                "reason": "failed_to_write_image",
            }
    finally:
        cap.release()

    return {
        "role": role,
        "status": "captured",
        "session_relative_path": os.path.relpath(target_path, output_paths["session_dir"]),
        "filename": os.path.basename(target_path),
        "association_key": association_key,
        "device_index": device_index,
    }


def start_workspace_webcam_timelapse(config, output_paths):
    webcam_config = config.get("capture", {}).get("workspace_webcam", {})
    if not webcam_config.get("enabled", True):
        return None
    if not webcam_config.get("timelapse_enabled", True):
        return None

    try:
        import cv2
    except ImportError:
        return {
            "role": webcam_config.get("role", "workspace"),
            "status": "capture_failed",
            "reason": "opencv_not_available",
        }

    device_index = int(webcam_config.get("device_index", 0))
    warmup_frames = int(webcam_config.get("warmup_frames", 8))
    role = str(webcam_config.get("role", "workspace") or "workspace")
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        return {
            "role": role,
            "status": "capture_failed",
            "reason": f"cannot_open_device_{device_index}",
        }

    for _ in range(max(1, warmup_frames)):
        cap.read()

    frames_dir = os.path.join(output_paths["reference_media_dir"], f"{sanitize_slug(role)}_frames")
    os.makedirs(frames_dir, exist_ok=True)
    return {
        "status": "recording",
        "role": role,
        "device_index": device_index,
        "frame_interval_sec": float(webcam_config.get("timelapse_interval_sec", 0.5)),
        "video_fps": float(webcam_config.get("timelapse_video_fps", 2.0)),
        "frames_dir": frames_dir,
        "frames_relative_dir": os.path.relpath(frames_dir, output_paths["session_dir"]),
        "next_capture_sec": 0.0,
        "frame_paths": [],
        "frame_size": None,
        "capture": cap,
    }


def step_workspace_webcam_timelapse(recording_state, elapsed_sec, output_paths):
    if not recording_state or recording_state.get("status") != "recording":
        return recording_state
    if elapsed_sec + 1e-9 < recording_state["next_capture_sec"]:
        return recording_state

    cap = recording_state.get("capture")
    if cap is None:
        recording_state["status"] = "capture_failed"
        recording_state["reason"] = "capture_not_initialized"
        return recording_state

    ok, frame = cap.read()
    if not ok or frame is None:
        recording_state["status"] = "capture_failed"
        recording_state["reason"] = "no_frame_returned"
        return recording_state

    frame_index = len(recording_state["frame_paths"])
    frame_filename = f"{sanitize_slug(recording_state['role'])}_{frame_index:04d}.jpg"
    frame_path = os.path.join(recording_state["frames_dir"], frame_filename)

    import cv2

    if not cv2.imwrite(frame_path, frame):
        recording_state["status"] = "capture_failed"
        recording_state["reason"] = "failed_to_write_frame"
        return recording_state

    recording_state["frame_paths"].append(frame_path)
    recording_state["frame_size"] = (int(frame.shape[1]), int(frame.shape[0]))
    recording_state["next_capture_sec"] += recording_state["frame_interval_sec"]
    return recording_state


def finalize_workspace_webcam_timelapse(recording_state, output_paths):
    if not recording_state:
        return None

    cap = recording_state.get("capture")
    if cap is not None:
        cap.release()
        recording_state["capture"] = None

    result = {
        "role": recording_state.get("role", "workspace"),
        "status": recording_state.get("status", "unknown"),
        "device_index": recording_state.get("device_index"),
        "frame_interval_sec": recording_state.get("frame_interval_sec"),
        "video_fps": recording_state.get("video_fps"),
        "frames_relative_dir": recording_state.get("frames_relative_dir"),
        "frame_count": len(recording_state.get("frame_paths", [])),
    }

    if recording_state.get("status") == "capture_failed":
        result["reason"] = recording_state.get("reason", "unknown_error")
        return result

    frame_paths = list(recording_state.get("frame_paths", []))
    if not frame_paths:
        result["status"] = "no_frames_captured"
        return result

    try:
        import cv2
    except ImportError:
        result["status"] = "capture_failed"
        result["reason"] = "opencv_not_available"
        return result

    frame_size = recording_state.get("frame_size")
    if frame_size is None:
        first_frame = cv2.imread(frame_paths[0])
        if first_frame is None:
            result["status"] = "capture_failed"
            result["reason"] = "failed_to_read_captured_frame"
            return result
        frame_size = (int(first_frame.shape[1]), int(first_frame.shape[0]))

    video_filename = f"{sanitize_slug(recording_state['role'])}_timelapse.mp4"
    video_path = os.path.join(output_paths["reference_media_dir"], video_filename)
    writer = cv2.VideoWriter(
        video_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(recording_state["video_fps"]),
        frame_size,
    )

    if not writer.isOpened():
        result["status"] = "capture_failed"
        result["reason"] = "failed_to_open_video_writer"
        return result

    try:
        for frame_path in frame_paths:
            frame = cv2.imread(frame_path)
            if frame is None:
                continue
            if (frame.shape[1], frame.shape[0]) != frame_size:
                frame = cv2.resize(frame, frame_size)
            writer.write(frame)
    finally:
        writer.release()

    result["status"] = "created"
    result["session_relative_path"] = os.path.relpath(video_path, output_paths["session_dir"])
    result["filename"] = os.path.basename(video_path)
    return result


def build_take_payload(
    config,
    config_path,
    output_paths,
    target_rigid_body,
    selected_robot_id,
    frames,
    rigid_body_ids,
    stop_reason,
    auto_reference_images=None,
    mocap_camera_inventory=None,
    webcam_timelapse=None,
):
    recorded_at = output_paths["recorded_at"]
    validate_target_rigid_body_motion(frames, target_rigid_body, output_paths.get("take_label"))
    available_rigid_bodies = sorted({name for frame in frames for name in frame["rigid_bodies"].keys()})
    copied_reference_images = _copy_reference_images(config, config_path, output_paths)
    auto_reference_images = list(auto_reference_images or [])
    return {
        "schema_version": SCHEMA_VERSION,
        "recorded_at": recorded_at.isoformat(),
        "date_folder": output_paths["date_folder"],
        "session_name": os.path.basename(output_paths["session_dir"]),
        "config_path": config_path,
        "config": config,
        "take_label": output_paths["take_label"],
        "target_rigid_body": target_rigid_body,
        "selected_robot_id": int(selected_robot_id),
        "selected_robot_name": target_rigid_body,
        "duration_sec": float(config["experiment"]["duration_sec"]),
        "stop_reason": stop_reason,
        "frame_count": len(frames),
        "take_stem": output_paths["take_stem"],
        "association_key": get_take_association_key(config, output_paths),
        "rigid_body_ids": {name: int(rb_id) for name, rb_id in sorted(rigid_body_ids.items())},
        "available_rigid_bodies": available_rigid_bodies,
        "mocap_camera_inventory": mocap_camera_inventory,
        "webcam_timelapse": webcam_timelapse,
        "reference_images": auto_reference_images + copied_reference_images,
        "frames": frames,
    }


def save_take_payload(payload, take_path, manifest_path):
    os.makedirs(os.path.dirname(take_path), exist_ok=True)
    with open(take_path, "w") as f:
        json.dump(payload, f, indent=2)
    _update_manifest(manifest_path, take_path, payload)
    return take_path


def _update_manifest(manifest_path, take_path, payload):
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "updated_at": datetime.now().isoformat(),
        "takes": [],
    }
    if os.path.exists(manifest_path):
        with open(manifest_path, "r") as f:
            manifest = json.load(f)

    session_dir = os.path.dirname(manifest_path)
    rel_take_path = os.path.relpath(take_path, session_dir)
    summary = {
        "recorded_at": payload["recorded_at"],
        "take_path": rel_take_path,
        "take_label": payload["take_label"],
        "target_rigid_body": payload["target_rigid_body"],
        "duration_sec": payload["duration_sec"],
        "frame_count": payload["frame_count"],
        "mocap_camera_count": (payload.get("mocap_camera_inventory") or {}).get("camera_count"),
        "webcam_timelapse_status": (payload.get("webcam_timelapse") or {}).get("status"),
        "take": payload["config"].get("take", {}),
        "tags": payload["config"].get("tags", {}),
        "reference_images": payload.get("reference_images", []),
    }

    takes = [entry for entry in manifest.get("takes", []) if entry.get("take_path") != rel_take_path]
    takes.append(summary)
    takes.sort(key=lambda entry: entry.get("recorded_at", ""))

    manifest["updated_at"] = datetime.now().isoformat()
    manifest["takes"] = takes

    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)


def _load_take_file(path):
    with open(path, "r") as f:
        payload = json.load(f)
    if not isinstance(payload, dict) or "frames" not in payload or "target_rigid_body" not in payload:
        return None
    payload["_source_path"] = path
    return payload


def _discover_session_photo_library_assets(take_payload):
    source_path = take_payload.get("_source_path")
    if not source_path:
        return []

    session_dir = os.path.dirname(os.path.dirname(source_path))
    photo_library_dir = os.path.join(session_dir, "photo_library")
    if not os.path.isdir(photo_library_dir):
        return []

    take_id = str(_get_nested_value(take_payload.get("config", {}), "take.take_id", default="") or "").strip()
    take_stem = str(take_payload.get("take_stem", "") or os.path.splitext(os.path.basename(source_path))[0])
    candidates = []
    if take_id:
        candidates.append(sanitize_slug(take_id))
    if take_stem:
        candidates.append(sanitize_slug(take_stem))
    candidates = [candidate for candidate in candidates if candidate]
    if not candidates:
        return []

    discovered = []
    for filename in sorted(os.listdir(photo_library_dir)):
        filepath = os.path.join(photo_library_dir, filename)
        if not os.path.isfile(filepath):
            continue
        basename, _ = os.path.splitext(filename)
        if "__" not in basename:
            continue
        prefix, role = basename.split("__", 1)
        if prefix not in candidates:
            continue
        discovered.append(
            {
                "role": role,
                "status": "discovered",
                "session_relative_path": os.path.relpath(filepath, session_dir),
                "filename": filename,
                "association_key": prefix,
            }
        )
    return discovered


def discover_take_files(inputs):
    if not inputs:
        raise ValueError("At least one input path is required.")

    discovered = []
    for input_path in inputs:
        if os.path.isfile(input_path):
            discovered.append(os.path.abspath(input_path))
            continue

        for dirpath, dirnames, filenames in os.walk(input_path):
            if os.path.basename(dirpath) == "analysis":
                continue
            for filename in filenames:
                if not filename.endswith(".json"):
                    continue
                discovered.append(os.path.abspath(os.path.join(dirpath, filename)))

    take_files = []
    for path in sorted(set(discovered)):
        payload = _load_take_file(path)
        if payload is not None:
            take_files.append(path)
    return take_files


def _metric_stats(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def _label_to_numeric(value):
    match = re.search(r"(-?\d+(?:\.\d+)?)", str(value))
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _humanize_condition_value(value):
    text = str(value or "n/a")
    if not text.strip():
        return "n/a"
    return text.replace("_", " ")


def _format_condition_label(value):
    return _humanize_condition_value(value).replace(" ", "\n")


def _sort_condition_values(values, key_name):
    values = [str(value) for value in values]
    if key_name == "workspace_position":
        preferred_order = {
            "center": 0,
            "right": 1,
            "left": 2,
            "front": 3,
            "back": 4,
        }
        return sorted(values, key=lambda value: (preferred_order.get(str(value).lower(), 99), str(value).lower()))
    return sorted(
        values,
        key=lambda value: (
            _label_to_numeric(value) is None,
            _label_to_numeric(value) if _label_to_numeric(value) is not None else str(value).lower(),
        ),
    )


def _collect_condition_catalog(metrics_by_take):
    catalog = defaultdict(set)
    for take_metrics in metrics_by_take:
        take = take_metrics.get("config", {}).get("take", {})
        for key in ("workspace_position", "marker_configuration", "camera_configuration", "cover_configuration"):
            catalog[key].add(str(take.get(key, "n/a")))
    return {key: _sort_condition_values(values, key) for key, values in catalog.items()}


def _select_matrix_axes(metrics_by_take):
    catalog = _collect_condition_catalog(metrics_by_take)
    varying_keys = [key for key, values in catalog.items() if len(values) > 1]
    if not varying_keys:
        return None

    x_key = "workspace_position" if "workspace_position" in varying_keys else varying_keys[0]
    for candidate in ("marker_configuration", "camera_configuration", "cover_configuration"):
        if candidate != x_key and candidate in varying_keys:
            y_key = candidate
            break
    else:
        remaining = [key for key in varying_keys if key != x_key]
        y_key = remaining[0] if remaining else None

    if y_key is None:
        return None

    return {
        "x_key": x_key,
        "y_key": y_key,
        "x_values": catalog.get(x_key, []),
        "y_values": catalog.get(y_key, []),
        "secondary_keys": [key for key in varying_keys if key not in {x_key, y_key}],
        "catalog": catalog,
    }


def _metric_summary_with_angle_max(metric_values):
    summary = {
        metric_name: _metric_stats(metric_values.get(metric_name, []))
        for metric_name, _, _ in METRIC_SPECS
    }
    angle_max_values = [
        max(x_value, y_value, z_value)
        for x_value, y_value, z_value in zip(
            metric_values.get("angle_x_deg", []),
            metric_values.get("angle_y_deg", []),
            metric_values.get("angle_z_deg", []),
        )
    ]
    summary["angle_max_deg"] = _metric_stats(angle_max_values)
    return summary


def _tracking_quality_summary(samples):
    valid_count = sum(1 for sample in samples if sample.get("tracking_valid") is True)
    invalid_count = sum(1 for sample in samples if sample.get("tracking_valid") is False)
    unknown_count = sum(1 for sample in samples if sample.get("tracking_valid") is None)
    known_count = valid_count + invalid_count
    invalid_rate = (float(invalid_count) / float(known_count)) if known_count else None
    return {
        "tracking_valid_frame_count": int(valid_count),
        "tracking_invalid_frame_count": int(invalid_count),
        "tracking_unknown_frame_count": int(unknown_count),
        "tracking_known_frame_count": int(known_count),
        "tracking_invalid_rate": invalid_rate,
        "tracking_invalid_rate_pct": (100.0 * invalid_rate) if invalid_rate is not None else None,
    }


def _get_nested_value(data, dotted_key, default=""):
    current = data
    for part in str(dotted_key).split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def _rotation_matrix_from_quaternion(quaternion_xyzw):
    from scipy.spatial.transform import Rotation as R

    return R.from_quat(np.asarray(quaternion_xyzw, dtype=float)).as_matrix()


def compute_take_metrics(take_payload):
    target_name = take_payload["target_rigid_body"]
    samples = []
    for frame in take_payload.get("frames", []):
        rigid_body = frame.get("rigid_bodies", {}).get(target_name)
        if rigid_body is None:
            continue
        marker_error = rigid_body.get("marker_error")
        tracking_valid = rigid_body.get("tracking_valid")
        samples.append(
            {
                "timestamp": frame.get("timestamp"),
                "elapsed_sec": frame.get("elapsed_sec"),
                "position_m": np.asarray(rigid_body["position_m"], dtype=float),
                "quaternion_xyzw": np.asarray(rigid_body["quaternion_xyzw"], dtype=float),
                "marker_error": None if marker_error is None else float(marker_error),
                "tracking_valid": None if tracking_valid is None else bool(tracking_valid),
            }
        )

    if not samples:
        return None

    baseline = samples[0]
    baseline_pos = baseline["position_m"]
    baseline_rot = _rotation_matrix_from_quaternion(baseline["quaternion_xyzw"])

    distance_mm = []
    angle_x_deg = []
    angle_y_deg = []
    angle_z_deg = []
    marker_error_values = []
    tracking_invalid_flags = []

    per_frame_metrics = []
    for sample in samples:
        position = sample["position_m"]
        rotation = _rotation_matrix_from_quaternion(sample["quaternion_xyzw"])

        distance = 1000.0 * np.linalg.norm(position - baseline_pos)
        distance_mm.append(float(distance))

        axis_angles = []
        for axis_index in range(3):
            dot = float(np.dot(rotation[:, axis_index], baseline_rot[:, axis_index]))
            dot = np.clip(dot, -1.0, 1.0)
            axis_angles.append(float(np.degrees(math.acos(dot))))

        angle_x_deg.append(axis_angles[0])
        angle_y_deg.append(axis_angles[1])
        angle_z_deg.append(axis_angles[2])
        if sample["marker_error"] is not None:
            marker_error_values.append(float(sample["marker_error"]))
        if sample["tracking_valid"] is not None:
            tracking_invalid_flags.append(0.0 if sample["tracking_valid"] else 1.0)

        per_frame_metrics.append(
            {
                "timestamp": sample["timestamp"],
                "elapsed_sec": sample["elapsed_sec"],
                "distance_mm": float(distance),
                "angle_x_deg": axis_angles[0],
                "angle_y_deg": axis_angles[1],
                "angle_z_deg": axis_angles[2],
                "marker_error": sample["marker_error"],
                "tracking_valid": sample["tracking_valid"],
                "tracking_invalid_flag": None if sample["tracking_valid"] is None else (0.0 if sample["tracking_valid"] else 1.0),
            }
        )

    group_by = take_payload.get("config", {}).get("report", {}).get("group_by") or _default_config()["report"]["group_by"]
    group_values = [
        str(_get_nested_value(take_payload.get("config", {}), key, default="n/a"))
        for key in group_by
    ]
    group_label = " | ".join(group_values)

    payload_reference_images = list(take_payload.get("reference_images", []))
    known_image_paths = {
        (entry.get("role"), entry.get("session_relative_path"))
        for entry in payload_reference_images
        if entry.get("session_relative_path")
    }
    for discovered in _discover_session_photo_library_assets(take_payload):
        key = (discovered.get("role"), discovered.get("session_relative_path"))
        if key not in known_image_paths:
            payload_reference_images.append(discovered)

    metric_values = {
        "distance_mm": distance_mm,
        "angle_x_deg": angle_x_deg,
        "angle_y_deg": angle_y_deg,
        "angle_z_deg": angle_z_deg,
        "marker_error": marker_error_values,
        "tracking_invalid_flag": tracking_invalid_flags,
    }
    quality_summary = _tracking_quality_summary(samples)
    return {
        "source_path": take_payload.get("_source_path"),
        "session_dir": os.path.dirname(os.path.dirname(take_payload.get("_source_path"))),
        "take_label": take_payload.get("take_label"),
        "group_label": group_label,
        "group_key": tuple(group_values),
        "target_rigid_body": target_name,
        "frame_count": len(samples),
        "group_by": group_by,
        "metrics": metric_values,
        "per_frame_metrics": per_frame_metrics,
        "summary": _metric_summary_with_angle_max(metric_values),
        "quality_summary": quality_summary,
        "config": take_payload.get("config", {}),
        "mocap_camera_inventory": take_payload.get("mocap_camera_inventory"),
        "webcam_timelapse": take_payload.get("webcam_timelapse"),
        "reference_images": payload_reference_images,
    }


def _write_csv(path, rows, fieldnames):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _flatten_group_label(group_label):
    return " | ".join(part.strip() for part in str(group_label).split("|"))


def _build_condition_matrix(metrics_by_take, axis_spec, summary_metric):
    cell_metrics = defaultdict(lambda: defaultdict(list))
    for take_metrics in metrics_by_take:
        take = take_metrics.get("config", {}).get("take", {})
        x_value = str(take.get(axis_spec["x_key"], "n/a"))
        y_value = str(take.get(axis_spec["y_key"], "n/a"))
        for metric_name, values in take_metrics["metrics"].items():
            cell_metrics[(y_value, x_value)][metric_name].extend(values)

    matrix = np.full((len(axis_spec["y_values"]), len(axis_spec["x_values"])), np.nan, dtype=float)
    for y_index, y_value in enumerate(axis_spec["y_values"]):
        for x_index, x_value in enumerate(axis_spec["x_values"]):
            stats = _metric_summary_with_angle_max(cell_metrics[(y_value, x_value)])
            stat_group = stats.get(summary_metric["metric_name"], {})
            metric_value = stat_group.get(summary_metric["stat_name"])
            if metric_value is not None:
                matrix[y_index, x_index] = float(metric_value)
    return matrix


def _write_heatmap_plot(path, matrix, axis_spec, title, colorbar_label, value_format, note_lines=None):
    import matplotlib.pyplot as plt

    note_lines = [line for line in (note_lines or []) if str(line).strip()]
    fig_width = max(9.0, len(axis_spec["x_values"]) * 2.6)
    fig_height = max(7.0, len(axis_spec["y_values"]) * 1.8 + (0.5 * len(note_lines)))
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    masked_matrix = np.ma.masked_invalid(matrix)
    cmap = plt.get_cmap("RdYlGn_r").copy()
    cmap.set_bad(color="#f0f0f0")
    image = ax.imshow(masked_matrix, aspect="auto", cmap=cmap)
    colorbar = fig.colorbar(image, ax=ax, shrink=0.9, pad=0.03)
    colorbar.set_label(colorbar_label)

    ax.set_xticks(range(len(axis_spec["x_values"])))
    ax.set_xticklabels([_humanize_condition_value(value) for value in axis_spec["x_values"]], fontsize=12)
    ax.set_yticks(range(len(axis_spec["y_values"])))
    ax.set_yticklabels([_humanize_condition_value(value) for value in axis_spec["y_values"]], fontsize=12)
    ax.set_xlabel(_humanize_condition_value(axis_spec["x_key"]).title(), fontsize=13)
    ax.set_ylabel(_humanize_condition_value(axis_spec["y_key"]).title(), fontsize=13)
    ax.set_title(title, fontsize=15, pad=16)
    ax.tick_params(axis="x", rotation=0)
    ax.set_xticks(np.arange(-0.5, len(axis_spec["x_values"]), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(axis_spec["y_values"]), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", bottom=False, left=False)

    finite_values = matrix[np.isfinite(matrix)]
    threshold = float(np.median(finite_values)) if finite_values.size else 0.0
    for row_index in range(matrix.shape[0]):
        for col_index in range(matrix.shape[1]):
            value = matrix[row_index, col_index]
            if not np.isfinite(value):
                annotation = "n/a"
                color = "#555555"
            else:
                annotation = format(value, value_format)
                color = "white" if value >= threshold else "#1f1f1f"
            ax.text(col_index, row_index, annotation, ha="center", va="center", fontsize=13, fontweight="bold", color=color)

    if note_lines:
        note_text = "\n".join(f"Note: {line}" for line in note_lines)
        fig.text(0.02, 0.02, note_text, fontsize=10, color="#444444")

    fig.tight_layout(rect=(0, 0.04 if note_lines else 0, 1, 1))
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _axis_coordinates(values):
    numeric_values = [_label_to_numeric(value) for value in values]
    if all(value is not None for value in numeric_values):
        return np.asarray(numeric_values, dtype=float), True
    return np.arange(len(values), dtype=float), False


def _write_metric_profile_plot(ax, matrix, axis_spec, title, ylabel):
    import matplotlib.pyplot as plt

    color_map = plt.get_cmap("tab10")
    varying_positions, varying_is_numeric = _axis_coordinates(axis_spec["y_values"])

    for workspace_index, workspace_label in enumerate(axis_spec["x_values"]):
        ax.plot(
            varying_positions,
            matrix[:, workspace_index],
            marker="o",
            linewidth=2.6,
            markersize=7.5,
            label=_humanize_condition_value(workspace_label),
            color=color_map(workspace_index % 10),
        )

    if varying_is_numeric:
        ax.set_xticks(varying_positions)
    else:
        ax.set_xticks(np.arange(len(axis_spec["y_values"])))
        ax.set_xticklabels([_humanize_condition_value(value) for value in axis_spec["y_values"]], fontsize=11)

    ax.set_xlabel(_humanize_condition_value(axis_spec["y_key"]).title())
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=10, frameon=True, title=_humanize_condition_value(axis_spec["x_key"]).title())


def _write_profile_plot(path, matrices, axis_spec, note_lines=None):
    import matplotlib.pyplot as plt

    note_lines = [line for line in (note_lines or []) if str(line).strip()]
    count = len(matrices)
    if count <= 2:
        rows, cols = 1, count
        figsize = (8.2 * count, 6.4)
    elif count <= 4:
        rows, cols = 2, 2
        figsize = (16.5, 12.0)
    else:
        cols = 3
        rows = int(math.ceil(float(count) / float(cols)))
        figsize = (7.2 * cols, 5.5 * rows)
    fig, axes = plt.subplots(rows, cols, figsize=figsize, sharex=False)
    axes = np.asarray(axes).reshape(-1)

    for axis, matrix_spec in zip(axes, matrices):
        _write_metric_profile_plot(
            axis,
            matrix_spec["matrix"],
            axis_spec,
            matrix_spec["title"],
            matrix_spec["ylabel"],
        )

    for axis in axes[len(matrices):]:
        axis.axis("off")

    if note_lines:
        fig.text(0.02, 0.02, "\n".join(f"Note: {line}" for line in note_lines), fontsize=10, color="#444444")

    fig.tight_layout(rect=(0, 0.04 if note_lines else 0, 1, 1))
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _write_boxplot_or_placeholder(ax, values, display_labels, title, ylabel):
    finite_exists = any(np.isfinite(np.asarray(value, dtype=float)).any() for value in values)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    if not finite_exists:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes, color="#666666")
        ax.set_xticks(range(1, len(display_labels) + 1))
        ax.set_xticklabels(display_labels, rotation=18, fontsize=9)
        return

    plot_values = [value if len(value) > 0 else [np.nan] for value in values]
    ax.boxplot(plot_values, labels=display_labels, patch_artist=True)
    ax.tick_params(axis="x", rotation=18, labelsize=9)


def _write_take_quality_plot(path, take_metrics):
    import matplotlib.pyplot as plt

    per_frame = take_metrics.get("per_frame_metrics", [])
    if not per_frame:
        return False

    times = np.asarray([float(frame.get("elapsed_sec") or 0.0) for frame in per_frame], dtype=float)
    marker_error = np.asarray(
        [np.nan if frame.get("marker_error") is None else float(frame.get("marker_error")) for frame in per_frame],
        dtype=float,
    )
    tracking_valid = np.asarray(
        [
            np.nan
            if frame.get("tracking_valid") is None
            else (1.0 if bool(frame.get("tracking_valid")) else 0.0)
            for frame in per_frame
        ],
        dtype=float,
    )

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 6.5))

    axes[0].plot(times, marker_error, color="#1f77b4", linewidth=1.6)
    axes[0].set_ylabel("Marker error")
    axes[0].set_title(f"Tracking Quality: {take_metrics['take_label']}")
    axes[0].grid(alpha=0.3)
    if not np.isfinite(marker_error).any():
        axes[0].text(0.5, 0.5, "No marker_error data", ha="center", va="center", transform=axes[0].transAxes, color="#666666")

    axes[1].step(times, tracking_valid, where="post", color="#d62728", linewidth=1.8)
    axes[1].set_ylabel("Tracking valid")
    axes[1].set_xlabel("Elapsed time [s]")
    axes[1].set_yticks([0.0, 1.0])
    axes[1].set_yticklabels(["invalid", "valid"])
    axes[1].set_ylim(-0.15, 1.15)
    axes[1].grid(alpha=0.3)
    if not np.isfinite(tracking_valid).any():
        axes[1].text(0.5, 0.5, "No tracking_valid data", ha="center", va="center", transform=axes[1].transAxes, color="#666666")

    fig.tight_layout()
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return True


def _write_condition_summary_table(path, group_rows):
    import matplotlib.pyplot as plt

    if not group_rows:
        return

    metric_specs = (
        ("distance_mm_median", "Dist med\n(mm)"),
        ("distance_mm_p95", "Dist p95\n(mm)"),
        ("angle_max_deg_median", "Angle med\n(deg)"),
        ("angle_max_deg_p95", "Angle p95\n(deg)"),
        ("marker_error_p95", "Marker err\np95"),
        ("tracking_invalid_rate_pct", "Invalid\ntracking %"),
    )
    row_labels = [_flatten_group_label(row["group_label"]) for row in group_rows]
    values = np.asarray(
        [[float(row.get(metric_name) or 0.0) for metric_name, _ in metric_specs] for row in group_rows],
        dtype=float,
    )

    fig_width = 10
    fig_height = max(6.5, len(group_rows) * 0.5)
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    cmap = plt.get_cmap("RdYlGn_r").copy()
    normalized = np.zeros_like(values)
    for column_index in range(values.shape[1]):
        column = values[:, column_index]
        col_min = float(np.min(column))
        col_max = float(np.max(column))
        if col_max - col_min < 1e-12:
            normalized[:, column_index] = 0.5
        else:
            normalized[:, column_index] = (column - col_min) / (col_max - col_min)

    image = ax.imshow(normalized, aspect="auto", cmap=cmap, vmin=0.0, vmax=1.0)
    ax.set_xticks(range(len(metric_specs)))
    ax.set_xticklabels([label for _, label in metric_specs], fontsize=11)
    ax.set_yticks(range(len(row_labels)))
    ax.set_yticklabels(row_labels, fontsize=10)
    ax.set_title("Condition Summary Table", fontsize=15, pad=16)
    ax.set_xlabel("Metric")
    ax.set_ylabel("Condition")
    ax.set_xticks(np.arange(-0.5, len(metric_specs), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(row_labels), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", bottom=False, left=False)

    for row_index in range(values.shape[0]):
        for column_index in range(values.shape[1]):
            value = values[row_index, column_index]
            text_color = "white" if normalized[row_index, column_index] > 0.58 else "#1f1f1f"
            ax.text(
                column_index,
                row_index,
                f"{value:.3f}",
                ha="center",
                va="center",
                fontsize=10,
                fontweight="bold",
                color=text_color,
            )

    colorbar = fig.colorbar(image, ax=ax, shrink=0.85, pad=0.03)
    colorbar.set_label("Relative score in each metric column\n(green = better, red = worse)")
    fig.tight_layout()
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _focus_label_from_catalog(catalog):
    camera_values = catalog.get("camera_configuration", [])
    marker_values = catalog.get("marker_configuration", [])
    workspace_values = catalog.get("workspace_position", [])
    varying = []
    if len(camera_values) > 1:
        varying.append("camera count")
    if len(marker_values) > 1:
        varying.append("marker count")
    if len(workspace_values) > 1:
        varying.append("workspace position")
    if not varying:
        return "single-condition session"
    return " / ".join(varying) + " sweep"


def _build_report_headline(metrics_by_take, group_rows):
    catalog = _collect_condition_catalog(metrics_by_take)
    session_names = sorted({os.path.basename(take_metrics["session_dir"]) for take_metrics in metrics_by_take})
    experiment_names = sorted(
        {
            str(take_metrics.get("config", {}).get("experiment", {}).get("name", "")).strip()
            for take_metrics in metrics_by_take
            if str(take_metrics.get("config", {}).get("experiment", {}).get("name", "")).strip()
        }
    )
    best_distance = min(group_rows, key=lambda row: row.get("distance_mm_p95") or float("inf")) if group_rows else None
    worst_distance = max(group_rows, key=lambda row: row.get("distance_mm_p95") or float("-inf")) if group_rows else None
    best_angle = min(group_rows, key=lambda row: row.get("angle_max_deg_p95") or float("inf")) if group_rows else None
    worst_angle = max(group_rows, key=lambda row: row.get("angle_max_deg_p95") or float("-inf")) if group_rows else None
    best_marker_error = min(group_rows, key=lambda row: row.get("marker_error_p95") or float("inf")) if group_rows else None
    worst_invalid_tracking = max(group_rows, key=lambda row: row.get("tracking_invalid_rate") or float("-inf")) if group_rows else None
    return {
        "session_names": session_names,
        "experiment_names": experiment_names,
        "catalog": catalog,
        "focus_label": _focus_label_from_catalog(catalog),
        "best_distance_group": best_distance["group_label"] if best_distance else None,
        "best_distance_value": best_distance.get("distance_mm_p95") if best_distance else None,
        "worst_distance_group": worst_distance["group_label"] if worst_distance else None,
        "worst_distance_value": worst_distance.get("distance_mm_p95") if worst_distance else None,
        "best_angle_group": best_angle["group_label"] if best_angle else None,
        "best_angle_value": best_angle.get("angle_max_deg_p95") if best_angle else None,
        "worst_angle_group": worst_angle["group_label"] if worst_angle else None,
        "worst_angle_value": worst_angle.get("angle_max_deg_p95") if worst_angle else None,
        "best_marker_error_group": best_marker_error["group_label"] if best_marker_error else None,
        "best_marker_error_value": best_marker_error.get("marker_error_p95") if best_marker_error else None,
        "worst_invalid_tracking_group": worst_invalid_tracking["group_label"] if worst_invalid_tracking else None,
        "worst_invalid_tracking_value": worst_invalid_tracking.get("tracking_invalid_rate_pct") if worst_invalid_tracking else None,
    }


def _write_camera_marker_3d_plot(path, group_rows, metric_name, title, z_label):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    workspace_order = ["center", "right", "left", "back"]
    workspace_groups = {}
    for row in group_rows:
        workspace = str(row.get("workspace_position", "") or "")
        camera_count = row.get("camera_count")
        marker_count = row.get("marker_count")
        metric_value = row.get(metric_name)
        if workspace and camera_count is not None and marker_count is not None and metric_value is not None:
            workspace_groups.setdefault(workspace, []).append(
                (float(camera_count), float(marker_count), float(metric_value))
            )

    summary_points = {}
    for row in group_rows:
        camera_count = row.get("camera_count")
        marker_count = row.get("marker_count")
        metric_value = row.get(metric_name)
        if camera_count is None or marker_count is None or metric_value is None:
            continue
        summary_points.setdefault((float(camera_count), float(marker_count)), []).append(float(metric_value))
    summary_points = [
        (camera_count, marker_count, float(np.mean(values)))
        for (camera_count, marker_count), values in sorted(summary_points.items())
    ]

    fig = plt.figure(figsize=(18, 11))
    plot_specs = [(workspace, workspace_groups.get(workspace, [])) for workspace in workspace_order] + [
        ("summary", summary_points)
    ]

    for subplot_index, (workspace_name, points) in enumerate(plot_specs, start=1):
        ax = fig.add_subplot(2, 3, subplot_index, projection="3d")
        ax.set_title("All positions mean" if workspace_name == "summary" else workspace_name.capitalize())
        ax.set_xlabel("Camera count")
        ax.set_ylabel("Marker count")
        ax.set_zlabel(z_label)
        if not points:
            ax.text2D(0.28, 0.5, "No data", transform=ax.transAxes)
            continue

        xs = np.asarray([point[0] for point in points], dtype=float)
        ys = np.asarray([point[1] for point in points], dtype=float)
        zs = np.asarray([point[2] for point in points], dtype=float)
        scatter = ax.scatter(xs, ys, zs, c=zs, cmap="viridis", s=80, edgecolors="black", linewidths=0.6)

        unique_points = {(x, y) for x, y in zip(xs, ys)}
        if len(unique_points) >= 3:
            try:
                ax.plot_trisurf(xs, ys, zs, cmap="viridis", alpha=0.45, linewidth=0.2)
            except Exception:
                pass

        order = np.lexsort((ys, xs))
        ax.plot(xs[order], ys[order], zs[order], color="#1f4e79", alpha=0.55)
        ax.set_xticks(sorted(set(xs)))
        ax.set_yticks(sorted(set(ys)))
        ax.view_init(elev=26, azim=-130)
        fig.colorbar(scatter, ax=ax, shrink=0.68, pad=0.05)

    fig.suptitle(title, fontsize=16, y=0.98)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _generate_boxplots(grouped_metrics, output_dir):
    import matplotlib.pyplot as plt

    plot_paths = {}
    labels = list(grouped_metrics.keys())
    display_labels = [label.replace(" | ", "\n") for label in labels]

    metric_count = len(METRIC_SPECS)
    combined_cols = 2 if metric_count <= 4 else 3
    combined_rows = int(math.ceil(float(metric_count) / float(combined_cols)))
    combined_fig, combined_axes = plt.subplots(
        combined_rows,
        combined_cols,
        figsize=(max(12, len(labels) * 1.2, combined_cols * 5.5), max(8, combined_rows * 4.2)),
    )
    combined_axes = combined_axes.flatten()

    for plot_index, (metric_name, title, ylabel) in enumerate(METRIC_SPECS):
        values = [grouped_metrics[label][metric_name] for label in labels]

        fig, ax = plt.subplots(figsize=(max(10, len(labels) * 1.2), 5.5))
        _write_boxplot_or_placeholder(ax, values, display_labels, title, ylabel)
        fig.tight_layout()

        plot_filename = f"{metric_name}_boxplot.png"
        plot_path = os.path.join(output_dir, plot_filename)
        fig.savefig(plot_path, dpi=200)
        plt.close(fig)
        plot_paths[metric_name] = plot_path

        combined_ax = combined_axes[plot_index]
        _write_boxplot_or_placeholder(combined_ax, values, display_labels, title, ylabel)

    for axis in combined_axes[metric_count:]:
        axis.axis("off")

    combined_fig.tight_layout()
    combined_path = os.path.join(output_dir, "combined_boxplots.png")
    combined_fig.savefig(combined_path, dpi=200)
    plt.close(combined_fig)
    plot_paths["combined"] = combined_path
    return plot_paths


def _generate_plots(grouped_metrics, group_rows, metrics_by_take, output_dir):
    plot_paths = _generate_boxplots(grouped_metrics, output_dir)

    axis_spec = _select_matrix_axes(metrics_by_take)
    matrix_plot_paths = {}
    analysis_notes = []
    if axis_spec:
        if axis_spec["secondary_keys"]:
            extra_keys = ", ".join(axis_spec["secondary_keys"])
            analysis_notes.append(
                f"Additional varying fields detected outside the matrix axes: {extra_keys}. Interpret matrix trends together with the group summary table."
            )

        distance_profile_matrix = _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "distance_mm", "stat_name": "median"}
        )
        distance_p95_matrix = _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "distance_mm", "stat_name": "p95"}
        )
        angle_median_matrix = _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "angle_max_deg", "stat_name": "median"}
        )
        angle_profile_matrix = _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "angle_max_deg", "stat_name": "p95"}
        )
        marker_error_p95_matrix = _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "marker_error", "stat_name": "p95"}
        )
        tracking_invalid_rate_matrix = 100.0 * _build_condition_matrix(
            metrics_by_take, axis_spec, {"metric_name": "tracking_invalid_flag", "stat_name": "mean"}
        )

        metric_profile_specs = (
            (
                "distance_median_profile",
                {
                    "matrix": distance_profile_matrix,
                    "title": "Distance Drift Median",
                    "ylabel": "Median distance drift (mm)",
                },
            ),
            (
                "distance_p95_profile",
                {
                    "matrix": distance_p95_matrix,
                    "title": "Distance Drift p95",
                    "ylabel": "Distance drift p95 (mm)",
                },
            ),
            (
                "angle_max_median_profile",
                {
                    "matrix": angle_median_matrix,
                    "title": "Orientation Drift Median",
                    "ylabel": "Median max-axis angular drift (deg)",
                },
            ),
            (
                "angle_max_p95_profile",
                {
                    "matrix": angle_profile_matrix,
                    "title": "Orientation Drift p95",
                    "ylabel": "Max-axis angular drift p95 (deg)",
                },
            ),
            (
                "marker_error_p95_profile",
                {
                    "matrix": marker_error_p95_matrix,
                    "title": "Marker Error p95",
                    "ylabel": "Marker error p95",
                },
            ),
            (
                "tracking_invalid_rate_profile",
                {
                    "matrix": tracking_invalid_rate_matrix,
                    "title": "Invalid Tracking Rate",
                    "ylabel": "Invalid tracking frames (%)",
                },
            ),
        )
        for plot_name, spec in metric_profile_specs:
            plot_path = os.path.join(output_dir, f"{plot_name}.png")
            _write_profile_plot(plot_path, (spec,), axis_spec, note_lines=analysis_notes)
            matrix_plot_paths[plot_name] = plot_path

        profile_path = os.path.join(output_dir, "condition_profiles.png")
        _write_profile_plot(
            profile_path,
            (
                {
                    "matrix": distance_profile_matrix,
                    "title": "Distance Drift Median",
                    "ylabel": "Median distance drift (mm)",
                },
                {
                    "matrix": distance_p95_matrix,
                    "title": "Distance Drift p95",
                    "ylabel": "Distance drift p95 (mm)",
                },
                {
                    "matrix": angle_median_matrix,
                    "title": "Orientation Drift Median",
                    "ylabel": "Median max-axis angular drift (deg)",
                },
                {
                    "matrix": angle_profile_matrix,
                    "title": "Orientation Drift p95",
                    "ylabel": "Max-axis angular drift p95 (deg)",
                },
                {
                    "matrix": marker_error_p95_matrix,
                    "title": "Marker Error p95",
                    "ylabel": "Marker error p95",
                },
                {
                    "matrix": tracking_invalid_rate_matrix,
                    "title": "Invalid Tracking Rate",
                    "ylabel": "Invalid tracking frames (%)",
                },
            ),
            axis_spec,
            note_lines=analysis_notes,
        )
        matrix_plot_paths["condition_profiles"] = profile_path

    summary_table_path = os.path.join(output_dir, "condition_summary_table.png")
    _write_condition_summary_table(summary_table_path, group_rows)
    matrix_plot_paths["condition_summary_table"] = summary_table_path

    camera_marker_plot_path = os.path.join(output_dir, "camera_marker_distance_p95_3d.png")
    _write_camera_marker_3d_plot(
        camera_marker_plot_path,
        group_rows,
        "distance_mm_p95",
        "Camera Count vs Marker Count vs Positional Drift (p95)",
        "Distance drift p95 (mm)",
    )
    matrix_plot_paths["camera_marker_distance_p95_3d"] = camera_marker_plot_path

    plot_paths.update(matrix_plot_paths)
    return plot_paths, axis_spec, analysis_notes


def run_analysis(input_paths, output_dir=None, group_by=None):
    take_files = discover_take_files(input_paths)
    if not take_files:
        raise RuntimeError(f"No MoCap experiment take files found in: {input_paths}")

    payloads = [_load_take_file(path) for path in take_files]
    metrics_by_take = [compute_take_metrics(payload) for payload in payloads]
    metrics_by_take = [metrics for metrics in metrics_by_take if metrics is not None]
    if not metrics_by_take:
        raise RuntimeError("No valid target rigid body samples found in the provided take files.")

    resolved_group_by = group_by or metrics_by_take[0]["group_by"]
    grouped_metrics = defaultdict(lambda: defaultdict(list))
    grouped_take_files = defaultdict(list)
    grouped_reference_images = {}
    grouped_reference_source_path = {}
    grouped_camera_inventory = {}
    grouped_webcam_timelapse = {}
    grouped_take_configs = {}
    grouped_tracking_quality = defaultdict(
        lambda: {
            "tracking_valid_frame_count": 0,
            "tracking_invalid_frame_count": 0,
            "tracking_unknown_frame_count": 0,
            "tracking_known_frame_count": 0,
        }
    )

    for take_metrics in metrics_by_take:
        if group_by:
            group_values = [
                str(_get_nested_value(take_metrics["config"], key, default="n/a"))
                for key in resolved_group_by
            ]
            group_label = " | ".join(group_values)
        else:
            group_label = take_metrics["group_label"]

        grouped_take_files[group_label].append(take_metrics["source_path"])
        grouped_reference_images.setdefault(group_label, take_metrics.get("reference_images", []))
        grouped_reference_source_path.setdefault(group_label, take_metrics["source_path"])
        grouped_take_configs.setdefault(group_label, take_metrics.get("config", {}).get("take", {}))
        if group_label not in grouped_camera_inventory or grouped_camera_inventory[group_label] is None:
            grouped_camera_inventory[group_label] = take_metrics.get("mocap_camera_inventory")
        if group_label not in grouped_webcam_timelapse or grouped_webcam_timelapse[group_label] is None:
            grouped_webcam_timelapse[group_label] = take_metrics.get("webcam_timelapse")
        for metric_name, metric_values in take_metrics["metrics"].items():
            grouped_metrics[group_label][metric_name].extend(metric_values)
        take_quality_summary = take_metrics.get("quality_summary", {})
        group_quality_summary = grouped_tracking_quality[group_label]
        for key in group_quality_summary:
            group_quality_summary[key] += int(take_quality_summary.get(key, 0) or 0)

    output_dir = output_dir or os.path.join(os.path.dirname(take_files[0]), "..", "analysis")
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    take_quality_dir = os.path.join(output_dir, "take_quality")
    os.makedirs(take_quality_dir, exist_ok=True)

    group_rows = []
    for group_label in sorted(grouped_metrics.keys()):
        row = {
            "group_label": group_label,
            "take_count": len(grouped_take_files[group_label]),
            "frame_count": len(grouped_metrics[group_label]["distance_mm"]),
            "reference_images": grouped_reference_images.get(group_label, []),
            "reference_source_path": grouped_reference_source_path.get(group_label),
            "mocap_camera_inventory": grouped_camera_inventory.get(group_label),
            "webcam_timelapse": grouped_webcam_timelapse.get(group_label),
        }
        take_config = grouped_take_configs.get(group_label, {})
        row["workspace_position"] = take_config.get("workspace_position")
        row["marker_configuration"] = take_config.get("marker_configuration")
        row["camera_configuration"] = take_config.get("camera_configuration")
        row["cover_configuration"] = take_config.get("cover_configuration")
        row["marker_count"] = _label_to_numeric(take_config.get("marker_configuration"))
        row["camera_count"] = _label_to_numeric(take_config.get("camera_configuration"))
        metric_summary = _metric_summary_with_angle_max(grouped_metrics[group_label])
        for metric_name, stats in metric_summary.items():
            for stat_name, stat_value in stats.items():
                row[f"{metric_name}_{stat_name}"] = stat_value
        group_quality_summary = grouped_tracking_quality[group_label]
        row.update(group_quality_summary)
        known_count = group_quality_summary["tracking_known_frame_count"]
        invalid_count = group_quality_summary["tracking_invalid_frame_count"]
        row["tracking_invalid_rate"] = (float(invalid_count) / float(known_count)) if known_count else None
        row["tracking_invalid_rate_pct"] = (
            100.0 * row["tracking_invalid_rate"] if row["tracking_invalid_rate"] is not None else None
        )
        group_rows.append(row)

    take_rows = []
    for take_metrics in metrics_by_take:
        take_quality_plot_filename = f"{sanitize_slug(take_metrics['take_label'])}__quality.png"
        take_quality_plot_path = os.path.join(take_quality_dir, take_quality_plot_filename)
        quality_plot_created = _write_take_quality_plot(take_quality_plot_path, take_metrics)
        row = {
            "take_label": take_metrics["take_label"],
            "group_label": take_metrics["group_label"],
            "source_path": take_metrics["source_path"],
            "target_rigid_body": take_metrics["target_rigid_body"],
            "frame_count": take_metrics["frame_count"],
            "reference_images": take_metrics.get("reference_images", []),
            "mocap_camera_inventory": take_metrics.get("mocap_camera_inventory"),
            "webcam_timelapse": take_metrics.get("webcam_timelapse"),
            "quality_plot_path": os.path.relpath(take_quality_plot_path, output_dir) if quality_plot_created else None,
        }
        for metric_name, stats in take_metrics["summary"].items():
            for stat_name, stat_value in stats.items():
                row[f"{metric_name}_{stat_name}"] = stat_value
        row.update(take_metrics.get("quality_summary", {}))
        take_rows.append(row)

    plot_paths, axis_spec, analysis_notes = _generate_plots(grouped_metrics, group_rows, metrics_by_take, output_dir)

    summary = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now().isoformat(),
        "group_by": resolved_group_by,
        "input_paths": [os.path.abspath(path) for path in input_paths],
        "output_dir": output_dir,
        "plot_paths": {name: os.path.relpath(path, output_dir) for name, path in plot_paths.items()},
        "matrix_axes": axis_spec,
        "analysis_notes": analysis_notes,
        "headline": _build_report_headline(metrics_by_take, group_rows),
        "groups": group_rows,
        "takes": take_rows,
    }

    summary_path = os.path.join(output_dir, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    group_fieldnames = [
        key for key in (list(group_rows[0].keys()) if group_rows else ["group_label", "take_count", "frame_count"])
        if key not in {"reference_images", "reference_source_path", "mocap_camera_inventory", "webcam_timelapse"}
    ]
    take_fieldnames = [
        key for key in (list(take_rows[0].keys()) if take_rows else ["take_label", "group_label", "source_path", "target_rigid_body", "frame_count"])
        if key not in {"reference_images", "mocap_camera_inventory", "webcam_timelapse"}
    ]
    _write_csv(
        os.path.join(output_dir, "group_metrics.csv"),
        [{key: value for key, value in row.items() if key in group_fieldnames} for row in group_rows],
        group_fieldnames,
    )
    _write_csv(
        os.path.join(output_dir, "take_metrics.csv"),
        [{key: value for key, value in row.items() if key in take_fieldnames} for row in take_rows],
        take_fieldnames,
    )

    return summary


def _asset_path_for_report(output_dir, source_path, asset_entry):
    session_relative_path = asset_entry.get("session_relative_path")
    if not session_relative_path:
        return None
    session_dir = os.path.dirname(os.path.dirname(source_path))
    asset_abs_path = os.path.join(session_dir, session_relative_path)
    return os.path.relpath(asset_abs_path, output_dir)


def write_markdown_report(summary, output_path=None):
    output_dir = summary["output_dir"]
    output_path = output_path or os.path.join(output_dir, "report.md")

    lines = []
    headline = summary.get("headline", {})
    session_names = headline.get("session_names") or []
    report_title = " / ".join(session_names) if session_names else "MoCap Experiment Report"
    lines.append(f"# {report_title}")
    lines.append("")
    lines.append("## Headline Summary")
    lines.append("")
    if headline.get("experiment_names"):
        lines.append(f"- Experiment: `{', '.join(headline['experiment_names'])}`")
    if headline.get("focus_label"):
        lines.append(f"- Session focus: `{headline['focus_label']}`")
    catalog = headline.get("catalog") or {}
    if catalog:
        lines.append(f"- Workspace positions: `{', '.join(catalog.get('workspace_position', []))}`")
        lines.append(f"- Marker configurations: `{', '.join(catalog.get('marker_configuration', []))}`")
        lines.append(f"- Camera configurations: `{', '.join(catalog.get('camera_configuration', []))}`")
    if headline.get("best_distance_group") is not None:
        lines.append(
            f"- Best positional drift p95: `{headline['best_distance_group']}` -> `{headline['best_distance_value']:.3f} mm`"
        )
    if headline.get("worst_distance_group") is not None:
        lines.append(
            f"- Worst positional drift p95: `{headline['worst_distance_group']}` -> `{headline['worst_distance_value']:.3f} mm`"
        )
    if headline.get("best_angle_group") is not None:
        lines.append(
            f"- Best orientation drift p95: `{headline['best_angle_group']}` -> `{headline['best_angle_value']:.3f} deg`"
        )
    if headline.get("worst_angle_group") is not None:
        lines.append(
            f"- Worst orientation drift p95: `{headline['worst_angle_group']}` -> `{headline['worst_angle_value']:.3f} deg`"
        )
    if headline.get("best_marker_error_group") is not None:
        lines.append(
            f"- Lowest marker error p95: `{headline['best_marker_error_group']}` -> `{headline['best_marker_error_value']:.6f}`"
        )
    if headline.get("worst_invalid_tracking_group") is not None:
        lines.append(
            f"- Highest invalid tracking rate: `{headline['worst_invalid_tracking_group']}` -> `{headline['worst_invalid_tracking_value']:.2f} %`"
        )
    lines.append("")
    lines.append(f"- Generated at: `{summary['generated_at']}`")
    lines.append(f"- Grouped by: `{', '.join(summary['group_by'])}`")
    lines.append(f"- Number of takes: `{len(summary['takes'])}`")
    lines.append(f"- Number of groups: `{len(summary['groups'])}`")
    lines.append("")
    lines.append("## 3D Camera-Marker View")
    lines.append("")
    lines.append(
        "_X axis = camera count, Y axis = marker count, Z axis = positional drift p95. The figure includes one subplot per workspace plus one overall mean subplot._"
    )
    lines.append("")
    lines.append(f"![Camera-marker positional drift 3D]({summary['plot_paths']['camera_marker_distance_p95_3d']})")
    lines.append("")
    if summary.get("matrix_axes"):
        matrix_axes = summary["matrix_axes"]
        lines.append("## Condition Profiles")
        lines.append("")
        lines.append(
            f"- X axis: `{matrix_axes['x_key']}`"
        )
        lines.append(
            f"- Y axis: `{matrix_axes['y_key']}`"
        )
        if summary.get("analysis_notes"):
            for note in summary["analysis_notes"]:
                lines.append(f"- {note}")
        lines.append("")
        lines.append("### Combined Overview")
        lines.append("")
        lines.append(f"![Condition profiles]({summary['plot_paths']['condition_profiles']})")
        lines.append("")
        lines.append("### Distance Drift Median")
        lines.append("")
        lines.append(f"![Distance drift median profile]({summary['plot_paths']['distance_median_profile']})")
        lines.append("")
        lines.append("### Distance Drift p95")
        lines.append("")
        lines.append(f"![Distance drift p95 profile]({summary['plot_paths']['distance_p95_profile']})")
        lines.append("")
        lines.append("### Orientation Drift Median")
        lines.append("")
        lines.append(f"![Orientation drift median profile]({summary['plot_paths']['angle_max_median_profile']})")
        lines.append("")
        lines.append("### Orientation Drift p95")
        lines.append("")
        lines.append(f"![Orientation drift p95 profile]({summary['plot_paths']['angle_max_p95_profile']})")
        lines.append("")
        lines.append("### Marker Error p95")
        lines.append("")
        lines.append(f"![Marker error p95 profile]({summary['plot_paths']['marker_error_p95_profile']})")
        lines.append("")
        lines.append("### Invalid Tracking Rate")
        lines.append("")
        lines.append(f"![Invalid tracking rate profile]({summary['plot_paths']['tracking_invalid_rate_profile']})")
        lines.append("")
        lines.append("## Condition Summary Table")
        lines.append("")
        lines.append(f"![Condition summary table]({summary['plot_paths']['condition_summary_table']})")
        lines.append("")

    lines.append("## Group Summary")
    lines.append("")
    lines.append(
        "| Group | Takes | Frames | Distance median (mm) | Distance p95 (mm) | Angle max median (deg) | Angle max p95 (deg) | Marker error p95 | Invalid tracking (%) | X median (deg) | Y median (deg) | Z median (deg) |"
    )
    lines.append(
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"
    )
    for group in summary["groups"]:
        safe_group_label = str(group["group_label"]).replace("|", "/")
        lines.append(
            "| {group_label} | {take_count} | {frame_count} | {distance_mm_median:.3f} | {distance_mm_p95:.3f} | {angle_max_deg_median:.3f} | {angle_max_deg_p95:.3f} | {marker_error_p95:.6f} | {tracking_invalid_rate_pct:.2f} | {angle_x_deg_median:.3f} | {angle_y_deg_median:.3f} | {angle_z_deg_median:.3f} |".format(
                group_label=safe_group_label,
                take_count=group["take_count"],
                frame_count=group["frame_count"],
                distance_mm_median=group["distance_mm_median"] or 0.0,
                distance_mm_p95=group["distance_mm_p95"] or 0.0,
                angle_max_deg_median=group["angle_max_deg_median"] or 0.0,
                angle_max_deg_p95=group["angle_max_deg_p95"] or 0.0,
                marker_error_p95=group["marker_error_p95"] or 0.0,
                tracking_invalid_rate_pct=group["tracking_invalid_rate_pct"] or 0.0,
                angle_x_deg_median=group["angle_x_deg_median"] or 0.0,
                angle_y_deg_median=group["angle_y_deg_median"] or 0.0,
                angle_z_deg_median=group["angle_z_deg_median"] or 0.0,
            )
        )

    lines.append("")
    lines.append("## Appendix: Detailed Boxplots")
    lines.append("")
    lines.append("_These per-frame boxplots are retained for debugging, but the heatmaps, line profiles, and summary table above should be the primary comparison figures for multi-condition sessions._")
    lines.append("")
    lines.append(f"![Combined boxplots]({summary['plot_paths']['combined']})")
    lines.append("")
    for metric_name, title, _ in METRIC_SPECS:
        lines.append(f"### {title}")
        lines.append("")
        lines.append(f"![{title}]({summary['plot_paths'][metric_name]})")
        lines.append("")

    lines.append("")
    lines.append("## Take Quality Plots")
    lines.append("")
    for take in summary["takes"]:
        lines.append(f"### {take['take_label']}")
        lines.append("")
        lines.append(
            f"- Invalid tracking frames: `{take.get('tracking_invalid_frame_count', 0)}` / `{take.get('tracking_known_frame_count', 0)}`"
        )
        if take.get("tracking_invalid_rate_pct") is not None:
            lines.append(f"- Invalid tracking rate: `{take['tracking_invalid_rate_pct']:.2f} %`")
        if take.get("marker_error_p95") is not None:
            lines.append(f"- Marker error p95: `{take['marker_error_p95']:.6f}`")
        quality_plot_path = take.get("quality_plot_path")
        if quality_plot_path:
            lines.append("")
            lines.append(f"![Tracking quality {take['take_label']}]({quality_plot_path})")
        lines.append("")

    lines.append("")
    lines.append("## MoCap Camera Inventory")
    lines.append("")
    for group in summary["groups"]:
        safe_group_label = str(group["group_label"]).replace("|", "/")
        lines.append(f"### {safe_group_label}")
        lines.append("")
        inventory = group.get("mocap_camera_inventory")
        if not inventory:
            lines.append("_No camera inventory snapshot recorded for this group._")
            lines.append("")
            continue
        lines.append(f"- Camera count: `{inventory.get('camera_count', 0)}`")
        for camera in inventory.get("cameras", []):
            lines.append(f"- `{camera.get('name', 'unknown')}`")
        lines.append("")

    lines.append("")
    lines.append("## Configuration References")
    lines.append("")
    for group in summary["groups"]:
        safe_group_label = str(group["group_label"]).replace("|", "/")
        lines.append(f"### {safe_group_label}")
        lines.append("")
        reference_images = group.get("reference_images", []) or []
        if not reference_images:
            lines.append("_No reference images linked for this configuration._")
            lines.append("")
            continue
        for asset_entry in reference_images:
            if asset_entry.get("status") not in {"copied", "captured", "discovered"}:
                continue
            asset_path = _asset_path_for_report(output_dir, group.get("reference_source_path"), asset_entry)
            if asset_path is None:
                continue
            role = asset_entry.get("role", "image")
            lines.append(f"**{role.capitalize()}**")
            lines.append("")
            lines.append(f"![{role}]({asset_path})")
            lines.append("")

    lines.append("")
    lines.append("## Webcam Timelapse")
    lines.append("")
    for take in summary["takes"]:
        lines.append(f"### {take['take_label']}")
        lines.append("")
        timelapse = take.get("webcam_timelapse")
        if not timelapse:
            lines.append("_No webcam timelapse recorded for this take._")
            lines.append("")
            continue
        lines.append(f"- Status: `{timelapse.get('status', 'unknown')}`")
        if timelapse.get("frame_count") is not None:
            lines.append(f"- Captured frames: `{timelapse.get('frame_count', 0)}`")
        if timelapse.get("frame_interval_sec") is not None:
            lines.append(f"- Frame interval: `{timelapse.get('frame_interval_sec')}` sec")
        if timelapse.get("reason"):
            lines.append(f"- Reason: `{timelapse.get('reason')}`")
        video_path = _asset_path_for_report(output_dir, take["source_path"], timelapse)
        if video_path is not None and timelapse.get("status") == "created":
            lines.append(f"- Video: `{video_path}`")
            lines.append("")
            lines.append(
                f'<video controls preload="metadata" src="{video_path}" style="max-width: 100%; height: auto;"></video>'
            )
            lines.append("")
        else:
            lines.append("")

    lines.append("")
    lines.append("## Take Files")
    lines.append("")
    for take in summary["takes"]:
        lines.append(
            f"- `{take['take_label']}`: `{take['source_path']}` ({take['frame_count']} frames)"
        )
        reference_images = take.get("reference_images", []) or []
        linked_images = [
            entry for entry in reference_images
            if entry.get("status") in {"copied", "captured", "discovered"}
        ]
        for asset_entry in linked_images:
            asset_path = _asset_path_for_report(output_dir, take["source_path"], asset_entry)
            if asset_path is None:
                continue
            lines.append(f"  - `{asset_entry.get('role', 'image')}`: `{asset_path}`")

    with open(output_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return output_path


def _build_arg_parser(command_name):
    parser = argparse.ArgumentParser(prog=command_name)
    parser.add_argument("inputs", nargs="+", help="Take JSON files, take directories, or a session directory.")
    parser.add_argument("--output-dir", dest="output_dir", default=None, help="Output directory for plots and summaries.")
    parser.add_argument(
        "--group-by",
        nargs="+",
        default=None,
        help="Override grouping keys, for example: take.workspace_position take.marker_configuration",
    )
    return parser


def main_analyze():
    parser = _build_arg_parser("mocap_experiment_analyze")
    args = parser.parse_args()
    summary = run_analysis(args.inputs, output_dir=args.output_dir, group_by=args.group_by)
    print(f"Summary written to {os.path.join(summary['output_dir'], 'summary.json')}")
    print(f"Plots written to {summary['output_dir']}")


def main_report():
    parser = _build_arg_parser("mocap_experiment_report")
    parser.add_argument("--report-path", dest="report_path", default=None, help="Optional markdown report output path.")
    args = parser.parse_args()
    summary = run_analysis(args.inputs, output_dir=args.output_dir, group_by=args.group_by)
    report_path = write_markdown_report(summary, output_path=args.report_path)
    print(f"Report written to {report_path}")
