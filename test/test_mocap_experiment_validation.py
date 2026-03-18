from datetime import datetime

import pytest

from husky_assembly_teleop import mocap_experiment


def _make_config():
    return {
        "experiment": {
            "name": "validation_test",
            "session_name": "validation_test_session",
            "duration_sec": 35.0,
        },
        "take": {
            "take_id": "validation_take",
            "workspace_position": "center",
            "marker_configuration": "markers_8",
            "camera_configuration": "Cam_21",
            "cover_configuration": "cover",
            "trial_index": 1,
            "reference_images": {
                "overview": "",
                "workspace": "",
                "markers": "",
                "camera": "",
                "cover": "",
                "extra": [],
            },
        },
        "capture": {
            "workspace_webcam": {
                "enabled": False,
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


def _make_output_paths(tmp_path, config):
    session_dir = tmp_path / "session"
    return {
        "recorded_at": datetime(2026, 3, 18, 12, 0, 0),
        "date_folder": "20260318",
        "session_dir": str(session_dir),
        "takes_dir": str(session_dir / "takes"),
        "analysis_dir": str(session_dir / "analysis"),
        "take_path": str(session_dir / "takes" / "take.json"),
        "manifest_path": str(session_dir / "manifest.json"),
        "take_label": mocap_experiment.format_take_label(config),
        "take_stem": "test_take",
        "reference_media_dir": str(session_dir / "reference_media" / "test_take"),
        "photo_library_dir": str(session_dir / "photo_library"),
    }


def _frame(
    timestamp,
    elapsed_sec,
    position,
    quaternion,
    rigid_body_name="/a200_0806",
    marker_error=None,
    tracking_valid=None,
):
    return {
        "timestamp": float(timestamp),
        "elapsed_sec": float(elapsed_sec),
        "rigid_bodies": {
            rigid_body_name: {
                "position_m": [float(value) for value in position],
                "quaternion_xyzw": [float(value) for value in quaternion],
                "marker_error": marker_error,
                "tracking_valid": tracking_valid,
            }
        },
    }


def test_build_take_payload_rejects_frozen_target_track(tmp_path):
    config = _make_config()
    output_paths = _make_output_paths(tmp_path, config)
    frames = [
        _frame(0.0, 0.0, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0]),
        _frame(0.1, 0.1, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0]),
        _frame(0.2, 0.2, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0]),
    ]

    with pytest.raises(mocap_experiment.InvalidTakeDataError, match="frozen MoCap data"):
        mocap_experiment.build_take_payload(
            config=config,
            config_path=str(tmp_path / "config.yaml"),
            output_paths=output_paths,
            target_rigid_body="/a200_0806",
            selected_robot_id=0,
            frames=frames,
            rigid_body_ids={"/a200_0806": 1},
            stop_reason="duration_elapsed",
        )


def test_build_take_payload_accepts_moving_target_track(tmp_path):
    config = _make_config()
    output_paths = _make_output_paths(tmp_path, config)
    frames = [
        _frame(0.0, 0.0, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0]),
        _frame(0.1, 0.1, [0.10001, 0.2, 0.3], [0.0, 0.0, 0.001, 0.9999995]),
        _frame(0.2, 0.2, [0.10002, 0.2, 0.3], [0.0, 0.0, 0.002, 0.999998]),
    ]

    payload = mocap_experiment.build_take_payload(
        config=config,
        config_path=str(tmp_path / "config.yaml"),
        output_paths=output_paths,
        target_rigid_body="/a200_0806",
        selected_robot_id=0,
        frames=frames,
        rigid_body_ids={"/a200_0806": 1},
        stop_reason="duration_elapsed",
    )

    assert payload["frame_count"] == 3


def test_compute_take_metrics_includes_quality_metrics(tmp_path):
    payload = {
        "_source_path": str(tmp_path / "session" / "takes" / "take.json"),
        "target_rigid_body": "/a200_0806",
        "take_label": "quality_take",
        "config": _make_config(),
        "reference_images": [],
        "frames": [
            _frame(0.0, 0.0, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], marker_error=0.001, tracking_valid=True),
            _frame(0.1, 0.1, [0.1001, 0.2, 0.3], [0.0, 0.0, 0.001, 0.9999995], marker_error=0.002, tracking_valid=False),
            _frame(0.2, 0.2, [0.1002, 0.2, 0.3], [0.0, 0.0, 0.002, 0.999998], marker_error=0.003, tracking_valid=True),
        ],
    }

    metrics = mocap_experiment.compute_take_metrics(payload)

    assert metrics["summary"]["marker_error"]["p95"] == pytest.approx(0.0029)
    assert metrics["summary"]["tracking_invalid_flag"]["mean"] == pytest.approx(1.0 / 3.0)
    assert metrics["quality_summary"]["tracking_invalid_frame_count"] == 1
    assert metrics["quality_summary"]["tracking_invalid_rate_pct"] == pytest.approx(100.0 / 3.0)


def test_run_analysis_outputs_quality_artifacts(tmp_path):
    config = _make_config()
    output_paths = _make_output_paths(tmp_path, config)
    frames = [
        _frame(0.0, 0.0, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], marker_error=0.001, tracking_valid=True),
        _frame(0.1, 0.1, [0.1001, 0.2, 0.3], [0.0, 0.0, 0.001, 0.9999995], marker_error=0.002, tracking_valid=False),
        _frame(0.2, 0.2, [0.1002, 0.2, 0.3], [0.0, 0.0, 0.002, 0.999998], marker_error=0.003, tracking_valid=True),
    ]
    payload = mocap_experiment.build_take_payload(
        config=config,
        config_path=str(tmp_path / "config.yaml"),
        output_paths=output_paths,
        target_rigid_body="/a200_0806",
        selected_robot_id=0,
        frames=frames,
        rigid_body_ids={"/a200_0806": 1},
        stop_reason="duration_elapsed",
    )
    take_path = tmp_path / "session" / "takes" / "take.json"
    take_path.parent.mkdir(parents=True, exist_ok=True)
    with open(take_path, "w") as f:
        import json

        json.dump(payload, f)

    summary = mocap_experiment.run_analysis([str(take_path)], output_dir=str(tmp_path / "analysis"))

    assert "marker_error_p95_profile" not in summary["plot_paths"] or summary["plot_paths"]["marker_error_p95_profile"]
    assert summary["takes"][0]["tracking_invalid_frame_count"] == 1
    assert summary["takes"][0]["quality_plot_path"]
    assert (tmp_path / "analysis" / summary["takes"][0]["quality_plot_path"]).is_file()

    report_path = mocap_experiment.write_markdown_report(summary, output_path=str(tmp_path / "analysis" / "report.md"))
    report_text = report_path and open(report_path).read()
    assert "Invalid tracking rate" in report_text
    assert "Marker error p95" in report_text
