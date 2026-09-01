from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "exp/scripts/table5_n200_aggregate.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "table5_n200_aggregate_tested", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


AGG = _load_module()


def _row(dataset_id: str, *, collision: bool | None = True) -> dict:
    joint = {
        "name": "hinge",
        "type": "revolute",
        "parent": "base",
        "child": "door",
        "lower": 0.0,
        "upper": 1.0,
        "effort": 1.0,
        "velocity": 1.0,
    }
    return {
        "dataset_slug": "demo",
        "dataset_name": "Demo",
        "dataset_id": dataset_id,
        "category": "cabinet",
        "joint_tree": {"links": ["base", "door"], "joints": [joint]},
        "scalar_joints": [joint],
        "strict_gates": {
            "strict_urdf_pass": True,
            "strict_kinematic_pass": True,
            "strict_collision_pass": collision,
        },
        "bounding_box_diagonal": 2.0,
    }


def _manifest(rows: list[dict]) -> dict:
    for row in rows:
        row["row_sha256"] = AGG._canonical_sha256(row, exclude_fields=("row_sha256",))
    return {
        "protocol": {
            "cross_simulator": {
                "joint_rmse": {"sample_steps": [0, 1]},
                "thresholds": {
                    "normalized_joint_rmse": 0.10,
                    "translation_over_bbox_diagonal": 0.02,
                    "rotation_rad": 0.10,
                },
            }
        },
        "datasets": [{"dataset_slug": "demo", "dataset_name": "Demo", "rows": rows}],
    }


def _record(trajectory: list[float], translation_x: float) -> dict:
    metrics = {name: True for name in AGG.TABLE5A_METRICS}
    evaluation = {
        "metrics": dict(metrics),
        "support": {
            "joints": [
                {
                    "name": "hinge",
                    "type": "revolute",
                    "eligible": True,
                    "runtime_mapped": True,
                }
            ]
        },
        "diagnostics": {
            "actuation": [
                {
                    "joint_name": "hinge",
                    "joint_type": "revolute",
                    "trajectory": {
                        "sample_steps": [0, 1],
                        "normalized_positions": trajectory,
                    },
                    "final_descendant_root_frame_poses": {
                        "door": {
                            "translation": [translation_x, 0.0, 0.0],
                            "rotation": [0.0, 0.0, 0.0, 1.0],
                        }
                    },
                    "missing_descendant_link_names": [],
                }
            ]
        },
    }
    return {
        "terminal_status": "completed",
        "metrics": metrics,
        "evaluation": evaluation,
    }


def _write_record(
    root: Path, simulator: str, dataset_id: str, record: dict, manifest: dict
) -> None:
    source_row = next(
        row
        for dataset in manifest["datasets"]
        for row in dataset["rows"]
        if row["dataset_id"] == dataset_id
    )
    record = dict(record)
    record["identity"] = {
        "dataset_slug": "demo",
        "dataset_name": "Demo",
        "dataset_id": dataset_id,
        "simulator": simulator,
        "manifest_sha256": AGG._canonical_sha256(manifest),
        "protocol_sha256": AGG._canonical_sha256(manifest["protocol"]),
        "row_sha256": source_row["row_sha256"],
    }
    path = root / "runtime" / "demo" / simulator / "assets" / f"{dataset_id}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record), encoding="utf-8")


def test_full_n_rates_and_cross_sim_coverage(tmp_path: Path) -> None:
    rows = [_row("asset_1"), _row("asset_2", collision=False)]
    manifest = _manifest(rows)
    _write_record(tmp_path, "pybullet", "asset_1", _record([0.0, 1.0], 0.00), manifest)
    _write_record(tmp_path, "genesis", "asset_1", _record([0.0, 1.02], 0.02), manifest)
    _write_record(tmp_path, "mujoco", "asset_1", _record([0.0, 0.98], 0.04), manifest)

    summary = AGG.aggregate_manifest(manifest, tmp_path)
    dataset = summary["datasets"]["demo"]
    table5a, table5b = dataset["table5a"], dataset["table5b"]

    assert table5a["pybullet"]["load"]["passed"] == 1
    assert table5a["pybullet"]["load"]["denominator"] == 2
    assert table5b["all_three_runtime_pass"]["passed"] == 1
    revolute = table5b["joint_normalized_trajectory_pairwise_max_rmse"]["revolute"]
    assert revolute["candidate_units"] == 2
    assert revolute["evaluable_units"] == 1
    assert revolute["coverage"]["percentage"] == 50.0
    assert math.isclose(revolute["population_max"], math.sqrt(0.04**2 / 2))
    pose = table5b["final_descendant_link_pose_pairwise_max_error"]
    assert pose["candidate_units"] == 2
    assert pose["evaluable_units"] == 1
    assert math.isclose(pose["translation_over_bbox_diagonal"]["population_max"], 0.02)
    assert table5b["strict_consistency"]["passed"] == 1
    assert table5b["strict_sim_ready"]["passed"] == 1
    assert table5b["strict_sim_ready"]["denominator"] == 2


def test_null_upstream_gate_is_ne_not_zero(tmp_path: Path) -> None:
    row = _row("physx_1", collision=None)
    manifest = _manifest([row])
    for simulator in AGG.SIMULATORS:
        _write_record(
            tmp_path,
            simulator,
            "physx_1",
            _record([0.0, 1.0], 0.0),
            manifest,
        )

    summary = AGG.aggregate_manifest(manifest, tmp_path)
    table5b = summary["datasets"]["demo"]["table5b"]

    collision = table5b["upstream_strict_gates"]["strict_collision_pass"]
    assert collision["status"] == "not_evaluable"
    assert collision["passed"] is None
    assert collision["denominator"] == 1
    assert table5b["strict_sim_ready"]["status"] == "not_evaluable"
    assert table5b["strict_sim_ready"]["passed"] is None


def test_atomic_output_bundle(tmp_path: Path) -> None:
    row = _row("asset_1")
    manifest = _manifest([row])
    for simulator in AGG.SIMULATORS:
        _write_record(
            tmp_path,
            simulator,
            "asset_1",
            _record([0.0, 1.0], 0.0),
            manifest,
        )
    summary = AGG.aggregate_manifest(manifest, tmp_path)
    output = tmp_path / "aggregate"

    AGG.write_outputs(summary, output)

    assert {path.name for path in output.iterdir()} == {
        "summary.json",
        "table5a.csv",
        "table5b.csv",
        "report.md",
    }
    saved = json.loads((output / "summary.json").read_text())
    assert saved["schema_version"] == AGG.SCHEMA_VERSION
    assert "Strict Sim-ready" in (output / "report.md").read_text()


def test_preflight_failure_with_null_tree_is_a_complete_terminal_failure(
    tmp_path: Path,
) -> None:
    row = {
        "dataset_slug": "demo",
        "dataset_name": "Demo",
        "dataset_id": "failed_1",
        "category": "unknown",
        "joint_tree": None,
        "scalar_joints": [],
        "preflight": {
            "status": "failed",
            "issues": ["xml_parse_failure"],
            "simulator_eligible": False,
        },
        "bounding_box_diagonal": "N/E",
        "strict_gates": {
            "table2": {"strict_urdf_pass": False},
            "table3": {"strict_kinematic_pass": None},
            "table4": {"strict_collision_pass": None},
        },
    }
    manifest = _manifest([row])
    failure = {
        "terminal_status": "preflight_failure",
        "metrics": {name: False for name in AGG.TABLE5A_METRICS},
        "evaluation": None,
    }
    for simulator in AGG.SIMULATORS:
        _write_record(tmp_path, simulator, "failed_1", failure, manifest)

    summary = AGG.aggregate_manifest(manifest, tmp_path)
    dataset = summary["datasets"]["demo"]

    assert summary["classification"] == "COMPLETE"
    assert dataset["completeness"]["terminal_records"] == 3
    assert dataset["table5a"]["pybullet"]["load"]["passed"] == 0
    joint = dataset["table5b"]["joint_rmse"]
    assert joint["revolute"]["candidate_units"] == 0
    assert joint["prismatic"]["candidate_units"] == 0
    assert dataset["table5b"]["strict_sim_ready"]["status"] == "not_evaluable"


def test_continuous_joint_is_valid_manifest_but_not_cross_sim_candidate(
    tmp_path: Path,
) -> None:
    row = _row("continuous_1")
    continuous = dict(row["scalar_joints"][0])
    continuous.update({"type": "continuous", "lower": None, "upper": None})
    row["joint_tree"]["joints"] = [continuous]
    row["scalar_joints"] = [continuous]
    manifest = _manifest([row])

    summary = AGG.aggregate_manifest(manifest, tmp_path)
    table5b = summary["datasets"]["demo"]["table5b"]

    assert table5b["joint_rmse"]["revolute"]["candidate_units"] == 0
    assert table5b["joint_rmse"]["prismatic"]["candidate_units"] == 0
    assert table5b["strict_consistency"]["passed"] == 0
