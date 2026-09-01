from __future__ import annotations

import csv
import importlib.util
import io
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "exp/scripts/table5_n200_hybrid_aggregate.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "table5_n200_hybrid_aggregate_tested", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


AGG = _load_module()


def _row(dataset_id: str = "asset_0000") -> dict:
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
    row = {
        "dataset_slug": "demo",
        "dataset_name": "Demo",
        "dataset_id": dataset_id,
        "asset_id": f"source/{dataset_id}",
        "category": "cabinet",
        "urdf_path": f"assets/{dataset_id}/model.urdf",
        "urdf_sha256": "a" * 64,
        "joint_tree": {"links": ["base", "door"], "joints": [joint]},
        "scalar_joints": [joint],
        "bounding_box_diagonal": 2.0,
        "strict_gates": {
            "strict_urdf_pass": True,
            "strict_kinematic_pass": True,
            "strict_collision_pass": True,
        },
    }
    row["row_sha256"] = AGG._full._canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def _manifest(rows: list[dict]) -> dict:
    return {
        "protocol": {
            "cross_simulator": {
                "thresholds": {
                    "normalized_joint_rmse": 0.10,
                    "translation_over_bbox_diagonal": 0.02,
                    "rotation_rad": 0.10,
                }
            }
        },
        "datasets": [{"dataset_slug": "demo", "dataset_name": "Demo", "rows": rows}],
    }


def _authority(manifest: dict, row: dict, simulator: str) -> dict:
    return {
        "dataset_slug": "demo",
        "dataset_name": "Demo",
        "dataset_id": row["dataset_id"],
        "asset_id": row["asset_id"],
        "simulator": simulator,
        "manifest_sha256": AGG._full._canonical_sha256(manifest),
        "protocol_sha256": AGG._full._canonical_sha256(manifest["protocol"]),
        "row_sha256": row["row_sha256"],
        "urdf_path": f"/intent/{row['dataset_id']}/model.urdf",
        "urdf_sha256": row["urdf_sha256"],
    }


def _full_record(manifest: dict, row: dict, *, load: bool = True) -> dict:
    metrics = {metric: True for metric in AGG.TABLE5A_METRICS}
    metrics["load"] = load
    if not load:
        for metric in AGG.TABLE5A_METRICS:
            metrics[metric] = False
    return {
        "schema_version": "table5_n200_runtime_asset_v1",
        "terminal": True,
        "terminal_status": "completed",
        "identity": {
            **_authority(manifest, row, "genesis"),
            "executable": "/envs/genesis/bin/python",
            "worker_source_sha256": "b" * 64,
            "timeout_s": 300.0,
            "effective_workers": 1,
        },
        "metrics": metrics,
        "evaluation": {"metrics": dict(metrics)},
        "failure": None,
        "process": {},
    }


def _load_only_record(
    manifest: dict,
    row: dict,
    simulator: str,
    *,
    load: bool,
    status: str = "completed",
) -> dict:
    profile = {
        "execution_profile": AGG.LOAD_ONLY_PROFILE_NAME,
        "execution_profile_sha256": AGG.LOAD_ONLY_PROFILE_SHA256,
        "planned_metrics": ["load"],
        "not_evaluated_metrics": list(AGG.NOT_EVALUATED_METRICS),
    }
    failure = None if status == "completed" else {"kind": status, "message": status}
    evaluation = {
        "schema_version": AGG.LOAD_ONLY_EVALUATION_SCHEMA,
        **profile,
        "metrics": {"load": load},
        "load": {"strict_load": load} if status == "completed" else None,
        "support": None,
        "diagnostics": {"warnings": [], "errors": []},
        "failure": failure,
    }
    return {
        "schema_version": AGG.LOAD_ONLY_ASSET_SCHEMA,
        "terminal": True,
        "terminal_status": status,
        "identity": {
            **_authority(manifest, row, simulator),
            "executable": f"/envs/{simulator}/bin/python",
            "runner_source_sha256": "c" * 64,
            "timeout_s": 300.0,
            "effective_workers": 2,
            "execution_profile": AGG.LOAD_ONLY_PROFILE_NAME,
            "execution_profile_sha256": AGG.LOAD_ONLY_PROFILE_SHA256,
        },
        **profile,
        "metrics": {"load": load},
        "evaluation": evaluation,
        "failure": failure,
        "process": {},
    }


def _write_full(root: Path, row: dict, record: dict) -> None:
    path = root / "runtime/demo/genesis/assets" / f"{row['dataset_id']}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record), encoding="utf-8")


def _write_load_only(root: Path, row: dict, simulator: str, record: dict) -> None:
    path = root / "demo" / simulator / "assets" / f"{row['dataset_id']}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record), encoding="utf-8")


def _write_complete_plan(
    run_root: Path,
    load_root: Path,
    manifest: dict,
    row: dict,
    *,
    pybullet_load: bool = True,
    mujoco_load: bool = True,
) -> None:
    _write_full(run_root, row, _full_record(manifest, row))
    _write_load_only(
        load_root,
        row,
        "pybullet",
        _load_only_record(manifest, row, "pybullet", load=pybullet_load),
    )
    _write_load_only(
        load_root,
        row,
        "mujoco",
        _load_only_record(manifest, row, "mujoco", load=mujoco_load),
    )


def test_hybrid_scope_matrix_and_atomic_outputs(tmp_path: Path) -> None:
    row = _row()
    manifest = _manifest([row])
    run_root = tmp_path / "full"
    load_root = tmp_path / "load-only"
    _write_complete_plan(
        run_root,
        load_root,
        manifest,
        row,
        pybullet_load=True,
        mujoco_load=False,
    )

    summary = AGG.aggregate_manifest(manifest, run_root, load_root)
    dataset = summary["datasets"]["demo"]
    table5a = dataset["table5a"]
    table5b = dataset["table5b"]

    assert AGG.LOAD_ONLY_PROFILE_SHA256 == (
        "276be11ea4a7c3395ca89683b5ced5812fd1572c0ccf753a67b2a41a8cf07688"
    )
    assert summary["classification"] == "COMPLETE"
    assert summary["table5_scope"] == "PARTIAL"
    assert summary["completeness"]["expected_records"] == 3
    assert summary["completeness"]["terminal_records"] == 3
    assert table5a["pybullet"]["load"]["passed"] == 1
    assert table5a["mujoco"]["load"]["passed"] == 0
    assert table5a["genesis"]["simulator_pass"]["passed"] == 1
    for simulator in ("pybullet", "mujoco"):
        for metric in AGG.NOT_EVALUATED_METRICS:
            assert table5a[simulator][metric]["status"] == "not_evaluable"
            assert table5a[simulator][metric]["passed"] is None
    assert table5b["per_simulator_pass"]["pybullet"]["status"] == "not_evaluable"
    assert table5b["per_simulator_pass"]["genesis"]["passed"] == 1
    assert table5b["per_simulator_pass"]["mujoco"]["status"] == "not_evaluable"
    assert table5b["all_three_load"]["passed"] == 0
    assert table5b["all_three_runtime_pass"]["status"] == "not_evaluable"
    assert table5b["joint_rmse"]["revolute"]["status"] == "not_evaluable"
    assert table5b["joint_rmse"]["revolute"]["coverage"]["status"] == "not_evaluable"
    assert table5b["link_pose_error"]["status"] == "not_evaluable"
    assert table5b["strict_consistency"]["status"] == "not_evaluable"
    assert table5b["strict_sim_ready"]["status"] == "not_evaluable"
    assert table5b["strict_urdf_pass"]["passed"] == 1

    out = tmp_path / "aggregate"
    AGG.write_outputs(summary, out)
    assert {path.name for path in out.iterdir()} == {
        "summary.json",
        "table5a.csv",
        "table5b.csv",
        "report.md",
    }
    saved = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert saved["schema_version"] == AGG.SCHEMA_VERSION
    report = (out / "report.md").read_text(encoding="utf-8")
    assert "TABLE 5 SCOPE: PARTIAL" in report
    assert "None" not in report
    table5a_rows = list(
        csv.DictReader(io.StringIO((out / "table5a.csv").read_text(encoding="utf-8")))
    )
    pybullet_row = next(
        item for item in table5a_rows if item["simulator"] == "pybullet"
    )
    assert pybullet_row["load_display"] == "1 / 1 (100.000%)"
    assert pybullet_row["reset_display"] == "N/E"
    table5b_csv = (out / "table5b.csv").read_text(encoding="utf-8")
    assert "N/E" in table5b_csv


def test_missing_planned_record_is_incomplete_and_load_fails_closed(
    tmp_path: Path,
) -> None:
    row = _row()
    manifest = _manifest([row])
    run_root = tmp_path / "full"
    load_root = tmp_path / "load-only"
    _write_full(run_root, row, _full_record(manifest, row))
    _write_load_only(
        load_root,
        row,
        "mujoco",
        _load_only_record(manifest, row, "mujoco", load=False, status="timeout"),
    )

    summary = AGG.aggregate_manifest(manifest, run_root, load_root)
    dataset = summary["datasets"]["demo"]

    assert summary["classification"] == "INCOMPLETE"
    assert summary["table5_scope"] == "PARTIAL"
    assert summary["completeness"] == {
        "classification": "INCOMPLETE",
        "expected_records": 3,
        "present_records": 2,
        "valid_records": 2,
        "terminal_records": 2,
        "missing_or_invalid_records": 1,
    }
    assert dataset["table5a"]["pybullet"]["load"] == {
        "status": "evaluated",
        "passed": 0,
        "denominator": 1,
        "percentage": 0.0,
    }
    assert dataset["table5a"]["pybullet"]["reset"]["status"] == "not_evaluable"
    assert dataset["table5a"]["mujoco"]["load"]["passed"] == 0
    assert (
        dataset["completeness"]["by_simulator"]["mujoco"]["classification"]
        == "COMPLETE"
    )


def test_identity_mismatch_is_present_but_untrusted(tmp_path: Path) -> None:
    row = _row()
    manifest = _manifest([row])
    run_root = tmp_path / "full"
    load_root = tmp_path / "load-only"
    _write_complete_plan(run_root, load_root, manifest, row)
    bad = _load_only_record(manifest, row, "pybullet", load=True)
    bad["identity"]["row_sha256"] = "0" * 64
    _write_load_only(load_root, row, "pybullet", bad)

    summary = AGG.aggregate_manifest(manifest, run_root, load_root)
    pybullet = summary["datasets"]["demo"]["completeness"]["by_simulator"]["pybullet"]

    assert summary["classification"] == "INCOMPLETE"
    assert pybullet["present_records"] == 1
    assert pybullet["valid_records"] == 0
    assert pybullet["terminal_records"] == 0
    assert pybullet["record_state_counts"] == {"identity_mismatch": 1}
    assert summary["datasets"]["demo"]["table5a"]["pybullet"]["load"]["passed"] == 0


def test_cli_writes_bundle_from_synthetic_inputs(tmp_path: Path) -> None:
    row = _row()
    manifest = _manifest([row])
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    run_root = tmp_path / "full"
    load_root = tmp_path / "load-only"
    out = tmp_path / "out"
    _write_complete_plan(run_root, load_root, manifest, row)

    assert (
        AGG.main(
            [
                "--manifest",
                str(manifest_path),
                "--run-root",
                str(run_root),
                "--load-only-root",
                str(load_root),
                "--out",
                str(out),
            ]
        )
        == 0
    )
    assert (
        json.loads((out / "summary.json").read_text(encoding="utf-8"))["table5_scope"]
        == "PARTIAL"
    )
