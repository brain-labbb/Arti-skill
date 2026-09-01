from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_ours_pva_per_class_n5.py"


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "run_urdf_table4_ours_pva_per_class_n5_test", RUNNER
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_formal_cohort_loader_binds_exact_stratified_manifest() -> None:
    runner = load_runner()

    cohort = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)

    assert runner.DATASET_LABEL == "Ours per-class N=5 (supplementary)"
    assert cohort["file_sha256"] == runner.EXPECTED_COHORT_FILE_SHA256
    assert cohort["content_sha256"] == runner.EXPECTED_COHORT_CONTENT_SHA256
    assert cohort["n_release"] == 302_440
    assert cohort["release_category_count"] == 531
    assert cohort["per_class"] == 5
    assert len(cohort["rows"]) == 2_655
    assert len({row["raw_category"] for row in cohort["rows"]}) == 531
    assert cohort["rows"][0]["asset_id"] == "PV-A/Accessories_Cushion/seed_0021"
    assert cohort["rows"][-1]["asset_id"] == "PV-A/zippo_lighter/seed_0303"
    assert all(Path(row["package"]).is_absolute() for row in cohort["rows"])


def test_frozen_items_bind_each_package_and_state_denominator() -> None:
    runner = load_runner()
    cohort = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)
    rows = cohort["rows"][:3]
    audits = {
        row["asset_id"]: runner.audit_asset(Path("/ignored"), row) for row in rows
    }

    items = runner.build_frozen_items(rows, audits, {"fixture": True})

    assert [item["dataset_id"] for item in items] == [row["asset_id"] for row in rows]
    assert [item["package"] for item in items] == [row["package"] for row in rows]
    assert all(item["package_audit_success"] for item in items)
    assert all(item["primary_urdf_relpath"].endswith("/model.urdf") for item in items)
    assert all(item["package_binding_content_manifest_sha256"] for item in items)
    assert sum(item["movable_dof_count"] for item in items) == 5
    assert sum(
        item["rest_state_expected"]
        + item["single_state_expected"]
        + item["sobol_state_expected"]
        for item in items
    ) == 300
    assert len({item["input_identity_sha256"] for item in items}) == 3

    changed_runtime = runner.build_frozen_items(
        rows, audits, {"fixture": "changed-after-freeze"}
    )
    assert [item["input_identity_sha256"] for item in changed_runtime] != [
        item["input_identity_sha256"] for item in items
    ]


def test_formal_denominator_gate_rejects_joint_count_drift() -> None:
    runner = load_runner()
    items = []
    for index in range(2_655):
        movable = 12_314 if index == 0 else 1
        items.append(
            {
                "category": f"category_{index // 5:03d}",
                "movable_dof_count": movable,
                "rest_state_expected": 1,
                "single_state_expected": 21 * movable,
                "sobol_state_expected": 64,
            }
        )

    assert runner.validate_formal_items(items) == {
        "n_eval": 2_655,
        "category_count": 531,
        "j_eval": 14_968,
        "expected_states": 486_903,
    }

    items[-1]["movable_dof_count"] = 0
    with pytest.raises(ValueError, match="J_eval"):
        runner.validate_formal_items(items)


def test_finalized_manifest_exposes_downstream_cohort_binding(tmp_path: Path) -> None:
    runner = load_runner()
    manifest = {
        "schema_version": "base",
        "dataset": "base",
        "source": {},
        "selection": {},
        "items": [
            {
                "category": "Accessories_Cushion",
                "movable_dof_count": 1,
                "rest_state_expected": 1,
                "single_state_expected": 21,
                "sobol_state_expected": 64,
                "package": "/absolute/package",
                "package_binding_content_manifest_sha256": "a" * 64,
                "package_binding_file_count": 2,
                "package_binding_total_bytes": 3,
                "input_identity_sha256": "b" * 64,
            }
        ],
    }
    manifest["manifest_content_sha256"] = runner._manifest_self_hash(manifest)
    (tmp_path / "frozen_manifest.json").write_text(json.dumps(manifest))
    (tmp_path / "summary.json").write_text(
        json.dumps({"manifest_content_sha256": manifest["manifest_content_sha256"]})
    )
    (tmp_path / "checkpoint.json").write_text(
        json.dumps({"manifest_content_sha256": manifest["manifest_content_sha256"]})
    )

    runner._finalize_metadata(tmp_path, runner.DEFAULT_COHORT_MANIFEST)

    finalized = json.loads((tmp_path / "frozen_manifest.json").read_text())
    summary = json.loads((tmp_path / "summary.json").read_text())
    checkpoint = json.loads((tmp_path / "checkpoint.json").read_text())
    assert finalized["schema_version"] == "table4_ours_pva_per_class_n5_frozen_manifest_v1"
    assert finalized["dataset"] == "Ours per-class N=5 (supplementary)"
    assert finalized["protocol_id"] == runner.PROTOCOL_ID
    assert finalized["source"]["cohort_manifest_file_sha256"] == runner.EXPECTED_COHORT_FILE_SHA256
    assert finalized["source"]["n_release"] == 302_440
    assert finalized["source"]["n_eval"] == 1
    assert finalized["source"]["release_category_count"] == 531
    assert finalized["source"]["per_class"] == 5
    assert finalized["source"]["per_item_package_paths"] is True
    assert finalized["manifest_content_sha256"] == runner._manifest_self_hash(finalized)
    assert summary["manifest_content_sha256"] == finalized["manifest_content_sha256"]
    assert checkpoint["manifest_content_sha256"] == finalized["manifest_content_sha256"]


def test_initial_manifest_write_is_immediately_resumable(tmp_path: Path) -> None:
    runner = load_runner()
    manifest = {
        "schema_version": "table4_ours_500k_frozen_manifest_v1",
        "dataset": runner.DATASET_LABEL,
        "protocol_id": runner.PROTOCOL_ID,
        "evaluation": {"workers": 2, "child_timeout_seconds": 120.0},
        "items": [],
    }
    manifest["manifest_content_sha256"] = runner._manifest_self_hash(manifest)
    runner._active_execution_controls = {
        "workers": 2,
        "audit_workers": 2,
        "child_timeout_seconds": 120.0,
    }
    try:
        runner._atomic_json_with_controls(tmp_path / "frozen_manifest.json", manifest)
    finally:
        runner._active_execution_controls = None

    frozen = json.loads((tmp_path / "frozen_manifest.json").read_text())
    assert frozen["schema_version"] == (
        "table4_ours_pva_per_class_n5_frozen_manifest_v1"
    )
    assert frozen["evaluation"]["audit_workers"] == 2
    assert frozen["manifest_content_sha256"] == runner._manifest_self_hash(frozen)


def test_artifact_receipt_closes_outputs_and_detects_tampering(tmp_path: Path) -> None:
    runner = load_runner()
    manifest = {
        "schema_version": "table4_ours_pva_per_class_n5_frozen_manifest_v1",
        "dataset": "Ours per-class N=5 (supplementary)",
        "protocol_id": runner.PROTOCOL_ID,
        "items": [
            {
                "movable_dof_count": 1,
                "rest_state_expected": 1,
                "single_state_expected": 21,
                "sobol_state_expected": 64,
            }
        ],
    }
    manifest["manifest_content_sha256"] = runner._manifest_self_hash(manifest)
    files = {
        "frozen_manifest.json": json.dumps(manifest),
        "asset_records.jsonl": "{}\n",
        "state_records.jsonl": "{}\n",
        "summary.json": json.dumps({"n_eval": 1}),
        "report.md": "report\n",
        "verification.json": json.dumps(
            {"status": "PASS", "expected_states": 86, "executed_states": 86}
        ),
        "checkpoint.json": json.dumps({"state": "complete"}),
        "protocol_document_at_freeze.md": "protocol\n",
    }
    for name, payload in files.items():
        (tmp_path / name).write_text(payload)

    runner._write_receipts(
        tmp_path,
        started_at_utc="2026-08-24T00:00:00Z",
        wall_time_seconds=12.5,
        resume=False,
    )

    timing = json.loads((tmp_path / "timing.json").read_text())
    artifact = json.loads((tmp_path / "artifact_manifest.json").read_text())
    assert timing["wall_time_seconds"] == 12.5
    assert timing["n_eval"] == 1
    assert timing["j_eval"] == 1
    assert timing["expected_states"] == 86
    assert timing["executed_states"] == 86
    assert timing["resume"] is False
    assert set(artifact["files"]) == {
        "frozen_manifest.json",
        "asset_records.jsonl",
        "state_records.jsonl",
        "summary.json",
        "report.md",
        "verification.json",
        "checkpoint.json",
        "protocol_document_at_freeze.md",
        "timing.json",
    }
    assert runner.verify_artifact_receipt(tmp_path)["status"] == "PASS"

    (tmp_path / "report.md").write_text("tampered\n")
    failed = runner.verify_artifact_receipt(tmp_path)
    assert failed["status"] == "FAIL"
    assert failed["checks"]["artifact_hashes_match"] is False


def test_resume_preserves_initial_wall_timing_and_appends_history(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    manifest = {
        "schema_version": "table4_ours_pva_per_class_n5_frozen_manifest_v1",
        "dataset": runner.DATASET_LABEL,
        "protocol_id": runner.PROTOCOL_ID,
        "items": [
            {
                "movable_dof_count": 1,
                "rest_state_expected": 1,
                "single_state_expected": 21,
                "sobol_state_expected": 64,
            }
        ],
    }
    manifest["manifest_content_sha256"] = runner._manifest_self_hash(manifest)
    payloads = {
        "frozen_manifest.json": json.dumps(manifest),
        "asset_records.jsonl": "{}\n",
        "state_records.jsonl": "{}\n",
        "summary.json": json.dumps({"n_eval": 1}),
        "report.md": "report\n",
        "verification.json": json.dumps(
            {"status": "PASS", "expected_states": 86, "executed_states": 86}
        ),
        "checkpoint.json": json.dumps({"state": "complete"}),
        "protocol_document_at_freeze.md": "protocol\n",
    }
    for name, payload in payloads.items():
        (tmp_path / name).write_text(payload)
    runner._write_receipts(
        tmp_path,
        started_at_utc="2026-08-24T00:00:00Z",
        wall_time_seconds=12.5,
        resume=False,
        workers=2,
        audit_workers=1,
        child_timeout_seconds=30.0,
    )
    initial = json.loads((tmp_path / "timing.json").read_text())
    headline_keys = (
        "started_at_utc",
        "completed_at_utc",
        "wall_time_seconds",
        "resume",
    )
    initial_headline = {key: initial[key] for key in headline_keys}

    runner._write_receipts(
        tmp_path,
        started_at_utc="2026-08-24T01:00:00Z",
        wall_time_seconds=0.25,
        resume=True,
        workers=2,
        audit_workers=1,
        child_timeout_seconds=30.0,
    )

    resumed = json.loads((tmp_path / "timing.json").read_text())
    assert {key: resumed[key] for key in headline_keys} == initial_headline
    assert resumed["resume_invocation_count"] == 1
    assert resumed["cumulative_wall_time_seconds"] == 12.75
    assert resumed["resume_history"][0]["started_at_utc"] == "2026-08-24T01:00:00Z"
    assert resumed["resume_history"][0]["wall_time_seconds"] == 0.25
    assert resumed["workers"] == 2
    assert resumed["audit_workers"] == 1
    assert resumed["child_timeout_seconds"] == 30.0
    assert resumed["resume_history"][0]["workers"] == 2
    assert resumed["resume_history"][0]["audit_workers"] == 1
    assert resumed["resume_history"][0]["child_timeout_seconds"] == 30.0
    assert runner.verify_artifact_receipt(tmp_path)["status"] == "PASS"


def test_evaluator_resolves_the_frozen_absolute_package() -> None:
    runner = load_runner()
    row = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)["rows"][0]
    audit = runner.audit_asset(Path("/ignored"), row)
    item = runner.build_frozen_items(
        [row], {row["asset_id"]: audit}, runner.current_runtime_identity()
    )[0]

    result = runner.evaluate_asset(item, Path("/ignored"))

    assert result["dataset_id"] == "PV-A/Accessories_Cushion/seed_0021"
    assert result["load_success"] is True
    assert result["measurement_complete"] is True
    assert len(result["state_records"]) == 86
    assert result["state_records_sha256"] == runner.canonical_sha256(
        result["state_records"]
    )
    assert result["runner_sha256"] == runner.sha256_file(runner.SCRIPT)
    runner.verify_result_against_item(item, result)

    mutations = {
        "runtime binding": lambda value: value["runtime_identity"].update(
            {"pybullet_module_sha256": "0" * 64}
        ),
        "adapter": lambda value: value.update({"runner_sha256": "0" * 64}),
        "collision core": lambda value: value.update(
            {"collision_core_sha256": "0" * 64}
        ),
    }
    for message, mutate in mutations.items():
        changed = copy.deepcopy(result)
        mutate(changed)
        with pytest.raises(ValueError, match=message):
            runner.verify_result_against_item(item, changed)

    changed_denominator = copy.deepcopy(result)
    changed_denominator["single_state_expected"] += 1
    with pytest.raises(ValueError, match="frozen field"):
        runner.verify_result_against_item(item, changed_denominator)

    changed_state_identity = copy.deepcopy(result)
    changed_state_identity["state_records"][0]["dataset_id"] = "other/asset"
    changed_state_identity["state_records_sha256"] = runner.canonical_sha256(
        changed_state_identity["state_records"]
    )
    with pytest.raises(ValueError, match="state frozen field"):
        runner.verify_result_against_item(item, changed_state_identity)


def test_cli_contract_preserves_smoke_and_resume_boundaries() -> None:
    runner = load_runner()

    smoke = runner.base.parse_args(["--limit", "3", "--workers", "2"])
    runner.validate_args(smoke)
    assert smoke.limit == 3
    assert smoke.workers == 2

    formal = runner.base.parse_args([])
    runner.validate_args(formal)
    assert formal.limit is None
    assert formal.workers == 4
    assert formal.audit_workers == 4
    assert formal.child_timeout_seconds == 900.0

    for argv, message in (
        (["--workers", "3"], "formal.*workers=4"),
        (["--audit-workers", "3"], "formal.*audit-workers=4"),
        (["--child-timeout-seconds", "899"], "formal.*child-timeout-seconds=900"),
    ):
        with pytest.raises(ValueError, match=message):
            runner.validate_args(runner.base.parse_args(argv))

    for argv, message in (
        (["--limit", "0"], "limit"),
        (["--limit", "2656"], "limit"),
        (["--workers", "0"], "workers"),
        (["--resume"], "--output"),
    ):
        with pytest.raises(ValueError, match=message):
            runner.validate_args(runner.base.parse_args(argv))


def test_resume_rejects_control_or_record_runtime_drift_before_scheduler(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    row = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)["rows"][0]
    audit = runner.audit_asset(Path("/ignored"), row)
    runtime = runner.current_runtime_identity()
    item = runner.build_frozen_items(
        [row], {row["asset_id"]: audit}, runtime
    )[0]
    manifest = {
        "schema_version": "table4_ours_pva_per_class_n5_frozen_manifest_v1",
        "dataset": runner.DATASET_LABEL,
        "classification": "SMOKE",
        "protocol_id": runner.PROTOCOL_ID,
        "evaluation": {
            "adapter_path": str(runner.SCRIPT),
            "adapter_sha256": runtime["adapter_runner_sha256"],
            "core_path": str(runner.base.CORE_SCRIPT),
            "core_sha256": runtime["collision_core_sha256"],
            "child_python": runtime["python_executable"],
            "runtime_identity": runtime,
            "workers": 2,
            "audit_workers": 1,
            "child_timeout_seconds": 30.0,
        },
        "items": [item],
    }
    manifest["manifest_content_sha256"] = runner._manifest_self_hash(manifest)
    (tmp_path / "frozen_manifest.json").write_text(json.dumps(manifest))

    record = runner.base._load_core().failure_record(item, "fixture_failure")
    record["runtime_identity"] = runtime
    record["runner_sha256"] = runtime["adapter_runner_sha256"]
    record["collision_core_sha256"] = runtime["collision_core_sha256"]
    changed = copy.deepcopy(record)
    changed["runtime_identity"]["pybullet_module_sha256"] = "0" * 64
    (tmp_path / "asset_records.jsonl").write_text(json.dumps(changed) + "\n")

    matching = runner.base.parse_args(
        [
            "--limit",
            "1",
            "--workers",
            "2",
            "--audit-workers",
            "1",
            "--child-timeout-seconds",
            "30",
            "--resume",
            "--output",
            str(tmp_path),
        ]
    )

    def scheduler_must_not_start(_args: object) -> Path:
        raise AssertionError("scheduler entered before resume provenance validation")

    monkeypatch.setattr(runner, "_base_run", scheduler_must_not_start)
    with pytest.raises(ValueError, match="runtime binding"):
        runner.run(matching)

    control_drift = runner.base.parse_args(
        [
            "--limit",
            "1",
            "--workers",
            "1",
            "--audit-workers",
            "1",
            "--child-timeout-seconds",
            "30",
            "--resume",
            "--output",
            str(tmp_path),
        ]
    )
    with pytest.raises(ValueError, match="resume.*workers"):
        runner.validate_args(control_drift)
