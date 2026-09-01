from __future__ import annotations

import errno
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from typing import Any

import pytest


REPO = Path(__file__).resolve().parents[2]
EXP = REPO / "exp"
RUNNER = EXP / "scripts/run_table3_infinite_mobility.py"
TABLE2_METRICS = (
    "parse_rate",
    "resource_resolution",
    "finite_fields",
    "valid_tree",
    "valid_joint_spec",
    "collision_coverage",
    "inertial_coverage",
    "inertia_validity",
    "strict_urdf_pass",
)
TABLE2_RESULT_SOURCE_FIELDS = (
    "selection_index",
    "selection_rank",
    "selection_hash",
    "cohort_record_sha256",
    "asset_id",
    "factory",
    "raw_category",
    "seed",
    "original_status",
    "recovery_used",
    "recovery_provenance",
    "source",
    "declared_joint_count_hint",
    "baseline_package_sha256",
    "package",
    "expected_package_path",
    "primary_urdf_relative_path",
    "primary_urdf_sha256",
    "model_urdf_sha256",
)
EXPECTED_TABLE2_SPAWN_EAGAIN_RETRY_POLICY = {
    "retryable_exception": "BlockingIOError",
    "retryable_errno": errno.EAGAIN,
    "scope": "subprocess.Popen only",
    "popen_action": "retry the same spawn in place while preserving active children",
    "total_backoff_wait_seconds": 1800.0,
    "initial_backoff_seconds": 1.0,
    "maximum_backoff_seconds": 30.0,
    "backoff_multiplier": 2.0,
    "exhaustion": "raise nonzero with running checkpoint preserved for --resume",
    "all_other_spawn_step_failures": "fail-closed metric record",
}
EXPECTED_TABLE3_SPAWN_EAGAIN_RETRY_POLICY = {
    "retryable_exception": "BlockingIOError",
    "retryable_errno": errno.EAGAIN,
    "scope": "one shared cumulative backoff budget per run across all workers",
    "popen_action": "retry the same job/run_token spawn in place",
    "total_backoff_wait_seconds": 1800.0,
    "initial_backoff_seconds": 1.0,
    "maximum_backoff_seconds": 30.0,
    "backoff_multiplier": 2.0,
    "exhaustion": "raise nonzero with running checkpoint preserved for --resume",
    "other_spawn_failures": "fail-closed asset and declared-joint record",
}


def load_runner():
    assert RUNNER.is_file(), "Infinite Mobility Table 3 adapter has not been implemented"
    name = "run_table3_infinite_mobility_test"
    spec = importlib.util.spec_from_file_location(name, RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_sha(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return _canonical_sha(payload)


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _package_binding(package: Path) -> dict[str, Any]:
    files = []
    for path in sorted(item for item in package.rglob("*") if item.is_file()):
        relative = path.relative_to(package).as_posix()
        files.append({"path": relative, "bytes": path.stat().st_size, "sha256": _sha(path)})
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": _canonical_sha(files),
    }


def _write_artifact_manifest(root: Path, names: list[str]) -> Path:
    artifact = {
        "schema_version": 1,
        "files": {
            name: {"bytes": (root / name).stat().st_size, "sha256": _sha(root / name)}
            for name in names
        },
    }
    path = root / "artifact_manifest.json"
    _write_json(path, artifact)
    return path


def _urdf(asset: str) -> str:
    return f"""<robot name="{asset}">
  <link name="base"><visual><geometry><box size="2 1 1"/></geometry></visual></link>
  <link name="door"><visual><geometry><box size="1 0.2 1"/></geometry></visual></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _fixture_inputs(tmp_path: Path) -> dict[str, Any]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# Frozen Table 3 protocol\n\nK = 21.\n", encoding="utf-8")
    cohort_preparer = tmp_path / "prepare_cohort.py"
    cohort_preparer.write_text("# frozen cohort preparer\n", encoding="utf-8")
    table2_evaluator = tmp_path / "table2_evaluator.py"
    table2_evaluator.write_text("# frozen table2 evaluator\n", encoding="utf-8")
    table2_adapter = tmp_path / "table2_adapter.py"
    table2_adapter.write_text("# frozen table2 adapter\n", encoding="utf-8")
    primary_source = tmp_path / "primary_manifest.json"
    recovery_source = tmp_path / "recovery_manifest.json"
    _write_json(primary_source, {"source": "primary"})
    _write_json(recovery_source, {"source": "recovery"})

    packages = []
    for seed in range(2):
        package = tmp_path / "packages" / f"seed_{seed:03d}"
        package.mkdir(parents=True)
        urdf = package / "scene.urdf"
        urdf.write_text(_urdf(f"asset_{seed}"), encoding="utf-8")
        packages.append((package, urdf, _package_binding(package)))

    source_bindings = [
        {"label": "primary_manifest", "path": str(primary_source), "sha256": _sha(primary_source)},
        {"label": "recovery_manifest", "path": str(recovery_source), "sha256": _sha(recovery_source)},
    ]
    cohort_assets = []
    for index, (package, urdf, binding) in enumerate(packages):
        recovered = index == 1
        cohort_assets.append(
            {
                "selection_index": index + 1,
                "selection_rank": index + 1,
                "asset_id": f"FactoryA/seed_{index:03d}",
                "factory": "FactoryA",
                "raw_category": "FactoryA",
                "seed": index,
                "original_status": "TIMEOUT" if recovered else "PASS",
                "recovery_used": recovered,
                "recovery_provenance": (
                    {
                        "original_record_sha256": "a" * 64,
                        "recovery_record_sha256": "b" * 64,
                    }
                    if recovered
                    else None
                ),
                "source": "recovery" if recovered else "primary",
                "package_path": str(package),
                "urdf_relpath": "scene.urdf",
                "primary_urdf_sha256": _sha(urdf),
                "declared_joint_count_hint": 1,
                "baseline_package_sha256": "f" * 64,
                "package_binding": binding,
            }
        )

    cohort_dir = tmp_path / "cohort"
    cohort_dir.mkdir()
    cohort_protocol = cohort_dir / "cohort_protocol_snapshot.json"
    _write_json(
        cohort_protocol,
        {"protocol_path": str(protocol), "protocol_sha256": _sha(protocol)},
    )
    cohort_manifest = {
        "schema_version": 1,
        "dataset": "Infinite Mobility",
        "release_status": "SUPPLEMENTARY_FULL_GENERATED_COHORT",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "N_release": 2,
        "N_eval": 2,
        "factory_order": ["FactoryA"],
        "seeds": [0, 1],
        "source_bindings": source_bindings,
        "source": {"bindings": source_bindings},
        "evaluation": {
            "freezer_path": str(cohort_preparer),
            "freezer_sha256": _sha(cohort_preparer),
            "preparer_path": str(cohort_preparer),
            "preparer_sha256": _sha(cohort_preparer),
            "protocol_path": str(protocol),
            "protocol_sha256": _sha(protocol),
        },
        "assets": cohort_assets,
    }
    cohort_manifest["manifest_content_sha256"] = _manifest_self_hash(cohort_manifest)
    cohort_path = cohort_dir / "manifest.json"
    _write_json(cohort_path, cohort_manifest)
    cohort_artifacts = _write_artifact_manifest(
        cohort_dir, ["manifest.json", "cohort_protocol_snapshot.json"]
    )

    table2_dir = tmp_path / "table2"
    table2_dir.mkdir()
    table2_protocol = table2_dir / "protocol_snapshot.md"
    table2_protocol.write_bytes(protocol.read_bytes())
    environment = {
        "python": "fixture",
        "evaluator": "fixture",
        "spawn_eagain_retry_policy": json.loads(
            json.dumps(EXPECTED_TABLE2_SPAWN_EAGAIN_RETRY_POLICY)
        ),
    }
    table2_records = []
    for row in cohort_assets:
        table2_records.append(
            {
                "selection_index": row["selection_index"],
                "selection_rank": row["selection_index"],
                "selection_hash": _canonical_sha([row["selection_index"] - 1, row["asset_id"]]),
                "cohort_record_sha256": _canonical_sha(row),
                "asset_id": row["asset_id"],
                "factory": row["factory"],
                "raw_category": row["raw_category"],
                "seed": row["seed"],
                "original_status": row["original_status"],
                "recovery_used": row["recovery_used"],
                "recovery_provenance": row["recovery_provenance"],
                "source": row["source"],
                "baseline_package_sha256": row["baseline_package_sha256"],
                "package": row["package_path"],
                "expected_package_path": row["package_path"],
                "primary_urdf_relative_path": row["urdf_relpath"],
                "primary_urdf_sha256": row["primary_urdf_sha256"],
                "model_urdf_sha256": row["primary_urdf_sha256"],
                "declared_joint_count_hint": row["declared_joint_count_hint"],
                "package_binding": row["package_binding"],
            }
        )
    table2_config = {"protocol_id": "fixture-table2", "workers": 1}
    table2_manifest = {
        "schema_version": 1,
        "dataset": "Infinite Mobility",
        "mode": "smoke",
        "classification": "NON_FORMAL_SMOKE",
        "source": {
            "cohort_manifest_path": str(cohort_path),
            "cohort_manifest_sha256": _sha(cohort_path),
            "cohort_manifest_content_sha256": cohort_manifest["manifest_content_sha256"],
            "cohort_artifact_manifest_sha256": _sha(cohort_artifacts),
            "source_bindings": source_bindings,
        },
        "selection": {
            "n_eval": 2,
            "selected_asset_ids_sha256": _canonical_sha(
                [row["asset_id"] for row in table2_records]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "evaluator_path": str(table2_evaluator),
            "evaluator_sha256": _sha(table2_evaluator),
            "shared_core_path": str(table2_evaluator),
            "shared_core_sha256": _sha(table2_evaluator),
            "adapter_path": str(table2_adapter),
            "adapter_sha256": _sha(table2_adapter),
            "protocol_source_path": str(protocol),
            "protocol_source_sha256_at_freeze": _sha(protocol),
            "protocol_path": str(table2_protocol),
            "protocol_sha256": _sha(protocol),
            "protocol_snapshot_sha256": _sha(table2_protocol),
            "config": table2_config,
            "config_sha256": _canonical_sha(table2_config),
            "environment": environment,
            "environment_sha256": _canonical_sha(environment),
        },
        "records": table2_records,
    }
    table2_manifest["manifest_content_sha256"] = _manifest_self_hash(table2_manifest)
    table2_path = table2_dir / "manifest.json"
    _write_json(table2_path, table2_manifest)
    table2_artifacts = _write_artifact_manifest(
        table2_dir, ["manifest.json", "protocol_snapshot.md"]
    )
    return {
        "protocol": protocol,
        "cohort_path": cohort_path,
        "cohort_manifest": cohort_manifest,
        "cohort_artifacts": cohort_artifacts,
        "table2_path": table2_path,
        "table2_manifest": table2_manifest,
        "table2_artifacts": table2_artifacts,
        "packages": packages,
    }


def _complete_table2_publication(fixture: dict[str, Any]) -> None:
    table2_path = fixture["table2_path"]
    root = table2_path.parent
    manifest = fixture["table2_manifest"]
    manifest["mode"] = "formal"
    manifest["classification"] = "FORMAL"
    manifest["source"].update(
        {
            "N_release": len(manifest["records"]),
            "original_status_counts": {"PASS": 1, "TIMEOUT": 1},
            "recovery_overlay_count": 1,
        }
    )
    manifest["evaluation"].update(
        {
            "workers": 8,
            "asset_timeout_seconds": 300.0,
            "standard_parser": "urdfpy",
            "standard_parser_version": "0.0.22",
            "adapter_config": {
                "workers": 8,
                "asset_timeout_seconds": 300.0,
                "standard_parser": True,
                "spawn_eagain_retry_policy": json.loads(
                    json.dumps(EXPECTED_TABLE2_SPAWN_EAGAIN_RETRY_POLICY)
                ),
            },
        }
    )
    manifest["evaluation"]["adapter_config_sha256"] = _canonical_sha(
        manifest["evaluation"]["adapter_config"]
    )
    _write_json(root / "environment.json", manifest["evaluation"]["environment"])
    manifest["evaluation"]["environment_file_sha256"] = _sha(root / "environment.json")
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    _write_json(table2_path, manifest)

    static_fields = {
        field: manifest["evaluation"][field]
        for field in (
            "evaluator_path",
            "evaluator_sha256",
            "protocol_path",
            "protocol_sha256",
            "config",
            "config_sha256",
            "environment",
            "environment_sha256",
        )
    }
    results = []
    for order, source in enumerate(manifest["records"], 1):
        token = f"{order:032x}"
        runtime_binding = {"run_token": token, **static_fields}
        results.append(
            {
                **{field: source[field] for field in TABLE2_RESULT_SOURCE_FIELDS},
                "status": "completed",
                "metrics": {name: {"pass": True} for name in TABLE2_METRICS},
                "strict_urdf_pass": True,
                "package_content_manifest_sha256": source["package_binding"][
                    "content_manifest_sha256"
                ],
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "result_origin": "child_attested",
                "job_runtime_binding": runtime_binding,
                "worker_runtime_binding": runtime_binding,
                "completion_order": order,
            }
        )
    records_path = root / "records.jsonl"
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in results),
        encoding="utf-8",
    )
    _write_json(
        root / "summary.json",
        {
            "status": "completed",
            "mode": "formal",
            "classification": "FORMAL",
            "dataset": "Infinite Mobility",
            "n_eval": len(results),
            "records_present": len(results),
            "records_missing_counted_as_failures": 0,
            "status_counts": {"completed": len(results)},
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "source_provenance": {"original_pass": 1, "recovery_overlay": 1},
        },
    )
    (root / "report.md").write_text("# Complete Table 2 fixture\n", encoding="utf-8")
    (root / ".run.lock").write_text("pid=fixture\n", encoding="utf-8")
    _write_json(
        root / "checkpoint.json",
        {
            "state": "complete",
            "completed": len(results),
            "remaining": 0,
            "n_eval": len(results),
            "completion_order": len(results),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "manifest_file_sha256": _sha(table2_path),
            "records_sha256": _sha(records_path),
        },
    )
    fixture["table2_artifacts"] = _write_artifact_manifest(
        root,
        [
            "manifest.json",
            "protocol_snapshot.md",
            "environment.json",
            "records.jsonl",
            "summary.json",
            "report.md",
            "checkpoint.json",
        ],
    )
    fixture["table2_results"] = results


def _rewrite_table2_results(
    fixture: dict[str, Any], results: list[dict[str, Any]]
) -> None:
    root = fixture["table2_path"].parent
    records_path = root / "records.jsonl"
    records_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in results
        ),
        encoding="utf-8",
    )
    checkpoint_path = root / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    checkpoint["records_sha256"] = _sha(records_path)
    _write_json(checkpoint_path, checkpoint)
    status_counts: dict[str, int] = {}
    for row in results:
        status = str(row["status"])
        status_counts[status] = status_counts.get(status, 0) + 1
    summary_path = root / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["status_counts"] = dict(sorted(status_counts.items()))
    _write_json(summary_path, summary)
    fixture["table2_artifacts"] = _write_artifact_manifest(
        root,
        [
            "manifest.json",
            "protocol_snapshot.md",
            "environment.json",
            "records.jsonl",
            "summary.json",
            "report.md",
            "checkpoint.json",
        ],
    )


def _smoke_args(runner: Any, fixture: dict[str, Any], output: Path) -> Any:
    return runner.parse_args(
        [
            "--mode",
            "smoke",
            "--cohort-manifest",
            str(fixture["cohort_path"]),
            "--table2-manifest",
            str(fixture["table2_path"]),
            "--protocol-path",
            str(fixture["protocol"]),
            "--output",
            str(output),
            "--workers",
            "2",
            "--asset-timeout-seconds",
            "30",
        ]
    )


def _internal_job_from_popen(command: list[str]) -> dict[str, Any] | None:
    if "--internal-job" not in command:
        return None
    job_path = Path(command[command.index("--internal-job") + 1])
    return json.loads(job_path.read_text(encoding="utf-8"))


def _attested_job(
    runner: Any,
    row: dict[str, Any],
    tmp_path: Path,
    protocol: Path,
    manifest_hash: str = "c" * 64,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    job = runner.make_job(
        row, manifest_content_sha256=manifest_hash, samples=21, mode="smoke"
    )
    config = runner.job_evaluation_config(samples=21)
    environment = runner._environment()
    child_runtime = runner.child_runtime_fingerprint()
    manifest = {
        "manifest_content_sha256": manifest_hash,
        "evaluation": {
            "adapter_path": str(RUNNER),
            "adapter_sha256": _sha(RUNNER),
            "core_evaluator_path": str(runner.CORE_PATH),
            "core_evaluator_sha256": _sha(runner.CORE_PATH),
            "protocol_path": str(protocol),
            "protocol_sha256": _sha(protocol),
            "config": config,
            "config_sha256": _canonical_sha(config),
            "environment": environment,
            "environment_sha256": _canonical_sha(environment),
            "child_runtime": child_runtime,
            "child_runtime_sha256": _canonical_sha(child_runtime),
        },
    }
    manifest_path = tmp_path / f"run-manifest-{row['selection_index']}.json"
    _write_json(manifest_path, manifest)
    return runner.attach_runtime_binding(job, manifest, manifest_path), manifest, manifest_path


def test_load_inputs_reuses_exact_table2_order_and_package_bindings(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)

    loaded = runner.load_inputs(
        fixture["cohort_path"], fixture["table2_path"], formal=False
    )

    assert [row["asset_id"] for row in loaded["assets"]] == [
        "FactoryA/seed_000",
        "FactoryA/seed_001",
    ]
    assert [row["selection_index"] for row in loaded["assets"]] == [1, 2]
    assert loaded["assets"][1]["original_status"] == "TIMEOUT"
    assert loaded["assets"][1]["recovery_used"] is True
    assert loaded["assets"][1]["recovery_provenance"]["recovery_record_sha256"] == "b" * 64
    assert loaded["assets"][0]["package_binding"] == fixture["packages"][0][2]
    assert loaded["table2_manifest_file_sha256"] == _sha(fixture["table2_path"])
    assert loaded["cohort_artifact_manifest_sha256"] == _sha(fixture["cohort_artifacts"])


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("cohort_self_hash", "cohort manifest self-hash"),
        ("table2_order", "Table 2 row identity/order"),
        ("package_binding", "package binding"),
        ("row_provenance", "Table 2 row provenance"),
        ("table2_evaluator", "Table 2 evaluator hash"),
        ("table2_adapter", "Table 2 adapter hash"),
        ("table2_protocol_snapshot", "protocol snapshot"),
    ],
)
def test_load_inputs_rejects_broken_provenance(
    tmp_path: Path, mutation: str, message: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    cohort = fixture["cohort_manifest"]
    table2 = fixture["table2_manifest"]

    if mutation == "cohort_self_hash":
        cohort["assets"][0]["seed"] = 9
        _write_json(fixture["cohort_path"], cohort)
    elif mutation == "table2_order":
        table2["records"][0]["asset_id"] = "FactoryA/seed_001"
        table2["selection"]["selected_asset_ids_sha256"] = _canonical_sha(
            [row["asset_id"] for row in table2["records"]]
        )
        table2["manifest_content_sha256"] = _manifest_self_hash(table2)
        _write_json(fixture["table2_path"], table2)
        _write_artifact_manifest(
            fixture["table2_path"].parent, ["manifest.json", "protocol_snapshot.md"]
        )
    elif mutation == "package_binding":
        table2["records"][0]["package_binding"]["total_bytes"] += 1
        table2["manifest_content_sha256"] = _manifest_self_hash(table2)
        _write_json(fixture["table2_path"], table2)
        _write_artifact_manifest(
            fixture["table2_path"].parent, ["manifest.json", "protocol_snapshot.md"]
        )
    elif mutation == "row_provenance":
        table2["records"][0]["declared_joint_count_hint"] = 2
        table2["manifest_content_sha256"] = _manifest_self_hash(table2)
        _write_json(fixture["table2_path"], table2)
        _write_artifact_manifest(
            fixture["table2_path"].parent, ["manifest.json", "protocol_snapshot.md"]
        )
    elif mutation == "table2_evaluator":
        Path(table2["evaluation"]["evaluator_path"]).write_text(
            "# drifted evaluator\n", encoding="utf-8"
        )
    elif mutation == "table2_adapter":
        Path(table2["evaluation"]["adapter_path"]).write_text(
            "# drifted adapter\n", encoding="utf-8"
        )
    elif mutation == "table2_protocol_snapshot":
        (fixture["table2_path"].parent / "protocol_snapshot.md").write_text(
            "drift\n", encoding="utf-8"
        )

    with pytest.raises((ValueError, RuntimeError), match=message):
        runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)


def test_formal_validator_requires_exact_matrix_provenance_and_joint_denominators() -> None:
    runner = load_runner()
    frozen = json.loads(
        (EXP / "runtime/infinite_mobility_urdf_table123_cohort/manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assets = frozen["assets"]

    runner.validate_formal_assets(assets)
    assert sum(row["declared_joint_count_hint"] for row in assets) == 4723
    assert sum(row["declared_joint_count_hint"] == 0 for row in assets) == 55

    assets[0]["selection_index"] = 2
    with pytest.raises(ValueError, match="exact 20 x 36 order"):
        runner.validate_formal_assets(assets)
    assets[0]["selection_index"] = 1
    recovered = next(row for row in assets if row["recovery_used"])
    original_recovery = dict(recovered["recovery_provenance"])
    recovered["recovery_provenance"] = None
    with pytest.raises(ValueError, match="recovery provenance"):
        runner.validate_formal_assets(assets)
    recovered["recovery_provenance"] = original_recovery
    assets[0]["declared_joint_count_hint"] += 1
    with pytest.raises(ValueError, match="4,723|joint-count"):
        runner.validate_formal_assets(assets)


def test_formal_contract_freezes_21_states_and_low_medium_python(tmp_path: Path) -> None:
    runner = load_runner()
    args = runner.parse_args([])
    runner.validate_contract(
        args,
        python_executable=runner.FORMAL_PYTHON,
        python_prefix=runner.FORMAL_PYTHON.parent.parent,
        runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
    )
    assert args.mode == "formal"
    assert args.samples == 21
    assert args.limit is None

    args.samples = 20
    with pytest.raises(ValueError, match="samples=21"):
        runner.validate_contract(
            args,
            python_executable=runner.FORMAL_PYTHON,
            python_prefix=runner.FORMAL_PYTHON.parent.parent,
            runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
        )
    args.samples = 21
    with pytest.raises(ValueError, match="low_medium"):
        runner.validate_contract(
            args,
            python_executable=Path("/usr/bin/python3"),
            python_prefix=Path("/usr"),
            runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
        )
    args.workers = 999
    with pytest.raises(ValueError, match="workers=4"):
        runner.validate_contract(
            args,
            python_executable=runner.FORMAL_PYTHON,
            python_prefix=runner.FORMAL_PYTHON.parent.parent,
            runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
        )
    args.workers = 4
    args.asset_timeout_seconds = 0.001
    with pytest.raises(ValueError, match="timeout=120"):
        runner.validate_contract(
            args,
            python_executable=runner.FORMAL_PYTHON,
            python_prefix=runner.FORMAL_PYTHON.parent.parent,
            runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
        )
    args.asset_timeout_seconds = 120
    with pytest.raises(ValueError, match="fingerprint"):
        runner.validate_contract(
            args,
            python_executable=runner.FORMAL_PYTHON,
            python_prefix=runner.FORMAL_PYTHON.parent.parent,
            runtime_fingerprint={**runner.FORMAL_RUNTIME_FINGERPRINT, "numpy": "drift"},
        )
    args.cohort_manifest = tmp_path / "cohort.json"
    with pytest.raises(ValueError, match="canonical cohort"):
        runner.validate_contract(
            args,
            python_executable=runner.FORMAL_PYTHON,
            python_prefix=runner.FORMAL_PYTHON.parent.parent,
            runtime_fingerprint=runner.FORMAL_RUNTIME_FINGERPRINT,
        )


@pytest.mark.parametrize(
    "missing",
    ["summary.json", "records.jsonl", "checkpoint.json", "environment.json", "report.md"],
)
def test_formal_table2_requires_exact_completed_artifact_closure(
    tmp_path: Path, monkeypatch: Any, missing: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    _complete_table2_publication(fixture)
    manifest = fixture["table2_manifest"]
    monkeypatch.setattr(runner, "FORMAL_N", 2)
    monkeypatch.setattr(runner, "FORMAL_TABLE2_MANIFEST_FILE_SHA256", _sha(fixture["table2_path"]))
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE2_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )
    artifact = json.loads(fixture["table2_artifacts"].read_text(encoding="utf-8"))
    artifact["files"].pop(missing)
    (fixture["table2_path"].parent / missing).unlink()
    _write_json(fixture["table2_artifacts"], artifact)

    with pytest.raises(ValueError, match="artifact closure"):
        runner.verify_formal_table2_publication(
            fixture["table2_path"], manifest, manifest["records"]
        )


def test_formal_table2_completed_records_are_deeply_bound(
    tmp_path: Path, monkeypatch: Any
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    _complete_table2_publication(fixture)
    manifest = fixture["table2_manifest"]
    monkeypatch.setattr(runner, "FORMAL_N", 2)
    monkeypatch.setattr(
        runner, "FORMAL_TABLE2_MANIFEST_FILE_SHA256", _sha(fixture["table2_path"])
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE2_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )

    receipt = runner.verify_formal_table2_publication(
        fixture["table2_path"], manifest, manifest["records"]
    )
    assert receipt["status_counts"] == {"completed": 2}
    assert all("package_binding" not in row for row in fixture["table2_results"])
    assert all(
        row["expected_package_path"] == manifest["records"][index]["package"]
        for index, row in enumerate(fixture["table2_results"])
    )

    records_path = fixture["table2_path"].parent / "records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text().splitlines()]
    rows[0]["status"] = "forged"
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows),
        encoding="utf-8",
    )
    checkpoint_path = fixture["table2_path"].parent / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    checkpoint["records_sha256"] = _sha(records_path)
    _write_json(checkpoint_path, checkpoint)
    fixture["table2_artifacts"] = _write_artifact_manifest(
        fixture["table2_path"].parent,
        [
            "manifest.json",
            "protocol_snapshot.md",
            "environment.json",
            "records.jsonl",
            "summary.json",
            "report.md",
            "checkpoint.json",
        ],
    )
    with pytest.raises(ValueError, match="status"):
        runner.verify_formal_table2_publication(
            fixture["table2_path"], manifest, manifest["records"]
        )


@pytest.mark.parametrize("binding_name", ("adapter_config", "environment"))
def test_formal_table2_rejects_spawn_eagain_retry_policy_drift(
    tmp_path: Path, monkeypatch: Any, binding_name: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    _complete_table2_publication(fixture)
    manifest = fixture["table2_manifest"]
    evaluation = manifest["evaluation"]
    policy = evaluation[binding_name]["spawn_eagain_retry_policy"]
    policy["maximum_backoff_seconds"] = 29.0
    evaluation[f"{binding_name}_sha256"] = _canonical_sha(
        evaluation[binding_name]
    )
    results = json.loads(json.dumps(fixture["table2_results"]))
    if binding_name == "environment":
        environment_path = fixture["table2_path"].parent / "environment.json"
        _write_json(environment_path, evaluation["environment"])
        evaluation["environment_file_sha256"] = _sha(environment_path)
        for result in results:
            for runtime_field in ("job_runtime_binding", "worker_runtime_binding"):
                result[runtime_field]["environment"] = json.loads(
                    json.dumps(evaluation["environment"])
                )
                result[runtime_field]["environment_sha256"] = evaluation[
                    "environment_sha256"
                ]
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    for result in results:
        result["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    _write_json(fixture["table2_path"], manifest)
    _rewrite_table2_results(fixture, results)
    summary_path = fixture["table2_path"].parent / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    _write_json(summary_path, summary)
    checkpoint_path = fixture["table2_path"].parent / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    checkpoint["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    checkpoint["manifest_file_sha256"] = _sha(fixture["table2_path"])
    _write_json(checkpoint_path, checkpoint)
    fixture["table2_artifacts"] = _write_artifact_manifest(
        fixture["table2_path"].parent,
        [
            "manifest.json",
            "protocol_snapshot.md",
            "environment.json",
            "records.jsonl",
            "summary.json",
            "report.md",
            "checkpoint.json",
        ],
    )
    monkeypatch.setattr(runner, "FORMAL_N", 2)
    monkeypatch.setattr(
        runner, "FORMAL_TABLE2_MANIFEST_FILE_SHA256", _sha(fixture["table2_path"])
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE2_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )

    with pytest.raises(ValueError, match="spawn EAGAIN retry policy"):
        runner.verify_formal_table2_publication(
            fixture["table2_path"], manifest, manifest["records"]
        )


@pytest.mark.parametrize(
    "mutation",
    ("missing_worker_attestation", "parent_completed", "worker_binding_drift"),
)
def test_formal_table2_rejects_unattested_completed_records(
    tmp_path: Path, monkeypatch: Any, mutation: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    _complete_table2_publication(fixture)
    manifest = fixture["table2_manifest"]
    monkeypatch.setattr(runner, "FORMAL_N", 2)
    monkeypatch.setattr(
        runner, "FORMAL_TABLE2_MANIFEST_FILE_SHA256", _sha(fixture["table2_path"])
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE2_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )
    results = json.loads(json.dumps(fixture["table2_results"]))
    if mutation == "missing_worker_attestation":
        results[0].pop("worker_runtime_binding")
    elif mutation == "parent_completed":
        results[0]["result_origin"] = "parent_synthesized"
        results[0].pop("worker_runtime_binding")
    else:
        results[0]["worker_runtime_binding"]["run_token"] = "f" * 32
    _rewrite_table2_results(fixture, results)

    with pytest.raises(ValueError, match="attestation|runtime|origin|parent"):
        runner.verify_formal_table2_publication(
            fixture["table2_path"], manifest, manifest["records"]
        )


def test_formal_table2_parent_failure_must_be_fully_fail_closed(
    tmp_path: Path, monkeypatch: Any
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    _complete_table2_publication(fixture)
    manifest = fixture["table2_manifest"]
    monkeypatch.setattr(runner, "FORMAL_N", 2)
    monkeypatch.setattr(
        runner, "FORMAL_TABLE2_MANIFEST_FILE_SHA256", _sha(fixture["table2_path"])
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE2_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )
    results = json.loads(json.dumps(fixture["table2_results"]))
    results[0]["result_origin"] = "parent_synthesized"
    results[0].pop("worker_runtime_binding")
    results[0]["status"] = "error"
    results[0]["metrics"]["parse_rate"]["pass"] = True
    results[0]["metrics"]["valid_tree"]["pass"] = False
    results[0]["strict_urdf_pass"] = False
    results[0]["metrics"]["strict_urdf_pass"]["pass"] = False
    _rewrite_table2_results(fixture, results)

    with pytest.raises(ValueError, match="fail.closed|parent"):
        runner.verify_formal_table2_publication(
            fixture["table2_path"], manifest, manifest["records"]
        )


@pytest.mark.parametrize("binding_name", ("config", "environment"))
def test_live_runtime_rejects_spawn_eagain_retry_policy_drift(
    tmp_path: Path, binding_name: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    args = _smoke_args(runner, fixture, tmp_path / "unused-output")
    loaded = runner.load_inputs(
        fixture["cohort_path"], fixture["table2_path"], formal=False
    )
    manifest = runner.build_manifest(
        args, loaded, fixture["protocol"].read_bytes(), runner._environment()
    )
    evaluation = manifest["evaluation"]
    evaluation[binding_name]["spawn_eagain_retry_policy"] = json.loads(
        json.dumps(EXPECTED_TABLE3_SPAWN_EAGAIN_RETRY_POLICY)
    )
    evaluation[binding_name]["spawn_eagain_retry_policy"][
        "maximum_backoff_seconds"
    ] = 29.0
    evaluation[f"{binding_name}_sha256"] = _canonical_sha(
        evaluation[binding_name]
    )
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    manifest_path = tmp_path / f"policy-drift-{binding_name}.json"
    _write_json(manifest_path, manifest)

    with pytest.raises(RuntimeError, match="spawn EAGAIN retry policy"):
        runner.verify_live_runtime_binding(manifest, manifest_path)


def test_fresh_child_runs_shared_core_with_21_states(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    loaded = runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)
    job, _manifest, _manifest_path = _attested_job(
        runner, loaded["assets"][0], tmp_path, fixture["protocol"]
    )

    records = runner.execute_jobs(
        [job], scratch_root=tmp_path / "scratch", timeout_seconds=30, max_workers=1
    )

    record = records[0]
    assert record["status"] == "completed"
    assert record["result_origin"] == "child_attested"
    assert record["worker_evidence"]["fresh_interpreter"] is True
    assert record["worker_evidence"]["child_pid"] != os.getpid()
    assert record["worker_evidence"]["package_binding_before"] is True
    assert record["worker_evidence"]["package_binding_after"] is True
    assert record["worker_evidence"]["runtime_binding_match"] is True
    assert record["run_token"] == job["run_token"]
    assert record["declared_joint_count"] == 1
    assert len(record["joints"]) == 1
    assert record["joints"][0]["sample_count_expected"] == 21
    assert record["joints"][0]["sample_count_executed"] == 21

    for mutation in ("status", "joint", "run_token", "package_attestation"):
        forged = json.loads(json.dumps(record))
        if mutation == "status":
            forged["status"] = "forged"
        elif mutation == "joint":
            forged["joints"][0] = {}
        elif mutation == "run_token":
            forged["run_token"] = "0" * 32
        else:
            forged["worker_evidence"]["package_binding_after"] = False
        with pytest.raises(ValueError, match="status|joint|token|attestation|binding"):
            runner.validate_record(job, forged, require_completion_order=False)


def test_child_fails_closed_on_evaluator_binding_drift(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    loaded = runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)
    job, manifest, manifest_path = _attested_job(
        runner, loaded["assets"][0], tmp_path, fixture["protocol"]
    )
    runner.verify_live_runtime_binding(manifest, manifest_path)
    job["runtime_binding"]["core_evaluator_sha256"] = "0" * 64

    record = runner.execute_jobs(
        [job], scratch_root=tmp_path / "scratch", timeout_seconds=30, max_workers=1
    )[0]

    assert record["status"] == "error"
    assert record["strict_kinematic_pass"] is False
    assert record["worker_evidence"]["runtime_binding_match"] is False
    manifest["evaluation"]["adapter_sha256"] = "0" * 64
    with pytest.raises(RuntimeError, match="live adapter"):
        runner.verify_live_runtime_binding(manifest, manifest_path)


def test_drift_timeout_and_child_error_fail_closed_with_joint_denominator(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    loaded = runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)
    drift_job, _manifest0, _manifest_path0 = _attested_job(
        runner, loaded["assets"][0], tmp_path, fixture["protocol"], "d" * 64
    )
    (fixture["packages"][0][0] / "late.txt").write_text("drift\n", encoding="utf-8")
    timeout_job, _manifest1, _manifest_path1 = _attested_job(
        runner, loaded["assets"][1], tmp_path, fixture["protocol"], "d" * 64
    )
    timeout_job["internal_test_action"] = "sleep"
    timeout_job["internal_test_seconds"] = 10
    error_job = dict(timeout_job)
    error_job["asset_key"] = "synthetic/error"
    error_job["asset_id"] = "synthetic/error"
    error_job["selection_index"] = 2
    error_job["run_token"] = "f" * 32
    error_job["runtime_binding"]["run_token"] = error_job["run_token"]
    error_job["internal_test_action"] = "raise"

    records = runner.execute_jobs(
        [drift_job, timeout_job, error_job],
        scratch_root=tmp_path / "scratch",
        timeout_seconds=0.5,
        max_workers=3,
    )

    assert [record["status"] for record in records] == ["error", "timeout", "error"]
    assert all(record["strict_kinematic_pass"] is False for record in records)
    assert all(record["declared_joint_count"] == 1 for record in records)
    assert all(len(record["joints"]) == 1 for record in records)
    timeout_termination = records[1]["worker_evidence"]["termination"]
    assert timeout_termination["reason"] == "asset_timeout"
    assert timeout_termination["term_sent"] is True
    assert timeout_termination["reaped"] is True
    summary = runner.CORE.aggregate_records(records, expected_n=3)
    assert summary["n_eval"] == 3
    assert summary["j_eval"] == 3
    assert summary["metrics"]["joint_level_pass"]["denominator"] == 3


def test_spawn_eagain_retries_same_job_without_metric_pollution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "spawn-eagain-recovers"
    args = _smoke_args(runner, fixture, output)
    args.limit = 1
    args.workers = 1
    original_popen = runner.subprocess.Popen
    observed_tokens: list[str] = []
    sleep_delays: list[float] = []
    failures_remaining = 3

    def flaky_popen(command: list[str], *popen_args: Any, **popen_kwargs: Any) -> Any:
        nonlocal failures_remaining
        job = _internal_job_from_popen(command)
        if job is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        observed_tokens.append(job["run_token"])
        if failures_remaining:
            failures_remaining -= 1
            raise BlockingIOError(errno.EAGAIN, "Resource temporarily unavailable")
        return original_popen(command, *popen_args, **popen_kwargs)

    monkeypatch.setattr(runner.subprocess, "Popen", flaky_popen)
    monkeypatch.setattr(
        runner,
        "_wait_spawn_retry",
        lambda _event, seconds: sleep_delays.append(seconds) or False,
        raising=False,
    )

    published = runner.run(args)

    manifest = json.loads((published / "manifest.json").read_text(encoding="utf-8"))
    policy = manifest["evaluation"]["config"]["spawn_eagain_retry_policy"]
    assert policy == EXPECTED_TABLE3_SPAWN_EAGAIN_RETRY_POLICY
    assert policy == manifest["evaluation"]["environment"][
        "spawn_eagain_retry_policy"
    ]
    assert sleep_delays == [1.0, 2.0, 4.0]
    assert len(observed_tokens) == 4
    assert set(observed_tokens) == {manifest["records"][0]["run_token"]}
    records = runner.load_jsonl(published / "asset_records.jsonl")
    assert len(records) == 1
    assert records[0]["status"] == "completed"
    assert records[0]["result_origin"] == "child_attested"
    assert records[0]["run_token"] == manifest["records"][0]["run_token"]
    assert "BlockingIOError" not in str(records[0].get("error"))


def test_spawn_eagain_exhaustion_preserves_running_checkpoint_for_resume(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "spawn-eagain-exhausted"
    args = _smoke_args(runner, fixture, output)
    args.limit = 1
    args.workers = 1
    original_popen = runner.subprocess.Popen
    first_run_tokens: list[str] = []
    sleep_delays: list[float] = []

    def always_eagain(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        job = _internal_job_from_popen(command)
        if job is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        first_run_tokens.append(job["run_token"])
        raise BlockingIOError(errno.EAGAIN, "Resource temporarily unavailable")

    monkeypatch.setattr(runner.subprocess, "Popen", always_eagain)
    monkeypatch.setattr(
        runner,
        "_wait_spawn_retry",
        lambda _event, seconds: sleep_delays.append(seconds) or False,
        raising=False,
    )

    with pytest.raises(RuntimeError, match="spawn EAGAIN retry budget exhausted"):
        runner.run(args)

    published = output.resolve()
    manifest = json.loads((published / "manifest.json").read_text(encoding="utf-8"))
    frozen_token = manifest["records"][0]["run_token"]
    assert first_run_tokens
    assert set(first_run_tokens) == {frozen_token}
    assert sum(sleep_delays) <= 1800.0
    assert max(sleep_delays) == 30.0
    assert runner.load_jsonl(published / "asset_records.jsonl") == []
    assert json.loads((published / "checkpoint.json").read_text(encoding="utf-8"))[
        "state"
    ] == "running"
    assert not (published / "artifact_manifest.json").exists()
    assert not (published / ".worker_scratch").exists()

    resumed_tokens: list[str] = []

    def capture_resume(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        job = _internal_job_from_popen(command)
        if job is not None:
            resumed_tokens.append(job["run_token"])
        return original_popen(command, *popen_args, **popen_kwargs)

    monkeypatch.setattr(runner.subprocess, "Popen", capture_resume)
    resume = _smoke_args(runner, fixture, output)
    resume.limit = 1
    resume.workers = 1
    resume.resume = True
    runner.run(resume)

    records = runner.load_jsonl(published / "asset_records.jsonl")
    assert resumed_tokens == [frozen_token]
    assert len(records) == 1
    assert records[0]["run_token"] == frozen_token
    assert records[0]["status"] == "completed"


def test_non_eagain_spawn_error_remains_fail_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "spawn-non-eagain"
    args = _smoke_args(runner, fixture, output)
    args.limit = 1
    args.workers = 1
    original_popen = runner.subprocess.Popen
    spawn_attempts = 0
    sleep_delays: list[float] = []

    def emfile_popen(command: list[str], *popen_args: Any, **popen_kwargs: Any) -> Any:
        nonlocal spawn_attempts
        if _internal_job_from_popen(command) is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        spawn_attempts += 1
        raise BlockingIOError(errno.EMFILE, "Too many open files")

    monkeypatch.setattr(runner.subprocess, "Popen", emfile_popen)
    monkeypatch.setattr(
        runner,
        "_wait_spawn_retry",
        lambda _event, seconds: sleep_delays.append(seconds) or False,
        raising=False,
    )

    published = runner.run(args)

    records = runner.load_jsonl(published / "asset_records.jsonl")
    assert spawn_attempts == 1
    assert sleep_delays == []
    assert len(records) == 1
    assert records[0]["status"] == "error"
    assert records[0]["result_origin"] == "parent_synthesized"
    assert records[0]["strict_kinematic_pass"] is False
    assert "Errno 24" in records[0]["error"]


@pytest.mark.parametrize(
    ("requires_kill", "communicate_error"),
    (
        (False, BlockingIOError(errno.EAGAIN, "post-spawn resource error")),
        (True, RuntimeError("post-spawn resource error")),
    ),
    ids=("eagain-term", "other-kill"),
)
def test_post_popen_exception_is_reaped_without_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    requires_kill: bool,
    communicate_error: Exception,
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "post-popen-eagain"
    args = _smoke_args(runner, fixture, output)
    args.limit = 1
    args.workers = 1
    original_popen = runner.subprocess.Popen
    spawn_attempts = 0
    sleep_delays: list[float] = []
    signals: list[int] = []
    wait_timeouts: list[float] = []

    class CommunicateEagainProcess:
        pid = os.getpid() + 1
        returncode: int | None = None

        def __init__(self) -> None:
            self.communicate_calls = 0
            self.reaped = False

        def communicate(self, timeout: float | None = None) -> tuple[bytes, bytes]:
            self.communicate_calls += 1
            raise communicate_error

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, timeout: float | None = None) -> int:
            assert timeout is not None
            wait_timeouts.append(timeout)
            if requires_kill and signals[-1] == runner.signal.SIGTERM:
                raise subprocess.TimeoutExpired("fake-worker", timeout)
            self.returncode = -signals[-1]
            self.reaped = True
            return self.returncode

    process = CommunicateEagainProcess()

    def post_spawn_eagain(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        nonlocal spawn_attempts
        if _internal_job_from_popen(command) is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        spawn_attempts += 1
        return process

    def record_killpg(pid: int, sent_signal: int) -> None:
        assert pid == process.pid
        signals.append(sent_signal)

    monkeypatch.setattr(runner.subprocess, "Popen", post_spawn_eagain)
    monkeypatch.setattr(runner.os, "killpg", record_killpg)
    monkeypatch.setattr(
        runner,
        "_wait_spawn_retry",
        lambda _event, seconds: sleep_delays.append(seconds) or False,
        raising=False,
    )

    published = runner.run(args)

    records = runner.load_jsonl(published / "asset_records.jsonl")
    assert spawn_attempts == 1
    assert sleep_delays == []
    assert len(records) == 1
    assert records[0]["status"] == "error"
    assert records[0]["result_origin"] == "parent_synthesized"
    assert "post-spawn resource error" in records[0]["error"]
    assert process.communicate_calls == 1
    assert process.reaped is True
    assert signals == (
        [runner.signal.SIGTERM, runner.signal.SIGKILL]
        if requires_kill
        else [runner.signal.SIGTERM]
    )
    assert wait_timeouts == ([2.0, 2.0] if requires_kill else [2.0])
    termination = records[0]["worker_evidence"]["termination"]
    assert termination["reason"] == "post_popen_exception"
    assert termination["process_group_id"] == process.pid
    assert termination["term_sent"] is True
    assert termination["kill_sent"] is requires_kill
    assert termination["reaped"] is True
    assert termination["returncode"] == process.returncode
    assert not (published / ".worker_scratch").exists()


def test_post_popen_unreapable_child_is_fatal_and_preserves_scratch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "post-popen-unreapable"
    args = _smoke_args(runner, fixture, output)
    args.limit = 1
    args.workers = 1
    original_popen = runner.subprocess.Popen
    spawn_attempts = 0
    sleep_delays: list[float] = []
    signals: list[int] = []
    wait_timeouts: list[float] = []

    class UnreapableProcess:
        pid = os.getpid() + 1
        returncode = None

        def __init__(self) -> None:
            self.communicate_calls = 0

        def communicate(self, timeout: float | None = None) -> tuple[bytes, bytes]:
            self.communicate_calls += 1
            raise BlockingIOError(errno.EAGAIN, "post-spawn resource error")

        def poll(self) -> None:
            return None

        def wait(self, timeout: float | None = None) -> int:
            assert timeout is not None
            wait_timeouts.append(timeout)
            raise subprocess.TimeoutExpired("fake-worker", timeout)

    process = UnreapableProcess()

    def post_spawn_eagain(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        nonlocal spawn_attempts
        if _internal_job_from_popen(command) is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        spawn_attempts += 1
        return process

    def record_killpg(pid: int, sent_signal: int) -> None:
        assert pid == process.pid
        signals.append(sent_signal)

    monkeypatch.setattr(runner.subprocess, "Popen", post_spawn_eagain)
    monkeypatch.setattr(runner.os, "killpg", record_killpg)
    monkeypatch.setattr(
        runner,
        "_wait_spawn_retry",
        lambda _event, seconds: sleep_delays.append(seconds) or False,
        raising=False,
    )

    with pytest.raises(RuntimeError, match="owned process lifecycle"):
        runner.run(args)

    published = output.resolve()
    assert spawn_attempts == 1
    assert sleep_delays == []
    assert process.communicate_calls == 1
    assert signals == [runner.signal.SIGTERM, runner.signal.SIGKILL]
    assert wait_timeouts == [2.0, 2.0]
    assert runner.load_jsonl(published / "asset_records.jsonl") == []
    checkpoint = json.loads(
        (published / "checkpoint.json").read_text(encoding="utf-8")
    )
    assert checkpoint["state"] == "running"
    assert not (published / "artifact_manifest.json").exists()
    scratch = published / ".worker_scratch"
    jobs = list(scratch.glob("job_*"))
    assert scratch.is_dir()
    assert len(jobs) == 1
    assert (jobs[0] / "job.json").is_file()
    failure = json.loads(
        (jobs[0] / "lifecycle_failure.json").read_text(encoding="utf-8")
    )
    assert failure["asset_key"] == "FactoryA/seed_000"
    assert failure["worker_evidence"]["termination"]["term_sent"] is True
    assert failure["worker_evidence"]["termination"]["kill_sent"] is True
    assert failure["worker_evidence"]["termination"]["reaped"] is False


def test_unreapable_child_stops_single_worker_queue_before_next_popen(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "single-worker-fatal-gate"
    args = _smoke_args(runner, fixture, output)
    args.workers = 1
    original_popen = runner.subprocess.Popen
    spawn_attempts = 0

    class UnreapableProcess:
        pid = os.getpid() + 100
        returncode = None

        def communicate(self, timeout: float | None = None) -> tuple[bytes, bytes]:
            raise BlockingIOError(errno.EAGAIN, "post-spawn resource error")

        def poll(self) -> None:
            return None

        def wait(self, timeout: float | None = None) -> int:
            assert timeout == 2.0
            raise subprocess.TimeoutExpired("fake-worker", timeout)

    def first_unreapable_then_spawn_error(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        nonlocal spawn_attempts
        if _internal_job_from_popen(command) is None:
            return original_popen(command, *popen_args, **popen_kwargs)
        spawn_attempts += 1
        if spawn_attempts == 1:
            return UnreapableProcess()
        raise BlockingIOError(errno.EMFILE, "queued job spawned after fatal")

    monkeypatch.setattr(runner.subprocess, "Popen", first_unreapable_then_spawn_error)
    monkeypatch.setattr(runner.os, "killpg", lambda _pid, _signal: None)

    with pytest.raises(RuntimeError, match="owned process lifecycle"):
        runner.run(args)

    assert spawn_attempts == 1
    assert runner.load_jsonl(output / "asset_records.jsonl") == []
    assert json.loads((output / "checkpoint.json").read_text(encoding="utf-8"))[
        "state"
    ] == "running"


def test_unreapable_child_stops_concurrent_queue_at_active_worker_count(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    loaded = runner.load_inputs(
        fixture["cohort_path"], fixture["table2_path"], formal=False
    )
    base_job, _manifest, _manifest_path = _attested_job(
        runner, loaded["assets"][0], tmp_path, fixture["protocol"]
    )
    jobs = []
    for index in range(8):
        job = json.loads(json.dumps(base_job))
        job["asset_key"] = f"synthetic/concurrent-{index}"
        job["asset_id"] = job["asset_key"]
        job["selection_index"] = index + 1
        job["run_token"] = f"{index + 1:032x}"
        job["runtime_binding"]["run_token"] = job["run_token"]
        jobs.append(job)

    max_workers = 3
    all_active = threading.Event()
    lifecycle_persisted = threading.Event()
    spawn_lock = threading.Lock()
    spawned_pids: list[int] = []
    original_preserve = runner._preserve_lifecycle_failure

    class CoordinatedProcess:
        def __init__(self, pid: int, fatal: bool) -> None:
            self.pid = pid
            self.fatal = fatal
            self.returncode: int | None = None

        def communicate(self, timeout: float | None = None) -> tuple[bytes, bytes]:
            gate = all_active if self.fatal else lifecycle_persisted
            if not gate.wait(timeout=5.0):
                raise RuntimeError("test coordination gate timed out")
            if self.fatal:
                raise BlockingIOError(errno.EAGAIN, "fatal post-spawn error")
            raise RuntimeError("active worker settled after lifecycle fatal")

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, timeout: float | None = None) -> int:
            assert timeout == 2.0
            if self.fatal:
                raise subprocess.TimeoutExpired("fake-worker", timeout)
            self.returncode = -runner.signal.SIGTERM
            return self.returncode

    def coordinated_popen(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        assert _internal_job_from_popen(command) is not None
        with spawn_lock:
            index = len(spawned_pids)
            pid = os.getpid() + 1000 + index
            spawned_pids.append(pid)
            if len(spawned_pids) == max_workers:
                all_active.set()
        return CoordinatedProcess(pid, fatal=index == 0)

    def preserve_then_release(*args: Any, **kwargs: Any) -> None:
        original_preserve(*args, **kwargs)
        lifecycle_persisted.set()

    monkeypatch.setattr(runner.subprocess, "Popen", coordinated_popen)
    monkeypatch.setattr(runner.os, "killpg", lambda _pid, _signal: None)
    monkeypatch.setattr(runner, "_preserve_lifecycle_failure", preserve_then_release)

    with pytest.raises(RuntimeError, match="owned process lifecycle"):
        runner.execute_jobs(
            jobs,
            scratch_root=tmp_path / "concurrent-scratch",
            timeout_seconds=30,
            max_workers=max_workers,
        )

    assert len(spawned_pids) == max_workers


@pytest.mark.parametrize("process_group_state", ("exists", "unknown", "gone"))
def test_resume_lifecycle_marker_requires_old_process_group_proven_gone(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    process_group_state: str,
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / f"resume-lifecycle-{process_group_state}"
    published = runner.run(_smoke_args(runner, fixture, output))
    manifest_path = published / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = runner.load_jsonl(published / "asset_records.jsonl")
    for name in ("artifact_manifest.json", "summary.json", "report.md"):
        (published / name).unlink()
    retained = json.loads(json.dumps(records[0]))
    retained["completion_order"] = 1
    runner.write_checkpoint(
        published,
        manifest,
        [retained],
        state="running",
        completion_order=1,
    )

    pending = manifest["records"][1]
    old_process_group_id = 424242
    scratch = published / ".worker_scratch"
    job_root = scratch / "job_interrupted"
    job_root.mkdir(parents=True)
    _write_json(
        job_root / "job.json",
        runner._job_for_manifest_row(pending, manifest, manifest_path),
    )
    marker = {
        "schema_version": 1,
        "asset_key": pending["asset_key"],
        "run_token": pending["run_token"],
        "process_group_id": old_process_group_id,
        "error": "owned process lifecycle cleanup failed after SIGKILL",
        "trigger": "BlockingIOError: post-spawn resource error",
        "worker_evidence": {
            "child_pid": old_process_group_id,
            "termination": {
                "process_group_id": old_process_group_id,
                "term_sent": True,
                "kill_sent": True,
                "reaped": False,
            },
        },
    }
    _write_json(job_root / "lifecycle_failure.json", marker)

    probes: list[tuple[int, int]] = []

    def probe_process_group(process_group_id: int, sent_signal: int) -> None:
        probes.append((process_group_id, sent_signal))
        assert sent_signal == 0
        if process_group_state == "unknown":
            raise PermissionError(errno.EPERM, "process-group ownership unknown")
        if process_group_state == "gone":
            raise ProcessLookupError(errno.ESRCH, "process group is gone")

    original_popen = runner.subprocess.Popen
    spawned_tokens: list[str] = []

    def capture_popen(
        command: list[str], *popen_args: Any, **popen_kwargs: Any
    ) -> Any:
        job = _internal_job_from_popen(command)
        if job is not None:
            spawned_tokens.append(job["run_token"])
        return original_popen(command, *popen_args, **popen_kwargs)

    monkeypatch.setattr(runner.os, "killpg", probe_process_group)
    monkeypatch.setattr(runner.subprocess, "Popen", capture_popen)
    resume = _smoke_args(runner, fixture, output)
    resume.resume = True

    if process_group_state != "gone":
        with pytest.raises(RuntimeError, match="prior worker process group"):
            runner.run(resume)
        assert spawned_tokens == []
        assert (job_root / "lifecycle_failure.json").is_file()
        checkpoint = json.loads(
            (published / "checkpoint.json").read_text(encoding="utf-8")
        )
        assert checkpoint["state"] == "running"
        assert checkpoint["completed"] == 1
    else:
        assert runner.run(resume) == published
        assert spawned_tokens == [pending["run_token"]]
        assert not scratch.exists()
        quarantines = list(
            output.parent.glob(f".{output.name}.worker_scratch.quarantine.*")
        )
        assert len(quarantines) == 1
        assert (
            quarantines[0]
            / "job_interrupted"
            / "lifecycle_failure.json"
        ).is_file()
    assert probes == [(old_process_group_id, 0)]


def test_smoke_publication_resume_artifacts_lock_and_no_overwrite(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "table3-output"
    args = _smoke_args(runner, fixture, output)

    published = runner.run(args)

    assert published == output.resolve()
    assert (published / "protocol_snapshot.md").read_bytes() == fixture["protocol"].read_bytes()
    manifest = json.loads((published / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["manifest_content_sha256"] == runner.manifest_self_hash(manifest)
    records = runner.load_jsonl(published / "asset_records.jsonl")
    assert [record["asset_id"] for record in records] == [
        "FactoryA/seed_000",
        "FactoryA/seed_001",
    ]
    summary = json.loads((published / "summary.json").read_text(encoding="utf-8"))
    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 2
    report = (published / "report.md").read_text(encoding="utf-8")
    assert "supplementary full generated cohort" in report.lower()
    assert "1 recovery" in report.lower()
    for evidence in (
        "Valid Range",
        "Joint Sweep Success",
        "Non-degenerate Motion",
        "Subtree Consistency",
        "FK Round-trip Error",
        "Joint-level Pass",
        "Strict Kinematic Pass",
        "Category macro",
        "Parse/tree",
        "Expected/observed joint denominator",
        "Zero-joint assets",
        "manifest.json",
        "asset_records.jsonl",
        "artifact_manifest.json",
    ):
        assert evidence in report
    runner.verify_artifact_manifest(published, exact=True)
    checkpoint = json.loads((published / "checkpoint.json").read_text(encoding="utf-8"))
    assert checkpoint["manifest_file_sha256"] == _sha(published / "manifest.json")
    assert checkpoint["records_sha256"] == _sha(published / "asset_records.jsonl")
    assert checkpoint["records_bytes"] == (published / "asset_records.jsonl").stat().st_size
    assert checkpoint["completed_asset_ids"] == [record["asset_id"] for record in records]
    assert len({record["run_token"] for record in records}) == 2

    with pytest.raises(RuntimeError, match="refusing to overwrite"):
        runner.run(_smoke_args(runner, fixture, output))
    complete_resume = _smoke_args(runner, fixture, output)
    complete_resume.resume = True
    with pytest.raises(RuntimeError, match="already complete"):
        runner.run(complete_resume)
    locked_output = tmp_path / "locked-output"
    with runner.output_lock(locked_output):
        with pytest.raises(RuntimeError, match="locked by another process"):
            runner.run(_smoke_args(runner, fixture, locked_output))

    # Simulate records rename succeeding while the next checkpoint rename has not happened.
    for name in ("artifact_manifest.json", "summary.json", "report.md"):
        (published / name).unlink()
    crash_records = json.loads(json.dumps(records))
    crash_records[0]["completion_order"] = 1
    crash_records[1]["completion_order"] = 2
    runner.write_checkpoint(
        published,
        manifest,
        [crash_records[0]],
        state="running",
        completion_order=1,
    )
    runner.write_records(published / "asset_records.jsonl", crash_records)
    resume_args = _smoke_args(runner, fixture, output)
    resume_args.resume = True
    runner.run(resume_args)

    resumed = runner.load_jsonl(published / "asset_records.jsonl")
    assert [record["asset_id"] for record in resumed] == [
        "FactoryA/seed_000",
        "FactoryA/seed_001",
    ]
    assert json.loads((published / "checkpoint.json").read_text(encoding="utf-8"))[
        "state"
    ] == "complete"
    runner.verify_artifact_manifest(published, exact=True)
    (published / "undeclared.txt").write_text("not in closure\n", encoding="utf-8")
    with pytest.raises(ValueError, match="undeclared"):
        runner.verify_artifact_manifest(published, exact=True)


def test_resume_rechecks_package_and_rejects_symlinked_scratch(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "resume-drift"
    published = runner.run(_smoke_args(runner, fixture, output))
    manifest = json.loads((published / "manifest.json").read_text(encoding="utf-8"))
    records = runner.load_jsonl(published / "asset_records.jsonl")
    for name in ("artifact_manifest.json", "summary.json", "report.md"):
        (published / name).unlink()
    runner.write_checkpoint(
        published,
        manifest,
        [records[0]],
        state="running",
        completion_order=records[0]["completion_order"],
    )
    (fixture["packages"][0][0] / "late.txt").write_text("drift\n", encoding="utf-8")
    resume = _smoke_args(runner, fixture, output)
    resume.resume = True
    runner.run(resume)
    resumed = runner.load_jsonl(published / "asset_records.jsonl")
    assert resumed[0]["status"] == "error"
    assert resumed[0]["strict_kinematic_pass"] is False

    scratch_output = tmp_path / "scratch-output"
    scratch_fixture = _fixture_inputs(tmp_path / "other")
    scratch_published = runner.run(_smoke_args(runner, scratch_fixture, scratch_output))
    scratch_manifest = json.loads(
        (scratch_published / "manifest.json").read_text(encoding="utf-8")
    )
    scratch_records = runner.load_jsonl(scratch_published / "asset_records.jsonl")
    for name in ("artifact_manifest.json", "summary.json", "report.md"):
        (scratch_published / name).unlink()
    runner.write_checkpoint(
        scratch_published,
        scratch_manifest,
        [scratch_records[0]],
        state="running",
        completion_order=scratch_records[0]["completion_order"],
    )
    scratch_target = tmp_path / "scratch-target"
    scratch_target.mkdir()
    (scratch_published / ".worker_scratch").symlink_to(
        scratch_target, target_is_directory=True
    )
    scratch_args = _smoke_args(runner, scratch_fixture, scratch_output)
    scratch_args.resume = True
    with pytest.raises(RuntimeError, match="scratch.*symlink"):
        runner.run(scratch_args)


def test_resume_seals_complete_checkpoint_after_artifact_manifest_crash(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / "seal-only-output"
    published = runner.run(_smoke_args(runner, fixture, output))
    before = {
        name: _sha(published / name)
        for name in (
            "manifest.json",
            "asset_records.jsonl",
            "summary.json",
            "report.md",
            "environment.json",
            "protocol_snapshot.md",
            "checkpoint.json",
        )
    }
    (published / "artifact_manifest.json").unlink()
    resume = _smoke_args(runner, fixture, output)
    resume.resume = True

    assert runner.run(resume) == published

    assert {
        name: _sha(published / name) for name in before
    } == before
    runner.verify_artifact_manifest(
        published, expected_files=runner.OUTPUT_ARTIFACT_FILES, exact=True
    )


@pytest.mark.parametrize("corruption", ("checkpoint_order", "record_order_gap"))
def test_seal_only_resume_rejects_non_contiguous_complete_order(
    tmp_path: Path, corruption: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / f"seal-only-order-{corruption}"
    published = runner.run(_smoke_args(runner, fixture, output))
    (published / "artifact_manifest.json").unlink()
    records_path = published / "asset_records.jsonl"
    records = runner.load_jsonl(records_path)
    checkpoint_path = published / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    tampered_order = len(records) + 1
    checkpoint["completion_order"] = tampered_order
    if corruption == "record_order_gap":
        records[-1]["completion_order"] = tampered_order
        runner.write_records(records_path, records)
        checkpoint["records_bytes"] = records_path.stat().st_size
        checkpoint["records_sha256"] = _sha(records_path)
    _write_json(checkpoint_path, checkpoint)
    resume = _smoke_args(runner, fixture, output)
    resume.resume = True

    with pytest.raises(ValueError, match="complete checkpoint completion order"):
        runner.run(resume)
    assert not (published / "artifact_manifest.json").exists()


@pytest.mark.parametrize(
    "corruption",
    ("missing_summary", "missing_report", "summary_tamper", "package_drift", "undeclared"),
)
def test_seal_only_resume_rejects_unverified_terminal_state(
    tmp_path: Path, corruption: str
) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    output = tmp_path / f"seal-only-invalid-{corruption}"
    published = runner.run(_smoke_args(runner, fixture, output))
    (published / "artifact_manifest.json").unlink()
    if corruption == "missing_summary":
        (published / "summary.json").unlink()
    elif corruption == "missing_report":
        (published / "report.md").unlink()
    elif corruption == "summary_tamper":
        summary = json.loads((published / "summary.json").read_text(encoding="utf-8"))
        summary["j_eval"] += 1
        _write_json(published / "summary.json", summary)
    elif corruption == "package_drift":
        (fixture["packages"][0][0] / "late.txt").write_text("drift\n", encoding="utf-8")
    else:
        (published / "undeclared.txt").write_text("not sealed\n", encoding="utf-8")
    resume = _smoke_args(runner, fixture, output)
    resume.resume = True

    with pytest.raises((ValueError, RuntimeError), match="seal-only validation"):
        runner.run(resume)
    assert not (published / "artifact_manifest.json").exists()


def test_loader_rejects_package_path_with_symlink_component(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    package_parent = fixture["packages"][0][0].parent
    alias = tmp_path / "package-alias"
    alias.symlink_to(package_parent, target_is_directory=True)
    cohort = fixture["cohort_manifest"]
    table2 = fixture["table2_manifest"]
    redirected = str(alias / fixture["packages"][0][0].name)
    cohort["assets"][0]["package_path"] = redirected
    cohort["manifest_content_sha256"] = _manifest_self_hash(cohort)
    _write_json(fixture["cohort_path"], cohort)
    _write_artifact_manifest(
        fixture["cohort_path"].parent,
        ["manifest.json", "cohort_protocol_snapshot.json"],
    )
    table2["source"]["cohort_manifest_sha256"] = _sha(fixture["cohort_path"])
    table2["source"]["cohort_manifest_content_sha256"] = cohort[
        "manifest_content_sha256"
    ]
    table2["source"]["cohort_artifact_manifest_sha256"] = _sha(
        fixture["cohort_artifacts"]
    )
    table2["records"][0]["package"] = redirected
    table2["records"][0]["cohort_record_sha256"] = _canonical_sha(cohort["assets"][0])
    table2["manifest_content_sha256"] = _manifest_self_hash(table2)
    _write_json(fixture["table2_path"], table2)
    _write_artifact_manifest(
        fixture["table2_path"].parent, ["manifest.json", "protocol_snapshot.md"]
    )

    with pytest.raises(ValueError, match="symlink"):
        runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)


def test_child_cli_rejects_non_smoke_test_action(tmp_path: Path) -> None:
    runner = load_runner()
    fixture = _fixture_inputs(tmp_path)
    loaded = runner.load_inputs(fixture["cohort_path"], fixture["table2_path"], formal=False)
    job = runner.make_job(
        loaded["assets"][0], manifest_content_sha256="e" * 64, samples=21, mode="formal"
    )
    job["internal_test_action"] = "sleep"
    job["internal_test_seconds"] = 0
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    _write_json(job_path, job)

    completed = subprocess.run(
        [
            str(EXP / ".venv_low_medium/bin/python"),
            str(RUNNER),
            "--internal-job",
            str(job_path),
            "--internal-result",
            str(result_path),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert completed.returncode != 0
    assert not result_path.exists()
