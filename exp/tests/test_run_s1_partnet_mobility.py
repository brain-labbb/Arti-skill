from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_s1_partnet_mobility.py"
VERIFIER = REPO / "exp/scripts/verify_s1_partnet_mobility.py"


def load_module(path: Path, name: str) -> ModuleType:
    if not path.is_file():
        pytest.fail(f"required PartNet S1 module is not implemented: {path.name}")
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_package(root: Path, dataset_id: str) -> Path:
    package = root / dataset_id
    mesh = package / "textured_objs" / "part.obj"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
    (package / "mobility.urdf").write_text(
        """<robot name="fixture">
  <link name="base"><collision><geometry><mesh filename="textured_objs/part.obj"/></geometry></collision></link>
  <link name="middle"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="tip"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="j0" type="revolute"><parent link="base"/><child link="middle"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="j1" type="revolute"><parent link="middle"/><child link="tip"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    return package


def manifest_item(package: Path, dataset_id: str, order: int) -> dict[str, Any]:
    urdf = package / "mobility.urdf"
    mesh = package / "textured_objs" / "part.obj"
    item = {
        "category": "Fixture",
        "collision_mesh_files": [
            {
                "exists": True,
                "path": "textured_objs/part.obj",
                "sha256": sha256_file(mesh),
                "size_bytes": mesh.stat().st_size,
            }
        ],
        "dataset_id": dataset_id,
        "movable_dof_count": 2,
        "order": order,
        "package_audit_success": True,
        "protocol_id": "fixture-table4-partnet-v1",
        "range_evaluable_dof_count": 2,
        "rest_state_expected": 1,
        "single_state_expected": 2,
        "sobol_state_expected": 2,
        "urdf_sha256": sha256_file(urdf),
    }
    item["input_identity_sha256"] = canonical_sha256(item)
    return item


def write_cohort_fixture(tmp_path: Path) -> dict[str, Any]:
    dataset_root = tmp_path / "dataset"
    second = write_package(dataset_root, "200")
    first = write_package(dataset_root, "100")
    items = [manifest_item(second, "200", 0), manifest_item(first, "100", 1)]
    manifest = tmp_path / "frozen_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "status": "FROZEN",
                "protocol_id": "fixture-table4-partnet-v1",
                "dataset_root": str(dataset_root),
                "sample_size": 2,
                "items": items,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return {"dataset_root": dataset_root, "manifest": manifest, "items": items}


def state_row(dataset_id: str, phase: str, sample_index: int, illegal: int) -> dict[str, Any]:
    return {
        "all_pair_contact_count": illegal,
        "all_pair_illegal_penetration_count": illegal,
        "all_pair_max_penetration_m": 0.01 if illegal else 0.0,
        "category": "Fixture",
        "dataset_id": dataset_id,
        "joint_name": None if phase != "single_joint_sweep" else "j0",
        "joint_values_sha256": f"{sample_index + 1:064x}",
        "metric_max_penetration_m": 0.01 if illegal else 0.0,
        "non_adjacent_contact_count": illegal,
        "non_adjacent_illegal_penetration_count": illegal,
        "non_adjacent_max_penetration_m": 0.01 if illegal else 0.0,
        "phase": phase,
        "reset_readback_max_abs_error": 0.0,
        "sample_index": sample_index,
    }


def write_table4_fixture(tmp_path: Path, items: list[dict[str, Any]]) -> dict[str, Path]:
    all_states: list[dict[str, Any]] = []
    assets: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        states = [
            state_row(item["dataset_id"], "rest", 0, 0),
            state_row(item["dataset_id"], "single_joint_sweep", 0, 0),
            state_row(item["dataset_id"], "single_joint_sweep", 1, 0),
            state_row(item["dataset_id"], "multi_joint_sobol", 0, 0),
            state_row(item["dataset_id"], "multi_joint_sobol", 1, int(index == 1)),
        ]
        all_states.extend(states)
        strict = index == 0
        assets.append(
            {
                "category": "Fixture",
                "dataset_id": item["dataset_id"],
                "input_identity_sha256": item["input_identity_sha256"],
                "measurement_complete": True,
                "movable_dof_count": 2,
                "multi_joint_sobol_cf": strict,
                "order": index,
                "protocol_id": "fixture-table4-partnet-v1",
                "range_evaluable_dof_count": 2,
                "rest_non_adjacent_cf": True,
                "rest_non_adjacent_free": 1,
                "rest_state_executed": 1,
                "rest_state_expected": 1,
                "single_joint_sweep_cf": True,
                "single_non_adjacent_free": 2,
                "single_state_executed": 2,
                "single_state_expected": 2,
                "sobol_non_adjacent_free": 2 if strict else 1,
                "sobol_state_executed": 2,
                "sobol_state_expected": 2,
                "state_records_sha256": canonical_sha256(states),
                "strict_collision_pass": strict,
            }
        )
    manifest = tmp_path / "table4_manifest.json"
    asset_records = tmp_path / "table4_assets.json"
    state_records = tmp_path / "table4_states.jsonl"
    manifest.write_text(
        json.dumps(
            {
                "status": "FROZEN",
                "protocol_id": "fixture-table4-partnet-v1",
                "dataset_root": str(tmp_path / "dataset"),
                "sample_size": len(items),
                "items": items,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    asset_records.write_text(json.dumps(assets) + "\n", encoding="utf-8")
    state_records.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in all_states),
        encoding="utf-8",
    )
    return {"manifest": manifest, "assets": asset_records, "states": state_records}


def rebind_artifact(run: Path, name: str) -> None:
    manifest_path = run / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"][name]["sha256"] = sha256_file(run / name)
    manifest["artifacts"][name]["bytes"] = (run / name).stat().st_size
    manifest_without_hash = dict(manifest)
    manifest_without_hash.pop("manifest_content_sha256", None)
    manifest["manifest_content_sha256"] = canonical_sha256(manifest_without_hash)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_s1_run_fixture(tmp_path: Path) -> tuple[Path, ModuleType, ModuleType]:
    fixture = write_cohort_fixture(tmp_path)
    table4 = write_table4_fixture(tmp_path, fixture["items"])
    runner = load_module(RUNNER, f"s1_partnet_runner_{tmp_path.name}")
    verifier = load_module(VERIFIER, f"s1_partnet_verifier_{tmp_path.name}")
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "run"
    runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=1,
    )
    return output, runner, verifier


def test_cohort_uses_manifest_order_and_mobility_urdf_binding(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_module(RUNNER, "s1_partnet_runner_cohort")

    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )

    assert [row["dataset_id"] for row in cohort["records"]] == ["200", "100"]
    assert [row["selection_index"] for row in cohort["records"]] == [0, 1]
    assert [row["primary_urdf_relative_path"] for row in cohort["records"]] == [
        "mobility.urdf",
        "mobility.urdf",
    ]
    assert cohort["ordered_dataset_ids_sha256"] == (
        "57e6bed6935182816642db935d28226a0de6c1ce2266f2eea121cf1fdf02bf08"
    )


def test_table4_strict_is_reaggregated_from_partnet_raw_states(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_module(RUNNER, "s1_partnet_runner_reaggregate")
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, fixture["items"])

    evidence = runner.load_table4_evidence(
        table4["manifest"], table4["assets"], table4["states"], cohort, formal=False
    )

    assert evidence["strict_passed"] == 1
    assert evidence["denominator"] == 2
    assert [row["strict_collision_pass"] for row in evidence["records"]] == [True, False]
    assert evidence["state_record_count"] == 10


def test_table4_missing_state_fails_closed_instead_of_trusting_asset_flag(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_module(RUNNER, "s1_partnet_runner_missing_state")
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, fixture["items"])
    rows = table4["states"].read_text(encoding="utf-8").splitlines()
    table4["states"].write_text("\n".join(rows[:-1]) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="state records SHA256 mismatch"):
        runner.load_table4_evidence(
            table4["manifest"], table4["assets"], table4["states"], cohort, formal=False
        )


def test_table4_duplicate_state_identity_is_rejected(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_module(RUNNER, "s1_partnet_runner_duplicate_state")
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, fixture["items"])
    rows = [json.loads(line) for line in table4["states"].read_text().splitlines()]
    rows[4] = dict(rows[3])
    table4["states"].write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )
    assets = json.loads(table4["assets"].read_text(encoding="utf-8"))
    assets[0]["state_records_sha256"] = canonical_sha256(rows[:5])
    table4["assets"].write_text(json.dumps(assets) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate Table 4 state identity"):
        runner.load_table4_evidence(
            table4["manifest"], table4["assets"], table4["states"], cohort, formal=False
        )


def test_static_audit_and_aggregate_keep_s1_denominators(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_module(RUNNER, "s1_partnet_runner_aggregate")
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4_paths = write_table4_fixture(tmp_path, fixture["items"])
    table4 = runner.load_table4_evidence(
        table4_paths["manifest"],
        table4_paths["assets"],
        table4_paths["states"],
        cohort,
        formal=False,
    )
    records = []
    for source, strict in zip(cohort["records"], table4["records"], strict=True):
        evidence = runner.audit_release_evidence(source)
        records.append(runner.build_s1_asset_record(source, evidence, strict))

    metrics = runner.aggregate_s1(records)

    assert metrics["receipt_bound_assets"] == {"passed": 0, "denominator": 2, "rate": 0.0}
    assert metrics["receipt_replay_pass"] == {"passed": 0, "denominator": 2, "rate": 0.0}
    assert metrics["deterministic_rebuild_match"]["status"] == "N/E"
    assert metrics["deterministic_rebuild_match"]["asset_denominator"] == 2
    assert metrics["allowance_density"] == {
        "registered_pairs": 0,
        "eligible_pairs": 2,
        "rate": 0.0,
    }
    assert metrics["strict_pass_no_method_allowance"] == {
        "passed": 1,
        "denominator": 2,
        "rate": 0.5,
    }
    assert metrics["registered_allowance_gain_pp"]["value"] == 0.0


def test_frozen_missing_mesh_is_retained_with_partial_closure(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    missing = fixture["dataset_root"] / "200" / "textured_objs" / "part.obj"
    missing.unlink()
    fixture["items"][0]["collision_mesh_files"][0] = {
        "exists": False,
        "path": "textured_objs/part.obj",
        "sha256": None,
        "size_bytes": None,
    }
    manifest = json.loads(fixture["manifest"].read_text(encoding="utf-8"))
    manifest["items"] = fixture["items"]
    fixture["manifest"].write_text(json.dumps(manifest) + "\n", encoding="utf-8")
    runner = load_module(RUNNER, "s1_partnet_runner_retained_missing")

    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    evidence = runner.audit_release_evidence(cohort["records"][0])

    assert len(cohort["records"]) == 2
    assert evidence["resource_closure"]["status"] == "PARTIAL"
    assert evidence["resource_closure"]["complete"] is False
    assert evidence["receipt"]["receipt_bound_asset"] == 0
    assert evidence["receipt_replay"]["passed"] is False
    assert evidence["allowance"]["eligible_nonadjacent_pair_count"] == 1


def test_run_artifacts_are_accepted_by_independent_verifier(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    table4 = write_table4_fixture(tmp_path, fixture["items"])
    runner = load_module(RUNNER, "s1_partnet_runner_end_to_end")
    verifier = load_module(VERIFIER, "s1_partnet_verifier_end_to_end")
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "run"

    summary = runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=1,
    )
    verification = verifier.verify_run(output, formal=False)

    assert summary["metrics"]["strict_pass_no_method_allowance"]["passed"] == 1
    assert verification["all_pass"] is True
    assert all(check["pass"] for check in verification["checks"])


def test_verifier_rejects_rebound_strict_result_tampering(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    table4 = write_table4_fixture(tmp_path, fixture["items"])
    runner = load_module(RUNNER, "s1_partnet_runner_tamper")
    verifier = load_module(VERIFIER, "s1_partnet_verifier_tamper")
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "run"
    runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=1,
    )
    records_path = output / "asset_records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["strict_collision_pass_no_method_allowance"] = False
    rows[0]["strict_collision_pass_registered_allowance"] = False
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )
    rebind_artifact(output, "asset_records.jsonl")

    with pytest.raises(verifier.VerificationError, match="asset record mismatch"):
        verifier.verify_run(output, formal=False)


def test_verifier_rejects_rebound_receipt_replay_tampering(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    records_path = output / "asset_records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["release_receipt_replay_pass"] = True
    rows[0]["receipt_replay_status"] = "REPLAY_MATCH"
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )
    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["metrics"]["receipt_replay_pass"] = {
        "passed": 1,
        "denominator": 2,
        "rate": 0.5,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    rebind_artifact(output, "asset_records.jsonl")
    rebind_artifact(output, "summary.json")

    with pytest.raises(verifier.VerificationError, match="asset record mismatch"):
        verifier.verify_run(output, formal=False)


def test_verifier_requires_exact_artifact_set(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"] = {}
    manifest_without_hash = dict(manifest)
    manifest_without_hash.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = canonical_sha256(manifest_without_hash)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    with pytest.raises(verifier.VerificationError, match="artifact set mismatch"):
        verifier.verify_run(output, formal=False)


def test_verifier_rejects_symlinked_run_manifest_before_loading(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    manifest = output / "manifest.json"
    real_manifest = output / ".real_manifest.json"
    manifest.rename(real_manifest)
    manifest.symlink_to(real_manifest.name)

    with pytest.raises(verifier.VerificationError, match="run manifest is not a safe regular child"):
        verifier.verify_run(output, formal=False)


def test_verifier_rejects_rebound_summary_markdown_tampering(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    (output / "summary.md").write_text("# Forged result\n", encoding="utf-8")
    rebind_artifact(output, "summary.md")

    with pytest.raises(verifier.VerificationError, match="summary markdown mismatch"):
        verifier.verify_run(output, formal=False)


def test_runner_rejects_symlinked_dataset_package(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    real_package = fixture["dataset_root"] / "_real_200"
    package = fixture["dataset_root"] / "200"
    package.rename(real_package)
    package.symlink_to(real_package, target_is_directory=True)
    runner = load_module(RUNNER, "s1_partnet_runner_symlink")

    with pytest.raises(ValueError, match="symlink"):
        runner.load_cohort(fixture["manifest"], fixture["dataset_root"], formal=False)


def test_verifier_rejects_unchecked_asset_record_field_tampering(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    records_path = output / "asset_records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["asset_id"] = "forged-asset-id"
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )
    rebind_artifact(output, "asset_records.jsonl")

    with pytest.raises(verifier.VerificationError, match="asset record mismatch"):
        verifier.verify_run(output, formal=False)


def test_evidence_candidate_inventory_is_frozen_and_reverified(tmp_path: Path) -> None:
    output, _, verifier = write_s1_run_fixture(tmp_path)
    inventory = output / "evidence_inventory.json"
    assert inventory.is_file()
    first = json.loads((output / "asset_records.jsonl").read_text().splitlines()[0])
    (Path(first["package"]) / "mechanical_receipt.json").write_text("{}\n", encoding="utf-8")

    with pytest.raises(verifier.VerificationError, match="evidence candidate inventory mismatch"):
        verifier.verify_run(output, formal=False)
