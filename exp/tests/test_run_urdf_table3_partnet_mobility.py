from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table3_partnet_mobility.py"
PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
SELECTION_ALGORITHM = (
    "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)"
)


def load_runner():
    assert RUNNER.is_file(), "PartNet-Mobility Table 3 runner has not been implemented"
    spec = importlib.util.spec_from_file_location(
        "urdf_table3_partnet_mobility", RUNNER
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selection_digest(dataset_id: str) -> str:
    return hashlib.sha256(f"{SELECTION_SALT}\0{dataset_id}".encode()).hexdigest()


def input_identity(item: dict[str, object]) -> str:
    fields = (
        "protocol_id",
        "order",
        "dataset_id",
        "selection_digest",
        "category",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "joint_specs_sha256",
        "runtime_identity_sha256",
        "urdf_sha256",
        "bounding_box_sha256",
        "collision_mesh_inventory_sha256",
        "object_bbox_diagonal_m",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
    )
    return canonical_sha256({field: item[field] for field in fields})


def refresh_manifest_hashes(manifest: dict[str, object]) -> None:
    items = manifest["items"]
    assert isinstance(items, list)
    for item in items:
        assert isinstance(item, dict)
        item["input_identity_sha256"] = input_identity(item)
    manifest["items_sha256"] = canonical_sha256(items)
    manifest["ordered_selected_ids_sha256"] = canonical_sha256(
        [item["dataset_id"] for item in items]
    )


def write_fixture(tmp_path: Path) -> tuple[Path, Path, list[str]]:
    source_root = tmp_path / "PartNet-Mobility" / "data" / "dataset"
    asset_ids = ["20", "10"]
    categories = ["Lamp", "Door"]
    items: list[dict[str, object]] = []
    joint_specs = [
        {
            "xml_index": 0,
            "name": "hinge",
            "type": "revolute",
            "lower": -1.0,
            "upper": 1.0,
            "range_evaluable": True,
        }
    ]
    runtime_identity = {"fixture_runtime": True}
    for order, (dataset_id, category) in enumerate(
        zip(asset_ids, categories, strict=True)
    ):
        package = source_root / dataset_id
        package.mkdir(parents=True)
        urdf = package / "mobility.urdf"
        urdf.write_text(
            """<robot name="fixture">
<link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
<origin xyz="1 0 0"/><axis xyz="0 0 1"/>
<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>\n""",
            encoding="utf-8",
        )
        bbox = package / "bounding_box.json"
        bbox.write_text(
            json.dumps({"min": [-1, -1, -1], "max": [1, 1, 1]}),
            encoding="utf-8",
        )
        (package / "meta.json").write_text(
            json.dumps({"model_cat": category}), encoding="utf-8"
        )
        item: dict[str, object] = {
            "audit_issue": None,
            "bounding_box_sha256": sha256_file(bbox),
            "category": category,
            "collision_mesh_files": [],
            "collision_mesh_inventory_sha256": canonical_sha256([]),
            "dataset_id": dataset_id,
            "joint_specs": joint_specs,
            "joint_specs_sha256": canonical_sha256(joint_specs),
            "missing_collision_mesh_reference_count": 0,
            "missing_core_files": [],
            "movable_dof_count": 1,
            "object_bbox_diagonal_m": math.sqrt(12.0),
            "order": order,
            "package_audit_success": True,
            "protocol_id": PROTOCOL_ID,
            "range_evaluable_dof_count": 1,
            "rest_state_expected": 1,
            "runtime_identity": runtime_identity,
            "runtime_identity_sha256": canonical_sha256(runtime_identity),
            "selection_digest": selection_digest(dataset_id),
            "single_state_expected": 21,
            "sobol_state_expected": 64,
            "urdf_sha256": sha256_file(urdf),
        }
        item["input_identity_sha256"] = input_identity(item)
        items.append(item)

    archive = tmp_path / "partnet-fixture.zip"
    archive.write_bytes(b"fixture archive")
    release_ids = sorted(asset_ids, key=int)
    manifest: dict[str, object] = {
        "archive": {
            "matches_expected_sha256": True,
            "path": str(archive),
            "sha256": sha256_file(archive),
            "size_bytes": archive.stat().st_size,
        },
        "candidate_pool_identity_sha256": canonical_sha256(release_ids),
        "cohort_boundary": {
            "is_full_release_cohort": False,
            "is_shared_category_balanced_cohort": False,
            "paper_table_role": "sampled release diagnostic",
        },
        "dataset_root": str(source_root),
        "items": items,
        "items_sha256": canonical_sha256(items),
        "ordered_selected_ids_sha256": canonical_sha256(asset_ids),
        "protocol_id": PROTOCOL_ID,
        "qualification_smoke": False,
        "release_asset_count": 2,
        "sample_size": 2,
        "selection_policy": {
            "algorithm": SELECTION_ALGORITHM,
            "identity_fields_used": ["dataset_id"],
            "outcome_based_filtering": False,
            "salt": SELECTION_SALT,
            "selected_failures_retained_without_replacement": True,
        },
        "status": "FROZEN",
    }
    manifest_path = tmp_path / "frozen_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, sort_keys=True), encoding="utf-8"
    )
    return source_root, manifest_path, asset_ids


def test_formal_contract_freezes_canonical_table4_cohort() -> None:
    runner = load_runner()
    args = runner.parse_args([])
    runner.validate_contract(args)

    assert args.mode == "formal"
    assert args.limit is None
    assert args.samples == 21
    assert args.workers == 4
    assert args.asset_timeout_seconds == 120.0

    for extra in (
        ["--limit", "3"],
        ["--samples", "20"],
        ["--workers", "3"],
        ["--asset-timeout-seconds", "30"],
        ["--source-root", "/tmp/other"],
        ["--cohort-manifest", "/tmp/other.json"],
    ):
        with pytest.raises(ValueError, match="formal"):
            runner.validate_contract(runner.parse_args(extra))


def test_formal_runtime_gate_rejects_identity_drift() -> None:
    runner = load_runner()
    frozen = runner._current_runtime_identity()

    runner._require_runtime_identity(frozen, formal=True)
    changed = dict(frozen, numpy_version="0.0-drift")
    with pytest.raises(RuntimeError, match="runtime identity"):
        runner._require_runtime_identity(changed, formal=True)


def test_loader_preserves_exact_item_order_and_binds_mobility_urdf(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    source_root, manifest_path, asset_ids = write_fixture(tmp_path)

    loaded = runner.load_cohort(source_root, manifest_path, formal=False)

    assert [row["asset_key"] for row in loaded["assets"]] == asset_ids
    assert [row["selection_index"] for row in loaded["assets"]] == [0, 1]
    assert [Path(row["urdf_path"]).name for row in loaded["assets"]] == [
        "mobility.urdf",
        "mobility.urdf",
    ]
    assert [row["category"] for row in loaded["assets"]] == ["Lamp", "Door"]
    assert all(row["declared_joint_count_hint"] == 1 for row in loaded["assets"])


@pytest.mark.parametrize("mutation", ["order", "duplicate", "package", "urdf", "category"])
def test_loader_rejects_manifest_or_live_package_drift(
    tmp_path: Path, mutation: str
) -> None:
    runner = load_runner()
    source_root, manifest_path, _asset_ids = write_fixture(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if mutation == "order":
        manifest["items"][0]["order"] = 1
        refresh_manifest_hashes(manifest)
    elif mutation == "duplicate":
        manifest["items"][1]["dataset_id"] = manifest["items"][0]["dataset_id"]
        manifest["items"][1]["selection_digest"] = manifest["items"][0][
            "selection_digest"
        ]
        refresh_manifest_hashes(manifest)
    elif mutation == "package":
        manifest["items"][0]["package"] = str(source_root)
        refresh_manifest_hashes(manifest)
    elif mutation == "urdf":
        package = source_root / manifest["items"][0]["dataset_id"]
        (package / "mobility.urdf").write_text(
            "<robot name='changed'/>", encoding="utf-8"
        )
    else:
        package = source_root / manifest["items"][0]["dataset_id"]
        (package / "meta.json").write_text(
            json.dumps({"model_cat": "changed"}), encoding="utf-8"
        )
    manifest_path.write_text(
        json.dumps(manifest, sort_keys=True), encoding="utf-8"
    )

    with pytest.raises((ValueError, RuntimeError)):
        runner.load_cohort(source_root, manifest_path, formal=False)


def test_smoke_run_uses_manifest_prefix_and_emits_bound_records(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    source_root, manifest_path, asset_ids = write_fixture(tmp_path)
    output = tmp_path / "output"
    runner.REPO_ROOT = tmp_path
    args = runner.parse_args(
        [
            "--mode",
            "smoke",
            "--source-root",
            str(source_root),
            "--cohort-manifest",
            str(manifest_path),
            "--limit",
            "1",
            "--workers",
            "1",
            "--asset-timeout-seconds",
            "30",
            "--output",
            str(output),
        ]
    )

    runner.run(args)

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert [row["asset_key"] for row in manifest["records"]] == asset_ids[:1]
    assert [row["asset_key"] for row in records] == asset_ids[:1]
    assert records[0]["package"] == str(source_root / asset_ids[0])
    assert records[0]["category"] == "Lamp"
    assert records[0]["manifest_content_sha256"] == manifest[
        "manifest_content_sha256"
    ]
    assert summary["n_eval"] == 1
    assert summary["j_eval"] == 1
    assert summary["metrics"]["strict_kinematic_pass"]["passed"] == 1
    assert summary["category_macro"]["category_count"] == 1

    resume_args = runner.parse_args(
        [
            "--mode",
            "smoke",
            "--source-root",
            str(source_root),
            "--cohort-manifest",
            str(manifest_path),
            "--limit",
            "1",
            "--workers",
            "1",
            "--asset-timeout-seconds",
            "30",
            "--output",
            str(output),
            "--resume",
        ]
    )
    with pytest.raises(RuntimeError, match="complete"):
        runner.run(resume_args)


def test_resume_record_binding_rejects_denominator_and_content_drift(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    source_root, manifest_path, _asset_ids = write_fixture(tmp_path)
    loaded = runner.load_cohort(source_root, manifest_path, formal=False)
    args = runner.parse_args(
        [
            "--mode",
            "smoke",
            "--source-root",
            str(source_root),
            "--cohort-manifest",
            str(manifest_path),
            "--limit",
            "1",
            "--workers",
            "1",
        ]
    )
    manifest = runner.build_manifest(args, loaded)
    job = {
        **manifest["records"][0],
        "samples": 21,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
    }
    record = runner._bind_record(
        runner.core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "fixture failure"
        ),
        job,
    )
    runner._validate_record_binding(record, job)

    denominator_drift = json.loads(json.dumps(record))
    denominator_drift["declared_joint_count"] = 0
    denominator_drift["joints"] = []
    denominator_drift["record_content_sha256"] = runner._record_self_hash(
        denominator_drift
    )
    with pytest.raises(RuntimeError, match="declared joint count"):
        runner._validate_record_binding(denominator_drift, job)

    content_drift = json.loads(json.dumps(record))
    content_drift["joints"][0]["joint_level_pass"] = True
    with pytest.raises(RuntimeError, match="content SHA256"):
        runner._validate_record_binding(content_drift, job)


@pytest.mark.parametrize("symlink_name", ["asset_records.jsonl", ".worker_scratch"])
def test_resume_path_validation_rejects_symlinks(
    tmp_path: Path, symlink_name: str
) -> None:
    runner = load_runner()
    output = tmp_path / "output"
    output.mkdir()
    (output / "manifest.json").write_text("{}", encoding="utf-8")
    (output / "checkpoint.json").write_text("{}", encoding="utf-8")
    target = tmp_path / "target"
    if symlink_name == "asset_records.jsonl":
        target.write_text("", encoding="utf-8")
        (output / symlink_name).symlink_to(target)
        (output / ".worker_scratch").mkdir()
    else:
        target.mkdir()
        (output / symlink_name).symlink_to(target, target_is_directory=True)
        (output / "asset_records.jsonl").write_text("", encoding="utf-8")

    with pytest.raises(RuntimeError, match="symlink"):
        runner._validate_resume_paths(output)


def test_internal_job_rejects_result_outside_job_directory(tmp_path: Path) -> None:
    runner = load_runner()
    scratch = tmp_path / ".worker_scratch"
    job_root = scratch / "job_fixture"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    job_path.write_text("{}", encoding="utf-8")
    outside = tmp_path / "outside.json"

    with pytest.raises(ValueError, match="internal result"):
        runner.run_internal_job(job_path, outside)
