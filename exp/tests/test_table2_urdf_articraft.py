#!/usr/bin/env python3
"""Focused tests for the Articraft-10K Table 2 URDF evaluator."""

import importlib.util
import base64
import csv
import errno
import hashlib
import json
import math
import os
import struct
import subprocess
import time
import sys
from pathlib import Path
from typing import Any

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "exp/scripts/run_table2_urdf_articraft.py"

spec = importlib.util.spec_from_file_location("table2_urdf_articraft_runner", RUNNER_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


def test_runner_module_exists() -> None:
    """Catch a missing executable entrypoint for the approved evaluation."""

    assert RUNNER_PATH.is_file()


def write_asset(tmp_path: Path, urdf: str, files: dict[str, bytes] | None = None) -> Path:
    asset = tmp_path / "asset"
    asset.mkdir(parents=True)
    (asset / "model.urdf").write_text(urdf.strip() + "\n", encoding="utf-8")
    for relative, payload in (files or {}).items():
        target = asset / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    return asset


def write_artiverse_fixture(
    tmp_path: Path,
    *,
    model_ids: tuple[str, ...] = ("model_a", "model_b"),
) -> tuple[Path, Path, dict[str, Any]]:
    artiverse = tmp_path / "artiverse"
    roots = [f"data/category_{index}/source_{index}/{model_id}" for index, model_id in enumerate(model_ids)]
    chunks = artiverse / "dataset_chunks"
    chunks.mkdir(parents=True)
    release = {
        "format": "artiverse-data-tar-gz-chunks-v1",
        "created_utc": "2026-01-01T00:00:00Z",
        "data_dir": "data",
        "chunk_count": 1,
        "model_count": len(roots),
        "file_count": len(roots),
        "input_bytes": len(roots),
        "chunks": [{
            "archive": "fixture.tar.gz",
            "sha256": "1" * 64,
            "archive_bytes": len(roots),
            "model_count": len(roots),
            "file_count": len(roots),
            "input_bytes": len(roots),
            "roots": roots,
        }],
    }
    release_path = chunks / "manifest.json"
    release_path.write_text(json.dumps(release), encoding="utf-8")
    release_hash = hashlib.sha256(release_path.read_bytes()).hexdigest()
    seed = "20260813"
    ranked: list[dict[str, Any]] = []
    for index, (root, model_id) in enumerate(zip(roots, model_ids, strict=True)):
        selection_hash = hashlib.sha256(
            "\0".join(("artiverse-table1-global-sample-v1", release_hash, seed, root)).encode()
        ).hexdigest()
        ranked.append({
            "asset_id": root,
            "manifest_root": root,
            "raw_category": f"category_{index}",
            "source": f"source_{index}",
            "model_id": model_id,
            "chunk_archive": "fixture.tar.gz",
            "selection_hash": selection_hash,
        })
        package = artiverse / root / "urdf_w_collider"
        package.mkdir(parents=True)
        (package / f"{model_id}.urdf").write_text(
            f'<robot name="{model_id}">{valid_link()}</robot>\n', encoding="utf-8"
        )
    ranked.sort(key=lambda row: (row["selection_hash"], row["asset_id"]))
    assets = [{**row, "selection_rank": rank} for rank, row in enumerate(ranked, start=1)]
    universe = hashlib.sha256(
        "".join(f"{asset_id}\n" for asset_id in sorted(roots)).encode()
    ).hexdigest()
    cohort = {
        "schema_version": 1,
        "dataset": "Artiverse",
        "release_status": "PRE_RELEASE_SUBSET",
        "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        "N_release": len(roots),
        "N_eval": len(assets),
        "seed": seed,
        "selection_protocol": "artiverse-table1-global-sample-v1",
        "release_manifest": "dataset_chunks/manifest.json",
        "release_manifest_sha256": release_hash,
        "release_universe_sha256": universe,
        "assets": assets,
    }
    cohort_path = tmp_path / "table1_manifest.json"
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")
    return artiverse, cohort_path, cohort


def valid_link(name: str = "base") -> str:
    return f"""
    <link name="{name}">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
      <visual><geometry><box size="1 1 1"/></geometry></visual>
      <collision><geometry><box size="1 1 1"/></geometry></collision>
    </link>
    """


def metric_states(record: dict[str, Any]) -> dict[str, bool]:
    return {name: record["metrics"][name]["pass"] for name in runner.METRIC_NAMES}


def valid_bound_record(asset_id: str = "asset") -> dict[str, Any]:
    metrics = {name: {"pass": True, "issues": []} for name in runner.METRIC_NAMES}
    return {
        "asset_id": asset_id,
        "status": "completed",
        "error": None,
        "metrics": metrics,
        "strict_urdf_pass": True,
        "model_urdf_sha256": "a" * 64,
        "package_content_manifest_sha256": "b" * 64,
        "manifest_content_sha256": "c" * 64,
    }


def fake_runtime_binding(run_token: str = "a" * 32) -> dict[str, Any]:
    config = {"schema_version": "test"}
    environment = {"python": "test"}
    return {
        "run_token": run_token,
        "evaluator_path": "/repo/evaluator.py",
        "evaluator_sha256": "1" * 64,
        "protocol_path": "/repo/protocol.md",
        "protocol_sha256": "2" * 64,
        "config": config,
        "config_sha256": runner.canonical_sha256(config),
        "environment": environment,
        "environment_sha256": runner.canonical_sha256(environment),
    }


def fake_runtime_evaluation(binding: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in binding.items() if key != "run_token"}


def fake_protocol_binding(tmp_path: Path) -> dict[str, str]:
    source = tmp_path / "builder_protocol_source.md"
    source.write_bytes(b"builder protocol\n")
    output = tmp_path / "builder_output"
    output.mkdir(exist_ok=True)
    snapshot = output / "protocol_snapshot.md"
    snapshot.write_bytes(source.read_bytes())
    digest = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    return {
        "protocol_source_path": str(source.resolve()),
        "protocol_source_sha256_at_freeze": digest,
        "protocol_path": str(snapshot.resolve()),
        "protocol_sha256": digest,
    }


def bind_current_runtime(job: dict[str, Any], output_root: Path) -> dict[str, Any]:
    output_root.mkdir(parents=True, exist_ok=True)
    snapshot = output_root / "protocol_snapshot.md"
    snapshot.write_bytes(b"worker protocol snapshot\n")
    job["output_root"] = str(output_root.resolve())
    job["runtime_binding"] = {"protocol_path": str(snapshot.resolve())}
    binding = runner.current_worker_runtime_binding(job)
    job["runtime_binding"] = binding
    return binding


def test_all_manifest_builders_use_explicit_snapshot_without_reading_live_protocol(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch any dataset builder reverting to an implicit mutable PROTOCOL_PATH read."""

    protocol_binding = fake_protocol_binding(tmp_path)
    missing_live_protocol = tmp_path / "deleted_live_protocol.md"
    monkeypatch.setattr(runner, "PROTOCOL_PATH", missing_live_protocol)

    articraft_root = tmp_path / "articraft"
    articraft_asset = articraft_root / "asset"
    articraft_asset.mkdir(parents=True)
    (articraft_asset / "model.urdf").write_text(
        f'<robot name="asset">{valid_link()}</robot>\n', encoding="utf-8"
    )
    monkeypatch.setattr(runner, "load_inventory_entry", lambda: ({
        "urdf_root": str(articraft_root.resolve()),
        "source": {"repo_id": "fixture/articraft", "revision": "fixture"},
    }, "i" * 64))
    articraft_manifest = runner.build_manifest(
        articraft_root, ["asset"], ["asset"], 1, 1, None, False, 1,
        protocol_binding,
    )

    artiverse_root, artiverse_cohort, _ = write_artiverse_fixture(tmp_path)
    artiverse_manifest = runner.build_artiverse_manifest(
        runner.load_artiverse_cohort(artiverse_root, artiverse_cohort, formal=False),
        requested_n=2, limit=None, standard_parser=False, workers=1,
        protocol_binding=protocol_binding, mode="smoke",
    )
    partnet_root, partnet_cohort, _ = write_partnet_mobility_fixture(tmp_path)
    partnet_manifest = runner.build_partnet_mobility_manifest(
        runner.load_partnet_mobility_cohort(partnet_root, partnet_cohort, formal=False),
        requested_n=2, limit=None, standard_parser=False, workers=1,
        protocol_binding=protocol_binding, mode="smoke",
    )
    lam_root, lam_cohort, lam_inventory, _lam_table3, _lam_records = write_lam_table2_fixture(
        tmp_path
    )
    monkeypatch.setattr(runner, "INVENTORY_PATH", lam_inventory)
    lam_manifest = runner.build_lam_manifest(
        runner.load_lam_cohort(lam_root, lam_cohort, formal=False),
        requested_n=4, limit=None, standard_parser=False, workers=1,
        protocol_binding=protocol_binding, mode="smoke",
    )

    for manifest in (articraft_manifest, artiverse_manifest, partnet_manifest, lam_manifest):
        assert {
            field: manifest["evaluation"][field] for field in runner.PROTOCOL_BINDING_FIELDS
        } == protocol_binding


def test_deterministic_selection_is_content_blind_and_stable() -> None:
    """Catch selection that depends on input order or post-evaluation outcomes."""

    pool = [f"asset_{index}" for index in range(10)]
    expected = ["asset_2", "asset_4", "asset_7", "asset_0"]
    assert runner.select_asset_ids(pool, n=4, seed=20260813) == expected
    assert runner.select_asset_ids(reversed(pool), n=4, seed=20260813) == expected
    with pytest.raises(ValueError):
        runner.select_asset_ids(pool, n=11, seed=20260813)


def test_valid_single_link_asset_passes_all_table2_metrics(tmp_path: Path) -> None:
    """Catch a gate that rejects a standard one-link URDF or omits a metric."""

    asset = write_asset(tmp_path, f'<robot name="valid">{valid_link()}</robot>')
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    assert metric_states(record) == {name: True for name in runner.METRIC_NAMES}
    assert record["strict_urdf_pass"] is True


def test_pinned_standard_parser_runs_under_the_frozen_numpy_environment(
    tmp_path: Path,
) -> None:
    """Catch urdfpy compatibility failures being misreported as asset parse failures."""

    mesh_link = valid_link().replace(
        '<visual><geometry><box size="1 1 1"/></geometry></visual>',
        '<visual><geometry><mesh filename="assets/triangle.obj"/></geometry>'
        '<material name="m"><color rgba="0.1 0.2 0.3 1"/></material></visual>',
    )
    asset = write_asset(
        tmp_path,
        f'<robot name="valid">{mesh_link}</robot>',
        {"assets/triangle.obj": b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"},
    )
    record = runner.audit_asset_package(asset, run_standard_parser=True)
    assert record["metrics"]["parse_rate"]["pass"] is True
    assert record["metrics"]["parse_rate"]["parser"] == "urdfpy"
    assert record["metrics"]["parse_rate"]["version"] == "0.0.22"


def test_formal_mode_contract_is_fixed_and_smoke_is_labeled_non_formal() -> None:
    formal = runner.parse_args([])
    runner.validate_run_contract(formal)
    assert formal.mode == "formal"
    assert (formal.n, formal.seed, formal.limit, formal.workers) == (800, 20260813, None, 4)
    assert formal.no_standard_parser is False

    invalid = runner.parse_args(["--mode", "formal", "--n", "5"])
    with pytest.raises(ValueError, match="formal"):
        runner.validate_run_contract(invalid)

    smoke = runner.parse_args(
        ["--mode", "smoke", "--n", "5", "--limit", "2", "--no-standard-parser"]
    )
    runner.validate_run_contract(smoke)
    assert runner.run_classification(smoke.mode) == "NON_FORMAL_SMOKE"


def test_cli_defaults_to_articraft_and_supports_formal_artiverse_profile() -> None:
    articraft = runner.parse_args([])
    assert articraft.dataset == "Articraft-10K"
    assert articraft.source_root == runner.DEFAULT_SOURCE_ROOT
    assert articraft.cohort_manifest is None

    artiverse = runner.parse_args(["--dataset", "Artiverse"])
    assert artiverse.source_root == runner.DEFAULT_ARTIVERSE_SOURCE_ROOT
    assert artiverse.cohort_manifest == runner.DEFAULT_ARTIVERSE_COHORT_MANIFEST
    runner.validate_run_contract(artiverse)

    for extra in (
        ["--n", "799"],
        ["--workers", "3"],
        ["--limit", "1"],
        ["--asset-timeout-seconds", "119"],
        ["--no-standard-parser"],
        ["--source-root", "/tmp/not-artiverse"],
        ["--cohort-manifest", "/tmp/not-the-table1-manifest.json"],
    ):
        invalid = runner.parse_args(["--dataset", "Artiverse", *extra])
        with pytest.raises(ValueError, match="formal"):
            runner.validate_run_contract(invalid)


def write_partnet_mobility_fixture(tmp_path: Path) -> tuple[Path, Path, dict[str, Any]]:
    """Build a hand-derived two-item frozen PartNet-Mobility cohort."""

    def canonical_sha256(value: Any) -> str:
        return hashlib.sha256(
            json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
        ).hexdigest()

    def item_identity(item: dict[str, Any]) -> str:
        return canonical_sha256({
            key: item[key]
            for key in (
                "protocol_id", "order", "dataset_id", "selection_digest", "category",
                "movable_dof_count", "range_evaluable_dof_count", "joint_specs_sha256",
                "runtime_identity_sha256", "urdf_sha256", "bounding_box_sha256",
                "collision_mesh_inventory_sha256", "object_bbox_diagonal_m",
                "rest_state_expected", "single_state_expected", "sobol_state_expected",
            )
        })

    source_root = tmp_path / "data" / "dataset"
    items: list[dict[str, Any]] = []
    for order, (dataset_id, category) in enumerate((("101", "Chair"), ("202", "Lamp"))):
        package = source_root / dataset_id
        package.mkdir(parents=True)
        primary = package / "mobility.urdf"
        collision = package / "textured_objs" / "collision.obj"
        collision.parent.mkdir()
        collision.write_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        urdf_link = valid_link().replace(
            '<collision><geometry><box size="1 1 1"/></geometry></collision>',
            '<collision><geometry><mesh filename="textured_objs/collision.obj"/></geometry></collision>',
        )
        primary.write_text(
            f'<robot name="{dataset_id}">{urdf_link}</robot>\n', encoding="utf-8"
        )
        bbox = package / "bounding_box.json"
        bbox.write_text('{"min": [0, 0, 0], "max": [1, 1, 1]}', encoding="utf-8")
        (package / "meta.json").write_text(json.dumps({"model_cat": category}), encoding="utf-8")
        collision_rows = [{
            "path": "textured_objs/collision.obj",
            "exists": True,
            "size_bytes": collision.stat().st_size,
            "sha256": hashlib.sha256(collision.read_bytes()).hexdigest(),
        }]
        item = {
            "dataset_id": dataset_id,
            "category": category,
            "order": order,
            "protocol_id": "urdf_sim_ready_table4_partnet_mobility_n800_v1",
            "urdf_sha256": hashlib.sha256(primary.read_bytes()).hexdigest(),
            "bounding_box_sha256": hashlib.sha256(bbox.read_bytes()).hexdigest(),
            "collision_mesh_files": collision_rows,
            "collision_mesh_inventory_sha256": canonical_sha256(collision_rows),
            "selection_digest": hashlib.sha256(
                f"urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813\0{dataset_id}".encode()
            ).hexdigest(),
            "movable_dof_count": 0,
            "range_evaluable_dof_count": 0,
            "joint_specs_sha256": canonical_sha256([]),
            "runtime_identity_sha256": "r" * 64,
            "object_bbox_diagonal_m": math.sqrt(3.0),
            "rest_state_expected": 1,
            "single_state_expected": 0,
            "sobol_state_expected": 0,
            "package_audit_success": True,
        }
        item["input_identity_sha256"] = item_identity(item)
        items.append(item)
    archive = tmp_path / "partnet-mobility-v0.zip"
    archive.write_bytes(b"hand-derived PartNet fixture archive")
    cohort = {
        "status": "FROZEN",
        "protocol_id": "urdf_sim_ready_table4_partnet_mobility_n800_v1",
        "sample_size": len(items),
        "release_asset_count": len(items),
        "candidate_pool_identity_sha256": runner.canonical_sha256(["101", "202"]),
        "selection_policy": {
            "algorithm": "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)",
            "salt": "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813",
            "outcome_based_filtering": False,
            "selected_failures_retained_without_replacement": True,
        },
        "items": items,
        "items_sha256": runner.canonical_sha256(items),
        "ordered_selected_ids_sha256": runner.canonical_sha256(["101", "202"]),
        "archive": {
            "path": str(archive),
            "size_bytes": archive.stat().st_size,
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        },
    }
    cohort_path = tmp_path / "frozen_manifest.json"
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")
    return source_root, cohort_path, cohort


def write_lam_table2_fixture(
    tmp_path: Path,
) -> tuple[Path, Path, Path, dict[str, Any], list[dict[str, Any]]]:
    """Build a four-item LAM release whose Table 3 JSONL is in completion order."""

    dataset_root = tmp_path / "Articulated-Object-Code"
    source_root = dataset_root / "released_outputs"
    release_rows = [
        ("shared_000", "tool", "viable", "objects/tool/shared_000"),
        ("shared_000", "appliance", "broken", "imperfect/appliance/shared_000"),
        ("third_000", "tool", "loads_only", "objects/tool/third_000"),
        ("fourth_000", "lamp", "broken", "imperfect/lamp/fourth_000"),
    ]
    frozen_records: list[dict[str, Any]] = []
    for release_order, (object_id, category, tier, rel_path) in enumerate(release_rows):
        package = source_root / rel_path
        package.mkdir(parents=True)
        primary = package / "generated.urdf"
        primary.write_text(
            f'<robot name="{object_id}_{release_order}">{valid_link()}</robot>\n',
            encoding="utf-8",
        )
        (package / "bounded-extra.txt").write_text(f"package-{release_order}\n", encoding="utf-8")
        asset_key = f"{tier}:{rel_path}"
        rank = release_order + 1
        frozen_records.append({
            "release_order": release_order,
            "asset_key": asset_key,
            "object_release_id": object_id,
            "category": category,
            "tier": tier,
            "rel_path": rel_path,
            "declared_joint_count_hint": 1,
            "urdf_path": str(primary),
            "urdf_exists": True,
            "urdf_sha256": hashlib.sha256(primary.read_bytes()).hexdigest(),
            "selection_rank": rank,
            "selection_hash": hashlib.sha256(
                f"lam-table3-v1\0{runner.DEFAULT_SEED}\0{asset_key}".encode()
            ).hexdigest(),
        })

    release_path = dataset_root / "manifest.csv"
    with release_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("object_release_id", "category", "tier", "rel_path", "n_movable"),
        )
        writer.writeheader()
        for object_id, category, tier, rel_path in release_rows:
            writer.writerow({
                "object_release_id": object_id,
                "category": category,
                "tier": tier,
                "rel_path": rel_path,
                "n_movable": 1,
            })
    dataset_api_path = dataset_root / "dataset_api.json"
    revision = "fixture-lam-revision"
    dataset_api_path.write_text(json.dumps({"sha": revision}), encoding="utf-8")
    archives = []
    for tier in ("viable", "loads_only", "broken"):
        archive = dataset_root / f"{tier}.tar.gz"
        archive.write_bytes(f"fixture-{tier}".encode())
        archives.append({
            "name": archive.name,
            "bytes": archive.stat().st_size,
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        })

    table3_root = tmp_path / "runtime" / "table3_lam_fixture"
    table3_root.mkdir(parents=True)
    table3_manifest_path = table3_root / "manifest.json"
    selected_keys = [row["asset_key"] for row in frozen_records]
    tier_counts = dict(sorted({tier: sum(row[2] == tier for row in release_rows) for tier in {
        row[2] for row in release_rows
    }}.items()))
    table3_manifest = {
        "schema_version": 1,
        "dataset": "LAM released outputs (Articulated-Object-Code)",
        "classification": "FORMAL",
        "source": {
            "source_root": str(source_root.resolve()),
            "release_manifest": str(release_path.resolve()),
            "release_manifest_sha256": hashlib.sha256(release_path.read_bytes()).hexdigest(),
            "dataset_api": str(dataset_api_path.resolve()),
            "dataset_api_sha256": hashlib.sha256(dataset_api_path.read_bytes()).hexdigest(),
            "upstream_revision": revision,
            "n_release": len(release_rows),
            "tier_counts": tier_counts,
            "candidate_pool_sha256": runner.canonical_sha256(sorted(selected_keys)),
        },
        "selection": {
            "algorithm": "random.Random(seed).sample(sorted(asset_key), n)",
            "quality_label_blind": True,
            "seed": runner.DEFAULT_SEED,
            "n_eval": len(frozen_records),
            "selected_asset_keys_sha256": runner.canonical_sha256(selected_keys),
        },
        "records": frozen_records,
    }
    table3_manifest["manifest_content_sha256"] = runner.manifest_self_hash(table3_manifest)
    table3_manifest_path.write_text(json.dumps(table3_manifest), encoding="utf-8")

    completion_rows = []
    for index, frozen in enumerate(frozen_records):
        is_error = index in {1, 3}
        completion_rows.append({
            "asset_key": frozen["asset_key"],
            "object_release_id": frozen["object_release_id"],
            "category": frozen["category"],
            "tier": frozen["tier"],
            "rel_path": frozen["rel_path"],
            "selection_rank": frozen["selection_rank"],
            "selection_hash": frozen["selection_hash"],
            "urdf_sha256": frozen["urdf_sha256"],
            "manifest_content_sha256": table3_manifest["manifest_content_sha256"],
            "status": "error" if is_error else "completed",
            "error": f"fixture error {index}" if is_error else None,
            "parse_success": not is_error,
            "tree_valid": not is_error,
            "strict_kinematic_pass": not is_error,
            "joints": [{"large_table3_payload_must_not_be_copied": "x" * 128}],
        })
    cohort_records_path = table3_root / "asset_records.jsonl"
    cohort_records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in reversed(completion_rows)),
        encoding="utf-8",
    )

    inventory_path = tmp_path / "dataset_inventory.json"
    inventory_path.write_text(json.dumps({"datasets": [{
        "name": "LAM released outputs",
        "status": "VERIFIED_RELEASE_COMPLETE",
        "canonical_path": str(dataset_root.resolve()),
        "released_outputs_root": str(source_root.resolve()),
        "manifest": str(release_path.resolve()),
        "source": {
            "type": "huggingface_dataset",
            "repo_id": "YipengGao/Articulated-Object-Code",
            "revision": revision,
            "license": "mit",
        },
        "archives": archives,
        "verification": {
            "manifest_objects": len(release_rows),
            "tier_counts": tier_counts,
            "manifest_urdf_files_present": len(release_rows),
            "manifest_urdf_xml_parse_failures": 0,
            "archive_sha256_failures": 0,
            "cross_archive_path_collisions": 0,
        },
    }]}), encoding="utf-8")
    return source_root, cohort_records_path, inventory_path, table3_manifest, completion_rows


def patch_lam_fixture_constants(
    monkeypatch: Any,
    source_root: Path,
    cohort_records_path: Path,
    inventory_path: Path,
    table3_manifest: dict[str, Any],
) -> None:
    """Point formal LAM identity constants at a synthetic release."""

    dataset_root = source_root.parent
    table3_manifest_path = cohort_records_path.parent / "manifest.json"
    release_path = dataset_root / "manifest.csv"
    dataset_api_path = dataset_root / "dataset_api.json"
    monkeypatch.setattr(runner, "INVENTORY_PATH", inventory_path)
    monkeypatch.setattr(runner, "DEFAULT_LAM_SOURCE_ROOT", source_root)
    monkeypatch.setattr(runner, "DEFAULT_LAM_COHORT_RECORDS", cohort_records_path)
    monkeypatch.setattr(runner, "DEFAULT_LAM_TABLE3_MANIFEST", table3_manifest_path)
    monkeypatch.setattr(runner, "DEFAULT_LAM_RELEASE_MANIFEST", release_path)
    monkeypatch.setattr(runner, "DEFAULT_LAM_DATASET_API", dataset_api_path)
    monkeypatch.setattr(runner, "LAM_COHORT_RECORDS_SHA256", hashlib.sha256(
        cohort_records_path.read_bytes()
    ).hexdigest())
    monkeypatch.setattr(runner, "LAM_TABLE3_MANIFEST_SHA256", hashlib.sha256(
        table3_manifest_path.read_bytes()
    ).hexdigest())
    monkeypatch.setattr(
        runner, "LAM_TABLE3_MANIFEST_CONTENT_SHA256", table3_manifest["manifest_content_sha256"]
    )
    monkeypatch.setattr(runner, "LAM_RELEASE_MANIFEST_SHA256", hashlib.sha256(
        release_path.read_bytes()
    ).hexdigest())
    monkeypatch.setattr(runner, "LAM_DATASET_API_SHA256", hashlib.sha256(
        dataset_api_path.read_bytes()
    ).hexdigest())
    monkeypatch.setattr(runner, "LAM_DATASET_INVENTORY_SHA256", hashlib.sha256(
        inventory_path.read_bytes()
    ).hexdigest())
    monkeypatch.setattr(runner, "LAM_FORMAL_RELEASE_COUNT", 4)
    monkeypatch.setattr(runner, "LAM_FORMAL_COHORT_COUNT", 4)
    monkeypatch.setattr(runner, "LAM_FORMAL_CATEGORY_COUNT", 3)
    monkeypatch.setattr(runner, "LAM_FORMAL_TIER_COUNTS", {
        "broken": 2, "loads_only": 1, "viable": 1,
    })
    monkeypatch.setattr(
        runner,
        "LAM_CANDIDATE_POOL_SHA256",
        table3_manifest["source"]["candidate_pool_sha256"],
    )
    monkeypatch.setattr(
        runner,
        "LAM_SELECTED_ASSET_KEYS_SHA256",
        table3_manifest["selection"]["selected_asset_keys_sha256"],
    )
    monkeypatch.setattr(runner, "LAM_UPSTREAM_REVISION", "fixture-lam-revision")
    monkeypatch.setattr(runner, "LAM_FORMAL_ARCHIVES", {
        row["name"]: {"bytes": row["bytes"], "sha256": row["sha256"]}
        for row in json.loads(inventory_path.read_text(encoding="utf-8"))["datasets"][0]["archives"]
    })


def test_lam_profile_formal_contract_defaults_and_worker_runtime_support() -> None:
    args = runner.parse_args(["--dataset", "LAM released outputs"])
    runner.validate_run_contract(args)
    assert args.source_root == runner.DEFAULT_LAM_SOURCE_ROOT
    assert args.cohort_manifest == runner.DEFAULT_LAM_COHORT_RECORDS
    assert runner.evaluator_config_for_dataset("LAM released outputs")["selection_algorithm"] == (
        "existing Table 3 manifest records by selection_rank; join completion JSONL by asset_key; "
        "no resampling/reselection"
    )
    for extra in (
        ["--n", "799"], ["--seed", "1"], ["--workers", "3"], ["--limit", "1"],
        ["--asset-timeout-seconds", "119"], ["--no-standard-parser"],
        ["--source-root", "/tmp/not-lam"], ["--cohort-manifest", "/tmp/not-table3.jsonl"],
    ):
        with pytest.raises(ValueError, match="formal"):
            runner.validate_run_contract(
                runner.parse_args(["--dataset", "LAM released outputs", *extra])
            )


def test_lam_loader_joins_completion_jsonl_by_asset_key_and_retains_two_errors(
    tmp_path: Path, monkeypatch: Any
) -> None:
    source_root, cohort_path, inventory_path, table3_manifest, completion_rows = (
        write_lam_table2_fixture(tmp_path)
    )
    patch_lam_fixture_constants(
        monkeypatch, source_root, cohort_path, inventory_path, table3_manifest
    )
    loaded = runner.load_lam_cohort(source_root, cohort_path, formal=True)
    assets = loaded["assets"]

    assert [row["asset_key"] for row in assets] == [
        row["asset_key"] for row in table3_manifest["records"]
    ]
    assert [row["table3_completion_index"] for row in assets] == [4, 3, 2, 1]
    assert sum(row["table3_status"] == "error" for row in assets) == 2
    assert len({row["object_release_id"] for row in assets}) == 3
    assert all("joints" not in row and "source_item" not in row for row in assets)
    by_key = {row["asset_key"]: row for row in completion_rows}
    assert all(
        row["table3_record_sha256"] == runner.canonical_sha256(by_key[row["asset_key"]])
        for row in assets
    )


@pytest.mark.parametrize(
    "mutation",
    (
        "duplicate_jsonl", "missing_jsonl", "jsonl_metadata", "manifest_self_hash",
        "manifest_path_escape", "primary_drift", "archive_drift", "release_drift",
        "dataset_api_drift", "inventory_drift",
    ),
)
def test_lam_loader_rejects_cohort_release_and_provenance_drift(
    tmp_path: Path, monkeypatch: Any, mutation: str
) -> None:
    source_root, cohort_path, inventory_path, table3_manifest, _completion_rows = (
        write_lam_table2_fixture(tmp_path)
    )
    patch_lam_fixture_constants(
        monkeypatch, source_root, cohort_path, inventory_path, table3_manifest
    )
    jsonl_rows = [json.loads(line) for line in cohort_path.read_text().splitlines()]
    if mutation == "duplicate_jsonl":
        jsonl_rows[-1] = dict(jsonl_rows[0])
        cohort_path.write_text("".join(json.dumps(row) + "\n" for row in jsonl_rows))
    elif mutation == "missing_jsonl":
        cohort_path.write_text("".join(json.dumps(row) + "\n" for row in jsonl_rows[:-1]))
    elif mutation == "jsonl_metadata":
        jsonl_rows[0]["category"] = "forged"
        cohort_path.write_text("".join(json.dumps(row) + "\n" for row in jsonl_rows))
    elif mutation == "manifest_self_hash":
        table3_manifest["records"][0]["category"] = "forged"
        (cohort_path.parent / "manifest.json").write_text(json.dumps(table3_manifest))
    elif mutation == "manifest_path_escape":
        table3_manifest["records"][0]["rel_path"] = "../outside"
        table3_manifest["manifest_content_sha256"] = runner.manifest_self_hash(table3_manifest)
        (cohort_path.parent / "manifest.json").write_text(json.dumps(table3_manifest))
    elif mutation == "primary_drift":
        first = table3_manifest["records"][0]
        (source_root / first["rel_path"] / "generated.urdf").write_text("<robot/>")
    elif mutation == "archive_drift":
        (source_root.parent / "broken.tar.gz").write_bytes(b"drift")
    elif mutation == "release_drift":
        (source_root.parent / "manifest.csv").write_text("changed\n")
    elif mutation == "dataset_api_drift":
        (source_root.parent / "dataset_api.json").write_text('{"sha":"changed"}')
    else:
        payload = json.loads(inventory_path.read_text())
        payload["datasets"][0]["status"] = "FORGED"
        inventory_path.write_text(json.dumps(payload))

    with pytest.raises((ValueError, RuntimeError), match={
        "duplicate_jsonl": "duplicate",
        "missing_jsonl": "count|keys|missing",
        "jsonl_metadata": "metadata|category|identity",
        "manifest_self_hash": "self-hash",
        "manifest_path_escape": "relative path|escape",
        "primary_drift": "URDF|drift|SHA-256",
        "archive_drift": "archive",
        "release_drift": "release manifest|columns|SHA-256",
        "dataset_api_drift": "dataset API|SHA-256|revision",
        "inventory_drift": "inventory",
    }[mutation]):
        runner.load_lam_cohort(source_root, cohort_path, formal=False)


@pytest.mark.parametrize("leaf", ("source", "cohort", "manifest", "archive", "package"))
def test_lam_loader_rejects_symlink_components_before_outside_read(
    tmp_path: Path, monkeypatch: Any, leaf: str
) -> None:
    source_root, cohort_path, inventory_path, table3_manifest, _ = write_lam_table2_fixture(tmp_path)
    patch_lam_fixture_constants(
        monkeypatch, source_root, cohort_path, inventory_path, table3_manifest
    )
    if leaf == "source":
        linked = tmp_path / "linked-source"
        linked.symlink_to(source_root, target_is_directory=True)
        source_root = linked
    elif leaf == "cohort":
        target = cohort_path.with_name("records-target.jsonl")
        cohort_path.replace(target)
        cohort_path.symlink_to(target)
    elif leaf == "manifest":
        manifest_path = cohort_path.parent / "manifest.json"
        target = manifest_path.with_name("manifest-target.json")
        manifest_path.replace(target)
        manifest_path.symlink_to(target)
    elif leaf == "archive":
        archive = source_root.parent / "viable.tar.gz"
        target = archive.with_name("viable-target.tar.gz")
        archive.replace(target)
        archive.symlink_to(target)
    else:
        first_path = table3_manifest["records"][0]["rel_path"]
        package = source_root / first_path
        target = tmp_path / "outside-package"
        package.replace(target)
        package.symlink_to(target, target_is_directory=True)

    with pytest.raises(RuntimeError, match="symlink"):
        runner.load_lam_cohort(source_root, cohort_path, formal=False)


def test_lam_run_rejects_source_symlink_before_output_creation(
    tmp_path: Path, monkeypatch: Any
) -> None:
    source_root, cohort_path, _inventory_path, _table3_manifest, _ = write_lam_table2_fixture(
        tmp_path
    )
    linked_source = tmp_path / "linked-lam-source"
    linked_source.symlink_to(source_root, target_is_directory=True)
    output = tmp_path / "runtime" / "must-not-exist"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    args = runner.parse_args([
        "--dataset", "LAM released outputs", "--mode", "smoke",
        "--source-root", str(linked_source), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "1", "--workers", "1", "--no-standard-parser",
    ])

    with pytest.raises(RuntimeError, match="source root.*symlink"):
        runner.run(args)
    assert not output.exists()


def test_lam_manifest_binds_bounded_metadata_package_and_resume_tamper(
    tmp_path: Path, monkeypatch: Any
) -> None:
    source_root, cohort_path, inventory_path, table3_manifest, _ = write_lam_table2_fixture(tmp_path)
    patch_lam_fixture_constants(
        monkeypatch, source_root, cohort_path, inventory_path, table3_manifest
    )
    loaded = runner.load_lam_cohort(source_root, cohort_path, formal=False)
    manifest = runner.build_lam_manifest(
        loaded, requested_n=4, limit=None, standard_parser=False, workers=1,
        protocol_binding=fake_protocol_binding(tmp_path), mode="smoke", command=["fixture"],
    )
    frozen = manifest["records"][1]
    assert manifest["dataset"] == "LAM released outputs"
    assert frozen["asset_id"] == frozen["asset_key"]
    assert frozen["primary_urdf_relative_path"] == "generated.urdf"
    assert frozen["table3_status"] == "error"
    assert "joints" not in json.dumps(manifest)
    runner.validate_manifest_record_source(frozen)
    (Path(frozen["package"]) / "bounded-extra.txt").write_text("package drift\n")
    with pytest.raises(RuntimeError, match="source package drifted"):
        runner.validate_manifest_record_source(frozen)

    record = valid_bound_record(frozen["asset_id"])
    binding = fake_runtime_binding()
    record.update({
        "model_urdf_sha256": frozen["model_urdf_sha256"],
        "primary_urdf_sha256": frozen["primary_urdf_sha256"],
        "primary_urdf_relative_path": "generated.urdf",
        "package_content_manifest_sha256": frozen["package_binding"]["content_manifest_sha256"],
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        **{field: frozen[field] for field in runner.LAM_METADATA_FIELDS},
        "result_origin": "child_attested",
        "job_runtime_binding": binding,
        "worker_runtime_binding": binding,
    })
    job = {
        **frozen,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "runtime_binding": binding,
    }
    runner.validate_frozen_job_result(job, record)
    record["table3_status"] = "completed"
    with pytest.raises(RuntimeError, match="table3_status"):
        runner.validate_frozen_job_result(job, record)


def test_lam_smoke_preserves_rank_order_errors_and_category_macro(
    tmp_path: Path, monkeypatch: Any
) -> None:
    source_root, cohort_path, inventory_path, table3_manifest, _ = write_lam_table2_fixture(tmp_path)
    patch_lam_fixture_constants(
        monkeypatch, source_root, cohort_path, inventory_path, table3_manifest
    )
    output = tmp_path / "runtime" / "table2_lam_smoke"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    argv = [
        "--dataset", "LAM released outputs", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "4", "--workers", "1", "--no-standard-parser",
    ]
    assert runner.run(runner.parse_args(argv)) == output.resolve()
    manifest = json.loads((output / "manifest.json").read_text())
    records = runner.load_jsonl(output / "asset_records.jsonl")
    summary = json.loads((output / "summary.json").read_text())
    expected = [row["asset_key"] for row in table3_manifest["records"]]
    assert [row["asset_id"] for row in manifest["records"]] == expected
    assert [row["asset_id"] for row in records] == expected
    assert sum(row["table3_status"] == "error" for row in records) == 2
    assert summary["category_macro"]["category_field"] == "category"
    assert summary["category_macro"]["category_count"] == 3
    before = (output / "asset_records.jsonl").read_bytes()
    assert runner.run(runner.parse_args([*argv, "--resume"])) == output.resolve()
    assert (output / "asset_records.jsonl").read_bytes() == before


def test_partnet_mobility_profile_formal_contract_and_defaults() -> None:
    """Catch a profile that does not pin the approved PartNet formal inputs."""

    args = runner.parse_args(["--dataset", "PartNet-Mobility"])
    runner.validate_run_contract(args)
    assert args.source_root == runner.DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT
    assert args.cohort_manifest == runner.DEFAULT_PARTNET_MOBILITY_COHORT_MANIFEST
    assert (args.n, args.seed, args.limit, args.workers) == (800, 20260813, None, 4)
    assert args.no_standard_parser is False
    assert runner.evaluator_config_for_dataset("PartNet-Mobility")["selection_algorithm"] == (
        "existing frozen manifest items order; no resampling/reselection"
    )

    for extra in (
        ["--n", "799"],
        ["--workers", "3"],
        ["--limit", "1"],
        ["--asset-timeout-seconds", "119"],
        ["--no-standard-parser"],
        ["--source-root", "/tmp/not-partnet"],
        ["--cohort-manifest", "/tmp/not-frozen.json"],
    ):
        with pytest.raises(ValueError, match="formal"):
            runner.validate_run_contract(
                runner.parse_args(["--dataset", "PartNet-Mobility", *extra])
            )


def test_partnet_loader_preserves_frozen_item_order_and_mobility_urdf_binding(
    tmp_path: Path,
) -> None:
    """Catch PartNet loading that resamples items or maps an ID to model.urdf."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    loaded = runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)

    assert [row["asset_id"] for row in loaded["assets"]] == ["101", "202"]
    assert [row["category"] for row in loaded["assets"]] == ["Chair", "Lamp"]
    assert [row["selection_index"] for row in loaded["assets"]] == [0, 1]
    assert all(row["primary_urdf_relative_path"] == "mobility.urdf" for row in loaded["assets"])
    assert loaded["items_sha256"] == cohort["items_sha256"]
    assert loaded["ordered_selected_ids_sha256"] == cohort["ordered_selected_ids_sha256"]


@pytest.mark.parametrize(
    "mutation, message",
    [
        ("bounding_box", "bounding-box"),
        ("collision_inventory", "collision mesh inventory"),
        ("selection_digest", "selection digest"),
        ("input_identity", "input identity"),
        ("meta_category", "meta.json"),
    ],
)
def test_partnet_loader_binds_authoritative_table4_item_identity(
    tmp_path: Path, mutation: str, message: str
) -> None:
    """Catch a Table 2 cohort accepted after its frozen Table 4 identity drifts."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    if mutation == "bounding_box":
        (source_root / "101" / "bounding_box.json").write_text("{}", encoding="utf-8")
    elif mutation == "collision_inventory":
        (source_root / "101" / "textured_objs" / "collision.obj").write_bytes(b"drift")
    elif mutation == "selection_digest":
        cohort["items"][0]["selection_digest"] = "0" * 64
        cohort["items_sha256"] = runner.canonical_sha256(cohort["items"])
    elif mutation == "input_identity":
        cohort["items"][0]["input_identity_sha256"] = "0" * 64
        cohort["items_sha256"] = runner.canonical_sha256(cohort["items"])
    else:
        (source_root / "101" / "meta.json").write_text(
            json.dumps({"model_cat": "Forged"}), encoding="utf-8"
        )
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")

    with pytest.raises((RuntimeError, ValueError), match=message):
        runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)


def test_partnet_loader_retains_frozen_missing_collision_references(tmp_path: Path) -> None:
    """Catch filtering of a frozen collision reference that was already missing in Table 4."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    item = cohort["items"][0]
    (source_root / "101" / "textured_objs" / "collision.obj").unlink()
    item["collision_mesh_files"] = [{
        "path": "textured_objs/collision.obj", "exists": False,
        "size_bytes": None, "sha256": None,
    }]
    item["collision_mesh_inventory_sha256"] = hashlib.sha256(
        json.dumps(item["collision_mesh_files"], sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    item["input_identity_sha256"] = hashlib.sha256(json.dumps({
        key: item[key]
        for key in (
            "protocol_id", "order", "dataset_id", "selection_digest", "category",
            "movable_dof_count", "range_evaluable_dof_count", "joint_specs_sha256",
            "runtime_identity_sha256", "urdf_sha256", "bounding_box_sha256",
            "collision_mesh_inventory_sha256", "object_bbox_diagonal_m",
            "rest_state_expected", "single_state_expected", "sobol_state_expected",
        )
    }, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    cohort["items_sha256"] = runner.canonical_sha256(cohort["items"])
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")

    loaded = runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)

    assert [row["asset_id"] for row in loaded["assets"]] == ["101", "202"]


@pytest.mark.parametrize("mutation", ["escape", "absolute", "leaf_symlink", "parent_symlink"])
def test_partnet_collision_paths_reject_before_outside_hash_or_read(
    tmp_path: Path, monkeypatch: Any, mutation: str
) -> None:
    """Catch collision references that reach an external target before containment rejects them."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    package = source_root / "101"
    outside = tmp_path / "outside.obj"
    outside.write_bytes(b"outside")
    if mutation in {"escape", "absolute"}:
        primary = package / "mobility.urdf"
        reference = "../outside.obj" if mutation == "escape" else str(outside)
        primary.write_text(
            primary.read_text(encoding="utf-8").replace("textured_objs/collision.obj", reference),
            encoding="utf-8",
        )
        item = cohort["items"][0]
        item["urdf_sha256"] = hashlib.sha256(primary.read_bytes()).hexdigest()
        item["input_identity_sha256"] = hashlib.sha256(json.dumps({
            key: item[key]
            for key in (
                "protocol_id", "order", "dataset_id", "selection_digest", "category",
                "movable_dof_count", "range_evaluable_dof_count", "joint_specs_sha256",
                "runtime_identity_sha256", "urdf_sha256", "bounding_box_sha256",
                "collision_mesh_inventory_sha256", "object_bbox_diagonal_m",
                "rest_state_expected", "single_state_expected", "sobol_state_expected",
            )
        }, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        cohort["items_sha256"] = runner.canonical_sha256(cohort["items"])
        cohort_path.write_text(json.dumps(cohort), encoding="utf-8")
    elif mutation == "leaf_symlink":
        collision = package / "textured_objs" / "collision.obj"
        collision.unlink()
        collision.symlink_to(outside)
    else:
        textured = package / "textured_objs"
        outside_directory = tmp_path / "outside-textured"
        textured.replace(outside_directory)
        textured.symlink_to(outside_directory, target_is_directory=True)

    original_sha256_file = runner.sha256_file
    attempted_outside = False

    def guarded_sha256(path: Path) -> str:
        nonlocal attempted_outside
        if Path(os.path.abspath(path)) == outside or Path(path).is_symlink():
            attempted_outside = True
            raise AssertionError("outside collision target was hashed")
        return original_sha256_file(path)

    monkeypatch.setattr(runner, "sha256_file", guarded_sha256)
    with pytest.raises((RuntimeError, ValueError), match="(collision|reference|symlink|absolute|escape)"):
        runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)
    assert attempted_outside is False


@pytest.mark.parametrize("field", ["bounding_box.json", "meta.json"])
def test_partnet_fixed_identity_paths_reject_symlinks_before_outside_read(
    tmp_path: Path, monkeypatch: Any, field: str
) -> None:
    """Catch bbox/meta symlinks followed before their fixed-path identity check."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    package = source_root / "101"
    outside = tmp_path / f"outside-{field}"
    outside.write_text("outside", encoding="utf-8")
    target = package / field
    target.unlink()
    target.symlink_to(outside)
    original_sha256_file = runner.sha256_file
    original_read_text = Path.read_text
    attempted_outside = False

    def guarded_sha256(path: Path) -> str:
        nonlocal attempted_outside
        if Path(path).is_symlink() or Path(os.path.abspath(path)) == outside:
            attempted_outside = True
            raise AssertionError("outside fixed identity target was hashed")
        return original_sha256_file(path)

    def guarded_read_text(path: Path, *args: Any, **kwargs: Any) -> str:
        nonlocal attempted_outside
        if Path(path).is_symlink() or Path(os.path.abspath(path)) == outside:
            attempted_outside = True
            raise AssertionError("outside fixed identity target was read")
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(runner, "sha256_file", guarded_sha256)
    monkeypatch.setattr(Path, "read_text", guarded_read_text)
    with pytest.raises(RuntimeError, match="symlink"):
        runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)
    assert attempted_outside is False


def test_partnet_run_rejects_source_root_symlink_before_output_or_hash(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch run_locked resolving a PartNet source-root alias before the loader can reject it."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    linked_root = tmp_path / "dataset-link"
    linked_root.symlink_to(source_root, target_is_directory=True)
    output = tmp_path / "runtime" / "should-not-exist"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    original_sha256_file = runner.sha256_file
    source_hash_attempted = False

    def guarded_sha256(path: Path) -> str:
        nonlocal source_hash_attempted
        if str(path).startswith(str(source_root)):
            source_hash_attempted = True
            raise AssertionError("source was hashed before source-root symlink rejection")
        return original_sha256_file(path)

    monkeypatch.setattr(runner, "sha256_file", guarded_sha256)
    args = runner.parse_args([
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(linked_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ])
    with pytest.raises(RuntimeError, match="source root.*symlink"):
        runner.run(args)
    assert source_hash_attempted is False
    assert not output.exists()


@pytest.mark.parametrize("leaf", ["source", "cohort", "archive"])
def test_partnet_loader_rejects_all_identity_leaf_symlinks(tmp_path: Path, leaf: str) -> None:
    """Catch a leaf symlink erased by resolving a formal PartNet identity first."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    if leaf == "source":
        linked = tmp_path / "dataset-link"
        linked.symlink_to(source_root, target_is_directory=True)
        source_root = linked
    elif leaf == "cohort":
        linked = tmp_path / "manifest-link.json"
        linked.symlink_to(cohort_path)
        cohort_path = linked
    else:
        archive = Path(cohort["archive"]["path"])
        target = tmp_path / "archive-target.zip"
        archive.replace(target)
        archive.symlink_to(target)

    with pytest.raises(RuntimeError, match="symlink"):
        runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)


def test_partnet_loader_hashes_archive_once_and_formal_checks_are_controllable(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch repeated archive reads or missing formal inventory/count/category gates."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    archive = Path(cohort["archive"]["path"])
    inventory = {
        "name": "PartNet-Mobility",
        "status": "LOCAL_COMPLETE_PROVENANCE_LIMITED",
        "urdf_root": str(source_root),
        "archive": {"path": str(archive)},
        "source": {"revision": "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"},
        "verification": {"local_ids_listed_by_fixed_hf_revision": 2347},
    }
    monkeypatch.setattr(runner, "DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT", source_root)
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_COHORT_MANIFEST_SHA256", hashlib.sha256(cohort_path.read_bytes()).hexdigest())
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_FORMAL_RELEASE_COUNT", 2)
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_FORMAL_CATEGORY_COUNT", 2)
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_CANDIDATE_POOL_SHA256", cohort["candidate_pool_identity_sha256"])
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_ITEMS_SHA256", cohort["items_sha256"])
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_SELECTED_IDS_SHA256", cohort["ordered_selected_ids_sha256"])
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_ARCHIVE_BYTES", archive.stat().st_size)
    monkeypatch.setattr(runner, "PARTNET_MOBILITY_ARCHIVE_SHA256", cohort["archive"]["sha256"])
    monkeypatch.setattr(runner, "DEFAULT_N", 2)
    monkeypatch.setattr(runner, "_partnet_mobility_inventory_entry", lambda: (inventory, "i" * 64))
    original_sha256_file = runner.sha256_file
    archive_hash_calls = 0

    def tracked_sha256_file(path: Path) -> str:
        nonlocal archive_hash_calls
        if path == archive:
            archive_hash_calls += 1
        return original_sha256_file(path)

    monkeypatch.setattr(runner, "sha256_file", tracked_sha256_file)
    loaded = runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=True)

    assert loaded["inventory"] == inventory
    assert archive_hash_calls == 1


@pytest.mark.parametrize("failure", ["missing", "wrong", "import"])
def test_formal_parent_parser_preflight_fails_before_output_creation(
    tmp_path: Path, monkeypatch: Any, failure: str
) -> None:
    """Catch formal parser failures being converted into per-asset records."""

    args = runner.parse_args(["--dataset", "PartNet-Mobility"])
    created = False

    def unexpected_output(*_args: Any, **_kwargs: Any) -> Path:
        nonlocal created
        created = True
        return tmp_path / "unexpected"

    if failure == "missing":
        def missing_version(_name: str) -> str:
            raise importlib.metadata.PackageNotFoundError
        monkeypatch.setattr(runner.importlib.metadata, "version", missing_version)
    elif failure == "wrong":
        monkeypatch.setattr(runner.importlib.metadata, "version", lambda _name: "9.9.9")
    else:
        monkeypatch.setattr(runner.importlib.metadata, "version", lambda _name: "0.0.22")
        monkeypatch.setattr(runner.importlib, "import_module", lambda _name: (_ for _ in ()).throw(ImportError("broken")))
    monkeypatch.setattr(runner, "prepare_output", unexpected_output)

    with pytest.raises(runner.FatalRuntimeBindingError, match="formal standard parser"):
        runner.run(args)
    assert created is False


@pytest.mark.parametrize(
    "mutation, message",
    [
        ("top_level_hash", "items SHA-256"),
        ("order", "order"),
        ("duplicate", "duplicate"),
        ("nonnumeric", "numeric"),
        ("path", "package"),
        ("symlink", "symlink"),
        ("missing", "mobility.urdf"),
        ("primary_hash", "primary URDF SHA-256"),
        ("pool", "candidate pool"),
        ("archive", "archive"),
    ],
)
def test_partnet_loader_fail_closes_tampered_frozen_or_live_identity(
    tmp_path: Path, mutation: str, message: str
) -> None:
    """Catch any PartNet manifest, package, live-pool, or archive identity drift."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    if mutation == "top_level_hash":
        cohort["items_sha256"] = "0" * 64
    elif mutation == "order":
        cohort["items"][1]["order"] = 0
    elif mutation == "duplicate":
        cohort["items"][1]["dataset_id"] = "101"
    elif mutation == "nonnumeric":
        cohort["items"][0]["dataset_id"] = "abc"
    elif mutation == "path":
        cohort["items"][0]["package"] = "../outside"
    elif mutation == "symlink":
        package = source_root / "101"
        primary = package / "mobility.urdf"
        target = tmp_path / "outside.urdf"
        target.write_bytes(primary.read_bytes())
        primary.unlink()
        primary.symlink_to(target)
    elif mutation == "missing":
        (source_root / "101" / "mobility.urdf").unlink()
    elif mutation == "primary_hash":
        (source_root / "101" / "mobility.urdf").write_text("<robot/>", encoding="utf-8")
    elif mutation == "pool":
        cohort["candidate_pool_identity_sha256"] = "0" * 64
    else:
        Path(cohort["archive"]["path"]).write_bytes(b"drift")
    if mutation in {"order", "duplicate", "nonnumeric", "path"}:
        cohort["items_sha256"] = runner.canonical_sha256(cohort["items"])
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")

    with pytest.raises((RuntimeError, ValueError), match=message):
        runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)


def test_partnet_category_macro_is_unweighted_over_item_category() -> None:
    """Catch a PartNet category macro weighted by category population."""

    all_pass = {name: {"pass": True} for name in runner.METRIC_NAMES}
    all_fail = {name: {"pass": False} for name in runner.METRIC_NAMES}
    summary = runner.aggregate_records(
        [
            {"asset_id": "101", "category": "Chair", "metrics": all_pass},
            {"asset_id": "102", "category": "Chair", "metrics": all_fail},
            {"asset_id": "201", "category": "Lamp", "metrics": all_pass},
        ],
        expected_n=3,
        category_field="category",
    )
    assert summary["metrics"]["parse_rate"] == {"passed": 2, "denominator": 3, "rate": pytest.approx(2 / 3)}
    assert summary["category_macro"]["denominator_policy"] == (
        "unweighted mean of per-category asset rates; all frozen assets and failures retained"
    )
    assert summary["category_macro"]["metrics"]["parse_rate"]["rate"] == pytest.approx(0.75)


def test_partnet_child_resume_metadata_and_source_binding_fail_closed(tmp_path: Path) -> None:
    """Catch child/resume metadata forgery or a changed frozen PartNet package."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    loaded = runner.load_partnet_mobility_cohort(source_root, cohort_path, formal=False)
    manifest = runner.build_partnet_mobility_manifest(
        loaded,
        requested_n=2,
        limit=None,
        standard_parser=False,
        workers=1,
        protocol_binding=fake_protocol_binding(tmp_path),
        mode="smoke",
    )
    frozen = manifest["records"][0]
    binding = fake_runtime_binding()
    record = valid_bound_record(frozen["asset_id"])
    record.update({
        "model_urdf_sha256": frozen["model_urdf_sha256"],
        "primary_urdf_sha256": frozen["primary_urdf_sha256"],
        "primary_urdf_relative_path": "mobility.urdf",
        "package_content_manifest_sha256": frozen["package_binding"]["content_manifest_sha256"],
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        **{field: frozen[field] for field in runner.PARTNET_MOBILITY_METADATA_FIELDS},
        "result_origin": "child_attested",
        "job_runtime_binding": binding,
        "worker_runtime_binding": binding,
    })
    job = {
        **frozen,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "runtime_binding": binding,
    }
    runner.validate_frozen_job_result(job, record)
    runner.validate_manifest_record_source(frozen)

    record["category"] = "forged"
    with pytest.raises(RuntimeError, match="category"):
        runner.validate_frozen_job_result(job, record)
    (Path(frozen["package"]) / "mobility.urdf").write_text("<robot/>", encoding="utf-8")
    with pytest.raises(RuntimeError, match="source package drifted"):
        runner.validate_manifest_record_source(frozen)


def test_partnet_smoke_subprocess_resume_preserves_order_category_and_selection_provenance(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch PartNet smoke/resume routing that changes frozen records or claims RNG selection."""

    source_root, cohort_path, cohort = write_partnet_mobility_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_partnet_smoke"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    argv = [
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ]
    assert runner.run(runner.parse_args(argv)) == output.resolve()
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    before = (output / "asset_records.jsonl").read_bytes()
    records = [json.loads(line) for line in before.decode().splitlines()]

    assert [row["asset_id"] for row in records] == ["101", "202"]
    assert [row["category"] for row in records] == ["Chair", "Lamp"]
    assert len({row["job_runtime_binding"]["run_token"] for row in records}) == 2
    assert manifest["selection"]["selection_policy"] == cohort["selection_policy"]
    assert "seed" not in manifest["selection"]
    assert cohort["selection_policy"]["salt"] in (output / "summary.md").read_text(encoding="utf-8")

    assert runner.run(runner.parse_args([*argv, "--resume"])) == output.resolve()
    assert (output / "asset_records.jsonl").read_bytes() == before


def test_protocol_snapshot_allows_partial_and_completed_resume_after_live_report_change(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch resume depending on a mutable report instead of the per-run frozen protocol."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_partnet_protocol_snapshot"
    live_protocol = tmp_path / "live_protocol.md"
    frozen_bytes = b"# frozen table 2 protocol\n"
    live_protocol.write_bytes(frozen_bytes)
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_PATH", live_protocol)
    argv = [
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ]

    assert runner.run(runner.parse_args(argv)) == output.resolve()
    snapshot = output / "protocol_snapshot.md"
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    evaluation = manifest["evaluation"]
    frozen_sha256 = hashlib.sha256(frozen_bytes).hexdigest()
    before = (output / "asset_records.jsonl").read_bytes()

    assert snapshot.read_bytes() == frozen_bytes
    assert evaluation["protocol_source_path"] == str(live_protocol.resolve())
    assert evaluation["protocol_source_sha256_at_freeze"] == frozen_sha256
    assert evaluation["protocol_path"] == str(snapshot.resolve())
    assert evaluation["protocol_sha256"] == frozen_sha256

    first_record = before.splitlines(keepends=True)[0]
    (output / "asset_records.jsonl").write_bytes(first_record)
    (output / "summary.json").unlink()
    (output / "summary.md").unlink()
    runner.atomic_write_json(output / "checkpoint.json", {
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed": 1,
        "n_eval": 2,
        "remaining": 1,
        "updated_at": runner.utc_now(),
    })
    live_protocol.unlink()
    assert runner.run(runner.parse_args([*argv, "--resume"])) == output.resolve()
    after_partial_resume = (output / "asset_records.jsonl").read_bytes()
    assert after_partial_resume.splitlines(keepends=True)[0] == first_record
    assert len(after_partial_resume.splitlines()) == 2
    assert snapshot.read_bytes() == frozen_bytes

    live_protocol.write_bytes(b"# post-run results update\n")
    assert runner.run(runner.parse_args([*argv, "--resume"])) == output.resolve()
    assert (output / "asset_records.jsonl").read_bytes() == after_partial_resume


@pytest.mark.parametrize("mutation", ["missing", "symlink", "hash_drift", "outside"])
def test_resume_rejects_invalid_protocol_snapshot(
    tmp_path: Path, monkeypatch: Any, mutation: str
) -> None:
    """Catch silent migration or use of a missing, replaced, escaped, or changed snapshot."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    output = tmp_path / "runtime" / f"table2_partnet_protocol_{mutation}"
    live_protocol = tmp_path / "live_protocol.md"
    live_protocol.write_bytes(b"# frozen table 2 protocol\n")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_PATH", live_protocol)
    argv = [
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ]
    runner.run(runner.parse_args(argv))
    snapshot = output / "protocol_snapshot.md"
    if mutation == "missing":
        snapshot.unlink()
    elif mutation == "symlink":
        outside = tmp_path / "outside_snapshot.md"
        outside.write_bytes(snapshot.read_bytes())
        snapshot.unlink()
        snapshot.symlink_to(outside)
    elif mutation == "hash_drift":
        snapshot.write_bytes(b"# changed frozen protocol\n")
    else:
        outside = tmp_path / "outside" / "protocol_snapshot.md"
        outside.parent.mkdir()
        outside.write_bytes(snapshot.read_bytes())
        manifest_path = output / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["evaluation"]["protocol_path"] = str(outside)
        manifest["manifest_content_sha256"] = runner.manifest_self_hash(manifest)
        runner.atomic_write_json(manifest_path, manifest)

    with pytest.raises(RuntimeError, match="protocol snapshot"):
        runner.run(runner.parse_args([*argv, "--resume"]))


def test_run_revalidates_protocol_snapshot_after_scheduler_before_completion(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch snapshot drift after the final child being published as a completed run."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_partnet_protocol_final_check"
    live_protocol = tmp_path / "live_protocol.md"
    live_protocol.write_bytes(b"# frozen table 2 protocol\n")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner, "PROTOCOL_PATH", live_protocol)
    original_scheduler = runner.execute_killable_jobs

    def mutate_snapshot_after_scheduler(*args: Any, **kwargs: Any) -> list[dict[str, Any]]:
        records = original_scheduler(*args, **kwargs)
        (output / "protocol_snapshot.md").write_bytes(b"# changed after scheduler\n")
        return records

    monkeypatch.setattr(runner, "execute_killable_jobs", mutate_snapshot_after_scheduler)
    argv = [
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ]

    with pytest.raises(RuntimeError, match="protocol snapshot"):
        runner.run(runner.parse_args(argv))
    assert not (output / "summary.json").exists()
    checkpoint = json.loads((output / "checkpoint.json").read_text(encoding="utf-8"))
    assert checkpoint.get("state") != "complete"


def test_snapshot_only_crash_state_requires_a_new_output_and_cannot_resume(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch a pre-manifest crash being silently reused or migrated on resume."""

    source_root, cohort_path, _cohort = write_partnet_mobility_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_partnet_snapshot_only"
    output.mkdir(parents=True)
    (output / "protocol_snapshot.md").write_bytes(b"orphaned snapshot\n")
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    argv = [
        "--dataset", "PartNet-Mobility", "--mode", "smoke",
        "--source-root", str(source_root), "--cohort-manifest", str(cohort_path),
        "--output", str(output), "--n", "2", "--workers", "1", "--no-standard-parser",
    ]

    with pytest.raises(FileExistsError, match="unique path"):
        runner.run(runner.parse_args(argv))
    with pytest.raises(RuntimeError, match="resume manifest"):
        runner.run(runner.parse_args([*argv, "--resume"]))


def test_child_runtime_binding_hashes_snapshot_and_ignores_live_protocol(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch parent/child attestation silently returning to the mutable live report."""

    output = tmp_path / "output"
    output.mkdir()
    snapshot = output / "protocol_snapshot.md"
    snapshot.write_bytes(b"frozen protocol\n")
    live_protocol = tmp_path / "live_protocol.md"
    live_protocol.write_bytes(b"live protocol\n")
    monkeypatch.setattr(runner, "PROTOCOL_PATH", live_protocol)
    job = {
        "dataset": "PartNet-Mobility",
        "run_token": "e" * 32,
        "run_standard_parser": False,
        "workers": 1,
        "output_root": str(output.resolve()),
        "runtime_binding": {"protocol_path": str(snapshot.resolve())},
    }
    binding = runner.current_worker_runtime_binding(job)
    job["runtime_binding"] = binding

    assert binding["protocol_path"] == str(snapshot.resolve())
    assert binding["protocol_sha256"] == hashlib.sha256(snapshot.read_bytes()).hexdigest()
    live_protocol.write_bytes(b"changed live protocol\n")
    assert runner.validate_child_runtime_binding(job) == binding

    snapshot.write_bytes(b"changed frozen protocol\n")
    with pytest.raises(runner.FatalRuntimeBindingError, match="runtime binding drift"):
        runner.validate_child_runtime_binding(job)


@pytest.mark.parametrize("mutation", ["missing", "symlink", "fifo", "directory", "outside"])
def test_child_runtime_binding_rejects_invalid_snapshot_path(
    tmp_path: Path, mutation: str
) -> None:
    """Catch a worker accepting a non-regular or non-output-owned protocol snapshot."""

    output = tmp_path / "output"
    output.mkdir()
    snapshot = output / "protocol_snapshot.md"
    if mutation == "missing":
        candidate = snapshot
    elif mutation == "symlink":
        outside = tmp_path / "outside_snapshot.md"
        outside.write_bytes(b"protocol\n")
        snapshot.symlink_to(outside)
        candidate = snapshot
    elif mutation == "fifo":
        os.mkfifo(snapshot)
        candidate = snapshot
    elif mutation == "directory":
        snapshot.mkdir()
        candidate = snapshot
    else:
        candidate = tmp_path / "outside" / "protocol_snapshot.md"
        candidate.parent.mkdir()
        candidate.write_bytes(b"protocol\n")
    job = {
        "dataset": "PartNet-Mobility",
        "run_token": "e" * 32,
        "run_standard_parser": False,
        "workers": 1,
        "output_root": str(output.resolve()),
        "runtime_binding": {"protocol_path": str(candidate.resolve(strict=False))},
    }

    with pytest.raises(runner.FatalRuntimeBindingError, match="protocol snapshot"):
        runner.current_worker_runtime_binding(job)


def test_child_revalidates_snapshot_after_asset_audit(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """Catch a snapshot replacement during a child audit escaping worker attestation."""

    package = write_asset(
        tmp_path / "package_fixture",
        f'<robot name="asset">{valid_link()}</robot>',
    )
    binding = runner.package_binding(package)
    job = {
        "asset_id": "asset",
        "package": str(package),
        "model_urdf_sha256": next(
            row["sha256"] for row in binding["files"] if row["path"] == "model.urdf"
        ),
        "package_binding": binding,
        "manifest_content_sha256": "c" * 64,
        "dataset": "Articraft-10K",
        "run_token": "e" * 32,
        "run_standard_parser": False,
        "workers": 1,
    }
    output = tmp_path / "child_output"
    bind_current_runtime(job, output)
    original_audit = runner.audit_asset_package

    def mutate_snapshot_after_audit(*args: Any, **kwargs: Any) -> dict[str, Any]:
        record = original_audit(*args, **kwargs)
        (output / "protocol_snapshot.md").write_bytes(b"changed during child audit\n")
        return record

    monkeypatch.setattr(runner, "audit_asset_package", mutate_snapshot_after_audit)
    with pytest.raises(runner.FatalRuntimeBindingError, match="runtime binding drift"):
        runner.audit_frozen_job(job)


def test_artiverse_loader_preserves_exact_manifest_order_and_package_identity(
    tmp_path: Path,
) -> None:
    artiverse, cohort_path, cohort = write_artiverse_fixture(tmp_path)

    loaded = runner.load_artiverse_cohort(artiverse, cohort_path, formal=False)

    assert [row["asset_id"] for row in loaded["assets"]] == [
        row["asset_id"] for row in cohort["assets"]
    ]
    assert [row["selection_rank"] for row in loaded["assets"]] == [1, 2]
    for row in loaded["assets"]:
        assert Path(row["package"]).name == "urdf_w_collider"
        assert row["primary_urdf_relative_path"] == f'{row["model_id"]}.urdf'
    assert loaded["cohort_manifest_sha256"] == hashlib.sha256(
        cohort_path.read_bytes()
    ).hexdigest()
    assert loaded["release_manifest_sha256"] == cohort["release_manifest_sha256"]


def test_audit_accepts_explicit_full_asset_id_and_non_model_primary_urdf(
    tmp_path: Path,
) -> None:
    package = tmp_path / "urdf_w_collider"
    package.mkdir()
    (package / "identifier.urdf").write_text(
        f'<robot name="valid">{valid_link()}</robot>\n', encoding="utf-8"
    )

    record = runner.audit_asset_package(
        package,
        run_standard_parser=False,
        asset_id="data/category/source/identifier",
        primary_urdf_relative_path="identifier.urdf",
    )

    assert record["asset_id"] == "data/category/source/identifier"
    assert record["primary_urdf_relative_path"] == "identifier.urdf"
    assert metric_states(record) == {name: True for name in runner.METRIC_NAMES}


@pytest.mark.parametrize(
    "mutation", ["duplicate", "escape", "rank", "hash", "order", "urdf_count"]
)
def test_artiverse_loader_rejects_invalid_frozen_cohort(
    tmp_path: Path,
    mutation: str,
) -> None:
    artiverse, cohort_path, cohort = write_artiverse_fixture(tmp_path)
    if mutation == "duplicate":
        cohort["assets"][1] = {**cohort["assets"][0], "selection_rank": 2}
    elif mutation == "escape":
        cohort["assets"][0].update({
            "asset_id": "../outside",
            "manifest_root": "../outside",
        })
    elif mutation == "rank":
        cohort["assets"][0]["selection_rank"] = 2
    elif mutation == "hash":
        cohort["assets"][0]["selection_hash"] = "f" * 64
    elif mutation == "order":
        cohort["assets"].reverse()
        for rank, row in enumerate(cohort["assets"], start=1):
            row["selection_rank"] = rank
    else:
        row = cohort["assets"][0]
        package = artiverse / row["manifest_root"] / "urdf_w_collider"
        (package / "extra.urdf").write_text('<robot name="extra"><link name="x"/></robot>')
    cohort_path.write_text(json.dumps(cohort), encoding="utf-8")

    with pytest.raises((ValueError, RuntimeError), match={
        "duplicate": "duplicate",
        "escape": "manifest root",
        "rank": "selection rank",
        "hash": "selection hash",
        "order": "selection order",
        "urdf_count": "exactly one",
    }[mutation]):
        runner.load_artiverse_cohort(artiverse, cohort_path, formal=False)


def test_artiverse_manifest_binds_cohort_release_package_and_original_metadata(
    tmp_path: Path,
) -> None:
    artiverse, cohort_path, _cohort = write_artiverse_fixture(tmp_path)
    loaded = runner.load_artiverse_cohort(artiverse, cohort_path, formal=False)

    manifest = runner.build_artiverse_manifest(
        loaded,
        requested_n=2,
        limit=None,
        standard_parser=False,
        workers=1,
        protocol_binding=fake_protocol_binding(tmp_path),
        mode="smoke",
        command=["fixture"],
    )

    assert manifest["dataset"] == "Artiverse"
    assert manifest["source"]["cohort_manifest_sha256"] == loaded["cohort_manifest_sha256"]
    assert manifest["source"]["release_manifest_sha256"] == loaded["release_manifest_sha256"]
    assert manifest["selection"]["algorithm"] == "existing Table 1 manifest order; no resampling/reselection"
    first = manifest["records"][0]
    for field in (
        "raw_category", "source", "model_id", "selection_rank", "selection_hash",
        "primary_urdf_relative_path", "primary_urdf_sha256", "package_binding",
    ):
        assert first[field]
    assert first["asset_id"].startswith("data/")


def test_resume_source_validation_detects_artiverse_package_drift(tmp_path: Path) -> None:
    artiverse, cohort_path, _cohort = write_artiverse_fixture(tmp_path)
    loaded = runner.load_artiverse_cohort(artiverse, cohort_path, formal=False)
    manifest = runner.build_artiverse_manifest(
        loaded,
        requested_n=2,
        limit=None,
        standard_parser=False,
        workers=1,
        protocol_binding=fake_protocol_binding(tmp_path),
        mode="smoke",
    )
    frozen = manifest["records"][0]
    runner.validate_manifest_record_source(frozen)
    package = Path(frozen["package"])
    (package / frozen["primary_urdf_relative_path"]).write_text("<robot>", encoding="utf-8")

    with pytest.raises(RuntimeError, match="source package drifted"):
        runner.validate_manifest_record_source(frozen)


def test_category_macro_uses_unweighted_raw_category_rates() -> None:
    all_pass = {name: {"pass": True} for name in runner.METRIC_NAMES}
    all_fail = {name: {"pass": False} for name in runner.METRIC_NAMES}
    rows = [
        {"asset_id": "a1", "raw_category": "a", "metrics": all_pass},
        {"asset_id": "a2", "raw_category": "a", "metrics": all_fail},
        {"asset_id": "b1", "raw_category": "b", "metrics": all_pass},
    ]

    summary = runner.aggregate_records(rows, expected_n=3, category_field="raw_category")

    category_macro = summary["category_macro"]
    assert category_macro["state"] == "evaluated"
    assert category_macro["category_count"] == 2
    assert category_macro["denominator_policy"] == (
        "unweighted mean of per-raw_category asset rates; all frozen assets and failures retained"
    )
    assert category_macro["metrics"]["parse_rate"]["rate"] == pytest.approx(0.75)
    assert category_macro["categories"]["a"]["metrics"]["parse_rate"]["rate"] == 0.5
    assert category_macro["categories"]["b"]["metrics"]["parse_rate"]["rate"] == 1.0


def test_artiverse_smoke_routes_exact_manifest_order_end_to_end(
    tmp_path: Path, monkeypatch: Any
) -> None:
    artiverse, cohort_path, cohort = write_artiverse_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_artiverse_smoke"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    args = runner.parse_args([
        "--dataset", "Artiverse",
        "--mode", "smoke",
        "--source-root", str(artiverse),
        "--cohort-manifest", str(cohort_path),
        "--output", str(output),
        "--n", "2",
        "--workers", "1",
        "--no-standard-parser",
    ])

    assert runner.run(args) == output.resolve()
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    expected_ids = [row["asset_id"] for row in cohort["assets"]]
    assert manifest["dataset"] == "Artiverse"
    assert [row["asset_id"] for row in manifest["records"]] == expected_ids
    assert [row["asset_id"] for row in records] == expected_ids
    assert summary["dataset"] == "Artiverse"
    assert summary["n_eval"] == 2
    assert summary["category_macro"]["category_count"] == 2


def test_child_result_must_match_frozen_artiverse_metadata() -> None:
    record = valid_bound_record("data/category/source/model")
    job = {
        "asset_id": record["asset_id"],
        "model_urdf_sha256": "a" * 64,
        "primary_urdf_sha256": "a" * 64,
        "primary_urdf_relative_path": "model.urdf",
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "raw_category": "category",
        "source": "source",
        "model_id": "model",
        "manifest_root": "data/category/source/model",
        "chunk_archive": "fixture.tar.gz",
        "selection_rank": 1,
        "selection_hash": "d" * 64,
        "runtime_binding": fake_runtime_binding(),
    }
    record.update({
        "primary_urdf_sha256": "a" * 64,
        "primary_urdf_relative_path": "model.urdf",
        **{field: job[field] for field in runner.ARTIVERSE_METADATA_FIELDS},
        "result_origin": "child_attested",
        "job_runtime_binding": job["runtime_binding"],
        "worker_runtime_binding": job["runtime_binding"],
    })
    record["raw_category"] = "tampered"

    with pytest.raises(RuntimeError, match="raw_category"):
        runner.validate_frozen_job_result(job, record)


def test_child_result_rejects_replayed_run_token() -> None:
    record = valid_bound_record()
    expected_runtime = fake_runtime_binding("c" * 32)
    job = {
        "asset_id": "asset",
        "model_urdf_sha256": "a" * 64,
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "runtime_binding": expected_runtime,
    }
    record["worker_runtime_binding"] = {**expected_runtime, "run_token": "d" * 32}

    with pytest.raises(RuntimeError, match="worker runtime binding"):
        runner.validate_frozen_job_result(job, record)


@pytest.mark.parametrize("drift", ["evaluator", "protocol", "config", "environment"])
def test_child_runtime_preflight_rejects_post_freeze_drift(
    tmp_path: Path, monkeypatch: Any, drift: str
) -> None:
    job = {
        "dataset": "Artiverse",
        "run_token": "e" * 32,
        "run_standard_parser": False,
        "workers": 1,
    }
    bind_current_runtime(job, tmp_path / "runtime_binding_output")
    if drift == "evaluator":
        job["runtime_binding"] = {
            **job["runtime_binding"],
            "evaluator_sha256": "0" * 64,
        }
    elif drift == "protocol":
        job["runtime_binding"] = {
            **job["runtime_binding"],
            "protocol_sha256": "0" * 64,
        }
    elif drift == "config":
        original_config = runner.evaluator_config_for_dataset

        def drifted_config(dataset: str) -> dict[str, Any]:
            return {**original_config(dataset), "changed_after_freeze": True}

        monkeypatch.setattr(runner, "evaluator_config_for_dataset", drifted_config)
    else:
        original = runner.environment_metadata

        def drifted_environment(enabled: bool, workers: int) -> dict[str, Any]:
            return {**original(enabled, workers), "python": "changed-after-freeze"}

        monkeypatch.setattr(runner, "environment_metadata", drifted_environment)

    with pytest.raises(RuntimeError, match="runtime binding drift"):
        runner.validate_child_runtime_binding(job)


def test_parent_synthesized_failure_has_frozen_job_binding_only(tmp_path: Path) -> None:
    binding = fake_runtime_binding()
    job = {
        "asset_id": "asset",
        "package": str(tmp_path / "asset"),
        "model_urdf_sha256": "a" * 64,
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "runtime_binding": binding,
    }

    record = runner.bound_job_failure(job, "child_spawn_failed")

    assert record["result_origin"] == "parent_synthesized"
    assert record["job_runtime_binding"] == binding
    assert "worker_runtime_binding" not in record


@pytest.mark.parametrize("mutation", ["missing_job", "tampered_job", "tampered_worker"])
def test_resume_rejects_invalid_runtime_provenance(mutation: str) -> None:
    binding = fake_runtime_binding()
    record = valid_bound_record()
    record.update({
        "result_origin": "child_attested",
        "job_runtime_binding": binding,
        "worker_runtime_binding": binding,
    })
    if mutation == "missing_job":
        del record["job_runtime_binding"]
    elif mutation == "tampered_job":
        record["job_runtime_binding"] = {
            **binding,
            "environment": {"python": "tampered"},
        }
    else:
        record["worker_runtime_binding"] = {**binding, "run_token": "f" * 32}

    with pytest.raises(RuntimeError, match="runtime provenance"):
        runner.validate_resume_record(
            record,
            "asset",
            "a" * 64,
            "b" * 64,
            "c" * 64,
            expected_runtime_evaluation=fake_runtime_evaluation(binding),
        )


def test_parent_synthesized_failure_runtime_provenance_resumes(tmp_path: Path) -> None:
    binding = fake_runtime_binding()
    job = {
        "asset_id": "asset",
        "package": str(tmp_path / "asset"),
        "model_urdf_sha256": "a" * 64,
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "runtime_binding": binding,
    }
    record = runner.bound_job_failure(job, "asset_timeout", status="timeout")

    runner.validate_resume_record(
        record,
        "asset",
        "a" * 64,
        "b" * 64,
        "c" * 64,
        expected_runtime_evaluation=fake_runtime_evaluation(binding),
    )


def test_child_source_failure_is_runtime_attested(tmp_path: Path) -> None:
    job = {
        "asset_id": "asset",
        "package": str(tmp_path / "missing-package"),
        "model_urdf_sha256": "a" * 64,
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "dataset": "Artiverse",
        "run_token": "a" * 32,
        "run_standard_parser": False,
        "workers": 1,
    }
    bind_current_runtime(job, tmp_path / "child_source_output")

    record = runner.audit_frozen_job(job)

    assert record["status"] == "error"
    assert record["result_origin"] == "child_attested"
    assert record["job_runtime_binding"] == job["runtime_binding"]
    assert record["worker_runtime_binding"] == job["runtime_binding"]


def test_resume_rejects_duplicate_runtime_tokens(tmp_path: Path, monkeypatch: Any) -> None:
    artiverse, cohort_path, _cohort = write_artiverse_fixture(tmp_path)
    output = tmp_path / "runtime" / "table2_artiverse_resume_token"
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    argv = [
        "--dataset", "Artiverse",
        "--mode", "smoke",
        "--source-root", str(artiverse),
        "--cohort-manifest", str(cohort_path),
        "--output", str(output),
        "--n", "2",
        "--workers", "1",
        "--no-standard-parser",
    ]
    runner.run(runner.parse_args(argv))
    records = runner.load_jsonl(output / "asset_records.jsonl")
    duplicate_token = records[0]["job_runtime_binding"]["run_token"]
    for field in ("job_runtime_binding", "worker_runtime_binding"):
        records[1][field] = {**records[1][field], "run_token": duplicate_token}
    runner.atomic_write_jsonl(output / "asset_records.jsonl", records)

    with pytest.raises(RuntimeError, match="duplicate.*run token"):
        runner.run(runner.parse_args([*argv, "--resume"]))


def test_runtime_binding_drift_aborts_scheduler(tmp_path: Path) -> None:
    job = {
        "asset_id": "asset",
        "dataset": "Artiverse",
        "run_token": "a" * 32,
        "run_standard_parser": False,
        "workers": 1,
    }
    binding = bind_current_runtime(job, tmp_path)
    job["runtime_binding"] = {
        **binding,
        "evaluator_sha256": "0" * 64,
    }

    with pytest.raises(runner.FatalRuntimeBindingError, match="runtime binding"):
        runner.execute_killable_jobs(
            [job],
            worker_scratch=tmp_path / "workers",
            timeout_seconds=2,
            max_workers=1,
            on_result=lambda record: None,
            timeout_factory=lambda item: {"asset_id": item["asset_id"], "status": "timeout"},
        )
    assert not (tmp_path / "workers").exists()


def test_subprocess_jobs_timeout_without_head_of_line_blocking(tmp_path: Path) -> None:
    completed: list[str] = []
    process_group_token = f"table2-process-group-{os.getpid()}-{time.time_ns()}"
    jobs = [
        {
            "asset_id": "slow",
            "internal_test_action": "spawn_descendant_sleep",
            "sleep": 0.60,
            "process_group_token": process_group_token,
        },
        {
            "asset_id": "fast",
            "internal_test_action": "delayed_echo_environment",
            "sleep": 0.02,
        },
    ]
    records = runner.execute_killable_jobs(
        jobs,
        worker_scratch=tmp_path / "workers",
        timeout_seconds=0.30,
        max_workers=2,
        on_result=lambda record: completed.append(record["asset_id"]),
        timeout_factory=lambda job: {"asset_id": job["asset_id"], "status": "timeout"},
    )
    assert completed == ["fast", "slow"]
    assert [record["status"] for record in records] == ["timeout", "completed"]
    assert records[0]["worker_evidence"]["termination"] in {"SIGTERM", "SIGKILL"}
    assert records[1]["worker_evidence"]["pid"] != os.getpid()
    assert records[1]["thread_environment"] == runner.CHILD_THREAD_ENVIRONMENT
    assert not (tmp_path / "workers").exists()
    assert not any(
        process_group_token.encode() in path.read_bytes()
        for path in Path("/proc").glob("[0-9]*/cmdline")
        if path.is_file()
    )


def test_subprocess_child_large_stderr_cannot_block_result(tmp_path: Path) -> None:
    records = runner.execute_killable_jobs(
        [
            {
                "asset_id": "noisy",
                "internal_test_action": "large_stderr",
                "stderr_bytes": 1024 * 1024 + 123,
            }
        ],
        worker_scratch=tmp_path / "workers",
        timeout_seconds=1.0,
        max_workers=1,
        on_result=lambda record: None,
        timeout_factory=lambda job: {"asset_id": job["asset_id"], "status": "timeout"},
    )
    record = records[0]
    assert record["status"] == "completed"
    evidence = record["worker_evidence"]
    assert evidence["stderr_bytes"] > 1024 * 1024
    assert evidence["stderr_truncated"] is True
    assert len(evidence["stderr_tail"]) <= 4000
    assert not (tmp_path / "workers").exists()


@pytest.mark.parametrize("mutation", ["wrong_id", "wrong_hash", "missing_metric", "strict"])
def test_scheduler_rejects_invalid_successful_child_results(
    tmp_path: Path, mutation: str
) -> None:
    record = valid_bound_record()
    job = {
        "asset_id": "asset",
        "internal_test_action": "emit_result",
        "model_urdf_sha256": "a" * 64,
        "package_binding": {"content_manifest_sha256": "b" * 64},
        "manifest_content_sha256": "c" * 64,
        "dataset": "Artiverse",
        "run_token": "a" * 32,
        "run_standard_parser": False,
        "workers": 1,
    }
    bind_current_runtime(job, tmp_path)
    record.update({
        "result_origin": "child_attested",
        "job_runtime_binding": job["runtime_binding"],
        "worker_runtime_binding": job["runtime_binding"],
    })
    if mutation == "wrong_id":
        record["asset_id"] = "other"
    elif mutation == "wrong_hash":
        record["model_urdf_sha256"] = "f" * 64
    elif mutation == "missing_metric":
        del record["metrics"]["finite_fields"]
    else:
        record["metrics"]["finite_fields"]["pass"] = False
    job["result_payload"] = record
    rows = runner.execute_killable_jobs(
        [job],
        worker_scratch=tmp_path / "workers",
        timeout_seconds=1,
        max_workers=1,
        on_result=lambda row: None,
        timeout_factory=lambda item: {"asset_id": item["asset_id"], "status": "timeout"},
        exception_factory=lambda item, reason: {
            "asset_id": item["asset_id"], "status": "error", "error": reason
        },
        result_validator=runner.validate_frozen_job_result,
    )
    assert rows[0]["asset_id"] == "asset"
    assert rows[0]["status"] == "error"
    assert "child_result_invalid" in rows[0]["error"]


def test_scheduler_continues_after_child_spawn_failure(tmp_path: Path, monkeypatch: Any) -> None:
    real_popen = runner.subprocess.Popen
    calls = 0

    def flaky_popen(*args: Any, **kwargs: Any) -> Any:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError(errno.EAGAIN, "try again")
        return real_popen(*args, **kwargs)

    monkeypatch.setattr(runner.subprocess, "Popen", flaky_popen)
    completed: list[str] = []
    rows = runner.execute_killable_jobs(
        [
            {"asset_id": "first", "internal_test_action": "echo_environment"},
            {"asset_id": "second", "internal_test_action": "echo_environment"},
        ],
        worker_scratch=tmp_path / "workers",
        timeout_seconds=1,
        max_workers=1,
        on_result=lambda row: completed.append(row["asset_id"]),
        timeout_factory=lambda item: {"asset_id": item["asset_id"], "status": "timeout"},
    )
    assert [row["status"] for row in rows] == ["error", "completed"]
    assert "child_spawn_failed" in rows[0]["error"]
    assert completed == ["first", "second"]
    assert not (tmp_path / "workers").exists()


def test_process_group_cleanup_survives_early_leader_exit(tmp_path: Path) -> None:
    token = f"table2-ignore-term-{os.getpid()}-{time.time_ns()}"
    rows = runner.execute_killable_jobs(
        [{
            "asset_id": "orphan",
            "internal_test_action": "spawn_ignoring_descendant_and_exit",
            "process_group_token": token,
        }],
        worker_scratch=tmp_path / "workers",
        timeout_seconds=1,
        max_workers=1,
        on_result=lambda row: None,
        timeout_factory=lambda item: {"asset_id": item["asset_id"], "status": "timeout"},
    )
    assert rows[0]["status"] == "completed"
    assert rows[0]["worker_evidence"]["termination"] == "SIGKILL"
    assert not any(
        token.encode() in path.read_bytes()
        for path in Path("/proc").glob("[0-9]*/cmdline")
        if path.is_file()
    )


def test_output_lock_is_nonblocking_and_released(tmp_path: Path) -> None:
    output = tmp_path / "output"
    output.mkdir()
    probe = (
        "import importlib.util,sys;"
        f"s=importlib.util.spec_from_file_location('r',{str(RUNNER_PATH)!r});"
        "r=importlib.util.module_from_spec(s);s.loader.exec_module(r);"
        "lock=r.acquire_output_lock(__import__('pathlib').Path(sys.argv[1]));"
        "lock.close()"
    )
    with runner.output_run_lock(output):
        blocked = subprocess.run(
            [sys.executable, "-c", probe, str(output)], capture_output=True, text=True
        )
        assert blocked.returncode != 0
        assert "already locked" in blocked.stderr
    released = subprocess.run(
        [sys.executable, "-c", probe, str(output)], capture_output=True, text=True
    )
    assert released.returncode == 0, released.stderr


def test_output_lock_rejects_symlink_without_truncating_its_target(tmp_path: Path) -> None:
    """Catch lock acquisition following a forged output-owned lock path."""

    output = tmp_path / "output"
    output.mkdir()
    target = tmp_path / "must_remain_unchanged.txt"
    original = b"unrelated data\n"
    target.write_bytes(original)
    (output / ".run.lock").symlink_to(target)

    with pytest.raises(RuntimeError, match="output lock.*non-symlink"):
        runner.acquire_output_lock(output)
    assert target.read_bytes() == original


@pytest.mark.parametrize(
    "joints",
    [
        """
        <joint name="j1" type="fixed"><parent link="root"/><child link="child"/></joint>
        <joint name="j2" type="fixed"><parent link="other"/><child link="child"/></joint>
        """,
        """
        <joint name="j1" type="fixed"><parent link="root"/><child link="child"/></joint>
        <joint name="j2" type="fixed"><parent link="child"/><child link="root"/></joint>
        """,
    ],
)
def test_valid_tree_rejects_multiple_parents_and_cycles(tmp_path: Path, joints: str) -> None:
    """Catch graph checks that accept a non-tree merely from edge count."""

    urdf = f'<robot name="bad">{valid_link("root")}{valid_link("other")}{valid_link("child")}{joints}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    assert record["metrics"]["valid_tree"]["pass"] is False
    assert record["strict_urdf_pass"] is False


def test_valid_tree_rejects_duplicate_link_names(tmp_path: Path) -> None:
    """Catch set-based graph logic that silently merges duplicate links."""

    urdf = f'<robot name="bad">{valid_link("same")}{valid_link("same")}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    assert record["metrics"]["valid_tree"]["pass"] is False


def test_valid_tree_rejects_duplicate_joint_names(tmp_path: Path) -> None:
    """Catch graph validation that ignores ambiguous joint identities."""

    joints = """
    <joint name="same" type="fixed"><parent link="root"/><child link="left"/></joint>
    <joint name="same" type="fixed"><parent link="root"/><child link="right"/></joint>
    """
    urdf = f'<robot name="bad">{valid_link("root")}{valid_link("left")}{valid_link("right")}{joints}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    assert record["metrics"]["valid_tree"]["pass"] is False


@pytest.mark.parametrize(
    ("joint", "expected_issue"),
    [
        (
            """<joint name="j" type="revolute"><parent link="base"/><child link="door"/>
            <axis xyz="0 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>""",
            "axis",
        ),
        (
            """<joint name="j" type="prismatic"><parent link="base"/><child link="door"/>
            <limit lower="1" upper="1" effort="1" velocity="1"/></joint>""",
            "lower_upper",
        ),
        (
            """<joint name="j" type="continuous"><parent link="base"/><child link="door"/>
            <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>""",
            "continuous_finite_interval",
        ),
    ],
)
def test_joint_spec_rejects_invalid_axis_ranges_and_continuous_limits(
    tmp_path: Path, joint: str, expected_issue: str
) -> None:
    """Catch permissive joint validation that contradicts the frozen protocol."""

    urdf = f'<robot name="bad">{valid_link("base")}{valid_link("door")}{joint}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    metric = record["metrics"]["valid_joint_spec"]
    assert metric["pass"] is False
    assert any(expected_issue in issue for issue in metric["issues"])


def test_joint_spec_applies_standard_default_axis(tmp_path: Path) -> None:
    """Catch treating a standards-defined default axis as a missing invalid field."""

    joint = """<joint name="j" type="revolute"><parent link="base"/><child link="door"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/></joint>"""
    urdf = f'<robot name="valid">{valid_link("base")}{valid_link("door")}{joint}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    assert record["metrics"]["valid_joint_spec"]["pass"] is True


def test_finite_fields_rejects_nan_and_infinity(tmp_path: Path) -> None:
    """Catch Python float parsing that accepts non-finite URDF values."""

    bad = valid_link().replace('xyz="0 0 0"', 'xyz="nan 0 0"', 1).replace(
        'mass value="1"', 'mass value="inf"', 1
    )
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="bad">{bad}</robot>'), run_standard_parser=False
    )
    assert record["metrics"]["finite_fields"]["pass"] is False
    assert len(record["metrics"]["finite_fields"]["issues"]) == 2


def test_all_declared_links_remain_in_coverage_denominators(tmp_path: Path) -> None:
    """Catch output-dependent filtering of links without physics metadata."""

    child = '<link name="child"><visual><geometry><sphere radius="1"/></geometry></visual></link>'
    joint = '<joint name="j" type="fixed"><parent link="base"/><child link="child"/></joint>'
    urdf = f'<robot name="partial">{valid_link("base")}{child}{joint}</robot>'
    record = runner.audit_asset_package(write_asset(tmp_path, urdf), run_standard_parser=False)
    assert record["metrics"]["collision_coverage"]["pass"] is False
    assert record["metrics"]["collision_coverage"]["denominator_links"] == 2
    assert record["metrics"]["inertial_coverage"]["pass"] is False
    assert record["metrics"]["inertial_coverage"]["denominator_links"] == 2


def test_collision_coverage_rejects_nonpositive_primitive_dimensions(tmp_path: Path) -> None:
    """Catch collision coverage that checks only for a geometry element."""

    link = valid_link().replace(
        '<collision><geometry><box size="1 1 1"/></geometry></collision>',
        '<collision><geometry><box size="1 0 1"/></geometry></collision>',
    )
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="bad">{link}</robot>'), run_standard_parser=False
    )
    assert record["metrics"]["collision_coverage"]["pass"] is False


def test_inertial_coverage_rejects_duplicate_blocks_but_allows_default_origin(
    tmp_path: Path,
) -> None:
    """Catch ambiguous duplicate inertials and rejecting the standard zero-COM default."""

    one = valid_link().replace('<origin xyz="0 0 0" rpy="0 0 0"/>', "", 1)
    valid_record = runner.audit_asset_package(
        write_asset(tmp_path / "one", f'<robot name="valid">{one}</robot>'),
        run_standard_parser=False,
    )
    assert valid_record["metrics"]["inertial_coverage"]["pass"] is True
    assert valid_record["metrics"]["inertia_validity"]["pass"] is True

    inertial = """
    <inertial><mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    """
    duplicate = one.replace("</link>", inertial + "</link>")
    duplicate_record = runner.audit_asset_package(
        write_asset(tmp_path / "duplicate", f'<robot name="bad">{duplicate}</robot>'),
        run_standard_parser=False,
    )
    assert duplicate_record["metrics"]["inertial_coverage"]["pass"] is False


def test_malformed_xml_returns_a_fail_closed_record(tmp_path: Path) -> None:
    """Catch parse exceptions that abort the cohort or disappear from aggregation."""

    asset = write_asset(tmp_path, '<robot name="broken"><link name="base"></robot>')
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    assert set(record["metrics"]) == set(runner.METRIC_NAMES)
    assert all(metric["pass"] is False for metric in record["metrics"].values())
    assert all(metric["issues"] for metric in record["metrics"].values())
    assert record["strict_urdf_pass"] is False


@pytest.mark.parametrize(
    ("inertia", "expected_issue"),
    [
        ('ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="3"', "triangle"),
        ('ixx="1" ixy="2" ixz="0" iyy="1" iyz="0" izz="1"', "positive_definite"),
    ],
)
def test_inertia_validity_checks_full_spd_and_principal_triangle(
    tmp_path: Path, inertia: str, expected_issue: str
) -> None:
    """Catch diagonal-only inertia validation and omitted realizability checks."""

    link = valid_link().replace(
        'ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"', inertia
    )
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="bad">{link}</robot>'), run_standard_parser=False
    )
    metric = record["metrics"]["inertia_validity"]
    assert metric["pass"] is False
    assert any(expected_issue in issue for issue in metric["issues"])


def test_resource_resolution_rejects_missing_and_escaping_meshes(tmp_path: Path) -> None:
    """Catch resource gates that only test filename presence or allow traversal."""

    for index, filename in enumerate(("assets/missing.obj", "../outside.obj")):
        case = tmp_path / f"case_{index}"
        case.mkdir()
        link = valid_link().replace(
            '<box size="1 1 1"/>', f'<mesh filename="{filename}"/>', 1
        )
        asset = write_asset(case, f'<robot name="bad">{link}</robot>')
        record = runner.audit_asset_package(asset, run_standard_parser=False)
        assert record["metrics"]["resource_resolution"]["pass"] is False


def test_resource_resolution_reads_obj_mtl_texture_closure(tmp_path: Path) -> None:
    """Catch stopping resource resolution at an existing OBJ file."""

    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/mesh.obj"/>', 1
    )
    files = {
        "assets/mesh.obj": b"mtllib material.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        "assets/material.mtl": b"newmtl m\nmap_Kd missing.png\n",
    }
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="bad">{link}</robot>', files),
        run_standard_parser=False,
    )
    metric = record["metrics"]["resource_resolution"]
    assert metric["pass"] is False
    assert any("missing.png" in issue for issue in metric["issues"])


def test_resource_resolution_accepts_unquoted_obj_mtl_filename_with_spaces(
    tmp_path: Path,
) -> None:
    """Accept exporter-style unquoted OBJ material filenames containing spaces."""

    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/mesh.obj"/>', 1
    )
    files = {
        "assets/mesh.obj": (
            b"mtllib material with spaces.mtl\n"
            b"usemtl painted\n"
            b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        ),
        "assets/material with spaces.mtl": b"newmtl painted\n",
    }
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="valid">{link}</robot>', files),
        run_standard_parser=False,
    )
    metric = record["metrics"]["resource_resolution"]
    assert metric["pass"] is True
    assert metric["issues"] == []


def test_nested_resource_specs_accepts_unquoted_mtl_texture_path_with_spaces(
    tmp_path: Path,
) -> None:
    mtl = tmp_path / "material.mtl"
    texture = tmp_path / "textures" / "painted surface.png"
    texture.parent.mkdir()
    texture.write_bytes(b"placeholder")
    mtl.write_text("map_Kd textures/painted surface.png\n", encoding="utf-8")

    assert runner.nested_resource_specs(mtl) == [
        ("mtl_texture", "textures/painted surface.png")
    ]


def test_obj_escaping_mtllib_is_rejected_before_loader_can_open_it(
    tmp_path: Path, monkeypatch: Any
) -> None:
    outside = tmp_path / "outside.mtl"
    outside.write_text("newmtl outside\n")
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/mesh.obj"/>', 1
    )
    asset = write_asset(
        tmp_path / "case",
        f'<robot name="bad">{link}</robot>',
        {
            "assets/mesh.obj": (
                b"mtllib ../../outside.mtl\n"
                b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
            )
        },
    )
    opened_outside = False
    loader_called = False
    original_open = Path.open

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside MTL was opened")
        return original_open(path, *args, **kwargs)

    def tracked_load(*args: Any, **kwargs: Any) -> Any:
        nonlocal loader_called
        loader_called = True
        raise AssertionError("mesh loader ran before nested containment preflight")

    monkeypatch.setattr(Path, "open", tracked_open)
    monkeypatch.setattr(runner.trimesh, "load", tracked_load)
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    metric = record["metrics"]["resource_resolution"]
    assert metric["pass"] is False
    assert any("escapes" in issue for issue in metric["issues"])
    assert opened_outside is False
    assert loader_called is False


def test_obj_mtl_escaping_texture_is_rejected_before_loader_can_open_it(
    tmp_path: Path, monkeypatch: Any
) -> None:
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"outside")
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/mesh.obj"/>', 1
    )
    asset = write_asset(
        tmp_path / "case",
        f'<robot name="bad">{link}</robot>',
        {
            "assets/mesh.obj": (
                b"mtllib material.mtl\n"
                b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
            ),
            "assets/material.mtl": b"newmtl m\nmap_Kd ../../outside.png\n",
        },
    )
    opened_outside = False
    loader_called = False
    original_open = Path.open

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside texture was opened")
        return original_open(path, *args, **kwargs)

    def tracked_load(*args: Any, **kwargs: Any) -> Any:
        nonlocal loader_called
        loader_called = True
        raise AssertionError("mesh loader ran before recursive containment preflight")

    monkeypatch.setattr(Path, "open", tracked_open)
    monkeypatch.setattr(runner.trimesh, "load", tracked_load)
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    metric = record["metrics"]["resource_resolution"]
    assert metric["pass"] is False
    assert any("escapes" in issue for issue in metric["issues"])
    assert opened_outside is False
    assert loader_called is False


def test_package_preflight_blocks_standard_parser_and_all_mesh_loaders(
    tmp_path: Path, monkeypatch: Any
) -> None:
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"outside")
    visual = (
        '<visual><geometry><mesh filename="assets/mesh.obj"/></geometry></visual>'
    )
    link = valid_link().replace(
        '<visual><geometry><box size="1 1 1"/></geometry></visual>', visual
    )
    asset = write_asset(
        tmp_path / "case",
        f'<robot name="bad">{link}</robot>',
        {
            "assets/mesh.obj": (
                b"mtllib material.mtl\n"
                b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
            ),
            "assets/material.mtl": b"newmtl m\nmap_Kd ../../outside.png\n",
        },
    )
    parser_called = False
    loader_called = False
    opened_outside = False
    original_open = Path.open

    def blocked_parser(*args: Any, **kwargs: Any) -> Any:
        nonlocal parser_called
        parser_called = True
        raise AssertionError("standard parser ran before package containment preflight")

    def blocked_loader(*args: Any, **kwargs: Any) -> Any:
        nonlocal loader_called
        loader_called = True
        raise AssertionError("mesh loader ran after failed package containment preflight")

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside texture was opened")
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(runner, "standard_parser_result", blocked_parser)
    monkeypatch.setattr(runner.trimesh, "load", blocked_loader)
    monkeypatch.setattr(Path, "open", tracked_open)
    record = runner.audit_asset_package(asset, run_standard_parser=True)
    assert record["metrics"]["parse_rate"]["pass"] is False
    assert "containment_preflight_failed_before_standard_parser" in record["metrics"]["parse_rate"]["issues"]
    assert record["metrics"]["resource_resolution"]["pass"] is False
    assert any("escapes" in issue for issue in record["metrics"]["resource_resolution"]["issues"])
    assert record["metrics"]["collision_coverage"]["pass"] is False
    assert any("containment_preflight" in issue for issue in record["metrics"]["collision_coverage"]["issues"])
    assert parser_called is False
    assert loader_called is False
    assert opened_outside is False


def test_package_preflight_blocks_collision_only_mesh_loader(
    tmp_path: Path, monkeypatch: Any
) -> None:
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"outside")
    collision = (
        '<collision><geometry><mesh filename="assets/mesh.obj"/></geometry></collision>'
    )
    link = valid_link().replace(
        '<collision><geometry><box size="1 1 1"/></geometry></collision>', collision
    )
    asset = write_asset(
        tmp_path / "case",
        f'<robot name="bad">{link}</robot>',
        {
            "assets/mesh.obj": (
                b"mtllib material.mtl\n"
                b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
            ),
            "assets/material.mtl": b"newmtl m\nmap_Kd ../../outside.png\n",
        },
    )
    loader_called = False
    opened_outside = False
    original_open = Path.open

    def blocked_loader(*args: Any, **kwargs: Any) -> Any:
        nonlocal loader_called
        loader_called = True
        raise AssertionError("collision loader ran after failed package containment preflight")

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside texture was opened")
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(runner.trimesh, "load", blocked_loader)
    monkeypatch.setattr(Path, "open", tracked_open)
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    assert record["metrics"]["resource_resolution"]["pass"] is False
    assert record["metrics"]["collision_coverage"]["pass"] is False
    assert loader_called is False
    assert opened_outside is False


def test_corrupt_glb_fails_resource_resolution(tmp_path: Path) -> None:
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/corrupt.glb"/>', 1
    )
    record = runner.audit_asset_package(
        write_asset(
            tmp_path,
            f'<robot name="bad">{link}</robot>',
            {"assets/corrupt.glb": b"not-a-glb"},
        ),
        run_standard_parser=False,
    )
    assert record["metrics"]["resource_resolution"]["pass"] is False


def test_glb_with_corrupt_embedded_texture_fails_resource_resolution(tmp_path: Path) -> None:
    from pygltflib import Buffer, BufferView, GLTF2, Image as GltfImage

    gltf = GLTF2(
        buffers=[Buffer(byteLength=9)],
        bufferViews=[BufferView(buffer=0, byteOffset=0, byteLength=9)],
        images=[GltfImage(bufferView=0, mimeType="image/png")],
    )
    gltf.set_binary_blob(b"not-a-png")
    case = tmp_path / "case"
    asset = write_asset(case, f'<robot name="bad">{valid_link()}</robot>')
    glb = asset / "assets" / "texture.glb"
    glb.parent.mkdir(parents=True)
    gltf.save_binary(glb)
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/texture.glb"/>', 1
    )
    (asset / "model.urdf").write_text(f'<robot name="bad">{link}</robot>')
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    assert record["metrics"]["resource_resolution"]["pass"] is False
    assert any("embedded_image" in issue for issue in record["metrics"]["resource_resolution"]["issues"])


def test_gltf_escaping_external_buffer_is_never_opened(tmp_path: Path, monkeypatch: Any) -> None:
    asset = write_asset(tmp_path, f'<robot name="bad">{valid_link()}</robot>')
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")
    gltf = asset / "mesh.gltf"
    gltf.write_text(json.dumps({"asset": {"version": "2.0"}, "buffers": [
        {"uri": "../outside.bin", "byteLength": 7}
    ]}))
    opened_outside = False
    original_open = Path.open

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside buffer was opened")
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", tracked_open)
    issue = runner.readable_resource_issue("urdf_mesh", gltf, asset)
    assert issue and "escapes" in issue
    assert opened_outside is False


def test_gltf_corrupt_data_uri_image_fails(tmp_path: Path) -> None:
    asset = write_asset(tmp_path, f'<robot name="bad">{valid_link()}</robot>')
    gltf = asset / "mesh.gltf"
    gltf.write_text(json.dumps({
        "asset": {"version": "2.0"},
        "images": [{"uri": "data:image/png;base64,bm90LWEtcG5n"}],
    }))
    issue = runner.readable_resource_issue("urdf_mesh", gltf, asset)
    assert issue and "data_uri_image" in issue


def test_gltf_escaping_external_image_is_never_opened(
    tmp_path: Path, monkeypatch: Any
) -> None:
    asset = write_asset(tmp_path / "case", f'<robot name="bad">{valid_link()}</robot>')
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"outside")
    gltf = asset / "mesh.gltf"
    gltf.write_text(json.dumps({
        "asset": {"version": "2.0"},
        "images": [{"uri": "../../outside.png"}],
    }))
    opened_outside = False
    original_open = Path.open

    def tracked_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        nonlocal opened_outside
        if path.resolve(strict=False) == outside.resolve():
            opened_outside = True
            raise AssertionError("outside image was opened")
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", tracked_open)
    issue = runner.readable_resource_issue("urdf_mesh", gltf, asset)
    assert issue and "escapes" in issue
    assert opened_outside is False


def test_gltf_contained_extensionless_external_image_is_accepted(tmp_path: Path) -> None:
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="mesh.gltf"/>', 1
    )
    asset = write_asset(tmp_path, f'<robot name="valid">{link}</robot>')
    png = base64.b64decode(
        "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUB"
        "AScY42YAAAAASUVORK5CYII="
    )
    geometry = struct.pack("<9f3H", 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 2)
    (asset / "mesh.bin").write_bytes(geometry)
    (asset / "texture").write_bytes(png)
    gltf = asset / "mesh.gltf"
    gltf.write_text(json.dumps({
        "asset": {"version": "2.0"},
        "buffers": [{"uri": "mesh.bin", "byteLength": len(geometry)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": 36, "target": 34962},
            {"buffer": 0, "byteOffset": 36, "byteLength": 6, "target": 34963},
        ],
        "accessors": [
            {
                "bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3",
                "min": [0, 0, 0], "max": [1, 1, 0],
            },
            {"bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR"},
        ],
        "images": [{"uri": "texture", "mimeType": "image/png"}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "nodes": [{"mesh": 0}],
        "scenes": [{"nodes": [0]}],
        "scene": 0,
    }))
    record = runner.audit_asset_package(asset, run_standard_parser=False)
    assert record["metrics"]["resource_resolution"]["pass"] is True


def test_standard_parse_and_resource_both_fail_for_missing_mesh(tmp_path: Path) -> None:
    link = valid_link().replace(
        '<box size="1 1 1"/>', '<mesh filename="assets/missing.obj"/>', 1
    )
    record = runner.audit_asset_package(
        write_asset(tmp_path, f'<robot name="bad">{link}</robot>'),
        run_standard_parser=True,
    )
    assert record["metrics"]["parse_rate"]["pass"] is False
    assert record["metrics"]["resource_resolution"]["pass"] is False
    assert "referenced resources" in record["metrics"]["parse_rate"]["scope"]


def test_release_discovery_rejects_symlink_packages_and_models(tmp_path: Path) -> None:
    root = tmp_path / "release"
    real = root / "real"
    real.mkdir(parents=True)
    (real / "model.urdf").write_text('<robot name="r"><link name="x"/></robot>')
    (root / "linked").symlink_to(real, target_is_directory=True)
    with pytest.raises(RuntimeError, match="symlink"):
        runner.release_asset_ids(root)

    (root / "linked").unlink()
    model = real / "model.urdf"
    target = tmp_path / "outside.urdf"
    target.write_text('<robot name="r"><link name="x"/></robot>')
    model.unlink()
    model.symlink_to(target)
    with pytest.raises(RuntimeError, match="model.urdf"):
        runner.release_asset_ids(root)


def test_strict_release_discovery_rejects_unexpected_top_level_file(tmp_path: Path) -> None:
    root = tmp_path / "release"
    package = root / "rec_one"
    package.mkdir(parents=True)
    (package / "model.urdf").write_text('<robot name="r"><link name="x"/></robot>')
    (root / "README.txt").write_text("unexpected")
    assert runner.release_asset_ids(root) == ["rec_one"]
    with pytest.raises(RuntimeError, match="non-directory"):
        runner.release_asset_ids(root, reject_non_directories=True)


def test_archive_release_identity_rejects_extracted_set_mismatch(tmp_path: Path) -> None:
    (tmp_path / "rec_one.tar.gz").write_bytes(b"one")
    (tmp_path / "rec_two.tar.gz").write_bytes(b"two")
    identity = runner.archive_release_identity(tmp_path, ["rec_one", "rec_two"])
    assert identity["archive_ids"] == ["rec_one", "rec_two"]
    assert identity["archive_id_list_sha256"]
    assert identity["archive_filename_list_sha256"]
    with pytest.raises(RuntimeError, match="identity mismatch"):
        runner.archive_release_identity(tmp_path, ["rec_one", "rec_three"])


def test_stale_worker_scratch_dead_job_is_quarantined(tmp_path: Path) -> None:
    output = tmp_path / "output"
    stale = output / ".worker_scratch" / "job_000000"
    stale.mkdir(parents=True)
    (stale / "ownership.json").write_text(json.dumps({"pid": 99999999, "pgid": 99999999}))
    output.mkdir(exist_ok=True)
    with runner.output_run_lock(output):
        recovery = runner.recover_stale_worker_scratch(output)
    assert recovery["quarantined"] is True
    assert not (output / ".worker_scratch").exists()
    assert any(path.name.startswith("stale_worker_scratch_") for path in output.iterdir())


def test_stale_worker_scratch_never_kills_unproven_process(tmp_path: Path) -> None:
    output = tmp_path / "output"
    stale = output / ".worker_scratch" / "job_000000"
    stale.mkdir(parents=True)
    sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"])
    try:
        (stale / "ownership.json").write_text(json.dumps({
            "pid": sleeper.pid,
            "pgid": os.getpgid(sleeper.pid),
            "run_token": "wrong",
            "job_path": str(stale / "job.json"),
            "process_start_identity": runner.proc_start_identity(sleeper.pid),
        }))
        with runner.output_run_lock(output):
            recovery = runner.recover_stale_worker_scratch(output)
        assert recovery["terminated_owned_groups"] == []
        assert sleeper.poll() is None
    finally:
        sleeper.terminate()
        sleeper.wait()


def test_proven_owned_process_rejects_forged_unrelated_pgid(tmp_path: Path) -> None:
    output = tmp_path / "output"
    job_root = output / ".worker_scratch" / "job_000000"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    token = f"owned-{time.time_ns()}"
    job_path.write_text(json.dumps({
        "asset_id": "owned",
        "internal_test_action": "sleep",
        "sleep": 30,
        "run_token": token,
    }))
    owned = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER_PATH),
            "--internal-child-job",
            str(job_path),
            "--internal-child-result",
            str(result_path),
        ],
        start_new_session=True,
    )
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True
    )
    try:
        metadata = {
            "pid": owned.pid,
            "pgid": unrelated.pid,
            "run_token": token,
            "job_path": str(job_path),
            "runner_script": str(RUNNER_PATH),
            "runner_sha256": runner.sha256_file(RUNNER_PATH),
            "output_root": str(output),
            "process_start_identity": runner.proc_start_identity(owned.pid),
        }
        (job_root / "ownership.json").write_text(json.dumps(metadata))
        with runner.output_run_lock(output):
            recovery = runner.recover_stale_worker_scratch(output)
        assert recovery["terminated_owned_groups"] == []
        assert recovery["quarantined"] is True
        assert owned.poll() is None
        assert unrelated.poll() is None
    finally:
        for process in (owned, unrelated):
            if process.poll() is None:
                os.killpg(process.pid, 9)
            process.wait()


def test_stale_worker_scratch_kills_only_proven_owned_child(tmp_path: Path) -> None:
    output = tmp_path / "output"
    job_root = output / ".worker_scratch" / "job_000000"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    token = f"owned-{time.time_ns()}"
    job_path.write_text(json.dumps({
        "asset_id": "owned",
        "internal_test_action": "sleep",
        "sleep": 30,
        "run_token": token,
    }))
    process = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER_PATH),
            "--internal-child-job",
            str(job_path),
            "--internal-child-result",
            str(result_path),
        ],
        start_new_session=True,
    )
    try:
        ownership = {
            "pid": process.pid,
            "pgid": process.pid,
            "run_token": token,
            "job_path": str(job_path),
            "runner_script": str(RUNNER_PATH),
            "runner_sha256": runner.sha256_file(RUNNER_PATH),
            "output_root": str(output),
            "process_start_identity": runner.proc_start_identity(process.pid),
        }
        (job_root / "ownership.json").write_text(json.dumps(ownership))
        with runner.output_run_lock(output):
            recovery = runner.recover_stale_worker_scratch(output)
        process.wait(timeout=2)
        assert process.pid in recovery["terminated_owned_groups"]
        assert process.poll() is not None
    finally:
        if process.poll() is None:
            os.killpg(process.pid, 9)
            process.wait()


def test_resume_record_validation_rejects_strict_and_schema_inconsistency() -> None:
    metrics = {name: {"pass": True, "issues": []} for name in runner.METRIC_NAMES}
    record = {
        "asset_id": "asset",
        "status": "completed",
        "error": None,
        "metrics": metrics,
        "strict_urdf_pass": True,
        "model_urdf_sha256": "a" * 64,
        "package_content_manifest_sha256": "b" * 64,
        "manifest_content_sha256": "c" * 64,
    }
    runner.validate_resume_record(record, "asset", "a" * 64, "b" * 64, "c" * 64)
    record["metrics"]["finite_fields"]["pass"] = False
    with pytest.raises(RuntimeError, match="strict"):
        runner.validate_resume_record(record, "asset", "a" * 64, "b" * 64, "c" * 64)


def test_frozen_config_and_dependency_metadata_are_explicit() -> None:
    config = runner.EVALUATOR_CONFIG
    assert config["axis_epsilon"] == runner.AXIS_EPSILON
    assert config["inertia_relative_tolerance"] == runner.INERTIA_RELATIVE_TOLERANCE
    assert config["inertia_dtype"] == "numpy.float64"
    assert config["asset_timeout_seconds"] == 120
    assert config["child_process"]["start_new_session"] is True
    assert config["child_process"]["thread_environment"] == runner.CHILD_THREAD_ENVIRONMENT
    assert "GLB" in config["resource_validation_scope"]
    environment = runner.environment_metadata(True, 4)
    expected_dependencies = {
        "numpy", "urdfpy", "trimesh", "Pillow", "networkx", "pycollada", "pygltflib",
        "lxml", "six", "scipy",
    }
    assert set(runner.EVALUATION_DEPENDENCIES) == expected_dependencies
    assert set(config["bound_dependencies"]) == expected_dependencies
    assert set(environment["dependencies"]) == expected_dependencies


def test_aggregate_preserves_intent_to_evaluate_denominator() -> None:
    """Catch aggregation that drops parse errors or strict failures."""

    passes = {name: {"pass": True} for name in runner.METRIC_NAMES}
    one_fail = {
        name: {"pass": name not in {"finite_fields", "strict_urdf_pass"}}
        for name in runner.METRIC_NAMES
    }
    hard_error = {name: {"pass": False} for name in runner.METRIC_NAMES}
    rows = [
        {"asset_id": "a", "metrics": passes, "strict_urdf_pass": True},
        {"asset_id": "b", "metrics": one_fail, "strict_urdf_pass": False},
        {"asset_id": "c", "metrics": hard_error, "strict_urdf_pass": False},
    ]
    summary = runner.aggregate_records(rows, expected_n=3)
    assert summary["n_eval"] == 3
    assert summary["metrics"]["parse_rate"] == {"passed": 2, "denominator": 3, "rate": 2 / 3}
    assert summary["metrics"]["finite_fields"] == {"passed": 1, "denominator": 3, "rate": 1 / 3}
    assert summary["strict_urdf_pass"] == {"passed": 1, "denominator": 3, "rate": 1 / 3}
    assert summary["error_count"] == 0
    assert summary["status_counts"] == {"unknown": 3}
    assert summary["category_macro"] == {
        "state": "not_evaluable",
        "reason": "Articraft-10K release has no authoritative category labels",
    }
