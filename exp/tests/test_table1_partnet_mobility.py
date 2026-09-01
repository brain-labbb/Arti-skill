from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table1_partnet_mobility.py"
PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
INPUT_IDENTITY_FIELDS = (
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


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_partnet_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_package(
    root: Path,
    dataset_id: str,
    *,
    category: str,
    missing_mesh: bool = False,
    textured: bool = False,
) -> tuple[Path, dict[str, object]]:
    package = root / "dataset" / dataset_id
    package.mkdir(parents=True)
    (package / "meta.json").write_text(
        json.dumps(
            {
                "anno_id": dataset_id,
                "model_id": f"model-{dataset_id}",
                "model_cat": category,
            }
        ),
        encoding="utf-8",
    )
    (package / "bounding_box.json").write_text(
        json.dumps({"min": [0, 0, 0], "max": [1, 1, 1]}),
        encoding="utf-8",
    )

    mesh_reference = "textured_objs/mesh.obj"
    mesh_path = package / mesh_reference
    if not missing_mesh:
        mesh_path.parent.mkdir()
        if textured:
            (package / "images").mkdir()
            (package / "images/texture.png").write_bytes(b"fixture-texture")
            (package / "textured_objs/mesh.mtl").write_text(
                "newmtl fixture\nmap_Kd ../images/texture.png\n",
                encoding="utf-8",
            )
            mesh_payload = "mtllib mesh.mtl\n"
        else:
            mesh_payload = ""
        mesh_path.write_text(
            mesh_payload + "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )

    urdf = package / "mobility.urdf"
    urdf.write_text(
        f"""<robot name="fixture-{dataset_id}">
  <link name="base">
    <visual><geometry><mesh filename="{mesh_reference}"/></geometry></visual>
    <collision><geometry><mesh filename="{mesh_reference}"/></geometry></collision>
  </link>
  <link name="child"/>
  <joint name="joint" type="revolute">
    <parent link="base"/><child link="child"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    mesh_row = {
        "path": mesh_reference,
        "exists": not missing_mesh,
        "size_bytes": mesh_path.stat().st_size if mesh_path.is_file() else None,
        "sha256": _sha256_file(mesh_path) if mesh_path.is_file() else None,
    }
    item: dict[str, object] = {
        "protocol_id": PROTOCOL_ID,
        "dataset_id": dataset_id,
        "category": category,
        "selection_digest": hashlib.sha256(
            f"{SELECTION_SALT}\0{dataset_id}".encode("utf-8")
        ).hexdigest(),
        "package_audit_success": True,
        "missing_core_files": [],
        "missing_collision_mesh_reference_count": int(missing_mesh),
        "movable_dof_count": 1,
        "range_evaluable_dof_count": 1,
        "joint_specs_sha256": _canonical_sha256([]),
        "runtime_identity_sha256": _canonical_sha256({"fixture": dataset_id}),
        "urdf_sha256": _sha256_file(urdf),
        "bounding_box_sha256": _sha256_file(package / "bounding_box.json"),
        "collision_mesh_files": [mesh_row],
        "collision_mesh_inventory_sha256": _canonical_sha256([mesh_row]),
        "object_bbox_diagonal_m": 1.0,
        "rest_state_expected": True,
        "single_state_expected": True,
        "sobol_state_expected": False,
    }
    return package, item


def _write_manifest(
    root: Path,
    packages_and_items: list[tuple[Path, dict[str, object]]],
) -> Path:
    archive = root / "partnet-mobility-v0.zip"
    archive.write_bytes(b"fixture archive")
    items = []
    for order, (_, raw) in enumerate(packages_and_items):
        item = dict(raw)
        item["order"] = order
        item["input_identity_sha256"] = _canonical_sha256(
            {key: item[key] for key in INPUT_IDENTITY_FIELDS}
        )
        items.append(item)
    release_ids = sorted(
        (path.name for path in (root / "dataset").iterdir()),
        key=int,
    )
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "status": "FROZEN",
        "qualification_smoke": False,
        "dataset_root": str((root / "dataset").resolve()),
        "archive": {
            "path": str(archive.resolve()),
            "size_bytes": archive.stat().st_size,
            "sha256": _sha256_file(archive),
            "matches_expected_sha256": True,
        },
        "release_asset_count": len(release_ids),
        "candidate_pool_identity_sha256": _canonical_sha256(release_ids),
        "sample_size": len(items),
        "selection_policy": {
            "algorithm": "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)",
            "salt": SELECTION_SALT,
            "identity_fields_used": ["dataset_id"],
            "outcome_based_filtering": False,
            "selected_failures_retained_without_replacement": True,
        },
        "cohort_boundary": {
            "is_full_release_cohort": False,
            "is_shared_category_balanced_cohort": False,
            "paper_table_role": "sampled release diagnostic",
        },
        "items": items,
    }
    manifest["ordered_selected_ids_sha256"] = _canonical_sha256(
        [str(item["dataset_id"]) for item in items]
    )
    manifest["items_sha256"] = _canonical_sha256(items)
    path = root / "frozen_manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_load_frozen_cohort_preserves_order_and_uses_mobility_urdf(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    second = _write_package(tmp_path, "20", category="Table")
    first = _write_package(tmp_path, "10", category="Chair")
    manifest = _write_manifest(tmp_path, [second, first])

    cohort = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "dataset",
        expected_n=2,
        formal=False,
    )

    assert [row["asset_id"] for row in cohort["assets"]] == ["20", "10"]
    assert [row["selection_index"] for row in cohort["assets"]] == [0, 1]
    assert [row["raw_category"] for row in cohort["assets"]] == ["Table", "Chair"]
    assert [row["primary_urdf_relative_path"] for row in cohort["assets"]] == [
        "mobility.urdf",
        "mobility.urdf",
    ]
    assert cohort["release_asset_count"] == 2
    assert cohort["release_category_count"] == 2


def test_load_frozen_cohort_rejects_ambiguous_top_level_urdf(tmp_path: Path) -> None:
    runner = _load_runner()
    package, item = _write_package(tmp_path, "10", category="Chair")
    manifest = _write_manifest(tmp_path, [(package, item)])
    (package / "alternate.urdf").write_text("<robot name='alternate'/>", encoding="utf-8")

    with pytest.raises(ValueError, match="exactly one top-level URDF"):
        runner.load_frozen_cohort(
            manifest,
            source_root=tmp_path / "dataset",
            expected_n=1,
            formal=False,
        )


def test_load_frozen_cohort_rejects_category_metadata_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    package, item = _write_package(tmp_path, "10", category="Chair")
    manifest = _write_manifest(tmp_path, [(package, item)])
    (package / "meta.json").write_text(
        json.dumps({"anno_id": "10", "model_cat": "Table"}),
        encoding="utf-8",
    )

    with pytest.raises(
        (ValueError, RuntimeError),
        match=r"category (?:metadata mismatch|drift)",
    ):
        runner.load_frozen_cohort(
            manifest,
            source_root=tmp_path / "dataset",
            expected_n=1,
            formal=False,
        )


def test_formal_release_category_contract_requires_46() -> None:
    runner = _load_runner()

    with pytest.raises(ValueError, match="exactly 46 release categories"):
        runner._validate_release_category_count(45, formal=True)

    runner._validate_release_category_count(45, formal=False)


def test_evaluate_package_retains_structure_when_fingerprint_is_incomplete(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    package, item = _write_package(
        tmp_path,
        "10",
        category="Chair",
        missing_mesh=True,
    )
    manifest = _write_manifest(tmp_path, [(package, item)])
    identity = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "dataset",
        expected_n=1,
        formal=False,
    )["assets"][0]

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATED_FINGERPRINT_INCOMPLETE"
    assert record["parse_success"] is True
    assert record["link_count"] == 2
    assert record["non_fixed_joint_count"] == 1
    assert record["valid_tree"] is True
    assert record["fingerprint_complete"] is False
    assert record["package_fingerprint"] is None
    assert record["missing_resources"] == ["textured_objs/mesh.obj"]


def test_evaluate_package_hashes_obj_mtl_texture_closure(tmp_path: Path) -> None:
    runner = _load_runner()
    package, item = _write_package(
        tmp_path,
        "10",
        category="Chair",
        textured=True,
    )
    manifest = _write_manifest(tmp_path, [(package, item)])
    identity = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "dataset",
        expected_n=1,
        formal=False,
    )["assets"][0]

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATED"
    assert record["fingerprint_complete"] is True
    assert record["referenced_resource_count"] == 3
    assert record["missing_resources"] == []


def test_evaluate_package_rejects_resource_drift_after_snapshot(tmp_path: Path) -> None:
    runner = _load_runner()
    package, item = _write_package(
        tmp_path,
        "10",
        category="Chair",
        textured=True,
    )
    manifest = _write_manifest(tmp_path, [(package, item)])
    identity = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "dataset",
        expected_n=1,
        formal=False,
    )["assets"][0]

    (package / "images/texture.png").write_bytes(b"changed-after-snapshot")
    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATION_FAILED"
    assert record["parse_success"] is False
    assert "package changed after Table 1 snapshot" in record["error"]


def test_evaluate_package_rejects_drift_after_second_fingerprint(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    package, item = _write_package(
        tmp_path,
        "10",
        category="Chair",
        textured=True,
    )
    manifest = _write_manifest(tmp_path, [(package, item)])
    identity = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "dataset",
        expected_n=1,
        formal=False,
    )["assets"][0]
    original_observation = runner._fingerprint_observation
    observation_count = 0

    def mutate_after_second_observation(urdf_path: Path):
        nonlocal observation_count
        observation = original_observation(urdf_path)
        observation_count += 1
        if observation_count == 2:
            (package / "images/texture.png").write_bytes(b"changed-after-second-read")
        return observation

    monkeypatch.setattr(
        runner,
        "_fingerprint_observation",
        mutate_after_second_observation,
    )
    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATION_FAILED"
    assert "package changed after Table 1 snapshot" in record["error"]


def test_aggregate_keeps_intent_to_evaluate_and_coverage_denominators() -> None:
    runner = _load_runner()
    records = [
        {
            "asset_id": "10",
            "raw_category": "Chair",
            "parse_success": True,
            "link_count": 2,
            "non_fixed_joint_count": 1,
            "joint_type_counts": {"revolute": 1},
            "valid_tree": True,
            "topology_hash": "topology-a",
            "fingerprint_complete": True,
            "package_fingerprint": "fingerprint-a",
        },
        {
            "asset_id": "20",
            "raw_category": "Table",
            "parse_success": True,
            "link_count": 3,
            "non_fixed_joint_count": 2,
            "joint_type_counts": {"prismatic": 2},
            "valid_tree": True,
            "topology_hash": "topology-b",
            "fingerprint_complete": False,
            "package_fingerprint": None,
        },
    ]

    summary = runner.aggregate_partnet_records(
        records,
        release_asset_count=2347,
        release_category_count=46,
    )

    assert summary["cohort"]["N_eval"] == 2
    assert summary["cohort"]["N_parse"] == 2
    assert summary["multi_joint_assets"]["denominator"] == 2
    assert summary["unique_topologies"]["denominator"] == 2
    assert summary["exact_duplicate_rate"]["denominator"] == 1
    assert summary["exact_duplicate_rate"]["coverage_denominator"] == 2
