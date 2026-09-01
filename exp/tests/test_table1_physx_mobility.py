from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table1_physx_mobility.py"
TABLE5_COMMON = Path(
    "/mnt/zsn/lyb/arti-skill-table5-physx-n800/exp/scripts/table5_physx_common.py"
)
RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_asset(
    root: Path,
    dataset_id: int,
    *,
    category: str,
    movable: int = 1,
    fixed_extra: bool = True,
    texture: bool = True,
) -> None:
    objs = root / "partseg" / str(dataset_id) / "objs"
    objs.mkdir(parents=True)
    if texture:
        (objs / f"{dataset_id}.png").write_bytes(b"fixture-texture")
        mtl_text = f"newmtl fixture\nmap_Kd {dataset_id}.png\n"
    else:
        mtl_text = "newmtl fixture\n"
    (objs / f"{dataset_id}.mtl").write_text(mtl_text, encoding="utf-8")
    (objs / f"{dataset_id}.obj").write_text(
        f"mtllib {dataset_id}.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )

    mesh = f"./../partseg/{dataset_id}/objs/{dataset_id}.obj"
    lines = ['<?xml version="1.0"?>', '<robot name="scene">', '  <link name="l_world"/>']
    lines.append(
        f'  <link name="l_0"><visual><geometry><mesh filename="{mesh}"/>'
        "</geometry></visual></link>"
    )
    lines.append(
        '  <joint name="j_0" type="revolute"><parent link="l_world"/><child link="l_0"/>'
        '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>'
    )
    for index in range(1, movable):
        lines.append(f'  <link name="l_{index}"/>')
        lines.append(
            f'  <joint name="j_{index}" type="revolute">'
            f'<parent link="l_{index - 1}"/><child link="l_{index}"/>'
            '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>'
        )
    if fixed_extra:
        lines.append('  <link name="l_f"/>')
        lines.append(
            '  <joint name="j_f" type="fixed"><parent link="l_0"/><child link="l_f"/></joint>'
        )
    lines.append("</robot>")

    (root / "urdf").mkdir(parents=True, exist_ok=True)
    (root / "urdf" / f"{dataset_id}.urdf").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (root / "finaljson").mkdir(parents=True, exist_ok=True)
    (root / "finaljson" / f"{dataset_id}.json").write_text(
        json.dumps({"category": category}), encoding="utf-8"
    )


def _build_row(runner, root: Path, dataset_id: int, rank: int) -> dict:
    metadata = runner.TABLE5.parse_urdf_metadata(root, dataset_id)
    category = json.loads(
        (root / "finaljson" / f"{dataset_id}.json").read_text(encoding="utf-8")
    )["category"]
    return {
        "dataset_id": dataset_id,
        "rank": rank,
        "rank_sha256": runner.TABLE5.rank_sha256(dataset_id),
        "category": category,
        "urdf_relative_path": f"urdf/{dataset_id}.urdf",
        "urdf_sha256": runner.TABLE5.sha256_file(root / "urdf" / f"{dataset_id}.urdf"),
        "finaljson_relative_path": f"finaljson/{dataset_id}.json",
        "finaljson_sha256": runner.TABLE5.sha256_file(
            root / "finaljson" / f"{dataset_id}.json"
        ),
        **metadata,
    }


def _build_manifest(runner, root: Path, dataset_ids: list[int]) -> tuple[dict, dict]:
    candidates = sorted(set(dataset_ids))
    selected = runner.TABLE5.select_ids(candidates, len(candidates))
    rows = [_build_row(runner, root, dataset_id, rank) for rank, dataset_id in enumerate(selected, start=1)]
    protocol = {"schema_version": "synthetic-table1-physx-protocol-v1"}
    manifest = {
        "schema_version": "table5_physx_mobility_manifest_v1",
        "selection": {
            "rank_salt": runner.TABLE5.RANK_SALT,
            "candidate_count": len(candidates),
            "selected_count": len(rows),
            "selection_rule": "ascending (rank_sha256, integer dataset_id), first N",
        },
        "source_receipt": {"synthetic": True},
        "protocol_sha256": runner.TABLE5.protocol_with_hash(protocol)["protocol_sha256"],
        "rows": rows,
    }
    manifest["cohort_sha256"] = runner.TABLE5.canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    )
    return manifest, protocol


def _dataset_root(tmp_path: Path) -> Path:
    root = tmp_path / "PhysX_mobility"
    root.mkdir()
    return root


def test_evaluate_asset_reports_declared_structure(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    _write_asset(root, 10, category="Chair", movable=1, fixed_extra=True)
    row = _build_row(runner, root, 10, rank=1)

    record = runner.evaluate_asset(root, row, tmp_path / "staging")

    assert record["status"] == "EVALUATED"
    assert record["parse_success"] is True
    assert record["link_count"] == 3
    assert record["joint_count"] == 2
    assert record["non_fixed_joint_count"] == 1
    assert record["joint_type_counts"] == {"fixed": 1, "revolute": 1}
    assert record["valid_tree"] is True
    assert isinstance(record["topology_hash"], str) and len(record["topology_hash"]) == 64
    assert record["fingerprint_complete"] is True
    assert record["referenced_resource_count"] == 3  # obj + mtl + png
    assert record["missing_resources"] == []
    assert record["raw_category"] == "Chair"
    assert record["manifest_row_sha256"] == runner.TABLE5.canonical_sha256(row)


def test_duplicate_assets_share_fingerprint_and_topology(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    _write_asset(root, 10, category="Chair")
    _write_asset(root, 20, category="Chair")
    row_a = _build_row(runner, root, 10, rank=1)
    row_b = _build_row(runner, root, 20, rank=2)

    record_a = runner.evaluate_asset(root, row_a, tmp_path / "staging")
    record_b = runner.evaluate_asset(root, row_b, tmp_path / "staging")

    assert record_a["status"] == record_b["status"] == "EVALUATED"
    assert record_a["package_fingerprint"] == record_b["package_fingerprint"]
    assert record_a["topology_hash"] == record_b["topology_hash"]


def test_missing_texture_marks_fingerprint_incomplete(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    _write_asset(root, 10, category="Chair", texture=True)
    row = _build_row(runner, root, 10, rank=1)
    row["resources"] = [
        resource
        for resource in row["resources"]
        if not resource["relative_path"].endswith(".png")
    ]

    record = runner.evaluate_asset(root, row, tmp_path / "staging")

    assert record["status"] == "EVALUATED_FINGERPRINT_INCOMPLETE"
    assert record["parse_success"] is True
    assert record["fingerprint_complete"] is False
    assert record["package_fingerprint"] is None
    assert record["missing_resources"] == [f"partseg/10/objs/10.png"]


def test_xml_count_drift_fails_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    _write_asset(root, 10, category="Chair")
    row = _build_row(runner, root, 10, rank=1)
    row["xml_counts"] = {**row["xml_counts"], "links": row["xml_counts"]["links"] + 1}

    record = runner.evaluate_asset(root, row, tmp_path / "staging")

    assert record["status"] == "EVALUATION_FAILED"
    assert record["parse_success"] is False
    assert "drifted" in record["error"]


def test_validate_cohort_accepts_frozen_synthetic_manifest(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    for dataset_id in (10, 20, 30):
        _write_asset(root, dataset_id, category="Chair")
    manifest, protocol = _build_manifest(runner, root, [10, 20, 30])

    runner.validate_cohort(manifest, protocol, root, verify_archive=False)

    manifest["rows"][0]["resources"][0]["sha256"] = "0" * 64
    with pytest.raises(runner.TABLE5.ManifestError, match="resource hash mismatch"):
        runner.validate_cohort(manifest, protocol, root, verify_archive=False)


def test_release_roster_counts_raw_and_normalized_categories(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    labels = {10: "Chair", 20: "chair", 30: "Chair ", 40: "Table"}
    for dataset_id, category in labels.items():
        _write_asset(root, dataset_id, category=category)

    roster = runner.release_roster(root, expected_count=4)

    assert roster["candidate_ids"] == [10, 20, 30, 40]
    assert roster["categories"] == labels
    assert len(set(roster["categories"].values())) == 4
    assert len({runner._normalize_category(v) for v in roster["categories"].values()}) == 2


def test_aggregate_physx_records_reports_rates(tmp_path: Path) -> None:
    runner = _load_runner()
    root = _dataset_root(tmp_path)
    _write_asset(root, 10, category="Chair", movable=1, fixed_extra=True)
    _write_asset(root, 20, category="Chair", movable=1, fixed_extra=True)
    _write_asset(root, 30, category="Table", movable=2, fixed_extra=False)
    rows = [_build_row(runner, root, dataset_id, rank) for rank, dataset_id in enumerate((10, 20, 30), start=1)]

    records = [runner.evaluate_asset(root, row, tmp_path / "staging") for row in rows]
    summary = runner.aggregate_physx_records(
        records,
        release_asset_count=4,
        release_category_count=3,
        roster_categories={10: "Chair", 20: "Chair", 30: "Table", 40: "chair"},
    )

    cohort = summary["cohort"]
    assert cohort["N_release"] == 4
    assert cohort["N_eval"] == 3
    assert cohort["N_parse"] == 3
    assert cohort["cohort_type"] == runner.COHORT_TYPE
    assert summary["links_per_asset"]["denominator"] == 3
    assert summary["movable_joints_per_asset"]["mean"] == pytest.approx(4 / 3)
    assert summary["multi_joint_assets"]["numerator"] == 1
    assert summary["multi_joint_assets"]["denominator"] == 3
    assert summary["unique_topologies"]["unique"] == 2
    assert summary["unique_topologies"]["denominator"] == 3
    assert summary["exact_duplicate_rate"]["duplicate_excess"] == 1
    assert summary["exact_duplicate_rate"]["denominator"] == 3
    assert summary["declared_collision_element_total"] == 0
    census = summary["category_census"]
    assert census["release_raw_categories"] == 3
    assert census["release_normalized_categories"] == 2
    assert census["eval_raw_categories"] == 2
    assert census["eval_normalized_categories"] == 2


@pytest.mark.skipif(not RECEIPT_SET.is_dir(), reason="frozen receipt set not present")
def test_real_receipt_set_is_bound() -> None:
    runner = _load_runner()
    protocol, manifest = runner.load_receipt_set(RECEIPT_SET)

    assert manifest["schema_version"] == runner.EXPECTED_MANIFEST_SCHEMA
    assert len(manifest["rows"]) == 800
    assert manifest["selection"]["candidate_count"] == runner.EXPECTED_RELEASE_ASSET_COUNT
    assert manifest["selection"]["selected_count"] == 800
    assert manifest["cohort_sha256"] == runner.TABLE5.canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    )
    assert manifest["protocol_sha256"] == protocol["protocol_sha256"]
    ids = [row["dataset_id"] for row in manifest["rows"]]
    assert len(set(ids)) == 800
