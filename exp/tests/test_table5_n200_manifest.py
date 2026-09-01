from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "exp/scripts/table5_n200_manifest.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("table5_n200_manifest_tested", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MANIFEST = _load_module()

VALID_URDF = """\
<robot name="fixture">
  <link name="base"/>
  <link name="door"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="door"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="2" velocity="3"/>
  </joint>
</robot>
"""


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def _joint_metadata() -> tuple[dict, list[dict]]:
    joint = {
        "name": "hinge",
        "type": "revolute",
        "parent": "base",
        "child": "door",
        "origin_xyz": [0.0, 0.0, 0.0],
        "origin_rpy": [0.0, 0.0, 0.0],
        "axis": [0.0, 0.0, 1.0],
        "fk_supported": True,
        "lower": 0.0,
        "upper": 1.0,
        "effort": 2.0,
        "velocity": 3.0,
    }
    return {"links": ["base", "door"], "root_links": ["base"], "joints": [joint]}, [
        joint
    ]


def _complete_table5_row(
    dataset_id: str,
    package_relative: str,
    urdf_relative: str,
    urdf_sha256: str,
    *,
    asset_id: str | None = None,
) -> dict:
    tree, scalar = _joint_metadata()
    row = {
        "dataset_id": dataset_id,
        "category": "fixture_category",
        "package_relative_path": package_relative,
        "urdf_relative_path": urdf_relative,
        "urdf_sha256": urdf_sha256,
        "joint_tree": tree,
        "scalar_joints": scalar,
        "bounding_box_diagonal": 2.0,
        "strict_gates": {
            "table2": {"strict_urdf_pass": True},
            "table3": {"strict_kinematic_pass": False},
            "table4": {"strict_collision_pass": True},
        },
        "preflight": {"status": "pass", "issues": [], "simulator_eligible": True},
        "reused_marker": "preserved",
    }
    if asset_id is not None:
        row["asset_id"] = asset_id
    row["row_sha256"] = MANIFEST.canonical_sha256(row)
    return row


def _upstream_rows(key: str, urdf_hash: str, bbox: float) -> dict[str, dict]:
    return {
        "table2": {
            "asset_id": key,
            "asset_key": key,
            "model_urdf_sha256": urdf_hash,
            "strict_urdf_pass": False,
            "metrics": {"parse_rate": {"pass": True}},
        },
        "table3": {
            "asset_id": key,
            "asset_key": key,
            "urdf_sha256": urdf_hash,
            "strict_kinematic_pass": True,
            "tree_valid": True,
        },
        "table4": {
            "asset_id": key,
            "asset_key": key,
            "model_urdf_sha256": urdf_hash,
            "strict_collision_pass": False,
            "load_success": True,
            "measurement_complete": True,
            "object_bbox_diagonal_m": bbox,
        },
    }


def test_build_manifest_keeps_prefix_failures_and_binds_evidence(
    tmp_path: Path,
) -> None:
    parent_paths: dict[str, Path] = {}
    roots: dict[str, Path] = {}
    upstream_paths: dict[str, dict[str, Path]] = {}
    categories = tmp_path / "categories"

    articraft_root = tmp_path / "articraft"
    articraft_bad = articraft_root / "a0"
    articraft_good = articraft_root / "a1"
    articraft_bad.mkdir(parents=True)
    articraft_good.mkdir(parents=True)
    (articraft_good / "model.urdf").write_text(VALID_URDF, encoding="utf-8")
    articraft_rows = []
    for asset_id, package in (("a0", articraft_bad), ("a1", articraft_good)):
        urdf_hash = (
            MANIFEST.sha256_file(package / "model.urdf")
            if (package / "model.urdf").is_file()
            else "0" * 64
        )
        articraft_rows.append(
            {
                "asset_id": asset_id,
                "package": str(package),
                "model_urdf_sha256": urdf_hash,
                "selection_index": len(articraft_rows),
            }
        )
        _write_json(
            categories / asset_id / "record.json",
            {"record_id": asset_id, "category_slug": "fixture_category"},
        )
    parent_paths["articraft_10k"] = tmp_path / "parents/articraft.json"
    _write_json(parent_paths["articraft_10k"], {"records": articraft_rows})
    roots["articraft_10k"] = articraft_root

    lam_root = tmp_path / "lam"
    lam_package = lam_root / "object0"
    lam_package.mkdir(parents=True)
    lam_urdf = lam_package / "generated.urdf"
    lam_urdf.write_text(VALID_URDF, encoding="utf-8")
    lam_hash = MANIFEST.sha256_file(lam_urdf)
    lam_key = "viable:objects/object0"
    parent_paths["lam_released_outputs"] = tmp_path / "parents/lam.json"
    _write_json(
        parent_paths["lam_released_outputs"],
        {
            "records": [
                {
                    "asset_key": lam_key,
                    "category": "fixture_category",
                    "urdf_path": str(lam_urdf),
                    "urdf_sha256": lam_hash,
                }
            ]
        },
    )
    roots["lam_released_outputs"] = lam_root

    sketch_root = tmp_path / "sketch"
    sketch_id = "data/source/category/s0"
    sketch_package = sketch_root / sketch_id
    sketch_package.mkdir(parents=True)
    sketch_urdf = sketch_package / "mobility.urdf"
    sketch_urdf.write_text(VALID_URDF, encoding="utf-8")
    sketch_hash = MANIFEST.sha256_file(sketch_urdf)
    parent_paths["sketchmobility"] = tmp_path / "parents/sketch.json"
    _write_json(
        parent_paths["sketchmobility"],
        {
            "assets": [
                {
                    "asset_id": sketch_id,
                    "source_category": "source/category",
                    "mobility_urdf_sha256": sketch_hash,
                }
            ]
        },
    )
    roots["sketchmobility"] = sketch_root

    generated = {
        "articraft_10k": ("a0", articraft_rows[0]["model_urdf_sha256"]),
        "lam_released_outputs": (lam_key, lam_hash),
        "sketchmobility": (sketch_id, sketch_hash),
    }
    for slug, (key, urdf_hash) in generated.items():
        records = _upstream_rows(key, urdf_hash, 1.5)
        if slug == "lam_released_outputs":
            records["table3"]["urdf_sha256"] = "f" * 64
        upstream_paths[slug] = {}
        for table, record in records.items():
            path = tmp_path / "upstream" / slug / f"{table}.jsonl"
            _write_jsonl(path, [record])
            upstream_paths[slug][table] = path

    artiverse_root = tmp_path / "artiverse"
    artiverse_package = artiverse_root / "asset/package"
    artiverse_package.mkdir(parents=True)
    artiverse_urdf = artiverse_package / "asset.urdf"
    artiverse_urdf.write_text(VALID_URDF, encoding="utf-8")
    artiverse_hash = MANIFEST.sha256_file(artiverse_urdf)
    parent_paths["artiverse"] = tmp_path / "parents/artiverse.json"
    _write_json(
        parent_paths["artiverse"],
        {
            "rows": [
                _complete_table5_row(
                    "artiverse_0000",
                    "asset/package",
                    "asset/package/asset.urdf",
                    artiverse_hash,
                    asset_id="asset/source/id",
                )
            ]
        },
    )
    roots["artiverse"] = artiverse_root

    partnet_root = tmp_path / "partnet"
    partnet_package = partnet_root / "100"
    partnet_package.mkdir(parents=True)
    partnet_urdf = partnet_package / "mobility.urdf"
    partnet_urdf.write_text(VALID_URDF, encoding="utf-8")
    partnet_hash = MANIFEST.sha256_file(partnet_urdf)
    parent_paths["partnet_mobility"] = tmp_path / "parents/partnet.json"
    _write_json(
        parent_paths["partnet_mobility"],
        {
            "rows": [
                _complete_table5_row("100", "100", "100/mobility.urdf", partnet_hash)
            ]
        },
    )
    roots["partnet_mobility"] = partnet_root

    physx_root = tmp_path / "physx"
    (physx_root / "urdf").mkdir(parents=True)
    physx_urdf = physx_root / "urdf/200.urdf"
    physx_urdf.write_text(VALID_URDF, encoding="utf-8")
    physx_hash = MANIFEST.sha256_file(physx_urdf)
    physx_row = _complete_table5_row("200", ".", "urdf/200.urdf", physx_hash)
    physx_row.pop("strict_gates")
    parent_paths["physx_mobility"] = tmp_path / "parents/physx.json"
    _write_json(parent_paths["physx_mobility"], {"rows": [physx_row]})
    roots["physx_mobility"] = physx_root

    manifest = MANIFEST.build_manifest(
        sample_size=1,
        parent_paths=parent_paths,
        dataset_roots=roots,
        articraft_category_records_root=categories,
        upstream_paths=upstream_paths,
    )
    MANIFEST.validate_manifest(manifest)

    assert manifest["total_rows"] == 6
    datasets = {entry["dataset_slug"]: entry for entry in manifest["datasets"]}
    articraft = datasets["articraft_10k"]
    assert articraft["rows"][0]["asset_id"] == "a0"
    assert articraft["rows"][0]["source_parent"]["order"] == 0
    assert articraft["rows"][0]["preflight"]["status"] == "failed"
    assert articraft["rows"][0]["dataset_id"] == "articraft_0000"
    assert articraft["rows"][0]["strict_gates"]["table2"]["strict_urdf_pass"] is False
    assert articraft["rows"][0]["bounding_box_diagonal"] == 1.5
    assert articraft["selection"]["replacement"] is False
    assert articraft["selection"]["outcome_filtering"] is False
    assert articraft["parent"]["sha256"] == MANIFEST.sha256_file(
        parent_paths["articraft_10k"]
    )
    lam = datasets["lam_released_outputs"]["rows"][0]
    assert lam["strict_gates"]["table2"]["strict_urdf_pass"] is False
    assert lam["strict_gates"]["table3"]["strict_kinematic_pass"] is None
    assert lam["strict_gates"]["table3"]["availability"] == "not_available"
    assert "hash mismatch" in lam["strict_gates"]["table3"]["reason"]

    artiverse = datasets["artiverse"]["rows"][0]
    assert artiverse["reused_marker"] == "preserved"
    assert artiverse["source_table5_row_sha256"]
    physx = datasets["physx_mobility"]["rows"][0]
    assert physx["strict_gates"]["table4"]["strict_collision_pass"] is None
    assert physx["strict_gates"]["table4"]["availability"] == "not_available"

    tampered = json.loads(json.dumps(manifest))
    tampered["datasets"][0]["rows"][0]["category"] = "changed"
    try:
        MANIFEST.validate_manifest(tampered, verify_parent_files=False)
    except MANIFEST.ManifestError as error:
        assert "self-hash mismatch" in str(error)
    else:
        raise AssertionError("tampered manifest must fail self-hash validation")


def test_sample_size_must_fit_every_parent(tmp_path: Path) -> None:
    try:
        MANIFEST.build_manifest(sample_size=0)
    except MANIFEST.ManifestError as error:
        assert "positive integer" in str(error)
    else:
        raise AssertionError("sample_size=0 must fail")


def test_real_six_dataset_sample_one_smoke() -> None:
    manifest = MANIFEST.build_manifest(sample_size=1)
    MANIFEST.validate_manifest(manifest)

    assert manifest["total_rows"] == 6
    assert [dataset["parent"]["parent_count"] for dataset in manifest["datasets"]] == [
        800,
        800,
        800,
        800,
        800,
        800,
    ]
    sketch = next(
        dataset
        for dataset in manifest["datasets"]
        if dataset["dataset_slug"] == "sketchmobility"
    )
    assert (
        sketch["parent"]["sha256"]
        == "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
    )
    physx = next(
        dataset
        for dataset in manifest["datasets"]
        if dataset["dataset_slug"] == "physx_mobility"
    )
    assert physx["rows"][0]["strict_gates"]["table4"] == {
        "strict_collision_pass": None,
        "availability": "not_available",
        "reason": "not bound by the reused Table 5 source row",
    }
