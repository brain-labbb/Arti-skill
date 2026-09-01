from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import build_table123_full_release_rosters as rosters


URDF = """<robot name="fixture"><link name="base"/><link name="door"/><joint name="hinge" type="revolute"><parent link="base"/><child link="door"/><axis xyz="0 0 1"/></joint></robot>"""


def _urdf(path: Path, text: str = URDF) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def test_discover_articraft_uses_record_directories_and_retains_malformed(tmp_path: Path) -> None:
    root = tmp_path / "released_urdf"
    _urdf(root / "rec_good" / "model.urdf")
    _urdf(root / "rec_bad" / "model.urdf", "<robot>")
    rows = rosters.discover_articraft(root)
    assert [row["asset_id"] for row in rows] == ["rec_bad", "rec_good"]
    assert rows[0]["primary_urdf_relative_path"] == "rec_bad/model.urdf"
    assert rows[0]["parse_status"] == "malformed"


def test_discover_lam_includes_all_manifest_tiers_and_binds_generated_urdf(tmp_path: Path) -> None:
    root = tmp_path / "lam"
    root.mkdir()
    rows = [
        {"object_release_id": "v", "category": "cabinet", "tier": "viable", "rel_path": "objects/cabinet/v"},
        {"object_release_id": "b", "category": "cabinet", "tier": "broken", "rel_path": "imperfect/cabinet/b"},
    ]
    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader(); writer.writerows(rows)
    _urdf(root / "released_outputs/objects/cabinet/v/generated.urdf")
    _urdf(root / "released_outputs/imperfect/cabinet/b/generated.urdf")
    discovered = rosters.discover_lam(root)
    assert [row["release_tier"] for row in discovered] == ["broken", "viable"]
    assert discovered[0]["asset_id"] == "b"


def test_discover_artiverse_keeps_malformed_xml(tmp_path: Path) -> None:
    root = tmp_path / "data"
    _urdf(root / "storage" / "good" / "model.urdf")
    _urdf(root / "storage" / "bad" / "model.urdf", "<robot>")
    rows = rosters.discover_artiverse(root)
    assert len(rows) == 2
    assert any(row["parse_status"] == "malformed" for row in rows)


def test_discover_physx_preserves_exact_raw_label(tmp_path: Path) -> None:
    root = tmp_path / "PhysX_mobility"
    _urdf(root / "urdf" / "0007.urdf")
    rows = rosters.discover_physx(root)
    assert rows[0]["asset_id"] == "0007"
    assert rows[0]["raw_category"] == "0007"


def test_discover_partnet_uses_mobility_urdf_and_meta_category(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    _urdf(root / "123" / "mobility.urdf")
    (root / "123" / "meta.json").write_text(json.dumps({"model_cat": "FoldingChair"}), encoding="utf-8")
    rows = rosters.discover_partnet(root)
    assert rows[0]["asset_id"] == "123"
    assert rows[0]["category"] == "FoldingChair"


def test_discover_sketch_uses_source_category_and_keeps_zero_joint_asset(tmp_path: Path) -> None:
    root = tmp_path / "data"
    _urdf(root / "PartNeXt" / "Table" / "table_0001" / "mobility.urdf", "<robot><link name='base'/></robot>")
    rows = rosters.discover_sketch(root)
    assert rows[0]["category"] == "Table"
    assert rows[0]["xml_counts"]["movable_joints"] == 0


def test_discover_infinigen_reads_extracted_urdfs(tmp_path: Path) -> None:
    root = tmp_path / "urdf"
    _urdf(root / "cabinet" / "0001" / "cabinet.urdf")
    rows = rosters.discover_infinigen(root)
    assert rows[0]["category"] == "cabinet"
    assert rows[0]["primary_urdf_relative_path"] == "cabinet/0001/cabinet.urdf"


def test_discover_infinite_follows_manifest_identity_order(tmp_path: Path) -> None:
    manifest = tmp_path / "cohort.json"
    manifest.write_text(json.dumps({"assets": [
        {"asset_id": "Factory/seed_002", "factory": "Factory", "seed": 2, "package_path": "pkg2", "urdf_relpath": "2/scene.urdf", "primary_urdf_sha256": "a" * 64},
        {"asset_id": "Factory/seed_001", "factory": "Factory", "seed": 1, "package_path": "pkg1", "urdf_relpath": "1/scene.urdf", "primary_urdf_sha256": "b" * 64},
    ]}), encoding="utf-8")
    rows = rosters.discover_infinite(manifest)
    assert [row["asset_id"] for row in rows] == ["Factory/seed_002", "Factory/seed_001"]


def test_build_roster_rejects_unknown_dataset(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="unknown dataset"):
        rosters.build_roster("Nonsense", source_root=tmp_path, output=tmp_path / "out.json")
