from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import table123_full_release_common as common
import verify_table123_full_release as verifier


def _fixture(tmp_path: Path, *, tables: tuple[str, ...] = ("table1",)) -> Path:
    dataset = tmp_path / "Fixture"
    source = tmp_path / "source"
    source.mkdir()
    rows = []
    for index, asset_id in enumerate(("asset-a", "asset-b")):
        package = source / asset_id
        package.mkdir()
        urdf = package / "model.urdf"
        urdf.write_text("<robot><link name='base'/></robot>\n", encoding="utf-8")
        rows.append({
            "asset_id": asset_id,
            "source_path": str(package),
            "source_relative_path": asset_id,
            "primary_urdf_path": str(urdf),
            "primary_urdf_relative_path": "model.urdf",
            "non_fixed_joints": [] if index == 0 else ["hinge"],
        })
    dataset.mkdir()
    manifest_path = dataset / "full_release_manifest.json"
    common.freeze_roster(rows, manifest_path, dataset="Fixture", source_bindings=[{"name": "fixture", "revision": "r1"}])
    roster = common.load_roster(manifest_path)
    for table in tables:
        output = dataset / table
        output.mkdir()
        records_name = "asset_records.jsonl" if table == "table2" else "records.jsonl"
        records = [
            {"asset_id": row["asset_id"], "ordinal": row["ordinal"], "status": "completed", "primary_urdf_sha256": row["primary_urdf_sha256"], "declared_joint_count": row["joint_count"], "metrics": {"parse_rate": {"pass": True}}}
            for row in roster["rows"]
        ]
        (output / records_name).write_text("".join(json.dumps(item, sort_keys=True) + "\n" for item in records), encoding="utf-8")
        (output / "roster_manifest.json").write_text(json.dumps(roster, sort_keys=True) + "\n", encoding="utf-8")
        protocol = {"dataset": "Fixture", "table": table, "roster_sha256": roster["roster_sha256"], "source_bindings": roster["source_bindings"]}
        (output / "protocol_snapshot.json").write_text(json.dumps(protocol, sort_keys=True) + "\n", encoding="utf-8")
        manifest = {"schema_version": "table123_full_release_run_v1", "dataset": "Fixture", "roster_sha256": roster["roster_sha256"], "roster_manifest_content_sha256": roster["manifest_content_sha256"], "N_eval": 2, "J_eval": 1}
        manifest["manifest_content_sha256"] = common.canonical_sha256(manifest)
        (output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8")
        summary = {"schema_version": "fixture", "dataset": "Fixture", "n_eval": 2, "j_eval": 1, "records_present": 2, "status_counts": {"completed": 2}, "metrics": {"parse_rate": {"pass_count": 2, "denominator": 2, "pass_rate": 1.0}}}
        (output / "summary.json").write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
        checkpoint = {"state": "complete", "n_eval": 2, "j_eval": 1, "roster_sha256": roster["roster_sha256"]}
        common.write_checkpoint(output / "checkpoint.json", checkpoint)
        paths = ["manifest.json", records_name, "roster_manifest.json", "protocol_snapshot.json", "summary.json", "checkpoint.json"]
        artifact = {"schema_version": common.ARTIFACT_SCHEMA_VERSION, "artifacts": [{"path": name, "size": (output / name).stat().st_size, "sha256": common.sha256_file(output / name)} for name in paths]}
        artifact["artifact_manifest_content_sha256"] = common.canonical_sha256(artifact)
        (output / "artifact_manifest.json").write_text(json.dumps(artifact, sort_keys=True) + "\n", encoding="utf-8")
    return dataset


def test_verify_dataset_receipts_reaggregates_complete_fixture(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path, tables=("table1", "table2", "table3"))
    result = verifier.verify_dataset_receipts(dataset)
    assert result["dataset"] == "Fixture"
    assert set(result["tables"]) == {"table1", "table2", "table3"}
    assert result["tables"]["table3"]["n_eval"] == 2


def test_table1_evaluated_status_is_not_counted_as_failure(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    roster = common.load_roster(dataset / "full_release_manifest.json")
    records = [
        {
            "asset_id": row["asset_id"],
            "ordinal": row["ordinal"],
            "status": "EVALUATED",
            "primary_urdf_sha256": row["primary_urdf_sha256"],
        }
        for row in roster["rows"]
    ]
    computed = verifier._independent_summary(records, roster, "table1")
    assert computed["asset_failure_count"] == 0


def test_table1_suffix_keeps_evaluated_status_success(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    roster = common.load_roster(dataset / "full_release_manifest.json")
    records = [
        {
            "asset_id": row["asset_id"],
            "ordinal": row["ordinal"],
            "status": "EVALUATED",
            "primary_urdf_sha256": row["primary_urdf_sha256"],
        }
        for row in roster["rows"]
    ]
    computed = verifier._independent_summary(records, roster, "table1_final")
    assert computed["asset_failure_count"] == 0


def test_rejects_artifact_hash_or_size_drift(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    table = dataset / "table1"
    (table / "records.jsonl").write_text((table / "records.jsonl").read_text() + "tampered\n", encoding="utf-8")
    with pytest.raises((ValueError, common.ManifestError), match="artifact|hash|size"):
        verifier.verify_dataset_receipts(dataset)


def test_rejects_record_count_and_order_mismatch(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    table = dataset / "table1"
    records = [json.loads(line) for line in (table / "records.jsonl").read_text().splitlines()]
    (table / "records.jsonl").write_text(json.dumps(records[:1]) + "\n", encoding="utf-8")
    with pytest.raises((ValueError, common.ManifestError), match="record|receipt|artifact"):
        verifier.verify_dataset_receipts(dataset)


def test_rejects_summary_mismatch(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    table = dataset / "table1"
    summary = json.loads((table / "summary.json").read_text())
    summary["n_eval"] = 999
    (table / "summary.json").write_text(json.dumps(summary) + "\n", encoding="utf-8")
    with pytest.raises((ValueError, common.ManifestError), match="summary|artifact"):
        verifier.verify_dataset_receipts(dataset)


def test_rejects_stale_protocol_snapshot(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    table = dataset / "table1"
    protocol = json.loads((table / "protocol_snapshot.json").read_text())
    protocol["roster_sha256"] = "stale"
    (table / "protocol_snapshot.json").write_text(json.dumps(protocol) + "\n", encoding="utf-8")
    with pytest.raises((ValueError, common.ManifestError), match="protocol|roster|binding|artifact"):
        verifier.verify_dataset_receipts(dataset)


def test_reaggregate_prefers_table_bound_roster_when_dataset_copy_is_stale(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path)
    table = dataset / "table1"
    # Keep the table-local roster produced by `_fixture`, but replace the
    # dataset-level copy with another valid manifest.  A receipt verifier must
    # follow the run manifest's roster hash instead of silently rejecting a
    # valid detached table when a stale convenience copy is present.
    stale_source = tmp_path / "stale-source"
    stale_row = {
        "asset_id": "stale",
        "source_path": str(stale_source),
        "source_relative_path": "stale",
        "primary_urdf_path": str(stale_source / "model.urdf"),
        "primary_urdf_relative_path": "model.urdf",
        "non_fixed_joints": [],
    }
    stale_source.mkdir()
    (stale_source / "model.urdf").write_text("<robot/>\n", encoding="utf-8")
    common.freeze_roster(
        [stale_row],
        dataset / "full_release_manifest.json",
        dataset="Fixture",
        source_bindings=[],
    )
    computed = verifier.reaggregate_table(table)
    assert computed["n_eval"] == 2


def test_verify_dataset_receipts_returns_table_bound_roster_when_copy_is_stale(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path, tables=("table1", "table2", "table3"))
    original = common.load_roster(dataset / "full_release_manifest.json")
    stale_source = tmp_path / "stale-source"
    stale_source.mkdir()
    (stale_source / "model.urdf").write_text("<robot/>\n", encoding="utf-8")
    common.freeze_roster(
        [
            {
                "asset_id": "stale",
                "source_path": str(stale_source),
                "source_relative_path": "stale",
                "primary_urdf_path": str(stale_source / "model.urdf"),
                "primary_urdf_relative_path": "model.urdf",
                "non_fixed_joints": [],
            }
        ],
        dataset / "full_release_manifest.json",
        dataset="Fixture",
        source_bindings=[],
    )
    result = verifier.verify_dataset_receipts(dataset)
    assert result["roster_sha256"] == original["roster_sha256"]


def test_render_full_release_rows_is_markdown_table(tmp_path: Path) -> None:
    dataset = _fixture(tmp_path, tables=("table1", "table2", "table3"))
    result = verifier.verify_dataset_receipts(dataset)
    rendered = verifier.render_full_release_rows(result)
    assert "Fixture" in rendered
    assert "table1" in rendered
    assert "N=2" in rendered
