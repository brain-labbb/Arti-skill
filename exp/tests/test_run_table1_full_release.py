from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import run_table1_full_release as runner
import table123_full_release_common as common


URDF = """<robot name="fixture">
  <link name="base"/>
</robot>
"""


def _row(root: Path, asset_id: str, *, category: str = "cat") -> dict:
    package = root / asset_id
    package.mkdir(parents=True)
    path = package / "model.urdf"
    path.write_text(URDF, encoding="utf-8")
    return {
        "asset_id": asset_id,
        "source_path": str(package),
        "source_relative_path": f"nested/{asset_id}",
        "primary_urdf_path": str(path),
        "primary_urdf_relative_path": "model.urdf",
        "raw_category": category,
        "non_fixed_joints": [],
    }


def _roster(tmp_path: Path, rows: list[dict]) -> Path:
    path = tmp_path / "full_release_manifest.json"
    common.freeze_roster(rows, path, dataset="Fixture", source_bindings=[])
    return path


def test_audit_row_uses_shared_metrics_and_preserves_failure(tmp_path: Path) -> None:
    row = _row(tmp_path / "source", "ok", category="unusual")
    record = runner.audit_row(row)
    assert record["asset_id"] == "ok"
    assert record["raw_category"] == "unusual"
    assert record["status"] == "EVALUATED"
    broken = dict(row, asset_id="broken", primary_urdf_path=str(tmp_path / "missing.urdf"))
    failed = runner.audit_row(broken)
    assert failed["asset_id"] == "broken"
    assert failed["parse_success"] is False
    assert failed["status"] == "error"


def test_aggregate_keeps_full_dynamic_denominator_and_categories(tmp_path: Path) -> None:
    rows = [_row(tmp_path / "source", "z", category="b"), _row(tmp_path / "source", "a", category="a")]
    manifest = common.load_roster(_roster(tmp_path, rows))
    records = [runner.audit_row(row) for row in manifest["rows"]]
    summary = runner.aggregate_full_release(records, manifest)
    assert summary["n_eval"] == 2
    assert summary["cohort"]["N_release"] == 2
    assert set(summary["category_breakdown"]) == {"a", "b"}


def test_run_rejects_legacy_n800_manifest(tmp_path: Path) -> None:
    legacy = tmp_path / "legacy.json"
    legacy.write_text(json.dumps({"N_eval": 800, "rows": []}), encoding="utf-8")
    with pytest.raises(ValueError, match="full-release roster"):
        runner.run_full_release(legacy, tmp_path / "out", workers=1, timeout_seconds=1)


def test_run_is_ordered_resumable_and_artifact_closed(tmp_path: Path) -> None:
    roster = _roster(tmp_path, [_row(tmp_path / "source", "z"), _row(tmp_path / "source", "a")])
    output = runner.run_full_release(roster, tmp_path / "out", workers=1, timeout_seconds=5)
    records = [json.loads(line) for line in (output / "asset_records.jsonl").read_text().splitlines()]
    assert [row["asset_id"] for row in records] == ["a", "z"]
    checkpoint = json.loads((output / "checkpoint.json").read_text())
    assert checkpoint["state"] == "complete"
    common.verify_artifacts(output)
    runner.run_full_release(roster, output, workers=1, timeout_seconds=5, resume=True)


def test_resume_rejects_tampered_checkpoint(tmp_path: Path) -> None:
    roster = _roster(tmp_path, [_row(tmp_path / "source", "a")])
    output = runner.run_full_release(roster, tmp_path / "out", workers=1, timeout_seconds=5)
    checkpoint = output / "checkpoint.json"
    payload = json.loads(checkpoint.read_text())
    payload["state"] = "running"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="checkpoint"):
        runner.run_full_release(roster, output, workers=1, timeout_seconds=5, resume=True)


def test_run_rejects_source_drift(tmp_path: Path) -> None:
    row = _row(tmp_path / "source", "a")
    roster = _roster(tmp_path, [row])
    (Path(row["source_path"]) / "extra.bin").write_bytes(b"drift")
    with pytest.raises(ValueError, match="package.*binding.*drift"):
        runner.run_full_release(roster, tmp_path / "out", workers=1, timeout_seconds=5)


def test_evaluate_rows_drains_large_child_result(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    row = _row(tmp_path / "source", "large")

    def large_result(row: dict, run_standard_parser: bool) -> dict:
        del run_standard_parser
        return {"asset_id": row["asset_id"], "status": "EVALUATED", "payload": "x" * (2 * 1024 * 1024)}

    monkeypatch.setattr(runner, "audit_row", large_result)
    records: list[dict] = []
    runner._evaluate_rows(
        [row], workers=1, timeout_seconds=5, run_standard_parser=False, save=records.append
    )
    assert len(records) == 1
    assert records[0]["status"] == "EVALUATED"
    assert len(records[0]["payload"]) == 2 * 1024 * 1024
