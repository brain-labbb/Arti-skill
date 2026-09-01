from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import run_table2sup_full_release as runner  # noqa: E402


def _fixture_manifest(tmp_path: Path) -> Path:
    package = tmp_path / "asset"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(
        '<robot name="fixture"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>\n',
        encoding="utf-8",
    )
    row = {
        "asset_id": "asset-0",
        "category": "fixture",
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "asset/model.urdf",
        "primary_urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "joint_count": 0,
    }
    manifest = {
        "schema_version": runner.ROSTER_SCHEMA,
        "dataset": "Fixture",
        "N_eval": 1,
        "J_eval": 0,
        "rows": [row],
    }
    path = tmp_path / "full_release_manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_run_dataset_publishes_bound_receipt_and_resumes(tmp_path: Path) -> None:
    roster = _fixture_manifest(tmp_path)
    output = tmp_path / "run"
    runner.run_dataset(roster, output, workers=2, timeout_seconds=30, source_kind="fixture")
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["n_eval"] == 1
    assert json.loads((output / "checkpoint.json").read_text())["state"] == "complete"
    assert json.loads((output / "artifact_manifest.json").read_text())["schema_version"] == "table123_artifact_manifest_v1"
    # A completed run is a no-op on resume and must preserve its immutable
    # manifest hash rather than creating a new timestamped identity.
    before = json.loads((output / "manifest.json").read_text(encoding="utf-8"))["manifest_content_sha256"]
    runner.run_dataset(roster, output, workers=2, timeout_seconds=30, resume=True, source_kind="fixture")
    after = json.loads((output / "manifest.json").read_text(encoding="utf-8"))["manifest_content_sha256"]
    assert after == before
