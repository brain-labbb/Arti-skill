from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[2] / "exp" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
import render_mobility_uniform as runner  # noqa: E402


def _manifest(root: Path) -> Path:
    rows = []
    for ordinal, (category, asset_id) in enumerate((("alpha", "z"), ("alpha", "a"), ("beta", "b"))):
        package = root / category / asset_id
        package.mkdir(parents=True)
        urdf = package / "model.urdf"
        urdf.write_text('<robot name="r"><link name="root"/></robot>\n', encoding="utf-8")
        rows.append({
            "ordinal": ordinal,
            "asset_id": asset_id,
            "category": category,
            "source_path": str(package),
            "primary_urdf_relative_path": f"{category}/{asset_id}/model.urdf",
            "primary_urdf_sha256": runner.sha256(urdf),
            "primary_urdf_bytes": urdf.stat().st_size,
        })
    manifest = {
        "schema_version": "table123_full_release_manifest_v1",
        "dataset": "Test",
        "N_eval": len(rows),
        "rows": rows,
        "source_bindings": [{"name": "Test", "path": str(root)}],
    }
    body = dict(manifest)
    manifest["roster_sha256"] = runner.canonical_sha(rows)
    manifest["manifest_content_sha256"] = runner.canonical_sha(manifest)
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_selection_uses_hash_then_asset_id(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "source"
    path = _manifest(root)
    monkeypatch.setitem(runner.EXPECTED, "infinigen", ("Test", 3, 2))
    _, selected = runner.load_items("infinigen", path, tmp_path / "out", strict=False)
    alpha = min(("z", "a"), key=lambda value: (hashlib.sha256(value.encode()).hexdigest(), value))
    assert [(item["category"], item["asset_id"]) for item in selected] == [("alpha", alpha), ("beta", "b")]
    assert all(item["category_one_shot"] for item in selected)


def test_studio_matches_pva_contract() -> None:
    pva = json.loads(Path("/mnt/zsn/data/particulate/datasets/PV-A/renders/uniform531_studio_256_v1/render_config.json").read_text())
    assert runner.studio() == pva["studio"]


def test_manifest_hash_rejects_tamper(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "source"
    path = _manifest(root)
    payload = json.loads(path.read_text())
    payload["rows"][0]["asset_id"] = "tampered"
    path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setitem(runner.EXPECTED, "infinigen", ("Test", 3, 2))
    with pytest.raises(ValueError, match="manifest self-hash"):
        runner.load_items("infinigen", path, tmp_path / "out", strict=False)
