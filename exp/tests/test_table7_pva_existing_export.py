from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import sys
import types
from pathlib import Path

import pytest


REPO = Path("/mnt/zsn/lyb/arti-skill")
RUNNER = REPO / "exp/scripts/run_table7_pva_existing_export.py"


def _load_runner():
    sys.modules.setdefault("trimesh", types.ModuleType("trimesh"))
    spec = importlib.util.spec_from_file_location("table7_pva_existing_export", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def test_mesh_payload_mutation_is_rejected(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    runner = _load_runner()
    original_manifest = json.loads(runner.COPY_MANIFEST.read_text(encoding="utf-8"))
    copied_root = tmp_path / "input_packages"
    copied_root.mkdir()
    copied_manifest = []

    for row in original_manifest:
        source = Path(row["copied_package"])
        destination = copied_root / source.name
        destination.mkdir()
        shutil.copy2(source / "model.urdf", destination / "model.urdf")
        shutil.copytree(source / "assets", destination / "assets")
        copied_manifest.append({**row, "copied_package": str(destination)})

    copied_manifest_path = copied_root / "input_manifest.json"
    copied_manifest_path.write_text(
        json.dumps(copied_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    expected_input_hashes = {
        **runner.EXPECTED_INPUT_HASHES,
        "copy_manifest": _sha256(copied_manifest_path),
    }
    monkeypatch.setattr(runner, "COPY_ROOT", copied_root)
    monkeypatch.setattr(runner, "COPY_MANIFEST", copied_manifest_path)
    monkeypatch.setattr(runner, "EXPECTED_INPUT_HASHES", expected_input_hashes)

    runner.validate_inputs()
    target_mesh = next(copied_root.rglob("*.obj"))
    with target_mesh.open("a", encoding="utf-8") as handle:
        handle.write("\n# provenance mutation\n")

    with pytest.raises(RuntimeError, match="mesh inventory hash mismatch"):
        runner.validate_inputs()
