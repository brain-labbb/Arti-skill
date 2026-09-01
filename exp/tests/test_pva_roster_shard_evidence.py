from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load_builder():
    path = SCRIPTS / "build_pva_full_release_roster.py"
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location("pva_roster_shard_evidence_builder", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _dataset(root: Path) -> Path:
    package = root / "extracted" / "Alpha" / "seed_0000"
    package.mkdir(parents=True)
    (root / "archives").mkdir()
    (root / "archives" / "Alpha.tar.zst").write_bytes(b"alpha archive")
    (package / "model.urdf").write_text(
        "<robot name=\"fixture\"><link name=\"base\"/></robot>\n",
        encoding="utf-8",
    )
    (package / "appearance.json").write_text("{}\n", encoding="utf-8")
    (package / "physics.json").write_text("{}\n", encoding="utf-8")
    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("slug", "stem", "seed", "asset_id", "overrides_json"),
        )
        writer.writeheader()
        writer.writerow(
            {
                "slug": "Alpha",
                "stem": "alpha",
                "seed": "0",
                "asset_id": "seed_0000",
                "overrides_json": "{}",
            }
        )
    return root


@pytest.fixture()
def frozen_roster(tmp_path: Path) -> tuple[object, Path]:
    builder = _load_builder()
    manifest_path = builder.build_roster(
        _dataset(tmp_path / "PV-A"),
        tmp_path / "roster",
        workers=1,
        expected_n=1,
        expected_categories=1,
    )
    return builder, manifest_path


def _json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_final_manifest_seals_each_shard_receipt_and_input(
    frozen_roster: tuple[object, Path],
) -> None:
    _builder, manifest_path = frozen_roster
    manifest = _json(manifest_path)
    shard = manifest["shards"][0]
    receipt_path = manifest_path.parent / shard["receipt"]
    receipt = _json(receipt_path)

    assert shard["receipt_bytes"] == receipt_path.stat().st_size
    assert shard["receipt_sha256"] == hashlib.sha256(receipt_path.read_bytes()).hexdigest()
    assert shard["receipt_content_sha256"] == receipt["receipt_content_sha256"]
    assert shard["shard_input_sha256"] == receipt["shard_input_sha256"]


def test_load_without_package_verification_rejects_missing_shard_receipt(
    frozen_roster: tuple[object, Path],
) -> None:
    builder, manifest_path = frozen_roster
    manifest = _json(manifest_path)
    receipt_path = manifest_path.parent / manifest["shards"][0]["receipt"]
    receipt_path.unlink()

    with pytest.raises((OSError, ValueError), match="shard.*receipt|receipt.*shard"):
        builder.load_roster_manifest(manifest_path, verify_rows=False)


def test_load_without_package_verification_rejects_tampered_shard_bytes(
    frozen_roster: tuple[object, Path],
) -> None:
    builder, manifest_path = frozen_roster
    manifest = _json(manifest_path)
    shard_path = manifest_path.parent / manifest["shards"][0]["path"]
    shard_path.write_bytes(shard_path.read_bytes() + b"\n")

    with pytest.raises(ValueError, match="shard hash mismatch"):
        builder.load_roster_manifest(manifest_path, verify_rows=False)


def test_load_recomputes_shard_input_from_bound_source_manifest(
    frozen_roster: tuple[object, Path],
) -> None:
    builder, manifest_path = frozen_roster
    manifest = _json(manifest_path)
    shard = manifest["shards"][0]
    receipt_path = manifest_path.parent / shard["receipt"]
    receipt = _json(receipt_path)

    shard_input = dict(receipt["shard_input"])
    shard_input["source_rows_sha256"] = "0" * 64
    shard_input["shard_input_sha256"] = builder.common.canonical_sha256(
        builder._without(shard_input, "shard_input_sha256")
    )
    receipt["shard_input"] = shard_input
    receipt["shard_input_sha256"] = shard_input["shard_input_sha256"]
    receipt["receipt_content_sha256"] = builder.common.canonical_sha256(
        builder._without(receipt, "receipt_content_sha256")
    )
    receipt_path.write_text(
        json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )

    shard["receipt_bytes"] = receipt_path.stat().st_size
    shard["receipt_sha256"] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
    shard["receipt_content_sha256"] = receipt["receipt_content_sha256"]
    shard["shard_input_sha256"] = receipt["shard_input_sha256"]
    manifest["manifest_content_sha256"] = builder.common.canonical_sha256(
        builder._without(manifest, "manifest_content_sha256")
    )
    manifest_path.write_text(
        json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="shard input binding mismatch"):
        builder.load_roster_manifest(manifest_path, verify_rows=False)
