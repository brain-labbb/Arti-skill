from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import sys
from typing import Any

import pytest


SCRIPT_DIR = Path(__file__).parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_pva_table4_mimic_aware_full_release as checker  # noqa: E402
import run_pva_table4_mimic_aware_full_release as runner  # noqa: E402


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture_row(logical_root: Path) -> dict[str, Any]:
    package = logical_root / "package"
    package.mkdir(parents=True)
    urdf = package / "model.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="mirror_fixture">
  <link name="base">
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    package_files = [
        {
            "path": "model.urdf",
            "size": urdf.stat().st_size,
            "sha256": _sha256_file(urdf),
        }
    ]
    return {
        "ordinal": 0,
        "asset_id": "fixture/seed_0000",
        "category": "fixture",
        "raw_category": "fixture",
        "joint_count": 0,
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": _sha256_file(urdf),
        "package_files": package_files,
        "package_binding_sha256": checker._canonical_sha256(package_files),
    }


def _source_evaluation(tmp_path: Path, row: dict[str, Any]) -> Path:
    source = tmp_path / "source_evaluation"
    source.mkdir()
    roster = {
        "schema_version": "pva_table1234_full_release_roster_v1",
        "N_eval": 1,
        "J_eval": 0,
        "source_bindings": {
            "extracted_root": str(Path(row["source_path"]).parent),
        },
    }
    roster["manifest_content_sha256"] = checker._self_hash(
        roster, "manifest_content_sha256"
    )
    roster_path = tmp_path / "roster_manifest.json"
    roster_path.write_text(json.dumps(roster), encoding="utf-8")
    roster_hash = _sha256_file(roster_path)

    database_path = source / "results.sqlite3"
    database = sqlite3.connect(database_path)
    database.executescript(
        """
        CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        """
    )
    meta = {
        "schema_version": runner.pva_run.RESULT_DB_SCHEMA_VERSION,
        "asset_import_state": "COMPLETE",
        "selected_n": 1,
        "selected_j": 0,
        "selected_category_count": 1,
        "roster_manifest_sha256": roster_hash,
        "roster_manifest_content_sha256": roster[
            "manifest_content_sha256"
        ],
    }
    database.executemany(
        "INSERT INTO meta(key, value) VALUES(?, ?)",
        [(key, runner._canonical_text(value)) for key, value in meta.items()],
    )
    row_text = runner._canonical_text(row)
    database.execute(
        "INSERT INTO assets VALUES(?, ?, ?, ?, ?, ?)",
        (
            0,
            row["asset_id"],
            row["raw_category"],
            0,
            hashlib.sha256(row_text.encode("utf-8")).hexdigest(),
            row_text,
        ),
    )
    database.commit()
    database.close()

    receipt = {
        "schema_version": runner.pva_run.RECEIPT_SCHEMA_VERSION,
        "N_eval": 1,
        "J_eval": 0,
        "eval_category_count": 1,
        "roster_manifest": str(roster_path),
        "roster_manifest_sha256": roster_hash,
        "roster_manifest_content_sha256": roster[
            "manifest_content_sha256"
        ],
        "result_database": "results.sqlite3",
        "result_database_sha256": _sha256_file(database_path),
    }
    receipt["receipt_content_sha256"] = checker._self_hash(
        receipt, "receipt_content_sha256"
    )
    (source / "full_release_receipt.json").write_text(
        json.dumps(receipt), encoding="utf-8"
    )
    return source


def _binding(logical_root: Path, physical_root: Path, roster_hash: str) -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema_version": checker.PACKAGE_ROOT_BINDING_SCHEMA,
        "mapping_policy": checker.PACKAGE_ROOT_MAPPING_POLICY,
        "logical_root": str(logical_root),
        "physical_root": str(physical_root),
        "roster_manifest_content_sha256": roster_hash,
        "package_verification": checker.PACKAGE_VERIFICATION,
    }
    value["binding_content_sha256"] = checker._self_hash(
        value, "binding_content_sha256"
    )
    return value


def test_package_root_binding_is_bound_to_sealed_logical_root(tmp_path: Path) -> None:
    logical_root = tmp_path / "logical"
    physical_root = tmp_path / "physical"
    logical_root.mkdir()
    physical_root.mkdir()
    roster_hash = "a" * 64
    roster = {
        "source_bindings": {"extracted_root": str(logical_root)},
        "manifest_content_sha256": roster_hash,
    }
    source = {"roster_manifest_content_sha256": roster_hash}
    binding = _binding(logical_root, physical_root, roster_hash)

    context = checker._package_root_context(
        {"package_root_binding": binding},
        {"package_root_binding": binding},
        roster,
        source,
    )
    assert context is not None
    assert context["physical_root"] == physical_root

    tampered = dict(binding)
    tampered["logical_root"] = str(tmp_path / "other_logical")
    tampered["binding_content_sha256"] = checker._self_hash(
        tampered, "binding_content_sha256"
    )
    with pytest.raises(checker.CheckError, match="sealed roster"):
        checker._package_root_context(
            {"package_root_binding": tampered},
            {"package_root_binding": tampered},
            roster,
            source,
        )

    broken_hash = dict(binding)
    broken_hash["physical_root"] = str(tmp_path / "retarget")
    with pytest.raises(checker.CheckError, match="self-hash"):
        checker._package_root_context(
            {"package_root_binding": broken_hash},
            {"package_root_binding": broken_hash},
            roster,
            source,
        )


def test_full_checker_uses_mirror_and_rejects_drift_symlink_and_retarget(
    tmp_path: Path,
) -> None:
    logical_root = tmp_path / "logical"
    row = _fixture_row(logical_root)
    source = _source_evaluation(tmp_path, row)
    mirror_root = tmp_path / "mirror"
    shutil.copytree(Path(row["source_path"]), mirror_root / "package")
    output = tmp_path / "output"

    runner.run_pva_table4_v2(
        source,
        output,
        workers=1,
        batch_size=1,
        package_root_override=mirror_root,
    )
    shutil.rmtree(logical_root)
    assert checker.check(output)["all_pass"] is True

    mirror_urdf = mirror_root / "package" / "model.urdf"
    original = mirror_urdf.read_bytes()
    mirror_urdf.write_bytes(original + b"\n<!-- drift -->\n")
    with pytest.raises(checker.CheckError, match="package binding drift"):
        checker.check(output)

    mirror_urdf.write_bytes(original)
    external = tmp_path / "external.urdf"
    external.write_bytes(original)
    mirror_urdf.unlink()
    mirror_urdf.symlink_to(external)
    with pytest.raises(checker.CheckError, match="symlink"):
        checker.check(output)

    mirror_urdf.unlink()
    mirror_urdf.write_bytes(original)
    real_root = tmp_path / "retargeted_mirror"
    mirror_root.rename(real_root)
    mirror_root.symlink_to(real_root, target_is_directory=True)
    with pytest.raises(checker.CheckError, match="physical package root contains a symlink"):
        checker.check(output)
