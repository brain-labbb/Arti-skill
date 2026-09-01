from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import table5_articraft_github_parent as parent


def _git(repo: Path, *arguments: str) -> None:
    completed = subprocess.run(
        [
            "git",
            "-C",
            str(repo),
            "-c",
            "user.name=Table5 Test",
            "-c",
            "user.email=table5@example.invalid",
            *arguments,
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def _fixture(tmp_path: Path, record_ids: list[str]) -> tuple[Path, Path]:
    source_root = tmp_path / "source"
    materialized_root = tmp_path / "materialized"
    source_root.mkdir()
    materialized_root.mkdir()
    rows = []
    for index, record_id in enumerate(record_ids):
        revision = "rev_000001"
        record_root = source_root / "records" / record_id
        model_root = record_root / "revisions" / revision
        model_root.mkdir(parents=True)
        model_path = model_root / "model.py"
        model_path.write_text(f"VALUE = {index}\n", encoding="utf-8")
        model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
        record = {
            "record_id": record_id,
            "active_revision_id": revision,
            "category_slug": f"category_{index}",
            "hashes": {"model_py_sha256": model_hash},
        }
        (record_root / "record.json").write_text(
            json.dumps(record, sort_keys=True) + "\n", encoding="utf-8"
        )
        rows.append(
            {
                "schema_version": 1,
                "record_id": record_id,
                "active_revision_id": revision,
                "category_slug": f"category_{index}",
            }
        )
    source = source_root / "records_manifest.jsonl"
    source.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _git(source_root, "init", "-q")
    _git(source_root, "add", ".")
    _git(source_root, "commit", "-q", "-m", "fixture")
    return source, materialized_root


def _expected_order(source: Path, salt: str, record_ids: list[str]) -> list[str]:
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()

    def rank(record_id: str) -> str:
        payload = (
            salt.encode()
            + b"\0"
            + source_hash.encode("ascii")
            + b"\0"
            + record_id.encode()
        )
        return hashlib.sha256(payload).hexdigest()

    return [
        record_id for _, record_id in sorted((rank(item), item) for item in record_ids)
    ]


def test_deterministic_intersection_rank_and_source_membership(tmp_path: Path) -> None:
    source_ids = ["rec_a", "rec_b", "rec_c", "rec_d", "rec_source_only"]
    source, materialized = _fixture(tmp_path, source_ids)
    eligible = ["rec_a", "rec_b", "rec_c", "rec_d"]
    for record_id in eligible + ["rec_materialized_only"]:
        package = materialized / record_id
        package.mkdir()
        (package / "model.urdf").write_text(
            f'<robot name="{record_id}"/>\n', encoding="utf-8"
        )
    salt = "fixture-salt"
    first = parent.build_parent_manifest(
        source=source,
        materialized_root=materialized,
        sample_size=3,
        salt=salt,
    )
    second = parent.build_parent_manifest(
        source=source,
        materialized_root=materialized,
        sample_size=3,
        salt=salt,
    )
    expected = _expected_order(source, salt, eligible)[:3]
    assert first == second
    assert [row["asset_id"] for row in first["records"]] == expected
    assert first["selection"]["eligible_count"] == 4
    assert "rec_source_only" not in expected
    assert "rec_materialized_only" not in expected
    parent.validate_parent_manifest(first, verify_inputs=True)


def test_missing_urdf_is_retained_without_replacement(tmp_path: Path) -> None:
    source_ids = ["rec_a", "rec_b", "rec_c", "rec_d"]
    source, materialized = _fixture(tmp_path, source_ids)
    for record_id in source_ids:
        (materialized / record_id).mkdir()
    salt = "missing-urdf-salt"
    ranked = _expected_order(source, salt, source_ids)
    missing_selected = ranked[0]
    for record_id in source_ids:
        if record_id != missing_selected:
            (materialized / record_id / "model.urdf").write_text(
                f'<robot name="{record_id}"/>\n', encoding="utf-8"
            )
    manifest = parent.build_parent_manifest(
        source=source,
        materialized_root=materialized,
        sample_size=2,
        salt=salt,
    )
    assert [row["asset_id"] for row in manifest["records"]] == ranked[:2]
    missing = manifest["records"][0]
    assert missing["asset_id"] == missing_selected
    assert missing["model_urdf_sha256"] is None
    assert missing["materialization"]["status"] == "model_urdf_missing"
    assert manifest["selection"]["replacement"] is False
    assert manifest["selection"]["outcome_based_reselection"] is False
    assert manifest["selection"]["urdf_preflight_before_selection"] is False


def test_self_and_row_hashes_reject_tampering(tmp_path: Path) -> None:
    source, materialized = _fixture(tmp_path, ["rec_a", "rec_b"])
    for record_id in ("rec_a", "rec_b"):
        package = materialized / record_id
        package.mkdir()
        (package / "model.urdf").write_text(
            f'<robot name="{record_id}"/>\n', encoding="utf-8"
        )
    manifest = parent.build_parent_manifest(
        source=source,
        materialized_root=materialized,
        sample_size=1,
        salt="hash-salt",
    )
    parent.validate_parent_manifest(manifest)
    assert manifest["manifest_content_sha256"] == parent.canonical_sha256(
        manifest, exclude_fields=("manifest_content_sha256",)
    )

    tampered = deepcopy(manifest)
    tampered["records"][0]["category_slug"] = "changed"
    with pytest.raises(parent.ParentManifestError, match="self-hash"):
        parent.validate_parent_manifest(tampered)


def test_duplicate_source_record_id_is_rejected(tmp_path: Path) -> None:
    source, materialized = _fixture(tmp_path, ["rec_a", "rec_b"])
    first_line = source.read_text(encoding="utf-8").splitlines()[0]
    source.write_text(first_line + "\n" + first_line + "\n", encoding="utf-8")
    (materialized / "rec_a").mkdir()
    with pytest.raises(parent.ParentManifestError, match="duplicate record_id"):
        parent.build_parent_manifest(
            source=source,
            materialized_root=materialized,
            sample_size=1,
        )
