from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPO = Path("/mnt/zsn/lyb/arti-skill")
RUNNER = REPO / "exp/scripts/run_table7_artiverse_n54.py"
PARENT_RECORDS = REPO / "exp/runtime/table7_artiverse/asset_records.json"
EXPECTED_COHORT_SHA256 = "ed2b4076dbb142d21932be3dbb715426b3d77d5c39b0c7444999d16ba8b128b6"


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _run(output: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(RUNNER), "--output", str(output), *extra],
        cwd=REPO,
        check=False,
        capture_output=True,
        text=True,
    )


@pytest.fixture
def workspace_output(tmp_path: Path):
    output = REPO / "exp/runtime/table7_artiverse_n54_pytest" / tmp_path.name
    shutil.rmtree(output, ignore_errors=True)
    yield output
    shutil.rmtree(output, ignore_errors=True)


def test_cli_freezes_exact_outcome_independent_n54_cohort(workspace_output: Path) -> None:
    output = workspace_output
    completed = _run(output)
    assert completed.returncode == 0, completed.stdout + completed.stderr

    manifest = _read_json(output / "manifest.json")
    records = _read_json(output / "asset_records.json")
    selected_ids = [row["asset_id"] for row in manifest["assets"]]
    selected_id_bytes = "".join(f"{asset_id}\n" for asset_id in selected_ids).encode("utf-8")

    assert len(selected_ids) == len(set(selected_ids)) == 54
    assert hashlib.sha256(selected_id_bytes).hexdigest() == EXPECTED_COHORT_SHA256
    assert [row["asset_id"] for row in records] == selected_ids
    assert manifest["selection_policy"] == {
        "algorithm": "sha256(salt + NUL + full asset_id), ascending by (digest, asset_id)",
        "missing_or_failed_assets_retained": True,
        "outcome_based_filtering": False,
        "requested_assets": 54,
        "salt": "nano3d-table7-artiverse-n54-v1",
    }

    parent_by_id = {row["asset_id"]: row for row in _read_json(PARENT_RECORDS)}
    assert records == [parent_by_id[asset_id] for asset_id in selected_ids]


def test_cli_aggregates_n54_with_explicit_asset_and_mesh_component_denominators(
    workspace_output: Path,
) -> None:
    output = workspace_output
    completed = _run(output)
    assert completed.returncode == 0, completed.stdout + completed.stderr

    summary = _read_json(output / "summary.json")
    geometry = summary["results"]["geometry"]
    assert summary["cohort"] == {
        "available_assets": 54,
        "category_count": 23,
        "geometry_evaluable_assets": 54,
        "package_evaluable_assets": 54,
        "requested_assets": 54,
        "selection": "fixed salted-SHA256 rank over the frozen 3,544-asset manifest; no outcome filtering",
        "source_repository_count": 10,
        "unavailable_assets": 0,
    }
    assert geometry["readable_geometries"] == 547
    assert geometry["watertight"]["geometry_level"] == {
        "denominator": 547,
        "numerator": 6,
        "rate": 6 / 547,
    }
    assert geometry["manifold"]["geometry_level"] == {
        "denominator": 547,
        "numerator": 546,
        "rate": 546 / 547,
    }
    assert geometry["open_edges"]["total"] == 666_748
    assert geometry["degenerate_faces"]["total"] == 305

    results = summary["results"]
    assert results["portable_package"]["pass"] == 54
    assert results["semantic_complete"]["not_evaluable"] == 54
    assert results["semantic_field_proxy"]["pass"] == 53
    assert results["kinematic_complete"]["pass"] == 54
    assert results["physical_complete"]["fail"] == 54
    for name in (
        "portable_package",
        "deterministic_build",
        "semantic_complete",
        "semantic_field_proxy",
        "kinematic_complete",
        "physical_complete",
    ):
        row = results[name]
        assert row["pass"] + row["fail"] + row["not_evaluable"] == 54


def test_verify_only_rejects_artifact_drift(workspace_output: Path) -> None:
    output = workspace_output
    created = _run(output)
    assert created.returncode == 0, created.stdout + created.stderr

    verified = _run(output, "--verify-only")
    assert verified.returncode == 0, verified.stdout + verified.stderr
    assert '"status": "PASS"' in verified.stdout

    report = output / "report.md"
    report.write_text(report.read_text(encoding="utf-8") + "drift\n", encoding="utf-8")
    rejected = _run(output, "--verify-only")
    assert rejected.returncode != 0
    assert "artifact hash mismatch: report.md" in rejected.stdout
