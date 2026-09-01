from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table1_articraft.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_articraft", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _package_binding(package: Path) -> dict[str, object]:
    files = []
    for current_raw, directory_names, file_names in os.walk(package):
        directory_names.sort()
        file_names.sort()
        current = Path(current_raw)
        for name in file_names:
            path = current / name
            files.append(
                {
                    "path": path.relative_to(package).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": _sha256_file(path),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": _canonical_sha256(files),
    }


def _write_package(root: Path, asset_id: str, *, category: str) -> tuple[Path, Path]:
    package = root / "released_urdf" / asset_id
    package.mkdir(parents=True)
    (package / "assets").mkdir()
    (package / "assets/mesh.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )
    (package / "model.urdf").write_text(
        """<robot name="fixture">
  <link name="root"><visual><geometry><mesh filename="assets/mesh.obj"/></geometry></visual></link>
  <link name="child"/>
  <joint name="joint" type="revolute"><parent link="root"/><child link="child"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    category_record = root / "records" / asset_id / "record.json"
    category_record.parent.mkdir(parents=True)
    category_record.write_text(
        json.dumps({"record_id": asset_id, "category_slug": category}),
        encoding="utf-8",
    )
    return package, category_record


def _write_manifest(path: Path, packages: list[Path]) -> Path:
    records = []
    for index, package in enumerate(packages):
        binding = _package_binding(package)
        records.append(
            {
                "selection_index": index,
                "asset_id": package.name,
                "package": str(package.resolve()),
                "model_urdf_sha256": _sha256_file(package / "model.urdf"),
                "package_binding": binding,
            }
        )
    asset_ids = sorted(package.name for package in packages)
    manifest = {
        "schema_version": "1.0.0",
        "dataset": "Articraft-10K",
        "classification": "FORMAL",
        "mode": "formal",
        "source": {
            "root": str(packages[0].parent.resolve()),
            "release_asset_count": len(packages),
            "release_asset_ids_sha256": _canonical_sha256(asset_ids),
            "repo_id": "fixture/articraft",
            "revision": "fixture-revision",
        },
        "selection": {
            "algorithm": "fixture frozen order",
            "seed": 20260813,
            "n_eval": len(packages),
            "selected_asset_ids_sha256": _canonical_sha256(
                [package.name for package in packages]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = _canonical_sha256(manifest)
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_load_frozen_cohort_preserves_manifest_order_and_joins_categories(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    second, _ = _write_package(tmp_path, "asset_b", category="category_two")
    first, _ = _write_package(tmp_path, "asset_a", category="category_one")
    manifest = _write_manifest(tmp_path / "manifest.json", [second, first])

    cohort = runner.load_frozen_cohort(
        manifest,
        source_root=tmp_path / "released_urdf",
        category_records_root=tmp_path / "records",
        expected_n=2,
    )

    assert [row["asset_id"] for row in cohort["assets"]] == ["asset_b", "asset_a"]
    assert [row["selection_index"] for row in cohort["assets"]] == [0, 1]
    assert [row["raw_category"] for row in cohort["assets"]] == [
        "category_two",
        "category_one",
    ]
    assert cohort["release_asset_count"] == 2
    assert cohort["release_category_count"] == 2
    assert cohort["eval_category_count"] == 2


@pytest.mark.parametrize("mutation", ["outside", "wrong_hash", "wrong_binding"])
def test_load_frozen_cohort_rejects_manifest_or_package_drift(
    tmp_path: Path,
    mutation: str,
) -> None:
    runner = _load_runner()
    package, _ = _write_package(tmp_path, "asset", category="category")
    manifest_path = _write_manifest(tmp_path / "manifest.json", [package])
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if mutation == "outside":
        manifest["records"][0]["package"] = str(tmp_path / "outside")
    elif mutation == "wrong_hash":
        manifest["records"][0]["model_urdf_sha256"] = "0" * 64
    else:
        manifest["records"][0]["package_binding"]["content_manifest_sha256"] = "0" * 64
    manifest["manifest_content_sha256"] = _canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError):
        runner.load_frozen_cohort(
            manifest_path,
            source_root=tmp_path / "released_urdf",
            category_records_root=tmp_path / "records",
            expected_n=1,
        )


def test_load_frozen_cohort_rejects_unselected_release_directory_symlink(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    selected, _ = _write_package(tmp_path, "selected", category="selected_category")
    unselected, _ = _write_package(tmp_path, "unselected", category="other_category")
    manifest_path = _write_manifest(tmp_path / "manifest.json", [selected])
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    release_ids = ["selected", "unselected"]
    manifest["source"]["release_asset_count"] = len(release_ids)
    manifest["source"]["release_asset_ids_sha256"] = _canonical_sha256(release_ids)
    manifest["manifest_content_sha256"] = _canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    external = tmp_path / "external"
    shutil.move(str(unselected), external)
    unselected.symlink_to(external, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        runner.load_frozen_cohort(
            manifest_path,
            source_root=tmp_path / "released_urdf",
            category_records_root=tmp_path / "records",
            expected_n=1,
        )


def test_evaluate_package_uses_shared_table1_metrics(tmp_path: Path) -> None:
    runner = _load_runner()
    package, _ = _write_package(tmp_path, "asset", category="category")
    binding = _package_binding(package)
    identity = {
        "asset_id": "asset",
        "selection_index": 0,
        "raw_category": "category",
        "package": str(package),
        "model_urdf_sha256": _sha256_file(package / "model.urdf"),
        "package_binding": binding,
    }

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATED"
    assert record["parse_success"] is True
    assert record["link_count"] == 2
    assert record["non_fixed_joint_count"] == 1
    assert record["valid_tree"] is True
    assert record["fingerprint_complete"] is True


def test_package_drift_after_evaluation_clears_partial_metrics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    package, _ = _write_package(tmp_path, "asset", category="category")
    identity = {
        "asset_id": "asset",
        "selection_index": 0,
        "raw_category": "category",
        "package": str(package),
        "model_urdf_sha256": _sha256_file(package / "model.urdf"),
        "package_binding": _package_binding(package),
    }
    real_fingerprint = runner.SHARED.fingerprint_package

    def mutate_after_fingerprint(path: Path) -> dict[str, object]:
        result = real_fingerprint(path)
        (package / "compile_report.json").write_text("drift", encoding="utf-8")
        return result

    monkeypatch.setattr(runner.SHARED, "fingerprint_package", mutate_after_fingerprint)

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATION_FAILED"
    assert record["parse_success"] is False
    assert record["topology_hash"] is None
    assert record["fingerprint_complete"] is False
    assert record["package_fingerprint"] is None


def test_fingerprint_failure_preserves_structural_metrics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    package, _ = _write_package(tmp_path, "asset", category="category")
    identity = {
        "asset_id": "asset",
        "selection_index": 0,
        "raw_category": "category",
        "package": str(package),
        "model_urdf_sha256": _sha256_file(package / "model.urdf"),
        "package_binding": _package_binding(package),
    }

    def fail_fingerprint(_path: Path) -> dict[str, object]:
        raise ValueError("fixture fingerprint failure")

    monkeypatch.setattr(runner.SHARED, "fingerprint_package", fail_fingerprint)

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATED_FINGERPRINT_INCOMPLETE"
    assert record["parse_success"] is True
    assert record["link_count"] == 2
    assert record["non_fixed_joint_count"] == 1
    assert record["valid_tree"] is True
    assert record["topology_hash"] is not None
    assert record["fingerprint_complete"] is False
    assert record["package_fingerprint"] is None
    assert "fixture fingerprint failure" in record["error"]


def test_git_revision_rejects_dirty_category_checkout(tmp_path: Path) -> None:
    runner = _load_runner()
    repository = tmp_path / "category-repository"
    repository.mkdir()
    subprocess.run(["git", "init", "-q", str(repository)], check=True)
    tracked = repository / "record.json"
    tracked.write_text("{}\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repository), "add", "record.json"], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(repository),
            "-c",
            "user.name=Fixture",
            "-c",
            "user.email=fixture@example.invalid",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )

    assert runner._git_revision(repository)
    tracked.write_text('{"category_slug":"changed"}\n', encoding="utf-8")

    with pytest.raises(ValueError, match="uncommitted"):
        runner._git_revision(repository)


def test_aggregate_keeps_frozen_denominators_and_category_counts() -> None:
    runner = _load_runner()
    records = [
        {
            "asset_id": "a",
            "raw_category": "one",
            "parse_success": True,
            "link_count": 2,
            "non_fixed_joint_count": 2,
            "joint_type_counts": {"revolute": 2},
            "valid_tree": True,
            "topology_hash": "top-a",
            "fingerprint_complete": True,
            "package_fingerprint": "same",
        },
        {
            "asset_id": "b",
            "raw_category": "two",
            "parse_success": False,
            "link_count": None,
            "non_fixed_joint_count": None,
            "joint_type_counts": None,
            "valid_tree": False,
            "topology_hash": None,
            "fingerprint_complete": False,
            "package_fingerprint": None,
        },
    ]

    summary = runner.aggregate_articraft_records(
        records,
        release_asset_count=9996,
        release_category_count=240,
    )

    assert summary["cohort"]["N_release"] == 9996
    assert summary["cohort"]["N_eval"] == 2
    assert summary["cohort"]["N_parse"] == 1
    assert summary["cohort"]["release_raw_categories"] == 240
    assert summary["cohort"]["eval_raw_categories"] == 2
    assert summary["multi_joint_assets"]["denominator"] == 2
    assert summary["unique_topologies"]["denominator"] == 1
    assert summary["exact_duplicate_rate"]["denominator"] == 1


def test_cli_evaluates_exact_frozen_manifest_and_publishes_artifacts(
    tmp_path: Path,
) -> None:
    second, _ = _write_package(tmp_path, "asset_b", category="category_two")
    first, _ = _write_package(tmp_path, "asset_a", category="category_one")
    input_manifest = _write_manifest(tmp_path / "input.json", [second, first])
    protocol = tmp_path / "protocol.md"
    protocol.write_text("fixture protocol\n", encoding="utf-8")
    output = tmp_path / "output"

    completed = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--source-root",
            str(tmp_path / "released_urdf"),
            "--input-manifest",
            str(input_manifest),
            "--category-records-root",
            str(tmp_path / "records"),
            "--protocol",
            str(protocol),
            "--output",
            str(output),
            "--expected-n",
            "2",
            "--workers",
            "2",
        ],
        cwd=REPO,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert [row["asset_id"] for row in records] == ["asset_b", "asset_a"]
    assert summary["cohort"]["N_eval"] == 2
    assert summary["cohort"]["eval_raw_categories"] == 2
    assert summary["status_counts"] == {"EVALUATED": 2}
    assert (output / "report.md").is_file()
    assert (output / "artifact_manifest.json").is_file()
