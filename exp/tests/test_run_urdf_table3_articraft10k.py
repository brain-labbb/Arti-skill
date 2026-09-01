from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table3_articraft10k.py"


def load_runner():
    assert RUNNER.is_file(), "Articraft-10K Table 3 runner has not been implemented"
    spec = importlib.util.spec_from_file_location("urdf_table3_articraft10k", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def package_binding(package: Path) -> dict[str, object]:
    files = []
    for path in sorted(item for item in package.rglob("*") if item.is_file()):
        relative = path.relative_to(package).as_posix()
        files.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def write_fixture(tmp_path: Path) -> tuple[Path, Path, Path, list[str]]:
    dataset_root = tmp_path / "Articraft-10K"
    release_root = dataset_root / "released_urdf"
    category_root = tmp_path / "official" / "records"
    asset_ids = ["rec_fixture_b", "rec_fixture_a"]
    categories = ["fixture_category_b", "fixture_category_a"]
    records = []
    for index, (asset_id, category) in enumerate(
        zip(asset_ids, categories, strict=True)
    ):
        package = release_root / asset_id
        package.mkdir(parents=True)
        urdf = package / "model.urdf"
        urdf.write_text(
            """<robot name="fixture">
<link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
""",
            encoding="utf-8",
        )
        (package / "compile_report.json").write_text(
            json.dumps({"record_id": asset_id, "status": "success"}),
            encoding="utf-8",
        )
        category_dir = category_root / asset_id
        category_dir.mkdir(parents=True)
        (category_dir / "record.json").write_text(
            json.dumps({"record_id": asset_id, "category_slug": category}),
            encoding="utf-8",
        )
        records.append(
            {
                "asset_id": asset_id,
                "selection_index": index,
                "package": str(package),
                "model_urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                "package_binding": package_binding(package),
            }
        )
    release_ids = sorted(asset_ids)
    manifest = {
        "schema_version": 1,
        "dataset": "Articraft-10K",
        "classification": "FORMAL",
        "mode": "formal",
        "selection": {
            "algorithm": "fixture-selection",
            "n_eval": 2,
            "requested_n": 2,
            "seed": 20260813,
            "selected_asset_ids_sha256": canonical_sha256(asset_ids),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "source": {
            "root": str(release_root),
            "release_asset_count": 2,
            "release_asset_ids_sha256": canonical_sha256(release_ids),
            "repo_id": "fixture/Articraft-10K",
            "revision": "fixture-revision",
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    manifest_path = tmp_path / "table2_manifest.json"
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    return dataset_root, manifest_path, category_root, asset_ids


def test_formal_contract_freezes_canonical_table2_cohort() -> None:
    runner = load_runner()
    args = runner.parse_args([])
    runner.validate_contract(args)

    assert args.mode == "formal"
    assert args.limit is None
    assert args.samples == 21
    assert args.workers == 4
    assert args.asset_timeout_seconds == 120.0

    for extra in (
        ["--limit", "3"],
        ["--samples", "20"],
        ["--workers", "3"],
        ["--asset-timeout-seconds", "30"],
        ["--dataset-root", "/tmp/other"],
        ["--cohort-manifest", "/tmp/other.json"],
        ["--category-records-root", "/tmp/categories"],
    ):
        with pytest.raises(ValueError, match="formal"):
            runner.validate_contract(runner.parse_args(extra))


def test_loader_preserves_exact_package_order_and_binds_categories(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    dataset_root, manifest_path, category_root, asset_ids = write_fixture(tmp_path)

    loaded = runner.load_cohort(
        dataset_root, manifest_path, category_root, formal=False
    )

    assert [row["asset_key"] for row in loaded["assets"]] == asset_ids
    assert [row["selection_index"] for row in loaded["assets"]] == [0, 1]
    assert [Path(row["urdf_path"]).name for row in loaded["assets"]] == [
        "model.urdf",
        "model.urdf",
    ]
    assert [row["category"] for row in loaded["assets"]] == [
        "fixture_category_b",
        "fixture_category_a",
    ]
    assert loaded["eval_category_count"] == 2
    assert all(row["declared_joint_count_hint"] == 1 for row in loaded["assets"])


@pytest.mark.parametrize(
    "mutation",
    ["selection_index", "duplicate", "package_path", "urdf_hash", "binding", "category"],
)
def test_loader_rejects_manifest_package_or_category_drift(
    tmp_path: Path, mutation: str
) -> None:
    runner = load_runner()
    dataset_root, manifest_path, category_root, asset_ids = write_fixture(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if mutation == "selection_index":
        manifest["records"][0]["selection_index"] = 1
    elif mutation == "duplicate":
        manifest["records"][1] = dict(manifest["records"][0], selection_index=1)
    elif mutation == "package_path":
        manifest["records"][0]["package"] = str(dataset_root)
    elif mutation == "urdf_hash":
        package = Path(manifest["records"][0]["package"])
        (package / "model.urdf").write_text("<robot name='changed'/>", encoding="utf-8")
    elif mutation == "binding":
        package = Path(manifest["records"][0]["package"])
        (package / "unexpected.txt").write_text("drift", encoding="utf-8")
    else:
        record_path = category_root / asset_ids[0] / "record.json"
        record_path.write_text(
            json.dumps({"record_id": "wrong", "category_slug": "fixture_category_b"}),
            encoding="utf-8",
        )
    manifest.pop("manifest_content_sha256", None)
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    with pytest.raises((ValueError, RuntimeError)):
        runner.load_cohort(dataset_root, manifest_path, category_root, formal=False)


def test_smoke_run_uses_manifest_prefix_and_emits_bound_records(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    dataset_root, manifest_path, category_root, asset_ids = write_fixture(tmp_path)
    output = tmp_path / "output"
    args = runner.parse_args(
        [
            "--mode",
            "smoke",
            "--dataset-root",
            str(dataset_root),
            "--cohort-manifest",
            str(manifest_path),
            "--category-records-root",
            str(category_root),
            "--limit",
            "1",
            "--workers",
            "1",
            "--asset-timeout-seconds",
            "30",
            "--output",
            str(output),
        ]
    )

    runner.run(args)

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert [row["asset_key"] for row in manifest["records"]] == asset_ids[:1]
    assert [row["asset_key"] for row in records] == asset_ids[:1]
    assert records[0]["package"] == str(
        dataset_root / "released_urdf" / asset_ids[0]
    )
    assert records[0]["category"] == "fixture_category_b"
    assert records[0]["manifest_content_sha256"] == manifest["manifest_content_sha256"]
    assert summary["n_eval"] == 1
    assert summary["j_eval"] == 1
    assert summary["metrics"]["strict_kinematic_pass"]["passed"] == 1
    assert summary["category_macro"]["category_count"] == 1
