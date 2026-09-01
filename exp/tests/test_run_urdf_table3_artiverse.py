from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table3_artiverse.py"
SELECTION_PROTOCOL = "artiverse-table1-global-sample-v1"


def load_runner():
    assert RUNNER.is_file(), "Artiverse Table 3 runner has not been implemented"
    spec = importlib.util.spec_from_file_location("urdf_table3_artiverse", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_fixture(tmp_path: Path) -> tuple[Path, Path, list[str]]:
    source = tmp_path / "artiverse"
    roots = [
        "data/cabinet/source_b/model_b",
        "data/door/source_a/model_a",
    ]
    categories = ["cabinet", "door"]
    sources = ["source_b", "source_a"]
    model_ids = ["model_b", "model_a"]
    for root, model_id in zip(roots, model_ids, strict=True):
        package = source / root / "urdf_w_collider"
        package.mkdir(parents=True)
        (package / f"{model_id}.urdf").write_text(
            """<robot name="fixture">
<link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>\n""",
            encoding="utf-8",
        )
    release = {
        "format": "artiverse-data-tar-gz-chunks-v1",
        "chunk_count": 1,
        "model_count": 2,
        "chunks": [{
            "archive": "fixture.tar.gz",
            "model_count": 2,
            "roots": roots,
        }],
    }
    release_path = source / "dataset_chunks/manifest.json"
    release_path.parent.mkdir(parents=True)
    release_path.write_text(json.dumps(release, sort_keys=True), encoding="utf-8")
    release_hash = hashlib.sha256(release_path.read_bytes()).hexdigest()
    universe_hash = hashlib.sha256(
        "".join(f"{root}\n" for root in sorted(roots)).encode("utf-8")
    ).hexdigest()
    assets = []
    for rank, (root, category, upstream_source, model_id) in enumerate(
        zip(roots, categories, sources, model_ids, strict=True), start=1
    ):
        selection_hash = hashlib.sha256(
            "\0".join((SELECTION_PROTOCOL, release_hash, "20260813", root)).encode("utf-8")
        ).hexdigest()
        assets.append({
            "asset_id": root,
            "manifest_root": root,
            "raw_category": category,
            "source": upstream_source,
            "model_id": model_id,
            "chunk_archive": "fixture.tar.gz",
            "selection_rank": rank,
            "selection_hash": selection_hash,
        })
    cohort = {
        "schema_version": 1,
        "dataset": "Artiverse",
        "release_status": "PRE_RELEASE_SUBSET",
        "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        "N_release": 2,
        "N_eval": 2,
        "seed": 20260813,
        "selection_protocol": SELECTION_PROTOCOL,
        "release_manifest": "dataset_chunks/manifest.json",
        "release_manifest_sha256": release_hash,
        "release_universe_sha256": universe_hash,
        "assets": assets,
    }
    cohort_path = tmp_path / "table1_manifest.json"
    cohort_path.write_text(json.dumps(cohort, sort_keys=True), encoding="utf-8")
    return source, cohort_path, roots


def test_formal_contract_freezes_canonical_table1_cohort() -> None:
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
        ["--source-root", "/tmp/other"],
        ["--cohort-manifest", "/tmp/other.json"],
    ):
        with pytest.raises(ValueError, match="formal"):
            runner.validate_contract(runner.parse_args(extra))


def test_loader_preserves_exact_assets_order_and_resolves_model_urdf(tmp_path: Path) -> None:
    runner = load_runner()
    source, cohort_path, roots = write_fixture(tmp_path)

    loaded = runner.load_cohort(source, cohort_path, formal=False)

    assert [row["asset_key"] for row in loaded["assets"]] == roots
    assert [row["selection_rank"] for row in loaded["assets"]] == [1, 2]
    assert [Path(row["urdf_path"]).name for row in loaded["assets"]] == [
        "model_b.urdf",
        "model_a.urdf",
    ]
    assert [row["category"] for row in loaded["assets"]] == ["cabinet", "door"]
    assert all(row["declared_joint_count_hint"] == 1 for row in loaded["assets"])


@pytest.mark.parametrize("mutation", ["rank", "path_metadata", "duplicate", "filename"])
def test_loader_rejects_cohort_or_package_identity_drift(
    tmp_path: Path, mutation: str
) -> None:
    runner = load_runner()
    source, cohort_path, _roots = write_fixture(tmp_path)
    cohort = json.loads(cohort_path.read_text(encoding="utf-8"))
    if mutation == "rank":
        cohort["assets"][0]["selection_rank"] = 2
    elif mutation == "path_metadata":
        cohort["assets"][0]["raw_category"] = "door"
    elif mutation == "duplicate":
        cohort["assets"][1] = dict(cohort["assets"][0], selection_rank=2)
    else:
        package = source / cohort["assets"][0]["manifest_root"] / "urdf_w_collider"
        (package / "model_b.urdf").rename(package / "wrong.urdf")
    cohort_path.write_text(json.dumps(cohort, sort_keys=True), encoding="utf-8")

    with pytest.raises((ValueError, RuntimeError)):
        runner.load_cohort(source, cohort_path, formal=False)


def test_smoke_run_uses_manifest_prefix_and_emits_bound_records(tmp_path: Path) -> None:
    runner = load_runner()
    source, cohort_path, roots = write_fixture(tmp_path)
    output = tmp_path / "output"
    args = runner.parse_args([
        "--mode", "smoke",
        "--source-root", str(source),
        "--cohort-manifest", str(cohort_path),
        "--limit", "1",
        "--workers", "1",
        "--asset-timeout-seconds", "30",
        "--output", str(output),
    ])

    runner.run(args)

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert [row["asset_key"] for row in manifest["records"]] == roots[:1]
    assert [row["asset_key"] for row in records] == roots[:1]
    assert records[0]["manifest_root"] == roots[0]
    assert records[0]["category"] == "cabinet"
    assert records[0]["manifest_content_sha256"] == manifest["manifest_content_sha256"]
    assert summary["n_eval"] == 1
    assert summary["j_eval"] == 1
    assert summary["metrics"]["strict_kinematic_pass"]["passed"] == 1
    assert summary["category_macro"]["category_count"] == 1
