from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table1_lam.py"
SEED = 20260813


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_lam", RUNNER)
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


def _write_package(
    source_root: Path,
    *,
    object_id: str,
    category: str,
    tier: str,
    missing_mesh: bool = False,
) -> tuple[Path, dict[str, object]]:
    prefix = "imperfect" if tier == "broken" else "objects"
    rel_path = f"{prefix}/{category}/{object_id}"
    package = source_root / rel_path
    (package / "links").mkdir(parents=True)
    if not missing_mesh:
        (package / "links/mesh.obj").write_text(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )
    (package / "export.js").write_text("export default {};\n", encoding="utf-8")
    urdf = package / "generated.urdf"
    urdf.write_text(
        f"""<robot name="fixture-{object_id}">
  <link name="base"><visual><geometry><mesh filename="links/mesh.obj"/></geometry></visual></link>
  <link name="child"/>
  <joint name="joint" type="revolute"><parent link="base"/><child link="child"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    return package, {
        "object_release_id": object_id,
        "category": category,
        "tier": tier,
        "rel_path": rel_path,
        "n_movable": "1",
    }


def _write_fixture(
    tmp_path: Path,
    *,
    missing_mesh_rank: int | None = None,
    source_error_rank: int | None = 1,
) -> SimpleNamespace:
    dataset_root = tmp_path / "Articulated-Object-Code"
    source_root = dataset_root / "released_outputs"
    package_a, csv_a = _write_package(
        source_root,
        object_id="asset_a",
        category="chair",
        tier="viable",
        missing_mesh=missing_mesh_rank == 2,
    )
    package_b, csv_b = _write_package(
        source_root,
        object_id="asset_b",
        category="lamp",
        tier="broken",
        missing_mesh=missing_mesh_rank == 1,
    )
    release_rows = [csv_a, csv_b]
    release_manifest = dataset_root / "manifest.csv"
    with release_manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("object_release_id", "category", "tier", "rel_path", "n_movable"),
        )
        writer.writeheader()
        writer.writerows(release_rows)
    dataset_api = dataset_root / "dataset_api.json"
    dataset_api.write_text(
        json.dumps({"id": "fixture/Articulated-Object-Code", "sha": "fixture-revision"}),
        encoding="utf-8",
    )

    fresh_rows = []
    for release_order, raw in enumerate(release_rows):
        package = source_root / str(raw["rel_path"])
        urdf = package / "generated.urdf"
        fresh_rows.append(
            {
                "release_order": release_order,
                "asset_key": f"{raw['tier']}:{raw['rel_path']}",
                "object_release_id": raw["object_release_id"],
                "category": raw["category"],
                "tier": raw["tier"],
                "rel_path": raw["rel_path"],
                "declared_joint_count_hint": 1,
                "urdf_path": str(urdf),
                "urdf_exists": True,
                "urdf_sha256": _sha256_file(urdf),
            }
        )
    selected = [fresh_rows[1], fresh_rows[0]]
    frozen_records = []
    for rank, row in enumerate(selected, 1):
        frozen_records.append(
            {
                **row,
                "selection_rank": rank,
                "selection_hash": hashlib.sha256(
                    f"lam-table3-v1\0{SEED}\0{row['asset_key']}".encode("utf-8")
                ).hexdigest(),
            }
        )
    manifest = {
        "schema_version": 1,
        "dataset": "LAM released outputs (Articulated-Object-Code)",
        "classification": "FORMAL",
        "created_at": "2026-08-14T00:00:00Z",
        "source": {
            "source_root": str(source_root.resolve()),
            "release_manifest": str(release_manifest.resolve()),
            "release_manifest_sha256": _sha256_file(release_manifest),
            "dataset_api": str(dataset_api.resolve()),
            "dataset_api_sha256": _sha256_file(dataset_api),
            "upstream_revision": "fixture-revision",
            "n_release": 2,
            "tier_counts": {"broken": 1, "viable": 1},
            "candidate_pool_sha256": _canonical_sha256(
                sorted(row["asset_key"] for row in fresh_rows)
            ),
        },
        "selection": {
            "algorithm": "random.Random(seed).sample(sorted(asset_key), n)",
            "quality_label_blind": True,
            "seed": SEED,
            "n_eval": 2,
            "selected_asset_keys_sha256": _canonical_sha256(
                [row["asset_key"] for row in frozen_records]
            ),
        },
        "evaluation": {"config": {"workers": 2}},
        "records": frozen_records,
    }
    manifest["manifest_content_sha256"] = _canonical_sha256(manifest)

    runtime = tmp_path / "runtime"
    runtime.mkdir()
    manifest_path = runtime / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    result_rows = []
    for row in frozen_records:
        rank = int(row["selection_rank"])
        is_error = rank == source_error_rank
        result_rows.append(
            {
                "asset_key": row["asset_key"],
                "object_release_id": row["object_release_id"],
                "category": row["category"],
                "tier": row["tier"],
                "rel_path": row["rel_path"],
                "selection_rank": rank,
                "selection_hash": row["selection_hash"],
                "urdf_sha256": row["urdf_sha256"],
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "status": "error" if is_error else "completed",
                "error": "unsupported floating joint" if is_error else None,
                "parse_success": True,
                "strict_kinematic_pass": not is_error,
            }
        )
    records_path = runtime / "asset_records.jsonl"
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in reversed(result_rows)),
        encoding="utf-8",
    )
    return SimpleNamespace(
        dataset_root=dataset_root,
        source_root=source_root,
        records=records_path,
        manifest=manifest_path,
        packages={1: package_b, 2: package_a},
    )


def _rewrite_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def test_load_frozen_cohort_restores_manifest_order_and_keeps_source_errors(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)

    cohort = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )

    assert [row["selection_rank"] for row in cohort["assets"]] == [1, 2]
    assert [row["asset_key"] for row in cohort["assets"]] == [
        "broken:imperfect/lamp/asset_b",
        "viable:objects/chair/asset_a",
    ]
    assert [row["source_table3_status"] for row in cohort["assets"]] == [
        "error",
        "completed",
    ]
    assert cohort["release_asset_count"] == 2
    assert cohort["release_category_count"] == 2
    assert cohort["eval_category_count"] == 2


@pytest.mark.parametrize(
    "mutation",
    ("missing", "duplicate", "wrong_rank", "wrong_category", "wrong_urdf_hash"),
)
def test_load_frozen_cohort_rejects_invalid_completion_join(
    tmp_path: Path,
    mutation: str,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    rows = [json.loads(line) for line in fixture.records.read_text().splitlines()]
    if mutation == "missing":
        rows.pop()
    elif mutation == "duplicate":
        rows[1] = dict(rows[0])
    elif mutation == "wrong_rank":
        rows[0]["selection_rank"] = 99
    elif mutation == "wrong_category":
        rows[0]["category"] = "changed"
    else:
        rows[0]["urdf_sha256"] = "0" * 64
    _rewrite_jsonl(fixture.records, rows)

    with pytest.raises(ValueError, match="completion|mismatch|expected exactly"):
        runner.load_frozen_cohort(
            fixture.records,
            dataset_root=fixture.dataset_root,
            expected_n=2,
            formal=False,
        )


def test_load_frozen_cohort_rejects_release_manifest_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    with (fixture.dataset_root / "manifest.csv").open("a", encoding="utf-8") as handle:
        handle.write("extra,extra,viable,objects/extra/extra,1\n")

    with pytest.raises(ValueError, match="release manifest.*hash"):
        runner.load_frozen_cohort(
            fixture.records,
            dataset_root=fixture.dataset_root,
            expected_n=2,
            formal=False,
        )


def test_load_frozen_cohort_rejects_package_symlink(tmp_path: Path) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    external = tmp_path / "external.bin"
    external.write_bytes(b"external")
    (fixture.packages[1] / "linked.bin").symlink_to(external)

    with pytest.raises(ValueError, match="symlink"):
        runner.load_frozen_cohort(
            fixture.records,
            dataset_root=fixture.dataset_root,
            expected_n=2,
            formal=False,
        )


def test_loader_uses_only_top_level_generated_urdf(tmp_path: Path) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    nested = fixture.packages[1] / "pipeline_logs/intermediate"
    nested.mkdir(parents=True)
    (nested / "generated.urdf").write_text("<robot name='intermediate'/>", encoding="utf-8")

    cohort = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )

    assert cohort["assets"][0]["primary_urdf_relative_path"] == "generated.urdf"
    assert Path(cohort["assets"][0]["package"]) / "generated.urdf" == fixture.packages[1] / "generated.urdf"


def test_prior_table3_error_is_metadata_not_table1_failure(tmp_path: Path) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    identity = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )["assets"][0]

    record = runner.evaluate_package(identity)

    assert record["source_table3_status"] == "error"
    assert record["source_table3_error"] == "unsupported floating joint"
    assert record["status"] == "EVALUATED"
    assert record["parse_success"] is True
    assert record["link_count"] == 2
    assert record["non_fixed_joint_count"] == 1


def test_incomplete_resource_preserves_structure_and_reduces_fingerprint_coverage(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path, missing_mesh_rank=1)
    identity = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )["assets"][0]

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATED_FINGERPRINT_INCOMPLETE"
    assert record["parse_success"] is True
    assert record["valid_tree"] is True
    assert record["fingerprint_complete"] is False
    assert record["package_fingerprint"] is None
    assert record["missing_resources"] == ["links/mesh.obj"]


def test_package_drift_after_snapshot_fails_same_asset_in_place(tmp_path: Path) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    identity = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )["assets"][0]
    (fixture.packages[1] / "added.bin").write_bytes(b"drift")

    record = runner.evaluate_package(identity)

    assert record["asset_key"] == identity["asset_key"]
    assert record["selection_rank"] == 1
    assert record["status"] == "EVALUATION_FAILED"
    assert record["parse_success"] is False
    assert record["topology_hash"] is None
    assert "package changed after Table 1 snapshot" in record["error"]


def test_package_drift_during_fingerprint_clears_partial_metrics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    identity = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=False,
    )["assets"][0]
    real_fingerprint = runner.SHARED.fingerprint_package

    def mutate_after_fingerprint(path: Path):
        result = real_fingerprint(path)
        (fixture.packages[1] / "added.bin").write_bytes(b"drift")
        return result

    monkeypatch.setattr(runner.SHARED, "fingerprint_package", mutate_after_fingerprint)

    record = runner.evaluate_package(identity)

    assert record["status"] == "EVALUATION_FAILED"
    assert record["parse_success"] is False
    assert record["topology_hash"] is None
    assert record["fingerprint_complete"] is False


def test_loader_rejects_release_manifest_changed_during_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    real_loader = runner._load_release_rows_from_bytes

    def mutate_after_parse(payload: bytes, source_root: Path):
        rows = real_loader(payload, source_root)
        with (fixture.dataset_root / "manifest.csv").open("a", encoding="utf-8") as handle:
            handle.write("\n")
        return rows

    monkeypatch.setattr(runner, "_load_release_rows_from_bytes", mutate_after_parse)

    with pytest.raises(ValueError, match="release manifest changed during cohort snapshot"):
        runner.load_frozen_cohort(
            fixture.records,
            dataset_root=fixture.dataset_root,
            expected_n=2,
            formal=False,
        )


def test_formal_loader_exercises_receipt_and_category_gates(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    manifest = json.loads(fixture.manifest.read_text(encoding="utf-8"))
    monkeypatch.setattr(runner, "FORMAL_N_EVAL", 2)
    monkeypatch.setattr(runner, "FORMAL_RELEASE_COUNT", 2)
    monkeypatch.setattr(runner, "FORMAL_RELEASE_CATEGORY_COUNT", 2)
    monkeypatch.setattr(runner, "FORMAL_EVAL_CATEGORY_COUNT", 2)
    monkeypatch.setattr(runner, "FORMAL_SEED", SEED)
    monkeypatch.setattr(runner, "FORMAL_UPSTREAM_REVISION", "fixture-revision")
    monkeypatch.setattr(
        runner,
        "FORMAL_INPUT_MANIFEST_FILE_SHA256",
        _sha256_file(fixture.manifest),
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_INPUT_MANIFEST_CONTENT_SHA256",
        manifest["manifest_content_sha256"],
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_INPUT_RECORDS_FILE_SHA256",
        _sha256_file(fixture.records),
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_RELEASE_MANIFEST_SHA256",
        manifest["source"]["release_manifest_sha256"],
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_DATASET_API_SHA256",
        manifest["source"]["dataset_api_sha256"],
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_SELECTED_ASSET_KEYS_SHA256",
        manifest["selection"]["selected_asset_keys_sha256"],
    )

    cohort = runner.load_frozen_cohort(
        fixture.records,
        dataset_root=fixture.dataset_root,
        expected_n=2,
        formal=True,
    )
    assert cohort["release_category_count"] == 2
    assert cohort["eval_category_count"] == 2

    monkeypatch.setattr(runner, "FORMAL_EVAL_CATEGORY_COUNT", 1)
    with pytest.raises(ValueError, match="formal evaluation category count mismatch"):
        runner.load_frozen_cohort(
            fixture.records,
            dataset_root=fixture.dataset_root,
            expected_n=2,
            formal=True,
        )


def test_formal_metric_contract_rejects_protocol_or_shared_runner_drift(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    protocol = tmp_path / "protocol.md"
    protocol.write_text("frozen protocol\n", encoding="utf-8")
    monkeypatch.setattr(
        runner,
        "FORMAL_TABLE1_PROTOCOL_SHA256",
        _sha256_file(protocol),
    )
    monkeypatch.setattr(
        runner,
        "FORMAL_SHARED_METRIC_RUNNER_SHA256",
        _sha256_file(Path(runner.SHARED.__file__)),
    )

    runner._validate_metric_contract(protocol, formal=True)
    protocol.write_text("changed protocol\n", encoding="utf-8")
    with pytest.raises(ValueError, match="formal Table 1 protocol hash mismatch"):
        runner._validate_metric_contract(protocol, formal=True)

    monkeypatch.setattr(runner, "FORMAL_TABLE1_PROTOCOL_SHA256", _sha256_file(protocol))
    monkeypatch.setattr(runner, "FORMAL_SHARED_METRIC_RUNNER_SHA256", "0" * 64)
    with pytest.raises(ValueError, match="formal shared metric runner hash mismatch"):
        runner._validate_metric_contract(protocol, formal=True)


def test_run_revalidates_earlier_packages_after_all_workers_finish(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    fixture = _write_fixture(tmp_path)
    protocol = tmp_path / "protocol.md"
    protocol.write_text("fixture protocol\n", encoding="utf-8")
    output = tmp_path / "staging"
    output.mkdir()
    real_evaluate = runner._evaluate_package_fail_closed

    def mutate_first_after_second(identity: dict[str, object]):
        record = real_evaluate(identity)
        if identity["selection_rank"] == 2:
            (fixture.packages[1] / "after-first-finished.bin").write_bytes(b"drift")
        return record

    monkeypatch.setattr(runner, "_evaluate_package_fail_closed", mutate_first_after_second)
    args = SimpleNamespace(
        dataset_root=fixture.dataset_root,
        input_records=fixture.records,
        output=tmp_path / "unused",
        protocol=protocol,
        expected_n=2,
        workers=1,
        formal=False,
    )

    runner._run_to_output(args, output)

    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert records[0]["selection_rank"] == 1
    assert records[0]["status"] == "EVALUATION_FAILED"
    assert records[0]["parse_success"] is False
    assert "package changed after Table 1 snapshot" in records[0]["error"]
    assert records[1]["status"] == "EVALUATED"


def test_aggregate_uses_explicit_intent_and_coverage_denominators() -> None:
    runner = _load_runner()
    records = [
        {
            "asset_key": "a",
            "raw_category": "chair",
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
            "asset_key": "b",
            "raw_category": "lamp",
            "parse_success": True,
            "link_count": 3,
            "non_fixed_joint_count": 1,
            "joint_type_counts": {"prismatic": 1},
            "valid_tree": False,
            "topology_hash": None,
            "fingerprint_complete": False,
            "package_fingerprint": None,
        },
    ]

    summary = runner.aggregate_lam_records(
        records,
        release_asset_count=3217,
        release_category_count=787,
    )

    assert summary["cohort"]["N_release"] == 3217
    assert summary["cohort"]["N_eval"] == 2
    assert summary["cohort"]["N_parse"] == 2
    assert summary["cohort"]["release_raw_categories"] == 787
    assert summary["cohort"]["eval_raw_categories"] == 2
    assert summary["multi_joint_assets"]["denominator"] == 2
    assert summary["unique_topologies"]["denominator"] == 1
    assert summary["exact_duplicate_rate"]["denominator"] == 1


def test_cli_publishes_ordered_sealed_artifacts(tmp_path: Path) -> None:
    fixture = _write_fixture(tmp_path)
    protocol = tmp_path / "protocol.md"
    protocol.write_text("fixture protocol\n", encoding="utf-8")
    output = tmp_path / "output"

    completed = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--dataset-root",
            str(fixture.dataset_root),
            "--input-records",
            str(fixture.records),
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
    assert output.is_symlink()
    result_rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [row["selection_rank"] for row in result_rows] == [1, 2]
    assert len(result_rows) == 2
    run_manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert run_manifest["N_release"] == 2
    assert run_manifest["N_eval"] == 2
    assert [row["selection_rank"] for row in run_manifest["assets"]] == [1, 2]
    artifact_manifest = json.loads(
        (output / "artifact_manifest.json").read_text(encoding="utf-8")
    )
    for name, expected in artifact_manifest["files"].items():
        path = output / name
        assert path.stat().st_size == expected["bytes"]
        assert _sha256_file(path) == expected["sha256"]
