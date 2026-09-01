from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_ours_pva_per_class_n5.py"
COHORT = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("table4b_ours_pva_n5", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def write_child_binding_fixture(root: Path) -> tuple[dict[str, object], Path, Path]:
    package = root / "seed_0001"
    package.mkdir(parents=True)
    urdf = package / "model.urdf"
    mesh = package / "shape.obj"
    urdf.write_text(
        """<robot name="fixture">
  <link name="base">
    <visual><geometry><mesh filename="shape.obj"/></geometry></visual>
    <collision><geometry><mesh filename="shape.obj"/></geometry></collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
    relevant = [
        {
            "path": "model.urdf",
            "bytes": urdf.stat().st_size,
            "sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        },
        {
            "path": "shape.obj",
            "bytes": mesh.stat().st_size,
            "sha256": hashlib.sha256(mesh.read_bytes()).hexdigest(),
        },
    ]
    job: dict[str, object] = {
        "selection_index": 0,
        "dataset_id": "PV-A/fixture/seed_0001",
        "asset_id": "PV-A/fixture/seed_0001",
        "category": "fixture",
        "package": str(package),
        "urdf_path": str(urdf),
        "expected_urdf_sha256": relevant[0]["sha256"],
        "expected_package_content_manifest_sha256": "b" * 64,
        "expected_relevant_file_binding": relevant,
        "expected_relevant_file_binding_sha256": canonical_sha256(relevant),
    }
    return job, urdf, mesh


def write_table4_fixture(root: Path) -> Path:
    cohort = json.loads(COHORT.read_text(encoding="utf-8"))
    cohort_file_sha256 = hashlib.sha256(COHORT.read_bytes()).hexdigest()
    items = []
    for index, row in enumerate(cohort["assets"]):
        package = Path(row["package"])
        items.append(
            {
                "order": index,
                "selection_rank": index,
                "dataset_id": row["dataset_id"],
                "asset_id": row["dataset_id"],
                "category": row["category"],
                "package": row["package"],
                "primary_urdf_relpath": f"{package.name}/model.urdf",
                "urdf_sha256": row["urdf_sha256"],
                "input_identity_sha256": hashlib.sha256(
                    f"fixture:{index}:{row['dataset_id']}".encode("utf-8")
                ).hexdigest(),
                "package_binding_content_manifest_sha256": row["package_binding"][
                    "content_manifest_sha256"
                ],
                "package_binding_file_count": row["package_binding"]["file_count"],
                "package_binding_total_bytes": row["package_binding"]["total_bytes"],
            }
        )
    manifest = {
        "schema_version": "table4_ours_pva_per_class_n5_frozen_manifest_v1",
        "protocol_id": "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1",
        "dataset": "Ours per-class N=5 (supplementary)",
        "classification": "FORMAL",
        "source": {
            "cohort_manifest_path": str(COHORT),
            "cohort_manifest_file_sha256": cohort_file_sha256,
            "cohort_manifest_content_sha256": cohort["manifest_content_sha256"],
            "cohort_type": "frozen PV-A per-class N=5 with max-joint overrides",
            "n_release": 302440,
            "n_eval": 2655,
            "release_category_count": 531,
            "eval_category_count": 531,
            "per_class": 5,
            "per_item_package_paths": True,
        },
        "selection": {
            "selected_asset_ids_sha256": hashlib.sha256(
                json.dumps(
                    [row["dataset_id"] for row in cohort["assets"]],
                    separators=(",", ":"),
                    ensure_ascii=True,
                ).encode("utf-8")
            ).hexdigest()
        },
        "items": items,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    root.mkdir(parents=True, exist_ok=True)
    (root / "frozen_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return root


@pytest.fixture(scope="module")
def runner():
    return load_runner()


@pytest.fixture(scope="module")
def table4_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return write_table4_fixture(tmp_path_factory.mktemp("table4"))


@pytest.fixture(scope="module")
def table4_context(runner, table4_dir: Path):
    binding = runner.configure_table4_dir(table4_dir)
    manifest = runner.load_source_manifest()
    jobs = runner.build_jobs(manifest)
    return binding, manifest, jobs


def test_table4_manifest_builds_all_2655_jobs_in_the_frozen_cohort_order(
    runner, table4_dir: Path, table4_context
) -> None:
    binding, _manifest, jobs = table4_context

    assert binding["source_manifest_sha256"] == hashlib.sha256(
        (table4_dir / "frozen_manifest.json").read_bytes()
    ).hexdigest()
    assert len(jobs) == 2655
    assert jobs[0]["dataset_id"] == "PV-A/Accessories_Cushion/seed_0021"
    assert jobs[-1]["dataset_id"] == "PV-A/zippo_lighter/seed_0303"
    assert [job["selection_index"] for job in jobs] == list(range(2655))
    assert jobs[0]["package"] == str(
        REPO / "exp/PV-A-per-class-n5/assets/Accessories_Cushion/seed_0021"
    )
    assert jobs[0]["expected_package_content_manifest_sha256"] == (
        "4a828326dcd516bd5a051205a3237a20ae16202688857673b6c88f755893528d"
    )
    assert jobs[0]["expected_relevant_file_binding"][0] == {
        "path": "model.urdf",
        "bytes": 11861,
        "sha256": "df9d68cf7b73a72ad33a8b4999adbdd586dc2f0d0529644fa6ddc9d538b8d97a",
    }
    assert jobs[0]["expected_relevant_file_binding_sha256"] == canonical_sha256(
        jobs[0]["expected_relevant_file_binding"]
    )
    assert runner.DATASET == "Ours per-class N=5 (supplementary)"
    assert runner.WORKERS == 16
    assert runner.CHILD_TIMEOUT_SECONDS == 900


def test_table4_manifest_rejects_self_hashed_package_binding_tampering(
    runner, tmp_path: Path
) -> None:
    table4_dir = write_table4_fixture(tmp_path / "table4")
    path = table4_dir / "frozen_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["items"][0]["package"] = manifest["items"][1]["package"]
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    path.write_text(json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8")

    with pytest.raises(SystemExit, match="package path binding mismatch"):
        runner.configure_table4_dir(table4_dir)


def test_table4_manifest_rejects_content_changed_without_a_new_self_hash(
    runner, tmp_path: Path
) -> None:
    table4_dir = write_table4_fixture(tmp_path / "table4")
    path = table4_dir / "frozen_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["items"][0]["category"] = "tampered"
    path.write_text(json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8")

    with pytest.raises(SystemExit, match="source manifest self-hash mismatch"):
        runner.configure_table4_dir(table4_dir)


def test_parent_timeout_record_preserves_input_binding_and_stays_fail_closed(
    runner,
) -> None:
    job = {
        "selection_index": 7,
        "dataset_id": "PV-A/fixture/seed_0007",
        "asset_id": "PV-A/fixture/seed_0007",
        "category": "fixture",
        "package": "/fixture/seed_0007",
        "expected_urdf_sha256": "a" * 64,
        "expected_package_content_manifest_sha256": "b" * 64,
        "expected_relevant_file_binding_sha256": "c" * 64,
        "expected_relevant_file_binding": [
            {"path": "model.urdf", "bytes": 12, "sha256": "a" * 64}
        ],
    }

    record = runner.base._failed_asset_record(job, "asset_timeout after 900s")
    metrics = runner.base.aggregate([record])

    assert record["status"] == "error"
    assert record["issues"] == ["asset_timeout after 900s"]
    assert record["expected_package_content_manifest_sha256"] == "b" * 64
    assert record["expected_relevant_file_binding_sha256"] == "c" * 64
    assert record["expected_relevant_file_binding"] == job[
        "expected_relevant_file_binding"
    ]
    assert record["observed_relevant_file_binding_before"] is None
    assert record["observed_relevant_file_binding_after"] is None
    assert record["visual_to_collision_p95_normalized"]["status"] == "N/E"
    assert record["collision_load_time_seconds"]["status"] == "N/E"
    assert metrics["status_counts"] == {"total": 1, "completed": 0, "error": 1}
    assert metrics["visual_to_collision_p95_normalized"]["intended"] == 1
    assert metrics["visual_to_collision_p95_normalized"]["measured"] == 0


def test_formal_scope_rejects_non_frozen_worker_count_before_output(
    runner, table4_dir: Path, tmp_path: Path
) -> None:
    runner.configure_table4_dir(table4_dir)
    output = tmp_path / "must-not-exist"
    args = SimpleNamespace(
        mode="formal",
        workers=15,
        smoke_receipt=None,
        output_dir=output,
        n=3,
        resume=False,
    )

    with pytest.raises(ValueError, match="formal mode requires workers=16"):
        runner.base.run_scope(args)

    assert output.exists() is False
    assert output.with_name(f".{output.name}.work").exists() is False


def test_child_rejects_mesh_drift_before_geometry_evaluation(
    runner, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    job, _urdf, mesh = write_child_binding_fixture(tmp_path)
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    job_path.write_text(json.dumps(job), encoding="utf-8")

    def geometry_delegate(_job_path: Path, output: Path) -> int:
        record = runner.failed_asset_record(job, "unused fixture failure")
        record["status"] = "completed"
        record["issues"] = []
        runner.base.atomic_write_json(output, record)
        return 0

    monkeypatch.setattr(runner, "_base_run_child", geometry_delegate)
    original = mesh.read_text(encoding="utf-8")
    mesh.write_text(original.replace("v 1 0 0", "v 0 0 1"), encoding="utf-8")
    assert mesh.stat().st_size == job["expected_relevant_file_binding"][1]["bytes"]

    return_code = runner.run_child(job_path, result_path)
    record = json.loads(result_path.read_text(encoding="utf-8"))

    assert return_code == 0
    assert record["status"] == "error"
    assert record["issues"][0].startswith("input_binding_drift_before_evaluation")
    assert record["expected_relevant_file_binding_sha256"] == job[
        "expected_relevant_file_binding_sha256"
    ]
    assert record["observed_relevant_file_binding_before_sha256"] != job[
        "expected_relevant_file_binding_sha256"
    ]
    assert record["observed_relevant_file_binding_after"] is None
    assert record["visual_to_collision_p95_normalized"]["status"] == "N/E"


def test_child_rejects_mesh_drift_during_geometry_evaluation(
    runner, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    job, _urdf, mesh = write_child_binding_fixture(tmp_path)
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    job_path.write_text(json.dumps(job), encoding="utf-8")

    def drifting_geometry_delegate(_job_path: Path, output: Path) -> int:
        original = mesh.read_text(encoding="utf-8")
        mesh.write_text(original.replace("v 1 0 0", "v 0 0 1"), encoding="utf-8")
        record = runner.failed_asset_record(job, "unused fixture failure")
        record["status"] = "completed"
        record["issues"] = []
        runner.base.atomic_write_json(output, record)
        return 0

    monkeypatch.setattr(runner, "_base_run_child", drifting_geometry_delegate)

    return_code = runner.run_child(job_path, result_path)
    record = json.loads(result_path.read_text(encoding="utf-8"))

    assert return_code == 0
    assert record["status"] == "error"
    assert record["issues"][0].startswith("input_binding_drift_after_evaluation")
    assert record["observed_relevant_file_binding_before_sha256"] == job[
        "expected_relevant_file_binding_sha256"
    ]
    assert record["observed_relevant_file_binding_after_sha256"] != job[
        "expected_relevant_file_binding_sha256"
    ]
    assert record["visual_to_collision_p95_normalized"]["status"] == "N/E"


@pytest.mark.parametrize(
    "tampered_field",
    (
        "observed_relevant_file_binding_before",
        "observed_relevant_file_binding_before_sha256",
        "observed_relevant_file_binding_after",
        "observed_relevant_file_binding_after_sha256",
    ),
)
def test_formal_verifier_rejects_completed_record_input_binding_drift(
    runner,
    table4_context,
    monkeypatch: pytest.MonkeyPatch,
    tampered_field: str,
) -> None:
    _binding, manifest, jobs = table4_context
    job = jobs[0]
    record = runner.failed_asset_record(job, "fixture")
    expected = job["expected_relevant_file_binding"]
    expected_sha256 = job["expected_relevant_file_binding_sha256"]
    record.update(
        {
            "status": "completed",
            "issues": [],
            "urdf_sha256": job["expected_urdf_sha256"],
            "observed_relevant_file_binding_before": expected,
            "observed_relevant_file_binding_before_sha256": expected_sha256,
            "observed_relevant_file_binding_after": expected,
            "observed_relevant_file_binding_after_sha256": expected_sha256,
        }
    )
    monkeypatch.setattr(runner, "N_EVAL", 1)
    aggregates = runner.base.aggregate([record])
    assert runner.verify_run(manifest, [record], aggregates)["all_pass"] is True

    if tampered_field.endswith("_sha256"):
        record[tampered_field] = "0" * 64
    else:
        record[tampered_field] = [
            {"path": "model.urdf", "bytes": 1, "sha256": "0" * 64}
        ]

    result = runner.verify_run(manifest, [record], runner.base.aggregate([record]))
    checks = {item["check"]: item["pass"] for item in result["checks"]}
    assert result["all_pass"] is False
    assert checks["completed_relevant_input_binding_matches_expected"] is False


def test_finalize_receipt_records_wall_time_and_closes_every_artifact(
    runner, table4_dir: Path, tmp_path: Path
) -> None:
    binding = runner.configure_table4_dir(table4_dir)
    outdir = tmp_path / "output"
    outdir.mkdir()
    (outdir / "asset_records.jsonl").write_text('{"dataset_id":"fixture"}\n')
    (outdir / "summary.json").write_text(
        json.dumps(
            {
                "protocol_id": runner.PROTOCOL_ID,
                "mode": "formal",
                "started_at_utc": "2026-08-24T04:00:00.000000Z",
                "completed_at_utc": "2026-08-24T04:00:12.500000Z",
                "wall_seconds": 12.5,
            }
        )
        + "\n"
    )
    (outdir / "manifest.json").write_text(
        json.dumps(
            {
                "protocol_id": runner.PROTOCOL_ID,
                "mode": "formal",
                "record_count": 2655,
                "wall_seconds": 12.5,
                "verification": {
                    "all_pass": True,
                    "check_count": 1,
                    "checks": [{"check": "record_count", "pass": True, "detail": "2655"}],
                },
            }
        )
        + "\n"
    )

    runner.finalize_receipt(outdir)

    timing = json.loads((outdir / "timing.json").read_text(encoding="utf-8"))
    verification = json.loads(
        (outdir / "verification.json").read_text(encoding="utf-8")
    )
    artifact = json.loads(
        (outdir / "artifact_manifest.json").read_text(encoding="utf-8")
    )
    assert timing["wall_seconds"] == 12.5
    assert verification["status"] == "PASS"
    assert verification["upstream_table4"]["source_manifest_sha256"] == binding[
        "source_manifest_sha256"
    ]
    assert artifact["content_manifest_sha256"] == canonical_sha256(artifact["files"])
    artifact_without_self_hash = dict(artifact)
    artifact_without_self_hash.pop("manifest_content_sha256")
    assert artifact["manifest_content_sha256"] == canonical_sha256(
        artifact_without_self_hash
    )
    assert artifact["input_binding"] == {
        "source_manifest_sha256": binding["source_manifest_sha256"],
        "source_manifest_content_sha256": binding[
            "source_manifest_content_sha256"
        ],
        "cohort_manifest_sha256": runner.EXPECTED_COHORT_MANIFEST_SHA256,
        "cohort_manifest_content_sha256": runner.EXPECTED_COHORT_CONTENT_SHA256,
        "ordered_ids_sha256": runner.EXPECTED_ORDERED_IDS_SHA256,
        "n_eval": 2655,
    }
    assert set(artifact["files"]) == {
        "asset_records.jsonl",
        "manifest.json",
        "summary.json",
        "timing.json",
        "verification.json",
    }
    assert runner.verify_artifact_manifest(outdir) is True

    (outdir / "summary.json").write_text('{"tampered":true}\n', encoding="utf-8")
    assert runner.verify_artifact_manifest(outdir) is False


def test_finalize_receipt_records_full_invocation_wall_and_preserves_it_on_resume(
    runner,
    table4_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner.configure_table4_dir(table4_dir)
    outdir = tmp_path / "output"
    outdir.mkdir()
    (outdir / "asset_records.jsonl").write_text('{"dataset_id":"fixture"}\n')
    (outdir / "summary.json").write_text(
        json.dumps(
            {
                "protocol_id": runner.PROTOCOL_ID,
                "mode": "formal",
                "started_at_utc": "2026-08-24T04:00:05.000000Z",
                "completed_at_utc": "2026-08-24T04:00:17.500000Z",
                "wall_seconds": 12.5,
            }
        )
        + "\n"
    )
    (outdir / "manifest.json").write_text(
        json.dumps(
            {
                "protocol_id": runner.PROTOCOL_ID,
                "mode": "formal",
                "record_count": 2655,
                "verification": {"all_pass": True, "checks": []},
            }
        )
        + "\n"
    )

    runner._FULL_RUN_STARTED_AT_UTC = "2026-08-24T04:00:00.000000Z"
    runner._FULL_RUN_STARTED_PERF = 100.0
    runner._FULL_RUN_RESUME = False
    monkeypatch.setattr(runner, "_perf_counter", lambda: 120.0, raising=False)
    runner.finalize_receipt(outdir)

    initial = json.loads((outdir / "timing.json").read_text(encoding="utf-8"))
    assert initial["started_at_utc"] == "2026-08-24T04:00:00.000000Z"
    assert initial["wall_seconds"] == 20.0
    assert initial["evaluation_child_wall_seconds"] == 12.5
    assert initial["resume"] is False

    runner._FULL_RUN_STARTED_AT_UTC = "2026-08-24T05:00:00.000000Z"
    runner._FULL_RUN_STARTED_PERF = 200.0
    runner._FULL_RUN_RESUME = True
    monkeypatch.setattr(runner, "_perf_counter", lambda: 201.25, raising=False)
    runner.finalize_receipt(outdir)

    resumed = json.loads((outdir / "timing.json").read_text(encoding="utf-8"))
    for key in ("started_at_utc", "completed_at_utc", "wall_seconds", "resume"):
        assert resumed[key] == initial[key]
    assert resumed["resume_invocation_count"] == 1
    assert resumed["resume_history"][0]["wall_seconds"] == 1.25
    assert runner.verify_artifact_manifest(outdir) is True


def test_artifact_verifier_rejects_header_tampering_and_finalize_gates_publish(
    runner,
    table4_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner.configure_table4_dir(table4_dir)
    outdir = tmp_path / "output"
    outdir.mkdir()
    (outdir / "asset_records.jsonl").write_text('{"dataset_id":"fixture"}\n')
    (outdir / "summary.json").write_text(
        json.dumps({"mode": "smoke", "wall_seconds": 1.0}) + "\n"
    )
    (outdir / "manifest.json").write_text(
        json.dumps({"record_count": 1, "verification": {"all_pass": None}}) + "\n"
    )
    runner.finalize_receipt(outdir)

    path = outdir / "artifact_manifest.json"
    original = json.loads(path.read_text(encoding="utf-8"))
    for key, value in (
        ("schema_version", "tampered/v9"),
        ("protocol_id", "tampered-protocol"),
        ("excludes", []),
        ("manifest_content_sha256", "0" * 64),
    ):
        tampered = dict(original)
        tampered[key] = value
        path.write_text(json.dumps(tampered) + "\n", encoding="utf-8")
        assert runner.verify_artifact_manifest(outdir) is False
    path.write_text(json.dumps(original) + "\n", encoding="utf-8")

    tampered = json.loads(json.dumps(original))
    tampered["input_binding"]["source_manifest_sha256"] = "0" * 64
    tampered_without_self_hash = dict(tampered)
    tampered_without_self_hash.pop("manifest_content_sha256")
    tampered["manifest_content_sha256"] = canonical_sha256(
        tampered_without_self_hash
    )
    path.write_text(json.dumps(tampered) + "\n", encoding="utf-8")
    assert runner.verify_artifact_manifest(outdir) is False
    path.write_text(json.dumps(original) + "\n", encoding="utf-8")

    monkeypatch.setattr(runner, "verify_artifact_manifest", lambda _path: False)
    with pytest.raises(RuntimeError, match="artifact manifest verification failed"):
        runner.finalize_receipt(outdir)
