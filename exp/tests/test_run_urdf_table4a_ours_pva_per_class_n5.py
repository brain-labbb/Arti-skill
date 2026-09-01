from __future__ import annotations

import datetime as dt
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4a_ours_pva_per_class_n5.py"


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def complete_early_affinity() -> dict[str, object]:
    cpus = [0, 1, 2, 3]
    return {
        "status": "COMPLETE",
        "pid": 12345,
        "requested": cpus,
        "observed": cpus,
    }


def utc_seconds_ago(seconds: float) -> str:
    value = dt.datetime.now(dt.timezone.utc) - dt.timedelta(seconds=seconds)
    return value.isoformat().replace("+00:00", "Z")


def load_runner():
    assert RUNNER.is_file(), "Table 4a PVA adapter has not been implemented"
    spec = importlib.util.spec_from_file_location("table4a_ours_pva_test", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_affinity_launcher(runner):
    path = runner.CPU_AFFINITY_LAUNCHER
    assert path.is_file(), "early CPU-affinity launcher has not been implemented"
    spec = importlib.util.spec_from_file_location("table4a_affinity_launcher_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_early_affinity_launcher_binds_and_reads_back_before_exec(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    launcher = load_affinity_launcher(runner)
    state = {"available": {0, 1, 2, 3, 4, 5}}
    calls: list[set[int]] = []
    monkeypatch.setenv(runner.shared.lam4a.CPU_AFFINITY_ENV, "1,2,3,4")
    monkeypatch.setattr(
        launcher.os,
        "sched_getaffinity",
        lambda _pid: set(state["available"]),
    )

    def set_affinity(_pid: int, cpus: set[int]) -> None:
        calls.append(set(cpus))
        state["available"] = set(cpus)

    monkeypatch.setattr(launcher.os, "sched_setaffinity", set_affinity)

    assert launcher.bind_from_environment() == [1, 2, 3, 4]
    assert calls == [{1, 2, 3, 4}]


@pytest.mark.parametrize(
    "raw",
    ("", " 1", "1 ", "01", "1,", "1,1", "2,1", "\uff11"),
)
def test_early_affinity_launcher_rejects_noncanonical_cpu_lists(
    raw: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    launcher = load_affinity_launcher(runner)
    monkeypatch.setenv(runner.shared.lam4a.CPU_AFFINITY_ENV, raw)
    with pytest.raises(ValueError):
        launcher.bind_from_environment()


def test_scheduler_affinity_uses_real_noncontiguous_cpu_ids() -> None:
    runner = load_runner()
    available = [2, 4, 7, 9, 12, 14, 17, 19]

    assert runner.shared._cpu_affinity_for_slot(available, 0, 4) == "2,4,7,9"
    assert runner.shared._cpu_affinity_for_slot(available, 1, 4) == "12,14,17,19"
    with pytest.raises(ValueError):
        runner.shared._cpu_affinity_for_slot(available, 2, 4)


def test_shared_child_command_uses_frozen_early_affinity_launcher(tmp_path: Path) -> None:
    runner = load_runner()
    runner.configure_shared_runner(None)
    job = tmp_path / "job.json"
    result = tmp_path / "result.json"

    command = runner.shared._child_launch_command("/frozen/genesis-python", job, result)

    assert command == [
        "/frozen/genesis-python",
        "-S",
        str(runner.CPU_AFFINITY_LAUNCHER.resolve()),
        "--",
        str(runner.SCRIPT),
        "--child",
        "--job",
        str(job),
        "--result",
        str(result),
    ]


def test_torch_thread_binding_sets_interop_before_genesis_import(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    state = {"intra": 4, "interop": 96}
    calls: list[tuple[str, int]] = []

    class FakeTorch:
        @staticmethod
        def get_num_threads() -> int:
            return state["intra"]

        @staticmethod
        def get_num_interop_threads() -> int:
            return state["interop"]

        @staticmethod
        def set_num_threads(value: int) -> None:
            calls.append(("intra", value))
            state["intra"] = value

        @staticmethod
        def set_num_interop_threads(value: int) -> None:
            calls.append(("interop", value))
            state["interop"] = value

    monkeypatch.setitem(sys.modules, "torch", FakeTorch)

    expected = {"intra_op_threads": 1, "inter_op_threads": 1}
    assert runner.bind_torch_threading() == expected
    assert calls == [("intra", 1), ("interop", 1)]

    # set_num_interop_threads may only be called before inter-op work starts.
    # The second runtime identity check therefore has to be readback-only.
    assert runner.bind_torch_threading() == expected
    assert calls == [("intra", 1), ("interop", 1)]


def test_torch_thread_attestation_is_fail_closed() -> None:
    runner = load_runner()
    expected = {"intra_op_threads": 1, "inter_op_threads": 1}
    complete = {
        "status": "COMPLETE",
        "expected": expected,
        "before_evaluation": expected,
        "after_evaluation": expected,
    }
    assert runner.torch_threading_attestation_valid(
        {"status": "completed", "torch_threading": complete}
    )

    drifted = json.loads(json.dumps(complete))
    drifted["after_evaluation"]["inter_op_threads"] = 96
    assert not runner.torch_threading_attestation_valid(
        {"status": "completed", "torch_threading": drifted}
    )

    unavailable = {
        "status": "NOT_OBSERVED",
        "expected": expected,
        "before_evaluation": None,
        "after_evaluation": None,
    }
    assert runner.torch_threading_attestation_valid(
        {
            "status": "error",
            "issues": ["asset_timeout after 1800s"],
            "torch_threading": unavailable,
        }
    )
    assert not runner.torch_threading_attestation_valid(
        {"status": "completed", "torch_threading": unavailable}
    )
    assert not runner.torch_threading_attestation_valid(
        {
            "status": "error",
            "issues": ["child_process_failed: rc=-6"],
            "torch_threading": unavailable,
        }
    )


def test_post_evaluation_thread_readback_does_not_heal_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    expected = dict(runner.TORCH_THREAD_COUNTS)
    observed = {"intra_op_threads": 1, "inter_op_threads": 96}
    monkeypatch.setattr(runner, "read_torch_threading", lambda: observed)

    def must_not_rebind() -> dict[str, int]:
        raise AssertionError("post-evaluation readback must not call a setter")

    monkeypatch.setattr(runner, "bind_torch_threading", must_not_rebind)
    record = runner._complete_torch_threading(
        {"status": "completed", "issues": []}, expected
    )

    assert record["status"] == "error"
    assert record["torch_threading"] == {
        "status": "FAILED",
        "expected": expected,
        "before_evaluation": expected,
        "after_evaluation": observed,
    }
    assert record["issues"][0].startswith("torch_thread_readback_failed")


def write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def refresh_output_file_bindings(runner, output: Path, *names: str) -> None:
    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    for name in names:
        path = output / name
        artifact["files"][name] = {
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(
        artifact
    )
    write_json(artifact_path, artifact)


def refreeze_wall_clock(runner, output: Path, **updates: object) -> None:
    timing_path = output / "timing.json"
    timing = json.loads(timing_path.read_text(encoding="utf-8"))
    timing.update(updates)
    write_json(timing_path, timing)
    wall_clock = {
        "started_at_utc": timing["started_at_utc"],
        "completed_at_utc": timing["completed_at_utc"],
        "wall_time_seconds": timing["wall_time_seconds"],
        "measurement_endpoint": timing["measurement_endpoint"],
    }
    verification_path = output / "verification.json"
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    verification["wall_clock"] = wall_clock
    write_json(verification_path, verification)
    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["wall_clock"] = wall_clock
    write_json(artifact_path, artifact)
    refresh_output_file_bindings(runner, output, "timing.json", "verification.json")


def package_row(
    root: Path,
    index: int,
    *,
    category: str = "category",
    lower: float = 0.0,
    upper: float = 1.0,
    with_mesh: bool = False,
) -> dict[str, object]:
    package = root / category / f"seed_{index:04d}"
    package.mkdir(parents=True)
    urdf = package / "model.urdf"
    child_geometry = (
        "<collision><geometry><mesh filename='mesh.obj'/></geometry></collision>"
        if with_mesh
        else ""
    )
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><link name='child'>"
        f"{child_geometry}</link>"
        "<joint name='hinge' type='revolute'><parent link='base'/><child link='child'/>"
        f"<limit lower='{lower}' upper='{upper}' effort='1' velocity='1'/></joint></robot>\n",
        encoding="utf-8",
    )
    urdf_sha = hashlib.sha256(urdf.read_bytes()).hexdigest()
    files = [{"bytes": urdf.stat().st_size, "path": "model.urdf", "sha256": urdf_sha}]
    if with_mesh:
        mesh = package / "mesh.obj"
        mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
        files.append(
            {
                "bytes": mesh.stat().st_size,
                "path": "mesh.obj",
                "sha256": hashlib.sha256(mesh.read_bytes()).hexdigest(),
            }
        )
    files.sort(key=lambda row: str(row["path"]))
    dataset_id = f"PV-A/{category}/seed_{index:04d}"
    return {
        "dataset_id": dataset_id,
        "asset_id": f"seed_{index:04d}",
        "category": category,
        "selection_index": index,
        "package": str(package),
        "primary_urdf_relative_path": "model.urdf",
        "urdf_sha256": urdf_sha,
        "package_binding": {
            "content_manifest_sha256": canonical_sha256(files),
            "file_count": len(files),
            "files": files,
            "total_bytes": sum(int(row["bytes"]) for row in files),
        },
    }


def write_table4_receipt(
    root: Path,
    cohort: list[dict[str, object]],
    *,
    tamper_item: int | None = None,
    valid_self_hash: bool = True,
) -> Path:
    table4_dir = root / "table4"
    table4_dir.mkdir()
    items = []
    for index, row in enumerate(cohort):
        binding = row["package_binding"]
        assert isinstance(binding, dict)
        item = {
            "asset_id": row["dataset_id"],
            "dataset_id": row["dataset_id"],
            "category": row["category"],
            "order": index,
            "package": row["package"],
            "package_binding_content_manifest_sha256": binding["content_manifest_sha256"],
            "package_binding_file_count": binding["file_count"],
            "package_binding_total_bytes": binding["total_bytes"],
            "primary_urdf_relpath": f"seed_{index:04d}/model.urdf",
            "urdf_sha256": row["urdf_sha256"],
            "movable_dof_count": 1,
            "input_identity_sha256": hashlib.sha256(str(index).encode()).hexdigest(),
        }
        items.append(item)
    if tamper_item is not None:
        items[tamper_item]["package_binding_content_manifest_sha256"] = "0" * 64
    manifest = {
        "schema_version": "table4_ours_pva_per_class_n5_frozen_manifest_v1",
        "protocol_id": "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1",
        "dataset": "Ours per-class N=5 (supplementary)",
        "classification": "FORMAL",
        "source": {"n_eval": len(items), "per_item_package_paths": True},
        "items": items,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    if not valid_self_hash:
        manifest["dataset"] = "tampered-after-freeze"
    write_json(table4_dir / "frozen_manifest.json", manifest)
    (table4_dir / "state_records.jsonl").write_text('{"fixture":"state"}\n', encoding="utf-8")
    (table4_dir / "asset_records.jsonl").write_text(
        "".join(
            json.dumps(
                {
                    "dataset_id": row["dataset_id"],
                    "order": index,
                    "strict_collision_pass": True,
                },
                sort_keys=True,
            )
            + "\n"
            for index, row in enumerate(cohort)
        ),
        encoding="utf-8",
    )
    expected_states = len(items) + 21 * len(items) + 64 * len(items)
    write_json(table4_dir / "checkpoint.json", {"state": "complete"})
    (table4_dir / "protocol_document_at_freeze.md").write_text(
        (REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md").read_text(
            encoding="utf-8"
        ),
        encoding="utf-8",
    )
    (table4_dir / "report.md").write_text("# Table 4 report\n", encoding="utf-8")
    write_json(table4_dir / "summary.json", {"n_eval": len(items)})
    write_json(
        table4_dir / "timing.json",
        {
            "n_eval": len(items),
            "j_eval": len(items),
            "expected_states": expected_states,
            "executed_states": 0,
        },
    )
    write_json(
        table4_dir / "verification.json",
        {"status": "PASS", "expected_states": expected_states, "executed_states": 0},
    )
    files = {}
    for name in (
        "asset_records.jsonl",
        "checkpoint.json",
        "frozen_manifest.json",
        "protocol_document_at_freeze.md",
        "report.md",
        "state_records.jsonl",
        "summary.json",
        "timing.json",
        "verification.json",
    ):
        path = table4_dir / name
        files[name] = {
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    artifact = {
        "schema_version": "table4_artifact_manifest_v1",
        "dataset": "Ours per-class N=5 (supplementary)",
        "protocol_id": "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1",
        "run_manifest_content_sha256": manifest["manifest_content_sha256"],
        "n_eval": len(items),
        "j_eval": len(items),
        "expected_states": expected_states,
        "executed_states": 0,
        "closure_checks": {
            "manifest_self_hash_valid": True,
            "base_verification_passed": True,
            "checkpoint_complete": True,
            "expected_states_match_verification": True,
            "executed_states_within_denominator": True,
        },
        "files": files,
    }
    artifact["artifact_manifest_content_sha256"] = canonical_sha256(artifact)
    write_json(
        table4_dir / "artifact_manifest.json",
        artifact,
    )
    return table4_dir


def write_shared_run_outputs(
    runner,
    output: Path,
    binding,
    *,
    protocol_text: str | None = None,
) -> None:
    output.mkdir()
    if protocol_text is None:
        protocol_text = runner.PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    (output / "protocol_snapshot.md").write_text(protocol_text, encoding="utf-8")
    dataset_id = str(binding.cohort_assets[0]["dataset_id"])
    package = str(binding.cohort_assets[0]["package"])
    package_binding = binding.cohort_assets[0]["package_binding"]
    expected_package_files = package_binding["files"]
    expected_package_sha256 = package_binding["content_manifest_sha256"]
    joint_record = {
        "asset_id": dataset_id,
        "asset_status": "completed",
        "category": "category",
        "dataset_id": dataset_id,
        "dof_position": 0,
        "full_range_cf_pass": True,
        "illegal_states": 0,
        "issues": [],
        "limit_endpoints_executed": 2,
        "limit_endpoints_intended": 2,
        "limit_reachable": True,
        "joint_name": "hinge",
        "joint_type": "revolute",
        "safe_dof": 1,
        "selection_index": 0,
        "state_summaries": [
            {
                "executed": True,
                "illegal_collision": False,
                "sample_index": sample_index,
            }
            for sample_index in range(21)
        ],
        "states_executed": 21,
        "states_intended": 21,
        "table3_joint_level_pass": True,
        "xml_index": 0,
    }
    asset_record = {
        "asset_id": dataset_id,
        "category": "category",
        "child": {"pid": 12345},
        "dataset_id": dataset_id,
        "input_identity_sha256": binding.manifest["items"][0][
            "input_identity_sha256"
        ],
        "issues": [],
        "expected_movable_dof": 1,
        "expected_urdf_sha256": binding.cohort_assets[0]["urdf_sha256"],
        "joint_records": [
            {
                key: value
                for key, value in joint_record.items()
                if key
                not in {
                    "asset_id",
                    "asset_status",
                    "category",
                    "dataset_id",
                    "selection_index",
                }
            }
        ],
        "package": package,
        "expected_package_content_manifest_sha256": expected_package_sha256,
        "expected_package_file_binding": expected_package_files,
        "expected_package_file_binding_sha256": expected_package_sha256,
        "observed_package_file_binding_before": expected_package_files,
        "observed_package_file_binding_before_sha256": expected_package_sha256,
        "observed_package_file_binding_after": expected_package_files,
        "observed_package_file_binding_after_sha256": expected_package_sha256,
        "selection_index": 0,
        "state_hash_cross_check": {"mismatch": 0, "no_reference": 0, "verified": 21},
        "states_executed": 21,
        "states_intended": 21,
        "status": "completed",
        "early_cpu_affinity": complete_early_affinity(),
        "torch_threading": {
            "status": "COMPLETE",
            "expected": dict(runner.TORCH_THREAD_COUNTS),
            "before_evaluation": dict(runner.TORCH_THREAD_COUNTS),
            "after_evaluation": dict(runner.TORCH_THREAD_COUNTS),
        },
    }
    (output / "asset_records.jsonl").write_text(
        json.dumps(asset_record, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "joint_records.jsonl").write_text(
        json.dumps(
            {key: value for key, value in joint_record.items() if key != "state_summaries"},
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    previous_j_eval = runner.shared.J_EVAL
    try:
        runner.shared.J_EVAL = runner.J_EVAL
        metrics = runner.shared.aggregate([asset_record], {dataset_id: True})
    finally:
        runner.shared.J_EVAL = previous_j_eval
    summary = {
        "classification": "SMOKE",
        "cohort": {
            "j_eval": 1,
            "n_eval": 1,
            "ordered_ids_sha256": binding.ordered_ids_sha256,
            "source_manifest": str(binding.manifest_path),
            "source_manifest_file_sha256": binding.frozen_manifest_sha256,
        },
        "dataset": runner.DATASET_LABEL,
        "engine_protocol_id": runner.shared.ENGINE_PROTOCOL_ID,
        "metrics": metrics,
        "mode": "smoke",
        "protocol_id": runner.PROTOCOL_ID,
        "run_directory": str(output),
        "status_counts": metrics["status_counts"],
        "wall_seconds": 5.0,
    }
    write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(
        runner._render_summary_markdown(summary),
        encoding="utf-8",
    )
    frozen_config = {
        "classification": "SMOKE",
        "cohort": {
            "j_eval": 1,
            "n_eval": 1,
            "ordered_ids_sha256": binding.ordered_ids_sha256,
            "source_manifest": str(binding.manifest_path),
            "source_manifest_content_sha256": binding.frozen_manifest_content_sha256,
            "source_manifest_file_sha256": binding.frozen_manifest_sha256,
            "table2_cohort_content_sha256": runner.TABLE2_COHORT_CONTENT_SHA256,
            "table2_cohort_file_sha256": runner.TABLE2_COHORT_FILE_SHA256,
            "table2_cohort_manifest": str(runner.TABLE2_COHORT_MANIFEST),
        },
        "dataset": runner.DATASET_LABEL,
        "execution": {
            "child_timeout_seconds": runner.CHILD_TIMEOUT_SECONDS,
            "early_cpu_affinity_launcher": runner._file_receipt(
                runner.CPU_AFFINITY_LAUNCHER
            ),
            "child_cpu_affinity_width": runner.shared.lam4a.CPU_AFFINITY_WIDTH,
            "table4_asset_records": {
                "path": str(binding.asset_records_path),
                "sha256": binding.asset_records_sha256,
            },
            "table4_state_records": {
                "path": str(binding.state_records_path),
                "sha256": binding.state_records_sha256,
            },
            "table3_source": {
                "records": str(runner.TABLE3_RECORDS),
                "sha256": runner.TABLE3_RECORDS_SHA256,
            },
            "workers": runner.FORMAL_WORKERS,
        },
        "mode": "smoke",
        "operationalization": {
            "early_cpu_affinity": runner.EARLY_CPU_AFFINITY_POLICY,
            "early_cpu_affinity_launcher_binding": runner._file_receipt(
                runner.CPU_AFFINITY_LAUNCHER
            ),
            "resume_policy": runner.RESUME_POLICY,
            "torch_threading": runner.TORCH_THREADING_POLICY,
            "shared_table4a_runner_binding": runner._file_receipt(
                runner._SHARED_RUNNER_SCRIPT
            ),
            "source_table4_artifact_manifest_binding": runner._file_receipt(
                binding.artifact_manifest_path
            ),
        },
        "protocol_document_sha256": hashlib.sha256(protocol_text.encode()).hexdigest(),
        "protocol_id": runner.PROTOCOL_ID,
        "runner_identity": {
            "lam_supplementary_runner_path": str(runner.shared.lam4a.SCRIPT),
            "lam_supplementary_runner_sha256": runner.sha256_file(runner.shared.lam4a.SCRIPT),
            "runner_path": str(runner.SCRIPT),
            "runner_sha256": runner.sha256_file(runner.SCRIPT),
            "static_atoms_path": str(runner.SCRIPT.with_name("lam_supplementary_static.py")),
            "static_atoms_sha256": runner.sha256_file(
                runner.SCRIPT.with_name("lam_supplementary_static.py")
            ),
        },
    }
    frozen_config["frozen_config_sha256"] = canonical_sha256(frozen_config)
    write_json(output / "frozen_config.json", frozen_config)
    outputs = {
        "asset_records_sha256": hashlib.sha256((output / "asset_records.jsonl").read_bytes()).hexdigest(),
        "joint_records_sha256": hashlib.sha256((output / "joint_records.jsonl").read_bytes()).hexdigest(),
        "summary_md_sha256": hashlib.sha256((output / "summary.md").read_bytes()).hexdigest(),
        "summary_sha256": hashlib.sha256((output / "summary.json").read_bytes()).hexdigest(),
    }
    write_json(
        output / "manifest.json",
        {
            "dataset": runner.DATASET_LABEL,
            "mode": "smoke",
            "outputs": outputs,
            "protocol_id": runner.PROTOCOL_ID,
            "record_count": 1,
            "status_counts": metrics["status_counts"],
            "verification": {"all_pass": None, "note": "smoke mode"},
            "wall_seconds": 5.0,
        },
    )


def build_mesh_bound_job(runner, tmp_path: Path):
    cohort = [package_row(tmp_path / "packages", 0, with_mesh=True)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    manifest = runner.source_manifest_from_binding(binding)
    dataset_id = str(cohort[0]["dataset_id"])
    jobs, _ = runner.build_jobs(
        manifest,
        {dataset_id: {"hinge": True}},
        {},
        formal=False,
    )
    return jobs[0], Path(str(cohort[0]["package"])) / "mesh.obj"


def configure_mini_formal_contract(
    runner,
    monkeypatch: pytest.MonkeyPatch,
    binding,
) -> None:
    monkeypatch.setattr(runner, "N_EVAL", 1)
    monkeypatch.setattr(runner, "J_EVAL", 1)
    monkeypatch.setattr(runner, "EXPECTED_CATEGORY_COUNT", 1)
    monkeypatch.setattr(runner, "EXPECTED_RANGE_EVALUABLE_JOINTS", 1)
    monkeypatch.setattr(runner, "EXPECTED_RANGE_INVALID_JOINTS", 0)
    monkeypatch.setattr(runner, "EXPECTED_SINGLE_STATES", 21)
    monkeypatch.setattr(runner, "EXPECTED_GENESIS_REPLAY_STATES", 21)
    monkeypatch.setattr(runner, "EXPECTED_FAIL_CLOSED_WITHOUT_REPLAY_STATES", 0)
    monkeypatch.setattr(runner, "EXPECTED_ORDERED_IDS_SHA256", binding.ordered_ids_sha256)
    dataset_id = str(binding.cohort_assets[0]["dataset_id"])
    monkeypatch.setattr(
        runner,
        "load_table3_joint_pass",
        lambda: ({dataset_id: {"hinge": True}}, 1),
    )


def promote_shared_fixture_to_formal(runner, output: Path) -> None:
    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["classification"] = "FORMAL"
    summary["mode"] = "formal"
    write_json(summary_path, summary)

    frozen_path = output / "frozen_config.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    frozen["classification"] = "FORMAL"
    frozen["mode"] = "formal"
    frozen.pop("frozen_config_sha256")
    frozen["frozen_config_sha256"] = canonical_sha256(frozen)
    write_json(frozen_path, frozen)

    proof = {
        "all_pass": True,
        "check_count": len(runner.FORMAL_VERIFICATION_CHECK_NAMES),
        "checks": [
            {"check": name, "detail": "fixture", "pass": True}
            for name in runner.FORMAL_VERIFICATION_CHECK_NAMES
        ],
    }
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["mode"] = "formal"
    manifest["verification"] = proof
    manifest["outputs"]["summary_sha256"] = hashlib.sha256(
        summary_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)


def test_table4_binding_freezes_actual_hashes_and_normalizes_absolute_packages(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0), package_row(tmp_path / "packages", 1)]
    table4_dir = write_table4_receipt(tmp_path, cohort)

    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=2,
        expected_j=2,
        expected_category_count=1,
    )
    manifest = runner.source_manifest_from_binding(binding)

    assert binding.state_records_sha256 == hashlib.sha256(
        (table4_dir / "state_records.jsonl").read_bytes()
    ).hexdigest()
    assert binding.asset_records_sha256 == hashlib.sha256(
        (table4_dir / "asset_records.jsonl").read_bytes()
    ).hexdigest()
    assert manifest["items"][0]["primary_urdf_relpath"] == str(
        Path(str(cohort[0]["package"])) / "model.urdf"
    )
    assert manifest["items"][0]["package_relpath"] == str(cohort[0]["package"])
    assert manifest["items"][0]["joint_specs"] == [
        {
            "lower": 0.0,
            "name": "hinge",
            "range_evaluable": True,
            "type": "revolute",
            "upper": 1.0,
            "xml_index": 0,
        }
    ]


def test_table4_binding_rejects_invalid_manifest_self_hash(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort, valid_self_hash=False)

    with pytest.raises(ValueError, match="self-hash"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=1,
            expected_j=1,
            expected_category_count=1,
        )


def test_table4_binding_rejects_package_binding_drift_even_when_refrozen(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort, tamper_item=0)

    with pytest.raises(ValueError, match="package binding"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=1,
            expected_j=1,
            expected_category_count=1,
        )


def test_table4_binding_rejects_per_asset_dof_redistribution_with_same_total(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [
        package_row(tmp_path / "packages", 0),
        package_row(tmp_path / "packages", 1),
    ]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    manifest_path = table4_dir / "frozen_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["items"][0]["movable_dof_count"] = 0
    manifest["items"][1]["movable_dof_count"] = 2
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    write_json(manifest_path, manifest)
    artifact_path = table4_dir / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["run_manifest_content_sha256"] = manifest["manifest_content_sha256"]
    artifact["files"]["frozen_manifest.json"] = {
        "bytes": manifest_path.stat().st_size,
        "sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
    }
    artifact.pop("artifact_manifest_content_sha256")
    artifact["artifact_manifest_content_sha256"] = canonical_sha256(artifact)
    write_json(artifact_path, artifact)

    with pytest.raises(ValueError, match="movable joint count"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=2,
            expected_j=2,
            expected_category_count=1,
        )


def test_table4_binding_rejects_upstream_artifact_without_valid_self_hash(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    artifact_path = table4_dir / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["artifact_manifest_content_sha256"] = "0" * 64
    write_json(artifact_path, artifact)

    with pytest.raises(ValueError, match="artifact self-hash"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=1,
            expected_j=1,
            expected_category_count=1,
        )


def test_table4_binding_rejects_refrozen_upstream_protocol_snapshot_drift(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    snapshot = table4_dir / "protocol_document_at_freeze.md"
    snapshot.write_text("# Refrozen upstream protocol drift\n", encoding="utf-8")
    artifact_path = table4_dir / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["files"][snapshot.name] = {
        "bytes": snapshot.stat().st_size,
        "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    }
    artifact.pop("artifact_manifest_content_sha256")
    artifact["artifact_manifest_content_sha256"] = canonical_sha256(artifact)
    write_json(artifact_path, artifact)

    with pytest.raises(ValueError, match="protocol snapshot"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=1,
            expected_j=1,
            expected_category_count=1,
        )


def test_table4_binding_rejects_symlinked_artifact_manifest(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    artifact_path = table4_dir / "artifact_manifest.json"
    external = tmp_path / "external_artifact_manifest.json"
    artifact_path.replace(external)
    artifact_path.symlink_to(external)

    with pytest.raises(ValueError, match="symlink"):
        runner.validate_table4_receipt(
            table4_dir,
            cohort,
            expected_n=1,
            expected_j=1,
            expected_category_count=1,
        )


def test_table4_strict_stream_accepts_records_in_worker_completion_order(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [
        package_row(tmp_path / "packages", 0),
        package_row(tmp_path / "packages", 1),
    ]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    manifest = json.loads((table4_dir / "frozen_manifest.json").read_text(encoding="utf-8"))
    rows = [
        {"dataset_id": cohort[1]["dataset_id"], "order": 1, "strict_collision_pass": False},
        {"dataset_id": cohort[0]["dataset_id"], "order": 0, "strict_collision_pass": True},
    ]
    path = table4_dir / "unordered_asset_records.jsonl"
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )

    assert runner._load_table4_strict_for_verification(path, manifest["items"]) == {
        str(cohort[0]["dataset_id"]): True,
        str(cohort[1]["dataset_id"]): False,
    }


def test_jobs_freeze_complete_package_file_binding(tmp_path: Path) -> None:
    runner = load_runner()
    job, _mesh = build_mesh_bound_job(runner, tmp_path)

    assert [row["path"] for row in job["expected_package_file_binding"]] == [
        "mesh.obj",
        "model.urdf",
    ]
    assert job["expected_package_file_binding_sha256"] == canonical_sha256(
        job["expected_package_file_binding"]
    )
    assert job["expected_package_content_manifest_sha256"] == job[
        "expected_package_file_binding_sha256"
    ]


def test_child_rejects_mesh_drift_before_genesis_replay(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    job, mesh = build_mesh_bound_job(runner, tmp_path)
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    write_json(job_path, job)
    original = mesh.read_text(encoding="utf-8")
    mesh.write_text(original.replace("v 1 0 0", "v 0 0 1"), encoding="utf-8")
    assert mesh.stat().st_size == job["expected_package_file_binding"][0]["bytes"]

    def must_not_replay(_job_path: Path, _result_path: Path) -> int:
        raise AssertionError("Genesis replay must not run on drifted package input")

    monkeypatch.setattr(runner, "_SHARED_RUN_CHILD", must_not_replay)
    monkeypatch.setattr(
        runner, "validate_early_cpu_affinity_receipt", complete_early_affinity
    )
    monkeypatch.setattr(
        runner, "bind_torch_threading", lambda: dict(runner.TORCH_THREAD_COUNTS)
    )
    assert runner.run_child(job_path, result_path) == 0
    record = json.loads(result_path.read_text(encoding="utf-8"))

    assert record["status"] == "error"
    assert record["issues"][0].startswith("input_binding_drift_before_evaluation")
    assert record["observed_package_file_binding_before_sha256"] != job[
        "expected_package_file_binding_sha256"
    ]
    assert record["observed_package_file_binding_after"] is None


def test_child_thread_binding_failure_never_delegates_to_genesis(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    job, _mesh = build_mesh_bound_job(runner, tmp_path)
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    write_json(job_path, job)

    def binding_failure() -> dict[str, int]:
        raise RuntimeError("fixture thread limit unavailable")

    def must_not_replay(_job_path: Path, _result_path: Path) -> int:
        raise AssertionError("Genesis replay must not run without a thread receipt")

    monkeypatch.setattr(runner, "bind_torch_threading", binding_failure)
    monkeypatch.setattr(
        runner, "validate_early_cpu_affinity_receipt", complete_early_affinity
    )
    monkeypatch.setattr(runner, "_SHARED_RUN_CHILD", must_not_replay)

    assert runner.run_child(job_path, result_path) == 0
    record = json.loads(result_path.read_text(encoding="utf-8"))
    assert record["status"] == "error"
    assert record["issues"][0].startswith("torch_thread_binding_failed")
    assert record["torch_threading"]["status"] == "FAILED"


def test_child_rejects_mesh_drift_during_genesis_replay(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    job, mesh = build_mesh_bound_job(runner, tmp_path)
    job_path = tmp_path / "job.json"
    result_path = tmp_path / "result.json"
    write_json(job_path, job)

    def drifting_replay(_job_path: Path, output: Path) -> int:
        original = mesh.read_text(encoding="utf-8")
        mesh.write_text(original.replace("v 1 0 0", "v 0 0 1"), encoding="utf-8")
        record = runner.failed_asset_record(job, "fixture")
        record["status"] = "completed"
        record["issues"] = []
        runner.atomic_json(output, record)
        return 0

    monkeypatch.setattr(runner, "_SHARED_RUN_CHILD", drifting_replay)
    monkeypatch.setattr(
        runner, "validate_early_cpu_affinity_receipt", complete_early_affinity
    )
    monkeypatch.setattr(
        runner, "bind_torch_threading", lambda: dict(runner.TORCH_THREAD_COUNTS)
    )
    assert runner.run_child(job_path, result_path) == 0
    record = json.loads(result_path.read_text(encoding="utf-8"))

    assert record["status"] == "error"
    assert record["issues"][0].startswith("input_binding_drift_after_evaluation")
    assert record["observed_package_file_binding_before_sha256"] == job[
        "expected_package_file_binding_sha256"
    ]
    assert record["observed_package_file_binding_after_sha256"] != job[
        "expected_package_file_binding_sha256"
    ]


def test_parent_timeout_record_preserves_expected_package_binding(tmp_path: Path) -> None:
    runner = load_runner()
    job, _mesh = build_mesh_bound_job(runner, tmp_path)

    record = runner.failed_asset_record(job, "asset_timeout after 1800s")

    assert record["status"] == "error"
    assert record["expected_package_file_binding"] == job[
        "expected_package_file_binding"
    ]
    assert record["expected_package_file_binding_sha256"] == job[
        "expected_package_file_binding_sha256"
    ]
    assert record["observed_package_file_binding_before"] is None
    assert record["observed_package_file_binding_after"] is None


def test_completed_record_package_attestation_is_fail_closed_on_drift(tmp_path: Path) -> None:
    runner = load_runner()
    job, _mesh = build_mesh_bound_job(runner, tmp_path)
    record = runner.failed_asset_record(job, "fixture")
    expected = job["expected_package_file_binding"]
    expected_sha256 = job["expected_package_file_binding_sha256"]
    record.update(
        {
            "status": "completed",
            "issues": [],
            "observed_package_file_binding_before": expected,
            "observed_package_file_binding_before_sha256": expected_sha256,
            "observed_package_file_binding_after": expected,
            "observed_package_file_binding_after_sha256": expected_sha256,
        }
    )
    assert runner.package_binding_attestation_valid(record) is True

    record["observed_package_file_binding_after_sha256"] = "0" * 64
    assert runner.package_binding_attestation_valid(record) is False


def test_package_attestation_is_bound_to_source_table4_item(tmp_path: Path) -> None:
    runner = load_runner()
    job, _mesh = build_mesh_bound_job(runner, tmp_path)
    record = runner.failed_asset_record(job, "fixture")
    source_item = {
        "dataset_id": job["dataset_id"],
        "order": job["selection_index"],
        "package": job["package"],
        "package_binding_content_manifest_sha256": job[
            "expected_package_content_manifest_sha256"
        ],
    }

    assert runner.package_binding_matches_source_item(record, source_item) is True
    record["expected_package_content_manifest_sha256"] = "0" * 64
    assert runner.package_binding_matches_source_item(record, source_item) is False


def test_zero_width_joint_stays_in_intended_denominator_and_is_marked_fail_closed(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0, lower=0.0, upper=0.0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )

    manifest = runner.source_manifest_from_binding(binding)
    jobs, _ = runner.build_jobs(
        manifest,
        {str(cohort[0]["dataset_id"]): {"hinge": True}},
        {},
        formal=False,
    )

    assert len(jobs) == 1
    assert jobs[0]["expected_movable_dof"] == 1
    assert jobs[0]["expected_state_count"] == 21
    assert jobs[0]["joints"][0]["range_evaluable"] is False
    assert jobs[0]["joints"][0]["values"] == []


def test_worker_receipt_merge_restores_non_evaluable_joint_fail_closed() -> None:
    runner = load_runner()
    valid_joint = {
        "name": "valid",
        "type": "revolute",
        "dof_position": 0,
        "xml_index": 0,
        "range_evaluable": True,
        "table3_joint_level_pass": True,
    }
    invalid_joint = {
        "name": "zero_width",
        "type": "revolute",
        "dof_position": 1,
        "xml_index": 1,
        "range_evaluable": False,
        "table3_joint_level_pass": False,
    }
    valid_record = {
        "joint_name": "valid",
        "states_intended": 21,
        "states_executed": 21,
        "full_range_cf_pass": True,
        "safe_dof": 1,
    }
    jobs = [
        {
            "selection_index": 0,
            "expected_state_count": 42,
            "joints": [valid_joint, invalid_joint],
        }
    ]
    records = [
        {
            "selection_index": 0,
            "status": "completed",
            "states_intended": 21,
            "states_executed": 21,
            "joint_records": [valid_record],
            "issues": [],
        }
    ]

    merged = runner.merge_range_failures(jobs, records)

    assert merged[0]["status"] == "error"
    assert merged[0]["states_intended"] == 42
    assert [row["joint_name"] for row in merged[0]["joint_records"]] == [
        "valid",
        "zero_width",
    ]
    assert merged[0]["joint_records"][0] is valid_record
    assert merged[0]["joint_records"][1]["states_executed"] == 0
    assert merged[0]["joint_records"][1]["full_range_cf_pass"] is False


def test_failed_worker_record_merge_preserves_source_joint_identity(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    manifest = runner.source_manifest_from_binding(binding)
    dataset_id = str(cohort[0]["dataset_id"])
    table3_pass = {dataset_id: {"hinge": True}}
    jobs, _ = runner.build_jobs(manifest, table3_pass, {}, formal=False)
    job = jobs[0]
    runnable = dict(job)
    runnable["joints"] = [
        dict(joint) for joint in job["joints"] if joint["range_evaluable"]
    ]
    runnable["expected_state_count"] = runner.SINGLE_SAMPLES * len(
        runnable["joints"]
    )

    failed = runner.failed_asset_record(runnable, "asset_timeout after 1800s")
    merged = runner.merge_range_failures(jobs, [failed])

    assert runner.execution_record_invariants_valid(merged, formal=False) is True
    assert runner.output_records_bind_source(
        merged,
        manifest["items"],
        table3_pass=table3_pass,
    ) is True
    assert merged[0]["joint_records"][0]["dof_position"] == 0
    assert merged[0]["joint_records"][0]["xml_index"] == 0
    assert merged[0]["input_identity_sha256"] == job["input_identity_sha256"]


def test_formal_contract_freezes_workers_timeout_and_denominators(tmp_path: Path) -> None:
    runner = load_runner()
    table4_dir = tmp_path / "table4"
    table4_dir.mkdir()
    args = runner.parse_args(["--mode", "formal", "--table4-dir", str(table4_dir)])
    runner.validate_cli_contract(args)

    assert args.workers == 1
    assert runner.CHILD_TIMEOUT_SECONDS == 1800
    assert runner.N_EVAL == 2655
    assert runner.J_EVAL == 14968
    assert runner.EXPECTED_CATEGORY_COUNT == 531
    assert runner.SINGLE_SAMPLES * runner.J_EVAL == 314328
    assert runner.EXPECTED_RANGE_EVALUABLE_JOINTS == 14943
    assert runner.EXPECTED_RANGE_INVALID_JOINTS == 25
    assert runner.EXPECTED_GENESIS_REPLAY_STATES == 313803
    assert runner.EXPECTED_FAIL_CLOSED_WITHOUT_REPLAY_STATES == 525
    assert "new output directory" in runner.RESUME_POLICY
    runner.configure_shared_runner(None)
    assert runner.shared.OPERATIONALIZATION["resume_policy"] == runner.RESUME_POLICY

    changed = runner.parse_args(
        ["--mode", "formal", "--table4-dir", str(table4_dir), "--workers", "5"]
    )
    with pytest.raises(ValueError, match="workers=1"):
        runner.validate_cli_contract(changed)
    with pytest.raises(SystemExit):
        runner.parse_args(
            ["--mode", "formal", "--table4-dir", str(table4_dir), "--resume"]
        )


def test_formal_verification_proof_must_match_run_manifest() -> None:
    runner = load_runner()
    proof = {
        "all_pass": True,
        "check_count": len(runner.FORMAL_VERIFICATION_CHECK_NAMES),
        "checks": [
            {"check": name, "detail": "fixture", "pass": True}
            for name in runner.FORMAL_VERIFICATION_CHECK_NAMES
        ],
    }
    assert runner.formal_verification_receipt_valid(proof, proof)

    manifest_proof = json.loads(json.dumps(proof))
    manifest_proof["checks"][0]["pass"] = False
    assert not runner.formal_verification_receipt_valid(proof, manifest_proof)

    malformed = json.loads(json.dumps(proof))
    malformed["checks"][0]["check"] = "invented_check"
    assert not runner.formal_verification_receipt_valid(malformed, malformed)


def test_main_captures_both_wall_clocks_before_source_receipt_hashing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    table4_dir = tmp_path / "table4"
    table4_dir.mkdir()
    output = tmp_path / "output"
    elapsed = {"seconds": 0.0}
    calls = {"utc": 0, "monotonic": 0}
    captured: dict[str, object] = {}
    base = dt.datetime(2026, 8, 24, tzinfo=dt.timezone.utc)

    def fake_utc_now() -> str:
        calls["utc"] += 1
        current = base + dt.timedelta(seconds=elapsed["seconds"])
        return current.isoformat().replace("+00:00", "Z")

    def fake_perf_counter() -> float:
        calls["monotonic"] += 1
        return elapsed["seconds"]

    def fake_run_scope(_args: object) -> int:
        elapsed["seconds"] = 10.0
        return 0

    def delayed_table4_receipt(_binding: object) -> dict[str, object]:
        captured["clock_calls_before_hashing"] = dict(calls)
        elapsed["seconds"] += 20.0
        return {"directory": str(table4_dir)}

    def capture_writer(_output: Path, **kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {"status": "PASS"}

    binding = type(
        "Binding",
        (),
        {"ordered_ids_sha256": runner.EXPECTED_ORDERED_IDS_SHA256},
    )()
    monkeypatch.setattr(runner, "utc_now", fake_utc_now)
    monkeypatch.setattr(runner.time, "perf_counter", fake_perf_counter)
    monkeypatch.setattr(runner, "load_canonical_cohort", lambda: [])
    monkeypatch.setattr(runner, "validate_table4_receipt", lambda *args, **kwargs: binding)
    monkeypatch.setattr(runner, "configure_shared_runner", lambda _binding: None)
    monkeypatch.setattr(runner.shared, "run_scope", fake_run_scope)
    monkeypatch.setattr(runner, "table4_input_receipt", delayed_table4_receipt)
    monkeypatch.setattr(runner, "write_post_run_receipts", capture_writer)

    assert runner.main(
        [
            "--mode",
            "smoke",
            "--table4-dir",
            str(table4_dir),
            "--output-dir",
            str(output),
        ]
    ) == 0
    assert captured["clock_calls_before_hashing"] == {"utc": 2, "monotonic": 2}
    assert captured["completed_at_utc"] == "2026-08-24T00:00:10Z"
    assert captured["wall_time_seconds"] == 10.0


def test_formal_output_verifier_rebinds_source_to_canonical_cohort(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    configure_mini_formal_contract(runner, monkeypatch, binding)
    monkeypatch.setattr(runner, "load_canonical_cohort", lambda: cohort)
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    promote_shared_fixture_to_formal(runner, output)
    runner.write_post_run_receipts(
        output,
        mode="formal",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    foreign = [package_row(tmp_path / "foreign", 0, category="foreign")]
    monkeypatch.setattr(runner, "load_canonical_cohort", lambda: foreign)
    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)

    assert result["status"] == "FAIL"
    assert result["checks"]["source_table4_files_match"] is False


def test_post_run_receipt_is_self_hashed_and_independently_verifiable(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    table4_inputs = runner.table4_input_receipt(binding)

    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=table4_inputs,
        workers=runner.FORMAL_WORKERS,
    )

    timing = json.loads((output / "timing.json").read_text(encoding="utf-8"))
    verification = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    artifact = json.loads((output / "artifact_manifest.json").read_text(encoding="utf-8"))
    assert timing["wall_time_seconds"] == 12.5
    assert timing["workers"] == 1
    assert timing["states_genesis_replay_intended"] == 21
    assert timing["states_fail_closed_without_replay"] == 0
    assert verification["status"] == "SMOKE"
    assert verification["table4_inputs"] == table4_inputs
    assert artifact["artifact_manifest_content_sha256"] == runner.artifact_manifest_self_hash(
        artifact
    )
    assert artifact["protocol_snapshot_sha256"] == hashlib.sha256(
        (output / "protocol_snapshot.md").read_bytes()
    ).hexdigest()
    assert artifact["source_table4"] == table4_inputs
    assert set(artifact["files"]) == {
        "frozen_config.json",
        "protocol_snapshot.md",
        "asset_records.jsonl",
        "joint_records.jsonl",
        "summary.json",
        "summary.md",
        "manifest.json",
        "timing.json",
        "verification.json",
    }
    for name, receipt in artifact["files"].items():
        path = output / name
        assert receipt == {
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "PASS", result
    assert all(result["checks"].values())
    assert runner.main(["--verify-output", str(output), "--table4-dir", str(table4_dir)]) == 0


@pytest.mark.parametrize("runtime_control", ("torch", "early_affinity"))
def test_verifier_rejects_refrozen_asset_runtime_control_drift(
    tmp_path: Path, runtime_control: str
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    asset_path = output / "asset_records.jsonl"
    asset = json.loads(asset_path.read_text(encoding="utf-8"))
    if runtime_control == "torch":
        asset["torch_threading"]["after_evaluation"]["inter_op_threads"] = 96
    else:
        asset["early_cpu_affinity"]["observed"][0] = 96
    asset_path.write_text(json.dumps(asset, sort_keys=True) + "\n", encoding="utf-8")
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["asset_records_sha256"] = hashlib.sha256(
        asset_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(runner, output, "asset_records.jsonl", "manifest.json")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["execution_invariants_valid"] is False


@pytest.mark.parametrize("runtime_control", ("torch", "early_affinity"))
def test_verifier_rejects_refrozen_runtime_control_policy_drift(
    tmp_path: Path, runtime_control: str
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    frozen_path = output / "frozen_config.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    if runtime_control == "torch":
        frozen["operationalization"]["torch_threading"]["expected"][
            "inter_op_threads"
        ] = 96
    else:
        frozen["operationalization"]["early_cpu_affinity"][
            "child_cpu_affinity_width"
        ] = 96
    frozen.pop("frozen_config_sha256")
    frozen["frozen_config_sha256"] = canonical_sha256(frozen)
    write_json(frozen_path, frozen)
    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["frozen_config_content_sha256"] = frozen["frozen_config_sha256"]
    artifact["files"]["frozen_config.json"] = {
        "bytes": frozen_path.stat().st_size,
        "sha256": hashlib.sha256(frozen_path.read_bytes()).hexdigest(),
    }
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(
        artifact
    )
    write_json(artifact_path, artifact)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["frozen_config_self_hash_valid"] is True
    assert result["checks"]["execution_controls_match"] is False


def test_verifier_rejects_refrozen_protocol_snapshot_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    snapshot = output / "protocol_snapshot.md"
    snapshot.write_text("# Drifted protocol\n", encoding="utf-8")
    artifact = json.loads((output / "artifact_manifest.json").read_text(encoding="utf-8"))
    artifact["files"]["protocol_snapshot.md"] = {
        "bytes": snapshot.stat().st_size,
        "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    }
    artifact["protocol_snapshot_sha256"] = artifact["files"]["protocol_snapshot.md"]["sha256"]
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(artifact)
    write_json(output / "artifact_manifest.json", artifact)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["protocol_snapshot_binding"] is False


def test_verifier_rejects_fully_refrozen_protocol_snapshot_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    snapshot = output / "protocol_snapshot.md"
    snapshot.write_text("# Fully refrozen drift\n", encoding="utf-8")
    snapshot_sha256 = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    frozen_path = output / "frozen_config.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    frozen["protocol_document_sha256"] = snapshot_sha256
    frozen.pop("frozen_config_sha256")
    frozen["frozen_config_sha256"] = canonical_sha256(frozen)
    write_json(frozen_path, frozen)
    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["protocol_snapshot_sha256"] = snapshot_sha256
    artifact["frozen_config_content_sha256"] = frozen["frozen_config_sha256"]
    write_json(artifact_path, artifact)
    refresh_output_file_bindings(
        runner, output, "protocol_snapshot.md", "frozen_config.json"
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["protocol_snapshot_binding"] is False


def test_verifier_rejects_refrozen_wall_clock_endpoint_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    refreeze_wall_clock(
        runner,
        output,
        measurement_endpoint="forged post-publication endpoint",
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["wall_clock_valid"] is False


def test_verifier_rejects_refrozen_utc_monotonic_duration_mismatch(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    refreeze_wall_clock(
        runner,
        output,
        started_at_utc=utc_seconds_ago(3600),
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["wall_clock_valid"] is False


def test_verifier_rejects_refrozen_summary_headline_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["metrics"]["joint_level_full_range_cf"]["numerator"] = 0
    write_json(summary_path, summary)
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["summary_sha256"] = hashlib.sha256(
        summary_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(runner, output, "summary.json", "manifest.json")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["derived_outputs_match"] is False


def test_verifier_rejects_refrozen_summary_markdown_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    summary_md = output / "summary.md"
    summary_md.write_text("# Refrozen but false public summary\n", encoding="utf-8")
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["summary_md_sha256"] = hashlib.sha256(
        summary_md.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(runner, output, "summary.md", "manifest.json")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["derived_outputs_match"] is False


def test_execution_invariants_reject_impossible_and_negative_state_counts() -> None:
    runner = load_runner()
    joint = {
        "full_range_cf_pass": False,
        "illegal_states": 0,
        "issues": [],
        "safe_dof": 0,
        "states_executed": 22,
        "states_intended": 21,
    }
    asset = {
        "expected_movable_dof": 1,
        "joint_records": [joint],
        "state_hash_cross_check": {"verified": 22, "mismatch": 0, "no_reference": 0},
        "states_executed": 22,
        "states_intended": 21,
    }
    assert not runner.execution_record_invariants_valid([asset], formal=True)

    joint["states_executed"] = 0
    asset["states_executed"] = 0
    asset["state_hash_cross_check"] = {
        "verified": 1,
        "mismatch": -1,
        "no_reference": 0,
    }
    assert not runner.execution_record_invariants_valid([asset], formal=True)


def test_formal_verifier_rejects_coherently_refrozen_impossible_execution_count(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    configure_mini_formal_contract(runner, monkeypatch, binding)
    monkeypatch.setattr(runner, "load_canonical_cohort", lambda: cohort)
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    promote_shared_fixture_to_formal(runner, output)
    runner.write_post_run_receipts(
        output,
        mode="formal",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    asset_path = output / "asset_records.jsonl"
    asset = json.loads(asset_path.read_text(encoding="utf-8"))
    asset["states_executed"] = 22
    asset["state_hash_cross_check"]["verified"] = 22
    asset["joint_records"][0]["states_executed"] = 22
    asset["joint_records"][0]["state_summaries"].append(
        {"executed": True, "illegal_collision": False, "sample_index": 21}
    )
    asset_path.write_text(json.dumps(asset, sort_keys=True) + "\n", encoding="utf-8")

    joint_path = output / "joint_records.jsonl"
    joint = json.loads(joint_path.read_text(encoding="utf-8"))
    joint["states_executed"] = 22
    joint_path.write_text(json.dumps(joint, sort_keys=True) + "\n", encoding="utf-8")

    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["metrics"] = runner._recompute_summary_metrics(
        [asset],
        {str(cohort[0]["dataset_id"]): True},
    )
    summary["status_counts"] = summary["metrics"]["status_counts"]
    write_json(summary_path, summary)
    (output / "summary.md").write_text(
        runner._render_summary_markdown(summary),
        encoding="utf-8",
    )

    timing_path = output / "timing.json"
    timing = json.loads(timing_path.read_text(encoding="utf-8"))
    timing["states_executed"] = 22
    write_json(timing_path, timing)
    verification_path = output / "verification.json"
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    verification["denominators"]["states_executed"] = 22
    write_json(verification_path, verification)

    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"] = {
        "asset_records_sha256": hashlib.sha256(asset_path.read_bytes()).hexdigest(),
        "joint_records_sha256": hashlib.sha256(joint_path.read_bytes()).hexdigest(),
        "summary_md_sha256": hashlib.sha256(
            (output / "summary.md").read_bytes()
        ).hexdigest(),
        "summary_sha256": hashlib.sha256(summary_path.read_bytes()).hexdigest(),
    }
    write_json(manifest_path, manifest)

    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["denominators"]["states_executed"] = 22
    write_json(artifact_path, artifact)
    refresh_output_file_bindings(
        runner,
        output,
        "asset_records.jsonl",
        "joint_records.jsonl",
        "summary.json",
        "summary.md",
        "timing.json",
        "verification.json",
        "manifest.json",
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["execution_invariants_valid"] is False

def test_verifier_rejects_refrozen_flat_joint_record_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    joint_path = output / "joint_records.jsonl"
    joint = json.loads(joint_path.read_text(encoding="utf-8"))
    joint["safe_dof"] = 0
    joint_path.write_text(json.dumps(joint, sort_keys=True) + "\n", encoding="utf-8")
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["joint_records_sha256"] = hashlib.sha256(
        joint_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(runner, output, "joint_records.jsonl", "manifest.json")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["derived_outputs_match"] is False


def test_verifier_rejects_refrozen_safe_dof_and_limit_semantic_drift(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    asset_path = output / "asset_records.jsonl"
    asset = json.loads(asset_path.read_text(encoding="utf-8"))
    asset["joint_records"][0]["table3_joint_level_pass"] = False
    asset["joint_records"][0]["limit_endpoints_executed"] = 0
    asset_path.write_text(json.dumps(asset, sort_keys=True) + "\n", encoding="utf-8")
    joint_path = output / "joint_records.jsonl"
    joint = json.loads(joint_path.read_text(encoding="utf-8"))
    joint["table3_joint_level_pass"] = False
    joint["limit_endpoints_executed"] = 0
    joint_path.write_text(json.dumps(joint, sort_keys=True) + "\n", encoding="utf-8")
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["asset_records_sha256"] = hashlib.sha256(
        asset_path.read_bytes()
    ).hexdigest()
    manifest["outputs"]["joint_records_sha256"] = hashlib.sha256(
        joint_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(
        runner,
        output,
        "asset_records.jsonl",
        "joint_records.jsonl",
        "manifest.json",
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["execution_invariants_valid"] is False


def test_verifier_rejects_refrozen_output_joint_identity_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )

    asset_path = output / "asset_records.jsonl"
    asset = json.loads(asset_path.read_text(encoding="utf-8"))
    asset["joint_records"][0]["joint_name"] = "forged_joint"
    asset_path.write_text(json.dumps(asset, sort_keys=True) + "\n", encoding="utf-8")
    joint_path = output / "joint_records.jsonl"
    joint = json.loads(joint_path.read_text(encoding="utf-8"))
    joint["joint_name"] = "forged_joint"
    joint_path.write_text(json.dumps(joint, sort_keys=True) + "\n", encoding="utf-8")
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["outputs"]["asset_records_sha256"] = hashlib.sha256(
        asset_path.read_bytes()
    ).hexdigest()
    manifest["outputs"]["joint_records_sha256"] = hashlib.sha256(
        joint_path.read_bytes()
    ).hexdigest()
    write_json(manifest_path, manifest)
    refresh_output_file_bindings(
        runner,
        output,
        "asset_records.jsonl",
        "joint_records.jsonl",
        "manifest.json",
    )

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["output_source_bindings_match"] is False


def test_output_source_binding_accepts_frozen_smoke_prefix(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [
        package_row(tmp_path / "packages", 0),
        package_row(tmp_path / "packages", 1),
    ]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=2,
        expected_j=2,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    assets = runner._read_jsonl(output / "asset_records.jsonl")

    assert runner.output_records_bind_source(
        assets,
        binding.manifest["items"],
        table3_pass=None,
    )


def test_verifier_rejects_upstream_table4_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    (table4_dir / "state_records.jsonl").write_text('{"drift":true}\n', encoding="utf-8")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["source_table4_files_match"] is False


def test_verifier_rejects_current_package_input_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    urdf = Path(str(cohort[0]["package"])) / "model.urdf"
    urdf.write_text(urdf.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["package_input_bindings_match"] is False


def test_verifier_rejects_malformed_artifact_file_binding_after_refreeze(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    artifact = json.loads((output / "artifact_manifest.json").read_text(encoding="utf-8"))
    artifact["files"]["summary.md"] = "not-a-file-binding"
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(artifact)
    write_json(output / "artifact_manifest.json", artifact)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["artifact_files_closed"] is False


def test_verifier_rejects_string_typed_artifact_byte_count(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    artifact_path = output / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    artifact["files"]["summary.md"]["bytes"] = str(
        artifact["files"]["summary.md"]["bytes"]
    )
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(
        artifact
    )
    write_json(artifact_path, artifact)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["artifact_files_closed"] is False


def test_verifier_rejects_unlisted_top_level_output(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    (output / "unlisted.txt").write_text("not closed\n", encoding="utf-8")

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["artifact_files_closed"] is False


def test_verifier_rejects_symlinked_artifact_manifest(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    artifact_path = output / "artifact_manifest.json"
    external = tmp_path / "external_output_artifact_manifest.json"
    artifact_path.replace(external)
    artifact_path.symlink_to(external)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"


def test_verifier_rejects_refrozen_frozen_config_binding_drift(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    artifact = json.loads((output / "artifact_manifest.json").read_text(encoding="utf-8"))
    artifact["frozen_config_content_sha256"] = "0" * 64
    artifact["artifact_manifest_content_sha256"] = runner.artifact_manifest_self_hash(artifact)
    write_json(output / "artifact_manifest.json", artifact)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["frozen_config_self_hash_valid"] is False


def test_verifier_requires_static_inputs_to_match_frozen_pins(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    runner.write_post_run_receipts(
        output,
        mode="smoke",
        return_code=0,
        started_at_utc=utc_seconds_ago(12.5),
        completed_at_utc=runner.utc_now(),
        wall_time_seconds=12.5,
        table4_inputs=runner.table4_input_receipt(binding),
        workers=runner.FORMAL_WORKERS,
    )
    monkeypatch.setattr(runner, "COHORT_FILE_SHA256", "0" * 64)

    result = runner.verify_output_receipt(output, expected_table4_dir=table4_dir)
    assert result["status"] == "FAIL"
    assert result["checks"]["static_inputs_match"] is False


def test_post_run_publication_gate_rejects_unbound_protocol(tmp_path: Path) -> None:
    runner = load_runner()
    cohort = [package_row(tmp_path / "packages", 0)]
    table4_dir = write_table4_receipt(tmp_path, cohort)
    binding = runner.validate_table4_receipt(
        table4_dir,
        cohort,
        expected_n=1,
        expected_j=1,
        expected_category_count=1,
    )
    output = tmp_path / "run"
    write_shared_run_outputs(runner, output, binding)
    frozen = json.loads((output / "frozen_config.json").read_text(encoding="utf-8"))
    frozen["protocol_document_sha256"] = "0" * 64
    frozen.pop("frozen_config_sha256")
    frozen["frozen_config_sha256"] = canonical_sha256(frozen)
    write_json(output / "frozen_config.json", frozen)

    with pytest.raises(RuntimeError, match="publication gate"):
        runner.write_post_run_receipts(
            output,
            mode="smoke",
            return_code=0,
            started_at_utc=utc_seconds_ago(12.5),
            completed_at_utc=runner.utc_now(),
            wall_time_seconds=12.5,
            table4_inputs=runner.table4_input_receipt(binding),
            workers=runner.FORMAL_WORKERS,
        )
