from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table2_urdf_physx_mobility.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table2_urdf_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


GOOD_URDF = """<robot name="fixture">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="child">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="2" ixy="0" ixz="0" iyy="2" iyz="0" izz="2"/>
    </inertial>
    <collision><geometry><box size="0.5 0.5 0.5"/></geometry></collision>
  </link>
  <joint name="j0" type="revolute">
    <parent link="base"/><child link="child"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
"""


def _write_package(root: Path, name: str, urdf_text: str = GOOD_URDF) -> Path:
    package = root / name
    package.mkdir(parents=True)
    urdf = package / "model.urdf"
    urdf.write_text(urdf_text, encoding="utf-8")
    return package


def _build_job(runner, package: Path, *, asset_id: str = "10", tamper_binding: bool = False) -> dict:
    binding = runner.TABLE2.package_binding(package)
    if tamper_binding:
        binding = {**binding, "content_manifest_sha256": "0" * 64}
    return {
        "asset_id": asset_id,
        "dataset_id": int(asset_id),
        "rank": 1,
        "rank_sha256": "r" + asset_id,
        "selection_index": 0,
        "raw_category": "Fixture",
        "package": str(package),
        "package_binding": binding,
        "run_standard_parser": True,
        "primary_urdf_relative_path": "model.urdf",
        "urdf_relative_path": "model.urdf",
        "model_urdf_sha256": _sha256_file(package / "model.urdf"),
        "primary_urdf_sha256": _sha256_file(package / "model.urdf"),
        "finaljson_sha256": "f" + asset_id,
        "resource_sha256": "s" + asset_id,
        "manifest_row_sha256": "m" + asset_id,
        "manifest_content_sha256": "manifest-" + asset_id,
        "evaluator_binding": runner.evaluator_binding(),
        "environment_binding": runner.environment_binding(),
    }


def _run_child_process(runner, tmp_path: Path, job: dict, name: str = "job_000000") -> subprocess.CompletedProcess:
    job_root = tmp_path / name
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    job_path.write_text(json.dumps(job, sort_keys=True), encoding="utf-8")
    return subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--internal-child-job",
            str(job_path),
            "--internal-child-result",
            str(result_path),
        ],
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_child_audit_passes_clean_package(tmp_path: Path) -> None:
    runner = _load_runner()
    package = _write_package(tmp_path, "10")
    job = _build_job(runner, package)

    completed = _run_child_process(runner, tmp_path, job)
    assert completed.returncode == 0, completed.stderr

    record = json.loads((tmp_path / "job_000000" / "result.json").read_text(encoding="utf-8"))
    assert record["status"] == "completed"
    assert record["result_origin"] == "child_attested"
    for metric_name in runner.TABLE2.METRIC_NAMES:
        assert record["metrics"][metric_name]["pass"] is True, metric_name
    assert record["strict_urdf_pass"] is True
    assert record["model_urdf_sha256"] == job["model_urdf_sha256"]
    assert record["package_content_manifest_sha256"] == job["package_binding"]["content_manifest_sha256"]
    assert record["manifest_content_sha256"] == job["manifest_content_sha256"]
    assert record["raw_category"] == "Fixture"


def test_child_fails_closed_on_nonfinite_mass(tmp_path: Path) -> None:
    runner = _load_runner()
    urdf_text = GOOD_URDF.replace('<mass value="1.0"/>', '<mass value="nan"/>', 1)
    package = _write_package(tmp_path, "20", urdf_text)
    job = _build_job(runner, package, asset_id="20")

    completed = _run_child_process(runner, tmp_path, job)
    assert completed.returncode == 0, completed.stderr

    record = json.loads((tmp_path / "job_000000" / "result.json").read_text(encoding="utf-8"))
    assert record["status"] == "completed"
    assert record["metrics"]["parse_rate"]["pass"] is True
    assert record["metrics"]["finite_fields"]["pass"] is False
    assert record["metrics"]["inertia_validity"]["pass"] is False
    assert record["metrics"]["strict_urdf_pass"]["pass"] is False
    assert record["strict_urdf_pass"] is False


def test_child_fails_closed_on_missing_mesh(tmp_path: Path) -> None:
    runner = _load_runner()
    urdf_text = GOOD_URDF.replace(
        '<collision><geometry><box size="0.5 0.5 0.5"/></geometry></collision>',
        '<collision><geometry><mesh filename="missing.obj"/></geometry></collision>',
        1,
    )
    package = _write_package(tmp_path, "30", urdf_text)
    job = _build_job(runner, package, asset_id="30")

    completed = _run_child_process(runner, tmp_path, job)
    assert completed.returncode == 0, completed.stderr

    record = json.loads((tmp_path / "job_000000" / "result.json").read_text(encoding="utf-8"))
    assert record["metrics"]["parse_rate"]["pass"] is False
    assert record["metrics"]["resource_resolution"]["pass"] is False
    assert record["metrics"]["collision_coverage"]["pass"] is False
    assert record["metrics"]["strict_urdf_pass"]["pass"] is False


def test_child_rejects_evaluator_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    package = _write_package(tmp_path, "40")
    job = _build_job(runner, package, asset_id="40")
    job["evaluator_binding"] = {**job["evaluator_binding"], "evaluator_sha256": "0" * 64}

    completed = _run_child_process(runner, tmp_path, job)
    assert completed.returncode == runner.RUNTIME_BINDING_FATAL_EXIT_CODE
    assert "FATAL_RUNTIME_BINDING" in completed.stderr


def test_child_detects_source_change(tmp_path: Path) -> None:
    runner = _load_runner()
    package = _write_package(tmp_path, "50")
    job = _build_job(runner, package, asset_id="50", tamper_binding=True)

    completed = _run_child_process(runner, tmp_path, job)
    assert completed.returncode == 0, completed.stderr

    record = json.loads((tmp_path / "job_000000" / "result.json").read_text(encoding="utf-8"))
    assert record["status"] == "error"
    assert record["error"] == "source_changed_before_audit"
    for metric_name in runner.TABLE2.METRIC_NAMES:
        assert record["metrics"][metric_name]["pass"] is False


def test_stage_all_packages_rejects_byte_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    dataset_root = tmp_path / "dataset"
    (dataset_root / "urdf").mkdir(parents=True)
    (dataset_root / "partseg/10/objs").mkdir(parents=True)
    urdf = dataset_root / "urdf/10.urdf"
    urdf.write_text(GOOD_URDF, encoding="utf-8")
    obj = dataset_root / "partseg/10/objs/10.obj"
    obj.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
    row = {
        "dataset_id": 10,
        "urdf_relative_path": "urdf/10.urdf",
        "urdf_sha256": _sha256_file(urdf),
        "resources": [{"relative_path": "partseg/10/objs/10.obj", "sha256": _sha256_file(obj)}],
    }

    staged = runner.stage_all_packages(dataset_root, [row], tmp_path / "staging")
    assert (tmp_path / "staging/10/urdf/10.urdf").is_file()
    assert (tmp_path / "staging/10/partseg/10/objs/10.obj").is_file()
    assert staged[10]["package_binding"]["file_count"] == 2

    obj.write_text("v 9 9 9\n", encoding="utf-8")
    with pytest.raises(ValueError, match="staged byte mismatch"):
        runner.stage_all_packages(dataset_root, [row], tmp_path / "staging2")


def test_harness_timeout_is_fail_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    package = _write_package(tmp_path, "60")
    job = _build_job(runner, package, asset_id="60")
    job["internal_test_action"] = "sleep"
    job["sleep"] = 10.0

    records = runner.execute_audit_jobs(
        [job],
        worker_scratch=tmp_path / "scratch",
        timeout_seconds=0.7,
        max_workers=1,
        on_result=lambda record: None,
    )

    assert len(records) == 1
    record = records[0]
    assert record["status"] == "timeout"
    assert record["result_origin"] == "parent_synthesized"
    for metric_name in runner.TABLE2.METRIC_NAMES:
        assert record["metrics"][metric_name]["pass"] is False
    assert record["strict_urdf_pass"] is False
    assert record["worker_evidence"]["termination"] in {
        "sigterm",
        "sigkill_after_grace",
        "sigterm_orphaned_group",
        "sigkill_orphaned_group",
    }


def test_harness_end_to_end_aggregation(tmp_path: Path) -> None:
    runner = _load_runner()
    good = _write_package(tmp_path, "10")
    bad = _write_package(tmp_path, "20", GOOD_URDF.replace('<mass value="1.0"/>', '<mass value="nan"/>', 1))
    jobs = [
        _build_job(runner, good, asset_id="10"),
        _build_job(runner, bad, asset_id="20"),
    ]
    jobs[1]["raw_category"] = "Broken"

    records = runner.execute_audit_jobs(
        jobs,
        worker_scratch=tmp_path / "scratch",
        timeout_seconds=120.0,
        max_workers=2,
        on_result=lambda record: None,
    )
    assert [record["status"] for record in records] == ["completed", "completed"]

    summary = runner.TABLE2.aggregate_records(records, 2, category_field="raw_category")
    assert summary["n_eval"] == 2
    assert summary["metrics"]["strict_urdf_pass"]["passed"] == 1
    assert summary["metrics"]["finite_fields"]["passed"] == 1
    assert summary["metrics"]["parse_rate"]["passed"] == 2
    macro = summary["category_macro"]
    assert macro["state"] == "evaluated"
    assert macro["category_count"] == 2
    assert macro["metrics"]["strict_urdf_pass"]["rate"] == 0.5
