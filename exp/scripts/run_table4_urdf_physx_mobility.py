#!/usr/bin/env python3
"""Evaluate Table 4 on the frozen PhysX-Mobility N=800 cohort.

The cohort is the frozen Table 5 receipt set
(``exp/runtime/table5_physx_mobility_n800_v2``). Collision and
mechanical-clearance metrics are computed by the frozen PyBullet Table 4
core (``run_urdf_table4_partnet_mobility.py``, SHA-256 bound), the same
core used by the PartNet-Mobility and LAM Table 4 formal runs:
URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION |
URDF_USE_SELF_COLLISION_INCLUDE_PARENT | URDF_IGNORE_VISUAL_SHAPES,
direct parent-child pair exclusion, illegal-penetration threshold 1e-6 m,
K=21 single-joint sweep states, R=64 Sobol multi-joint states
(seed 20260813), q=0 rest state.

PhysX-Mobility release geometry places resources in ``partseg/`` as a
sibling of the flat ``urdf/`` directory, so each asset is staged into a
self-contained package replicating the release geometry (byte-for-byte,
hash-verified against the frozen manifest row) before the child audit.

Scale source (frozen for this run): the per-asset release OBJ-vertex
union bounding-box diagonal bound in the frozen Table 5 manifest
(``bounding_box_diagonal``), the PhysX analogue of the release
bounding-box diagonal used by the PartNet-Mobility Table 4 run. The
official URDFs declare zero collision elements, so all PyBullet contact
queries over this cohort are empty; the claim boundary is recorded in
the summary and the Table 5 receipt set's frozen ``strict_collision:
N/E`` adjudication is cross-referenced.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import types
from typing import Any
import uuid


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
DEFAULT_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
CORE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_partnet_mobility.py"
TABLE1_RUNNER_PATH = REPO / "exp/scripts/run_table1_physx_mobility.py"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
SCRIPT = Path(__file__).resolve()
EXPECTED_CORE_SHA256 = "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
PROTOCOL_ID = "urdf_sim_ready_table4_physx_mobility_n800_v1"
SCALE_PROTOCOL = "frozen_table5_release_obj_vertex_union_bbox_diagonal_v1"
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def load_core() -> Any:
    source_bytes = CORE_SCRIPT.read_bytes()
    observed_sha = hashlib.sha256(source_bytes).hexdigest()
    if observed_sha != EXPECTED_CORE_SHA256:
        raise RuntimeError(
            f"collision core drifted from frozen SHA-256: {observed_sha} != {EXPECTED_CORE_SHA256}"
        )
    module = types.ModuleType("table4_pybullet_collision_core_shared")
    module.__file__ = str(CORE_SCRIPT)
    code = compile(source_bytes, str(CORE_SCRIPT), "exec", dont_inherit=True)
    exec(code, module.__dict__)  # noqa: S102 - frozen evaluator core
    return module


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


TABLE1P = _load_module(TABLE1_RUNNER_PATH, "run_table4_physx_table1_cohort_shared")


def stage_package(dataset_root: Path, row: dict[str, Any], staging_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve()
    dataset_id = int(row["dataset_id"])
    package = staging_root / str(dataset_id)
    items = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    items.extend(
        (str(resource["relative_path"]), str(resource["sha256"]))
        for resource in row["resources"]
    )
    seen: set[str] = set()
    for relative, expected_sha256 in items:
        if relative in seen:
            raise ValueError(f"duplicate resource binding: {relative}")
        seen.add(relative)
        source = dataset_root / relative
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if sha256_file(target) != expected_sha256:
            raise ValueError(f"staged byte mismatch: {dataset_id}: {relative}")
    binding = package_binding(package)
    return {"package": package, "package_binding": binding}


def package_binding(package: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(f"package contains symlink: {path.relative_to(package)}")
            rows.append(
                {
                    "path": path.relative_to(package).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    rows.sort(key=lambda item: item["path"])
    return {
        "file_count": len(rows),
        "total_bytes": sum(item["bytes"] for item in rows),
        "content_manifest_sha256": canonical_sha256(rows),
    }


def build_item(row: dict[str, Any], staged: dict[str, Any], order: int) -> dict[str, Any]:
    core = load_core()
    dataset_id = int(row["dataset_id"])
    diagonal = float(row["bounding_box_diagonal"])
    if diagonal <= 0:
        raise ValueError(f"non-positive release bbox diagonal: {dataset_id}")
    joint_specs = core.parse_urdf_joints(
        Path(staged["package"]) / str(row["urdf_relative_path"])
    )
    return {
        "joint_specs": joint_specs,
        "protocol_id": PROTOCOL_ID,
        "order": order,
        "dataset_id": str(dataset_id),
        "rank": int(row["rank"]),
        "rank_sha256": str(row["rank_sha256"]),
        "category": str(row["category"]),
        "object_bbox_diagonal_m": diagonal,
        "movable_dof_count": int(row["xml_counts"]["movable_joints"]),
        "declared_collision_elements": int(row["xml_counts"]["collision_elements"]),
        "package": str(staged["package"]),
        "urdf_relative_path": str(row["urdf_relative_path"]),
        "urdf_sha256": str(row["urdf_sha256"]),
        "finaljson_sha256": str(row["finaljson_sha256"]),
        "resource_sha256": str(row["resource_sha256"]),
        "manifest_row_sha256": TABLE1P.TABLE5.canonical_sha256(row),
        "package_binding": staged["package_binding"],
        "rest_state_expected": 1,
        "single_state_expected": core.SINGLE_SAMPLES * len(joint_specs),
        "sobol_state_expected": core.SOBOL_SAMPLES if joint_specs else 0,
    }


def evaluate_physx_asset(item: dict[str, Any], core: Any) -> dict[str, Any]:
    import pybullet as bullet

    result = core.failure_record(item, "evaluation_not_completed")
    result["issues"] = []
    result["state_records"] = []
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    result["runtime_identity"] = core.current_runtime_identity()
    result["scale_protocol"] = SCALE_PROTOCOL

    package = Path(item["package"])
    urdf_path = package / str(item["urdf_relative_path"])
    joints = core.parse_urdf_joints(urdf_path)
    result["movable_dof_count"] = len(joints)
    result["range_evaluable_dof_count"] = sum(bool(row["range_evaluable"]) for row in joints)
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = index
        missing_names = [row["name"] for row in joints if row["name"] not in simulator_by_name]
        if missing_names:
            raise RuntimeError(f"simulator joint mapping missing: {missing_names}")
        joint_indices = [simulator_by_name[row["name"]] for row in joints]
        for index in joint_indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        direct_pairs = core._direct_parent_pairs(bullet, body, client)
        rest_values = [0.0] * len(joints)

        def observe(values: list[float], phase: str, sample_index: int, joint_name: str | None = None) -> dict[str, Any]:
            collision, readback_error = core._reset_and_observe(
                bullet, body, client, joint_indices, values, direct_pairs
            )
            metric_key = (
                "all_pair_max_penetration_m" if phase == "rest" else "non_adjacent_max_penetration_m"
            )
            state = {
                "dataset_id": item["dataset_id"],
                "category": item["category"],
                "protocol_id": item["protocol_id"],
                "order": item["order"],
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(values),
                "reset_readback_max_abs_error": readback_error,
                "metric_max_penetration_m": float(collision[metric_key]),
                **collision,
            }
            result["state_records"].append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["rest_all_pair_cf"] = rest["all_pair_illegal_penetration_count"] == 0
        result["rest_non_adjacent_cf"] = rest["non_adjacent_illegal_penetration_count"] == 0
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])

        joint_sweep_passes = 0
        for joint_position, row in enumerate(joints):
            if not row["range_evaluable"]:
                result["issues"].append(f"joint_range_not_evaluable:{row['name']}")
                continue
            joint_free = True
            for sample_index, value in enumerate(core.single_joint_values(row)):
                values = list(rest_values)
                values[joint_position] = value
                state = observe(values, "single_joint_sweep", sample_index, str(row["name"]))
                result["single_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["single_non_adjacent_free"] += int(free)
                joint_free = joint_free and free
            joint_sweep_passes += int(joint_free)
        result["joint_single_sweep_cf_passed"] = joint_sweep_passes

        if joints and all(row["range_evaluable"] for row in joints):
            for sample_index, values in enumerate(core.sobol_joint_values(joints)):
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                result["sobol_non_adjacent_free"] += int(
                    state["non_adjacent_illegal_penetration_count"] == 0
                )

        result["single_joint_sweep_cf"] = bool(
            result["single_state_executed"] == result["single_state_expected"]
            and result["single_non_adjacent_free"] == result["single_state_expected"]
        )
        result["multi_joint_sobol_cf"] = bool(
            result["movable_dof_count"] > 0
            and result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
            and result["sobol_non_adjacent_free"] == result["sobol_state_expected"]
        )
        expected_total = sum(
            int(result[f"{phase}_state_expected"]) for phase in ("rest", "single", "sobol")
        )
        executed_total = sum(
            int(result[f"{phase}_state_executed"]) for phase in ("rest", "single", "sobol")
        )
        result["measurement_complete"] = bool(
            result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and executed_total == expected_total
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
        if result["issues"] == ["evaluation_not_completed"]:
            result["issues"] = []
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    states = result["state_records"]
    if states:
        result["max_penetration_m"] = max(float(state["metric_max_penetration_m"]) for state in states)
        result["max_penetration_normalized"] = (
            float(result["max_penetration_m"]) / float(item["object_bbox_diagonal_m"])
        )
        result["max_reset_readback_error"] = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
    result["state_records_sha256"] = canonical_sha256(states)
    return result


def fail_closed(item: dict[str, Any], reason: str, *, timed_out: bool) -> dict[str, Any]:
    core = load_core()
    result = core.failure_record(item, reason, timed_out=timed_out)
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    result["scale_protocol"] = SCALE_PROTOCOL
    result["result_origin"] = "parent_synthesized"
    return result


def validate_result(item: dict[str, Any], result: dict[str, Any]) -> None:
    if str(result.get("dataset_id")) != str(item["dataset_id"]):
        raise ValueError("result dataset_id mismatch")
    required = (
        "load_success",
        "measurement_complete",
        "rest_all_pair_cf",
        "rest_non_adjacent_cf",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
        "strict_collision_pass",
        "rest_state_expected",
        "rest_state_executed",
        "single_state_expected",
        "single_state_executed",
        "sobol_state_expected",
        "sobol_state_executed",
    )
    for field in required:
        if field not in result:
            raise ValueError(f"result missing field: {field}")
    recomputed_strict = bool(
        result["measurement_complete"]
        and result["rest_non_adjacent_cf"]
        and result["single_joint_sweep_cf"]
        and result["multi_joint_sobol_cf"]
    )
    if bool(result["strict_collision_pass"]) is not recomputed_strict:
        raise ValueError("strict_collision_pass inconsistent")
    core = load_core()
    if not core._result_counters_valid(result, result.get("state_records", []), item):
        raise ValueError("state counters invalid")
    if result.get("state_records_sha256") != canonical_sha256(result.get("state_records", [])):
        raise ValueError("state_records_sha256 mismatch")


def terminate_process_group(pgid: int, process: subprocess.Popen[bytes]) -> str:
    if process.poll() is None:
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return "already_exited"
        try:
            process.communicate(timeout=2.0)
            return "sigterm"
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.communicate()
            return "sigkill_after_grace"
    return "none_required"


def execute_jobs(
    items: list[dict[str, Any]],
    *,
    worker_scratch: Path,
    timeout_seconds: float,
    max_workers: int,
) -> list[dict[str, Any]]:
    if timeout_seconds <= 0 or max_workers <= 0:
        raise ValueError("timeout_seconds and max_workers must be positive")
    worker_scratch.mkdir(parents=False)
    worker_scratch = worker_scratch.resolve(strict=True)
    pending = list(enumerate(items))
    active: dict[int, tuple[int, dict[str, Any], subprocess.Popen[bytes], Path, Path, float]] = {}
    results: dict[int, dict[str, Any]] = {}
    try:
        while pending or active:
            while pending and len(active) < max_workers:
                index, item = pending.pop(0)
                job_root = Path(tempfile.mkdtemp(prefix=f"job_{index:06d}_", dir=worker_scratch))
                job_path = job_root / "job.json"
                result_path = job_root / "result.json"
                atomic_json(job_path, item)
                environment = dict(os.environ)
                environment.update(
                    {
                        "OMP_NUM_THREADS": "1",
                        "OPENBLAS_NUM_THREADS": "1",
                        "MKL_NUM_THREADS": "1",
                        "NUMEXPR_NUM_THREADS": "1",
                    }
                )
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(SCRIPT),
                        "--internal-job",
                        str(job_path),
                        "--internal-result",
                        str(result_path),
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    env=environment,
                    start_new_session=True,
                )
                active[process.pid] = (index, item, process, job_root, result_path, time.monotonic())

            emitted = False
            for pid, state in list(active.items()):
                index, item, process, job_root, result_path, started = state
                returncode = process.poll()
                timed_out = returncode is None and time.monotonic() - started >= timeout_seconds
                if returncode is None and not timed_out:
                    continue
                if timed_out:
                    terminate_process_group(pid, process)
                    record = fail_closed(item, f"asset_timeout_after_{timeout_seconds:g}s", timed_out=True)
                else:
                    _stdout, stderr = process.communicate()
                    if returncode != 0 or not result_path.is_file():
                        detail = stderr.decode("utf-8", errors="replace")[-4000:]
                        record = fail_closed(item, f"worker_exit_{returncode}: {detail}", timed_out=False)
                    else:
                        try:
                            record = json.loads(result_path.read_text(encoding="utf-8"))
                            validate_result(item, record)
                        except Exception as exc:  # noqa: BLE001
                            record = fail_closed(
                                item, f"worker_result_invalid: {type(exc).__name__}: {exc}", timed_out=False
                            )
                record["worker_evidence"] = {
                    "pid": pid,
                    "returncode": process.returncode,
                    "timed_out": timed_out,
                    "fresh_interpreter": True,
                    "start_new_session": True,
                }
                del active[pid]
                shutil.rmtree(job_root, ignore_errors=True)
                results[index] = record
                emitted = True
            if active and not emitted:
                time.sleep(0.005)
    finally:
        for (_index, _item, process, job_root, _result_path, _started) in active.values():
            terminate_process_group(process.pid, process)
            shutil.rmtree(job_root, ignore_errors=True)
        if worker_scratch.is_dir() and not any(worker_scratch.iterdir()):
            worker_scratch.rmdir()
    return [results[index] for index in range(len(items))]


def run_internal_job(job_path: Path, result_path: Path) -> int:
    item = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    if not isinstance(item, dict) or "dataset_id" not in item:
        raise RuntimeError("internal job schema is invalid")
    if item.get("internal_test_action") == "sleep":
        time.sleep(float(item["sleep"]))
        atomic_json(result_path, {"dataset_id": item["dataset_id"], "status": "completed"})
        return 0

    core = load_core()
    package = Path(item["package"])
    urdf_path = package / str(item["urdf_relative_path"])
    try:
        binding_before = package_binding(package)
    except Exception as exc:  # noqa: BLE001
        record = fail_closed(item, f"source_precheck_failed: {type(exc).__name__}: {exc}", timed_out=False)
        record["result_origin"] = "child_attested"
        atomic_json(result_path, record)
        return 0
    if binding_before != item["package_binding"] or not urdf_path.is_file() or urdf_path.is_symlink():
        record = fail_closed(item, "package changed before evaluation", timed_out=False)
        record["result_origin"] = "child_attested"
        atomic_json(result_path, record)
        return 0
    if sha256_file(urdf_path) != item["urdf_sha256"]:
        record = fail_closed(item, "URDF changed before evaluation", timed_out=False)
        record["result_origin"] = "child_attested"
        atomic_json(result_path, record)
        return 0

    record = evaluate_physx_asset(item, core)

    try:
        changed = (
            package_binding(package) != item["package_binding"]
            or sha256_file(urdf_path) != item["urdf_sha256"]
        )
    except Exception:  # noqa: BLE001
        changed = True
    if changed:
        record = fail_closed(item, "package changed during evaluation", timed_out=False)
    record["result_origin"] = "child_attested"
    atomic_json(result_path, record)
    return 0


def freeze_protocol_snapshot(output: Path) -> dict[str, str]:
    source_bytes = PROTOCOL_DOCUMENT.read_bytes()
    source_sha256 = hashlib.sha256(source_bytes).hexdigest()
    snapshot = output / "protocol_snapshot.md"
    if snapshot.exists() or snapshot.is_symlink():
        raise RuntimeError("protocol snapshot already exists")
    atomic_text(snapshot, source_bytes.decode("utf-8"))
    if hashlib.sha256(snapshot.read_bytes()).hexdigest() != source_sha256:
        raise RuntimeError("protocol snapshot write verification failed")
    return {
        "protocol_source_path": str(PROTOCOL_DOCUMENT.resolve()),
        "protocol_source_sha256_at_freeze": source_sha256,
        "protocol_path": str(snapshot.resolve()),
        "protocol_sha256": source_sha256,
    }


def _format_metric(row: dict[str, Any]) -> str:
    percentage = "N/A" if row["rate"] is None else f"{100 * row['rate']:.3f}%"
    return f"{row['passed']} / {row['denominator']} ({percentage})"


def report_text(summary: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    max_pen = metrics["max_penetration"]
    lines = [
        "# PhysX-Mobility Table 4 collision audit",
        "",
        f"Run classification: **{summary.get('classification', 'FORMAL')}**.",
        "",
        (
            f"Frozen cohort: N={summary['cohort']['selected']}, exact frozen Table 5 receipt-set "
            "order; no resampling/reselection."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| rest_all_pair_cf | {_format_metric(metrics['rest_all_pair_cf'])} |",
        f"| rest_non_adjacent_cf | {_format_metric(metrics['rest_non_adjacent_cf'])} |",
        f"| single_joint_sweep_cf | {_format_metric(metrics['single_joint_sweep_cf'])} |",
        f"| multi_joint_sobol_cf | {_format_metric(metrics['multi_joint_sobol_cf'])} |",
        (
            f"| collision_state_rate | {metrics['collision_state_rate']['collision_states']} / "
            f"{metrics['collision_state_rate']['denominator']} "
            f"({100 * metrics['collision_state_rate']['rate']:.3f}%) |"
        ),
        "| aor | N/E |",
        (
            f"| max_penetration | {max_pen['maximum_observed_normalized']} "
            f"({max_pen['fully_measured_assets']} / {max_pen['denominator']} measured; {max_pen['status']}) |"
        ),
        (
            f"| collision_free_range | {metrics['collision_free_range']['passed_states']} / "
            f"{metrics['collision_free_range']['denominator']} "
            f"({100 * metrics['collision_free_range']['rate']:.3f}%) |"
            if metrics["collision_free_range"]["rate"] is not None
            else "| collision_free_range | N/A |"
        ),
        f"| strict_collision_pass | {_format_metric(metrics['strict_collision_pass'])} |",
        "",
        (
            "Claim boundary: official PhysX-Mobility URDFs declare zero collision elements; all "
            "PyBullet contact queries over this cohort are empty, so collision-free outcomes are "
            "vacuous. The frozen Table 5 receipt set adjudicates strict_collision as N/E for this "
            "method (reason: official_urdf_zero_collision_elements)."
        ),
        "",
    ]
    return "\n".join(lines)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    receipt_dir = args.receipt_set.resolve(strict=True)
    started_at = utc_now()
    core = load_core()

    cohort = TABLE1P.load_formal_cohort(
        receipt_dir,
        dataset_root,
        expected_n=args.expected_n,
        verify_archive=not args.skip_archive_binding,
    )
    manifest5 = cohort["manifest"]
    rows = manifest5["rows"][: args.limit] if args.limit is not None else manifest5["rows"]

    pair_policy = core.run_pair_policy_smoke(output)

    staging_root = Path(
        tempfile.mkdtemp(prefix="table4_physx_mobility_staging.", dir=args.staging_parent)
    )
    try:
        items: list[dict[str, Any]] = []
        for order, row in enumerate(rows):
            staged = stage_package(dataset_root, row, staging_root)
            items.append(build_item(row, staged, order))

        protocol_binding = freeze_protocol_snapshot(output)
        runtime_identity = core.current_runtime_identity()
        run_manifest = {
            "schema_version": "1.0.0-physx-mobility-table4",
            "created_at": started_at,
            "protocol_id": PROTOCOL_ID,
            "dataset": "PhysX-Mobility",
            "mode": "formal" if args.limit is None else "smoke",
            "classification": "FORMAL" if args.limit is None else "SMOKE",
            "sample_size": len(rows),
            "source": {
                "dataset_root": str(dataset_root),
                "receipt_set": str(receipt_dir),
                "receipt_set_marker_sha256": sha256_file(receipt_dir / "receipt_set.json"),
                "protocol_file_sha256": sha256_file(receipt_dir / "protocol.json"),
                "manifest_file_sha256": sha256_file(receipt_dir / "manifest.json"),
                "manifest_cohort_sha256": manifest5["cohort_sha256"],
                "manifest_protocol_sha256": manifest5["protocol_sha256"],
                "selection": manifest5["selection"],
                "source_receipt": manifest5["source_receipt"],
                "archive_binding_verified": not args.skip_archive_binding,
            },
            "evaluation": {
                "command": [str(SCRIPT), *sys.argv[1:]],
                "collision_core": str(CORE_SCRIPT.resolve()),
                "collision_core_sha256": sha256_file(CORE_SCRIPT),
                "runner_path": str(SCRIPT),
                "runner_sha256": sha256_file(SCRIPT),
                "table1_cohort_loader_path": str(TABLE1_RUNNER_PATH.resolve()),
                "table1_cohort_loader_sha256": sha256_file(TABLE1_RUNNER_PATH),
                "table5_common_path": str(Path(TABLE1P.TABLE5.__file__).resolve()),
                "table5_common_sha256": sha256_file(Path(TABLE1P.TABLE5.__file__)),
                "workers": args.workers,
                "asset_timeout_seconds": args.asset_timeout_seconds,
                "single_samples": core.SINGLE_SAMPLES,
                "sobol_samples": core.SOBOL_SAMPLES,
                "sobol_seed": core.SOBOL_SEED,
                "penetration_threshold_m": core.PENETRATION_THRESHOLD_M,
                "reset_tolerance": core.RESET_TOLERANCE,
                "scale_protocol": SCALE_PROTOCOL,
                "scale_source": (
                    "per-asset release OBJ-vertex union bounding-box diagonal bound in the frozen "
                    "Table 5 manifest (bounding_box_diagonal)"
                ),
                "pair_policy_smoke": pair_policy,
                "runtime_identity": runtime_identity,
                "staging_policy": (
                    "per-asset self-contained package replicating release geometry; byte-for-byte "
                    "hash-verified copies bound to the frozen manifest rows and official archive"
                ),
                "python_executable": sys.executable,
                **protocol_binding,
            },
            "claim_boundary": {
                "official_urdf_zero_collision_elements": True,
                "table5_receipt_strict_collision": "N/E",
                "table5_receipt_strict_collision_reason": "official_urdf_zero_collision_elements",
                "vacuous_collision_free_outcomes": (
                    "all PyBullet contact queries over this cohort are empty; collision-free "
                    "outcomes mean no collision geometry exists to collide, not verified clearance"
                ),
                "continuous_collision_detection": "not_run",
                "semantic_joint_correctness": "not_evaluated",
                "physical_dynamics_validity": "not_evaluated",
            },
            "items": items,
        }
        run_manifest["manifest_content_sha256"] = canonical_sha256(
            {key: value for key, value in run_manifest.items() if key != "manifest_content_sha256"}
        )
        atomic_json(output / "manifest.json", run_manifest)

        records = execute_jobs(
            items,
            worker_scratch=output / ".worker_scratch",
            timeout_seconds=args.asset_timeout_seconds,
            max_workers=args.workers,
        )
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)

    with open(output / "asset_records.json", "w", encoding="utf-8") as handle:
        json.dump(records, handle, sort_keys=True, indent=1)
        handle.write("\n")
    with open(output / "state_records.jsonl", "w", encoding="utf-8") as handle:
        for record in records:
            for state in record.get("state_records", []):
                handle.write(json.dumps(state, sort_keys=True) + "\n")

    summary = core.summarize_records({"sample_size": len(records), "protocol_id": PROTOCOL_ID}, records)
    summary["metrics"]["max_penetration"]["normalization"] = (
        "frozen Table 5 manifest release OBJ-vertex union bounding-box diagonal"
    )
    summary["classification"] = run_manifest["classification"]
    summary["created_at"] = started_at
    summary["completed_at"] = utc_now()
    summary["cohort"] = {
        **summary["cohort"],
        "label": "PhysX-Mobility N=800 frozen Table 5 receipt-set cohort",
        "N_release": cohort["release_asset_count"],
        "N_eval": len(rows),
        "release_raw_categories": cohort["release_category_count"],
        "eval_raw_categories": len({str(row["category"]) for row in rows}),
        "cohort_type": TABLE1P.COHORT_TYPE,
        "manifest_cohort_sha256": manifest5["cohort_sha256"],
    }
    summary["claim_boundary"] = run_manifest["claim_boundary"]
    summary["scale_protocol"] = SCALE_PROTOCOL
    summary["declared_collision_element_total"] = sum(
        int(item["declared_collision_elements"]) for item in items
    )
    atomic_json(output / "summary.json", summary)
    atomic_text(output / "report.md", report_text(summary))

    artifact_files = (
        "manifest.json",
        "asset_records.json",
        "state_records.jsonl",
        "summary.json",
        "report.md",
        "protocol_snapshot.md",
        "pair_policy_smoke.json",
        "pair_policy_smoke.urdf",
    )
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "files": {
            name: {"bytes": (output / name).stat().st_size, "sha256": sha256_file(output / name)}
            for name in artifact_files
        },
    }
    atomic_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the PhysX-Mobility dataset root")

    with TABLE1P.SHARED._output_lock(output):
        staging = output.parent / (f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}")
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            TABLE1P.SHARED._verify_staged_artifacts(staging)
            TABLE1P.SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--receipt-set", type=Path, default=DEFAULT_RECEIPT_SET)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--expected-n", type=int, default=800)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--skip-archive-binding", action="store_true")
    parser.add_argument("--staging-parent", type=Path, default=None)
    return parser.parse_args(argv)


def main() -> int:
    effective_argv = list(sys.argv[1:])
    if effective_argv and effective_argv[0] == "--internal-job":
        if len(effective_argv) != 4 or effective_argv[2] != "--internal-result":
            print("ERROR: invalid internal job invocation", file=sys.stderr)
            return 2
        try:
            return run_internal_job(Path(effective_argv[1]), Path(effective_argv[3]))
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR: internal job failed: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1

    args = parse_args(effective_argv)
    if args.expected_n <= 0 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise SystemExit("--expected-n, --workers and --asset-timeout-seconds must be positive")
    if args.limit is not None and args.limit <= 0:
        raise SystemExit("--limit must be positive")
    if args.output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n800"
        args.output = REPO / f"exp/runtime/urdf_table4_physx_mobility_table5cohort_{suffix}_{stamp}"
    summary = run(args)
    metrics = summary["metrics"]
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "n_eval": summary["cohort"]["N_eval"],
                "strict_collision_pass": metrics["strict_collision_pass"]["passed"],
                "collision_state_rate_denominator": metrics["collision_state_rate"]["denominator"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
