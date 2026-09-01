#!/usr/bin/env python3
"""Evaluate Table 3 on the frozen PhysX-Mobility N=800 cohort.

The cohort is the frozen Table 5 receipt set
(``exp/runtime/table5_physx_mobility_n800_v2``). All seven Table 3
kinematic metrics are computed by the shared frozen FK core
(``run_urdf_table3_lam.py::evaluate_urdf``): K=21 uniform sweep states
per bounded joint (continuous joints use the frozen [-pi, pi]
interval), one joint driven at a time, pose errors normalized by the
q0 visual/collision geometry union-AABB diagonal, thresholds
1e-6 (motion) / 1e-9 (unchanged subtree, round-trip).

PhysX-Mobility release geometry places resources in ``partseg/`` as a
sibling of the flat ``urdf/`` directory. The frozen core derives the
kinematic scale from mesh references that must resolve strictly inside
the URDF directory, so each asset is staged into a self-contained
evaluation package: resources are copied byte-for-byte
(hash-verified against the frozen manifest row) and the URDF's mesh
``filename`` attributes are relativized into the evaluation package.
The rewrite touches nothing but mesh reference strings; joint/link
declarations are verified structurally invariant before freeze. FK
inputs (joint types, origins, axes, limits) and mesh vertex data are
therefore identical to the released asset.
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
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
DEFAULT_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
CORE_PATH = REPO / "exp/scripts/run_urdf_table3_lam.py"
TABLE1_RUNNER_PATH = REPO / "exp/scripts/run_table1_physx_mobility.py"
TABLE2_EVALUATOR_PATH = REPO / "exp/scripts/run_table2_urdf_articraft.py"
SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0


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


CORE = _load_module(CORE_PATH, "table3_kinematic_fk_core_shared")
TABLE1P = _load_module(TABLE1_RUNNER_PATH, "run_table3_physx_table1_cohort_shared")
TABLE2 = _load_module(TABLE2_EVALUATOR_PATH, "run_table3_physx_table2_snapshot_shared")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _canonical_element(element: ET.Element) -> bytes:
    return ET.tostring(element, encoding="utf-8")


def build_eval_package(
    dataset_root: Path,
    row: dict[str, Any],
    staging_root: Path,
) -> dict[str, Any]:
    """Stage one asset into a self-contained evaluation package.

    Resources are copied byte-for-byte (hash-verified against the frozen
    manifest row). The URDF is rewritten so that mesh ``filename``
    attributes resolve inside the evaluation package; joint and link
    declarations are verified structurally invariant.
    """
    dataset_root = dataset_root.resolve()
    dataset_id = int(row["dataset_id"])
    eval_pkg = staging_root / str(dataset_id)

    items = [(str(resource["relative_path"]), str(resource["sha256"])) for resource in row["resources"]]
    seen: set[str] = set()
    for relative, expected_sha256 in items:
        if relative in seen:
            raise ValueError(f"duplicate resource binding: {relative}")
        seen.add(relative)
        source = dataset_root / relative
        target = eval_pkg / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if CORE.sha256_file(target) != expected_sha256:
            raise ValueError(f"staged byte mismatch: {dataset_id}: {relative}")

    source_urdf = dataset_root / str(row["urdf_relative_path"])
    if CORE.sha256_file(source_urdf) != str(row["urdf_sha256"]):
        raise ValueError(f"source URDF drifted from frozen row: {dataset_id}")

    original_tree = ET.parse(source_urdf)
    original_root = original_tree.getroot()
    urdf_dir = source_urdf.parent
    reference_map: list[dict[str, str]] = []
    for mesh in original_root.iter("mesh"):
        raw = mesh.get("filename", "")
        resolved = (urdf_dir / raw).resolve(strict=True)
        try:
            relative = resolved.relative_to(dataset_root)
        except ValueError as exc:
            raise ValueError(f"mesh reference escapes dataset root: {raw}") from exc
        if not (eval_pkg / relative).is_file():
            raise ValueError(f"mesh resource not in frozen closure: {relative.as_posix()}")
        mesh.set("filename", relative.as_posix())
        reference_map.append({"original": raw, "relativized": relative.as_posix()})

    rewritten_urdf = eval_pkg / f"{dataset_id}.urdf"
    original_tree.write(rewritten_urdf, encoding="utf-8", xml_declaration=True)
    _verify_kinematic_invariance(source_urdf, rewritten_urdf, reference_map)

    return {
        "eval_package": eval_pkg,
        "eval_urdf": rewritten_urdf,
        "eval_urdf_sha256": CORE.sha256_file(rewritten_urdf),
        "eval_package_binding": _package_binding(eval_pkg),
        "reference_map": reference_map,
    }


def _strip_mesh_filenames(element: ET.Element) -> ET.Element:
    import copy

    stripped = copy.deepcopy(element)
    for mesh in stripped.iter("mesh"):
        mesh.attrib.pop("filename", None)
    return stripped


def _verify_kinematic_invariance(
    original_path: Path, rewritten_path: Path, reference_map: list[dict[str, str]]
) -> None:
    original = ET.parse(original_path).getroot()
    rewritten = ET.parse(rewritten_path).getroot()
    if original.tag != rewritten.tag or original.tag != "robot":
        raise ValueError("rewrite changed the root element")
    original_links = original.findall("link")
    rewritten_links = rewritten.findall("link")
    if [link.get("name") for link in original_links] != [link.get("name") for link in rewritten_links]:
        raise ValueError("rewrite changed link declarations")
    for left, right in zip(original_links, rewritten_links):
        if _canonical_element(_strip_mesh_filenames(left)) != _canonical_element(_strip_mesh_filenames(right)):
            raise ValueError(f"rewrite changed link content: {left.get('name')!r}")
    original_joints = original.findall("joint")
    rewritten_joints = rewritten.findall("joint")
    if len(original_joints) != len(rewritten_joints):
        raise ValueError("rewrite changed joint count")
    for left, right in zip(original_joints, rewritten_joints):
        if _canonical_element(left) != _canonical_element(right):
            raise ValueError(f"rewrite changed joint declaration: {left.get('name')!r}")
    original_meshes = list(original.iter("mesh"))
    rewritten_meshes = list(rewritten.iter("mesh"))
    if len(original_meshes) != len(rewritten_meshes) or len(original_meshes) != len(reference_map):
        raise ValueError("rewrite changed mesh reference count")
    for mesh, entry in zip(rewritten_meshes, reference_map):
        if mesh.get("filename") != entry["relativized"]:
            raise ValueError("rewrite produced unexpected mesh reference")


def _package_binding(package: Path) -> dict[str, Any]:
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
                    "sha256": CORE.sha256_file(path),
                }
            )
    rows.sort(key=lambda item: item["path"])
    return {
        "file_count": len(rows),
        "total_bytes": sum(item["bytes"] for item in rows),
        "content_manifest_sha256": CORE.canonical_sha256(rows),
    }


def parent_failure(core_record: dict[str, Any], job: dict[str, Any], *, status: str) -> dict[str, Any]:
    record = dict(core_record)
    record["status"] = status
    record["result_origin"] = "parent_synthesized"
    record["category"] = job["category"]
    for field in (
        "selection_index",
        "dataset_id",
        "rank",
        "rank_sha256",
        "urdf_sha256",
        "eval_urdf_sha256",
        "manifest_row_sha256",
        "manifest_content_sha256",
    ):
        record[field] = job[field]
    record["eval_package_content_manifest_sha256"] = job["eval_package_binding"][
        "content_manifest_sha256"
    ]
    return record


def fail_closed_record(job: dict[str, Any], reason: str, *, status: str) -> dict[str, Any]:
    return parent_failure(
        CORE.failed_record(job["asset_key"], job["declared_joint_count_hint"], reason, status=status),
        job,
        status=status,
    )


def validate_record(job: dict[str, Any], record: dict[str, Any]) -> None:
    if record.get("asset_key") != job["asset_key"]:
        raise ValueError("record asset key mismatch")
    if record.get("status") not in {"completed", "error", "timeout"}:
        raise ValueError(f"record status is invalid: {record.get('status')!r}")
    joints = record.get("joints")
    if not isinstance(joints, list) or len(joints) != int(record.get("declared_joint_count", -1)):
        raise ValueError("record joint denominator mismatch")
    recomputed = bool(joints) and all(bool(joint.get("joint_level_pass")) for joint in joints)
    if record.get("status") == "completed" and bool(record.get("tree_valid")):
        if bool(record.get("strict_kinematic_pass")) is not recomputed:
            raise ValueError("record strict pass is inconsistent")
    elif bool(record.get("strict_kinematic_pass")):
        raise ValueError("failed record claims strict pass")
    for field in (
        "urdf_sha256",
        "eval_urdf_sha256",
        "manifest_row_sha256",
        "manifest_content_sha256",
    ):
        if record.get(field) != job[field]:
            raise ValueError(f"record binding mismatch: {field}")
    if record.get("eval_package_content_manifest_sha256") != job["eval_package_binding"]["content_manifest_sha256"]:
        raise ValueError("record binding mismatch: eval_package_content_manifest_sha256")
    if record.get("result_origin") not in {"child_attested", "parent_synthesized"}:
        raise ValueError("record origin is invalid")


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
    jobs: list[dict[str, Any]],
    *,
    worker_scratch: Path,
    timeout_seconds: float,
    max_workers: int,
) -> list[dict[str, Any]]:
    if timeout_seconds <= 0 or max_workers <= 0:
        raise ValueError("timeout_seconds and max_workers must be positive")
    worker_scratch.mkdir(parents=False)
    worker_scratch = worker_scratch.resolve(strict=True)
    pending = list(enumerate(jobs))
    active: dict[int, tuple[int, dict[str, Any], subprocess.Popen[bytes], Path, Path, float]] = {}
    results: dict[int, dict[str, Any]] = {}
    try:
        while pending or active:
            while pending and len(active) < max_workers:
                index, job = pending.pop(0)
                job_root = Path(tempfile.mkdtemp(prefix=f"job_{index:06d}_", dir=worker_scratch))
                job_path = job_root / "job.json"
                result_path = job_root / "result.json"
                CORE.atomic_write_json(job_path, job)
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
                        str(SCRIPT_PATH),
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
                active[process.pid] = (index, job, process, job_root, result_path, time.monotonic())

            emitted = False
            for pid, state in list(active.items()):
                index, job, process, job_root, result_path, started = state
                returncode = process.poll()
                timed_out = returncode is None and time.monotonic() - started >= timeout_seconds
                if returncode is None and not timed_out:
                    continue
                if timed_out:
                    terminate_process_group(pid, process)
                    record = fail_closed_record(
                        job, f"asset_timeout_after_{timeout_seconds:g}s", status="timeout"
                    )
                else:
                    _stdout, stderr = process.communicate()
                    if returncode != 0 or not result_path.is_file():
                        detail = stderr.decode("utf-8", errors="replace")[-4000:]
                        record = fail_closed_record(
                            job, f"worker_exit_{returncode}: {detail}", status="error"
                        )
                    else:
                        try:
                            record = json.loads(result_path.read_text(encoding="utf-8"))
                            validate_record(job, record)
                        except Exception as exc:  # noqa: BLE001
                            record = fail_closed_record(
                                job,
                                f"worker_result_invalid: {type(exc).__name__}: {exc}",
                                status="error",
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
        for (_index, _job, process, job_root, _result_path, _started) in active.values():
            terminate_process_group(process.pid, process)
            shutil.rmtree(job_root, ignore_errors=True)
        if worker_scratch.is_dir() and not any(worker_scratch.iterdir()):
            worker_scratch.rmdir()
    return [results[index] for index in range(len(jobs))]


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    if not isinstance(job, dict) or not isinstance(job.get("asset_key"), str):
        raise RuntimeError("internal job schema is invalid")
    if job.get("internal_test_action") == "sleep":
        time.sleep(float(job["sleep"]))
        CORE.atomic_write_json(result_path, {"asset_key": job["asset_key"], "status": "completed"})
        return 0

    def finish(record: dict[str, Any]) -> dict[str, Any]:
        record["result_origin"] = "child_attested"
        record["category"] = job["category"]
        for field in (
            "selection_index",
            "dataset_id",
            "rank",
            "rank_sha256",
            "urdf_sha256",
            "eval_urdf_sha256",
            "eval_package_content_manifest_sha256",
            "manifest_row_sha256",
            "manifest_content_sha256",
        ):
            if field == "eval_package_content_manifest_sha256":
                record[field] = job["eval_package_binding"]["content_manifest_sha256"]
            else:
                record[field] = job[field]
        return record

    eval_pkg = Path(job["eval_package"])
    eval_urdf = Path(job["eval_urdf"])
    try:
        binding_before = _package_binding(eval_pkg)
    except Exception as exc:  # noqa: BLE001
        record = fail_closed_record(job, f"source_precheck_failed: {type(exc).__name__}: {exc}", status="error")
        CORE.atomic_write_json(result_path, finish(record))
        return 0
    if binding_before != job["eval_package_binding"] or not eval_urdf.is_file() or eval_urdf.is_symlink():
        record = fail_closed_record(job, "eval package changed before evaluation", status="error")
        CORE.atomic_write_json(result_path, finish(record))
        return 0
    if CORE.sha256_file(eval_urdf) != job["eval_urdf_sha256"]:
        record = fail_closed_record(job, "eval URDF changed before evaluation", status="error")
        CORE.atomic_write_json(result_path, finish(record))
        return 0

    record = CORE.evaluate_urdf(
        eval_urdf,
        job["asset_key"],
        samples=job["samples"],
        declared_joint_count_hint=job["declared_joint_count_hint"],
    )

    try:
        binding_after = _package_binding(eval_pkg)
        changed = (
            binding_after != job["eval_package_binding"]
            or CORE.sha256_file(eval_urdf) != job["eval_urdf_sha256"]
        )
    except Exception:  # noqa: BLE001
        changed = True
    if changed:
        record = fail_closed_record(job, "eval package changed during evaluation", status="error")
    CORE.atomic_write_json(result_path, finish(record))
    return 0


def build_jobs(
    rows: list[dict[str, Any]],
    staged: dict[int, dict[str, Any]],
    *,
    manifest_content_sha256: str,
    samples: int,
) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for row in rows:
        dataset_id = int(row["dataset_id"])
        entry = staged[dataset_id]
        jobs.append(
            {
                "asset_key": str(dataset_id),
                "dataset_id": dataset_id,
                "selection_index": int(row["rank"]) - 1,
                "rank": int(row["rank"]),
                "rank_sha256": str(row["rank_sha256"]),
                "category": str(row["category"]),
                "eval_package": str(entry["eval_package"]),
                "eval_urdf": str(entry["eval_urdf"]),
                "eval_urdf_sha256": entry["eval_urdf_sha256"],
                "eval_package_binding": entry["eval_package_binding"],
                "urdf_sha256": str(row["urdf_sha256"]),
                "manifest_row_sha256": TABLE1P.TABLE5.canonical_sha256(row),
                "manifest_content_sha256": manifest_content_sha256,
                "declared_joint_count_hint": int(row["xml_counts"]["movable_joints"]),
                "samples": samples,
            }
        )
    return jobs


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    roundtrip = metrics["fk_roundtrip_error"]
    lines = [
        "# PhysX-Mobility Table 3 kinematic audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Frozen cohort: N={summary['n_eval']}, J_eval={summary['j_eval']}, exact frozen "
            "Table 5 receipt-set order; no resampling/reselection."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for name in ("valid_range", "joint_sweep_success", "non_degenerate_motion", "subtree_consistency", "joint_level_pass"):
        metric = metrics[name]
        lines.append(
            f"| {name} | {metric['passed']} / {metric['denominator']} ({100 * metric['rate']:.2f}%) |"
        )
    roundtrip_cell = (
        f"{roundtrip['max_normalized_translation']:.6f} normalized translation / "
        f"{roundtrip['max_rotation_rad']:.6e} rad rotation "
        f"({roundtrip['measured_joint_count']} / {roundtrip['denominator']} measured; {roundtrip['status']})"
        if roundtrip["measured_joint_count"]
        else "N/E"
    )
    lines.append(f"| fk_roundtrip_error | {roundtrip_cell} |")
    strict = metrics["strict_kinematic_pass"]
    lines.append(
        f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} ({100 * strict['rate']:.2f}%) |"
    )
    lines.append("")
    return "\n".join(lines)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    receipt_dir = args.receipt_set.resolve(strict=True)
    started_at = utc_now()

    cohort = TABLE1P.load_formal_cohort(
        receipt_dir,
        dataset_root,
        expected_n=args.expected_n,
        verify_archive=not args.skip_archive_binding,
    )
    manifest5 = cohort["manifest"]
    rows = manifest5["rows"][: args.limit] if args.limit is not None else manifest5["rows"]

    staging_root = Path(
        tempfile.mkdtemp(prefix="table3_physx_mobility_staging.", dir=args.staging_parent)
    )
    try:
        staged: dict[int, dict[str, Any]] = {}
        for row in rows:
            entry = build_eval_package(dataset_root, row, staging_root)
            staged[int(row["dataset_id"])] = entry

        protocol_binding = TABLE2.freeze_protocol_snapshot(output)
        evaluation = {
            "command": [str(SCRIPT_PATH), *sys.argv[1:]],
            "core_path": str(CORE_PATH.resolve()),
            "core_sha256": CORE.sha256_file(CORE_PATH.resolve()),
            "runner_path": str(SCRIPT_PATH),
            "runner_sha256": CORE.sha256_file(SCRIPT_PATH),
            "table1_cohort_loader_path": str(TABLE1_RUNNER_PATH.resolve()),
            "table1_cohort_loader_sha256": CORE.sha256_file(TABLE1_RUNNER_PATH.resolve()),
            "table5_common_path": str(Path(TABLE1P.TABLE5.__file__).resolve()),
            "table5_common_sha256": CORE.sha256_file(Path(TABLE1P.TABLE5.__file__).resolve()),
            "samples": args.samples,
            "workers": args.workers,
            "asset_timeout_seconds": args.asset_timeout_seconds,
            "continuous_interval": list(CORE.CONTINUOUS_INTERVAL),
            "translation_motion_threshold": CORE.TRANSLATION_MOTION_THRESHOLD,
            "rotation_motion_threshold_rad": CORE.ROTATION_MOTION_THRESHOLD_RAD,
            "unchanged_translation_tolerance": CORE.UNCHANGED_TRANSLATION_TOLERANCE,
            "unchanged_rotation_tolerance_rad": CORE.UNCHANGED_ROTATION_TOLERANCE_RAD,
            "roundtrip_translation_tolerance": CORE.ROUNDTRIP_TRANSLATION_TOLERANCE,
            "roundtrip_rotation_tolerance_rad": CORE.ROUNDTRIP_ROTATION_TOLERANCE_RAD,
            "supported_movable_types": sorted(CORE.SUPPORTED_MOVABLE_TYPES),
            "denominator_policy": "all frozen selected assets and all declared non-fixed joints, including failures",
            "staging_policy": (
                "per-asset self-contained evaluation package: resources copied byte-for-byte with "
                "hash verification against the frozen manifest row; URDF mesh filename attributes "
                "relativized into the package; joint/link declarations verified structurally invariant"
            ),
            "python_executable": sys.executable,
            "python_version": sys.version,
            **protocol_binding,
        }

        assets = []
        for row in rows:
            dataset_id = int(row["dataset_id"])
            entry = staged[dataset_id]
            assets.append(
                {
                    "selection_index": int(row["rank"]) - 1,
                    "asset_id": str(dataset_id),
                    "dataset_id": dataset_id,
                    "rank": int(row["rank"]),
                    "rank_sha256": str(row["rank_sha256"]),
                    "category": str(row["category"]),
                    "urdf_relative_path": str(row["urdf_relative_path"]),
                    "urdf_sha256": str(row["urdf_sha256"]),
                    "finaljson_sha256": str(row["finaljson_sha256"]),
                    "resource_sha256": str(row["resource_sha256"]),
                    "manifest_row_sha256": TABLE1P.TABLE5.canonical_sha256(row),
                    "declared_joint_count_hint": int(row["xml_counts"]["movable_joints"]),
                    "eval_package": str(entry["eval_package"]),
                    "eval_urdf": str(entry["eval_urdf"]),
                    "eval_urdf_sha256": entry["eval_urdf_sha256"],
                    "eval_package_binding": entry["eval_package_binding"],
                    "reference_map": entry["reference_map"],
                }
            )

        run_manifest = {
            "schema_version": "1.0.0-physx-mobility-table3",
            "created_at": started_at,
            "dataset": "PhysX-Mobility",
            "mode": "formal" if args.limit is None else "smoke",
            "classification": "FORMAL" if args.limit is None else "SMOKE",
            "source": {
                "dataset_root": str(dataset_root),
                "receipt_set": str(receipt_dir),
                "receipt_set_marker_sha256": CORE.sha256_file(receipt_dir / "receipt_set.json"),
                "protocol_file_sha256": CORE.sha256_file(receipt_dir / "protocol.json"),
                "manifest_file_sha256": CORE.sha256_file(receipt_dir / "manifest.json"),
                "manifest_cohort_sha256": manifest5["cohort_sha256"],
                "manifest_protocol_sha256": manifest5["protocol_sha256"],
                "selection": manifest5["selection"],
                "source_receipt": manifest5["source_receipt"],
                "archive_binding_verified": not args.skip_archive_binding,
            },
            "evaluation": evaluation,
            "n_requested": args.expected_n,
            "n_eval": len(rows),
            "staging_root": str(staging_root),
            "assets": assets,
        }
        run_manifest["manifest_content_sha256"] = TABLE2.manifest_self_hash(run_manifest)
        CORE.atomic_write_json(output / "manifest.json", run_manifest)

        jobs = build_jobs(
            rows,
            staged,
            manifest_content_sha256=run_manifest["manifest_content_sha256"],
            samples=args.samples,
        )
        records = execute_jobs(
            jobs,
            worker_scratch=output / ".worker_scratch",
            timeout_seconds=args.asset_timeout_seconds,
            max_workers=args.workers,
        )
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)

    with open(output / "asset_records.jsonl", "w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True) + "\n")

    summary = CORE.aggregate_records(records, len(rows))
    summary["dataset"] = "PhysX-Mobility"
    summary["classification"] = run_manifest["classification"]
    summary["created_at"] = started_at
    summary["completed_at"] = utc_now()
    summary["cohort"] = {
        "N_release": cohort["release_asset_count"],
        "N_eval": len(rows),
        "release_raw_categories": cohort["release_category_count"],
        "eval_raw_categories": len({str(row["category"]) for row in rows}),
        "cohort_type": TABLE1P.COHORT_TYPE,
        "manifest_cohort_sha256": manifest5["cohort_sha256"],
    }
    summary["kinematic_scale_coverage"] = dict(
        sorted(Counter(str(record.get("kinematic_scale_status", "UNAVAILABLE")) for record in records).items())
    )
    CORE.atomic_write_json(output / "summary.json", summary)
    CORE.atomic_write_text(output / "summary.md", _summary_markdown(summary, run_manifest))

    artifact_files = (
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
        TABLE2.PROTOCOL_SNAPSHOT_NAME,
    )
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": CORE.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    CORE.atomic_write_json(output / "artifact_manifest.json", artifact_manifest)
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
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
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
    if args.expected_n <= 0 or args.workers <= 0 or args.samples < 2:
        raise SystemExit("--expected-n, --workers and --samples must be positive")
    if args.limit is not None and args.limit <= 0:
        raise SystemExit("--limit must be positive")
    if args.output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n800"
        args.output = REPO / f"exp/runtime/urdf_table3_physx_mobility_table5cohort_{suffix}_{stamp}"
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "n_eval": summary["n_eval"],
                "j_eval": summary["j_eval"],
                "strict_kinematic_pass": summary["metrics"]["strict_kinematic_pass"]["passed"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
