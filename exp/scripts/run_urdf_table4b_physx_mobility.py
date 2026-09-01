#!/usr/bin/env python3
"""Fail-closed Table 4b runner for PhysX-Mobility (exact collision-representation geometry).

Runs the proposed Table 4b Collision Representation Quality and Cost metrics
over the frozen Table 5 PhysX-Mobility N=800 receipt-set cohort, reusing the
frozen ``lam_supplementary_geometry_v1`` geometry atoms (exact surface backend
``trimesh.proximity.ProximityQuery.on_surface + rtree``, 32,768 area-weighted
samples per direction, weld relative tolerance 1e-9, D_visual = q0 loadable
visual union AABB diagonal) with parameters bit-identical to the LAM and
Artiverse Table 4b runs.

PhysX-Mobility release geometry places resources in ``partseg/`` as a sibling
of the flat ``urdf/`` directory, while the frozen geometry atoms resolve mesh
references inside the URDF package directory. Each asset is therefore staged
into a self-contained evaluation package: resources are copied byte-for-byte
(hash-verified against the frozen manifest row) and the URDF's mesh
``filename`` attributes are relativized into the package; joint/link
declarations are verified structurally invariant before freeze.

Claim boundary: official PhysX-Mobility URDFs declare zero collision
elements, so every collision-representation metric is N/E fail-closed;
D_visual derivation from loadable visual geometry still executes.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_geometry as geometry  # noqa: E402

SCHEMA_VERSION = "table4b-physx-mobility/v1"
PROTOCOL_ID = "table4b_physx_mobility_table5cohort_n800_v1"
GEOMETRY_PROTOCOL_ID = geometry.GEOMETRY_PROTOCOL_ID
DATASET = "PhysX-Mobility"
CLASSIFICATION = "FORMAL"

DEFAULT_DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
DEFAULT_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

N_EVAL = 800
CHILD_TIMEOUT_SECONDS = 900
WORKERS = 16
LAUNCH_STAGGER_SECONDS = 1.5
GEOMETRY_SAMPLES_PER_DIRECTION = 32768
GEOMETRY_WELD_REL_TOL = 1e-9

CHILD_PYTHON = Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python")
CPU_AFFINITY_WIDTH = 4
CPU_AFFINITY_ENV = "TABLE4B_PHYSX_CPU_AFFINITY"
THREAD_ENV_KEYS = (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)
THREAD_ENV_VALUES = {key: "1" for key in THREAD_ENV_KEYS}


def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n",
    )


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


TABLE1P = _load_module(
    REPO / "exp/scripts/run_table1_physx_mobility.py",
    "run_table4b_physx_table1_cohort_shared",
)


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


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
                    "sha256": sha256_bytes(path.read_bytes()),
                }
            )
    rows.sort(key=lambda item: item["path"])
    return {
        "file_count": len(rows),
        "total_bytes": sum(item["bytes"] for item in rows),
        "content_manifest_sha256": canonical_sha256(rows),
    }


def _strip_mesh_filenames(element: ET.Element) -> ET.Element:
    import copy

    stripped = copy.deepcopy(element)
    for mesh in stripped.iter("mesh"):
        mesh.attrib.pop("filename", None)
    return stripped


def _canonical_element(element: ET.Element) -> bytes:
    return ET.tostring(element, encoding="utf-8")


def _verify_kinematic_invariance(original_urdf: Path, rewritten_urdf: Path) -> None:
    original = ET.parse(original_urdf).getroot()
    rewritten = ET.parse(rewritten_urdf).getroot()
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
    if len(list(original.iter("mesh"))) != len(list(rewritten.iter("mesh"))):
        raise ValueError("rewrite changed mesh reference count")


def stage_package(dataset_root: Path, row: Mapping[str, Any], staging_root: Path) -> dict[str, Any]:
    """Stage one asset into a self-contained evaluation package.

    Resources are copied byte-for-byte (hash-verified against the frozen
    manifest row). The URDF is rewritten so that mesh ``filename`` attributes
    resolve inside the evaluation package; joint/link declarations are
    verified structurally invariant.
    """
    dataset_root = dataset_root.resolve()
    dataset_id = int(row["dataset_id"])
    package = staging_root / str(dataset_id)

    items = [(str(resource["relative_path"]), str(resource["sha256"])) for resource in row["resources"]]
    seen: set[str] = set()
    for relative, expected_sha256 in items:
        if relative in seen:
            raise ValueError(f"duplicate resource binding: {relative}")
        seen.add(relative)
        source = dataset_root / relative
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if sha256_bytes(target.read_bytes()) != expected_sha256:
            raise ValueError(f"staged byte mismatch: {dataset_id}: {relative}")

    source_urdf = dataset_root / str(row["urdf_relative_path"])
    if sha256_bytes(source_urdf.read_bytes()) != str(row["urdf_sha256"]):
        raise ValueError(f"source URDF drifted from frozen row: {dataset_id}")
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    reference_map = []
    for mesh in root.iter("mesh"):
        raw = mesh.get("filename", "")
        resolved = (source_urdf.parent / raw).resolve(strict=True)
        try:
            relative = resolved.relative_to(dataset_root)
        except ValueError as exc:
            raise ValueError(f"mesh reference escapes dataset root: {raw}") from exc
        if not (package / relative).is_file():
            raise ValueError(f"mesh resource not in frozen closure: {relative.as_posix()}")
        mesh.set("filename", relative.as_posix())
        reference_map.append({"original": raw, "relativized": relative.as_posix()})
    rewritten_urdf = package / f"{dataset_id}.urdf"
    tree.write(rewritten_urdf, encoding="utf-8", xml_declaration=True)
    _verify_kinematic_invariance(source_urdf, rewritten_urdf)
    return {
        "package": package,
        "rewritten_urdf": rewritten_urdf,
        "rewritten_urdf_sha256": sha256_bytes(rewritten_urdf.read_bytes()),
        "package_binding": package_binding(package),
        "reference_map": reference_map,
    }


def build_jobs(rows: Sequence[Mapping[str, Any]], staged: Mapping[int, Mapping[str, Any]]) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for order, row in enumerate(rows):
        dataset_id = int(row["dataset_id"])
        entry = staged[dataset_id]
        jobs.append(
            {
                "selection_index": order,
                "dataset_id": str(dataset_id),
                "asset_id": str(dataset_id),
                "rank": int(row["rank"]),
                "category": str(row["category"]),
                "package": str(entry["package"]),
                "package_binding": entry["package_binding"],
                "urdf_path": str(entry["rewritten_urdf"]),
                "expected_urdf_sha256": str(entry["rewritten_urdf_sha256"]),
                "original_urdf_sha256": str(row["urdf_sha256"]),
                "input_identity_sha256": TABLE1P.TABLE5.canonical_sha256(dict(row)),
            }
        )
    return jobs


def _failed_asset_record(job: Mapping[str, Any], issue: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "selection_index": int(job["selection_index"]),
        "dataset_id": str(job["dataset_id"]),
        "asset_id": str(job["asset_id"]),
        "category": str(job["category"]),
        "package": str(job["package"]),
        "urdf_sha256": None,
        "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
        "status": "error",
        "issues": [issue],
        "analytic_collision_element_count": 0,
        "loadable_collision_element_count": 0,
        "visual_bearing_link_count": 0,
        "visual_to_collision_p95_normalized": {"status": "N/E", "value": None, "reason": issue},
        "collision_to_visual_p95_normalized": {"status": "N/E", "value": None, "reason": issue},
        "shapes_per_visual_bearing_link": {"status": "N/E", "value": None, "reason": issue},
        "collision_mesh_triangles_per_asset": {"status": "N/E", "value": None, "reason": issue},
        "collision_load_time_seconds": {"status": "N/E", "value": None, "reason": issue},
        "intra_link_redundancy": {
            "status": "N/E", "value": None, "shape_volume_m3": None, "redundant_volume_m3": None,
            "intended_link_count": 0, "measured_link_count": 0, "reason": issue,
        },
        "intra_link_redundant_volume_m3": None,
        "intra_link_shape_volume_m3": None,
    }


def run_child(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.read_text(encoding="utf-8"))
    record: dict[str, Any]
    try:
        package = Path(str(job["package"]))
        urdf_path = Path(str(job["urdf_path"]))
        binding_before = package_binding(package)
        if binding_before != job["package_binding"]:
            raise RuntimeError("package changed before evaluation")
        urdf_sha256 = sha256_bytes(urdf_path.read_bytes())
        if urdf_sha256 != str(job["expected_urdf_sha256"]):
            raise RuntimeError(
                f"urdf_sha256_mismatch: got {urdf_sha256}, expected {job['expected_urdf_sha256']}"
            )
        asset_key = str(job["asset_id"])
        geometry_record = geometry.evaluate_table4b_geometry(
            urdf_path, asset_key, PROTOCOL_ID,
            GEOMETRY_SAMPLES_PER_DIRECTION, GEOMETRY_WELD_REL_TOL,
        )
        timing_record = geometry.measure_collision_load_time_in_asset_child(
            urdf_path, weld_rel_tol=GEOMETRY_WELD_REL_TOL
        )
        triangle_measurement = geometry.collision_triangle_validation_measurement(geometry_record)
        redundancy_measurement = geometry.collision_redundancy_measurement(geometry_record)

        visual_count = int(geometry_record.get("visual_bearing_link_count", 0))
        declared_collision = int(geometry_record.get("declared_collision_element_count", 0))
        loadable_collision = int(geometry_record.get("loadable_collision_element_count", 0))
        extraction_complete = (
            geometry_record.get("tree_valid") is True
            and not geometry_record.get("issues")
            and loadable_collision > 0
            and loadable_collision == declared_collision
            and visual_count > 0
        )
        shapes_measurement = (
            {"status": "COMPLETE", "value": loadable_collision / visual_count, "reason": None}
            if extraction_complete
            else {
                "status": "N/E",
                "value": None,
                "reason": "tree/resource/collision extraction or visual-bearing denominator is incomplete",
            }
        )

        def p95_measurement(direction_record: Mapping[str, Any]) -> dict[str, Any]:
            if direction_record.get("status") == "COMPLETE":
                value = direction_record.get("normalized_p95")
                if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)):
                    return {"status": "COMPLETE", "value": float(value), "reason": None}
            return {
                "status": "N/E",
                "value": None,
                "reason": str(direction_record.get("reason") or "exact surface measurement unavailable"),
            }

        timing_measurement = (
            {"status": "COMPLETE", "value": float(timing_record["value"]), "reason": None}
            if timing_record.get("status") == "COMPLETE"
            else {"status": "N/E", "value": None, "reason": str(timing_record.get("reason") or "timing unavailable")}
        )

        try:
            binding_after = package_binding(package)
        except Exception:  # noqa: BLE001
            binding_after = None
        if binding_after != job["package_binding"]:
            raise RuntimeError("package changed during evaluation")

        record = {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
            "selection_index": int(job["selection_index"]),
            "dataset_id": str(job["dataset_id"]),
            "asset_id": str(job["asset_id"]),
            "category": str(job["category"]),
            "package": str(job["package"]),
            "urdf_sha256": urdf_sha256,
            "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
            "original_urdf_sha256": str(job["original_urdf_sha256"]),
            "status": "completed",
            "issues": list(geometry_record.get("issues", [])),
            "geometry_status": geometry_record.get("status"),
            "tree_valid": bool(geometry_record.get("tree_valid")),
            "declared_link_count": int(geometry_record.get("declared_link_count", 0)),
            "declared_visual_element_count": int(geometry_record.get("declared_visual_element_count", 0)),
            "loadable_visual_element_count": int(geometry_record.get("loadable_visual_element_count", 0)),
            "declared_collision_element_count": declared_collision,
            "loadable_collision_element_count": loadable_collision,
            "visual_bearing_link_count": visual_count,
            "d_visual": geometry_record.get("d_visual"),
            "d_visual_status": geometry_record.get("d_visual_status"),
            "analytic_collision_element_count": int(geometry_record.get("analytic_collision_element_count", 0)),
            "collision_mesh_element_count": int(geometry_record.get("collision_mesh_element_count", 0)),
            "collision_mesh_valid_triangle_count": int(geometry_record.get("collision_mesh_valid_triangle_count", 0)),
            "collision_watertight_mesh_count": int(geometry_record.get("collision_watertight_mesh_count", 0)),
            "visual_to_collision_p95_normalized": p95_measurement(geometry_record.get("visual_to_collision", {})),
            "collision_to_visual_p95_normalized": p95_measurement(geometry_record.get("collision_to_visual", {})),
            "shapes_per_visual_bearing_link": shapes_measurement,
            "collision_mesh_triangles_per_asset": {
                "status": triangle_measurement["status"],
                "value": triangle_measurement.get("value"),
                "reason": triangle_measurement.get("reason"),
                "intended_mesh_count": triangle_measurement.get("intended_mesh_count"),
                "measured_mesh_count": triangle_measurement.get("measured_mesh_count"),
            },
            "collision_load_time_seconds": timing_measurement | {
                "backend": "trimesh.Trimesh(process=False).triangles_tree+rtree",
                "cache_mode": "same_asset_one_unmeasured_warmup_then_warm_cache",
                "completed_repeats": timing_record.get("completed_repeats", 0),
                "raw_times_seconds": timing_record.get("raw_times_seconds", []),
            },
            "intra_link_redundancy": redundancy_measurement,
            "intra_link_redundant_volume_m3": redundancy_measurement.get("redundant_volume_m3"),
            "intra_link_shape_volume_m3": redundancy_measurement.get("shape_volume_m3"),
            "geometry_record": geometry_record,
        }
    except Exception as exc:  # noqa: BLE001
        record = _failed_asset_record(job, issue=f"child_failed: {type(exc).__name__}: {exc}")
    record["child"] = {
        "finished_at_utc": utc_now_iso(),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
    }
    atomic_write_json(result_path, record)
    return 0


def spawn_children(
    jobs: Sequence[Mapping[str, Any]],
    outdir: Path,
    *,
    workers: int,
    timeout_seconds: float,
) -> list[dict[str, Any]]:
    child_python = CHILD_PYTHON.resolve(strict=True)
    job_dir = outdir / "jobs"
    child_dir = outdir / "children"
    log_dir = outdir / "child_logs"
    for directory in (job_dir, child_dir, log_dir):
        directory.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    pending: dict[int, dict[str, Any]] = {}
    total = len(jobs)
    next_index = 0
    cpu_count = os.cpu_count() or 1

    def launch(index: int) -> None:
        job = dict(jobs[index])
        width = max(1, min(CPU_AFFINITY_WIDTH, cpu_count))
        base = (index * width) % cpu_count
        affinity = ",".join(str((base + offset) % cpu_count) for offset in range(width))
        job_path = job_dir / f"rank_{index + 1:04d}.json"
        result_path = child_dir / f"rank_{index + 1:04d}.json"
        atomic_write_json(job_path, job)
        env = os.environ.copy()
        for key, value in THREAD_ENV_VALUES.items():
            env[key] = value
        env[CPU_AFFINITY_ENV] = affinity
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        log_path = log_dir / f"rank_{index + 1:04d}.log"
        log_handle = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            [str(child_python), str(SCRIPT), "--child", "--job", str(job_path), "--result", str(result_path)],
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            cwd=str(REPO),
        )
        pending[index] = {
            "proc": proc,
            "result_path": result_path,
            "log_handle": log_handle,
            "deadline": time.time() + timeout_seconds,
        }

    def finalize(index: int, entry: dict[str, Any], reason: str | None) -> None:
        try:
            entry.get("log_handle").close()
        except Exception:  # noqa: BLE001
            pass
        result_path: Path = entry["result_path"]
        record = None
        if result_path.is_file():
            try:
                record = json.loads(result_path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                record = None
        if record is None:
            record = _failed_asset_record(jobs[index], issue=reason or "child_result_missing")
            atomic_write_json(result_path, record)
        results.append(record)

    def kill_entry(entry: dict[str, Any]) -> None:
        proc = entry["proc"]
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
            pass
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                pass

    while len(results) < total:
        launched_now = False
        while len(pending) < workers and next_index < total:
            launch(next_index)
            next_index += 1
            launched_now = True
        if launched_now:
            time.sleep(LAUNCH_STAGGER_SECONDS)
        time.sleep(0.1)
        for index in sorted(pending):
            entry = pending[index]
            proc = entry["proc"]
            rc = proc.poll()
            if rc is not None:
                del pending[index]
                finalize(index, entry, None if rc == 0 else f"child_process_failed: rc={rc}")
            elif time.time() >= entry["deadline"]:
                kill_entry(entry)
                del pending[index]
                finalize(index, entry, f"asset_timeout after {timeout_seconds}s")
    results.sort(key=lambda r: int(r["selection_index"]))
    return results


def percentile_linear(values: Sequence[float], q: float) -> float:
    if not values:
        raise ValueError("empty values")
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = q * (len(ordered) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return float(ordered[lo])
    fraction = position - lo
    return float(ordered[lo] + (ordered[hi] - ordered[lo]) * fraction)


def _measurement(record: Mapping[str, Any], field: str) -> tuple[str, float | None]:
    raw = record[field]
    status = raw.get("status")
    value = raw.get("value")
    if status == "COMPLETE":
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
            raise ValueError(f"measurement {field} has an invalid COMPLETE value")
        return status, float(value)
    if value is not None:
        raise ValueError(f"measurement {field} must be null when status is {status}")
    return status, None


def aggregate(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    analytic = 0
    loadable = 0
    shapes_ratio: list[float] = []
    triangles: list[float] = []
    load_times: list[float] = []
    visual_to_collision: list[float] = []
    collision_to_visual: list[float] = []
    redundant_volume = 0.0
    shape_volume = 0.0
    redundancy_measured_links = 0
    redundancy_intended_links = 0
    redundancy_measured_assets = 0
    redundancy_complete_assets = 0
    d_visual_complete = 0
    for record in records:
        analytic += int(record.get("analytic_collision_element_count", 0))
        loadable += int(record.get("loadable_collision_element_count", 0))
        if record.get("d_visual_status") == "COMPLETE" and record.get("d_visual") is not None:
            d_visual_complete += 1
        for field, target in (
            ("visual_to_collision_p95_normalized", visual_to_collision),
            ("collision_to_visual_p95_normalized", collision_to_visual),
            ("collision_load_time_seconds", load_times),
            ("shapes_per_visual_bearing_link", shapes_ratio),
            ("collision_mesh_triangles_per_asset", triangles),
        ):
            status, value = _measurement(record, field)
            if status == "COMPLETE" and value is not None:
                target.append(value)
        redundancy = record["intra_link_redundancy"]
        r_status = redundancy.get("status")
        measured_links = int(redundancy.get("measured_link_count", 0))
        intended_links = int(redundancy.get("intended_link_count", 0))
        redundancy_measured_links += measured_links
        redundancy_intended_links += intended_links
        if r_status == "COMPLETE":
            redundancy_complete_assets += 1
        redundant = redundancy.get("redundant_volume_m3")
        volume = redundancy.get("shape_volume_m3")
        if r_status in {"COMPLETE", "PARTIAL"} and redundant is not None and volume is not None:
            redundant_volume += float(redundant)
            shape_volume += float(volume)
            redundancy_measured_assets += 1

    if redundancy_measured_assets == 0:
        redundancy_status = "N/E"
    elif redundancy_complete_assets == len(records) and redundancy_measured_links == redundancy_intended_links:
        redundancy_status = "COMPLETE"
    else:
        redundancy_status = "PARTIAL"

    def summary(values: list[float], intended: int, reason: str) -> dict[str, Any]:
        measured = len(values)
        status = "COMPLETE" if measured == intended else ("PARTIAL" if measured > 0 else "N/E")
        result: dict[str, Any] = {
            "status": status,
            "measured": measured,
            "intended": intended,
            "coverage": None if intended == 0 else measured / intended,
            "median": percentile_linear(values, 0.5) if values else None,
            "p90": percentile_linear(values, 0.9) if values else None,
        }
        if status == "N/E":
            result["reason"] = reason
        return result

    n = len(records)
    v2c = summary(visual_to_collision, n, "no exact visual-to-collision surface measurements")
    c2v = summary(collision_to_visual, n, "no exact collision-to-visual surface measurements")
    v2c["value"] = v2c["median"]
    c2v["value"] = c2v["median"]
    return {
        "status_counts": {
            "completed": sum(1 for r in records if r.get("status") == "completed"),
            "error": sum(1 for r in records if r.get("status") != "completed"),
            "total": n,
        },
        "analytic_collision_share": {
            "analytic_elements": analytic,
            "loadable_collision_elements": loadable,
            "rate": None if loadable == 0 else analytic / loadable,
        },
        "visual_to_collision_p95_normalized": v2c,
        "collision_to_visual_p95_normalized": c2v,
        "shapes_per_visual_bearing_link": summary(shapes_ratio, n, "no asset had a visual-bearing link")
        | ({"mean": sum(shapes_ratio) / len(shapes_ratio)} if shapes_ratio else {"mean": None}),
        "collision_mesh_triangles_per_asset": summary(triangles, n, "no collision mesh triangle measurements")
        | ({"mean": sum(triangles) / len(triangles)} if triangles else {"mean": None}),
        "intra_link_redundancy": {
            "status": redundancy_status,
            "measured": redundancy_measured_links,
            "intended": redundancy_intended_links,
            "link_coverage": None if redundancy_intended_links == 0 else redundancy_measured_links / redundancy_intended_links,
            "complete_assets": redundancy_complete_assets,
            "measured_assets": redundancy_measured_assets,
            "intended_assets": n,
            "asset_coverage": None if n == 0 else redundancy_measured_assets / n,
            "redundant_volume_m3": redundant_volume if shape_volume > 0.0 else None,
            "shape_volume_m3": shape_volume if shape_volume > 0.0 else None,
            "value": redundant_volume / shape_volume if shape_volume > 0.0 else None,
            **({"reason": "no measurable finite collision volume"} if redundancy_status == "N/E" else {}),
        },
        "collision_load_time_seconds": summary(load_times, n, "no collision-load timing measurements")
        | ({"mean": sum(load_times) / len(load_times)} if load_times else {"mean": None}),
        "d_visual_coverage": {"complete": d_visual_complete, "intended": n},
    }


def verify_run(
    rows: Sequence[Mapping[str, Any]],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    check("record_count", len(records) == N_EVAL, f"{len(records)}")
    order_ok = all(
        int(r["selection_index"]) == i and str(r["dataset_id"]) == str(rows[i]["dataset_id"])
        for i, r in enumerate(records)
    )
    check("frozen_order_preserved", order_ok)
    zero_collision = all(
        int(r.get("declared_collision_element_count", 0)) == 0
        and int(r.get("loadable_collision_element_count", 0)) == 0
        for r in records
    )
    check("claim_boundary_zero_collision_elements", zero_collision)
    d_visual = aggregates["d_visual_coverage"]
    check(
        "d_visual_derivation_coverage_accounted",
        d_visual["intended"] == len(records)
        and 0 <= d_visual["complete"] <= d_visual["intended"],
        f"{d_visual['complete']}/{d_visual['intended']}",
    )
    recomputed = aggregate(records)
    check(
        "aggregate_recomputation_matches",
        json.dumps(recomputed, sort_keys=True) == json.dumps(dict(aggregates), sort_keys=True),
    )
    all_pass = all(c["pass"] for c in checks)
    return {"all_pass": all_pass, "check_count": len(checks), "checks": checks}


def render_summary_md(summary: Mapping[str, Any]) -> str:
    m = summary["metrics"]
    share = m["analytic_collision_share"]
    v2c = m["visual_to_collision_p95_normalized"]
    c2v = m["collision_to_visual_p95_normalized"]
    shapes = m["shapes_per_visual_bearing_link"]
    triangles = m["collision_mesh_triangles_per_asset"]
    redundancy = m["intra_link_redundancy"]
    load_time = m["collision_load_time_seconds"]

    def p95_cell(cell: Mapping[str, Any]) -> str:
        if cell["status"] == "N/E" or cell.get("value") is None:
            return f"N/E ({cell['measured']}/{cell['intended']})"
        return f"{cell['value']:.2e} ({cell['measured']}/{cell['intended']}, {cell['status']})"

    def triple_cell(cell: Mapping[str, Any]) -> str:
        if cell["status"] == "N/E" or cell.get("median") is None:
            return f"N/E ({cell['measured']}/{cell['intended']})"
        return (
            f"{cell['mean']} / {cell['median']} / {cell['p90']} "
            f"({cell['measured']}/{cell['intended']}, {cell['status']})"
        )

    share_cell = (
        f"{share['analytic_elements']} / {share['loadable_collision_elements']} (N/E)"
        if share["rate"] is None
        else f"{share['analytic_elements']} / {share['loadable_collision_elements']} ({100.0 * share['rate']:.2f}%)"
    )
    load_cell = (
        f"N/E ({load_time['measured']}/{load_time['intended']})"
        if load_time["status"] == "N/E" or load_time.get("median") is None
        else f"{load_time['median']:.4f} s / {load_time['p90']:.4f} s ({load_time['measured']}/{load_time['intended']}, {load_time['status']})"
    )
    lines = [
        "# Table 4b — PhysX-Mobility (frozen Table 5 receipt-set cohort, N=800; exact collision-representation geometry)",
        "",
        f"- Protocol ID: `{summary['protocol_id']}` (geometry `{summary['geometry_protocol_id']}`)",
        f"- Run directory: `{summary['run_directory']}`",
        f"- N_eval = {summary['cohort']['n_eval']}",
        f"- Status: completed = {summary['status_counts']['completed']}, error = {summary['status_counts']['error']}",
        "- Claim boundary: official PhysX-Mobility URDFs declare zero collision elements; all collision-representation metrics are N/E fail-closed; D_visual derivation from loadable visual geometry still executes.",
        "",
        "| Metric | Result |",
        "|---|---|",
        f"| Analytic Collision Share | {share_cell} |",
        f"| Visual→Collision P95 | {p95_cell(v2c)} |",
        f"| Collision→Visual P95 | {p95_cell(c2v)} |",
        f"| Shapes/Visual-bearing Link | {triple_cell(shapes)} |",
        f"| Collision Mesh Triangles/Asset | {triple_cell(triangles)} |",
        f"| Intra-link Redundancy | {'N/E' if redundancy['value'] is None else redundancy['value']} "
        f"(links {redundancy['measured']}/{redundancy['intended']}, assets {redundancy['measured_assets']}/{redundancy['intended_assets']}, {redundancy['status']}) |",
        f"| Collision Load Time/Asset | {load_cell} |",
        "",
    ]
    return "\n".join(lines) + "\n"


def run_scope(args: argparse.Namespace) -> int:
    mode = args.mode
    timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if mode == "formal":
        outdir = REPO / f"exp/runtime/table4b_urdf_physx_mobility_table5cohort_n800_{timestamp}"
    else:
        outdir = REPO / f"exp/runtime/table4b_urdf_physx_mobility_smoke_n{args.n}_{timestamp}"
    if outdir.exists():
        raise SystemExit(f"output directory already exists: {outdir}")
    outdir.mkdir(parents=True)

    dataset_root = (args.dataset_root or DEFAULT_DATASET_ROOT).resolve(strict=True)
    receipt_dir = (args.receipt_set or DEFAULT_RECEIPT_SET).resolve(strict=True)
    cohort = TABLE1P.load_formal_cohort(
        receipt_dir,
        dataset_root,
        expected_n=N_EVAL,
        verify_archive=not args.skip_archive_binding,
    )
    manifest5 = cohort["manifest"]
    rows = manifest5["rows"]

    staging_root = outdir / "staging"
    staging_root.mkdir(parents=True)
    staged: dict[int, dict[str, Any]] = {}
    for row in rows:
        staged[int(row["dataset_id"])] = stage_package(dataset_root, row, staging_root)

    jobs = build_jobs(rows, staged)
    if mode == "smoke":
        jobs = jobs[: args.n]

    protocol_snapshot_text = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(outdir / "protocol_snapshot.md", protocol_snapshot_text)

    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "frozen_at_utc": utc_now_iso(),
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "protocol_document_sha256": sha256_bytes(protocol_snapshot_text.encode("utf-8")),
        "cohort": {
            "receipt_set": str(receipt_dir),
            "receipt_manifest_sha256": sha256_bytes((receipt_dir / "manifest.json").read_bytes()),
            "manifest_cohort_sha256": manifest5["cohort_sha256"],
            "n_eval": len(jobs),
            "selection_policy": "all Table 5 frozen receipt-set rows in existing rank order; no resampling or result-based filtering",
        },
        "geometry_parameters": {
            "exact_surface_backend": "trimesh.proximity.ProximityQuery.on_surface + rtree (no Genesis SDF substitute)",
            "samples_per_direction": GEOMETRY_SAMPLES_PER_DIRECTION,
            "weld_relative_tolerance": GEOMETRY_WELD_REL_TOL,
            "d_visual_definition": "q0_loadable_visual_union_aabb_diagonal_v1",
            "collision_load_time_backend": "trimesh.Trimesh(process=False).triangles_tree+rtree",
            "collision_load_time_cache_mode": "same_asset_one_unmeasured_warmup_then_warm_cache",
            "collision_load_time_repeats": "one unmeasured warmup + five measured repeats, geometry rebuilt per repeat",
            "collision_load_time_threads": "native thread count pinned to 1 via threadpoolctl",
        },
        "staging_policy": (
            "官方发布几何中资源位于扁平 urdf/ 目录的 sibling 目录 partseg/；冻结几何 atom 要求 mesh 引用在 URDF "
            "package 目录内解析，因此每个资产被 staging 为自包含 evaluation package：资源逐字节哈希校验复制（绑定冻结 "
            "Table 5 manifest 行），URDF 仅将 mesh filename 属性相对化进 package；joint/link 声明在冻结前经结构化不变性校验；"
            "审计前后重算 package binding。"
        ),
        "claim_boundary": (
            "官方 PhysX-Mobility URDF 未声明任何 collision 元素（冻结 Table 5 manifest xml_counts.collision_elements "
            "总和为 0）：全部 collision 表示类指标 N/E fail-closed；D_visual 从 loadable visual geometry 推导仍执行。"
        ),
        "execution": {
            "child_timeout_seconds": CHILD_TIMEOUT_SECONDS,
            "workers": WORKERS if args.workers is None else args.workers,
            "child_interpreter": str(CHILD_PYTHON.resolve(strict=True)),
            "denominator_policy": "all frozen assets including failures and unmeasured items",
            "archive_binding_verified": not args.skip_archive_binding,
        },
        "runner_identity": {
            "runner_script": str(SCRIPT),
            "runner_script_sha256": sha256_bytes(SCRIPT.read_bytes()),
            "geometry_module": str(Path(geometry.__file__).resolve()),
            "geometry_module_sha256": sha256_bytes(Path(geometry.__file__).resolve().read_bytes()),
        },
    }
    atomic_write_json(outdir / "frozen_config.json", frozen_config)
    frozen_config_sha256 = sha256_bytes((outdir / "frozen_config.json").read_bytes())

    started_at = utc_now_iso()
    wall_start = time.time()
    records = spawn_children(
        jobs,
        outdir,
        workers=WORKERS if args.workers is None else args.workers,
        timeout_seconds=CHILD_TIMEOUT_SECONDS,
    )
    wall_seconds = round(time.time() - wall_start, 3)

    slim_rows = []
    for record in records:
        slim = {k: v for k, v in record.items() if k != "geometry_record"}
        slim_rows.append(slim)
    atomic_write_text(
        outdir / "asset_records.jsonl",
        "\n".join(json.dumps(r, sort_keys=True, ensure_ascii=True, allow_nan=False) for r in slim_rows) + "\n",
    )

    aggregates = aggregate(records)
    completed_at = utc_now_iso()
    summary = {
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "run_directory": str(outdir),
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_seconds": wall_seconds,
        "cohort": {
            "receipt_set": str(receipt_dir),
            "receipt_manifest_sha256": sha256_bytes((receipt_dir / "manifest.json").read_bytes()),
            "manifest_cohort_sha256": manifest5["cohort_sha256"],
            "n_eval": len(jobs),
        },
        "status_counts": aggregates["status_counts"],
        "metrics": aggregates,
        "frozen_config_sha256": frozen_config_sha256,
        "claim_boundary": {
            "official_urdf_zero_collision_elements": True,
            "collision_representation_metrics_n_e_fail_closed": True,
            "d_visual_derivation_executed": True,
        },
        "notes": [
            "Exact surface backend is trimesh proximity + rtree; Genesis SDF is not used for Table 4b.",
            "D_visual = q0 loadable visual union AABB diagonal; assets without a positive finite D_visual keep surface metrics N/E fail-closed.",
            "Collision load time follows the frozen timing protocol (one unmeasured warmup + five measured repeats, process isolation, native thread count 1).",
            "Official PhysX-Mobility URDFs declare zero collision elements: all collision-representation cells are N/E fail-closed; this is a data property, not an evaluator gap.",
            "All failure modes (parse, resource, measurement, crash, timeout) remain fail-closed in every denominator.",
        ],
    }
    atomic_write_json(outdir / "summary.json", summary)
    atomic_write_text(outdir / "summary.md", render_summary_md(summary))

    verification = (
        verify_run(rows, records, aggregates)
        if mode == "formal"
        else {"all_pass": None, "note": "smoke mode: formal verification skipped"}
    )
    atomic_write_json(outdir / "verification.json", verification)
    run_manifest = {
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "created_at_utc": completed_at,
        "command": sys.argv,
        "frozen_config_sha256": frozen_config_sha256,
        "record_count": len(records),
        "status_counts": aggregates["status_counts"],
        "wall_seconds": wall_seconds,
        "outputs": {
            "asset_records_sha256": sha256_bytes((outdir / "asset_records.jsonl").read_bytes()),
            "summary_sha256": sha256_bytes((outdir / "summary.json").read_bytes()),
            "summary_md_sha256": sha256_bytes((outdir / "summary.md").read_bytes()),
            "verification_sha256": sha256_bytes((outdir / "verification.json").read_bytes()),
        },
        "verification": verification,
    }
    atomic_write_json(outdir / "manifest.json", run_manifest)
    run_manifest["manifest_self_sha256_at_write"] = sha256_bytes((outdir / "manifest.json").read_bytes())
    atomic_write_json(outdir / "manifest.json", run_manifest)

    print(json.dumps(
        {
            "mode": mode,
            "run_directory": str(outdir),
            "status_counts": aggregates["status_counts"],
            "metrics": aggregates,
            "verification_all_pass": verification.get("all_pass"),
        },
        indent=2,
        sort_keys=True,
        ensure_ascii=True,
        default=float,
    ))
    if mode == "formal" and not verification["all_pass"]:
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3, help="smoke sample size (smoke mode only)")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--receipt-set", type=Path, default=None)
    parser.add_argument("--skip-archive-binding", action="store_true")
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return run_child(args.job, args.result)
    if args.mode is None:
        raise SystemExit("--mode is required unless --child is given")
    return run_scope(args)


if __name__ == "__main__":
    raise SystemExit(main())
