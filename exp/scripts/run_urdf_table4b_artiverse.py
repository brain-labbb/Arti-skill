#!/usr/bin/env python3
"""Fail-closed Table 4b runner for Artiverse (exact collision-representation geometry).

Runs the proposed Table 4b Collision Representation Quality and Cost metrics
over the frozen Table 4 Artiverse N=800 cohort (the Table 1 fixed global
sample, sample expression ``jq -r '.assets[].manifest_root'
exp/runtime/table1_artiverse/manifest.json``), reusing the version-pinned
``lam_supplementary_geometry`` module so that the geometry protocol
(``lam_supplementary_geometry_v1``) is identical to the LAM formal run:
exact surface backend ``trimesh.proximity.ProximityQuery.on_surface + rtree``
(no Genesis SDF substitute), 32,768 area-weighted samples per direction,
weld relative tolerance 1e-9, ``D_visual`` = q0 loadable visual union AABB
diagonal (``q0_loadable_visual_union_aabb_diagonal_v1``), and the frozen
collision-load timing protocol (``trimesh.Trimesh(process=False).triangles_tree
+ rtree``, one unmeasured warmup then warm-cache repeats, geometry rebuilt per
repeat, one process per asset, native thread count pinned to 1).

One fresh interpreter process evaluates one asset (geometry record, load-time
timing, triangle validation and intra-link redundancy).  Parse, resource,
measurement, crash and timeout failures all remain fail-closed in every
denominator.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
from pathlib import Path
import platform
import signal
import shutil
import statistics
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_geometry as geometry  # noqa: E402

SCHEMA_VERSION = "table4b-artiverse/v1"
PROTOCOL_ID = "table4b_artiverse_table1cohort_n800_seed20260813_v1"
GEOMETRY_PROTOCOL_ID = geometry.GEOMETRY_PROTOCOL_ID
DATASET = "Artiverse"
CLASSIFICATION = "FORMAL"
FORMAL_OUTPUT_NAME = "table4b_urdf_artiverse_table1cohort_n800_seed20260813_{timestamp}"
SMOKE_OUTPUT_NAME = "table4b_urdf_artiverse_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = (
    "Table 4b - Artiverse (frozen Table 4 Table-1 cohort, N=800; "
    "exact collision-representation geometry)"
)
SELECTION_POLICY = (
    "all Table 4 frozen manifest items (Table 1 fixed global sample) in existing order; "
    "no resampling or result-based filtering"
)
REQUIRE_SMOKE_RECEIPT = False
CHILD_SOURCE_ROOT: Path | None = None

SOURCE_MANIFEST = REPO / "exp/runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json"
EXPECTED_SOURCE_MANIFEST_SHA256 = "0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb"
EXPECTED_ORDERED_IDS_SHA256 = "118038a746cafb91251afde5eb4f1164915d141acb3b529ea721a9d376bde3fa"
TABLE4_STATE_RECORDS = SOURCE_MANIFEST.with_name("state_records.jsonl")
TABLE4_ASSET_RECORDS = SOURCE_MANIFEST.with_name("asset_records.json")
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_artiverse_table1_n800_20260814T031800Z/asset_records.jsonl"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

N_EVAL = 800
EXPECTED_CATEGORY_COUNT = 67
CHILD_TIMEOUT_SECONDS = 900  # matches the LAM supplementary frozen child timeout; timeouts remain
# fail-closed and stay in every denominator.
WORKERS = 16
LAUNCH_STAGGER_SECONDS = 1.5
GEOMETRY_SAMPLES_PER_DIRECTION = 32768
GEOMETRY_WELD_REL_TOL = 1e-9

GENESIS_PYTHON = Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python")
CPU_AFFINITY_WIDTH = 4
CPU_AFFINITY_ENV = "TABLE4B_ARTIVERSE_CPU_AFFINITY"
THREAD_ENV_KEYS = (
    "OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS",
    "NUMEXPR_NUM_THREADS", "BLIS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS",
    "QD_NUM_THREADS", "GS_PARA_LEVEL",
)
THREAD_ENV_VALUES = {
    **{key: "1" for key in THREAD_ENV_KEYS if key != "GS_PARA_LEVEL"},
    "GS_PARA_LEVEL": "0",
}

def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_bytes(payload: bytes) -> str:
    import hashlib

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


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    digest = sha256_bytes(payload)
    if digest != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit(f"source manifest sha256 mismatch: {digest}")
    manifest = json.loads(payload)
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} items")
    if int(manifest.get("sample_size", -1)) != N_EVAL:
        raise SystemExit("source manifest sample_size mismatch")
    ids = [item["asset_id"] for item in items]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    if ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(f"ordered asset id sha256 mismatch: {ordered}")
    for index, item in enumerate(items):
        if item.get("order") != index:
            raise SystemExit(f"item order field mismatch at index {index}")
    return manifest


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    dataset_root = Path(str(manifest["dataset_root"]))
    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        urdf_path = dataset_root / str(item["primary_urdf_relpath"])
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": str(item["dataset_id"]),
                "asset_id": str(item["asset_id"]),
                "category": str(item.get("category")),
                "package": str(urdf_path.parent),
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "input_identity_sha256": str(item["input_identity_sha256"]),
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
        urdf_path = Path(str(job["urdf_path"]))
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
    job_dir = outdir / "child_jobs"
    child_dir = outdir / "children"
    log_dir = outdir / "child_logs"
    job_dir.mkdir(parents=True, exist_ok=True)
    child_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    cache_root = outdir / "genesis-cache"
    cache_root.mkdir(parents=True, exist_ok=True)
    template_cache: dict[str, str | None] = {"path": None}
    genesis_python = str(GENESIS_PYTHON.resolve(strict=True))
    results: list[dict[str, Any]] = []
    pending: dict[int, dict[str, Any]] = {}
    total = len(jobs)
    cpu_count = len(os.sched_getaffinity(0))
    remaining_indices: list[int] = []
    for index, job in enumerate(jobs):
        result_path = child_dir / f"rank_{index + 1:04d}.json"
        if not result_path.is_file():
            remaining_indices.append(index)
            continue
        record = json.loads(result_path.read_text(encoding="utf-8"))
        if (
            int(record.get("selection_index", -1)) != int(job["selection_index"])
            or record.get("asset_id") != job["asset_id"]
            or record.get("expected_urdf_sha256") != job["expected_urdf_sha256"]
            or (
                "expected_package_content_manifest_sha256" in job
                and record.get("expected_package_content_manifest_sha256")
                != job["expected_package_content_manifest_sha256"]
            )
        ):
            raise RuntimeError(f"resume child binding mismatch: rank {index + 1}")
        results.append(record)

    def launch(index: int) -> None:
        job = dict(jobs[index])
        slot = len(results) + len(pending)
        width = max(1, min(CPU_AFFINITY_WIDTH, cpu_count))
        base = (slot * width) % cpu_count
        affinity = ",".join(str((base + offset) % cpu_count) for offset in range(width))
        job["genesis_cache_path"] = str(cache_root / f"rank_{index + 1:04d}")
        job["template_cache_path"] = template_cache["path"]
        job_path = job_dir / f"rank_{index + 1:04d}.json"
        result_path = child_dir / f"rank_{index + 1:04d}.json"
        atomic_write_json(job_path, job)
        env = os.environ.copy()
        env["GS_CACHE_FILE_PATH"] = job["genesis_cache_path"]
        for key, value in THREAD_ENV_VALUES.items():
            env[key] = value
        env[CPU_AFFINITY_ENV] = affinity
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        if CHILD_SOURCE_ROOT is not None:
            env["PYTHONPATH"] = str(CHILD_SOURCE_ROOT)
            env["SKETCHMOBILITY_REPO_ROOT"] = str(REPO)
        log_path = log_dir / f"rank_{index + 1:04d}.log"
        log_handle = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            [genesis_python, str(SCRIPT), "--child", "--job", str(job_path), "--result", str(result_path)],
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

    for index in range(total):
        candidate = cache_root / f"rank_{index + 1:04d}"
        if candidate.is_dir():
            template_cache["path"] = str(candidate)
            break
    if remaining_indices and workers > 1 and template_cache["path"] is None:
        # Warmup: run the first rank alone so its Genesis cache (taichi kernel
        # cache etc.) becomes the read template copied into each later rank's
        # private cache directory. Attempt 1 showed a single shared cache
        # caused concurrent-write SIGABRTs; private caches avoid that.
        warmup_index = remaining_indices.pop(0)
        warmup_target = len(results) + 1
        launch(warmup_index)
        while len(results) < warmup_target:
            time.sleep(0.1)
            for windex in sorted(pending):
                entry = pending[windex]
                proc = entry["proc"]
                rc = proc.poll()
                if rc is not None:
                    del pending[windex]
                    finalize(windex, entry, None if rc == 0 else f"child_process_failed: rc={rc}")
                elif time.time() >= entry["deadline"]:
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
                        proc.wait(timeout=10.0)
                    del pending[windex]
                    finalize(windex, entry, f"asset_timeout after {timeout_seconds}s")
        if template_cache["path"] is None:
            template_cache["path"] = str(
                cache_root / f"rank_{warmup_index + 1:04d}"
            )
    while len(results) < total:
        launched_now = False
        while len(pending) < workers and remaining_indices:
            launch(remaining_indices.pop(0))
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
                    proc.wait(timeout=10.0)
                del pending[index]
                finalize(index, entry, f"asset_timeout after {timeout_seconds}s")
    results.sort(key=lambda r: int(r["selection_index"]))
    return results


def rate(numerator: float, denominator: float) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "percent": round(100.0 * numerator / denominator, 4) if denominator else None,
    }


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
    """Frozen per-asset measurement read: {status, value, reason} semantics."""

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
    for record in records:
        analytic += int(record.get("analytic_collision_element_count", 0))
        loadable += int(record.get("loadable_collision_element_count", 0))
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
    analytic_measured_assets = sum(
        record.get("status") == "completed"
        and int(record.get("loadable_collision_element_count", 0))
        == int(record.get("declared_collision_element_count", -1))
        and int(record.get("loadable_collision_element_count", 0)) > 0
        for record in records
    )
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
            "percent": None if loadable == 0 else 100.0 * analytic / loadable,
            "status": (
                "COMPLETE"
                if n > 0 and analytic_measured_assets == n
                else ("PARTIAL" if analytic_measured_assets > 0 else "N/E")
            ),
            "measured_assets": analytic_measured_assets,
            "intended_assets": n,
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
    }


def verify_run(manifest: Mapping[str, Any], records: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    items = manifest["items"]
    check("source_manifest_sha256", sha256_bytes(SOURCE_MANIFEST.read_bytes()) == EXPECTED_SOURCE_MANIFEST_SHA256)
    ids = [str(item["asset_id"]) for item in items]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check("record_count", len(records) == N_EVAL, f"{len(records)}")
    order_ok = all(
        int(r["selection_index"]) == i and str(r["asset_id"]) == str(items[i]["asset_id"])
        for i, r in enumerate(records)
    )
    check("frozen_order_preserved", order_ok)
    sha_ok = all(
        (r.get("urdf_sha256") is None and r.get("status") != "completed")
        or r.get("urdf_sha256") == str(items[int(r["selection_index"])]["urdf_sha256"])
        for r in records
    )
    check("urdf_identity_matches_frozen_manifest", sha_ok)
    recomputed = aggregate(records)
    check(
        "aggregate_recomputation_matches",
        json.dumps(recomputed, sort_keys=True) == json.dumps(dict(aggregates), sort_keys=True),
    )
    all_pass = all(c["pass"] for c in checks)
    return {"all_pass": all_pass, "check_count": len(checks), "checks": checks}


def _fmt_ratio_pair(cell: Mapping[str, Any]) -> str:
    if not cell.get("intended"):
        return "N/E"
    pct = 100.0 * cell["measured"] / cell["intended"]
    return f"{cell['measured']}/{cell['intended']}, {cell['status']}"


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
            return "N/E"
        return f"{cell['value']:.2e} ({cell['measured']}/{cell['intended']}, {cell['status']})"

    def triple_cell(cell: Mapping[str, Any]) -> str:
        if cell["status"] == "N/E" or cell.get("median") is None:
            return "N/E"
        return (
            f"{cell['mean']} / {cell['median']} / {cell['p90']} "
            f"({cell['measured']}/{cell['intended']}, {cell['status']})"
        )

    def load_time_cell(cell: Mapping[str, Any]) -> str:
        if cell["status"] == "N/E" or cell.get("median") is None:
            return "N/E"
        return (
            f"{cell['median']:.4f} s / {cell['p90']:.4f} s "
            f"({cell['measured']}/{cell['intended']}, {cell['status']})"
        )

    lines = [
        f"# {SUMMARY_TITLE}",
        "",
        f"- Protocol ID: `{summary['protocol_id']}` (geometry `{summary['geometry_protocol_id']}`)",
        f"- Run directory: `{summary['run_directory']}`",
        f"- N_eval = {summary['cohort']['n_eval']}",
        f"- Status: completed = {summary['status_counts']['completed']}, error = {summary['status_counts']['error']}",
        "",
        "| Metric | Result |",
        "|---|---|",
        "| Analytic Collision Share | "
        + (
            "N/E"
            if share["rate"] is None
            else f"{share['analytic_elements']} / {share['loadable_collision_elements']} "
            f"({100.0 * share['rate']:.2f}%; {share['measured_assets']}/"
            f"{share['intended_assets']} assets, {share['status']})"
        )
        + " |",
        f"| Visual→Collision P95 | {p95_cell(v2c)} |",
        f"| Collision→Visual P95 | {p95_cell(c2v)} |",
        f"| Shapes/Visual-bearing Link | {triple_cell(shapes)} |",
        f"| Collision Mesh Triangles/Asset | {triple_cell(triangles)} |",
        f"| Intra-link Redundancy | {redundancy['value']} (links {redundancy['measured']}/{redundancy['intended']}, "
        f"assets {redundancy['measured_assets']}/{redundancy['intended_assets']}, {redundancy['status']}) |",
        f"| Collision Load Time/Asset | {load_time_cell(load_time)} |",
        "",
    ]
    return "\n".join(lines) + "\n"


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode!r}")


def validate_smoke_receipt(path: Path | None) -> dict[str, Any] | None:
    if REQUIRE_SMOKE_RECEIPT:
        raise ValueError("this adapter must define smoke receipt validation")
    return None


def validate_jobs(jobs: Sequence[Mapping[str, Any]], workers: int) -> None:
    del jobs, workers


def snapshot_sources(outdir: Path) -> dict[str, str]:
    del outdir
    return {}


def finalize_receipt(outdir: Path) -> None:
    del outdir


def run_scope(args: argparse.Namespace) -> int:
    mode = args.mode
    if mode == "formal" and args.workers not in {None, WORKERS}:
        raise ValueError(f"formal mode requires workers={WORKERS}")
    smoke_binding = (
        validate_smoke_receipt(args.smoke_receipt) if mode == "formal" else None
    )
    timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    published_outdir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else REPO / "exp/runtime" / output_directory_name(mode, args.n, timestamp)
    )
    if published_outdir.exists():
        raise SystemExit(f"output directory already exists: {published_outdir}")
    outdir = published_outdir.with_name(f".{published_outdir.name}.work")
    if args.resume:
        if not outdir.is_dir():
            raise SystemExit(f"resume staging directory does not exist: {outdir}")
    else:
        if outdir.exists():
            raise SystemExit(f"staging directory already exists: {outdir}")
        outdir.mkdir(parents=True)

    manifest = load_source_manifest()
    jobs = build_jobs(manifest)
    if mode == "smoke":
        jobs = jobs[: args.n]
    validate_jobs(jobs, WORKERS if args.workers is None else args.workers)

    protocol_snapshot_text = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    if args.resume:
        if (
            not (outdir / "protocol_snapshot.md").is_file()
            or (outdir / "protocol_snapshot.md").read_text(encoding="utf-8")
            != protocol_snapshot_text
        ):
            raise RuntimeError("resume protocol snapshot drift")
    else:
        atomic_write_text(outdir / "protocol_snapshot.md", protocol_snapshot_text)
    source_snapshots = snapshot_sources(outdir)

    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "frozen_at_utc": utc_now_iso(),
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "protocol_document_sha256": sha256_bytes(protocol_snapshot_text.encode("utf-8")),
        "cohort": {
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_sha256": EXPECTED_SOURCE_MANIFEST_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "n_eval": len(jobs),
            "selection_policy": SELECTION_POLICY,
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
        "execution": {
            "child_timeout_seconds": CHILD_TIMEOUT_SECONDS,
            "workers": WORKERS if args.workers is None else args.workers,
            "child_interpreter": str(GENESIS_PYTHON.resolve(strict=True)),
            "process_isolation": "one fresh interpreter process per asset",
            "denominator_policy": "all frozen assets including failures and unmeasured items",
        },
        "smoke_receipt": smoke_binding,
        "source_snapshots": source_snapshots,
        "runner_identity": {
            "runner_script": str(SCRIPT),
            "runner_script_sha256": sha256_bytes(SCRIPT.read_bytes()),
            "geometry_module": str(Path(geometry.__file__).resolve()),
            "geometry_module_sha256": sha256_bytes(Path(geometry.__file__).resolve().read_bytes()),
        },
    }
    if args.resume:
        existing_config = json.loads(
            (outdir / "frozen_config.json").read_text(encoding="utf-8")
        )
        comparable_existing = dict(existing_config)
        comparable_current = dict(frozen_config)
        comparable_existing.pop("frozen_at_utc", None)
        comparable_current.pop("frozen_at_utc", None)
        if comparable_existing != comparable_current:
            raise RuntimeError("resume frozen configuration drift")
        frozen_config = existing_config
    else:
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
    cache_root = outdir / "genesis-cache"
    if cache_root.is_dir():
        shutil.rmtree(cache_root)

    atomic_write_text(
        outdir / "asset_records.jsonl",
        "\n".join(json.dumps(r, sort_keys=True, ensure_ascii=True, allow_nan=False) for r in records) + "\n",
    )

    aggregates = aggregate(records)
    completed_at = utc_now_iso()
    summary = {
        "protocol_id": PROTOCOL_ID,
        "geometry_protocol_id": geometry.GEOMETRY_PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "run_directory": str(published_outdir),
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_seconds": wall_seconds,
        "cohort": {
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_sha256": EXPECTED_SOURCE_MANIFEST_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "n_eval": len(jobs),
        },
        "status_counts": aggregates["status_counts"],
        "metrics": aggregates,
        "frozen_config_sha256": frozen_config_sha256,
        "notes": [
            "Exact surface backend is trimesh proximity + rtree; Genesis SDF is not used for Table 4b.",
            "D_visual = q0 loadable visual union AABB diagonal; assets without a positive finite D_visual keep surface metrics N/E fail-closed.",
            "Collision load time follows the frozen timing protocol (one unmeasured warmup + five measured repeats, process isolation, native thread count 1).",
            "All failure modes (parse, resource, measurement, crash, timeout) remain fail-closed in every denominator.",
        ],
    }
    atomic_write_json(outdir / "summary.json", summary)
    atomic_write_text(outdir / "summary.md", render_summary_md(summary))

    verification = (
        verify_run(manifest, records, aggregates)
        if mode == "formal"
        else {"all_pass": None, "note": "smoke mode: formal verification skipped"}
    )
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
        },
        "verification": verification,
    }
    atomic_write_json(outdir / "manifest.json", run_manifest)
    run_manifest["manifest_self_sha256_at_write"] = sha256_bytes((outdir / "manifest.json").read_bytes())
    atomic_write_json(outdir / "manifest.json", run_manifest)

    finalize_receipt(outdir)

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
    os.replace(outdir, published_outdir)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3, help="smoke sample size (smoke mode only)")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--smoke-receipt", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output-dir", type=Path)
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
