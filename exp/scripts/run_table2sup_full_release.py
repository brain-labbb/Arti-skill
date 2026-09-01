#!/usr/bin/env python3
"""Full-release runner for the proposed Table 2 supplementary diagnostics.

The runner is deliberately method agnostic: every row is evaluated through
``table2_supplementary_static.audit_package``.  It owns only roster binding,
bounded child execution, resumable records and denominator-preserving
aggregation.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

try:
    from . import table2_supplementary_static as static
except ImportError:  # direct script execution
    import table2_supplementary_static as static  # type: ignore

SCHEMA_VERSION = "table2sup_full_release_run_v2"
ROSTER_SCHEMA = "table123_full_release_manifest_v1"
DATASETS = ("articraft", "lam", "artiverse", "partnet", "physx", "sketch", "infinite", "infinigen")
DISPLAY_NAMES = {
    "articraft": "Articraft-10K", "lam": "LAM", "artiverse": "Artiverse",
    "partnet": "PartNet-Mobility", "physx": "PhysX-Mobility", "sketch": "SketchMobility",
    "infinite": "Infinite Mobility", "infinigen": "Infinigen-Sim",
}


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write(path: Path, payload: str) -> None:
    """Atomically publish text and refuse to overwrite a symlink."""

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to overwrite symlink: {path}")
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_write(
        path,
        json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False)
        + "\n",
    )


def _append_jsonl(path: Path, value: Mapping[str, Any]) -> None:
    """Append one complete JSONL row; only the parent process calls this."""

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to append to symlink: {path}")
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(value, sort_keys=True, ensure_ascii=True, allow_nan=False))
        handle.write("\n")
        handle.flush()


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _without_hash(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    return static.canonical_sha256(_without_hash(value, field))


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"JSONL row is not an object: {path}:{line_number}")
        rows.append(value)
    return rows


def package_relative_urdf(row: Mapping[str, Any]) -> str:
    """Return the primary URDF path relative to the evaluator package.

    Release manifests retain a source-root-relative path for provenance, while
    the static auditor requires a package-relative path.  The absolute frozen
    path is authoritative when available.  Infinite Mobility rows use the
    already package-scoped ``urdf_relpath``/``urdf`` fields.
    """

    for key in ("urdf_relpath", "urdf_relative_path"):
        value = row.get(key)
        if isinstance(value, str) and value and not Path(value).is_absolute():
            if key == "urdf_relpath" and row.get("package_path") is not None:
                return Path(value).as_posix()
    package_value = row.get("source_path") or row.get("package_path") or row.get("package_root") or row.get("package")
    primary_value = row.get("primary_urdf_path") or row.get("urdf_path") or row.get("primary_urdf")
    if isinstance(primary_value, str) and primary_value:
        primary = Path(primary_value)
        if not primary.is_absolute() and isinstance(package_value, str):
            primary = Path(package_value) / primary
    else:
        declared = row.get("primary_urdf_relative_path") or row.get("urdf_relpath") or row.get("urdf")
        if isinstance(declared, str) and declared:
            return Path(declared).as_posix()
        return "model.urdf"
    if not isinstance(package_value, str) or not package_value:
        return primary.as_posix()
    package = Path(package_value).resolve(strict=False)
    try:
        return primary.resolve(strict=False).relative_to(package).as_posix()
    except ValueError:
        raise ValueError("primary URDF escapes source package")


def _expected_hash(row: Mapping[str, Any]) -> str | None:
    for key in ("primary_urdf_sha256", "expected_primary_urdf_sha256", "urdf_sha256"):
        value = row.get(key)
        if isinstance(value, str) and value:
            return value
    return None


def _expected_joints(row: Mapping[str, Any]) -> int:
    value = row.get("joint_count")
    if value is None:
        value = row.get("declared_joint_count_hint")
    if value is None:
        value = row.get("expected_movable_joints", 0)
    if isinstance(value, bool):
        raise ValueError("joint count must be a non-negative integer")
    try:
        value = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("joint count must be a non-negative integer") from error
    if value < 0:
        raise ValueError("joint count must be a non-negative integer")
    return value


def build_jobs_from_manifest(manifest: Mapping[str, Any], source_kind: str | None = None) -> list[dict[str, Any]]:
    """Translate immutable full-release rows into static-auditor jobs."""

    if manifest.get("schema_version") != ROSTER_SCHEMA:
        raise ValueError("table123_full_release_manifest_v1 required")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ValueError("manifest rows are missing")
    if manifest.get("N_eval") is not None and int(manifest["N_eval"]) != len(rows):
        raise ValueError("manifest N_eval does not match row count")
    expected_j = sum(_expected_joints(row) for row in rows if isinstance(row, Mapping))
    if manifest.get("J_eval") is not None and int(manifest["J_eval"]) != expected_j:
        raise ValueError("manifest J_eval does not match row joint counts")
    declared_manifest_hash = manifest.get("manifest_content_sha256")
    if declared_manifest_hash:
        observed_manifest_hash = static.canonical_sha256(
            {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
        )
        if declared_manifest_hash != observed_manifest_hash:
            raise ValueError("roster manifest self-hash mismatch")
    jobs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping) or not isinstance(row.get("asset_id"), str):
            raise ValueError(f"invalid roster row at index {index}")
        package = row.get("package_path") if source_kind == "infinite" and row.get("package_path") else row.get("source_path") or row.get("package_root") or row.get("package")
        if not isinstance(package, str) or not package:
            raise ValueError(f"roster row has no package: {row['asset_id']}")
        jobs.append({
            "selection_index": index,
            "asset_id": str(row["asset_id"]),
            "category": str(row.get("category") or row.get("raw_category") or ""),
            "package": package,
            "primary_urdf_relative_path": package_relative_urdf(row),
            "expected_primary_urdf_sha256": _expected_hash(row),
            "expected_movable_joints": _expected_joints(row),
        })
    return jobs


def _rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {"numerator": int(numerator), "denominator": int(denominator), "percent": round(100.0 * numerator / denominator, 2) if denominator else None}


def aggregate_records(records: Sequence[Mapping[str, Any]], n_eval: int, j_eval: int) -> dict[str, Any]:
    """Aggregate diagnostics while retaining the supplied N/J denominators."""

    status_counts = dict(Counter(str(record.get("status", "error")) for record in records))
    visual_passed = visual_declared = visual_covered = extraction_complete = zero_visual = 0
    port_passed = port_extracted = dyn_covered = dyn_extracted = 0
    dynamic_links = complete_inertial = 0
    categories: dict[str, dict[str, int]] = {}
    for record in records:
        t2 = record.get("table2_supplementary") or {}
        visual = t2.get("visual_bearing_collision_coverage") or {}
        port = t2.get("joint_limit_portability") or {}
        dyn = t2.get("joint_dynamics_coverage") or {}
        ph = t2.get("placeholder_mass_incidence") or {}
        visual_passed += int(bool(visual.get("asset_pass")))
        visual_declared += int(visual.get("visual_bearing_links_declared", 0) or 0)
        visual_covered += int(visual.get("covered_visual_bearing_links", 0) or 0)
        extraction_complete += int(bool(visual.get("link_extraction_complete")))
        if record.get("status") == "completed" and int(visual.get("visual_bearing_links_declared", 0) or 0) == 0:
            zero_visual += 1
        port_passed += int(port.get("joints_passed", 0) or 0)
        port_extracted += int(port.get("joints_extracted", 0) or 0)
        dyn_covered += int(dyn.get("joints_covered", 0) or 0)
        dyn_extracted += int(dyn.get("joints_extracted", 0) or 0)
        dynamic_links += int(ph.get("dynamic_links", 0) or 0)
        complete_inertial += int(ph.get("complete_inertial_links", 0) or 0)
        category = str(record.get("category") or "")
        bucket = categories.setdefault(category, {"assets": 0, "visual_passed": 0, "port_passed": 0, "dyn_covered": 0, "port_intended": 0, "dyn_intended": 0})
        bucket["assets"] += 1
        bucket["visual_passed"] += int(bool(visual.get("asset_pass")))
        bucket["port_passed"] += int(port.get("joints_passed", 0) or 0)
        bucket["dyn_covered"] += int(dyn.get("joints_covered", 0) or 0)
        bucket["port_intended"] += int(port.get("joints_intended", 0) or 0)
        bucket["dyn_intended"] += int(dyn.get("joints_intended", 0) or 0)
    category_macro = {
        "category_count": len(categories),
        "categories": {
            name: {"assets": b["assets"], "visual_bearing_asset_rate": _rate(b["visual_passed"], b["assets"]), "portability_joint_rate": _rate(b["port_passed"], b["port_intended"]), "dynamics_joint_rate": _rate(b["dyn_covered"], b["dyn_intended"])}
            for name, b in sorted(categories.items())
        },
    }
    return {
        "n_eval": int(n_eval), "j_eval": int(j_eval), "status_counts": status_counts,
        "parse_passed_assets": sum(1 for r in records if (r.get("parse") or {}).get("success")),
        "metrics": {
            "visual_bearing_collision_coverage": {"asset": _rate(visual_passed, n_eval), "link_micro": _rate(visual_covered, visual_declared), "link_extraction_complete_assets": extraction_complete, "zero_visual_bearing_assets_completed": zero_visual},
            "joint_limit_portability": _rate(port_passed, j_eval) | {"joints_extracted": port_extracted},
            "joint_dynamics_coverage": _rate(dyn_covered, j_eval) | {"joints_extracted": dyn_extracted},
            "placeholder_mass_incidence": {"status": "N/E", "reason": "placeholder_registry_empty", "registry_ids": [], "complete_inertial_links": complete_inertial, "dynamic_links_measured": dynamic_links, "coverage": _rate(complete_inertial, dynamic_links)},
        },
        "category_macro": category_macro,
    }


def source_binding_for_parts_zip(path: Path | str) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"parts archive is not a file: {path}")
    return {"path": str(path), "sha256": _hash_file(path), "bytes": path.stat().st_size}


def _failure(job: Mapping[str, Any], issue: str, status: str = "error") -> dict[str, Any]:
    intended = int(job.get("expected_movable_joints", 0))
    return {
        "schema_version": static.SCHEMA_VERSION,
        "asset_id": str(job["asset_id"]),
        "selection_index": int(job.get("selection_index", 0)),
        "category": str(job.get("category", "")),
        "package": str(job.get("package", "")),
        "urdf_relative_path": str(job.get("primary_urdf_relative_path", "model.urdf")),
        "status": status,
        "parse": {"success": False, "issues": [issue]},
        "result_origin": "parent_synthesized",
        "expected_primary_urdf_sha256": job.get("expected_primary_urdf_sha256"),
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {"asset_pass": False, "visual_bearing_links_declared": 0, "covered_visual_bearing_links": 0, "link_extraction_complete": False},
            "joint_limit_portability": {"joints_intended": intended, "joints_extracted": 0, "joints_passed": 0},
            "joint_dynamics_coverage": {"joints_intended": intended, "joints_extracted": 0, "joints_covered": 0},
            "placeholder_mass_incidence": {"dynamic_links": 0, "complete_inertial_links": 0},
        },
        "issues": [issue],
    }


def _run_child(job: Mapping[str, Any], result_path: Path) -> None:
    try:
        # Give each audit an owned process group so timeout cleanup also
        # terminates descendants a parser/auditor may create.
        if hasattr(os, "setsid"):
            try:
                os.setsid()
            except OSError:
                pass
        expected = job.get("expected_primary_urdf_sha256")
        package = Path(str(job["package"])).resolve(strict=True)
        urdf_rel = str(job["primary_urdf_relative_path"])
        urdf = (package / urdf_rel).resolve(strict=False)
        try:
            urdf.relative_to(package)
        except ValueError as error:
            raise ValueError("primary URDF escapes package") from error
        if expected and (not urdf.is_file() or _hash_file(urdf) != expected):
            raise ValueError("primary URDF hash drift before audit")
        record = static.audit_package(
            package,
            urdf_relative_path=urdf_rel,
            asset_id=str(job["asset_id"]),
            expected_movable_joints=int(job.get("expected_movable_joints", 0)),
            placeholder_registry=[],
        )
        if expected and urdf.is_file() and _hash_file(urdf) != expected:
            raise ValueError("primary URDF hash drift after audit")
        record["category"] = str(job.get("category", ""))
        record["selection_index"] = int(job.get("selection_index", 0))
        record["result_origin"] = "child_audit"
        record["child_pid"] = os.getpid()
        record["expected_primary_urdf_sha256"] = expected
        record["expected_movable_joints"] = int(job.get("expected_movable_joints", 0))
        if job.get("run_manifest_content_sha256"):
            record["run_manifest_content_sha256"] = job["run_manifest_content_sha256"]
    except BaseException as error:  # fail closed
        record = _failure(job, f"audit_exception: {type(error).__name__}: {error}")
        record["selection_index"] = int(job.get("selection_index", 0))
    _atomic_write(result_path, json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n")


def _child_main(job_path: Path, result_path: Path) -> int:
    try:
        job = json.loads(job_path.read_text(encoding="utf-8"))
        _run_child(job, result_path)
        return 0
    except BaseException as error:
        try:
            job = json.loads(job_path.read_text(encoding="utf-8"))
            _atomic_write(result_path, json.dumps(_failure(job, f"child_exception: {type(error).__name__}: {error}"), sort_keys=True, ensure_ascii=True) + "\n")
        except Exception:
            pass
        return 1


def _run_subprocesses_legacy(
    pending: Sequence[Mapping[str, Any]],
    output: Path,
    *,
    workers: int,
    timeout_seconds: float,
    records: dict[str, dict[str, Any]],
    on_batch: Any,
) -> None:
    """Run child audits concurrently with process-group timeout cleanup."""
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    child_jobs = output / "child_jobs"; child_jobs.mkdir(parents=True, exist_ok=True)
    children = output / "children"; children.mkdir(parents=True, exist_ok=True)
    queue = list(pending); active: dict[int, tuple[Mapping[str, Any], subprocess.Popen[str], Path, float]] = {}
    next_index = 0
    changed = 0
    env = dict(os.environ)
    env.update({"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1"})

    def launch(job: Mapping[str, Any]) -> None:
        index = int(job.get("selection_index", 0)); job_path = child_jobs / f"{index:06d}.json"; result_path = children / f"{index:06d}.json"
        _atomic_write(job_path, json.dumps(dict(job), sort_keys=True, ensure_ascii=True) + "\n")
        proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--child", "--job", str(job_path), "--result", str(result_path)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True, text=True)
        active[proc.pid] = (job, proc, result_path, time.monotonic())

    with ThreadPoolExecutor(max_workers=workers) as launcher_pool:
      while queue or active:
        launch_futures = []
        while queue and len(active) + len(launch_futures) < workers:
            launch_futures.append(launcher_pool.submit(launch, queue.pop(0)))
        for future in launch_futures:
            future.result()
        now = time.monotonic()
        for pid, (job, proc, result_path, started) in list(active.items()):
            timed_out = now - started >= timeout_seconds and proc.poll() is None
            if timed_out:
                try: os.killpg(proc.pid, signal.SIGTERM)
                except OSError: proc.terminate()
                try: proc.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    try: os.killpg(proc.pid, signal.SIGKILL)
                    except OSError: proc.kill()
                    proc.wait(timeout=1)
                records[str(job["asset_id"])] = _failure(job, f"asset_timeout_after_{timeout_seconds:g}_seconds", status="timeout")
            elif proc.poll() is None:
                continue
            else:
                if result_path.exists():
                    try: records[str(job["asset_id"])] = json.loads(result_path.read_text(encoding="utf-8"))
                    except Exception as error: records[str(job["asset_id"])] = _failure(job, f"child_result_invalid: {error}")
                else:
                    records[str(job["asset_id"])] = _failure(job, f"child_exit_{proc.returncode}")
            del active[pid]; changed += 1
            if changed >= 32:
                on_batch(); changed = 0
        if active:
            time.sleep(0.01)
    if changed:
        on_batch()


def _kill_process_group(process: subprocess.Popen[str]) -> None:
    """Terminate a timed-out child and all descendants, best effort."""

    try:
        os.killpg(process.pid, signal.SIGTERM)
    except (OSError, AttributeError):
        try:
            process.terminate()
        except OSError:
            pass
    try:
        process.wait(timeout=1.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except (OSError, AttributeError):
        try:
            process.kill()
        except OSError:
            pass
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _execute_job(
    job: Mapping[str, Any],
    output: Path,
    *,
    timeout_seconds: float,
) -> dict[str, Any]:
    """Execute one job in a fresh interpreter and return a bound record."""

    index = int(job.get("selection_index", 0))
    child_jobs = output / "child_jobs"
    children = output / "children"
    child_jobs.mkdir(parents=True, exist_ok=True)
    children.mkdir(parents=True, exist_ok=True)
    job_path = child_jobs / f"{index:06d}.json"
    result_path = children / f"{index:06d}.json"
    _atomic_json(job_path, dict(job))
    result_path.unlink(missing_ok=True)
    env = dict(os.environ)
    env.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    process = subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--child",
            "--job",
            str(job_path),
            "--result",
            str(result_path),
        ],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        text=True,
    )
    try:
        return_code = process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        _kill_process_group(process)
        record = _failure(
            job,
            f"asset_timeout_after_{timeout_seconds:g}_seconds",
            status="timeout",
        )
        _atomic_json(result_path, record)
        return record
    if return_code != 0 or not result_path.is_file():
        record = _failure(job, f"child_exit_{return_code}")
        _atomic_json(result_path, record)
        return record
    try:
        record = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        record = _failure(job, f"child_result_invalid: {error}")
        _atomic_json(result_path, record)
        return record
    if not isinstance(record, dict):
        record = _failure(job, "child_result_not_object")
        _atomic_json(result_path, record)
        return record
    if (
        str(record.get("asset_id")) != str(job.get("asset_id"))
        or int(record.get("selection_index", -1)) != index
        or record.get("expected_primary_urdf_sha256")
        != job.get("expected_primary_urdf_sha256")
    ):
        record = _failure(job, "child_result_binding_mismatch")
        _atomic_json(result_path, record)
        return record
    return record


def _run_subprocesses(
    pending: Sequence[Mapping[str, Any]],
    output: Path,
    *,
    workers: int,
    timeout_seconds: float,
    records: dict[str, dict[str, Any]],
    on_batch: Any,
) -> None:
    """Run child audits concurrently with process-group timeout cleanup."""

    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    changed = 0
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                _execute_job,
                job,
                output,
                timeout_seconds=timeout_seconds,
            ): job
            for job in pending
        }
        from concurrent.futures import as_completed

        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except BaseException as error:  # fail closed, retain denominator
                record = _failure(
                    job,
                    f"parent_executor_exception: {type(error).__name__}: {error}",
                )
            records[str(job["asset_id"])] = record
            changed += 1
            if changed >= 32:
                on_batch()
                changed = 0
    if changed:
        on_batch()


def build_combined_receipt(outputs: Mapping[str, Path | str] | Sequence[Path | str], root: Path | str) -> dict[str, Any]:
    root = Path(root).resolve(); entries = []
    items = outputs.items() if isinstance(outputs, Mapping) else ((Path(p).name, p) for p in outputs)
    for key, value in items:
        path = Path(value); summary_path = path / "summary.json" if path.is_dir() else path
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        manifest_path = summary_path.parent / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
        slug = str(key).lower().replace("_", "-")
        entries.append({"slug": slug, "dataset": manifest.get("dataset", key), "display": DISPLAY_NAMES.get(slug, manifest.get("dataset", key)), "N_eval": summary.get("n_eval"), "J_eval": summary.get("j_eval"), "status": "complete", "metrics": summary.get("metrics", {}), "evidence": {"summary": str(summary_path.relative_to(root)) if summary_path.is_relative_to(root) else str(summary_path)}})
    return {"schema_version": "table2sup_full_release_receipt_v1", "root": str(root), "methods": entries}


def _run_dataset_legacy(roster_path: Path, output: Path, *, workers: int, timeout_seconds: float, resume: bool = False, source_kind: str | None = None, parts_zip: Path | None = None, source_root: Path | None = None) -> Path:
    manifest = json.loads(Path(roster_path).read_text(encoding="utf-8"))
    jobs = build_jobs_from_manifest(manifest, source_kind=source_kind)
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=True)
    children = output / "children"; children.mkdir(exist_ok=True)
    records_path = output / "records.jsonl"
    records: dict[str, dict[str, Any]] = {}
    if resume and records_path.exists():
        manifest_path = output / "manifest.json"
        if manifest_path.exists():
            previous_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            unsigned = dict(previous_manifest); declared_manifest_hash = unsigned.pop("manifest_content_sha256", None)
            if declared_manifest_hash and declared_manifest_hash != static.canonical_sha256(unsigned):
                raise ValueError("run manifest self-hash mismatch")
            if previous_manifest.get("roster_sha256") != _hash_file(Path(roster_path)):
                raise ValueError("resume roster hash mismatch")
        checkpoint_path = output / "checkpoint.json"
        if checkpoint_path.exists():
            checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
            declared = checkpoint.get("checkpoint_content_sha256")
            unsigned = dict(checkpoint); unsigned.pop("checkpoint_content_sha256", None)
            if declared and declared != static.canonical_sha256(unsigned):
                raise ValueError("checkpoint self-hash mismatch")
            if checkpoint.get("manifest_content_sha256") and manifest_path.exists() and checkpoint.get("manifest_content_sha256") != previous_manifest.get("manifest_content_sha256"):
                raise ValueError("checkpoint manifest binding mismatch")
            declared_records = checkpoint.get("records_sha256")
            if declared_records and _hash_file(records_path) != declared_records:
                raise ValueError("checkpoint records hash mismatch")
        for line in records_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                item = json.loads(line)
                asset_id = str(item.get("asset_id", ""))
                if not asset_id or asset_id in records:
                    raise ValueError("records contain duplicate or missing asset IDs")
                records[asset_id] = item
    manifest_out = {"schema_version": SCHEMA_VERSION, "dataset": manifest.get("dataset"), "source_kind": source_kind, "roster": str(Path(roster_path).resolve()), "N_eval": manifest.get("N_eval", len(jobs)), "J_eval": manifest.get("J_eval", sum(j["expected_movable_joints"] for j in jobs)), "source_bindings": list(manifest.get("source_bindings", []))}
    manifest_out["roster_sha256"] = _hash_file(Path(roster_path))
    manifest_out["roster_manifest_content_sha256"] = manifest.get("manifest_content_sha256")
    if parts_zip is not None: manifest_out["source_bindings"].append({"name": "parts_zip", **source_binding_for_parts_zip(parts_zip)})
    if source_root is not None:
        source_root = Path(source_root).resolve(strict=True)
        if not source_root.is_dir():
            raise ValueError(f"source root is not a directory: {source_root}")
        manifest_out["source_bindings"].append({"name": "source_root", "path": str(source_root)})
    manifest_out["manifest_content_sha256"] = static.canonical_sha256({k: v for k, v in manifest_out.items() if k != "manifest_content_sha256"})
    _atomic_write(output / "manifest.json", json.dumps(manifest_out, indent=2, sort_keys=True) + "\n")
    pending = [job for job in jobs if job["asset_id"] not in records]
    expected_ids = {job["asset_id"] for job in jobs}
    if any(asset_id not in expected_ids for asset_id in records):
        raise ValueError("records contain foreign asset IDs")

    def persist_running() -> None:
        ordered_partial = [records[job["asset_id"]] for job in jobs if job["asset_id"] in records]
        _atomic_write(records_path, "".join(json.dumps(r, sort_keys=True, ensure_ascii=True) + "\n" for r in ordered_partial))
        checkpoint_running = {"state": "running", "records": len(ordered_partial), "records_sha256": _hash_file(records_path), "roster_sha256": manifest_out["roster_sha256"], "manifest_content_sha256": manifest_out["manifest_content_sha256"]}
        checkpoint_running["checkpoint_content_sha256"] = static.canonical_sha256(checkpoint_running)
        _atomic_write(output / "checkpoint.json", json.dumps(checkpoint_running, indent=2, sort_keys=True) + "\n")

    _run_subprocesses(
        pending,
        output,
        workers=workers,
        timeout_seconds=timeout_seconds,
        records=records,
        on_batch=persist_running,
    )
    ordered = [records[job["asset_id"]] for job in jobs]
    _atomic_write(records_path, "".join(json.dumps(r, sort_keys=True, ensure_ascii=True) + "\n" for r in ordered))
    summary = aggregate_records(ordered, int(manifest_out["N_eval"]), int(manifest_out["J_eval"]))
    _atomic_write(output / "summary.json", json.dumps(summary, indent=2, sort_keys=True) + "\n")
    _atomic_write(output / "summary.md", f"# {manifest.get('dataset', source_kind)} Table 2 supplementary\n\n- N_eval: {summary['n_eval']}\n- J_eval: {summary['j_eval']}\n")
    checkpoint = {"state": "complete", "records": len(ordered), "records_sha256": _hash_file(records_path), "roster_sha256": manifest_out["roster_sha256"], "manifest_content_sha256": manifest_out["manifest_content_sha256"]}
    checkpoint["checkpoint_content_sha256"] = static.canonical_sha256(checkpoint)
    _atomic_write(output / "checkpoint.json", json.dumps(checkpoint, indent=2, sort_keys=True) + "\n")
    _atomic_write(output / "protocol_snapshot.md", "# Protocol snapshot\n\nEvaluator: `table2_supplementary_static.audit_package`\n")
    artifacts = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json": artifacts.append({"path": str(path.relative_to(output)), "bytes": path.stat().st_size, "sha256": _hash_file(path)})
    _atomic_write(output / "artifact_manifest.json", json.dumps({"schema_version": "artifact-closure/v1", "files": artifacts}, indent=2, sort_keys=True) + "\n")
    return output


def _read_manifest(path: Path, source_kind: str | None = None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"roster is not a file: {path}")
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid roster manifest: {error}") from error
    if not isinstance(manifest, dict):
        raise ValueError("roster manifest must be an object")
    jobs = build_jobs_from_manifest(manifest, source_kind=source_kind)
    n_eval = int(manifest.get("N_eval", len(jobs)))
    j_eval = int(manifest.get("J_eval", sum(job["expected_movable_joints"] for job in jobs)))
    if n_eval != len(jobs):
        raise ValueError(f"N_eval mismatch: {n_eval} != {len(jobs)}")
    if j_eval != sum(job["expected_movable_joints"] for job in jobs):
        raise ValueError("J_eval mismatch with frozen row joint counts")
    return manifest, jobs


def _binding_list(
    manifest: Mapping[str, Any],
    *,
    source_kind: str | None,
    parts_zip: Path | None,
    source_root: Path | None,
    roster_path: Path,
) -> list[dict[str, Any]]:
    bindings: list[dict[str, Any]] = []
    for raw in manifest.get("source_bindings", []):
        if isinstance(raw, Mapping) and raw.get("path"):
            bindings.append(dict(raw))
    if parts_zip is not None:
        bindings.append({"name": "parts_zip", **source_binding_for_parts_zip(parts_zip)})
    if source_root is not None:
        resolved = Path(source_root).resolve(strict=True)
        if not resolved.is_dir():
            raise ValueError(f"source root is not a directory: {resolved}")
        bindings.append({"name": "source_root", "path": str(resolved)})
    if source_kind == "infinigen":
        candidates = (
            roster_path.parent.parent / "infinigen_archive_validation_receipt.json",
            roster_path.parent / "infinigen_archive_validation_receipt.json",
        )
        for candidate in candidates:
            if candidate.is_file() and not any(
                str(item.get("path")) == str(candidate.resolve()) for item in bindings
            ):
                bindings.append(
                    {
                        "name": "infinigen_archive_validation_receipt",
                        "path": str(candidate.resolve()),
                        "sha256": _hash_file(candidate),
                        "bytes": candidate.stat().st_size,
                    }
                )
                break
    # Stable de-duplication keeps the run manifest deterministic when a source
    # manifest already contains one of the explicit bindings above.
    unique: dict[tuple[str, str], dict[str, Any]] = {}
    for binding in bindings:
        key = (str(binding.get("name", "")), str(binding.get("path", "")))
        unique[key] = binding
    return [unique[key] for key in sorted(unique)]


def _validate_record_for_job(record: Mapping[str, Any], job: Mapping[str, Any], run_hash: str | None = None) -> None:
    if str(record.get("asset_id")) != str(job.get("asset_id")):
        raise ValueError("record asset_id does not match job")
    if int(record.get("selection_index", -1)) != int(job.get("selection_index", -2)):
        raise ValueError("record selection_index does not match job")
    if record.get("expected_primary_urdf_sha256") != job.get("expected_primary_urdf_sha256"):
        raise ValueError("record primary URDF binding does not match job")
    if run_hash is not None and record.get("run_manifest_content_sha256") not in {None, run_hash}:
        raise ValueError("record run manifest binding does not match job")
    if str(record.get("status")) not in {"completed", "error", "timeout"}:
        raise ValueError("record has an unknown terminal status")


def _artifact_manifest(output: Path) -> dict[str, Any]:
    entries: list[dict[str, Any]] = []
    for path in sorted(output.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        if path.is_symlink():
            raise ValueError(f"artifact closure contains symlink: {path}")
        entries.append(
            {
                "path": path.relative_to(output).as_posix(),
                "size": path.stat().st_size,
                "sha256": _hash_file(path),
            }
        )
    artifact: dict[str, Any] = {
        "schema_version": "table123_artifact_manifest_v1",
        "artifacts": entries,
    }
    artifact["artifact_manifest_content_sha256"] = _self_hash(
        artifact, "artifact_manifest_content_sha256"
    )
    return artifact


def run_dataset(
    roster_path: Path,
    output: Path,
    *,
    workers: int,
    timeout_seconds: float,
    resume: bool = False,
    source_kind: str | None = None,
    parts_zip: Path | None = None,
    source_root: Path | None = None,
) -> Path:
    """Evaluate one immutable full-release roster and publish a receipt."""

    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    roster_path = Path(roster_path).resolve(strict=True)
    manifest, jobs = _read_manifest(roster_path, source_kind=source_kind)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if output.is_symlink():
        raise ValueError(f"output is a symlink: {output}")
    records_path = output / "records.jsonl"
    old_manifest_path = output / "manifest.json"
    protocol_source = Path(__file__).resolve().parents[1] / "URDF-Sim-Ready-Automatic-Evaluation.md"
    if not protocol_source.is_file():
        raise ValueError(f"protocol document is missing: {protocol_source}")
    protocol_snapshot = output / "protocol_snapshot.md"
    if protocol_snapshot.exists():
        protocol_hash = _hash_file(protocol_snapshot)
    else:
        _atomic_write(protocol_snapshot, protocol_source.read_text(encoding="utf-8"))
        protocol_hash = _hash_file(protocol_snapshot)
    roster_bytes_hash = _hash_file(roster_path)
    source_bindings = _binding_list(
        manifest,
        source_kind=source_kind,
        parts_zip=parts_zip,
        source_root=source_root,
        roster_path=roster_path,
    )
    n_eval = int(manifest.get("N_eval", len(jobs)))
    j_eval = int(manifest.get("J_eval", sum(job["expected_movable_joints"] for job in jobs)))
    jobs_hash = static.canonical_sha256(jobs)
    evaluator_path = Path(static.__file__).resolve()
    evaluator_hash = _hash_file(evaluator_path)
    base_manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "dataset": manifest.get("dataset", source_kind),
        "source_kind": source_kind,
        "roster": str(roster_path),
        "roster_sha256": roster_bytes_hash,
        "roster_manifest_content_sha256": manifest.get("manifest_content_sha256"),
        "jobs_sha256": jobs_hash,
        "N_eval": n_eval,
        "J_eval": j_eval,
        "source_bindings": source_bindings,
        "protocol_snapshot": str(protocol_snapshot),
        "protocol_snapshot_sha256": protocol_hash,
        "evaluator": {
            "module": str(evaluator_path),
            "module_sha256": evaluator_hash,
            "schema_version": static.SCHEMA_VERSION,
        },
        "execution": {
            "workers": int(workers),
            "timeout_seconds": float(timeout_seconds),
            "fresh_interpreter_per_asset": True,
        },
        "started_at": _utc_now(),
    }
    base_manifest["manifest_content_sha256"] = _self_hash(
        base_manifest, "manifest_content_sha256"
    )

    records: dict[str, dict[str, Any]] = {}
    persisted_ids: set[str] = set()
    previous: dict[str, Any] | None = None
    if resume and old_manifest_path.is_file():
        previous = json.loads(old_manifest_path.read_text(encoding="utf-8"))
        declared = previous.get("manifest_content_sha256")
        if declared != _self_hash(previous, "manifest_content_sha256"):
            raise ValueError("run manifest self-hash mismatch on resume")
        for key in (
            "roster_sha256",
            "jobs_sha256",
            "N_eval",
            "J_eval",
            "protocol_snapshot_sha256",
        ):
            if previous.get(key) != base_manifest.get(key):
                raise ValueError(f"resume configuration mismatch: {key}")
        if previous.get("source_bindings") != base_manifest.get("source_bindings"):
            raise ValueError("resume source bindings mismatch")
        # Keep the original immutable run manifest hash.  Recomputing it with
        # a new timestamp would invalidate every child record on resume.
        base_manifest = previous
    elif not resume and (records_path.exists() or old_manifest_path.exists()):
        raise ValueError("output already contains a run; use --resume")
    if resume and records_path.exists():
        for item in _load_jsonl(records_path):
            asset_id = str(item.get("asset_id", ""))
            if not asset_id or asset_id in records:
                raise ValueError("records contain duplicate or missing asset IDs")
            records[asset_id] = item
            persisted_ids.add(asset_id)
    by_id = {job["asset_id"]: job for job in jobs}
    if len(by_id) != len(jobs):
        raise ValueError("roster contains duplicate asset IDs")
    for asset_id, record in records.items():
        job = by_id.get(asset_id)
        if job is None:
            raise ValueError("records contain a foreign asset ID")
        _validate_record_for_job(record, job, base_manifest["manifest_content_sha256"])
    # A child may have finished just before a parent interruption.  Recover
    # only files carrying the exact job binding; stale child outputs are ignored.
    children_dir = output / "children"
    if resume and children_dir.is_dir():
        for job in jobs:
            asset_id = job["asset_id"]
            if asset_id in records:
                continue
            child_path = children_dir / f"{int(job['selection_index']):06d}.json"
            if not child_path.is_file():
                continue
            try:
                candidate = json.loads(child_path.read_text(encoding="utf-8"))
                _validate_record_for_job(candidate, job, base_manifest["manifest_content_sha256"])
            except Exception:
                continue
            records[asset_id] = candidate

    _atomic_json(old_manifest_path, base_manifest)
    if not records_path.exists():
        _atomic_write(records_path, "")

    def persist_batch(state: str = "running") -> None:
        # Append only records not already durable.  This keeps checkpoint cost
        # linear in completed assets instead of repeatedly rewriting the full
        # prefix of a large release.
        for job in jobs:
            asset_id = job["asset_id"]
            if asset_id in records and asset_id not in persisted_ids:
                _append_jsonl(records_path, records[asset_id])
                persisted_ids.add(asset_id)
        checkpoint: dict[str, Any] = {
            "schema_version": "table2sup_checkpoint_v1",
            "state": state,
            "records": len(records),
            "records_bytes": records_path.stat().st_size,
            "manifest_content_sha256": base_manifest["manifest_content_sha256"],
            "updated_at": _utc_now(),
        }
        checkpoint["checkpoint_content_sha256"] = _self_hash(
            checkpoint, "checkpoint_content_sha256"
        )
        _atomic_json(output / "checkpoint.json", checkpoint)

    # Persist any recovered child records before launching new work.
    persist_batch("running")
    pending = [job for job in jobs if job["asset_id"] not in records]
    bound_pending = [
        {**job, "run_manifest_content_sha256": base_manifest["manifest_content_sha256"]}
        for job in pending
    ]
    try:
        _run_subprocesses(
            bound_pending,
            output,
            workers=workers,
            timeout_seconds=timeout_seconds,
            records=records,
            on_batch=persist_batch,
        )
    except BaseException:
        persist_batch("interrupted")
        raise
    if len(records) != len(jobs):
        missing = [job["asset_id"] for job in jobs if job["asset_id"] not in records]
        raise RuntimeError(f"missing terminal records: {missing[:5]}")
    ordered = []
    for job in jobs:
        record = records[job["asset_id"]]
        _validate_record_for_job(record, job, base_manifest["manifest_content_sha256"])
        ordered.append(record)
    _atomic_write(
        records_path,
        "".join(json.dumps(item, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n" for item in ordered),
    )
    records_hash = _hash_file(records_path)
    summary = aggregate_records(ordered, n_eval, j_eval)
    summary.update(
        {
            "schema_version": "table2sup_full_release_summary_v1",
            "dataset": manifest.get("dataset", source_kind),
            "manifest_content_sha256": base_manifest["manifest_content_sha256"],
            "records_sha256": records_hash,
            "completed_at": _utc_now(),
            "source_bindings": source_bindings,
        }
    )
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    _atomic_json(output / "summary.json", summary)
    metrics = summary["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]["asset"]
    portability = metrics["joint_limit_portability"]
    dynamics = metrics["joint_dynamics_coverage"]
    def _percent(value: Any) -> str:
        return "N/E" if value is None else f"{float(value):.2f}%"

    summary_md = (
        f"# {manifest.get('dataset', source_kind)} Table 2 supplementary\n\n"
        f"- N_eval: {n_eval}\n- J_eval: {j_eval}\n"
        f"- Visual-bearing Collision Coverage: {visual['numerator']} / {visual['denominator']} ({_percent(visual['percent'])})\n"
        f"- Joint-limit Portability: {portability['numerator']} / {portability['denominator']} ({_percent(portability['percent'])})\n"
        f"- Joint Dynamics Coverage: {dynamics['numerator']} / {dynamics['denominator']} ({_percent(dynamics['percent'])})\n"
        "- Placeholder-mass Incidence: N/E (placeholder_registry_empty)\n\n"
        "Errors and timeouts remain in the frozen denominators.\n"
    )
    _atomic_write(output / "summary.md", summary_md)
    checkpoint: dict[str, Any] = {
        "schema_version": "table2sup_checkpoint_v1",
        "state": "complete",
        "records": len(ordered),
        "records_sha256": records_hash,
        "summary_sha256": _hash_file(output / "summary.json"),
        "manifest_content_sha256": base_manifest["manifest_content_sha256"],
        "completed_at": _utc_now(),
    }
    checkpoint["checkpoint_content_sha256"] = _self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    _atomic_json(output / "checkpoint.json", checkpoint)
    _atomic_json(output / "artifact_manifest.json", _artifact_manifest(output))
    return output


def write_combined_receipt(
    outputs: Mapping[str, Path | str] | Sequence[Path | str], root: Path | str
) -> Path:
    """Write a self-hashed combined receipt and return its path."""

    root_path = Path(root).resolve()
    receipt = build_combined_receipt(outputs, root_path)
    receipt["created_at"] = _utc_now()
    receipt["receipt_content_sha256"] = _self_hash(receipt, "receipt_content_sha256")
    path = root_path / "full_release_receipt.json"
    _atomic_json(path, receipt)
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=DATASETS)
    parser.add_argument("--roster", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--parts-zip", type=Path)
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--job", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--result", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.child:
        if args.job is None or args.result is None:
            parser.error("--child requires --job and --result")
        return _child_main(args.job, args.result)
    if args.dataset is None or args.roster is None or args.output is None:
        parser.error("--dataset, --roster and --output are required")
    try:
        run_dataset(args.roster, args.output, workers=args.workers, timeout_seconds=args.timeout_seconds, resume=args.resume, source_kind=args.dataset, parts_zip=args.parts_zip, source_root=args.source_root)
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr); return 1
    print(json.dumps({"status": "completed", "output": str(args.output.resolve())}, sort_keys=True)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
