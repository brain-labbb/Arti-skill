#!/usr/bin/env python3
"""Run Table 3 on the exact frozen Artiverse Table 1 cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from typing import Any
import uuid
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/artiverse"
DEFAULT_COHORT_MANIFEST = REPO_ROOT / "exp/runtime/table1_artiverse/manifest.json"
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "Artiverse"
FORMAL_N_RELEASE = 3544
FORMAL_N_EVAL = 800
FORMAL_CATEGORY_COUNT = 67
FORMAL_SEED = 20260813
FORMAL_COHORT_SHA256 = "f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c"
FORMAL_RELEASE_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
SELECTION_PROTOCOL = "artiverse-table1-global-sample-v1"
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0


def _load_core():
    spec = importlib.util.spec_from_file_location("urdf_table3_core", CORE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import Table 3 core: {CORE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load_core()


def _safe_relative(raw: str, *, field: str) -> Path:
    relative = Path(raw)
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        raise ValueError(f"unsafe {field}: {raw!r}")
    return relative


def _identity(raw: str) -> tuple[str, str, str]:
    parts = _safe_relative(raw, field="manifest_root").parts
    if len(parts) != 4 or parts[0] != "data" or any(not part for part in parts):
        raise ValueError(f"invalid Artiverse manifest_root: {raw!r}")
    return parts[1], parts[2], parts[3]


def _load_release_manifest(path: Path) -> tuple[dict[str, Any], dict[str, dict[str, str]]]:
    release = json.loads(path.read_text(encoding="utf-8"))
    chunks = release.get("chunks") if isinstance(release, dict) else None
    if not isinstance(chunks, list) or release.get("chunk_count") != len(chunks):
        raise ValueError("invalid Artiverse release manifest chunks")
    identities: dict[str, dict[str, str]] = {}
    total = 0
    for chunk in chunks:
        if not isinstance(chunk, dict) or not isinstance(chunk.get("roots"), list):
            raise ValueError("invalid Artiverse release manifest chunk")
        archive = chunk.get("archive")
        if not isinstance(archive, str) or not archive:
            raise ValueError("Artiverse release chunk archive is missing")
        roots = chunk["roots"]
        if chunk.get("model_count") != len(roots):
            raise ValueError("Artiverse release chunk model_count mismatch")
        total += len(roots)
        for raw_root in roots:
            if not isinstance(raw_root, str):
                raise ValueError("Artiverse release root is not a string")
            category, source, model_id = _identity(raw_root)
            if raw_root in identities:
                raise ValueError(f"duplicate Artiverse release root: {raw_root}")
            identities[raw_root] = {
                "raw_category": category,
                "source": source,
                "model_id": model_id,
                "chunk_archive": archive,
            }
    if release.get("model_count") != total:
        raise ValueError("Artiverse release model_count mismatch")
    return release, identities


def _declared_joint_hint(urdf_path: Path) -> int:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:  # noqa: BLE001
        return 0
    return sum(element.get("type", "") != "fixed" for element in root.findall("joint"))


def load_cohort(source_root: Path, cohort_manifest: Path, *, formal: bool) -> dict[str, Any]:
    source_root = source_root.resolve(strict=True)
    cohort_manifest = cohort_manifest.resolve(strict=True)
    cohort_hash = core.sha256_file(cohort_manifest)
    cohort = json.loads(cohort_manifest.read_text(encoding="utf-8"))
    if not isinstance(cohort, dict) or cohort.get("dataset") != DATASET_NAME:
        raise ValueError("cohort manifest dataset must be Artiverse")
    if cohort.get("release_status") != "PRE_RELEASE_SUBSET":
        raise ValueError("Artiverse cohort must be PRE_RELEASE_SUBSET")
    if cohort.get("cohort_type") != "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED":
        raise ValueError("unexpected Artiverse cohort type")
    if str(cohort.get("seed")) != str(FORMAL_SEED):
        raise ValueError(f"Artiverse cohort seed must be {FORMAL_SEED}")
    if cohort.get("selection_protocol") != SELECTION_PROTOCOL:
        raise ValueError("unexpected Artiverse selection protocol")
    if cohort.get("release_manifest") != "dataset_chunks/manifest.json":
        raise ValueError("unexpected Artiverse release manifest path")

    release_path = source_root / "dataset_chunks/manifest.json"
    if release_path.is_symlink() or not release_path.is_file():
        raise RuntimeError("Artiverse release manifest is missing or symlinked")
    release_hash = core.sha256_file(release_path)
    if cohort.get("release_manifest_sha256") != release_hash:
        raise ValueError("Artiverse cohort release manifest SHA256 mismatch")
    release, identities = _load_release_manifest(release_path)
    if cohort.get("N_release") != len(identities):
        raise ValueError("Artiverse cohort N_release mismatch")
    universe_hash = hashlib.sha256(
        "".join(f"{asset_id}\n" for asset_id in sorted(identities)).encode("utf-8")
    ).hexdigest()
    if cohort.get("release_universe_sha256") != universe_hash:
        raise ValueError("Artiverse release universe SHA256 mismatch")
    raw_assets = cohort.get("assets")
    if not isinstance(raw_assets, list) or cohort.get("N_eval") != len(raw_assets):
        raise ValueError("Artiverse cohort N_eval mismatch")

    if formal:
        checks = {
            "cohort manifest SHA256": (cohort_hash, FORMAL_COHORT_SHA256),
            "release manifest SHA256": (release_hash, FORMAL_RELEASE_MANIFEST_SHA256),
            "N_release": (len(identities), FORMAL_N_RELEASE),
            "N_eval": (len(raw_assets), FORMAL_N_EVAL),
        }
        for name, (observed, expected) in checks.items():
            if observed != expected:
                raise RuntimeError(f"formal Artiverse {name} mismatch: {observed!r} != {expected!r}")

    seen_ids: set[str] = set()
    seen_hashes: set[str] = set()
    assets: list[dict[str, Any]] = []
    for rank, raw in enumerate(raw_assets, start=1):
        if not isinstance(raw, dict):
            raise ValueError("Artiverse cohort asset is not an object")
        manifest_root = raw.get("manifest_root")
        if not isinstance(manifest_root, str) or raw.get("asset_id") != manifest_root:
            raise ValueError("Artiverse asset_id must equal manifest_root")
        if manifest_root in seen_ids:
            raise ValueError(f"duplicate Artiverse cohort asset: {manifest_root}")
        seen_ids.add(manifest_root)
        category, source, model_id = _identity(manifest_root)
        if (
            raw.get("raw_category") != category
            or raw.get("source") != source
            or raw.get("model_id") != model_id
        ):
            raise ValueError(f"Artiverse path metadata mismatch: {manifest_root}")
        if raw.get("selection_rank") != rank:
            raise ValueError(f"Artiverse selection rank mismatch: {manifest_root}")
        selection_hash = hashlib.sha256(
            "\0".join((SELECTION_PROTOCOL, release_hash, str(FORMAL_SEED), manifest_root)).encode("utf-8")
        ).hexdigest()
        if raw.get("selection_hash") != selection_hash or selection_hash in seen_hashes:
            raise ValueError(f"Artiverse selection hash mismatch: {manifest_root}")
        seen_hashes.add(selection_hash)
        release_identity = identities.get(manifest_root)
        if release_identity is None:
            raise ValueError(f"Artiverse cohort asset is absent from release: {manifest_root}")
        for field in ("raw_category", "source", "model_id", "chunk_archive"):
            if raw.get(field) != release_identity[field]:
                raise ValueError(f"Artiverse release identity mismatch for {field}: {manifest_root}")

        model_root = (source_root / _safe_relative(manifest_root, field="manifest_root")).resolve(strict=True)
        model_root.relative_to(source_root)
        package_raw = model_root / "urdf_w_collider"
        if package_raw.is_symlink() or not package_raw.is_dir():
            raise RuntimeError(f"Artiverse urdf_w_collider package is missing: {manifest_root}")
        package = package_raw.resolve(strict=True)
        package.relative_to(source_root)
        urdf_candidates = sorted(package.glob("*.urdf"), key=lambda path: path.name)
        if len(urdf_candidates) != 1:
            raise RuntimeError(f"Artiverse package must have exactly one top-level URDF: {manifest_root}")
        urdf_path = urdf_candidates[0]
        if urdf_path.is_symlink() or not urdf_path.is_file() or urdf_path.name != f"{model_id}.urdf":
            raise RuntimeError(f"Artiverse primary URDF identity mismatch: {manifest_root}")
        assets.append({
            "asset_key": manifest_root,
            "asset_id": manifest_root,
            "manifest_root": manifest_root,
            "raw_category": category,
            "category": category,
            "source": source,
            "model_id": model_id,
            "chunk_archive": raw["chunk_archive"],
            "selection_rank": rank,
            "selection_hash": selection_hash,
            "package": str(package),
            "urdf_path": str(urdf_path.resolve(strict=True)),
            "urdf_sha256": core.sha256_file(urdf_path),
            "declared_joint_count_hint": _declared_joint_hint(urdf_path),
        })
    if formal and len({row["category"] for row in assets}) != FORMAL_CATEGORY_COUNT:
        raise RuntimeError("formal Artiverse cohort must cover exactly 67 raw categories")
    return {
        "source_root": str(source_root),
        "cohort_manifest_path": str(cohort_manifest),
        "cohort_manifest_sha256": cohort_hash,
        "release_manifest_path": str(release_path.resolve(strict=True)),
        "release_manifest_sha256": release_hash,
        "release_universe_sha256": universe_hash,
        "release_status": cohort["release_status"],
        "cohort_type": cohort["cohort_type"],
        "selection_protocol": cohort["selection_protocol"],
        "seed": int(cohort["seed"]),
        "n_release": len(identities),
        "release_manifest": release,
        "assets": assets,
    }


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-artiverse-table1-cohort-v1",
        "samples_per_joint": args.samples,
        "continuous_interval": list(core.CONTINUOUS_INTERVAL),
        "continuous_non_degenerate_policy": "maximum_excursion_from_q0_over_frozen_interval",
        "q0_policy": "zero_clipped_to_declared_interval",
        "translation_motion_threshold_bbox_diagonal": core.TRANSLATION_MOTION_THRESHOLD,
        "rotation_motion_threshold_rad": core.ROTATION_MOTION_THRESHOLD_RAD,
        "unchanged_translation_tolerance_bbox_diagonal": core.UNCHANGED_TRANSLATION_TOLERANCE,
        "unchanged_rotation_tolerance_rad": core.UNCHANGED_ROTATION_TOLERANCE_RAD,
        "roundtrip_translation_tolerance_bbox_diagonal": core.ROUNDTRIP_TRANSLATION_TOLERANCE,
        "roundtrip_rotation_tolerance_rad": core.ROUNDTRIP_ROTATION_TOLERANCE_RAD,
        "strict_asset_requires_at_least_one_declared_movable_joint": True,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
    }


def build_manifest(args: argparse.Namespace, loaded: dict[str, Any]) -> dict[str, Any]:
    selected = loaded["assets"] if args.limit is None else loaded["assets"][: args.limit]
    config = _config(args)
    environment = core._environment()
    manifest = {
        "schema_version": 1,
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "created_at": core.utc_now(),
        "source": {
            "source_root": loaded["source_root"],
            "release_status": loaded["release_status"],
            "n_release": loaded["n_release"],
            "release_manifest_path": loaded["release_manifest_path"],
            "release_manifest_sha256": loaded["release_manifest_sha256"],
            "release_universe_sha256": loaded["release_universe_sha256"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_sha256": loaded["cohort_manifest_sha256"],
            "cohort_asset_count": len(loaded["assets"]),
        },
        "selection": {
            "algorithm": "exact existing Table 1 manifest .assets order; optional smoke prefix only",
            "source_protocol": loaded["selection_protocol"],
            "seed": loaded["seed"],
            "cohort_type": loaded["cohort_type"],
            "requested_limit": args.limit,
            "n_eval": len(selected),
            "selected_asset_ids_sha256": core.canonical_sha256([row["asset_key"] for row in selected]),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "protocol_path": str(PROTOCOL_PATH.resolve(strict=True)),
            "protocol_sha256": core.sha256_file(PROTOCOL_PATH.resolve(strict=True)),
            "adapter_path": str(SCRIPT_PATH),
            "adapter_sha256": core.sha256_file(SCRIPT_PATH),
            "core_evaluator_path": str(CORE_PATH.resolve(strict=True)),
            "core_evaluator_sha256": core.sha256_file(CORE_PATH.resolve(strict=True)),
            "config": config,
            "config_sha256": core.canonical_sha256(config),
            "environment": environment,
            "environment_sha256": core.canonical_sha256(environment),
        },
        "records": selected,
    }
    manifest["manifest_content_sha256"] = core._manifest_self_hash(manifest)
    return manifest


def _bind_record(record: dict[str, Any], job: dict[str, Any]) -> dict[str, Any]:
    record.update({
        "asset_id": job["asset_id"],
        "manifest_root": job["manifest_root"],
        "category": job["category"],
        "raw_category": job["raw_category"],
        "source": job["source"],
        "model_id": job["model_id"],
        "chunk_archive": job["chunk_archive"],
        "selection_rank": job["selection_rank"],
        "selection_hash": job["selection_hash"],
        "urdf_sha256": job["urdf_sha256"],
        "manifest_content_sha256": job["manifest_content_sha256"],
        "completed_at": core.utc_now(),
    })
    return record


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    urdf_path = Path(job["urdf_path"])
    if not urdf_path.is_file() or urdf_path.is_symlink():
        record = core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "selected URDF is missing"
        )
    elif core.sha256_file(urdf_path) != job["urdf_sha256"]:
        record = core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "selected URDF changed after freeze"
        )
    else:
        record = core.evaluate_urdf(
            urdf_path,
            job["asset_key"],
            samples=job["samples"],
            declared_joint_count_hint=job["declared_joint_count_hint"],
        )
        if core.sha256_file(urdf_path) != job["urdf_sha256"]:
            record = core.failed_record(
                job["asset_key"], job["declared_joint_count_hint"], "selected URDF changed during evaluation"
            )
    core.atomic_write_json(result_path, _bind_record(record, job))
    return 0


def _execute_job(job: dict[str, Any], scratch_root: Path, timeout_seconds: float) -> dict[str, Any]:
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch_root))
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    core.atomic_write_json(job_path, job)
    environment = dict(os.environ)
    environment.update({
        "OMP_NUM_THREADS": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
    })
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT_PATH), "--internal-job", str(job_path), "--internal-result", str(result_path)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
    )
    try:
        _stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
        return _bind_record(core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"],
            f"asset timeout after {timeout_seconds:g} seconds", status="timeout"
        ), job)
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-4000:]
        return _bind_record(core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"],
            f"worker failed with exit {process.returncode}: {detail}", status="error"
        ), job)
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("asset_key") != job["asset_key"]:
        return _bind_record(core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "worker result asset binding mismatch"
        ), job)
    return result


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# Artiverse Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact Table 1 manifest cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}, seed={manifest['selection']['seed']}; "
            f"J_eval={summary['j_eval']}. Existing manifest order was preserved without resampling."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in core.JOINT_RATE_METRICS:
        value = metrics[metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {value['passed']} / {value['denominator']} ({percentage}) |")
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} ({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    strict_percentage = "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    lines.append(
        f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} ({strict_percentage}) |"
    )
    macro = summary["category_macro"]
    lines.extend([
        "",
        (
            f"Category macro average over {macro['category_count']} observed raw categories "
            f"({macro['joint_metric_category_count']} with at least one declared movable joint):"
        ),
        "",
        "| Metric | Category macro |",
        "|---|---:|",
    ])
    for metric in (*core.JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = macro["metrics"][metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {percentage} (categories={value['category_count']}) |")
    lines.extend([
        "",
        "This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.",
        "",
    ])
    return "\n".join(lines)


def _prepare_output(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        timestamp = core.datetime.now(core.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = DEFAULT_OUTPUT_PARENT / f"urdf_table3_artiverse_table1_n{n_eval}_{timestamp}"
    output = args.output.resolve(strict=False)
    try:
        output.relative_to(REPO_ROOT.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"output must be inside repository: {output}") from exc
    if args.resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)
    return output


def run(args: argparse.Namespace) -> Path:
    validate_contract(args)
    loaded = load_cohort(args.source_root, args.cohort_manifest, formal=args.mode == "formal")
    n_eval = len(loaded["assets"]) if args.limit is None else args.limit
    output = _prepare_output(args, n_eval)
    manifest_path = output / "manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    scratch = output / ".worker_scratch"
    scratch.mkdir(exist_ok=True)
    if args.resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("manifest_content_sha256") != core._manifest_self_hash(manifest):
            raise RuntimeError("resume manifest self-hash mismatch")
        fresh = build_manifest(args, loaded)
        for field in ("source", "selection", "evaluation", "records"):
            if fresh[field] != manifest[field]:
                raise RuntimeError(f"resume binding mismatch: {field}")
    else:
        manifest = build_manifest(args, loaded)
        core.atomic_write_json(manifest_path, manifest)
        records_path.touch(exist_ok=False)
        core.atomic_write_json(checkpoint_path, {
            "state": "frozen", "completed": 0, "remaining": n_eval, "n_eval": n_eval,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": core.utc_now(),
        })
    records = core.load_jsonl(records_path)
    selected_keys = [row["asset_key"] for row in manifest["records"]]
    by_key: dict[str, dict[str, Any]] = {}
    for record in records:
        key = record.get("asset_key")
        if key not in selected_keys or key in by_key:
            raise RuntimeError(f"invalid or duplicate resume record: {key!r}")
        if record.get("manifest_content_sha256") != manifest["manifest_content_sha256"]:
            raise RuntimeError(f"resume record manifest binding mismatch: {key}")
        by_key[key] = record
    pending = [row for row in manifest["records"] if row["asset_key"] not in by_key]
    jobs = [{
        **row,
        "samples": args.samples,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
    } for row in pending]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(_execute_job, job, scratch, args.asset_timeout_seconds): job
            for job in jobs
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = _bind_record(core.failed_record(
                    job["asset_key"], job["declared_joint_count_hint"],
                    f"parent worker exception: {type(exc).__name__}: {exc}"
                ), job)
            by_key[record["asset_key"]] = record
            core.append_jsonl(records_path, record)
            completed = len(by_key)
            core.atomic_write_json(checkpoint_path, {
                "state": "running" if completed < n_eval else "aggregating",
                "completed": completed,
                "remaining": n_eval - completed,
                "n_eval": n_eval,
                "last_completed_asset_key": record["asset_key"],
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": core.utc_now(),
            })
            print(f"[{completed}/{n_eval}] {record['asset_key']} {record['status']}", flush=True)
    ordered = [by_key[key] for key in selected_keys]
    summary = core.aggregate_records(ordered, n_eval)
    summary.update({
        "schema_version": 1,
        "status": "completed",
        "classification": manifest["classification"],
        "dataset": DATASET_NAME,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed_at": core.utc_now(),
    })
    core.atomic_write_json(output / "summary.json", summary)
    core.atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    core.atomic_write_json(checkpoint_path, {
        "state": "complete", "completed": n_eval, "remaining": 0, "n_eval": n_eval,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "updated_at": core.utc_now(),
    })
    return output


def validate_contract(args: argparse.Namespace) -> None:
    if args.samples < 2 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("samples must be >=2 and workers/timeout must be positive")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError("limit must be in [1, 800]")
    if args.mode == "formal":
        if (
            args.limit is not None
            or args.samples != DEFAULT_SAMPLES
            or args.workers != DEFAULT_WORKERS
            or args.asset_timeout_seconds != DEFAULT_TIMEOUT_SECONDS
            or args.source_root.resolve(strict=False) != DEFAULT_SOURCE_ROOT.resolve(strict=False)
            or args.cohort_manifest.resolve(strict=False) != DEFAULT_COHORT_MANIFEST.resolve(strict=False)
        ):
            raise ValueError("formal mode freezes canonical source/cohort, N=800, K=21, workers=4, timeout=120")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--limit", type=int, help="smoke mode: evaluate exact manifest prefix")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--internal-job", type=Path)
    parser.add_argument("--internal-result", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_job is not None or args.internal_result is not None:
        if args.internal_job is None or args.internal_result is None:
            raise ValueError("internal job mode requires both paths")
        return run_internal_job(args.internal_job, args.internal_result)
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
