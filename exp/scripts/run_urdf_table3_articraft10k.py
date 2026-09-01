#!/usr/bin/env python3
"""Run Table 3 on the exact frozen Articraft-10K Table 2 cohort."""

from __future__ import annotations

import argparse
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
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_DATASET_ROOT = REPO_ROOT / "exp/Articraft-10K"
DEFAULT_COHORT_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
DEFAULT_CATEGORY_RECORDS_ROOT = (
    REPO_ROOT / "exp/baselines/Articraft-10K-official/records"
)
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "Articraft-10K"
FORMAL_N_RELEASE = 9996
FORMAL_N_EVAL = 800
FORMAL_RELEASE_CATEGORY_COUNT = 240
FORMAL_EVAL_CATEGORY_COUNT = 222
FORMAL_SEED = 20260813
FORMAL_COHORT_FILE_SHA256 = (
    "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
)
FORMAL_COHORT_CONTENT_SHA256 = (
    "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
)
FORMAL_SELECTED_ASSET_IDS_SHA256 = (
    "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784"
)
FORMAL_RELEASE_ASSET_IDS_SHA256 = (
    "a52fab1cc35e9948ea75e5a7cb4e99408ae39a2e5d27eca3a479c24d5c9606ff"
)
FORMAL_SELECTED_CATEGORY_MAPPING_SHA256 = (
    "0305569f49d2aa1acb72fbb7bc8dcaf68ca3dd4a5bd7eba140b5bac4c8c0f449"
)
FORMAL_CATEGORY_RECORDS_REVISION = "677ca9722427dce500873730255874c8c3f07eb2"
SELECTION_PROTOCOL = "exact-table2-record-package-order-v1"
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


def _declared_joint_hint(urdf_path: Path) -> int:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:  # noqa: BLE001
        return 0
    return sum(element.get("type", "") != "fixed" for element in root.findall("joint"))


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": core.sha256_file(canonical),
                }
            )
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": core.canonical_sha256(files),
    }


def _category_revision(records_root: Path) -> str | None:
    checkout = records_root.parent
    result = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def _read_category(records_root: Path, asset_id: str) -> str:
    path = records_root / asset_id / "record.json"
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"category record is unavailable or invalid: {asset_id}") from exc
    if record.get("record_id") != asset_id:
        raise ValueError(f"category record identity mismatch: {asset_id}")
    category = str(record.get("category_slug", "")).strip()
    if not category:
        raise ValueError(f"category_slug is missing: {asset_id}")
    return category


def load_cohort(
    dataset_root: Path,
    cohort_manifest: Path,
    category_records_root: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    release_root = (dataset_root / "released_urdf").resolve(strict=True)
    category_records_root = category_records_root.resolve(strict=True)
    cohort_manifest = cohort_manifest.resolve(strict=True)
    cohort_file_hash = core.sha256_file(cohort_manifest)
    cohort = json.loads(cohort_manifest.read_text(encoding="utf-8"))
    if cohort.get("manifest_content_sha256") != core._manifest_self_hash(cohort):
        raise ValueError("Table 2 cohort manifest self-hash mismatch")
    if (
        cohort.get("dataset") != DATASET_NAME
        or cohort.get("classification") != "FORMAL"
        or cohort.get("mode") != "formal"
    ):
        raise ValueError("cohort manifest must be the formal Articraft-10K Table 2 run")
    source = cohort.get("source", {})
    if Path(str(source.get("root", ""))).resolve(strict=True) != release_root:
        raise ValueError("Table 2 source root does not match the dataset release root")
    release_entries = list(release_root.iterdir())
    if any(path.is_symlink() for path in release_entries):
        raise ValueError("Articraft-10K release root contains a symlink child")
    release_ids = sorted(path.name for path in release_entries if path.is_dir())
    if len(release_ids) != source.get("release_asset_count"):
        raise ValueError("release asset count differs from the Table 2 manifest")
    if core.canonical_sha256(release_ids) != source.get("release_asset_ids_sha256"):
        raise ValueError("release asset universe differs from the Table 2 manifest")
    raw_records = cohort.get("records")
    selection = cohort.get("selection", {})
    if not isinstance(raw_records, list) or selection.get("n_eval") != len(raw_records):
        raise ValueError("Table 2 cohort record count mismatch")
    asset_ids = [str(row.get("asset_id", "")) for row in raw_records]
    if not all(asset_ids) or len(set(asset_ids)) != len(asset_ids):
        raise ValueError("Table 2 cohort asset IDs must be unique and nonempty")
    if core.canonical_sha256(asset_ids) != selection.get("selected_asset_ids_sha256"):
        raise ValueError("Table 2 selected asset ID hash mismatch")
    if selection.get("selection_order_preserved") is not True:
        raise ValueError("Table 2 cohort does not preserve selection order")
    if selection.get("outcome_based_reselection") is not False:
        raise ValueError("Table 2 cohort permits outcome-based reselection")

    if formal:
        checks = {
            "cohort file SHA256": (cohort_file_hash, FORMAL_COHORT_FILE_SHA256),
            "cohort content SHA256": (
                cohort["manifest_content_sha256"],
                FORMAL_COHORT_CONTENT_SHA256,
            ),
            "release assets": (len(release_ids), FORMAL_N_RELEASE),
            "cohort assets": (len(raw_records), FORMAL_N_EVAL),
            "seed": (selection.get("seed"), FORMAL_SEED),
            "selected asset IDs": (
                selection.get("selected_asset_ids_sha256"),
                FORMAL_SELECTED_ASSET_IDS_SHA256,
            ),
            "release asset IDs": (
                source.get("release_asset_ids_sha256"),
                FORMAL_RELEASE_ASSET_IDS_SHA256,
            ),
        }
        for name, (observed, expected) in checks.items():
            if observed != expected:
                raise RuntimeError(
                    f"formal Articraft-10K {name} mismatch: {observed!r} != {expected!r}"
                )

    assets: list[dict[str, Any]] = []
    category_rows: list[dict[str, str]] = []
    seen_packages: set[str] = set()
    for index, raw in enumerate(raw_records):
        if not isinstance(raw, dict) or raw.get("selection_index") != index:
            raise ValueError(f"Table 2 selection index mismatch at {index}")
        asset_id = asset_ids[index]
        if "/" in asset_id or "\\" in asset_id or asset_id in {".", ".."}:
            raise ValueError(f"invalid Articraft-10K asset ID: {asset_id!r}")
        expected_package = release_root / asset_id
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        if package != expected_package.resolve(strict=True) or package.name != asset_id:
            raise ValueError(f"Table 2 package path mismatch: {asset_id}")
        package_text = str(package)
        if package_text in seen_packages:
            raise ValueError(f"duplicate Table 2 package: {package}")
        seen_packages.add(package_text)
        binding = _package_binding(package)
        if binding != raw.get("package_binding"):
            raise ValueError(f"Table 2 package binding mismatch: {asset_id}")
        urdf_path = package / "model.urdf"
        if urdf_path.is_symlink() or not urdf_path.is_file():
            raise ValueError(f"model.urdf is missing or symlinked: {asset_id}")
        urdf_hash = core.sha256_file(urdf_path)
        if urdf_hash != raw.get("model_urdf_sha256"):
            raise ValueError(f"model.urdf hash mismatch: {asset_id}")
        category = _read_category(category_records_root, asset_id)
        category_rows.append({"asset_id": asset_id, "category_slug": category})
        selection_hash = hashlib.sha256(
            "\0".join(
                (SELECTION_PROTOCOL, cohort_file_hash, str(index), package_text)
            ).encode("utf-8")
        ).hexdigest()
        assets.append(
            {
                "asset_key": asset_id,
                "asset_id": asset_id,
                "raw_category": category,
                "category": category,
                "selection_index": index,
                "selection_rank": index + 1,
                "selection_hash": selection_hash,
                "package": package_text,
                "package_binding": binding,
                "package_content_manifest_sha256": binding[
                    "content_manifest_sha256"
                ],
                "urdf_path": str(urdf_path.resolve(strict=True)),
                "urdf_sha256": urdf_hash,
                "declared_joint_count_hint": _declared_joint_hint(urdf_path),
            }
        )
    category_mapping_hash = core.canonical_sha256(category_rows)
    eval_category_count = len({row["category"] for row in assets})
    category_revision = _category_revision(category_records_root)
    if formal:
        if category_mapping_hash != FORMAL_SELECTED_CATEGORY_MAPPING_SHA256:
            raise RuntimeError("formal selected category mapping SHA256 mismatch")
        if eval_category_count != FORMAL_EVAL_CATEGORY_COUNT:
            raise RuntimeError("formal cohort must cover exactly 222 categories")
        if category_revision != FORMAL_CATEGORY_RECORDS_REVISION:
            raise RuntimeError("formal category-record revision mismatch")
    return {
        "dataset_root": str(dataset_root),
        "release_root": str(release_root),
        "cohort_manifest_path": str(cohort_manifest),
        "cohort_manifest_file_sha256": cohort_file_hash,
        "cohort_manifest_content_sha256": cohort["manifest_content_sha256"],
        "source_repo_id": source.get("repo_id"),
        "source_revision": source.get("revision"),
        "release_asset_ids_sha256": source.get("release_asset_ids_sha256"),
        "category_records_root": str(category_records_root),
        "category_records_revision": category_revision,
        "category_mapping_sha256": category_mapping_hash,
        "release_category_count": FORMAL_RELEASE_CATEGORY_COUNT if formal else None,
        "eval_category_count": eval_category_count,
        "selection_algorithm": selection.get("algorithm"),
        "selected_asset_ids_sha256": selection.get("selected_asset_ids_sha256"),
        "seed": int(selection.get("seed")),
        "n_release": len(release_ids),
        "assets": assets,
    }


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-articraft10k-table2-cohort-v1",
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
            "dataset_root": loaded["dataset_root"],
            "release_root": loaded["release_root"],
            "n_release": loaded["n_release"],
            "release_asset_ids_sha256": loaded["release_asset_ids_sha256"],
            "source_repo_id": loaded["source_repo_id"],
            "source_revision": loaded["source_revision"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_file_sha256": loaded[
                "cohort_manifest_file_sha256"
            ],
            "cohort_manifest_content_sha256": loaded[
                "cohort_manifest_content_sha256"
            ],
            "cohort_asset_count": len(loaded["assets"]),
            "category_records_root": loaded["category_records_root"],
            "category_records_revision": loaded["category_records_revision"],
            "category_mapping_policy": "exact asset_id join to official record.json category_slug",
            "category_mapping_sha256": loaded["category_mapping_sha256"],
            "release_category_count": loaded["release_category_count"],
            "eval_category_count": loaded["eval_category_count"],
        },
        "selection": {
            "algorithm": "exact existing Table 2 manifest .records[].package order; optional smoke prefix only",
            "source_algorithm": loaded["selection_algorithm"],
            "source_protocol": SELECTION_PROTOCOL,
            "seed": loaded["seed"],
            "cohort_type": "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED",
            "requested_limit": args.limit,
            "n_eval": len(selected),
            "source_selected_asset_ids_sha256": loaded[
                "selected_asset_ids_sha256"
            ],
            "selected_asset_ids_sha256": core.canonical_sha256(
                [row["asset_id"] for row in selected]
            ),
            "selected_packages_sha256": core.canonical_sha256(
                [row["package"] for row in selected]
            ),
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
    record.update(
        {
            "asset_id": job["asset_id"],
            "category": job["category"],
            "raw_category": job["raw_category"],
            "selection_index": job["selection_index"],
            "selection_rank": job["selection_rank"],
            "selection_hash": job["selection_hash"],
            "package": job["package"],
            "package_content_manifest_sha256": job[
                "package_content_manifest_sha256"
            ],
            "urdf_sha256": job["urdf_sha256"],
            "manifest_content_sha256": job["manifest_content_sha256"],
            "completed_at": core.utc_now(),
        }
    )
    return record


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    urdf_path = Path(job["urdf_path"])
    package = Path(job["package"])
    failure: str | None = None
    if not urdf_path.is_file() or urdf_path.is_symlink():
        failure = "selected URDF is missing"
    elif core.sha256_file(urdf_path) != job["urdf_sha256"]:
        failure = "selected URDF changed after freeze"
    elif _package_binding(package) != job["package_binding"]:
        failure = "selected package changed after freeze"
    if failure is not None:
        record = core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], failure
        )
    else:
        record = core.evaluate_urdf(
            urdf_path,
            job["asset_key"],
            samples=job["samples"],
            declared_joint_count_hint=job["declared_joint_count_hint"],
        )
        if (
            core.sha256_file(urdf_path) != job["urdf_sha256"]
            or _package_binding(package) != job["package_binding"]
        ):
            record = core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                "selected package changed during evaluation",
            )
    core.atomic_write_json(result_path, _bind_record(record, job))
    return 0


def _execute_job(
    job: dict[str, Any], scratch_root: Path, timeout_seconds: float
) -> dict[str, Any]:
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch_root))
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    core.atomic_write_json(job_path, job)
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
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                f"asset timeout after {timeout_seconds:g} seconds",
                status="timeout",
            ),
            job,
        )
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-4000:]
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                f"worker failed with exit {process.returncode}: {detail}",
                status="error",
            ),
            job,
        )
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("asset_key") != job["asset_key"]:
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                "worker result asset binding mismatch",
            ),
            job,
        )
    return result


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# Articraft-10K Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact Table 2 manifest cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}, seed={manifest['selection']['seed']}; "
            f"J_eval={summary['j_eval']}. Existing package order was preserved without resampling."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in core.JOINT_RATE_METRICS:
        value = metrics[metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(
            f"| {metric} | {value['passed']} / {value['denominator']} ({percentage}) |"
        )
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} "
        f"({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    strict_percentage = (
        "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    )
    lines.append(
        f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} "
        f"({strict_percentage}) |"
    )
    macro = summary["category_macro"]
    lines.extend(
        [
            "",
            (
                f"Category macro average over {macro['category_count']} observed raw categories "
                f"({macro['joint_metric_category_count']} with at least one declared movable joint):"
            ),
            "",
            "| Metric | Category macro |",
            "|---|---:|",
        ]
    )
    for metric in (*core.JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = macro["metrics"][metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(
            f"| {metric} | {percentage} (categories={value['category_count']}) |"
        )
    lines.extend(
        [
            "",
            "This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.",
            "",
        ]
    )
    return "\n".join(lines)


def _prepare_output(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        timestamp = core.datetime.now(core.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = (
            DEFAULT_OUTPUT_PARENT
            / f"urdf_table3_articraft10k_table2_n{n_eval}_{timestamp}"
        )
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
    loaded = load_cohort(
        args.dataset_root,
        args.cohort_manifest,
        args.category_records_root,
        formal=args.mode == "formal",
    )
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
        core.atomic_write_json(
            checkpoint_path,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": n_eval,
                "n_eval": n_eval,
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": core.utc_now(),
            },
        )
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
    jobs = [
        {
            **row,
            "samples": args.samples,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        }
        for row in pending
    ]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                _execute_job, job, scratch, args.asset_timeout_seconds
            ): job
            for job in jobs
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = _bind_record(
                    core.failed_record(
                        job["asset_key"],
                        job["declared_joint_count_hint"],
                        f"parent worker exception: {type(exc).__name__}: {exc}",
                    ),
                    job,
                )
            by_key[record["asset_key"]] = record
            core.append_jsonl(records_path, record)
            completed = len(by_key)
            core.atomic_write_json(
                checkpoint_path,
                {
                    "state": "running" if completed < n_eval else "aggregating",
                    "completed": completed,
                    "remaining": n_eval - completed,
                    "n_eval": n_eval,
                    "last_completed_asset_key": record["asset_key"],
                    "manifest_content_sha256": manifest["manifest_content_sha256"],
                    "updated_at": core.utc_now(),
                },
            )
            print(
                f"[{completed}/{n_eval}] {record['asset_key']} {record['status']}",
                flush=True,
            )
    ordered = [by_key[key] for key in selected_keys]
    summary = core.aggregate_records(ordered, n_eval)
    summary.update(
        {
            "schema_version": 1,
            "status": "completed",
            "classification": manifest["classification"],
            "dataset": DATASET_NAME,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed_at": core.utc_now(),
        }
    )
    core.atomic_write_json(output / "summary.json", summary)
    core.atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    core.atomic_write_json(
        checkpoint_path,
        {
            "state": "complete",
            "completed": n_eval,
            "remaining": 0,
            "n_eval": n_eval,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": core.utc_now(),
        },
    )
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
            or args.dataset_root.resolve(strict=False)
            != DEFAULT_DATASET_ROOT.resolve(strict=False)
            or args.cohort_manifest.resolve(strict=False)
            != DEFAULT_COHORT_MANIFEST.resolve(strict=False)
            or args.category_records_root.resolve(strict=False)
            != DEFAULT_CATEGORY_RECORDS_ROOT.resolve(strict=False)
        ):
            raise ValueError(
                "formal mode freezes canonical dataset/cohort/categories, N=800, "
                "K=21, workers=4, timeout=120"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument(
        "--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST
    )
    parser.add_argument(
        "--category-records-root", type=Path, default=DEFAULT_CATEGORY_RECORDS_ROOT
    )
    parser.add_argument("--limit", type=int, help="smoke mode: evaluate exact manifest prefix")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument(
        "--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
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
