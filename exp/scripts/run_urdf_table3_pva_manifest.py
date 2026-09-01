#!/usr/bin/env python3
"""Run Table 3 kinematic executability on the frozen PV-A cohort.

This is a cohort adapter around ``run_urdf_table3_ours_500k.py``.  The child
process protocol and all FK/metric calculations remain shared with that
runner; this module only binds the PV-A per-category N=5 packages and the
existing Table 1/Table 2 receipts.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = REPO_ROOT / "exp/scripts/run_urdf_table3_lam.py"
BASE_RUNNER_PATH = REPO_ROOT / "exp/scripts/run_urdf_table3_ours_500k.py"
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_COHORT_MANIFEST = REPO_ROOT / "exp/PV-A-per-class-n5-max-joints/manifest.json"
DEFAULT_TABLE1_RECEIPT = REPO_ROOT / "exp/runtime/table1_pva_per_class_n5_max_joints"
DEFAULT_TABLE2_RECEIPT = REPO_ROOT / "exp/runtime/table2_pva_per_class_n5_max_joints"
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "PV-A-per-class-n5-max-joints"
FORMAL_N_RELEASE = 302_440
FORMAL_N_EVAL = 2_655
FORMAL_RELEASE_CATEGORY_COUNT = 531
COHORT_TYPE = "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES"
SELECTION_PROTOCOL = "exact-frozen-pva-per-class-n5-max-joints-manifest-order-v1"
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0

# These are the immutable inputs used by the formal run.  Keeping the hashes
# here makes an accidental substitution of a similarly named receipt fail
# before any worker is started.
FORMAL_COHORT_FILE_SHA256 = (
    "e78f4b767023f8a5c1517d96bfab35a39482d6eee28238820a9b91ac3ea8d293"
)
FORMAL_COHORT_CONTENT_SHA256 = (
    "eea55287dd70b710a7c03b11b16c6685208bbaa63cde925232293cb9012c8158"
)
FORMAL_TABLE1_FILE_SHA256 = (
    "4b0360a398a5efba3532e9ca87c37bbedc5a3416783679f9f26f89e930e19644"
)
FORMAL_TABLE2_FILE_SHA256 = (
    "97b7df486b75f961978bc86eb7a2985cb8e40eba63336ed7531700a4ff234471"
)
FORMAL_TABLE2_CONTENT_SHA256 = (
    "afff3beb6ba320ccb4855ecb380de47b6de7ca91fdc77f2ba071a53944e03c14"
)


def _load_base():
    spec = importlib.util.spec_from_file_location("urdf_table3_pva_base", BASE_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import shared Table 3 runner: {BASE_RUNNER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = _load_base()
core = BASE.core


def sha256_file(path: Path) -> str:
    return core.sha256_file(path)


def canonical_sha256(value: Any) -> str:
    return core.canonical_sha256(value)


def _self_hash(payload: dict[str, Any]) -> str:
    return core._manifest_self_hash(payload)


def _declared_joint_hint(urdf_path: Path) -> int:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:  # noqa: BLE001
        return 0
    return sum(element.get("type", "") != "fixed" for element in root.findall("joint"))


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _validate_package(row: dict[str, Any], *, live: bool) -> tuple[Path, dict[str, Any], str]:
    package_raw = row.get("package")
    if not isinstance(package_raw, str) or not Path(package_raw).is_absolute():
        raise ValueError(f"package path is not absolute: {package_raw!r}")
    package = Path(package_raw).resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    if row.get("primary_urdf_relative_path", "model.urdf") != "model.urdf":
        raise ValueError("PV-A primary URDF must be model.urdf")
    binding = row.get("package_binding")
    if not isinstance(binding, dict):
        raise ValueError("PV-A package_binding is missing")
    urdf = package / "model.urdf"
    if urdf.is_symlink() or not urdf.is_file():
        raise ValueError(f"PV-A model.urdf is missing or symlinked: {package}")
    expected = str(row.get("urdf_sha256") or row.get("primary_urdf_sha256") or "")
    if len(expected) != 64:
        raise ValueError("PV-A URDF SHA-256 is missing")
    if live:
        observed_urdf = sha256_file(urdf)
        if observed_urdf != expected:
            raise ValueError(f"PV-A URDF hash mismatch: {package}")
        observed_binding = BASE._package_binding(package)
        if observed_binding != binding:
            raise ValueError(f"PV-A package binding mismatch: {package}")
        return package, observed_binding, observed_urdf
    return package, binding, expected


def _compare_receipt_rows(
    source_assets: list[dict[str, Any]], table1: dict[str, Any], table2: dict[str, Any]
) -> None:
    """Verify that both prior formal receipts bind exactly this asset order."""

    table1_assets = table1.get("assets")
    table2_assets = table2.get("assets")
    if not isinstance(table1_assets, list) or len(table1_assets) != len(source_assets):
        raise ValueError("Table 1 receipt asset count does not match PV-A cohort")
    if not isinstance(table2_assets, list) or len(table2_assets) != len(source_assets):
        raise ValueError("Table 2 receipt asset count does not match PV-A cohort")
    for index, source in enumerate(source_assets):
        t1 = table1_assets[index]
        t2 = table2_assets[index]
        if t1.get("selection_index") != index or t2.get("selection_index") != index:
            raise ValueError(f"receipt selection order mismatch at {index}")
        for receipt, label in ((t1, "Table 1"), (t2, "Table 2")):
            if receipt.get("dataset_id") != source.get("dataset_id"):
                raise ValueError(f"{label} dataset identity mismatch at {index}")
            if receipt.get("package") != source.get("package"):
                raise ValueError(f"{label} package mismatch at {index}")
            if receipt.get("package_binding") != source.get("package_binding"):
                raise ValueError(f"{label} package binding mismatch at {index}")
            expected_urdf = source.get("urdf_sha256")
            observed_urdf = receipt.get("primary_urdf_sha256", receipt.get("urdf_sha256"))
            if observed_urdf != expected_urdf:
                raise ValueError(f"{label} URDF hash mismatch at {index}")
            category = receipt.get("raw_category", receipt.get("category"))
            if category != source.get("category"):
                raise ValueError(f"{label} category mismatch at {index}")


def _validate_formal_receipts(
    source: dict[str, Any], source_assets: list[dict[str, Any]]
) -> dict[str, Any]:
    if sha256_file(DEFAULT_TABLE1_RECEIPT / "manifest.json") != FORMAL_TABLE1_FILE_SHA256:
        raise RuntimeError("formal PV-A Table 1 receipt hash mismatch")
    table1 = _load_json(DEFAULT_TABLE1_RECEIPT / "manifest.json")
    if table1.get("schema_version") != "table1_pva_manifest_v1":
        raise ValueError("unexpected PV-A Table 1 receipt schema")
    if table1.get("source_manifest_sha256") != source["file_sha256"]:
        raise ValueError("Table 1 source manifest file hash mismatch")
    if table1.get("source_manifest_content_sha256") != source["content_sha256"]:
        raise ValueError("Table 1 source manifest content hash mismatch")

    if sha256_file(DEFAULT_TABLE2_RECEIPT / "manifest.json") != FORMAL_TABLE2_FILE_SHA256:
        raise RuntimeError("formal PV-A Table 2 receipt hash mismatch")
    table2 = _load_json(DEFAULT_TABLE2_RECEIPT / "manifest.json")
    if table2.get("manifest_content_sha256") != FORMAL_TABLE2_CONTENT_SHA256:
        raise ValueError("PV-A Table 2 receipt self-hash mismatch")
    if _self_hash(table2) != table2.get("manifest_content_sha256"):
        raise ValueError("PV-A Table 2 receipt computed self-hash mismatch")
    if table2.get("source", {}).get("cohort_manifest_sha256") != source["file_sha256"]:
        raise ValueError("Table 2 source manifest file hash mismatch")
    if table2.get("source", {}).get("cohort_manifest_content_sha256") != source["content_sha256"]:
        raise ValueError("Table 2 source manifest content hash mismatch")
    _compare_receipt_rows(source_assets, table1, table2)
    return {
        "table1_receipt": str(DEFAULT_TABLE1_RECEIPT.resolve(strict=True)),
        "table1_manifest_sha256": sha256_file(DEFAULT_TABLE1_RECEIPT / "manifest.json"),
        "table1_asset_records_sha256": sha256_file(DEFAULT_TABLE1_RECEIPT / "asset_records.jsonl"),
        "table2_receipt": str(DEFAULT_TABLE2_RECEIPT.resolve(strict=True)),
        "table2_manifest_sha256": sha256_file(DEFAULT_TABLE2_RECEIPT / "manifest.json"),
        "table2_manifest_content_sha256": table2["manifest_content_sha256"],
        "table2_asset_records_sha256": sha256_file(DEFAULT_TABLE2_RECEIPT / "asset_records.jsonl"),
    }


def load_cohort(cohort_manifest: Path, *, formal: bool) -> dict[str, Any]:
    """Load the frozen PV-A manifest and bind every package before evaluation."""

    path = cohort_manifest.resolve(strict=True)
    file_hash = sha256_file(path)
    manifest = _load_json(path)
    content_hash = manifest.get("manifest_content_sha256")
    if content_hash != _self_hash(manifest):
        raise ValueError("PV-A cohort manifest self-hash mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or not assets:
        raise ValueError("PV-A cohort assets are missing")
    if formal:
        checks = {
            "cohort file hash": (file_hash, FORMAL_COHORT_FILE_SHA256),
            "cohort content hash": (content_hash, FORMAL_COHORT_CONTENT_SHA256),
            "asset count": (len(assets), FORMAL_N_EVAL),
            "n_eval": (manifest.get("n_eval"), FORMAL_N_EVAL),
            "class count": (manifest.get("class_count"), FORMAL_RELEASE_CATEGORY_COUNT),
            "per-class count": (manifest.get("per_class"), 5),
            "dataset": (manifest.get("dataset"), "PV-A-per-class-n5"),
            "protocol": (manifest.get("protocol_id"), "pva-per-class-n5-fence-ferris-max-movable-joints-v1"),
        }
        for label, (observed, expected) in checks.items():
            if observed != expected:
                raise RuntimeError(f"formal PV-A {label} mismatch: {observed!r} != {expected!r}")

    normalized: list[dict[str, Any]] = []
    identifiers: set[str] = set()
    categories: Counter[str] = Counter()
    for index, raw in enumerate(assets):
        if not isinstance(raw, dict) or raw.get("selection_index") != index:
            raise ValueError(f"PV-A selection index mismatch at {index}")
        dataset_id = str(raw.get("dataset_id", ""))
        category = str(raw.get("category", "")).strip()
        if not dataset_id or not category or dataset_id in identifiers:
            raise ValueError(f"PV-A asset identity/category invalid at {index}")
        identifiers.add(dataset_id)
        categories[category] += 1
        package, binding, urdf_hash = _validate_package(raw, live=True)
        normalized.append(
            {
                "asset_key": dataset_id,
                "asset_id": dataset_id,
                "dataset_id": dataset_id,
                "source_asset_id": str(raw.get("asset_id", "")),
                "seed_name": str(raw.get("asset_id", "")),
                "raw_category": category,
                "category": category,
                "selection_index": index,
                "selection_rank": index + 1,
                "selection_hash": hashlib.sha256(
                    "\0".join((SELECTION_PROTOCOL, file_hash, str(index), str(package))).encode()
                ).hexdigest(),
                "package": str(package),
                "package_binding": binding,
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
                "primary_urdf_relative_path": "model.urdf",
                "urdf_path": str((package / "model.urdf").resolve(strict=True)),
                "urdf_sha256": urdf_hash,
                "declared_joint_count_hint": _declared_joint_hint(package / "model.urdf"),
            }
        )
    if formal:
        if len(categories) != FORMAL_RELEASE_CATEGORY_COUNT or set(categories.values()) != {5}:
            raise RuntimeError("formal PV-A category/per-class coverage mismatch")
        receipt = _validate_formal_receipts(
            {"file_sha256": file_hash, "content_sha256": content_hash}, assets
        )
    else:
        receipt = {}
    return {
        "cohort_manifest_path": str(path),
        "cohort_manifest_file_sha256": file_hash,
        "cohort_manifest_content_sha256": content_hash,
        "dataset_root": str(manifest.get("source_cohort_manifest", "")),
        "table1_receipt": receipt.get("table1_receipt", str(DEFAULT_TABLE1_RECEIPT)),
        "table1_manifest_sha256": receipt.get("table1_manifest_sha256"),
        "table1_asset_records_sha256": receipt.get("table1_asset_records_sha256"),
        "table2_receipt": receipt.get("table2_receipt", str(DEFAULT_TABLE2_RECEIPT)),
        "table2_manifest_sha256": receipt.get("table2_manifest_sha256"),
        "table2_manifest_content_sha256": receipt.get("table2_manifest_content_sha256"),
        "table2_asset_records_sha256": receipt.get("table2_asset_records_sha256"),
        "cohort_type": COHORT_TYPE,
        "n_release": int(manifest.get("n_release", FORMAL_N_RELEASE)),
        "release_category_count": int(manifest.get("class_count", len(categories))),
        "eval_category_count": len(categories),
        "assets": normalized,
    }


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-pva-per-class-n5-max-joints-v1",
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
    manifest: dict[str, Any] = {
        "schema_version": "urdf-table3-pva-manifest-v1",
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "created_at": core.utc_now(),
        "source": {
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_file_sha256": loaded["cohort_manifest_file_sha256"],
            "cohort_manifest_content_sha256": loaded["cohort_manifest_content_sha256"],
            "cohort_asset_count": len(loaded["assets"]),
            "dataset_root": loaded["dataset_root"],
            "n_release": loaded["n_release"],
            "cohort_type": loaded["cohort_type"],
            "release_category_count": loaded["release_category_count"],
            "eval_category_count": loaded["eval_category_count"],
            "table1_receipt": loaded["table1_receipt"],
            "table1_manifest_sha256": loaded["table1_manifest_sha256"],
            "table1_asset_records_sha256": loaded["table1_asset_records_sha256"],
            "table2_receipt": loaded["table2_receipt"],
            "table2_manifest_sha256": loaded["table2_manifest_sha256"],
            "table2_manifest_content_sha256": loaded["table2_manifest_content_sha256"],
            "table2_asset_records_sha256": loaded["table2_asset_records_sha256"],
        },
        "selection": {
            "algorithm": "exact frozen PV-A cohort manifest asset order; optional smoke prefix only",
            "source_protocol": SELECTION_PROTOCOL,
            "cohort_type": loaded["cohort_type"],
            "requested_limit": args.limit,
            "n_eval": len(selected),
            "selected_asset_ids_sha256": canonical_sha256([row["asset_id"] for row in selected]),
            "selected_packages_sha256": canonical_sha256([row["package"] for row in selected]),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "protocol_path": str(PROTOCOL_PATH.resolve(strict=True)),
            "protocol_sha256": sha256_file(PROTOCOL_PATH.resolve(strict=True)),
            "adapter_path": str(SCRIPT_PATH),
            "adapter_sha256": sha256_file(SCRIPT_PATH),
            "shared_ours_runner_path": str(BASE_RUNNER_PATH.resolve(strict=True)),
            "shared_ours_runner_sha256": sha256_file(BASE_RUNNER_PATH),
            "core_evaluator_path": str(CORE_PATH.resolve(strict=True)),
            "core_evaluator_sha256": sha256_file(CORE_PATH),
            "config": config,
            "config_sha256": canonical_sha256(config),
            "environment": environment,
            "environment_sha256": canonical_sha256(environment),
        },
        "records": selected,
    }
    manifest["manifest_content_sha256"] = _self_hash(manifest)
    return manifest


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# PV-A per-class N=5 Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact frozen PV-A cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}; "
            f"J_eval={summary['j_eval']}; K={manifest['evaluation']['config']['samples_per_joint']}."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in core.JOINT_RATE_METRICS:
        value = metrics[metric]
        rate = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {value['passed']} / {value['denominator']} ({rate}) |")
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} ({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    rate = "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    lines.append(f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} ({rate}) |")
    macro = summary["category_macro"]
    lines.extend([
        "",
        f"Category macro average over {macro['category_count']} observed raw categories "
        f"({macro['joint_metric_category_count']} with declared movable joints):",
        "",
        "| Metric | Category macro |",
        "|---|---:|",
    ])
    for metric in (*core.JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = macro["metrics"][metric]
        rate = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {rate} (categories={value['category_count']}) |")
    lines.extend(["", "This evaluates executable declared kinematics only.", ""])
    return "\n".join(lines)


def _prepare_output(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = DEFAULT_OUTPUT_PARENT / f"urdf_table3_pva_per_class_n5_max_joints_n{n_eval}_{timestamp}"
    output = args.output.resolve(strict=False)
    output.relative_to(REPO_ROOT.resolve(strict=True))
    if args.resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)
    return output


def _write_receipt(output: Path, started_at: str, started_perf: float) -> None:
    completed_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    timing = {
        "schema_version": 1,
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_time_seconds": round(time.perf_counter() - started_perf, 6),
        "n_eval": json.loads((output / "summary.json").read_text(encoding="utf-8")).get("n_eval"),
    }
    core.atomic_write_json(output / "timing.json", timing)
    names = ("manifest.json", "asset_records.jsonl", "summary.json", "summary.md", "timing.json")
    artifact = {
        "schema_version": 1,
        "created_at_utc": completed_at,
        "files": {
            name: {"bytes": (output / name).stat().st_size, "sha256": sha256_file(output / name)}
            for name in names
        },
    }
    core.atomic_write_json(output / "artifact_manifest.json", artifact)


def run(args: argparse.Namespace) -> Path:
    started_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    started_perf = time.perf_counter()
    output = BASE.run(args)
    _write_receipt(output, started_at, started_perf)
    return output


def validate_contract(args: argparse.Namespace) -> None:
    if args.resume_frozen and not args.resume:
        raise ValueError("--resume-frozen requires --resume")
    if args.samples < 2 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("samples must be >=2 and workers/timeout must be positive")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError(f"limit must be in [1, {FORMAL_N_EVAL}]")
    if args.mode == "formal":
        expected = (
            args.limit is None,
            args.samples == DEFAULT_SAMPLES,
            args.workers == DEFAULT_WORKERS,
            args.asset_timeout_seconds == DEFAULT_TIMEOUT_SECONDS,
            args.cohort_manifest.resolve(strict=False) == DEFAULT_COHORT_MANIFEST.resolve(strict=False),
        )
        if not all(expected):
            raise ValueError(
                "formal mode freezes canonical PV-A cohort, N=2655, K=21, workers=4, timeout=120"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--limit", type=int, help="smoke mode: evaluate exact manifest prefix")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--resume-frozen", action="store_true")
    parser.add_argument("--internal-job", type=Path)
    parser.add_argument("--internal-result", type=Path)
    return parser.parse_args(argv)


# Patch the shared runner's global namespace.  Its run loop and child harness
# then execute unchanged, while resolving these adapter-specific hooks.
BASE.SCRIPT_PATH = SCRIPT_PATH
BASE.REPO_ROOT = REPO_ROOT
BASE.CORE_PATH = CORE_PATH
BASE.DEFAULT_COHORT_MANIFEST = DEFAULT_COHORT_MANIFEST
BASE.DEFAULT_OUTPUT_PARENT = DEFAULT_OUTPUT_PARENT
BASE.DATASET_NAME = DATASET_NAME
BASE.FORMAL_N_RELEASE = FORMAL_N_RELEASE
BASE.FORMAL_N_EVAL = FORMAL_N_EVAL
BASE.FORMAL_RELEASE_CATEGORY_COUNT = FORMAL_RELEASE_CATEGORY_COUNT
BASE.DEFAULT_SAMPLES = DEFAULT_SAMPLES
BASE.DEFAULT_WORKERS = DEFAULT_WORKERS
BASE.DEFAULT_TIMEOUT_SECONDS = DEFAULT_TIMEOUT_SECONDS
BASE.SELECTION_PROTOCOL = SELECTION_PROTOCOL
BASE.load_cohort = load_cohort
BASE.build_manifest = build_manifest
BASE.validate_contract = validate_contract
BASE._summary_markdown = _summary_markdown
BASE._prepare_output = _prepare_output


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_job is not None or args.internal_result is not None:
        if args.internal_job is None or args.internal_result is None:
            raise ValueError("internal job mode requires both paths")
        return BASE.run_internal_job(args.internal_job, args.internal_result)
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
