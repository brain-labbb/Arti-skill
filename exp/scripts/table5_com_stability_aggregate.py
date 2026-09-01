#!/usr/bin/env python3
"""Aggregate Genesis center-of-mass stability for one or more datasets."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import statistics
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_com_stability_runtime as _runtime  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402


SCHEMA_VERSION = "table5_com_stability_aggregate_v1"


class AggregateError(RuntimeError):
    pass


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AggregateError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AggregateError(f"{label} must be an object: {path}")
    return value


def _dataset(manifest: Mapping[str, Any], slug: str) -> dict[str, Any]:
    matches = [
        dataset
        for dataset in manifest.get("datasets", [])
        if isinstance(dataset, Mapping) and dataset.get("dataset_slug") == slug
    ]
    if len(matches) != 1:
        raise AggregateError(f"dataset {slug!r} is not uniquely present")
    return deepcopy(dict(matches[0]))


def _rollout(record: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    if not isinstance(record, Mapping) or record.get("terminal_status") != "completed":
        return None
    evaluation = record.get("evaluation")
    evidence = evaluation.get("com_stability") if isinstance(evaluation, Mapping) else None
    rollout = evidence.get("rollout") if isinstance(evidence, Mapping) else None
    if (
        not isinstance(rollout, Mapping)
        or not isinstance(evidence, Mapping)
        or evidence.get("protocol", {}).get("protocol_id") != _runtime.PROTOCOL_ID
    ):
        return None
    return rollout


def _record_protocol(record: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    if not isinstance(record, Mapping) or record.get("terminal_status") != "completed":
        return None
    evaluation = record.get("evaluation")
    evidence = evaluation.get("com_stability") if isinstance(evaluation, Mapping) else None
    protocol = evidence.get("protocol") if isinstance(evidence, Mapping) else None
    if (
        not isinstance(protocol, Mapping)
        or protocol.get("protocol_id") != _runtime.PROTOCOL_ID
    ):
        return None
    return protocol


def _common_protocol(
    protocols: Sequence[Mapping[str, Any] | None],
) -> dict[str, Any] | None:
    available = [protocol for protocol in protocols if protocol is not None]
    if not available:
        return None
    hashes = {str(protocol.get("protocol_sha256")) for protocol in available}
    if len(hashes) != 1 or "None" in hashes:
        raise AggregateError("runtime records contain inconsistent protocol receipts")
    return deepcopy(dict(available[0]))


def _record_path(run_root: Path, slug: str, dataset_id: str) -> Path:
    return run_root / slug / "genesis" / "assets" / f"{dataset_id}.json"


def _aggregate_dataset(
    prepared: Path, run_root: Path, slug: str
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    manifest = _read_json(prepared, "prepared manifest")
    dataset = _dataset(manifest, slug)
    rows = [dict(row) for row in dataset.get("rows", []) if isinstance(row, Mapping)]
    if not rows:
        raise AggregateError(f"dataset {slug!r} has no rows")
    records = {}
    rollouts: dict[str, Mapping[str, Any] | None] = {}
    for row in rows:
        dataset_id = str(row["dataset_id"])
        path = _record_path(run_root, slug, dataset_id)
        record = _read_json(path, "asset record") if path.is_file() else None
        records[dataset_id] = record
        rollouts[dataset_id] = _rollout(record)
    protocol = _common_protocol([_record_protocol(record) for record in records.values()])
    n = len(rows)
    completed = sum(record is not None for record in records.values())
    valid_rollouts = [rollout for rollout in rollouts.values() if rollout is not None]
    supports = [
        rollout["com_support"]
        for rollout in valid_rollouts
        if isinstance(rollout.get("com_support"), Mapping)
        and rollout["com_support"].get("available") is True
    ]
    margins = [float(item["normalized_signed_margin"]) for item in supports]
    raw_margins = [float(item["signed_margin_m"]) for item in supports]
    engine_static_count = sum(
        rollout is not None
        and isinstance(rollout.get("com_support"), Mapping)
        and rollout["com_support"].get("available") is True
        and float(rollout["com_support"].get("signed_margin_m", float("-inf")))
        >= -_runtime.STATIC_MARGIN_TOLERANCE_M
        and rollout.get("rollout_passed") is True
        for rollout in rollouts.values()
    )
    ready_ids = {
        str(row["dataset_id"])
        for row in rows
        if row.get("physics", {}).get("status") == "ready"
    }
    attributable_supports = [
        rollout["com_support"]
        for dataset_id, rollout in rollouts.items()
        if dataset_id in ready_ids
        and rollout is not None
        and isinstance(rollout.get("com_support"), Mapping)
        and rollout["com_support"].get("available") is True
    ]
    attributable_margins = [
        float(item["normalized_signed_margin"]) for item in attributable_supports
    ]
    attributable_raw_margins = [
        float(item["signed_margin_m"]) for item in attributable_supports
    ]
    attributable_static_count = sum(
        dataset_id in ready_ids
        and rollout is not None
        and isinstance(rollout.get("com_support"), Mapping)
        and rollout["com_support"].get("available") is True
        and float(rollout["com_support"].get("signed_margin_m", float("-inf")))
        >= -_runtime.STATIC_MARGIN_TOLERANCE_M
        and rollout.get("rollout_passed") is True
        for dataset_id, rollout in rollouts.items()
    )
    physics_groups: dict[str, dict[str, int]] = {}
    for row in rows:
        status = str(row.get("physics", {}).get("status", "unknown"))
        group = physics_groups.setdefault(
            status,
            {"n": 0, "completed": 0, "support_available": 0, "static_pass": 0},
        )
        group["n"] += 1
        rollout = rollouts[str(row["dataset_id"])]
        group["completed"] += int(rollout is not None)
        group["support_available"] += int(
            rollout is not None
            and isinstance(rollout.get("com_support"), Mapping)
            and rollout["com_support"].get("available") is True
        )
        group["static_pass"] += int(
            rollout is not None
            and isinstance(rollout.get("com_support"), Mapping)
            and rollout["com_support"].get("available") is True
            and float(rollout["com_support"].get("signed_margin_m", float("-inf")))
            >= -_runtime.STATIC_MARGIN_TOLERANCE_M
            and rollout.get("rollout_passed") is True
        )
    result = {
        "dataset_name": dataset["dataset_name"],
        "dataset_slug": slug,
        "n": n,
        "prepared_manifest": str(prepared.resolve()),
        "prepared_manifest_sha256": manifest.get("manifest_sha256"),
        "run_root": str(run_root.resolve()),
        "terminal_record_count": completed,
        "valid_rollout_count": len(valid_rollouts),
        "metrics": {
            "physics_parameter_coverage": {
                "count": len(ready_ids),
                "denominator": n,
                "percentage": 100.0 * len(ready_ids) / n,
            },
            "dataset_attributable_com_support_margin": {
                "count": len(attributable_margins),
                "denominator": n,
                "coverage_percentage": 100.0 * len(attributable_margins) / n,
                "mean_normalized_signed_margin": (
                    statistics.fmean(attributable_margins)
                    if attributable_margins
                    else None
                ),
                "median_normalized_signed_margin": (
                    statistics.median(attributable_margins)
                    if attributable_margins
                    else None
                ),
                "mean_signed_margin_mm": (
                    1000.0 * statistics.fmean(attributable_raw_margins)
                    if attributable_raw_margins
                    else None
                ),
                "median_signed_margin_mm": (
                    1000.0 * statistics.median(attributable_raw_margins)
                    if attributable_raw_margins
                    else None
                ),
            },
            "dataset_attributable_com_static_stability": {
                "count": attributable_static_count,
                "denominator": n,
                "percentage": 100.0 * attributable_static_count / n,
                "conditional_ready_count": attributable_static_count,
                "conditional_ready_denominator": len(ready_ids),
                "conditional_ready_percentage": (
                    100.0 * attributable_static_count / len(ready_ids)
                    if ready_ids
                    else None
                ),
            },
            "engine_finalized_diagnostic": {
                "com_support_margin": {
                    "count": len(margins),
                    "denominator": n,
                    "coverage_percentage": 100.0 * len(margins) / n,
                    "mean_normalized_signed_margin": (
                        statistics.fmean(margins) if margins else None
                    ),
                    "median_normalized_signed_margin": (
                        statistics.median(margins) if margins else None
                    ),
                    "mean_signed_margin_mm": (
                        1000.0 * statistics.fmean(raw_margins) if raw_margins else None
                    ),
                    "median_signed_margin_mm": (
                        1000.0 * statistics.median(raw_margins) if raw_margins else None
                    ),
                },
                "com_static_stability": {
                    "count": engine_static_count,
                    "denominator": n,
                    "percentage": 100.0 * engine_static_count / n,
                },
            },
        },
        "physics_status_breakdown": physics_groups,
    }
    return result, protocol


def aggregate_dataset(prepared: Path, run_root: Path, slug: str) -> dict[str, Any]:
    result, _ = _aggregate_dataset(prepared, run_root, slug)
    return result


def _aggregate_result(
    dataset_results: Sequence[tuple[dict[str, Any], dict[str, Any] | None]],
) -> dict[str, Any]:
    datasets = [dataset for dataset, _ in dataset_results]
    protocol = _common_protocol([receipt for _, receipt in dataset_results])
    return {
        "schema_version": SCHEMA_VERSION,
        "classification": (
            "complete"
            if all(
                dataset["terminal_record_count"] == dataset["n"]
                and dataset["valid_rollout_count"] == dataset["n"]
                for dataset in datasets
            )
            else "partial"
        ),
        "protocol": protocol,
        "publication_attribution_policy": {
            "physics_requirement": "prepared row physics.status must equal ready",
            "fallback_policy": "Genesis-native COM fallback retained only as a diagnostic",
            "static_denominator": "all free-standing eligible assets; missing attributable physics fails closed",
            "margin_population": "physics-ready assets with a non-degenerate Genesis contact support polygon",
        },
        "datasets": datasets,
    }


def aggregate(
    pva_prepared: Path,
    pva_run: Path,
    articraft_prepared: Path,
    articraft_run: Path,
) -> dict[str, Any]:
    dataset_results = [
        _aggregate_dataset(pva_prepared, pva_run, "pva"),
        _aggregate_dataset(articraft_prepared, articraft_run, "articraft_10k"),
    ]
    return _aggregate_result(dataset_results)


def aggregate_many(
    dataset_specs: Sequence[tuple[str, Path, Path]],
) -> dict[str, Any]:
    if not dataset_specs:
        raise AggregateError("at least one dataset specification is required")
    slugs = [slug for slug, _, _ in dataset_specs]
    if len(slugs) != len(set(slugs)):
        raise AggregateError("dataset specifications contain duplicate slugs")
    dataset_results = [
        _aggregate_dataset(prepared, run_root, slug)
        for slug, prepared, run_root in dataset_specs
    ]
    return _aggregate_result(dataset_results)


def _number(value: Any, digits: int) -> str:
    return "N/A" if value is None else f"{float(value):.{digits}f}"


def _percentage(value: Any) -> str:
    return "N/A" if value is None else f"{float(value):.1f}%"


def _report(result: Mapping[str, Any]) -> str:
    lines = [
        "# Genesis Center-of-Mass Stability (Exploratory)",
        "",
        "Joints are hard-locked at canonical q=0; the root remains free. The support polygon is built from Genesis ground-contact positions during the initial contact window.",
        "",
        "## Dataset-attributable metrics",
        "",
        "| Dataset | Eligible N | Physics Coverage ↑ | CoM Margin Coverage ↑ | Normalized CoM Support Margin ↑ | CoM Static Stability ↑ |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for dataset in result["datasets"]:
        metrics = dataset["metrics"]
        coverage = metrics["physics_parameter_coverage"]
        margin = metrics["dataset_attributable_com_support_margin"]
        static = metrics["dataset_attributable_com_static_stability"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_percentage(coverage['percentage'])} | "
            f"{_percentage(margin['coverage_percentage'])} | "
            f"{_number(margin['median_normalized_signed_margin'], 4)} | "
            f"{_percentage(static['percentage'])} |"
        )
    lines.extend(
        [
            "",
            "The primary view requires dataset-provided valid physics. Missing physics fails closed for CoM Static Stability; margin summaries include only attributable, non-degenerate support measurements.",
            "CoM Support Margin is the signed COM-to-support-boundary distance. The normalized value divides it by support-polygon diameter.",
            "The primary table reports the normalized margin because the millimeter margin is sensitive to asset scale.",
            "",
            "## Supplementary attributable diagnostics",
            "",
            "| Dataset | CoM Support Margin median (mm) ↑ | Conditional Stability ↑ |",
            "|---|---:|---:|",
        ]
    )
    for dataset in result["datasets"]:
        metrics = dataset["metrics"]
        margin = metrics["dataset_attributable_com_support_margin"]
        static = metrics["dataset_attributable_com_static_stability"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_number(margin['median_signed_margin_mm'], 2)} | "
            f"{_percentage(static['conditional_ready_percentage'])} |"
        )
    lines.extend(
        [
            "",
            "Conditional Stability uses physics-ready assets as its denominator and separates physical completeness from dynamics.",
            "",
            "## Genesis-finalized diagnostic",
            "",
            "| Dataset | N | CoM Support Margin median (normalized) ↑ | CoM Support Margin median (mm) ↑ | CoM Static Stability ↑ | Support coverage ↑ |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in result["datasets"]:
        metrics = dataset["metrics"]["engine_finalized_diagnostic"]
        margin = metrics["com_support_margin"]
        static = metrics["com_static_stability"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_number(margin['median_normalized_signed_margin'], 4)} | "
            f"{_number(margin['median_signed_margin_mm'], 2)} | "
            f"{_percentage(static['percentage'])} | "
            f"{_percentage(margin['coverage_percentage'])} |"
        )
    lines.extend(
        [
            "",
            "The diagnostic view includes Genesis-native fallback mass and COM for assets whose released physics is missing; it is not attributed to the source dataset.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-prepared", type=Path)
    parser.add_argument("--pva-run", type=Path)
    parser.add_argument("--articraft-prepared", type=Path)
    parser.add_argument("--articraft-run", type=Path)
    parser.add_argument(
        "--dataset",
        action="append",
        nargs=3,
        metavar=("SLUG", "PREPARED", "RUN_ROOT"),
        help="repeatable dataset specification for a general multi-dataset aggregate",
    )
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args(argv)
    legacy = (
        arguments.pva_prepared,
        arguments.pva_run,
        arguments.articraft_prepared,
        arguments.articraft_run,
    )
    if arguments.dataset:
        if any(value is not None for value in legacy):
            parser.error("--dataset cannot be combined with legacy PV-A/Articraft arguments")
        result = aggregate_many(
            [
                (slug, Path(prepared), Path(run_root))
                for slug, prepared, run_root in arguments.dataset
            ]
        )
    else:
        if any(value is None for value in legacy):
            parser.error(
                "provide one or more --dataset triples, or all four legacy PV-A/Articraft arguments"
            )
        result = aggregate(*legacy)
    arguments.out.mkdir(parents=True, exist_ok=True)
    _core._runtime.atomic_write_json(arguments.out / "aggregate.json", result)
    report_path = arguments.out / "report.md"
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(_report(result), encoding="utf-8")
    temporary.replace(report_path)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
