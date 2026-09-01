#!/usr/bin/env python3
"""Aggregate Genesis tip-over stability for PV-A and Articraft."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_tipover_runtime as _runtime  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402


SCHEMA_VERSION = "table5_tipover_stability_aggregate_v1"


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


def _record_path(run_root: Path, slug: str, dataset_id: str) -> Path:
    return run_root / slug / "genesis" / "assets" / f"{dataset_id}.json"


def _rollout(record: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    if not isinstance(record, Mapping) or record.get("terminal_status") != "completed":
        return None
    evaluation = record.get("evaluation")
    evidence = evaluation.get("tipover_stability") if isinstance(evaluation, Mapping) else None
    rollout = evidence.get("rollout") if isinstance(evidence, Mapping) else None
    if (
        not isinstance(rollout, Mapping)
        or rollout.get("schema_version") != _runtime.RECEIPT_SCHEMA
        or rollout.get("protocol_id") != _runtime.PROTOCOL_ID
    ):
        return None
    return rollout


def aggregate_dataset(prepared: Path, run_root: Path, slug: str) -> dict[str, Any]:
    manifest = _read_json(prepared, "prepared manifest")
    dataset = _dataset(manifest, slug)
    rows = [dict(row) for row in dataset.get("rows", []) if isinstance(row, Mapping)]
    if not rows:
        raise AggregateError(f"dataset {slug!r} has no rows")
    records: dict[str, dict[str, Any] | None] = {}
    rollouts: dict[str, Mapping[str, Any] | None] = {}
    for row in rows:
        dataset_id = str(row["dataset_id"])
        path = _record_path(run_root, slug, dataset_id)
        record = _read_json(path, "asset record") if path.is_file() else None
        records[dataset_id] = record
        rollouts[dataset_id] = _rollout(record)
    n = len(rows)
    completed = sum(record is not None for record in records.values())
    valid_assets = sum(rollout is not None for rollout in rollouts.values())
    gravity_count = sum(
        rollout is not None and rollout.get("tip_over_stability_passed") is True
        for rollout in rollouts.values()
    )
    perturbed_count = sum(
        rollout is not None and rollout.get("perturbed_tip_over_stability_passed") is True
        for rollout in rollouts.values()
    )
    planned_trials = sum(
        int(rollout.get("planned_trial_count", 0))
        for rollout in rollouts.values()
        if rollout is not None
    )
    valid_trials = sum(
        int(rollout.get("valid_trial_count", 0))
        for rollout in rollouts.values()
        if rollout is not None
    )
    physics_groups: dict[str, dict[str, int]] = {}
    for row in rows:
        status = str(row.get("physics", {}).get("status", "unknown"))
        group = physics_groups.setdefault(status, {"n": 0, "tip_over_pass": 0, "perturbed_pass": 0})
        group["n"] += 1
        rollout = rollouts[str(row["dataset_id"])]
        group["tip_over_pass"] += int(
            rollout is not None and rollout.get("tip_over_stability_passed") is True
        )
        group["perturbed_pass"] += int(
            rollout is not None and rollout.get("perturbed_tip_over_stability_passed") is True
        )
    return {
        "dataset_name": dataset["dataset_name"],
        "dataset_slug": slug,
        "n": n,
        "prepared_manifest": str(prepared.resolve()),
        "prepared_manifest_sha256": manifest.get("manifest_sha256"),
        "run_root": str(run_root.resolve()),
        "terminal_record_count": completed,
        "valid_asset_count": valid_assets,
        "metrics": {
            "tip_over_stability": {
                "count": gravity_count,
                "denominator": n,
                "percentage": 100.0 * gravity_count / n,
            },
            "perturbed_tip_over_stability": {
                "count": perturbed_count,
                "denominator": n,
                "percentage": 100.0 * perturbed_count / n,
            },
            "valid_trial_coverage": {
                "count": valid_trials,
                "denominator": 5 * n,
                "percentage": 100.0 * valid_trials / (5 * n),
                "observed_planned_trial_count": planned_trials,
            },
        },
        "physics_status_breakdown": physics_groups,
    }


def aggregate(
    pva_prepared: Path,
    pva_run: Path,
    articraft_prepared: Path,
    articraft_run: Path,
) -> dict[str, Any]:
    datasets = [
        aggregate_dataset(pva_prepared, pva_run, "pva"),
        aggregate_dataset(articraft_prepared, articraft_run, "articraft_10k"),
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "classification": (
            "complete"
            if all(dataset["terminal_record_count"] == dataset["n"] for dataset in datasets)
            else "partial"
        ),
        "protocol": _runtime._protocol(),
        "datasets": datasets,
    }


def _report(result: Mapping[str, Any]) -> str:
    lines = [
        "# Genesis Tip-Over Stability (Exploratory)",
        "",
        "Joints are hard-locked at canonical q=0; the root remains free.",
        "",
        "| Dataset | N | Tip-Over Stability ↑ | Perturbed Tip-Over Stability ↑ | Valid Trial Coverage ↑ |",
        "|---|---:|---:|---:|---:|",
    ]
    for dataset in result["datasets"]:
        metrics = dataset["metrics"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{metrics['tip_over_stability']['percentage']:.1f}% | "
            f"{metrics['perturbed_tip_over_stability']['percentage']:.1f}% | "
            f"{metrics['valid_trial_coverage']['percentage']:.1f}% |"
        )
    lines.extend(
        [
            "",
            "The perturbed metric requires the gravity-only trial and all four ±X/±Y trials to pass.",
            "Articraft physics-status strata are retained in aggregate.json because missing released inertials use Genesis native fallback.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-prepared", type=Path, required=True)
    parser.add_argument("--pva-run", type=Path, required=True)
    parser.add_argument("--articraft-prepared", type=Path, required=True)
    parser.add_argument("--articraft-run", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args(argv)
    result = aggregate(
        arguments.pva_prepared,
        arguments.pva_run,
        arguments.articraft_prepared,
        arguments.articraft_run,
    )
    arguments.out.mkdir(parents=True, exist_ok=True)
    _core._runtime.atomic_write_json(arguments.out / "aggregate.json", result)
    report_path = arguments.out / "report.md"
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(_report(result), encoding="utf-8")
    temporary.replace(report_path)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AggregateError as error:
        print(f"table5_tipover_aggregate: {error}", file=sys.stderr)
        raise SystemExit(2)
