#!/usr/bin/env python3
"""Aggregate exploratory passive-stability metrics for one Table 5 dataset."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_passive_stability_runtime as _runtime  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402


SCHEMA_VERSION = "table5_passive_stability_aggregate_v1"
SIMULATORS = ("genesis", "pybullet", "mujoco")


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


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    _core._runtime.atomic_write_json(path, value)


def _atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _dataset(manifest: Mapping[str, Any], slug: str) -> dict[str, Any]:
    matches = [
        dataset
        for dataset in manifest.get("datasets", [])
        if isinstance(dataset, Mapping) and dataset.get("dataset_slug") == slug
    ]
    if len(matches) != 1:
        raise AggregateError(f"dataset {slug!r} is not uniquely present in manifest")
    return deepcopy(dict(matches[0]))


def _record_path(run_root: Path, simulator: str, slug: str, dataset_id: str) -> Path:
    return run_root / simulator / slug / simulator / "assets" / f"{dataset_id}.json"


def _load_record(
    run_root: Path, simulator: str, slug: str, dataset_id: str
) -> dict[str, Any] | None:
    path = _record_path(run_root, simulator, slug, dataset_id)
    return _read_json(path, "asset record") if path.is_file() else None


def _rollout(record: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    if not isinstance(record, Mapping) or record.get("terminal_status") != "completed":
        return None
    evaluation = record.get("evaluation")
    evidence = (
        evaluation.get("passive_stability")
        if isinstance(evaluation, Mapping)
        else None
    )
    rollout = evidence.get("rollout") if isinstance(evidence, Mapping) else None
    if (
        not isinstance(rollout, Mapping)
        or rollout.get("schema_version") != _runtime.RECEIPT_SCHEMA
        or rollout.get("protocol_id") != _runtime.PROTOCOL_ID
    ):
        return None
    return rollout


def _trial_map(rollout: Mapping[str, Any] | None) -> dict[str, Mapping[str, Any]]:
    if not isinstance(rollout, Mapping):
        return {}
    result: dict[str, Mapping[str, Any]] = {}
    for trial in rollout.get("trials", []):
        if isinstance(trial, Mapping) and isinstance(trial.get("trial_id"), str):
            result[str(trial["trial_id"])] = trial
    return result


def _asset_pass(rollout: Mapping[str, Any] | None) -> bool:
    return bool(isinstance(rollout, Mapping) and rollout.get("asset_passed") is True)


def _endpoint_distance(endpoints: Sequence[Mapping[str, Any]]) -> float:
    if len(endpoints) != 3:
        raise AggregateError("endpoint distance requires exactly three engines")
    names = {endpoint.get("joint_name") for endpoint in endpoints}
    modes = {endpoint.get("distance_mode") for endpoint in endpoints}
    scales = [endpoint.get("scale") for endpoint in endpoints]
    if len(names) != 1 or len(modes) != 1 or not all(
        isinstance(scale, (int, float))
        and not isinstance(scale, bool)
        and math.isfinite(float(scale))
        and float(scale) > 0.0
        for scale in scales
    ):
        raise AggregateError("cross-simulator endpoint metadata differs")
    reference_scale = float(scales[0])
    if any(abs(float(scale) - reference_scale) > 1.0e-12 for scale in scales[1:]):
        raise AggregateError("cross-simulator endpoint scales differ")
    spec = {"scale": reference_scale, "distance_mode": next(iter(modes))}
    pairwise = [
        _runtime.normalized_distance(
            float(endpoints[left]["q"]), float(endpoints[right]["q"]), spec
        )
        for left, right in ((0, 1), (0, 2), (1, 2))
    ]
    return statistics.median(pairwise)


def aggregate(
    prepared: Path, run_root: Path, dataset_slug: str
) -> dict[str, Any]:
    manifest = _read_json(prepared, "prepared manifest")
    dataset = _dataset(manifest, dataset_slug)
    rows = [dict(row) for row in dataset.get("rows", []) if isinstance(row, Mapping)]
    if not rows:
        raise AggregateError("selected dataset has no prepared rows")
    records: dict[str, dict[str, dict[str, Any] | None]] = {
        simulator: {
            str(row["dataset_id"]): _load_record(
                run_root, simulator, dataset_slug, str(row["dataset_id"])
            )
            for row in rows
        }
        for simulator in SIMULATORS
    }
    rollouts = {
        simulator: {
            dataset_id: _rollout(record)
            for dataset_id, record in simulator_records.items()
        }
        for simulator, simulator_records in records.items()
    }
    asset_passes = {
        simulator: {
            dataset_id: _asset_pass(rollout)
            for dataset_id, rollout in simulator_rollouts.items()
        }
        for simulator, simulator_rollouts in rollouts.items()
    }
    n = len(rows)
    genesis_count = sum(asset_passes["genesis"].values())
    all_three_count = sum(
        all(asset_passes[simulator][str(row["dataset_id"])] for simulator in SIMULATORS)
        for row in rows
    )
    pairwise: dict[str, dict[str, Any]] = {}
    pairwise_values: list[float] = []
    for left, right in (("genesis", "pybullet"), ("genesis", "mujoco"), ("pybullet", "mujoco")):
        count = sum(
            asset_passes[left][str(row["dataset_id"])]
            == asset_passes[right][str(row["dataset_id"])]
            for row in rows
        )
        percentage = 100.0 * count / n
        pairwise[f"{left}__{right}"] = {"count": count, "denominator": n, "percentage": percentage}
        pairwise_values.append(percentage)

    planned_trial_count = 0
    all_three_valid_trial_count = 0
    endpoint_distances: list[float] = []
    endpoint_asset_ids: set[str] = set()
    for row in rows:
        dataset_id = str(row["dataset_id"])
        planned = _runtime.trial_specs(row)
        planned_trial_count += len(planned)
        trial_maps = {
            simulator: _trial_map(rollouts[simulator][dataset_id])
            for simulator in SIMULATORS
        }
        for trial_spec in planned:
            trial_id = str(trial_spec["trial_id"])
            matched = [trial_maps[simulator].get(trial_id) for simulator in SIMULATORS]
            if not all(
                isinstance(trial, Mapping) and trial.get("valid") is True
                for trial in matched
            ):
                continue
            all_three_valid_trial_count += 1
            if trial_spec.get("target_joint_name") is None:
                continue
            endpoints = [trial.get("target_endpoint") for trial in matched]
            if not all(isinstance(endpoint, Mapping) for endpoint in endpoints):
                continue
            endpoint_distances.append(_endpoint_distance(endpoints))
            endpoint_asset_ids.add(dataset_id)

    endpoint_p95 = _percentile(endpoint_distances, 95.0)
    terminal_counts = {
        simulator: sum(record is not None for record in records[simulator].values())
        for simulator in SIMULATORS
    }
    result: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "classification": (
            "complete"
            if all(count == n for count in terminal_counts.values())
            else "partial"
        ),
        "dataset_name": dataset["dataset_name"],
        "dataset_slug": dataset_slug,
        "n": n,
        "prepared_manifest": str(prepared.resolve()),
        "prepared_manifest_sha256": manifest.get("manifest_sha256"),
        "run_root": str(run_root.resolve()),
        "terminal_record_counts": terminal_counts,
        "protocol": _runtime._protocol(),
        "metrics": {
            "genesis_settle_success": {
                "count": genesis_count,
                "denominator": n,
                "percentage": 100.0 * genesis_count / n,
            },
            "all_three_settle_success": {
                "count": all_three_count,
                "denominator": n,
                "percentage": 100.0 * all_three_count / n,
            },
            "mean_pairwise_settle_agreement": {
                "percentage": statistics.mean(pairwise_values),
                "pairs": pairwise,
            },
            "joint_endpoint_dispersion_p95": {
                "normalized": endpoint_p95,
                "percentage_of_joint_range": (
                    100.0 * endpoint_p95 if endpoint_p95 is not None else None
                ),
                "matched_endpoint_count": len(endpoint_distances),
                "covered_asset_count": len(endpoint_asset_ids),
            },
            "all_three_valid_trial_coverage": {
                "count": all_three_valid_trial_count,
                "denominator": planned_trial_count,
                "percentage": (
                    100.0 * all_three_valid_trial_count / planned_trial_count
                    if planned_trial_count
                    else 0.0
                ),
            },
        },
    }
    result["summary_sha256"] = _core._runtime.canonical_sha256(
        result, exclude_fields=("summary_sha256",)
    )
    return result


def _format(value: Any) -> str:
    return "N/A" if value is None else f"{float(value):.2f}"


def report(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]
    return "\n".join(
        [
            "# Table 5 passive-stability exploratory extension",
            "",
            f"Run classification: **{str(summary['classification']).upper()}**.",
            "",
            "The existing Table 5 metrics are unchanged. This extension uses a fixed root, manifest-bound physics, gravity, contacts, zero applied force, one released canonical joint per trial, a full 10 s horizon, and a final 1 s normalized settling gate.",
            "",
            "| Dataset | Genesis Settle Success (%) ↑ | All-3 Settle Success (%) ↑ | Mean Pairwise Settle Agreement (%) ↑ | Joint Endpoint Dispersion P95 (% range) ↓ | All-3 Valid Trial Coverage (%) ↑ |",
            "|---|---:|---:|---:|---:|---:|",
            f"| {summary['dataset_name']} | "
            f"{_format(metrics['genesis_settle_success']['percentage'])} | "
            f"{_format(metrics['all_three_settle_success']['percentage'])} | "
            f"{_format(metrics['mean_pairwise_settle_agreement']['percentage'])} | "
            f"{_format(metrics['joint_endpoint_dispersion_p95']['percentage_of_joint_range'])} | "
            f"{_format(metrics['all_three_valid_trial_coverage']['percentage'])} |",
            "",
            "Pairwise agreement counts both matched passes and matched failures, so it must be interpreted together with All-3 Settle Success and valid-trial coverage.",
            "",
        ]
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--dataset", default="pva")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise AggregateError(f"output already exists: {args.out}")
    summary = aggregate(args.prepared, args.run_root, args.dataset)
    args.out.mkdir(parents=True)
    _atomic_json(args.out / "summary.json", summary)
    _atomic_text(args.out / "report.md", report(summary))
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "out": str(args.out.resolve()),
                "summary_sha256": summary["summary_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AggregateError as error:
        print(f"table5_passive_stability_aggregate: {error}", file=sys.stderr)
        raise SystemExit(2)
