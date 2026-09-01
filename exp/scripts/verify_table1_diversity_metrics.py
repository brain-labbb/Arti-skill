#!/usr/bin/env python3
"""Independently verify Table 1 diversity metric artifacts."""

from __future__ import annotations

import argparse
from collections import defaultdict
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Iterator, Mapping

import numpy as np
from scipy.spatial import cKDTree


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_OUTPUT = PROJECT_ROOT / "exp/runtime/table1_var_neardup_diagnostic_20260827"
PAIR_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f4")])
CHAMFER_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f8")])


class VerificationError(ValueError):
    pass


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iter_jsonl(path: Path) -> Iterator[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise VerificationError(f"invalid JSONL {path}:{line_number}: {error}") from error
            if not isinstance(row, dict):
                raise VerificationError(f"non-object JSONL row {path}:{line_number}")
            yield row


def verify_self_hash(path: Path, field: str) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise VerificationError(f"JSON artifact is not an object: {path}")
    unsigned = dict(value)
    declared = unsigned.pop(field, None)
    if declared != canonical_sha256(unsigned):
        raise VerificationError(f"self-hash mismatch: {path}")
    return value


def verify_file_binding(binding: Mapping[str, Any], label: str) -> None:
    path = Path(str(binding.get("path")))
    if not path.is_file() or path.stat().st_size != binding.get("bytes"):
        raise VerificationError(f"bound input is missing or resized: {label}")
    if sha256_file(path) != binding.get("sha256"):
        raise VerificationError(f"bound input hash drift: {label}")


def category_key(dataset: str, row: Mapping[str, Any]) -> str:
    category = row.get("raw_category", row.get("category"))
    if not isinstance(category, str) or not category.strip():
        raise VerificationError(f"{dataset} has an empty category")
    category = category.strip()
    if dataset == "sketch":
        asset_id = row.get("asset_id")
        if not isinstance(asset_id, str) or "/" not in asset_id:
            raise VerificationError("Sketch row has no source prefix")
        fields = asset_id.split("/")
        source = fields[1] if fields[0] == "data" and len(fields) > 2 else fields[0]
        return f"{source}/{category}"
    return category


def category_bootstrap_ci95(values: list[float], seed_key: str) -> list[float]:
    array = np.asarray(values, dtype=np.float64)
    if len(array) == 1:
        return [float(array[0]), float(array[0])]
    seed = int(hashlib.sha256(seed_key.encode("utf-8")).hexdigest(), 16)
    generator = np.random.Generator(np.random.PCG64DXSM(seed))
    means = np.empty(10_000, dtype=np.float64)
    for start in range(0, 10_000, 256):
        stop = min(start + 256, 10_000)
        picks = generator.integers(0, len(array), size=(stop - start, len(array)))
        means[start:stop] = np.mean(array[picks], axis=1)
    return [float(value) for value in np.quantile(means, (0.025, 0.975))]


def independently_aggregate_variance(
    dataset: str, records: Iterable[Mapping[str, Any]]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    buckets: dict[str, dict[str, int]] = defaultdict(
        lambda: {"n_total": 0, "n_valid": 0, "sum": 0, "sum_sq": 0}
    )
    n_total = n_valid = 0
    for row in records:
        n_total += 1
        category = category_key(dataset, row)
        bucket = buckets[category]
        bucket["n_total"] += 1
        value = row.get("non_fixed_joint_count")
        if row.get("parse_success") is True and type(value) is int and value >= 0:
            n_valid += 1
            bucket["n_valid"] += 1
            bucket["sum"] += value
            bucket["sum_sq"] += value * value
    per_category: list[dict[str, Any]] = []
    exact_values: list[Fraction] = []
    for category, bucket in sorted(buckets.items()):
        count = bucket["n_valid"]
        exact = None if not count else Fraction(
            count * bucket["sum_sq"] - bucket["sum"] ** 2, count ** 2
        )
        if exact is not None:
            exact_values.append(exact)
        per_category.append({
            "dataset_key": dataset,
            "category": category,
            "n_total": bucket["n_total"],
            "n_valid": count,
            "sum_joints": bucket["sum"],
            "sum_squared_joints": bucket["sum_sq"],
            "population_variance": None if exact is None else float(exact),
            "population_variance_exact": None if exact is None else f"{exact.numerator}/{exact.denominator}",
        })
    if not exact_values:
        raise VerificationError(f"{dataset} has no valid joint counts")
    macro = sum(exact_values, Fraction()) / len(exact_values)
    return ({
        "n_total": n_total,
        "n_valid": n_valid,
        "asset_coverage": n_valid / n_total,
        "category_count": len(buckets),
        "valid_category_count": len(exact_values),
        "category_coverage": len(exact_values) / len(buckets),
        "all_invalid_category_count": len(buckets) - len(exact_values),
        "macro_population_variance": float(macro),
        "macro_population_variance_exact": f"{macro.numerator}/{macro.denominator}",
        "macro_population_variance_ci95": category_bootstrap_ci95(
            [float(value) for value in exact_values],
            f"within-release-label-population-variance-macro-v1|bootstrap|{dataset}",
        ),
        "bootstrap_resamples": 10_000,
    }, per_category)


def verify_variance(output: Path) -> dict[str, Any]:
    summary_path = output / "variance/summary.json"
    summary = verify_self_hash(summary_path, "summary_content_sha256")
    categories_path = output / "variance/category_records.jsonl"
    if sha256_file(categories_path) != summary.get("category_records_sha256"):
        raise VerificationError("variance category-record hash mismatch")
    frozen_categories = list(iter_jsonl(categories_path))
    by_dataset: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in frozen_categories:
        by_dataset[str(row.get("dataset_key"))].append(row)
    checked = []
    for frozen in summary.get("datasets", []):
        dataset = str(frozen["dataset_key"])
        records = Path(str(frozen["records_path"]))
        if sha256_file(records) != frozen.get("records_sha256"):
            raise VerificationError(f"{dataset} source record hash drift")
        recomputed, category_rows = independently_aggregate_variance(dataset, iter_jsonl(records))
        for field, value in recomputed.items():
            observed = frozen.get(field)
            if isinstance(value, float):
                if not isinstance(observed, (int, float)) or not math.isclose(
                    float(observed), value, rel_tol=0.0, abs_tol=1e-15
                ):
                    raise VerificationError(f"{dataset} variance field mismatch: {field}")
            elif observed != value:
                raise VerificationError(f"{dataset} variance field mismatch: {field}")
        if by_dataset.get(dataset) != category_rows:
            raise VerificationError(f"{dataset} per-category variance records mismatch")
        checked.append(dataset)
    if set(by_dataset) != set(checked):
        raise VerificationError("variance category records contain a foreign dataset")
    return {"status": "PASS", "dataset_count": len(checked), "datasets": checked}


def _geometry_index(path: Path) -> tuple[dict[int, dict[str, Any]], dict[str, int]]:
    by_ordinal: dict[int, dict[str, Any]] = {}
    statuses: dict[str, int] = defaultdict(int)
    for row in iter_jsonl(path):
        ordinal = row.get("ordinal")
        if type(ordinal) is not int or ordinal < 0 or ordinal in by_ordinal:
            raise VerificationError(f"invalid or duplicate geometry ordinal: {ordinal!r}")
        by_ordinal[ordinal] = row
        statuses[str(row.get("status"))] += 1
    if sorted(by_ordinal) != list(range(len(by_ordinal))):
        raise VerificationError("geometry ordinals are not contiguous")
    return by_ordinal, dict(statuses)


def verify_scratch_bindings(summary: Mapping[str, Any]) -> None:
    bindings = summary.get("scratch_bindings")
    if not isinstance(bindings, Mapping):
        raise VerificationError("geometry summary has no scratch bindings")
    for name in ("points", "descriptors", "database"):
        binding = bindings.get(name)
        if not isinstance(binding, Mapping):
            raise VerificationError(f"geometry summary lacks {name} scratch binding")
        path = Path(str(binding.get("path")))
        if not path.is_file() or path.stat().st_size != binding.get("bytes"):
            raise VerificationError(f"geometry scratch artifact is missing or resized: {name}")
        if sha256_file(path) != binding.get("sha256"):
            raise VerificationError(f"geometry scratch artifact hash drift: {name}")
        if name != "database":
            array = np.load(path, mmap_mode="r")
            if list(array.shape) != binding.get("shape") or str(array.dtype) != binding.get("dtype"):
                raise VerificationError(f"geometry scratch array contract drift: {name}")


def symmetric_chamfer(left: np.ndarray, right: np.ndarray) -> float:
    left_to_right = cKDTree(np.asarray(right, dtype=np.float64)).query(
        np.asarray(left, dtype=np.float64), k=1, workers=1
    )[0]
    right_to_left = cKDTree(np.asarray(left, dtype=np.float64)).query(
        np.asarray(right, dtype=np.float64), k=1, workers=1
    )[0]
    return float(0.5 * (np.mean(left_to_right) + np.mean(right_to_left)))


def independently_candidate_pairs(
    descriptors: np.ndarray, top_k: int, exhaustive_limit: int
) -> list[tuple[int, int, float]]:
    values = np.asarray(descriptors, dtype=np.float64)
    pair_distances: dict[tuple[int, int], float] = {}
    if len(values) <= exhaustive_limit:
        for left in range(len(values)):
            distances = np.linalg.norm(values[left + 1:] - values[left], axis=1)
            for right, distance in enumerate(distances, left + 1):
                pair_distances[(left, right)] = float(distance)
    elif len(values) >= 2:
        k = min(len(values), top_k + 1)
        distances, indices = cKDTree(values).query(values, k=k, workers=1)
        if k == 1:
            distances, indices = distances[:, None], indices[:, None]
        for left in range(len(values)):
            for distance, right_value in zip(distances[left], indices[left], strict=True):
                right = int(right_value)
                if right == left:
                    continue
                pair = min(left, right), max(left, right)
                previous = pair_distances.get(pair)
                pair_distances[pair] = (
                    float(distance) if previous is None else min(previous, float(distance))
                )
    return [
        (left, right, pair_distances[(left, right)])
        for left, right in sorted(pair_distances)
    ]


def verify_candidate_file(
    path: Path,
    expected_hash: str,
    expected_count: int,
    geometry: Mapping[int, Mapping[str, Any]],
    descriptors: np.ndarray,
    *,
    top_k: int,
    exhaustive_limit: int,
) -> None:
    if sha256_file(path) != expected_hash:
        raise VerificationError(f"candidate-pair hash mismatch: {path}")
    if path.stat().st_size != expected_count * PAIR_DTYPE.itemsize:
        raise VerificationError(f"candidate-pair byte count mismatch: {path}")
    group_members: dict[tuple[str, str], list[int]] = defaultdict(list)
    for ordinal, row in geometry.items():
        if row.get("status") == "EVALUATED":
            group_members[(str(row.get("category")), str(row.get("graph_hash")))].append(ordinal)
    recompute_groups = {
        group for group, members in group_members.items()
        if 2 <= len(members) <= exhaustive_limit
    }
    large_groups = [group for group, members in group_members.items() if len(members) > exhaustive_limit]
    recompute_groups.update(sorted(
        large_groups,
        key=lambda group: hashlib.sha256(canonical_bytes(group)).hexdigest(),
    )[:8])
    if not expected_count:
        if any(len(members) >= 2 for members in group_members.values()):
            raise VerificationError("candidate file is empty despite an eligible pair group")
        return
    pairs = np.memmap(path, dtype=PAIR_DTYPE, mode="r")
    sampled_indices = set(
        np.linspace(0, len(pairs) - 1, min(64, len(pairs)), dtype=int).tolist()
    )
    last_group: tuple[str, str] | None = None
    last_pair: tuple[int, int] | None = None
    closed_groups: set[tuple[str, str]] = set()
    observed_recomputed: dict[tuple[str, str], list[tuple[int, int, float]]] = defaultdict(list)
    for index, row in enumerate(pairs):
        left, right, distance = int(row["left"]), int(row["right"]), float(row["distance"])
        if left >= right or left not in geometry or right not in geometry:
            raise VerificationError("candidate pair has invalid ordinals")
        if not math.isfinite(distance) or distance < 0:
            raise VerificationError("candidate descriptor distance is not finite non-negative")
        left_row, right_row = geometry[left], geometry[right]
        if left_row.get("status") != "EVALUATED" or right_row.get("status") != "EVALUATED":
            raise VerificationError("candidate pair includes an unevaluable asset")
        group = (str(left_row.get("category")), str(left_row.get("graph_hash")))
        if group != (str(right_row.get("category")), str(right_row.get("graph_hash"))):
            raise VerificationError("candidate pair crosses category/graph gate")
        pair = (left, right)
        if group != last_group:
            if group in closed_groups:
                raise VerificationError("candidate group is not contiguous")
            if last_group is not None:
                closed_groups.add(last_group)
            last_group, last_pair = group, None
        if last_pair is not None and pair <= last_pair:
            raise VerificationError("candidate pairs are duplicate or out of order within group")
        last_pair = pair
        if index in sampled_indices:
            recomputed_distance = float(np.linalg.norm(
                np.asarray(descriptors[left], dtype=np.float64)
                - np.asarray(descriptors[right], dtype=np.float64)
            ))
            if not math.isclose(recomputed_distance, distance, rel_tol=0, abs_tol=2e-6):
                raise VerificationError("candidate descriptor distance recomputation mismatch")
        if group in recompute_groups:
            observed_recomputed[group].append((left, right, distance))
    for group in recompute_groups:
        ordinals = sorted(group_members[group])
        expected = independently_candidate_pairs(
            descriptors[ordinals], top_k, exhaustive_limit
        )
        expected_global = [
            (ordinals[left], ordinals[right], distance)
            for left, right, distance in expected
        ]
        observed = observed_recomputed.get(group, [])
        if [(left, right) for left, right, _ in observed] != [
            (left, right) for left, right, _ in expected_global
        ]:
            raise VerificationError("candidate group completeness recomputation mismatch")
        for frozen, recomputed in zip(observed, expected_global, strict=True):
            if not math.isclose(frozen[2], recomputed[2], rel_tol=0, abs_tol=2e-6):
                raise VerificationError("candidate group distance recomputation mismatch")


def independently_duplicate_rates(n: int, edges: Iterable[tuple[int, int]]) -> dict[str, Any]:
    parent = list(range(n))
    touched: set[int] = set()

    def root(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    for left, right in edges:
        if not 0 <= left < n or not 0 <= right < n or left == right:
            raise VerificationError("positive edge is outside the evaluable denominator")
        touched.update((left, right))
        a, b = root(left), root(right)
        if a != b:
            parent[b] = a
    components = len({root(index) for index in range(n)})
    excess = n - components
    return {
        "n_evaluable": n,
        "neighbor_asset_count": len(touched),
        "neighbor_asset_rate": len(touched) / n if n else None,
        "component_count": components,
        "duplicate_excess_count": excess,
        "cluster_excess_rate": excess / n if n else None,
    }


def independently_category_macro_rates(
    categories: list[str], edges: list[tuple[int, int]], seed_key: str
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    members: dict[str, list[int]] = defaultdict(list)
    grouped_edges: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for index, category in enumerate(categories):
        members[category].append(index)
    for left, right in edges:
        if categories[left] != categories[right]:
            raise VerificationError("positive edge crosses a category boundary")
        grouped_edges[categories[left]].append((left, right))
    rows = []
    for category, indices in sorted(members.items()):
        local = {global_index: local_index for local_index, global_index in enumerate(indices)}
        rates = independently_duplicate_rates(
            len(indices),
            ((local[left], local[right]) for left, right in grouped_edges.get(category, [])),
        )
        rows.append({"category": category, **rates})
    clusters = [float(row["cluster_excess_rate"]) for row in rows]
    neighbors = [float(row["neighbor_asset_rate"]) for row in rows]
    return ({
        "category_count": len(rows),
        "category_macro_cluster_excess_rate": float(np.mean(clusters)),
        "category_macro_cluster_excess_rate_ci95": category_bootstrap_ci95(
            clusters, f"{seed_key}|cluster"
        ),
        "category_macro_neighbor_asset_rate": float(np.mean(neighbors)),
        "category_macro_neighbor_asset_rate_ci95": category_bootstrap_ci95(
            neighbors, f"{seed_key}|neighbor"
        ),
        "bootstrap_resamples": 10_000,
    }, rows)


def _wilson_ci95(successes: int, trials: int) -> list[float] | None:
    if trials == 0:
        return None
    z = 1.959963984540054
    probability = successes / trials
    denominator = 1.0 + z * z / trials
    center = (probability + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(
        probability * (1.0 - probability) / trials
        + z * z / (4.0 * trials * trials)
    ) / denominator
    return [max(0.0, center - half), min(1.0, center + half)]


def _precision_recall(rows: list[tuple[float, bool]], tau: float) -> dict[str, Any]:
    tp = sum(distance < tau and label for distance, label in rows)
    fp = sum(distance < tau and not label for distance, label in rows)
    fn = sum(distance >= tau and label for distance, label in rows)
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": tp / (tp + fp) if tp + fp else None,
        "precision_ci95": _wilson_ci95(tp, tp + fp),
        "recall": tp / (tp + fn) if tp + fn else None,
        "recall_ci95": _wilson_ci95(tp, tp + fn),
    }


def independently_verify_calibration(
    receipt: Mapping[str, Any],
    labels: list[dict[str, Any]],
    tasks: Mapping[str, Mapping[str, Any]],
    audits: Mapping[str, Mapping[str, Any]],
    manifest: Mapping[str, Any],
) -> None:
    label_ids = [str(row.get("pair_id")) for row in labels]
    if len(label_ids) != len(set(label_ids)) or set(label_ids) != set(tasks):
        raise VerificationError("completed labels do not close the annotation task set")
    calibration_protocol = str(manifest["protocols"]["calibration"])
    resolved: list[tuple[str, float, bool]] = []
    uncertain = 0
    for row in labels:
        pair_id = str(row.get("pair_id"))
        label = row.get("label")
        if label == "uncertain":
            uncertain += 1
        elif label in {"duplicate", "not_duplicate"}:
            distance = float(audits[pair_id]["chamfer_distance"])
            resolved.append((pair_id, distance, label == "duplicate"))
        else:
            raise VerificationError(f"completed label is not explicit: {pair_id}")
    if len(resolved) < int(manifest["min_resolved_labels"]):
        raise VerificationError("threshold receipt has fewer resolved labels than the manifest gate")
    train: list[tuple[float, bool]] = []
    heldout: list[tuple[float, bool]] = []
    split_excluded = 0
    for pair_id, distance, label in resolved:
        audit = audits[pair_id]
        dataset = str(audit["dataset_key"])

        def bucket(asset_id: str) -> int:
            digest = hashlib.sha256(
                f"{calibration_protocol}|asset-split|{dataset}|{asset_id}".encode()
            ).digest()
            return int.from_bytes(digest[:8], "big") % 10

        left_bucket = bucket(str(audit["left_asset_id"]))
        right_bucket = bucket(str(audit["right_asset_id"]))
        if left_bucket < 4 and right_bucket < 4:
            heldout.append((distance, label))
        elif left_bucket >= 4 and right_bucket >= 4:
            train.append((distance, label))
        else:
            split_excluded += 1
    if (
        not train
        or not heldout
        or not any(label for _, label in train)
        or not any(not label for _, label in train)
        or not any(label for _, label in heldout)
        or not any(not label for _, label in heldout)
    ):
        raise VerificationError(
            "calibration split lacks asset-disjoint support for both classes"
        )
    target_precision = float(manifest["target_precision"])
    candidates = sorted({math.nextafter(distance, math.inf) for distance, _ in train})
    viable: list[tuple[float, float, int]] = []
    for tau in candidates:
        metrics = _precision_recall(train, tau)
        predicted = int(metrics["tp"] + metrics["fp"])
        if (
            predicted >= 20
            and metrics["precision"] is not None
            and metrics["precision"] >= target_precision
        ):
            viable.append((float(metrics["recall"] or 0.0), tau, predicted))
    if not viable:
        raise VerificationError("threshold receipt exists although no train threshold is viable")
    _, tau, _ = max(viable, key=lambda item: (item[0], -item[1]))
    train_metrics = _precision_recall(train, tau)
    heldout_metrics = _precision_recall(heldout, tau)
    passed = (
        heldout_metrics["precision"] is not None
        and heldout_metrics["precision"] >= target_precision
        and heldout_metrics["tp"] + heldout_metrics["fp"] >= 10
    )
    expected = {
        "calibration_protocol": calibration_protocol,
        "tau": tau,
        "target_precision": target_precision,
        "submitted_label_count": len(labels),
        "resolved_label_count": len(resolved),
        "uncertain_or_unlabeled_count": uncertain,
        "train_count": len(train),
        "heldout_count": len(heldout),
        "split_excluded_count": split_excluded,
        "train_metrics": train_metrics,
        "heldout_metrics": heldout_metrics,
        "status": "PASS" if passed else "FAILED_HELDOUT_PRECISION",
    }
    for field, value in expected.items():
        if receipt.get(field) != value:
            raise VerificationError(f"threshold receipt recomputation mismatch: {field}")


def verify_near_duplicate(output: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    expected_n = {
        str(key): int(value) for key, value in manifest["dataset_expected_n"].items()
    }
    root = output / "near_duplicate"
    if not root.is_dir():
        return {
            "status": "NOT_STARTED",
            "pipeline_complete": False,
            "dataset_count": 0,
            "candidate_dataset_count": 0,
            "score_dataset_count": 0,
            "missing_geometry_datasets": sorted(expected_n),
            "missing_candidate_datasets": sorted(expected_n),
            "missing_score_datasets": sorted(expected_n),
            "datasets": [],
            "calibration": "NOT_STARTED",
        }
    checked: list[str] = []
    candidate_checked: list[str] = []
    score_checked: list[str] = []
    for directory in sorted(path for path in root.iterdir() if path.is_dir() and path.name != "calibration"):
        geometry_summary_path = directory / "geometry_summary.json"
        if not geometry_summary_path.is_file():
            continue
        geometry_summary = verify_self_hash(geometry_summary_path, "summary_content_sha256")
        verify_scratch_bindings(geometry_summary)
        key = str(geometry_summary["dataset_key"])
        if key != directory.name or key not in expected_n:
            raise VerificationError(f"foreign near-duplicate dataset directory: {directory}")
        records_path = directory / "geometry_records.jsonl"
        if sha256_file(records_path) != geometry_summary.get("geometry_records_sha256"):
            raise VerificationError(f"{key} geometry record hash mismatch")
        geometry, statuses = _geometry_index(records_path)
        if len(geometry) != expected_n[key] or len(geometry) != geometry_summary.get("n_total"):
            raise VerificationError(f"{key} geometry denominator mismatch")
        if statuses != geometry_summary.get("status_counts"):
            raise VerificationError(f"{key} geometry status aggregation mismatch")
        evaluable = sum(row.get("status") == "EVALUATED" for row in geometry.values())
        if evaluable != geometry_summary.get("n_evaluable"):
            raise VerificationError(f"{key} geometry evaluable count mismatch")
        retried = 0
        transient_exhausted = 0
        for row in geometry.values():
            attempts = row.get("attempt_count")
            if type(attempts) is not int or not 1 <= attempts <= 3:
                raise VerificationError(f"{key} geometry retry accounting is invalid")
            retried += attempts > 1
            transient_exhausted += row.get("error_kind") == "TRANSIENT_EXHAUSTED"
            expected_fingerprint = row.get("expected_package_fingerprint")
            if (
                row.get("status") == "EVALUATED"
                and expected_fingerprint is not None
                and row.get("package_fingerprint") != expected_fingerprint
            ):
                raise VerificationError(f"{key} evaluated package fingerprint mismatch")
            if row.get("status") == "EVALUATED":
                binding_mode = row.get("visual_binding_mode")
                if binding_mode not in {
                    "TABLE1_PACKAGE_FINGERPRINT",
                    "ROSTER_RESOURCE_MANIFEST",
                    "URDF_HASHED_PRIMITIVES_ONLY",
                }:
                    raise VerificationError(f"{key} evaluated geometry lacks a frozen visual binding")
                if (
                    binding_mode == "URDF_HASHED_PRIMITIVES_ONLY"
                    and row.get("visual_resource_bindings")
                ):
                    raise VerificationError(f"{key} primitive-only binding loaded an external resource")
        if (
            retried != geometry_summary.get("retried_asset_count")
            or transient_exhausted != geometry_summary.get("transient_exhausted_count")
            or geometry_summary.get("error_review_required")
            is not (int(statuses.get("ERROR", 0)) > 0)
        ):
            raise VerificationError(f"{key} geometry error/retry aggregation mismatch")
        candidate_path = directory / "candidate_summary.json"
        if candidate_path.is_file():
            candidate = verify_self_hash(candidate_path, "summary_content_sha256")
            if candidate.get("geometry_summary_sha256") != sha256_file(geometry_summary_path):
                raise VerificationError(f"{key} candidate-to-geometry binding mismatch")
            verify_candidate_file(
                Path(str(candidate["candidate_pairs_path"])),
                str(candidate["candidate_pairs_sha256"]),
                int(candidate["candidate_pair_count"]),
                geometry,
                np.load(
                    Path(str(geometry_summary["scratch_descriptors_path"])),
                    mmap_mode="r",
                ),
                top_k=int(candidate["top_k"]),
                exhaustive_limit=int(candidate["exhaustive_group_limit"]),
            )
            checkpoint_path = directory / "candidate_checkpoint.json"
            if sha256_file(checkpoint_path) != candidate.get("candidate_checkpoint_sha256"):
                raise VerificationError(f"{key} candidate checkpoint binding mismatch")
            checkpoint = verify_self_hash(
                checkpoint_path, "checkpoint_content_sha256"
            )
            if (
                checkpoint.get("state") != "COMPLETE"
                or checkpoint.get("candidate_pairs_sha256")
                != candidate.get("candidate_pairs_sha256")
                or checkpoint.get("candidate_pair_count")
                != candidate.get("candidate_pair_count")
            ):
                raise VerificationError(f"{key} candidate checkpoint completion mismatch")
            candidate_checked.append(key)
        score_path = directory / "score.json"
        if score_path.is_file():
            score = verify_self_hash(score_path, "score_content_sha256")
            if not candidate_path.is_file():
                raise VerificationError(f"{key} score exists without candidate summary")
            candidate = verify_self_hash(candidate_path, "summary_content_sha256")
            receipt_path = root / "calibration/threshold_receipt.json"
            if not receipt_path.is_file():
                raise VerificationError(f"{key} score exists without threshold receipt")
            if score.get("geometry_summary_sha256") != sha256_file(geometry_summary_path):
                raise VerificationError(f"{key} score-to-geometry binding mismatch")
            if score.get("candidate_summary_sha256") != sha256_file(candidate_path):
                raise VerificationError(f"{key} score-to-candidate summary binding mismatch")
            if score.get("candidate_pairs_sha256") != candidate.get("candidate_pairs_sha256"):
                raise VerificationError(f"{key} score-to-candidate pair binding mismatch")
            if score.get("threshold_receipt_sha256") != sha256_file(receipt_path):
                raise VerificationError(f"{key} score-to-threshold binding mismatch")
            receipt = verify_self_hash(receipt_path, "receipt_content_sha256")
            if not math.isclose(float(score.get("tau")), float(receipt.get("tau")), rel_tol=0, abs_tol=0):
                raise VerificationError(f"{key} score threshold differs from its receipt")
            if (
                score.get("retrieval_recall_status") != "NOT_ESTABLISHED"
                or score.get("rate_interpretation")
                != "REQUESTED_DENOMINATOR_CANDIDATE_DETECTED_LOWER_BOUND_WITH_EVALUABLE_CONDITIONAL_DIAGNOSTICS"
                or score.get("table1_headline_eligible") is not False
            ):
                raise VerificationError(f"{key} diagnostic lower-bound classification drift")
            distance_path = directory / "chamfer_pairs.bin"
            if sha256_file(distance_path) != score.get("chamfer_pairs_sha256"):
                raise VerificationError(f"{key} Chamfer pair hash mismatch")
            if distance_path.stat().st_size != int(score["candidate_pair_count"]) * CHAMFER_DTYPE.itemsize:
                raise VerificationError(f"{key} Chamfer pair byte count mismatch")
            ordinals = [ordinal for ordinal, row in geometry.items() if row.get("status") == "EVALUATED"]
            dense = {ordinal: index for index, ordinal in enumerate(ordinals)}
            tau = float(score["tau"])
            edges = []
            source_pairs_path = Path(str(candidate["candidate_pairs_path"]))
            source_pairs = (
                np.memmap(source_pairs_path, dtype=PAIR_DTYPE, mode="r")
                if source_pairs_path.stat().st_size else []
            )
            chamfer_pairs = (
                np.memmap(distance_path, dtype=CHAMFER_DTYPE, mode="r")
                if distance_path.stat().st_size else []
            )
            if distance_path.stat().st_size:
                for index, row in enumerate(chamfer_pairs):
                    left, right, distance = int(row["left"]), int(row["right"]), float(row["distance"])
                    source_row = source_pairs[index]
                    if left != int(source_row["left"]) or right != int(source_row["right"]):
                        raise VerificationError(f"{key} Chamfer/candidate identity mismatch")
                    if not math.isfinite(distance) or distance < 0 or left not in dense or right not in dense:
                        raise VerificationError(f"{key} invalid Chamfer record")
                    if distance < tau:
                        edges.append((dense[left], dense[right]))
                points = np.load(Path(str(geometry_summary["scratch_points_path"])), mmap_mode="r")
                audit_count = min(32, len(chamfer_pairs))
                audit_indices = sorted(set(
                    np.linspace(0, len(chamfer_pairs) - 1, audit_count, dtype=int).tolist()
                ))
                for index in audit_indices:
                    row = chamfer_pairs[index]
                    recomputed_distance = symmetric_chamfer(
                        points[int(row["left"])], points[int(row["right"])]
                    )
                    if not math.isclose(
                        recomputed_distance, float(row["distance"]), rel_tol=0.0, abs_tol=1e-12
                    ):
                        raise VerificationError(f"{key} sampled Chamfer recomputation mismatch")
            recomputed = independently_duplicate_rates(len(ordinals), edges)
            for field, value in recomputed.items():
                observed = score.get(field)
                if isinstance(value, float):
                    if not math.isclose(float(observed), value, rel_tol=0, abs_tol=1e-15):
                        raise VerificationError(f"{key} score mismatch: {field}")
                elif observed != value:
                    raise VerificationError(f"{key} score mismatch: {field}")
            categories = [str(geometry[ordinal]["category"]) for ordinal in ordinals]
            macro, category_rows = independently_category_macro_rates(
                categories,
                edges,
                f"connected-component-excess-over-geometry-evaluable-v1|bootstrap|{key}",
            )
            if score.get("category_macro") != macro:
                raise VerificationError(f"{key} category-macro score mismatch")
            requested_lower_bound = recomputed["duplicate_excess_count"] / expected_n[key]
            if score.get("candidate_detected_excess_over_requested_lower_bound") != requested_lower_bound:
                raise VerificationError(f"{key} requested-denominator lower bound mismatch")
            if score.get("candidate_detected_cluster_excess_rate_on_evaluable_assets") != recomputed["cluster_excess_rate"]:
                raise VerificationError(f"{key} evaluable-conditional rate mismatch")
            if score.get("candidate_detected_category_macro_rate_on_evaluable_assets") != macro["category_macro_cluster_excess_rate"]:
                raise VerificationError(f"{key} category-macro conditional rate mismatch")
            category_path = directory / "score_categories.jsonl"
            if sha256_file(category_path) != score.get("score_categories_sha256"):
                raise VerificationError(f"{key} score-category hash mismatch")
            if list(iter_jsonl(category_path)) != category_rows:
                raise VerificationError(f"{key} score-category records mismatch")
            score_checked.append(key)
        checked.append(key)

    calibration = root / "calibration"
    packet_path = calibration / "annotation_packet.json"
    calibration_status = "NOT_STARTED"
    packet: dict[str, Any] | None = None
    if packet_path.is_file():
        packet = verify_self_hash(packet_path, "packet_content_sha256")
        if packet.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
            raise VerificationError("annotation packet run-manifest binding mismatch")
        tasks_path = calibration / "annotation_tasks.jsonl"
        audit_path = calibration / "annotation_audit.jsonl"
        template_path = calibration / "annotation_labels_template.jsonl"
        previews_path = calibration / "preview_manifest.jsonl"
        bindings = (
            (tasks_path, "annotation_tasks_sha256"),
            (audit_path, "annotation_audit_sha256"),
            (template_path, "labels_template_sha256"),
            (previews_path, "preview_manifest_sha256"),
        )
        for path, field in bindings:
            if sha256_file(path) != packet.get(field):
                raise VerificationError(f"annotation packet binding mismatch: {field}")
        task_rows = list(iter_jsonl(tasks_path))
        audit_rows = list(iter_jsonl(audit_path))
        preview_rows = list(iter_jsonl(previews_path))
        tasks = {str(row["pair_id"]): row for row in task_rows}
        audits = {str(row["pair_id"]): row for row in audit_rows}
        previews = {str(row["pair_id"]): row for row in preview_rows}
        if (
            len(tasks) != len(task_rows)
            or len(audits) != len(audit_rows)
            or len(previews) != len(preview_rows)
            or len(tasks) != packet.get("task_count")
            or set(tasks) != set(audits)
            or set(tasks) != set(previews)
        ):
            raise VerificationError("annotation task/audit identity mismatch")
        for pair_id, row in tasks.items():
            if row.get("label") is not None:
                raise VerificationError("blind annotation task contains a prefilled label")
            preview = output / str(row["preview_path"])
            if not preview.is_file():
                raise VerificationError(f"annotation preview is missing: {pair_id}")
            if previews[pair_id].get("path") != row.get("preview_path") or sha256_file(preview) != previews[pair_id].get("sha256"):
                raise VerificationError(f"annotation preview binding mismatch: {pair_id}")
        calibration_status = "AWAITING_HUMAN_LABELS"
        receipt_path = calibration / "threshold_receipt.json"
        if receipt_path.is_file():
            receipt = verify_self_hash(receipt_path, "receipt_content_sha256")
            if receipt.get("annotation_audit_sha256") != sha256_file(audit_path):
                raise VerificationError("threshold receipt audit binding mismatch")
            labels = Path(str(receipt["labels_path"]))
            expected_labels = (calibration / "completed_labels.jsonl").resolve()
            if labels.resolve() != expected_labels:
                raise VerificationError("threshold receipt depends on labels outside the run output")
            if sha256_file(labels) != receipt.get("labels_sha256"):
                raise VerificationError("threshold receipt label binding mismatch")
            if receipt.get("annotation_packet_sha256") != sha256_file(packet_path):
                raise VerificationError("threshold receipt packet binding mismatch")
            if receipt.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
                raise VerificationError("threshold receipt run-manifest binding mismatch")
            independently_verify_calibration(
                receipt, list(iter_jsonl(labels)), tasks, audits, manifest
            )
            calibration_status = str(receipt.get("status"))

    expected = set(expected_n)
    status_path = output / "status.json"
    declared_phase = None
    if status_path.is_file():
        declared_phase = str(verify_self_hash(status_path, "status_content_sha256").get("status"))
    checked_set, candidate_set, score_set = set(checked), set(candidate_checked), set(score_checked)
    if not checked_set.issubset(expected) or not candidate_set.issubset(checked_set) or not score_set.issubset(candidate_set):
        raise VerificationError("near-duplicate stage sets violate the manifest")
    if packet is not None:
        packet_keys = set(packet.get("dataset_bindings", {}))
        if packet_keys != candidate_set:
            raise VerificationError("annotation packet dataset bindings do not match candidates")
        for key, binding in packet["dataset_bindings"].items():
            directory = root / key
            if (
                binding.get("geometry_summary_sha256") != sha256_file(directory / "geometry_summary.json")
                or binding.get("candidate_summary_sha256") != sha256_file(directory / "candidate_summary.json")
            ):
                raise VerificationError(f"annotation packet upstream binding mismatch: {key}")

    if declared_phase in {
        "BLOCKED_CALIBRATION", "CALIBRATED_DIAGNOSTIC_READY_TO_SCORE",
        "DIAGNOSTIC_SCORE_COMPLETE",
    }:
        if checked_set != expected or candidate_set != expected or packet is None:
            raise VerificationError(f"declared phase {declared_phase} is missing a dataset/candidate/packet")
    if declared_phase in {"CALIBRATED_DIAGNOSTIC_READY_TO_SCORE", "DIAGNOSTIC_SCORE_COMPLETE"}:
        if calibration_status != "PASS":
            raise VerificationError(f"declared phase {declared_phase} lacks a passing threshold receipt")
    if declared_phase == "DIAGNOSTIC_SCORE_COMPLETE":
        if score_set != expected:
            raise VerificationError("complete diagnostic score is missing dataset scores")
        summary_path = root / "summary.json"
        summary = verify_self_hash(summary_path, "summary_content_sha256")
        if summary.get("threshold_receipt_sha256") != sha256_file(calibration / "threshold_receipt.json"):
            raise VerificationError("near-duplicate summary threshold binding mismatch")
        summary_keys = {str(row.get("dataset_key")) for row in summary.get("datasets", [])}
        if summary_keys != expected or len(summary.get("datasets", [])) != len(expected):
            raise VerificationError("near-duplicate summary dataset closure mismatch")
        if (
            summary.get("retrieval_recall_status") != "NOT_ESTABLISHED"
            or summary.get("rate_interpretation")
            != "REQUESTED_DENOMINATOR_CANDIDATE_DETECTED_LOWER_BOUND_WITH_EVALUABLE_CONDITIONAL_DIAGNOSTICS"
            or summary.get("table1_headline_eligible") is not False
        ):
            raise VerificationError("near-duplicate summary diagnostic classification drift")

    if declared_phase == "DIAGNOSTIC_SCORE_COMPLETE":
        phase = "COMPLETE"
    elif declared_phase == "CALIBRATED_DIAGNOSTIC_READY_TO_SCORE":
        phase = "CALIBRATED"
    elif declared_phase == "BLOCKED_CALIBRATION":
        phase = "PREPARED"
    elif checked_set or root.exists():
        phase = "IN_PROGRESS"
    else:
        phase = "NOT_STARTED"
    return {
        "status": phase,
        "pipeline_complete": phase == "COMPLETE",
        "dataset_count": len(checked),
        "candidate_dataset_count": len(candidate_checked),
        "score_dataset_count": len(score_checked),
        "missing_geometry_datasets": sorted(expected - checked_set),
        "missing_candidate_datasets": sorted(expected - candidate_set),
        "missing_score_datasets": sorted(expected - score_set),
        "datasets": checked,
        "calibration": calibration_status,
    }


def verify(output: Path) -> dict[str, Any]:
    output = output.resolve()
    manifest = verify_self_hash(output / "manifest.json", "manifest_content_sha256")
    runner = Path(str(manifest["script_path"]))
    if sha256_file(runner) != manifest.get("script_sha256"):
        raise VerificationError("runner script has drifted since run initialization")
    verify_file_binding(manifest["fingerprint_core"], "fingerprint core")
    dataset_inputs = manifest.get("dataset_inputs")
    if not isinstance(dataset_inputs, Mapping) or set(dataset_inputs) != set(manifest["dataset_keys"]):
        raise VerificationError("manifest dataset input closure mismatch")
    for key, bindings in dataset_inputs.items():
        verify_file_binding(bindings["records"], f"{key} records")
        verify_file_binding(bindings["roster"], f"{key} roster")
    for name, binding in manifest.get("pva_mirror_receipts", {}).items():
        verify_file_binding(binding, f"PV-A mirror receipt {name}")
    variance = verify_variance(output)
    near = verify_near_duplicate(output, manifest)
    result = {
        "schema_version": "table1_diversity_verification_v1",
        "integrity_pass": True,
        "all_pass": near["pipeline_complete"],
        "pipeline_complete": near["pipeline_complete"],
        "output": str(output),
        "run_manifest_sha256": sha256_file(output / "manifest.json"),
        "variance": variance,
        "near_duplicate": near,
    }
    result["verification_content_sha256"] = canonical_sha256(result)
    temporary = output / ".verification.json.tmp"
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(output / "verification.json")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        result = verify(args.output)
    except Exception as error:  # noqa: BLE001
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
