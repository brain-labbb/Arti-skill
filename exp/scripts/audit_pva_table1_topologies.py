#!/usr/bin/env python3
"""Read-only audit of the frozen PV-A Table 1 topology aggregates."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, field
from decimal import Decimal, localcontext
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
DEFAULT_TABLE1 = REPO / (
    "exp/runtime/pva_table1234_full_release_20260826/evaluation/table1"
)
DEFAULT_RECORDS = DEFAULT_TABLE1 / "asset_records.jsonl"
DEFAULT_SUMMARY = DEFAULT_TABLE1 / "summary.json"


class TopologyAuditError(ValueError):
    """Raised when records or their frozen summary do not satisfy the contract."""


@dataclass
class CategoryCounts:
    n_eval: int = 0
    n_parse: int = 0
    topologies: Counter[str] = field(default_factory=Counter)


@dataclass
class TopologyCounts:
    n_eval: int = 0
    n_parse: int = 0
    categories: dict[str, CategoryCounts] = field(default_factory=dict)


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _add_record(counts: TopologyCounts, row: Mapping[str, Any], label: str) -> None:
    category = row.get("raw_category")
    if not isinstance(category, str) or not category:
        raise TopologyAuditError(f"{label}: raw_category must be a non-empty string")
    parse_success = row.get("parse_success")
    valid_tree = row.get("valid_tree")
    if not isinstance(parse_success, bool):
        raise TopologyAuditError(f"{label}: parse_success must be boolean")
    if not isinstance(valid_tree, bool):
        raise TopologyAuditError(f"{label}: valid_tree must be boolean")

    topology_hash = row.get("topology_hash")
    if valid_tree and (not isinstance(topology_hash, str) or not topology_hash):
        raise TopologyAuditError(
            f"{label}: a valid tree must have a non-empty topology_hash"
        )

    category_counts = counts.categories.setdefault(category, CategoryCounts())
    counts.n_eval += 1
    category_counts.n_eval += 1
    if parse_success:
        counts.n_parse += 1
        category_counts.n_parse += 1
    if valid_tree and topology_hash:
        category_counts.topologies[topology_hash] += 1


def aggregate_records(records: Iterable[Mapping[str, Any]]) -> TopologyCounts:
    """Aggregate in-memory rows with the frozen Table 1 denominator policy."""

    counts = TopologyCounts()
    for index, row in enumerate(records, start=1):
        if not isinstance(row, Mapping):
            raise TopologyAuditError(f"record {index}: expected a JSON object")
        _add_record(counts, row, f"record {index}")
    if not counts.n_eval:
        raise TopologyAuditError("asset records are empty")
    return counts


def read_records(path: Path) -> tuple[TopologyCounts, str]:
    """Stream JSONL records once, returning aggregates and the raw file hash."""

    path = Path(path).resolve(strict=True)
    counts = TopologyCounts()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            digest.update(raw_line)
            if not raw_line.strip():
                raise TopologyAuditError(f"{path}:{line_number}: blank JSONL row")
            try:
                row = json.loads(raw_line)
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise TopologyAuditError(
                    f"{path}:{line_number}: invalid JSON: {error}"
                ) from error
            if not isinstance(row, Mapping):
                raise TopologyAuditError(
                    f"{path}:{line_number}: expected a JSON object"
                )
            _add_record(counts, row, f"{path}:{line_number}")
    if not counts.n_eval:
        raise TopologyAuditError(f"asset records are empty: {path}")
    return counts, digest.hexdigest()


def read_frozen_summary(path: Path) -> tuple[dict[str, Any], str]:
    path = Path(path).resolve(strict=True)
    try:
        summary = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise TopologyAuditError(f"invalid summary JSON: {path}: {error}") from error
    if not isinstance(summary, dict):
        raise TopologyAuditError(f"summary must be a JSON object: {path}")

    declared = summary.get("summary_content_sha256")
    if not isinstance(declared, str) or len(declared) != 64:
        raise TopologyAuditError("summary_content_sha256 is missing or invalid")
    unhashed = dict(summary)
    unhashed.pop("summary_content_sha256")
    calculated = canonical_sha256(unhashed)
    if declared != calculated:
        raise TopologyAuditError(
            "summary_content_sha256 mismatch: "
            f"declared {declared}, calculated {calculated}"
        )
    return summary, _sha256_file(path)


def _decimal(value: Fraction, places: int = 36) -> str:
    with localcontext() as context:
        context.prec = places + 8
        decimal = Decimal(value.numerator) / Decimal(value.denominator)
        return format(decimal, f".{places}f")


def _expected_unique(frequencies: Iterable[int], n: int, k: int) -> Fraction:
    sample_count = math.comb(n, k)
    expected = Fraction()
    for frequency in frequencies:
        missed = math.comb(n - frequency, k)
        expected += Fraction(sample_count - missed, sample_count)
    return expected


def topology_metrics(counts: TopologyCounts, *, k: int = 5) -> dict[str, Any]:
    """Calculate pooled, conditioned, macro, and exact rarefaction views."""

    if isinstance(k, bool) or not isinstance(k, int) or k < 1:
        raise TopologyAuditError("k must be a positive integer")

    category_rows: list[tuple[str, CategoryCounts, int]] = []
    pooled_hashes: set[str] = set()
    for category, category_counts in sorted(counts.categories.items()):
        denominator = sum(category_counts.topologies.values())
        category_rows.append((category, category_counts, denominator))
        pooled_hashes.update(category_counts.topologies)

    topology_denominator = sum(row[2] for row in category_rows)
    if not topology_denominator:
        raise TopologyAuditError("no valid topology records are available")
    conditioned_unique = sum(len(row[1].topologies) for row in category_rows)
    evaluable = [row for row in category_rows if row[2] > 0]
    macro_exact = sum(
        (
            Fraction(len(category_counts.topologies), denominator)
            for _, category_counts, denominator in evaluable
        ),
        Fraction(),
    ) / len(evaluable)

    undersized = [
        (category, denominator)
        for category, _, denominator in category_rows
        if denominator < k
    ]
    if undersized:
        preview = ", ".join(
            f"{category}={denominator}"
            for category, denominator in undersized[:5]
        )
        suffix = " ..." if len(undersized) > 5 else ""
        raise TopologyAuditError(
            f"rarefaction k={k} exceeds valid topology count in "
            f"{len(undersized)} categories: {preview}{suffix}"
        )

    category_rarefaction: list[Fraction] = []
    for _, category_counts, denominator in category_rows:
        expected_unique = _expected_unique(
            category_counts.topologies.values(), denominator, k
        )
        category_rarefaction.append(expected_unique / k)
    rarefaction_exact = sum(category_rarefaction, Fraction()) / len(
        category_rarefaction
    )

    return {
        "pooled": {
            "unique": len(pooled_hashes),
            "denominator": topology_denominator,
            "rate": len(pooled_hashes) / topology_denominator,
        },
        "category_conditioned": {
            "unique": conditioned_unique,
            "denominator": topology_denominator,
            "rate": conditioned_unique / topology_denominator,
        },
        "category_macro": {
            "category_count": len(evaluable),
            "rate": float(macro_exact),
        },
        "category_stratified_rarefaction": {
            "k": k,
            "category_count": len(category_rarefaction),
            "expected_unique_macro": statistics.fmean(
                float(value * k) for value in category_rarefaction
            ),
            "rate": statistics.fmean(float(value) for value in category_rarefaction),
            "exact_rate_numerator": str(rarefaction_exact.numerator),
            "exact_rate_denominator": str(rarefaction_exact.denominator),
            "rate_decimal_36": _decimal(rarefaction_exact),
        },
    }


def _field(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for component in path.split("."):
        if not isinstance(current, Mapping) or component not in current:
            raise TopologyAuditError(f"frozen summary is missing {path}")
        current = current[component]
    return current


def _check_integer(summary: Mapping[str, Any], path: str, audited: int) -> None:
    reported = _field(summary, path)
    if (
        isinstance(reported, bool)
        or not isinstance(reported, int)
        or reported != audited
    ):
        raise TopologyAuditError(
            f"frozen summary {path} mismatch: reported {reported!r}, audited {audited}"
        )


def _check_rate(summary: Mapping[str, Any], path: str, audited: float) -> None:
    reported = _field(summary, path)
    if isinstance(reported, bool) or not isinstance(reported, (int, float)):
        raise TopologyAuditError(f"frozen summary {path} is not numeric: {reported!r}")
    if not math.isclose(float(reported), audited, rel_tol=0.0, abs_tol=1e-15):
        raise TopologyAuditError(
            f"frozen summary {path} mismatch: reported {reported!r}, audited {audited!r}"
        )


def validate_summary(
    counts: TopologyCounts, metrics: Mapping[str, Any], summary: Mapping[str, Any]
) -> dict[str, Any]:
    """Fail closed unless every topology aggregate agrees with the summary."""

    _check_integer(summary, "cohort.N_eval", counts.n_eval)
    _check_integer(summary, "cohort.N_parse", counts.n_parse)
    _check_integer(summary, "cohort.eval_raw_categories", len(counts.categories))
    if "n_eval" in summary:
        _check_integer(summary, "n_eval", counts.n_eval)

    pooled = metrics["pooled"]
    _check_integer(summary, "unique_topologies.unique", pooled["unique"])
    _check_integer(summary, "unique_topologies.denominator", pooled["denominator"])
    _check_rate(summary, "unique_topologies.rate", pooled["rate"])
    _check_integer(summary, "unique_topologies.coverage_denominator", counts.n_eval)
    _check_rate(
        summary,
        "unique_topologies.coverage_rate",
        pooled["denominator"] / counts.n_eval,
    )

    breakdown = _field(summary, "category_breakdown")
    if not isinstance(breakdown, Mapping):
        raise TopologyAuditError("frozen summary category_breakdown must be an object")
    if set(breakdown) != set(counts.categories):
        missing = sorted(set(counts.categories) - set(breakdown))
        extra = sorted(set(breakdown) - set(counts.categories))
        raise TopologyAuditError(
            f"frozen summary category set mismatch: missing={missing}, extra={extra}"
        )
    for category, category_counts in sorted(counts.categories.items()):
        prefix = f"category_breakdown.{category}"
        denominator = sum(category_counts.topologies.values())
        unique = len(category_counts.topologies)
        _check_integer(summary, f"{prefix}.N_eval", category_counts.n_eval)
        _check_integer(summary, f"{prefix}.N_parse", category_counts.n_parse)
        _check_integer(summary, f"{prefix}.unique_topologies.unique", unique)
        _check_integer(
            summary, f"{prefix}.unique_topologies.denominator", denominator
        )
        _check_rate(
            summary,
            f"{prefix}.unique_topologies.rate",
            unique / denominator,
        )

    macro = metrics["category_macro"]
    _check_integer(summary, "category_macro.category_count", len(counts.categories))
    _check_integer(
        summary,
        "category_macro.unique_topologies_evaluable_categories",
        macro["category_count"],
    )
    _check_rate(summary, "category_macro.unique_topologies_rate", macro["rate"])
    return {
        "frozen_summary_consistent": True,
        "category_rows_verified": len(counts.categories),
        "record_rows_verified": counts.n_eval,
    }


def audit_topology_records(
    records: Iterable[Mapping[str, Any]],
    summary: Mapping[str, Any],
    *,
    k: int = 5,
) -> dict[str, Any]:
    counts = aggregate_records(records)
    metrics = topology_metrics(counts, k=k)
    return {
        "schema_version": "pva_table1_topology_audit_v1",
        **metrics,
        "validation": validate_summary(counts, metrics, summary),
    }


def audit_files(
    records_path: Path = DEFAULT_RECORDS,
    summary_path: Path = DEFAULT_SUMMARY,
    *,
    k: int = 5,
) -> dict[str, Any]:
    records_path = Path(records_path).resolve(strict=True)
    summary_path = Path(summary_path).resolve(strict=True)
    summary, summary_sha256 = read_frozen_summary(summary_path)
    counts, records_sha256 = read_records(records_path)
    metrics = topology_metrics(counts, k=k)
    validation = validate_summary(counts, metrics, summary)
    validation["summary_content_sha256_verified"] = True
    return {
        "schema_version": "pva_table1_topology_audit_v1",
        "sources": {
            "asset_records": str(records_path),
            "asset_records_sha256": records_sha256,
            "summary": str(summary_path),
            "summary_sha256": summary_sha256,
            "summary_content_sha256": summary["summary_content_sha256"],
        },
        **metrics,
        "validation": validation,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("-k", type=int, default=5, help="per-category sample budget")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = audit_files(args.records, args.summary, k=args.k)
    except (OSError, TopologyAuditError) as error:
        print(f"topology audit failed: {error}", file=sys.stderr)
        return 2
    json.dump(result, sys.stdout, indent=2, sort_keys=True, ensure_ascii=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
