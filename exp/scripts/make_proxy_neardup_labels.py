#!/usr/bin/env python3
"""Create explicitly non-human, geometry-assisted near-duplicate labels.

The formal near-duplicate runner deliberately requires a label file before it
can choose ``tau``.  This utility is a reproducible fallback for diagnostics
when a human panel is unavailable: it applies a frozen Chamfer-distance rubric
to the already selected, category/kinematic-graph-gated annotation packet.
The output is *not* a human ground-truth annotation and must not be used as a
headline prevalence estimate.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import os
import tempfile
from typing import Any, Iterable, Iterator, Mapping


DEFAULT_LOW = 0.012
DEFAULT_HIGH = 0.015
SOURCE_TAG = "synthetic_proxy_not_human"


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError(f"refusing to replace symlink: {path}")
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def iter_jsonl(path: Path) -> Iterator[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{line_number} is not a JSON object")
            yield value


def classify(distance: float, low: float, high: float) -> str:
    if not math.isfinite(distance) or distance < 0:
        raise ValueError(f"invalid Chamfer distance: {distance!r}")
    if distance < low:
        return "duplicate"
    if distance < high:
        return "uncertain"
    return "not_duplicate"


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def build_labels(
    rows: Iterable[Mapping[str, Any]], *, low: float, high: float
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    audits = list(rows)
    if not audits:
        raise ValueError("annotation audit is empty")
    seen: set[str] = set()
    labels: list[dict[str, Any]] = []
    distances: list[float] = []
    by_dataset: dict[str, Counter[str]] = defaultdict(Counter)
    by_mode: dict[str, Counter[str]] = defaultdict(Counter)
    rule = (
        f"duplicate iff chamfer_distance < {low:.12g}; "
        f"uncertain iff {low:.12g} <= chamfer_distance < {high:.12g}; "
        f"not_duplicate iff chamfer_distance >= {high:.12g}"
    )
    for audit in audits:
        pair_id = audit.get("pair_id")
        if not isinstance(pair_id, str) or not pair_id:
            raise ValueError(f"invalid pair_id: {pair_id!r}")
        if pair_id in seen:
            raise ValueError(f"duplicate pair_id: {pair_id}")
        seen.add(pair_id)
        try:
            distance = float(audit["chamfer_distance"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"missing/invalid distance for {pair_id}") from error
        label = classify(distance, low, high)
        confidence = "high" if label != "uncertain" else "low"
        labels.append(
            {
                "pair_id": pair_id,
                "label": label,
                "annotation_source": SOURCE_TAG,
                "reviewer_confidence": confidence,
                "decision_rule": rule,
                "notes": (
                    "Analyst/AI-assisted proxy label from frozen normalized "
                    "Chamfer distance; not independent human annotation."
                ),
            }
        )
        distances.append(distance)
        dataset = str(audit.get("dataset_key", ""))
        mode = str(audit.get("selection_mode", ""))
        by_dataset[dataset][label] += 1
        by_mode[mode][label] += 1
    labels.sort(key=lambda row: row["pair_id"])
    counts = Counter(row["label"] for row in labels)
    summary = {
        "schema_version": "table1_neardup_proxy_annotation_receipt_v1",
        "annotation_source": SOURCE_TAG,
        "human_annotation_claim": False,
        "rubric": {
            "low_duplicate_threshold": low,
            "high_not_duplicate_threshold": high,
            "rule": rule,
        },
        "task_count": len(labels),
        "label_counts": dict(sorted(counts.items())),
        "dataset_label_counts": {
            key: dict(sorted(value.items()))
            for key, value in sorted(by_dataset.items())
        },
        "selection_mode_label_counts": {
            key: dict(sorted(value.items()))
            for key, value in sorted(by_mode.items())
        },
        "distance_quantiles": {
            "min": min(distances),
            "q01": percentile(distances, 0.01),
            "q05": percentile(distances, 0.05),
            "q50": percentile(distances, 0.50),
            "q95": percentile(distances, 0.95),
            "q99": percentile(distances, 0.99),
            "max": max(distances),
        },
        "selection_limitation": (
            "The packet is a gated hard/random candidate sample and is not "
            "prevalence-weighted; scores remain diagnostic lower bounds."
        ),
    }
    return labels, summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--low", type=float, default=DEFAULT_LOW)
    parser.add_argument("--high", type=float, default=DEFAULT_HIGH)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not (0.0 < args.low < args.high):
        raise SystemExit("require 0 < --low < --high")
    labels, receipt = build_labels(iter_jsonl(args.audit), low=args.low, high=args.high)
    label_payload = b"".join(canonical_bytes(row) + b"\n" for row in labels)
    atomic_write(args.labels, label_payload)
    receipt["audit_path"] = str(args.audit.resolve())
    receipt["audit_sha256"] = sha256_file(args.audit)
    receipt["labels_path"] = str(args.labels.resolve())
    receipt["labels_sha256"] = sha256_bytes(label_payload)
    receipt["generator_script"] = str(Path(__file__).resolve())
    receipt["generator_script_sha256"] = sha256_file(Path(__file__))
    unsigned = dict(receipt)
    unsigned.pop("receipt_content_sha256", None)
    receipt["receipt_content_sha256"] = sha256_bytes(canonical_bytes(unsigned))
    atomic_write(
        args.receipt,
        json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=True).encode()
        + b"\n",
    )
    print(json.dumps({"labels": str(args.labels), "receipt": str(args.receipt), **receipt["label_counts"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
