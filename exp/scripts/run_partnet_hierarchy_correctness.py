#!/usr/bin/env python3
"""Score a standardized 30-row URDF manifest against frozen PartNet ontology."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from partnet_hierarchy_correctness import aggregate, evaluate_urdf, load_protocol


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument(
        "--link-labels",
        type=Path,
        help="Optional prediction-side package link labels; never ontology gold.",
    )
    args = parser.parse_args()
    manifest_path = contained(args.manifest)
    protocol_path = contained(args.protocol)
    output = contained(args.output, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    protocol = load_protocol(protocol_path)
    rows = load_jsonl(manifest_path)
    for row in rows:
        category = str(row["category"])
        row["category"] = {
            "storage_furniture_cabinet": "storage_furniture"
        }.get(category, category)
    if len(rows) != 30:
        raise ValueError(f"expected frozen requested N=30, found {len(rows)}")
    methods = {str(row["method"]) for row in rows}
    if len(methods) != 1:
        raise ValueError(f"expected one method per manifest, found {sorted(methods)}")
    if len({str(row["sample_id"]) for row in rows}) != len(rows):
        raise ValueError("duplicate sample_id")
    expected_categories = set(protocol["categories"])
    category_counts = {
        category: sum(str(row["category"]) == category for row in rows)
        for category in expected_categories
    }
    unexpected_categories = sorted(
        {str(row["category"]) for row in rows} - expected_categories
    )
    if unexpected_categories or set(category_counts.values()) != {6}:
        raise ValueError(
            "expected exactly six requested assets in each protocol category; "
            f"counts={category_counts}, unexpected={unexpected_categories}"
        )
    for category in expected_categories:
        ranks = sorted(
            int(row["selection_rank"])
            for row in rows
            if str(row["category"]) == category
        )
        if ranks not in (list(range(6)), list(range(1, 7))):
            raise ValueError(
                f"{category} selection ranks must be contiguous 0..5 or 1..6, found {ranks}"
            )

    labels_by_sample: dict[str, dict[str, str]] = {}
    link_labels_sha256 = None
    if args.link_labels is not None:
        link_labels_path = contained(args.link_labels)
        link_labels_sha256 = sha256(link_labels_path)
        label_rows = json.loads(link_labels_path.read_text(encoding="utf-8"))
        if not isinstance(label_rows, list):
            raise ValueError("link-label file must contain a JSON list")
        for label_row in label_rows:
            if label_row.get("assignment_status") != "PREDICTION_ONLY_NOT_GOLD":
                raise ValueError("link labels must be marked PREDICTION_ONLY_NOT_GOLD")
            sample_id = str(label_row["sample_id"])
            link_name = str(label_row["link_name"])
            raw_part_name = str(label_row["raw_part_name"])
            sample_labels = labels_by_sample.setdefault(sample_id, {})
            if link_name in sample_labels:
                raise ValueError(f"duplicate link label for {sample_id}/{link_name}")
            sample_labels[link_name] = raw_part_name

    records = []
    for source in rows:
        row = dict(source)
        row["evaluation_complete"] = False
        if source.get("available"):
            urdf = contained(Path(str(source["urdf_path"])))
            current_hash = sha256(urdf)
            if current_hash != source["urdf_sha256"]:
                raise ValueError(f"URDF hash mismatch for {source['sample_id']}")
            try:
                row.update(
                    evaluate_urdf(
                        urdf,
                        str(row["category"]),
                        protocol,
                        labels_by_sample.get(str(source["sample_id"])),
                    )
                )
                row["evaluation_complete"] = True
                row["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        else:
            row["evaluation_error"] = source.get("unavailable_reason", "unavailable")
        records.append(row)

    categories = sorted({str(row["category"]) for row in rows})
    per_category = {
        category: aggregate([row for row in records if row["category"] == category])
        for category in categories
    }
    summary = {
        "protocol_id": "nano3d_partnet_hierarchy_correctness_v1",
        "method": next(iter(methods)),
        "claim_boundary": protocol["claim_boundary"],
        "selection_manifest_sha256": sha256(manifest_path),
        "partnet_protocol_sha256": sha256(protocol_path),
        "scorer_sha256": sha256(Path(__file__).with_name("partnet_hierarchy_correctness.py")),
        "prediction_side_link_labels_sha256": link_labels_sha256,
        "prediction_side_link_label_count": sum(
            int(row.get("package_part_label_count", 0)) for row in records
        ),
        "overall": aggregate(records),
        "per_category": per_category,
    }
    (output / "records.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in records),
        encoding="utf-8",
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
