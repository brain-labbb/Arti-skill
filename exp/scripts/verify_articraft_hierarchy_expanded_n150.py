#!/usr/bin/env python3
"""Fail-closed verification for the Articraft Table 3 N=150 expansion."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import hierarchy_extended_metrics as extended
import run_articraft_hierarchy_expanded_n150 as runner


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
DEFAULT_OUTPUT = WORKSPACE / "arti-skill/exp/runtime/nano3d_hierarchy_expanded_n150/articraft"
EXPECTED_CATEGORIES = {
    "storage_furniture_cabinet",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
}
ONTOLOGY_CATEGORIES = {
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
}
PER_CATEGORY = 30
REQUESTED = 150


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    if not resolved.is_relative_to(WORKSPACE):
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected object: {path}")
    return payload


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def check(condition: bool, message: str, checks: list[str]) -> None:
    if not condition:
        raise ValueError(message)
    checks.append(message)


def category_counts(rows: list[dict[str, Any]], key: str) -> Counter[str]:
    return Counter(str(row[key]) for row in rows)


def expected_counts(categories: set[str]) -> Counter[str]:
    return Counter({category: PER_CATEGORY for category in categories})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = contained(args.output)
    checks: list[str] = []

    selection_path = contained(output / "selection_manifest.jsonl")
    audit_path = contained(output / "selection_audit.json")
    records_path = contained(output / "records.jsonl")
    evaluation_path = contained(output / "evaluation_manifest.jsonl")
    extended_records_path = contained(output / "extended_structure_records.jsonl")
    extended_summary_path = contained(output / "extended_structure_summary.json")
    summary_path = contained(output / "summary.json")
    report_path = contained(output / "report.md")
    score_records_path = contained(output / "partnet_scores/records.jsonl")
    score_summary_path = contained(output / "partnet_scores/summary.json")

    selection = load_jsonl(selection_path)
    audit = load_json(audit_path)
    deterministic_selection, deterministic_audit = runner.frozen_selection()
    check(len(selection) == REQUESTED, "selection has exactly 150 rows", checks)
    check(selection == deterministic_selection, "selection equals deterministic recomputation", checks)
    check(
        category_counts(selection, "cohort") == expected_counts(EXPECTED_CATEGORIES),
        "selection has 30 rows in each of five cohorts",
        checks,
    )
    check(
        len({str(row["record_id"]) for row in selection}) == REQUESTED,
        "selection record IDs are globally unique",
        checks,
    )
    for category in EXPECTED_CATEGORIES:
        ranks = sorted(
            int(row["rank"]) for row in selection if row["cohort"] == category
        )
        check(ranks == list(range(1, 31)), f"{category} ranks are exactly 1..30", checks)
        check(
            audit["nesting_checks"][category]["original_is_exact_prefix"] is True,
            f"{category} original frozen-6 is declared exact prefix",
            checks,
        )
        check(
            audit["nesting_checks"][category]["original_record_ids"]
            == audit["nesting_checks"][category]["expanded_prefix_record_ids"],
            f"{category} original and expanded first-six IDs are identical",
            checks,
        )
    check(
        audit["selection_manifest_sha256"] == sha256(selection_path),
        "selection manifest hash matches audit",
        checks,
    )
    check(
        audit["released_manifest_sha256"] == deterministic_audit["released_manifest_sha256"],
        "released manifest hash matches deterministic audit",
        checks,
    )

    results = load_jsonl(records_path)
    check(len(results) == REQUESTED, "materialization has exactly 150 terminal rows", checks)
    check(
        [row["record_id"] for row in results] == [row["record_id"] for row in selection],
        "materialization rows preserve frozen selection order without replacement",
        checks,
    )
    check(
        all(row.get("materialization_mode") == "fresh_compile" for row in results),
        "all 150 rows are fresh compile attempts",
        checks,
    )
    check(all(bool(row.get("source_complete")) for row in results), "all selected sources are complete", checks)
    result_by_id = {str(row["record_id"]): row for row in results}
    for row in results:
        if row.get("final_urdf_present"):
            urdf = contained(Path(str(row["model_urdf"])))
            check(
                sha256(urdf) == row.get("model_urdf_sha256"),
                f"URDF hash matches for {row['record_id']}",
                checks,
            )
        else:
            check(
                not bool(row.get("parseable_final_urdf")),
                f"missing URDF is unavailable for {row['record_id']}",
                checks,
            )

    evaluation = load_jsonl(evaluation_path)
    check(len(evaluation) == REQUESTED, "evaluation manifest has exactly 150 rows", checks)
    check(
        category_counts(evaluation, "category") == expected_counts(ONTOLOGY_CATEGORIES),
        "evaluation manifest has 30 rows in each ontology category",
        checks,
    )
    check(
        len({str(row["sample_id"]) for row in evaluation}) == REQUESTED,
        "evaluation sample IDs are unique",
        checks,
    )
    for selected, evaluated in zip(selection, evaluation):
        source = result_by_id[str(selected["record_id"])]
        expected_available = bool(
            source["final_urdf_present"] and source["parseable_final_urdf"]
        )
        check(
            int(evaluated["selection_rank"]) == int(selected["rank"]),
            f"evaluation rank matches selection for {selected['record_id']}",
            checks,
        )
        check(
            bool(evaluated["available"]) == expected_available,
            f"evaluation availability matches terminal result for {selected['record_id']}",
            checks,
        )
        check(
            evaluated.get("urdf_sha256") == source.get("model_urdf_sha256"),
            f"evaluation URDF hash matches terminal result for {selected['record_id']}",
            checks,
        )

    extended_records = load_jsonl(extended_records_path)
    expected_extended = [row for row in results if row.get("parseable_final_urdf")]
    check(
        len(extended_records) == len(expected_extended),
        "extended records contain every and only parseable asset",
        checks,
    )
    extended_by_id = {str(row["record_id"]): row for row in extended_records}
    for source in expected_extended:
        observed = extended_by_id[str(source["record_id"])]
        recomputed = extended.analyze_urdf(contained(Path(str(source["model_urdf"]))))
        for field, value in recomputed.items():
            check(
                observed.get(field) == value,
                f"extended {field} recomputes for {source['record_id']}",
                checks,
            )
    extended_summary = load_json(extended_summary_path)
    check(extended_summary["requested_count"] == REQUESTED, "structure requested denominator is 150", checks)
    check(extended_summary["requested_per_category"] == PER_CATEGORY, "structure per-category denominator is 30", checks)
    check(
        extended_summary["records_sha256"] == sha256(extended_records_path),
        "extended-record hash matches structure summary",
        checks,
    )

    score_records = load_jsonl(score_records_path)
    score_summary = load_json(score_summary_path)
    check(len(score_records) == REQUESTED, "PartNet scorer emitted exactly 150 records", checks)
    check(
        [row["sample_id"] for row in score_records]
        == [row["sample_id"] for row in evaluation],
        "PartNet score records correspond one-to-one and in order with evaluation manifest",
        checks,
    )
    check(
        category_counts(score_records, "category") == expected_counts(ONTOLOGY_CATEGORIES),
        "PartNet scores have 30 rows in each ontology category",
        checks,
    )
    check(score_summary["overall"]["requested_count"] == REQUESTED, "PartNet requested denominator is 150", checks)
    check(
        score_summary["selection_manifest_sha256"] == sha256(evaluation_path),
        "PartNet scorer pins expanded evaluation manifest hash",
        checks,
    )
    scorer_core = contained(Path(str(score_summary["scorer_core"])))
    check(
        score_summary["scorer_sha256"] == sha256(scorer_core),
        "PartNet shared scorer module hash matches the current pinned module",
        checks,
    )
    check(
        score_summary["bootstrap"]["replicates"] == 10_000,
        "PartNet expanded bootstrap has 10000 replicates",
        checks,
    )
    check(
        score_summary["bootstrap"]["seed"] == 20260812,
        "PartNet expanded bootstrap seed is pinned",
        checks,
    )
    check(
        score_summary["requested_per_category"] == PER_CATEGORY,
        "PartNet expanded per-category denominator is 30",
        checks,
    )

    summary = load_json(summary_path)
    check(summary["protocol_id"] == runner.PROTOCOL, "summary protocol ID matches runner", checks)
    check(summary["selection"]["manifest_sha256"] == sha256(selection_path), "summary pins selection hash", checks)
    check(summary["materialization"]["requested_count"] == REQUESTED, "summary materialization denominator is 150", checks)
    check(summary["materialization"]["fresh_compile_attempt_count"] == REQUESTED, "summary reports 150 fresh attempts", checks)
    check(summary["hashes"]["records"] == sha256(records_path), "summary pins terminal-record hash", checks)
    check(summary["hashes"]["evaluation_manifest"] == sha256(evaluation_path), "summary pins evaluation hash", checks)
    check(summary["alignment"]["status"] == "COMPLETE", "summary marks alignment complete", checks)
    check(
        summary["alignment"]["summary"] == score_summary,
        "top-level summary embeds the exact PartNet score summary",
        checks,
    )
    check(
        summary["hashes"]["alignment_records"] == sha256(score_records_path),
        "top-level summary pins PartNet records hash",
        checks,
    )
    check(
        summary["hashes"]["alignment_summary"] == sha256(score_summary_path),
        "top-level summary pins PartNet summary hash",
        checks,
    )
    check(report_path.stat().st_size > 0, "report is nonempty", checks)

    verification = {
        "status": "PASS",
        "protocol_id": runner.PROTOCOL,
        "requested_count": REQUESTED,
        "per_category_requested": PER_CATEGORY,
        "check_count": len(checks),
        "selection_manifest_sha256": sha256(selection_path),
        "records_sha256": sha256(records_path),
        "evaluation_manifest_sha256": sha256(evaluation_path),
        "extended_records_sha256": sha256(extended_records_path),
        "partnet_records_sha256": sha256(score_records_path),
        "partnet_summary_sha256": sha256(score_summary_path),
        "summary_sha256": sha256(summary_path),
        "checks": checks,
    }
    verification_path = contained(output / "verification.json", exists=False)
    verification_path.write_text(
        json.dumps(verification, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(verification, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
