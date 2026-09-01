#!/usr/bin/env python3
"""Verify frozen inputs and recompute all PartNet semantic-alignment aggregates."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from partnet_hierarchy_correctness import aggregate


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
COMBINED = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/combined"
METHOD_DIRS = {
    "PV-A": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/partnet_scores",
    "LAM": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam/partnet_scores",
    "Articraft": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/articraft/partnet_scores",
    "Infinite Mobility": (
        EXP_ROOT
        / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/partnet_scores"
    ),
}
CATEGORIES = {
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
}


def contained(path: Path) -> Path:
    resolved = path.resolve(strict=True)
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


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: {actual!r} != {expected!r}")


def main() -> int:
    protocol_path = contained(PROTOCOL)
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    protocol_hash = sha256(protocol_path)
    checks = []

    for category, definition in protocol["categories"].items():
        ontology_roles = set(definition["ontology"]["roles"])
        rule_roles = {rule["role"] for rule in definition["ordered_role_rules"]}
        assert_equal(rule_roles - ontology_roles, set(), f"{category} unknown rule roles")
    checks.append("all lexical rule roles exist in their official ontology")

    scorer_hashes = set()
    method_values = {}
    for method, directory in METHOD_DIRS.items():
        score_dir = contained(directory)
        summary_path = contained(score_dir / "summary.json")
        records_path = contained(score_dir / "records.jsonl")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        records = load_jsonl(records_path)
        assert_equal(len(records), 30, f"{method} requested count")
        assert_equal({row["category"] for row in records}, CATEGORIES, f"{method} categories")
        for category in CATEGORIES:
            category_records = [row for row in records if row["category"] == category]
            assert_equal(len(category_records), 6, f"{method}/{category} count")
            assert_equal(
                aggregate(category_records),
                summary["per_category"][category],
                f"{method}/{category} aggregate",
            )
        assert_equal(aggregate(records), summary["overall"], f"{method} overall aggregate")
        assert_equal(summary["partnet_protocol_sha256"], protocol_hash, f"{method} protocol hash")
        scorer_hashes.add(summary["scorer_sha256"])
        method_values[method] = summary["overall"]
    assert_equal(len(scorer_hashes), 1, "shared scorer hash count")
    checks.append("all four 30-asset panels recompute exactly with one scorer/protocol")

    name_only_path = contained(
        EXP_ROOT
        / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel"
        / "partnet_scores_name_only/summary.json"
    )
    name_only = json.loads(name_only_path.read_text(encoding="utf-8"))
    assert_equal(
        name_only["overall"]["semantic_role_coverage_requested_macro"],
        0.0,
        "Infinite Mobility name-only role coverage",
    )
    assert_equal(
        name_only["prediction_side_link_label_count"],
        0,
        "Infinite Mobility name-only package labels",
    )
    checks.append("Infinite Mobility name-only sensitivity has zero mapped-role coverage")

    combined_path = contained(COMBINED / "summary.json")
    combined = json.loads(combined_path.read_text(encoding="utf-8"))
    assert_equal(combined["partnet_protocol_sha256"], protocol_hash, "combined protocol hash")
    assert_equal(combined["scorer_sha256"], next(iter(scorer_hashes)), "combined scorer hash")
    for method, values in method_values.items():
        assert_equal(combined["methods"][method]["overall"], values, f"{method} combined copy")
    checks.append("combined summary matches every method summary")

    expected_primary = {
        "PV-A": 0.5012433862433863,
        "LAM": 0.5177771425162484,
        "Articraft": 0.4377554381348499,
        "Infinite Mobility": 0.04526455026455026,
    }
    for method, expected in expected_primary.items():
        assert_equal(
            method_values[method]["coverage_weighted_induced_edge_f1_requested_macro"],
            expected,
            f"{method} primary metric",
        )
    checks.append("paper-table primary values match frozen summaries")

    result = {
        "status": "PASS",
        "check_count": len(checks),
        "checks": checks,
        "partnet_protocol_sha256": protocol_hash,
        "scorer_sha256": next(iter(scorer_hashes)),
        "combined_summary_sha256": sha256(combined_path),
    }
    output = COMBINED / "self_check.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
