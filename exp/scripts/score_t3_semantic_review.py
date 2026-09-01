#!/usr/bin/env python3
"""Score the two blinded model reviews used by the formal T3 experiment.

The two reviewers are reported separately because they independently induce the
expected role/edge ontology from the render.  Agreement is computed only on the
shared emitted parts and edges, for which the labels are directly comparable.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
ROOT = EXP_ROOT / "runtime/t3_formal_v1/semantic_review"


def ratio(num: int, den: int) -> float | None:
    return num / den if den else None


def wilson(successes: int, total: int, z: float = 1.959963984540054) -> list[float] | None:
    if total == 0:
        return None
    p = successes / total
    d = 1.0 + z * z / total
    centre = (p + z * z / (2.0 * total)) / d
    half = z * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / d
    return [max(0.0, centre - half), min(1.0, centre + half)]


def f1(precision: float | None, recall: float | None) -> float | None:
    if precision is None or recall is None:
        return None
    return 0.0 if precision + recall == 0 else 2.0 * precision * recall / (precision + recall)


def load_reviewer(name: str) -> dict[str, Any]:
    path = ROOT / f"reviewer_{name}.json"
    edge_run = ROOT / f"reviewer_{name}_edge_run.json"
    if not edge_run.exists():
        raise RuntimeError(f"edge augmentation is not complete for {name}: {edge_run}")
    run = json.loads(edge_run.read_text(encoding="utf-8"))
    if run.get("process_exit_code") != 0 or not run.get("valid"):
        raise RuntimeError(f"invalid edge augmentation for {name}: {run}")
    return json.loads(path.read_text(encoding="utf-8"))


def score_one(name: str, payload: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    counts = {
        "valid_parts": 0,
        "emitted_parts": 0,
        "matched_role_instances": 0,
        "expected_role_instances": 0,
        "correct_emitted_edges": 0,
        "emitted_edges": 0,
        "matched_expected_edges": 0,
        "expected_edges": 0,
        "hierarchy_exact_assets": 0,
        "instance_discriminable_assets": 0,
        "assets": 0,
    }
    labels: dict[str, Any] = {}
    per_panel: list[dict[str, Any]] = []
    problems: list[str] = []
    for panel in payload.get("panels", []):
        pid = panel["panel_id"]
        fields = panel["review_fields"]
        emitted_parts = panel["emitted_parts"]
        emitted_edges = {tuple(row) for row in panel["emitted_parent_child_edges"]}
        part_labels = {row["name"]: bool(row["semantically_valid"]) for row in fields["parts"]}
        edge_labels = {
            (row["parent"], row["child"]): bool(row["semantically_correct"])
            for row in fields["edges"]
        }
        if set(part_labels) != set(emitted_parts):
            problems.append(f"{pid}: part labels do not cover emitted parts exactly")
        if set(edge_labels) != emitted_edges:
            problems.append(f"{pid}: edge labels do not cover emitted edges exactly")

        role_expected = 0
        role_matched = 0
        for row in fields["expected_visible_or_functional_roles"]:
            minimum = int(row["min_instances"])
            matched = len(set(row["matched_parts"]) & set(emitted_parts))
            role_expected += minimum
            role_matched += min(minimum, matched)

        expected_edges = fields.get("expected_parent_child_edges")
        if not isinstance(expected_edges, list):
            problems.append(f"{pid}: missing expected_parent_child_edges")
            expected_edges = []
        edge_recall_tp = sum(bool(row["matched_emitted_edge"]) for row in expected_edges)
        correct_emitted = sum(edge_labels.values())

        counts["valid_parts"] += sum(part_labels.values())
        counts["emitted_parts"] += len(emitted_parts)
        counts["matched_role_instances"] += role_matched
        counts["expected_role_instances"] += role_expected
        counts["correct_emitted_edges"] += correct_emitted
        counts["emitted_edges"] += len(emitted_edges)
        counts["matched_expected_edges"] += edge_recall_tp
        counts["expected_edges"] += len(expected_edges)
        counts["hierarchy_exact_assets"] += bool(fields["hierarchy_exact_match"])
        counts["instance_discriminable_assets"] += bool(fields["instance_discriminability"])
        counts["assets"] += 1

        part_p = ratio(sum(part_labels.values()), len(emitted_parts))
        part_r = ratio(role_matched, role_expected)
        edge_p = ratio(correct_emitted, len(emitted_edges))
        edge_r = ratio(edge_recall_tp, len(expected_edges))
        per_panel.append(
            {
                "panel_id": pid,
                "asset_id": panel["asset_id"],
                "part_precision": part_p,
                "role_instance_recall": part_r,
                "part_f1": f1(part_p, part_r),
                "edge_precision": edge_p,
                "edge_recall": edge_r,
                "edge_f1": f1(edge_p, edge_r),
                "hierarchy_exact_match": bool(fields["hierarchy_exact_match"]),
                "instance_discriminability": bool(fields["instance_discriminability"]),
            }
        )
        labels[pid] = {
            "parts": part_labels,
            "edges": {"\u0000".join(edge): value for edge, value in edge_labels.items()},
            "hierarchy_exact_match": bool(fields["hierarchy_exact_match"]),
            "instance_discriminability": bool(fields["instance_discriminability"]),
        }

    if problems:
        raise RuntimeError(f"invalid review by {name}: " + "; ".join(problems))
    part_precision = ratio(counts["valid_parts"], counts["emitted_parts"])
    role_recall = ratio(counts["matched_role_instances"], counts["expected_role_instances"])
    edge_precision = ratio(counts["correct_emitted_edges"], counts["emitted_edges"])
    edge_recall = ratio(counts["matched_expected_edges"], counts["expected_edges"])
    metrics = {
        "reviewer": name,
        "counts": counts,
        "metrics": {
            "part_precision": part_precision,
            "part_precision_wilson_95": wilson(counts["valid_parts"], counts["emitted_parts"]),
            "role_instance_recall": role_recall,
            "role_instance_recall_wilson_95": wilson(
                counts["matched_role_instances"], counts["expected_role_instances"]
            ),
            "part_f1": f1(part_precision, role_recall),
            "edge_precision": edge_precision,
            "edge_precision_wilson_95": wilson(
                counts["correct_emitted_edges"], counts["emitted_edges"]
            ),
            "edge_recall": edge_recall,
            "edge_recall_wilson_95": wilson(
                counts["matched_expected_edges"], counts["expected_edges"]
            ),
            "edge_f1": f1(edge_precision, edge_recall),
            "hierarchy_exact_asset_rate": ratio(
                counts["hierarchy_exact_assets"], counts["assets"]
            ),
            "instance_discriminability_asset_rate": ratio(
                counts["instance_discriminable_assets"], counts["assets"]
            ),
        },
        "per_panel": per_panel,
    }
    return metrics, labels


def agreement(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    part_equal = part_total = edge_equal = edge_total = 0
    hierarchy_equal = instance_equal = panels = 0
    for pid in sorted(set(a) & set(b)):
        panels += 1
        for key in sorted(set(a[pid]["parts"]) & set(b[pid]["parts"])):
            part_total += 1
            part_equal += a[pid]["parts"][key] == b[pid]["parts"][key]
        for key in sorted(set(a[pid]["edges"]) & set(b[pid]["edges"])):
            edge_total += 1
            edge_equal += a[pid]["edges"][key] == b[pid]["edges"][key]
        hierarchy_equal += a[pid]["hierarchy_exact_match"] == b[pid]["hierarchy_exact_match"]
        instance_equal += (
            a[pid]["instance_discriminability"] == b[pid]["instance_discriminability"]
        )
    return {
        "shared_panels": panels,
        "part_label_exact_agreement": ratio(part_equal, part_total),
        "part_labels_equal": part_equal,
        "part_labels_compared": part_total,
        "edge_label_exact_agreement": ratio(edge_equal, edge_total),
        "edge_labels_equal": edge_equal,
        "edge_labels_compared": edge_total,
        "hierarchy_asset_label_agreement": ratio(hierarchy_equal, panels),
        "instance_discriminability_label_agreement": ratio(instance_equal, panels),
        "note": "Recall ontologies are reviewer-induced and therefore are not pairwise aligned.",
    }


def main() -> int:
    results = []
    labels = []
    for name in ("claude", "codex"):
        result, label = score_one(name, load_reviewer(name))
        results.append(result)
        labels.append(label)
    summary = {
        "schema_version": "t3-semantic-model-review-score-v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "review_type": "two independent blinded model reviewers; not human annotation",
        "reviewers": results,
        "inter_reviewer_agreement": agreement(labels[0], labels[1]),
        "interpretation": {
            "part_precision": "semantically valid emitted parts / all emitted parts",
            "role_instance_recall": "matched required role instances / expected role instances",
            "edge_precision": "semantically correct emitted directed edges / emitted edges",
            "edge_recall": "matched required directed relations / reviewer-expected relations",
            "unmatched_or_incorrect_items": "counted as failures, never silently omitted",
        },
    }
    path = ROOT / "semantic_scores.json"
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(path), "reviewers": [r["reviewer"] for r in results]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
