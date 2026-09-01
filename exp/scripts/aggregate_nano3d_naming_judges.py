#!/usr/bin/env python3
"""Validate and aggregate three independent Nano3D Naming judge files.

The harness never creates semantic verdicts. It requires three externally
completed JSONL files, validates every field against the frozen queue, and
reports formal metrics only when the annotations needed by that metric are
complete. Partial runs are exposed as explicitly named coverage/lower-bound
or consensus-only diagnostics.
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
DEFAULT_QUEUE = EXP_ROOT / "runtime/nano3d_naming/judge_queue.jsonl"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_naming/judge_consensus.json"
ALLOWED = {"spec_match", "extra_real_part", "invalid_or_hallucinated", "uncertain"}
REAL_VERDICTS = {"spec_match", "extra_real_part"}
NONE = "none"
NOT_APPLICABLE = "not_applicable"


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
    return rows


def role_config(task: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["role"]): item for item in task["reference_roles"]}


def normalized_field(row: dict[str, Any], field: str) -> str | None:
    value = row.get(field)
    if value is None:
        return None
    value = str(value).strip()
    return value or None


def index_rows(
    path: Path, queue_by_id: dict[str, dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    rows = read_jsonl(path)
    indexed = {str(row.get("item_id")): row for row in rows}
    if len(indexed) != len(rows):
        raise RuntimeError(f"{path}: duplicate item_id")
    expected = set(queue_by_id)
    missing = sorted(expected - set(indexed))
    extra = sorted(set(indexed) - expected)
    if missing or extra:
        raise RuntimeError(f"{path}: item mismatch; missing={len(missing)}, extra={len(extra)}")

    for item_id, row in indexed.items():
        task = queue_by_id[item_id]
        verdict = normalized_field(row, "judge_verdict")
        matched_role = normalized_field(row, "judge_matched_role")
        instance_id = normalized_field(row, "judge_instance_id")
        same_part = normalized_field(row, "judge_same_semantic_part_as")
        if verdict not in ALLOWED:
            raise RuntimeError(f"{path}: {item_id}: invalid judge_verdict={verdict!r}")
        if not normalized_field(row, "judge_reason"):
            raise RuntimeError(f"{path}: {item_id}: judge_reason is required")

        roles = role_config(task)
        if verdict == "spec_match":
            if matched_role not in roles:
                raise RuntimeError(
                    f"{path}: {item_id}: judge_matched_role={matched_role!r} "
                    f"is not in reference roles {sorted(roles)}"
                )
            repeated = int(roles[matched_role]["min_count"]) > 1
            if repeated and instance_id in {None, NOT_APPLICABLE}:
                raise RuntimeError(
                    f"{path}: {item_id}: repeated role {matched_role!r} requires judge_instance_id"
                )
            if not repeated and instance_id != NOT_APPLICABLE:
                raise RuntimeError(
                    f"{path}: {item_id}: non-repeated role requires "
                    f"judge_instance_id={NOT_APPLICABLE!r}"
                )
        elif verdict in {"extra_real_part", "invalid_or_hallucinated"}:
            if matched_role is not None:
                raise RuntimeError(f"{path}: {item_id}: {verdict} requires judge_matched_role=null")
            if instance_id != NOT_APPLICABLE:
                raise RuntimeError(
                    f"{path}: {item_id}: {verdict} requires "
                    f"judge_instance_id={NOT_APPLICABLE!r}"
                )
        else:
            if any(value is not None for value in (matched_role, instance_id, same_part)):
                raise RuntimeError(
                    f"{path}: {item_id}: uncertain requires role/instance/same-part fields to be null"
                )

        if verdict in REAL_VERDICTS:
            allowed_targets = set(map(str, task["asset_mesh_node_names"])) - {str(task["node_name"])}
            if same_part != NONE and same_part not in allowed_targets:
                raise RuntimeError(
                    f"{path}: {item_id}: real part requires judge_same_semantic_part_as={NONE!r} "
                    "or another mesh node name in the same asset"
                )
        elif verdict == "invalid_or_hallucinated" and same_part != NOT_APPLICABLE:
            raise RuntimeError(
                f"{path}: {item_id}: invalid part requires "
                f"judge_same_semantic_part_as={NOT_APPLICABLE!r}"
            )
    return indexed


def majority(values: list[str | None], *, ignored: set[str | None] | None = None) -> str | None:
    ignored = ignored or {None}
    counts = Counter(value for value in values if value not in ignored)
    if not counts:
        return None
    value, count = counts.most_common(1)[0]
    return str(value) if count >= 2 else None


def verdict_consensus(rows: list[dict[str, Any]]) -> str | None:
    return majority(
        [normalized_field(row, "judge_verdict") for row in rows],
        ignored={None, "uncertain"},
    )


def field_consensus(
    rows: list[dict[str, Any]],
    field: str,
    verdict: str | None,
    *,
    matched_role: str | None = None,
) -> str | None:
    if verdict is None:
        return None
    values = []
    for row in rows:
        if normalized_field(row, "judge_verdict") != verdict:
            continue
        if matched_role is not None and normalized_field(row, "judge_matched_role") != matched_role:
            continue
        values.append(normalized_field(row, field))
    return majority(values)


def safe_mean(values: list[float]) -> float | None:
    return statistics.mean(values) if values else None


class UnionFind:
    def __init__(self, values: list[str]) -> None:
        self.parent = {value: value for value in values}

    def find(self, value: str) -> str:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--queue", type=Path, default=DEFAULT_QUEUE)
    parser.add_argument("--judge", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if len(args.judge) != 3:
        raise RuntimeError("exactly three independent --judge files are required")
    output = args.output.resolve()
    if EXP_ROOT not in output.parents:
        raise RuntimeError(f"output must remain under {EXP_ROOT}")

    queue = read_jsonl(args.queue.resolve())
    queue_by_id = {str(row["item_id"]): row for row in queue}
    if len(queue_by_id) != len(queue):
        raise RuntimeError(f"{args.queue}: duplicate item_id")
    judges = [index_rows(path.resolve(), queue_by_id) for path in args.judge]

    verdict_rows = []
    for item_id in sorted(queue_by_id):
        task = queue_by_id[item_id]
        vote_rows = [judge[item_id] for judge in judges]
        votes = [str(row["judge_verdict"]) for row in vote_rows]
        decided = verdict_consensus(vote_rows)
        decided_role = (
            field_consensus(vote_rows, "judge_matched_role", decided)
            if decided == "spec_match"
            else None
        )
        roles = role_config(task)
        repeated = bool(
            decided_role in roles and int(roles[decided_role]["min_count"]) > 1
        )
        decided_instance = (
            field_consensus(
                vote_rows,
                "judge_instance_id",
                decided,
                matched_role=decided_role,
            )
            if decided == "spec_match" and repeated
            else None
        )
        decided_same_part = (
            field_consensus(vote_rows, "judge_same_semantic_part_as", decided)
            if decided in REAL_VERDICTS
            else None
        )
        verdict_rows.append(
            {
                "item_id": item_id,
                "asset_id": task["asset_id"],
                "node_name": task["node_name"],
                "has_visual": task["has_visual"],
                "votes": votes,
                "consensus_verdict": decided,
                "consensus_matched_role": decided_role,
                "consensus_instance_id": decided_instance,
                "consensus_same_semantic_part_as": decided_same_part,
            }
        )

    decided_rows = [row for row in verdict_rows if row["consensus_verdict"] is not None]
    consensus_complete = len(decided_rows) == len(queue)
    real_rows = [row for row in decided_rows if row["consensus_verdict"] in REAL_VERDICTS]
    invalid_rows = [
        row for row in decided_rows if row["consensus_verdict"] == "invalid_or_hallucinated"
    ]
    spec_verdict_rows = [row for row in decided_rows if row["consensus_verdict"] == "spec_match"]
    spec_matches = [row for row in spec_verdict_rows if row["consensus_matched_role"]]
    role_annotation_coverage = (
        len(spec_matches) / len(spec_verdict_rows) if spec_verdict_rows else 1.0
    )
    role_consensus_complete = role_annotation_coverage == 1.0

    required_by_asset: dict[str, Counter[str]] = {}
    functional_by_asset: dict[str, Counter[str]] = {}
    repeated_by_asset: dict[str, Counter[str]] = {}
    for task in queue:
        asset_id = str(task["asset_id"])
        if asset_id in required_by_asset:
            continue
        required_by_asset[asset_id] = Counter(
            {str(item["role"]): int(item["min_count"]) for item in task["reference_roles"]}
        )
        functional_by_asset[asset_id] = Counter(
            {
                str(item["role"]): int(item["min_count"])
                for item in task["reference_roles"]
                if bool(item.get("functional"))
            }
        )
        repeated_by_asset[asset_id] = Counter(
            {
                str(item["role"]): int(item["min_count"])
                for item in task["reference_roles"]
                if int(item["min_count"]) > 1
            }
        )

    matched_by_asset: dict[str, Counter[str]] = defaultdict(Counter)
    real_mesh_by_asset: Counter[str] = Counter()
    extra_by_asset: Counter[str] = Counter()
    for row in spec_matches:
        matched_by_asset[row["asset_id"]][row["consensus_matched_role"]] += 1
    for row in real_rows:
        if row["has_visual"]:
            real_mesh_by_asset[row["asset_id"]] += 1
        if row["consensus_verdict"] == "extra_real_part":
            extra_by_asset[row["asset_id"]] += 1

    recalls: list[float] = []
    functional_coverages: list[float] = []
    richness: list[float] = []
    matched_total = required_total = 0
    functional_matched_total = functional_required_total = 0
    for asset_id, required in required_by_asset.items():
        matched = sum(
            min(count, matched_by_asset[asset_id].get(role, 0))
            for role, count in required.items()
        )
        denominator = sum(required.values())
        matched_total += matched
        required_total += denominator
        recalls.append(matched / denominator if denominator else 1.0)
        richness.append(real_mesh_by_asset[asset_id] / denominator if denominator else 0.0)

        functional = functional_by_asset[asset_id]
        functional_matched = sum(
            min(count, matched_by_asset[asset_id].get(role, 0))
            for role, count in functional.items()
        )
        functional_denominator = sum(functional.values())
        functional_matched_total += functional_matched
        functional_required_total += functional_denominator
        if functional_denominator:
            functional_coverages.append(functional_matched / functional_denominator)

    repeated_spec_matches = [
        row
        for row in spec_matches
        if row["consensus_matched_role"] in repeated_by_asset[row["asset_id"]]
    ]
    repeated_with_instance = [
        row for row in repeated_spec_matches if row["consensus_instance_id"] is not None
    ]
    instance_annotation_coverage = (
        len(repeated_with_instance) / len(repeated_spec_matches) if repeated_spec_matches else 1.0
    )
    instance_consensus_complete = instance_annotation_coverage == 1.0
    instance_keys: dict[tuple[str, str], set[str]] = defaultdict(set)
    for row in repeated_with_instance:
        instance_keys[(row["asset_id"], row["consensus_matched_role"])].add(
            row["consensus_instance_id"]
        )
    instance_distinguishable = 0
    instance_required = 0
    instance_groups = 0
    for asset_id, required in repeated_by_asset.items():
        for role, count in required.items():
            instance_groups += 1
            instance_required += count
            instance_distinguishable += min(len(instance_keys[(asset_id, role)]), count)
    instance_lower_bound = (
        instance_distinguishable / instance_required if instance_required else None
    )

    real_by_asset_node = {
        (str(row["asset_id"]), str(row["node_name"])): row for row in real_rows
    }
    same_part_consensus_rows = [
        row for row in real_rows if row["consensus_same_semantic_part_as"] is not None
    ]
    same_part_annotation_coverage = (
        len(same_part_consensus_rows) / len(real_rows) if real_rows else 1.0
    )
    unresolved_same_part_targets = []
    same_part_edges: list[tuple[str, str, str]] = []
    for row in same_part_consensus_rows:
        target = row["consensus_same_semantic_part_as"]
        if target == NONE:
            continue
        key = (str(row["asset_id"]), str(target))
        if key not in real_by_asset_node:
            unresolved_same_part_targets.append(
                {"asset_id": row["asset_id"], "node_name": row["node_name"], "target": target}
            )
            continue
        same_part_edges.append((str(row["asset_id"]), str(row["node_name"]), str(target)))
    same_part_consensus_complete = (
        same_part_annotation_coverage == 1.0 and not unresolved_same_part_targets
    )

    overseg_asset_rates: list[float] = []
    overseg_excess_fragments = 0
    overseg_real_nodes = len(real_rows)
    for asset_id in sorted(required_by_asset):
        nodes = sorted(
            str(row["node_name"]) for row in real_rows if row["asset_id"] == asset_id
        )
        if not nodes:
            overseg_asset_rates.append(0.0)
            continue
        union_find = UnionFind(nodes)
        for edge_asset, source, target in same_part_edges:
            if edge_asset == asset_id:
                union_find.union(source, target)
        component_sizes = Counter(union_find.find(node) for node in nodes)
        excess = sum(max(size - 1, 0) for size in component_sizes.values())
        overseg_excess_fragments += excess
        overseg_asset_rates.append(excess / len(nodes))
    overseg_micro_consensus_only = (
        overseg_excess_fragments / overseg_real_nodes if overseg_real_nodes else 0.0
    )
    overseg_macro_consensus_only = safe_mean(overseg_asset_rates)

    recall_ready = consensus_complete and role_consensus_complete
    instance_ready = recall_ready and instance_consensus_complete
    overseg_ready = consensus_complete and same_part_consensus_complete
    precision_consensus_only = len(real_rows) / len(decided_rows) if decided_rows else None
    recall_macro_lower_bound = safe_mean(recalls)
    recall_micro_lower_bound = matched_total / required_total if required_total else None
    functional_macro_lower_bound = safe_mean(functional_coverages)
    functional_micro_lower_bound = (
        functional_matched_total / functional_required_total
        if functional_required_total
        else None
    )
    richness_lower_bound = safe_mean(richness)
    richness_micro_lower_bound = (
        sum(real_mesh_by_asset.values()) / required_total if required_total else None
    )
    extra_real_parts_lower_bound = (
        sum(extra_by_asset.values()) / len(required_by_asset) if required_by_asset else None
    )

    result = {
        "protocol": "nano3d_naming_three_judge_consensus_v2.3",
        "judge_files": [str(path.resolve()) for path in args.judge],
        "queue_items": len(queue),
        "consensus_items": len(decided_rows),
        "consensus_coverage": len(decided_rows) / len(queue) if queue else None,
        "consensus_complete": consensus_complete,
        "role_consensus_items": len(spec_matches),
        "role_consensus_required_items": len(spec_verdict_rows),
        "role_annotation_coverage": role_annotation_coverage,
        "role_consensus_complete": role_consensus_complete,
        "semantic_precision": precision_consensus_only if consensus_complete else None,
        "semantic_precision_consensus_only": precision_consensus_only,
        "invalid_or_hallucinated_count": len(invalid_rows),
        "semantic_recall_macro": recall_macro_lower_bound if recall_ready else None,
        "semantic_recall_micro": recall_micro_lower_bound if recall_ready else None,
        "semantic_recall_macro_lower_bound": recall_macro_lower_bound,
        "semantic_recall_micro_lower_bound": recall_micro_lower_bound,
        "judge_validated_richness_mean": richness_lower_bound if consensus_complete else None,
        "judge_validated_richness_micro": richness_micro_lower_bound if consensus_complete else None,
        "judge_validated_richness_lower_bound": richness_lower_bound,
        "judge_validated_richness_micro_lower_bound": richness_micro_lower_bound,
        "extra_real_parts_per_asset": extra_real_parts_lower_bound if consensus_complete else None,
        "extra_real_parts_per_asset_lower_bound": extra_real_parts_lower_bound,
        "functional_core_coverage_macro": (
            functional_macro_lower_bound if recall_ready else None
        ),
        "functional_core_coverage_micro": (
            functional_micro_lower_bound if recall_ready else None
        ),
        "functional_core_coverage_macro_lower_bound": functional_macro_lower_bound,
        "functional_core_coverage_micro_lower_bound": functional_micro_lower_bound,
        "instance_annotation_items": len(repeated_with_instance),
        "instance_annotation_required_items": len(repeated_spec_matches),
        "instance_annotation_coverage": instance_annotation_coverage,
        "instance_consensus_complete": instance_consensus_complete,
        "instance_applicable_groups": instance_groups,
        "instance_distinguishable_count": instance_distinguishable,
        "instance_required_count": instance_required,
        "instance_discriminability": instance_lower_bound if instance_ready else None,
        "instance_discriminability_lower_bound": instance_lower_bound,
        "same_part_annotation_items": len(same_part_consensus_rows),
        "same_part_annotation_required_items": len(real_rows),
        "same_part_annotation_coverage": same_part_annotation_coverage,
        "same_part_consensus_complete": same_part_consensus_complete,
        "same_part_unresolved_target_count": len(unresolved_same_part_targets),
        "same_part_unresolved_targets": unresolved_same_part_targets,
        "over_segmentation_excess_fragments": overseg_excess_fragments,
        "over_segmentation_real_node_count": overseg_real_nodes,
        "over_segmentation_rate_macro": overseg_macro_consensus_only if overseg_ready else None,
        "over_segmentation_rate_micro": overseg_micro_consensus_only if overseg_ready else None,
        "over_segmentation_rate_macro_consensus_only": overseg_macro_consensus_only,
        "over_segmentation_rate_micro_consensus_only": overseg_micro_consensus_only,
        "verdicts": verdict_rows,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {key: value for key, value in result.items() if key not in {"verdicts", "same_part_unresolved_targets"}},
            indent=2,
            ensure_ascii=False,
        )
    )
    print(f"output={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
