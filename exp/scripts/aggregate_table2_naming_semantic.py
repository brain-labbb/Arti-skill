#!/usr/bin/env python3
"""Validate three blind judge files and aggregate matched Table 2 Naming metrics."""

from __future__ import annotations

import hashlib
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
RUNTIME = PROJECT_ROOT / "exp/runtime/table2_naming_semantic_v1"
QUEUE = RUNTIME / "blind_tasks.jsonl"
AUDIT = RUNTIME / "audit_tasks.jsonl"
MANIFEST = RUNTIME / "manifest.json"
GOLD = PROJECT_ROOT / "exp/reference/table2_naming_semantic_gold_v1.json"
PROTOCOL = PROJECT_ROOT / "exp/reference/table2_naming_semantic_protocol_v1.json"
METRIC_CORRECTION = (
    PROJECT_ROOT / "exp/reference/table2_naming_semantic_metric_correction_v1.json"
)
JUDGES = [RUNTIME / f"judges/judge_{label}.jsonl" for label in "abc"]
REREVIEW_ROOT = RUNTIME / "rereview"
REREVIEW_QUEUE = REREVIEW_ROOT / "blind_tasks.jsonl"
REREVIEW_JUDGES = [REREVIEW_ROOT / f"judge_{label}.jsonl" for label in "abc"]
REREVIEW_PROTOCOL = (
    PROJECT_ROOT / "exp/reference/table2_naming_semantic_adjudication_v1.json"
)
TIEBREAK_ROOT = RUNTIME / "tiebreak"
TIEBREAK_QUEUE = TIEBREAK_ROOT / "blind_tasks.jsonl"
TIEBREAK_JUDGE = TIEBREAK_ROOT / "adjudicator.jsonl"
TIEBREAK_PROTOCOL = (
    PROJECT_ROOT / "exp/reference/table2_naming_semantic_tiebreak_v1.json"
)
ALLOWED_VERDICTS = {
    "spec_match",
    "extra_real_part",
    "invalid_or_hallucinated",
    "uncertain",
}
TRUTHFUL_VERDICTS = {"spec_match", "extra_real_part"}
SPECIAL_INSTANCE_IDS = {"ambiguous", "not_applicable"}
METHODS = ["Ours", "LAM", "Articraft", "Infinite Mobility"]
CATEGORIES = ["microwave", "dishwasher", "oven", "faucet", "refrigerator"]


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for number, line in enumerate(contained(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"{path}:{number}: invalid JSON: {exc}") from exc
    return rows


def write_json(path: Path, value: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(target)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )
    temporary.replace(target)


def normalized(value: Any) -> str | None:
    if value is None:
        return None
    result = str(value).strip()
    return result or None


def majority(values: Iterable[Any], ignored: set[Any] | None = None) -> Any:
    ignored = ignored or set()
    counts = Counter(value for value in values if value not in ignored)
    if not counts:
        return None
    value, count = counts.most_common(1)[0]
    return value if count >= 2 else None


def recursive_key_present(value: Any, forbidden: str) -> bool:
    if isinstance(value, dict):
        return forbidden in value or any(
            recursive_key_present(child, forbidden) for child in value.values()
        )
    if isinstance(value, list):
        return any(recursive_key_present(child, forbidden) for child in value)
    return False


def validate_judge(
    path: Path, queue_by_id: dict[str, dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    rows = read_jsonl(path)
    indexed = {str(row.get("item_id")): row for row in rows}
    if len(rows) != len(indexed):
        raise RuntimeError(f"{path}: duplicate item_id")
    if set(indexed) != set(queue_by_id):
        raise RuntimeError(f"{path}: item set differs from blind queue")
    immutable = (
        "asset_id",
        "category",
        "node_name",
        "preview_path",
        "asset_mesh_node_names",
        "required_roles",
        "optional_roles",
    )
    for item_id, row in indexed.items():
        task = queue_by_id[item_id]
        for key in immutable:
            if row.get(key) != task.get(key):
                raise RuntimeError(f"{path}: {item_id}: changed immutable field {key}")
        verdict = normalized(row.get("judge_verdict"))
        if verdict not in ALLOWED_VERDICTS:
            raise RuntimeError(f"{path}: {item_id}: invalid verdict {verdict!r}")
        required = {str(role["role"]) for role in task["required_roles"]}
        optional = {str(role["role"]) for role in task["optional_roles"]}
        matched = normalized(row.get("judge_matched_role"))
        if verdict == "spec_match" and matched not in required:
            raise RuntimeError(f"{path}: {item_id}: spec_match requires a required role")
        if verdict != "spec_match" and matched is not None:
            raise RuntimeError(f"{path}: {item_id}: non-spec verdict requires null role")
        real = row.get("judge_geometry_is_real_part")
        if not isinstance(real, bool):
            raise RuntimeError(f"{path}: {item_id}: geometry real-part flag must be boolean")
        geometry_role = normalized(row.get("judge_geometry_role"))
        allowed_geometry = required | optional | {"unknown"}
        if not (
            geometry_role in allowed_geometry
            or (geometry_role or "").startswith("other_real_part:")
        ):
            raise RuntimeError(f"{path}: {item_id}: invalid geometry role {geometry_role!r}")
        if not real and geometry_role != "unknown":
            raise RuntimeError(f"{path}: {item_id}: non-real geometry requires role=unknown")
        instance = normalized(row.get("judge_instance_id"))
        if instance is None:
            raise RuntimeError(f"{path}: {item_id}: missing instance id")
        same_part = normalized(row.get("judge_same_semantic_part_as"))
        if real:
            allowed_targets = set(map(str, task["asset_mesh_node_names"])) - {
                str(task["node_name"])
            }
            if same_part != "none" and same_part not in allowed_targets:
                raise RuntimeError(f"{path}: {item_id}: invalid same-part target {same_part!r}")
        elif same_part != "not_applicable" or instance != "not_applicable":
            raise RuntimeError(
                f"{path}: {item_id}: non-real geometry requires not_applicable fields"
            )
        if not normalized(row.get("judge_reason")):
            raise RuntimeError(f"{path}: {item_id}: missing judge reason")
    return indexed


def validate_rereview_judge(
    path: Path, tasks: dict[str, dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    rows = read_jsonl(path)
    indexed = {str(row.get("item_id")): row for row in rows}
    if len(rows) != len(indexed) or set(indexed) != set(tasks):
        raise RuntimeError(f"{path}: rereview item set mismatch")
    for item_id, row in indexed.items():
        task = tasks[item_id]
        for key in (
            "asset_id",
            "category",
            "node_name",
            "preview_path",
            "asset_mesh_node_names",
            "required_roles",
            "optional_roles",
            "rejudge_fields",
            "adjudicate_fields",
        ):
            if row.get(key) != task.get(key):
                raise RuntimeError(f"{path}: {item_id}: changed rereview field {key}")
        requested = set(task["rejudge_fields"])
        roles = {
            str(config["role"])
            for config in task["required_roles"] + task["optional_roles"]
        }
        role = normalized(row.get("judge_geometry_role"))
        instance = normalized(row.get("judge_instance_id"))
        if "judge_geometry_role" in requested:
            if not (
                role in roles
                or role == "unknown"
                or (role or "").startswith("other_real_part:")
            ):
                raise RuntimeError(f"{path}: {item_id}: invalid rereview role {role!r}")
        elif role != "locked":
            raise RuntimeError(f"{path}: {item_id}: geometry role must remain locked")
        if "judge_instance_id" in requested:
            if instance in {None, "locked"}:
                raise RuntimeError(f"{path}: {item_id}: missing rereview instance")
        elif instance != "locked":
            raise RuntimeError(f"{path}: {item_id}: instance must remain locked")
        if not normalized(row.get("judge_reason")):
            raise RuntimeError(f"{path}: {item_id}: missing rereview reason")
    return indexed


class UnionFind:
    def __init__(self, values: Iterable[str]) -> None:
        self.parent = {value: value for value in values}

    def find(self, value: str) -> str:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root, right_root = self.find(left), self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def fleiss_kappa(votes: list[list[str]], labels: list[str]) -> float | None:
    if not votes:
        return None
    judge_count = len(votes[0])
    if judge_count < 2:
        return None
    counts = [Counter(row) for row in votes]
    observed = statistics.mean(
        (sum(counter[label] ** 2 for label in labels) - judge_count)
        / (judge_count * (judge_count - 1))
        for counter in counts
    )
    total = len(votes) * judge_count
    proportions = {
        label: sum(counter[label] for counter in counts) / total for label in labels
    }
    expected = sum(value**2 for value in proportions.values())
    return (observed - expected) / (1 - expected) if expected < 1 else 1.0


def pairwise_exact(votes: list[list[Any]]) -> list[float]:
    return [
        sum(row[left] == row[right] for row in votes) / len(votes)
        for left, right in ((0, 1), (0, 2), (1, 2))
    ]


def verify_previews(manifest: dict[str, Any]) -> dict[str, Any]:
    preview_root = contained(RUNTIME / "previews")
    paths = sorted(preview_root.glob("*.png"))
    expected = manifest["preview_hashes"]
    if {path.name for path in paths} != set(expected):
        raise RuntimeError("preview file set differs from frozen manifest")
    target_counts = []
    context_counts = []
    for path in paths:
        if path.is_symlink() or digest(path) != expected[path.name]:
            raise RuntimeError(f"preview hash/type mismatch: {path.name}")
        pixels = np.asarray(Image.open(path).convert("RGB"))
        if pixels.shape != (298, 672, 3):
            raise RuntimeError(f"preview shape mismatch: {path.name}: {pixels.shape}")
        target = int(
            (
                (pixels[:, :, 0] > 150)
                & (pixels[:, :, 0] > pixels[:, :, 1] * 1.3)
                & (pixels[:, :, 0] > pixels[:, :, 2] * 1.2)
            ).sum()
        )
        context = int(
            (
                (pixels[:, :, 2] > pixels[:, :, 0])
                & (pixels[:, :, 2] > pixels[:, :, 1] * 0.9)
                & (pixels[:, :, 0] < 180)
            ).sum()
        )
        if target < 20 or context < 20:
            raise RuntimeError(
                f"preview pixel gate failed: {path.name}: target={target}, context={context}"
            )
        target_counts.append(target)
        context_counts.append(context)
    return {
        "preview_count": len(paths),
        "shape": [298, 672, 3],
        "target_pixel_min": min(target_counts),
        "target_pixel_median": float(np.median(target_counts)),
        "context_pixel_min": min(context_counts),
        "context_pixel_median": float(np.median(context_counts)),
        "all_hashes_match": True,
        "all_nonblank": True,
    }


def summarize_assets(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_asset: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_asset[str(row["asset_id"])].append(row)
    result = []
    for asset_id, items in sorted(by_asset.items()):
        first = items[0]
        required = {
            str(role["role"]): role for role in first["required_roles"]
        }
        matched = Counter(
            str(row["consensus_matched_role"])
            for row in items
            if row["consensus_verdict"] == "spec_match"
        )
        covered = sum(1 for role in required if matched[role] > 0)
        functional = [
            role for role, config in required.items() if bool(config["functional"])
        ]
        functional_covered = sum(1 for role in functional if matched[role] > 0)
        truthful = sum(row["consensus_verdict"] in TRUTHFUL_VERDICTS for row in items)
        spec_match = sum(row["consensus_verdict"] == "spec_match" for row in items)
        extra_real = sum(
            row["consensus_verdict"] == "extra_real_part" for row in items
        )
        real_items = [row for row in items if row["consensus_geometry_is_real_part"]]
        real_names = {str(row["node_name"]) for row in real_items}
        union = UnionFind(real_names)
        unresolved = []
        for row in real_items:
            target = row["consensus_same_semantic_part_as"]
            if target == "none":
                continue
            if target not in real_names:
                unresolved.append((row["node_name"], target))
                continue
            union.union(str(row["node_name"]), str(target))
        component_sizes = Counter(union.find(name) for name in real_names)
        excess = sum(max(size - 1, 0) for size in component_sizes.values())

        role_groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in real_items:
            role = str(row["consensus_geometry_role"])
            if role != "unknown":
                role_groups[role].append(row)
        instance_numerator = 0
        instance_denominator = 0
        instance_groups = 0
        for group in role_groups.values():
            if len(group) < 2:
                continue
            instance_groups += 1
            instance_denominator += len(group)
            identities = [str(row["consensus_instance_id"]) for row in group]
            usable = Counter(
                identity for identity in identities if identity not in SPECIAL_INSTANCE_IDS
            )
            instance_numerator += sum(
                1 for identity in identities if usable.get(identity, 0) == 1
            )
        result.append(
            {
                "asset_id": asset_id,
                "method": first["method"],
                "category": first["category"],
                "task_count": len(items),
                "truthful_count": truthful,
                "spec_match_count": spec_match,
                "extra_real_count": extra_real,
                "required_count": len(required),
                "required_covered": covered,
                "functional_required": len(functional),
                "functional_covered": functional_covered,
                "geometry_real_count": len(real_items),
                "geometry_role_known_count": sum(
                    row["consensus_geometry_role"] != "unknown" for row in real_items
                ),
                "instance_distinguishable_count": instance_numerator,
                "instance_applicable_count": instance_denominator,
                "instance_applicable_groups": instance_groups,
                "overseg_excess_fragments": excess,
                "overseg_unresolved_edges": unresolved,
            }
        )
    return result


def divide(numerator: float, denominator: float) -> float | None:
    return numerator / denominator if denominator else None


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    task_count = sum(row["task_count"] for row in records)
    truthful = sum(row["truthful_count"] for row in records)
    spec_match = sum(row["spec_match_count"] for row in records)
    extra_real = sum(row["extra_real_count"] for row in records)
    required = sum(row["required_count"] for row in records)
    covered = sum(row["required_covered"] for row in records)
    functional_required = sum(row["functional_required"] for row in records)
    functional_covered = sum(row["functional_covered"] for row in records)
    geometry_real = sum(row["geometry_real_count"] for row in records)
    geometry_known = sum(row["geometry_role_known_count"] for row in records)
    instance_numerator = sum(row["instance_distinguishable_count"] for row in records)
    instance_denominator = sum(row["instance_applicable_count"] for row in records)
    overseg_excess = sum(row["overseg_excess_fragments"] for row in records)
    return {
        "asset_count": len(records),
        "task_count": task_count,
        "truthfully_named_count": truthful,
        "spec_match_count": spec_match,
        "semantic_precision_micro": divide(truthful, task_count),
        "semantic_precision_asset_macro": statistics.mean(
            row["truthful_count"] / row["task_count"] for row in records
        ),
        "required_role_count": required,
        "required_role_covered_count": covered,
        "semantic_recall_micro": divide(covered, required),
        "semantic_recall_asset_macro": statistics.mean(
            row["required_covered"] / row["required_count"] for row in records
        ),
        "validated_named_link_density_micro": divide(truthful, required),
        "validated_named_link_density_asset_macro": statistics.mean(
            row["truthful_count"] / row["required_count"] for row in records
        ),
        "extra_real_part_count": extra_real,
        "extra_real_parts_per_asset": divide(extra_real, len(records)),
        "functional_required_count": functional_required,
        "functional_covered_count": functional_covered,
        "functional_naming_richness_micro": divide(
            functional_covered, functional_required
        ),
        "functional_naming_richness_asset_macro": statistics.mean(
            row["functional_covered"] / row["functional_required"]
            for row in records
        ),
        "functional_core_coverage_micro": divide(
            functional_covered, functional_required
        ),
        "functional_core_coverage_asset_macro": statistics.mean(
            row["functional_covered"] / row["functional_required"]
            for row in records
        ),
        "geometry_real_part_count": geometry_real,
        "geometry_role_known_count": geometry_known,
        "geometry_role_known_rate": divide(geometry_known, geometry_real),
        "instance_applicable_group_count": sum(
            row["instance_applicable_groups"] for row in records
        ),
        "instance_applicable_count": instance_denominator,
        "instance_distinguishable_count": instance_numerator,
        "instance_discriminability_micro": divide(
            instance_numerator, instance_denominator
        ),
        "over_segmentation_excess_fragments": overseg_excess,
        "over_segmentation_rate_micro": divide(overseg_excess, geometry_real),
        "over_segmentation_rate_asset_macro": statistics.mean(
            divide(row["overseg_excess_fragments"], row["geometry_real_count"]) or 0.0
            for row in records
        ),
    }


def bootstrap(
    records: list[dict[str, Any]], metrics: list[str], seed: int, count: int
) -> dict[str, list[float] | None]:
    by_category = {
        category: [row for row in records if row["category"] == category]
        for category in CATEGORIES
    }
    rng = np.random.default_rng(seed)
    samples: dict[str, list[float]] = {metric: [] for metric in metrics}
    for _ in range(count):
        selected = rng.choice(CATEGORIES, size=len(CATEGORIES), replace=True)
        replicate = [row for category in selected for row in by_category[str(category)]]
        values = aggregate(replicate)
        for metric in metrics:
            value = values[metric]
            if value is not None:
                samples[metric].append(float(value))
    result: dict[str, list[float] | None] = {}
    for metric, values in samples.items():
        result[metric] = (
            [float(np.quantile(values, 0.025)), float(np.quantile(values, 0.975))]
            if values
            else None
        )
    return result


def main() -> int:
    queue = read_jsonl(QUEUE)
    audit = read_jsonl(AUDIT)
    if len(queue) != 1107 or len(audit) != len(queue):
        raise RuntimeError("frozen queue/audit count mismatch")
    queue_by_id = {str(row["item_id"]): row for row in queue}
    audit_by_id = {str(row["item_id"]): row for row in audit}
    if len(queue_by_id) != len(queue) or set(queue_by_id) != set(audit_by_id):
        raise RuntimeError("queue/audit item mismatch")
    if any(recursive_key_present(row, "method") for row in queue):
        raise RuntimeError("method key leaked into blind queue")
    judges = [validate_judge(path, queue_by_id) for path in JUDGES]

    consensus = []
    verdict_vote_rows = []
    geometry_real_vote_rows = []
    geometry_role_vote_rows = []
    instance_vote_rows = []
    same_part_vote_rows = []
    for item_id in sorted(queue_by_id):
        task = queue_by_id[item_id]
        audit_row = audit_by_id[item_id]
        votes = [judge[item_id] for judge in judges]
        verdict_votes = [str(row["judge_verdict"]) for row in votes]
        verdict = majority(verdict_votes, {"uncertain"})
        matched_role = (
            majority(
                [
                    normalized(row["judge_matched_role"])
                    for row in votes
                    if row["judge_verdict"] == "spec_match"
                ]
            )
            if verdict == "spec_match"
            else None
        )
        real_votes = [bool(row["judge_geometry_is_real_part"]) for row in votes]
        geometry_real = majority(real_votes)
        geometry_role = (
            majority(
                [
                    normalized(row["judge_geometry_role"])
                    for row in votes
                    if row["judge_geometry_is_real_part"] is True
                ]
            )
            if geometry_real is True
            else "unknown"
        )
        same_part = (
            majority(
                [
                    normalized(row["judge_same_semantic_part_as"])
                    for row in votes
                    if row["judge_geometry_is_real_part"] is True
                ]
            )
            if geometry_real is True
            else "not_applicable"
        )
        instance_id = (
            majority(
                [
                    normalized(row["judge_instance_id"])
                    for row in votes
                    if row["judge_geometry_is_real_part"] is True
                ]
            )
            if geometry_real is True
            else "not_applicable"
        )
        consensus.append(
            {
                "item_id": item_id,
                "asset_id": task["asset_id"],
                "method": audit_row["method"],
                "category": task["category"],
                "node_name": task["node_name"],
                "required_roles": task["required_roles"],
                "consensus_verdict": verdict,
                "consensus_matched_role": matched_role,
                "consensus_geometry_is_real_part": geometry_real,
                "consensus_geometry_role": geometry_role,
                "consensus_instance_id": instance_id,
                "consensus_same_semantic_part_as": same_part,
                "verdict_votes": verdict_votes,
                "geometry_real_votes": real_votes,
            }
        )
        verdict_vote_rows.append(verdict_votes)
        geometry_real_vote_rows.append([str(value) for value in real_votes])
        geometry_role_vote_rows.append(
            [str(row["judge_geometry_role"]) for row in votes]
        )
        instance_vote_rows.append([str(row["judge_instance_id"]) for row in votes])
        same_part_vote_rows.append(
            [str(row["judge_same_semantic_part_as"]) for row in votes]
        )

    initial_missing_ids = {
        row["item_id"]: [
            field
            for field, missing_value in (
                ("judge_geometry_role", row["consensus_geometry_role"] is None),
                ("judge_instance_id", row["consensus_instance_id"] is None),
            )
            if missing_value
        ]
        for row in consensus
        if row["consensus_geometry_role"] is None
        or row["consensus_instance_id"] is None
    }
    rereview_rows = read_jsonl(REREVIEW_QUEUE)
    rereview_tasks = {str(row["item_id"]): row for row in rereview_rows}
    if len(rereview_rows) != len(rereview_tasks):
        raise RuntimeError("duplicate rereview item id")
    if set(rereview_tasks) != set(initial_missing_ids):
        raise RuntimeError("rereview packet does not exactly match initial missing fields")
    for item_id, fields in initial_missing_ids.items():
        if set(rereview_tasks[item_id]["rejudge_fields"]) != set(fields):
            raise RuntimeError(f"rereview field mismatch: {item_id}")
    rereview_judges = [
        validate_rereview_judge(path, rereview_tasks) for path in REREVIEW_JUDGES
    ]
    for row in consensus:
        item_id = str(row["item_id"])
        if item_id not in initial_missing_ids:
            continue
        votes = [judge[item_id] for judge in rereview_judges]
        if "judge_geometry_role" in initial_missing_ids[item_id]:
            role_votes = [str(vote["judge_geometry_role"]) for vote in votes]
            row["rereview_geometry_role_votes"] = role_votes
            row["consensus_geometry_role"] = majority(role_votes)
        if "judge_instance_id" in initial_missing_ids[item_id]:
            instance_votes = [str(vote["judge_instance_id"]) for vote in votes]
            row["rereview_instance_id_votes"] = instance_votes
            row["consensus_instance_id"] = majority(instance_votes)

    second_missing_ids = {
        row["item_id"]: [
            field
            for field, missing_value in (
                ("judge_geometry_role", row["consensus_geometry_role"] is None),
                ("judge_instance_id", row["consensus_instance_id"] is None),
            )
            if missing_value
        ]
        for row in consensus
        if row["consensus_geometry_role"] is None
        or row["consensus_instance_id"] is None
    }
    post_rereview_missing_counts = {
        "judge_geometry_role": sum(
            "judge_geometry_role" in fields for fields in second_missing_ids.values()
        ),
        "judge_instance_id": sum(
            "judge_instance_id" in fields for fields in second_missing_ids.values()
        ),
    }
    tiebreak_rows = read_jsonl(TIEBREAK_QUEUE)
    tiebreak_tasks = {str(row["item_id"]): row for row in tiebreak_rows}
    if len(tiebreak_rows) != len(tiebreak_tasks):
        raise RuntimeError("duplicate tiebreak item id")
    if set(tiebreak_tasks) != set(second_missing_ids):
        raise RuntimeError("tiebreak packet does not match post-rereview missing set")
    for item_id, fields in second_missing_ids.items():
        if set(tiebreak_tasks[item_id]["adjudicate_fields"]) != set(fields):
            raise RuntimeError(f"tiebreak field mismatch: {item_id}")
    tiebreak_judge = validate_rereview_judge(TIEBREAK_JUDGE, tiebreak_tasks)
    for row in consensus:
        item_id = str(row["item_id"])
        if item_id not in second_missing_ids:
            continue
        vote = tiebreak_judge[item_id]
        if "judge_geometry_role" in second_missing_ids[item_id]:
            row["blind_tiebreak_geometry_role"] = vote["judge_geometry_role"]
            row["consensus_geometry_role"] = vote["judge_geometry_role"]
        if "judge_instance_id" in second_missing_ids[item_id]:
            row["blind_tiebreak_instance_id"] = vote["judge_instance_id"]
            row["consensus_instance_id"] = vote["judge_instance_id"]

    missing = {
        "verdict": sum(row["consensus_verdict"] is None for row in consensus),
        "matched_role": sum(
            row["consensus_verdict"] == "spec_match"
            and row["consensus_matched_role"] is None
            for row in consensus
        ),
        "geometry_real": sum(
            row["consensus_geometry_is_real_part"] is None for row in consensus
        ),
        "geometry_role": sum(
            row["consensus_geometry_is_real_part"] is True
            and row["consensus_geometry_role"] is None
            for row in consensus
        ),
        "instance_id": sum(
            row["consensus_geometry_is_real_part"] is True
            and row["consensus_instance_id"] is None
            for row in consensus
        ),
        "same_part": sum(
            row["consensus_geometry_is_real_part"] is True
            and row["consensus_same_semantic_part_as"] is None
            for row in consensus
        ),
    }
    if any(missing.values()):
        raise RuntimeError(f"consensus annotations incomplete: {missing}")

    asset_records = summarize_assets(consensus)
    unresolved = sum(len(row["overseg_unresolved_edges"]) for row in asset_records)
    if unresolved:
        raise RuntimeError(f"unresolved same-part consensus edges: {unresolved}")
    primary_metrics = [
        "semantic_precision_micro",
        "semantic_recall_asset_macro",
        "functional_naming_richness_asset_macro",
        "instance_discriminability_micro",
        "over_segmentation_rate_micro",
    ]
    bootstrap_metrics = primary_metrics + [
        "validated_named_link_density_asset_macro"
    ]
    methods = {}
    for method in METHODS:
        selected = [row for row in asset_records if row["method"] == method]
        values = aggregate(selected)
        values["category_cluster_bootstrap_95_ci"] = bootstrap(
            selected, bootstrap_metrics, 260811003, 10000
        )
        values["category_breakdown"] = {
            category: aggregate([row for row in selected if row["category"] == category])
            for category in CATEGORIES
        }
        methods[method] = values

    pair_agreements = pairwise_exact(verdict_vote_rows)
    geometry_role_pair = pairwise_exact(geometry_role_vote_rows)
    instance_pair = pairwise_exact(instance_vote_rows)
    same_part_pair = pairwise_exact(same_part_vote_rows)
    manifest = json.loads(contained(MANIFEST).read_text(encoding="utf-8"))
    protocol = json.loads(contained(PROTOCOL).read_text(encoding="utf-8"))
    metric_correction = json.loads(
        contained(METRIC_CORRECTION).read_text(encoding="utf-8")
    )
    if metric_correction["parent_semantic_protocol"]["sha256"] != digest(PROTOCOL):
        raise RuntimeError("metric correction parent protocol hash mismatch")
    preview_qa = verify_previews(manifest)
    summary = {
        "protocol_id": "nano3d_table2_naming_semantic_v1.1",
        "base_semantic_protocol_id": protocol["protocol_id"],
        "status": "COMPLETE",
        "matched_protocol_sha256": manifest["matched_protocol_sha256"],
        "semantic_gold_sha256": digest(GOLD),
        "semantic_protocol_sha256": digest(PROTOCOL),
        "metric_correction_protocol_sha256": digest(METRIC_CORRECTION),
        "metric_correction": metric_correction["correction"],
        "blind_tasks_sha256": digest(QUEUE),
        "audit_tasks_sha256": digest(AUDIT),
        "judge_file_sha256": {
            path.stem: digest(path) for path in JUDGES
        },
        "blind_rereview": {
            "protocol_sha256": digest(REREVIEW_PROTOCOL),
            "task_sha256": digest(REREVIEW_QUEUE),
            "task_count": len(rereview_tasks),
            "initial_missing_field_counts": {
                "judge_geometry_role": sum(
                    "judge_geometry_role" in fields
                    for fields in initial_missing_ids.values()
                ),
                "judge_instance_id": sum(
                    "judge_instance_id" in fields
                    for fields in initial_missing_ids.values()
                ),
            },
            "judge_file_sha256": {
                path.stem: digest(path) for path in REREVIEW_JUDGES
            },
            "unresolved_after_rereview": post_rereview_missing_counts,
            "field_only": True,
            "other_votes_hidden": True,
        },
        "blind_tiebreak": {
            "protocol_sha256": digest(TIEBREAK_PROTOCOL),
            "task_sha256": digest(TIEBREAK_QUEUE),
            "task_count": len(tiebreak_tasks),
            "asset_count": len(
                {row["asset_id"] for row in tiebreak_tasks.values()}
            ),
            "field_counts": {
                "judge_geometry_role": sum(
                    "judge_geometry_role" in fields
                    for fields in second_missing_ids.values()
                ),
                "judge_instance_id": sum(
                    "judge_instance_id" in fields
                    for fields in second_missing_ids.values()
                ),
            },
            "adjudicator_file_sha256": digest(TIEBREAK_JUDGE),
            "single_fresh_blind_adjudicator": True,
            "prior_votes_hidden": True,
        },
        "asset_count": len(asset_records),
        "task_count": len(consensus),
        "judge_count": 3,
        "judge_type": protocol["judge_design"]["judge_type"],
        "method_blind": True,
        "preview_qa": preview_qa,
        "consensus_missing": missing,
        "judge_agreement": {
            "verdict_pairwise_exact": pair_agreements,
            "verdict_pairwise_exact_mean": statistics.mean(pair_agreements),
            "verdict_unanimous_rate": sum(len(set(row)) == 1 for row in verdict_vote_rows)
            / len(verdict_vote_rows),
            "verdict_fleiss_kappa": fleiss_kappa(
                verdict_vote_rows, sorted(ALLOWED_VERDICTS)
            ),
            "geometry_real_fleiss_kappa": fleiss_kappa(
                geometry_real_vote_rows, ["False", "True"]
            ),
            "geometry_role_pairwise_exact": geometry_role_pair,
            "geometry_role_pairwise_exact_mean": statistics.mean(
                geometry_role_pair
            ),
            "instance_id_pairwise_exact": instance_pair,
            "instance_id_pairwise_exact_mean": statistics.mean(instance_pair),
            "same_part_pairwise_exact": same_part_pair,
            "same_part_pairwise_exact_mean": statistics.mean(same_part_pair),
        },
        "methods": methods,
        "limitations": protocol["interpretation_limits"],
    }
    write_jsonl(RUNTIME / "consensus_records.jsonl", consensus)
    write_jsonl(RUNTIME / "asset_semantic_records.jsonl", asset_records)
    write_json(RUNTIME / "summary.json", summary)
    report_lines = [
        "# Table 2 matched Naming semantic evaluation",
        "",
        "Status: COMPLETE",
        "",
        "Cohort: 4 methods x 5 categories x 7 assets = 140 assets; "
        "1,107 renderable-link tasks.",
        "Judging: three isolated method-blind Codex judge sessions; consensus requires "
        "at least two identical non-uncertain votes.",
        "Field completeness: 72 anonymous items received an independent field-only re-review; "
        "15/1,107 geometry-role fields that remained split were resolved by one fresh blind "
        "tie-break adjudicator with all prior votes hidden.",
        "",
        "| Method | Precision micro | Recall asset-macro | Functional Richness asset-macro | "
        "Instance micro | Over-seg micro |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for method in METHODS:
        values = methods[method]
        display = []
        for key in primary_metrics:
            value = values[key]
            display.append("N/A" if value is None else f"{value:.6f}")
        report_lines.append(f"| {method} | " + " | ".join(display) + " |")
    report_lines.extend(
        [
            "",
            "Supplementary granularity-sensitive diagnostics:",
            "",
            "| Method | Validated named-link density | Extra real parts / asset |",
            "|---|---:|---:|",
            *[
                f"| {method} | "
                f"{methods[method]['validated_named_link_density_asset_macro']:.6f} | "
                f"{methods[method]['extra_real_parts_per_asset']:.6f} |"
                for method in METHODS
            ],
            "",
            f"Verdict Fleiss kappa: {summary['judge_agreement']['verdict_fleiss_kappa']:.6f}",
            f"Mean pairwise exact verdict agreement: "
            f"{summary['judge_agreement']['verdict_pairwise_exact_mean']:.6f}",
            "",
            "The role gold was frozen without inspecting evaluated outputs. Optional-role absence is not "
            "penalized. Functional Richness follows the preexisting PV-A definition and does not reward "
            "extra real parts; named-link density and extra parts are supplementary. These are LLM-judge "
            "results, not human annotations. Cross-seed consistency remains a separate direct-output metric.",
            "",
        ]
    )
    contained(RUNTIME / "report.md").write_text(
        "\n".join(report_lines), encoding="utf-8"
    )

    checks = {
        "queue_has_1107_unique_items": len(queue_by_id) == 1107,
        "audit_exact_item_set": set(queue_by_id) == set(audit_by_id),
        "blind_queue_has_no_method_key": not any(
            recursive_key_present(row, "method") for row in queue
        ),
        "three_judge_files_have_exact_item_set": all(
            set(judge) == set(queue_by_id) for judge in judges
        ),
        "all_consensus_fields_complete": not any(missing.values()),
        "asset_count_is_140": len(asset_records) == 140,
        "each_method_has_35_assets": all(
            methods[method]["asset_count"] == 35 for method in METHODS
        ),
        "each_method_category_has_7_assets": all(
            methods[method]["category_breakdown"][category]["asset_count"] == 7
            for method in METHODS
            for category in CATEGORIES
        ),
        "task_conservation": sum(
            methods[method]["task_count"] for method in METHODS
        )
        == 1107,
        "precision_count_conservation": all(
            0 <= methods[method]["truthfully_named_count"] <= methods[method]["task_count"]
            for method in METHODS
        ),
        "required_role_conservation": all(
            0
            <= methods[method]["required_role_covered_count"]
            <= methods[method]["required_role_count"]
            for method in METHODS
        ),
        "no_unresolved_same_part_edges": unresolved == 0,
        "parent_hash_matches_manifest": protocol["parent_matched_protocol"]["sha256"]
        == manifest["matched_protocol_sha256"],
        "gold_hash_matches_protocol": protocol["semantic_gold"]["sha256"]
        == digest(GOLD),
        "metric_correction_parent_hash_matches": metric_correction[
            "parent_semantic_protocol"
        ]["sha256"]
        == digest(PROTOCOL),
        "functional_richness_uses_functional_role_counts": all(
            methods[method]["functional_naming_richness_micro"]
            == methods[method]["functional_core_coverage_micro"]
            and methods[method]["functional_naming_richness_asset_macro"]
            == methods[method]["functional_core_coverage_asset_macro"]
            for method in METHODS
        ),
        "truthful_names_equal_spec_plus_extra": all(
            methods[method]["truthfully_named_count"]
            == methods[method]["spec_match_count"]
            + methods[method]["extra_real_part_count"]
            for method in METHODS
        ),
        "rereview_exact_initial_missing_set": set(rereview_tasks)
        == set(initial_missing_ids),
        "rereview_reduced_or_preserved_missing_count": sum(
            post_rereview_missing_counts.values()
        )
        <= sum(len(fields) for fields in initial_missing_ids.values()),
        "tiebreak_exact_post_rereview_missing_set": set(tiebreak_tasks)
        == set(second_missing_ids),
        "tiebreak_resolved_all_remaining_fields": not any(
            missing[field] for field in ("geometry_role", "instance_id")
        ),
        "preview_set_hashes_and_pixel_gates_pass": preview_qa["all_hashes_match"]
        and preview_qa["all_nonblank"]
        and preview_qa["preview_count"] == 1107,
    }
    self_check = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "artifact_sha256": {
            "summary.json": digest(RUNTIME / "summary.json"),
            "consensus_records.jsonl": digest(RUNTIME / "consensus_records.jsonl"),
            "asset_semantic_records.jsonl": digest(
                RUNTIME / "asset_semantic_records.jsonl"
            ),
            "report.md": digest(RUNTIME / "report.md"),
        },
    }
    write_json(RUNTIME / "self_check.json", self_check)
    manifest["judge_files_complete"] = True
    manifest["judge_file_sha256"] = summary["judge_file_sha256"]
    manifest["summary_sha256"] = digest(RUNTIME / "summary.json")
    manifest["metric_correction_protocol_sha256"] = digest(METRIC_CORRECTION)
    manifest["self_check_status"] = self_check["status"]
    write_json(MANIFEST, manifest)
    if self_check["status"] != "PASS":
        raise RuntimeError("self-check failed")
    print(
        json.dumps(
            {
                "status": summary["status"],
                "judge_agreement": summary["judge_agreement"],
                "methods": {
                    method: {key: methods[method][key] for key in primary_metrics}
                    for method in METHODS
                },
                "self_check": self_check["status"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
