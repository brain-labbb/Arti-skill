#!/usr/bin/env python3
"""Paper-aligned Nano3D Naming evaluation over exp-local URDF snapshots.

This harness follows the Nova3D Naming capability staircase while preserving
the boundary between direct measurements, source-derived role proxies, and
judge-only semantic metrics. It never treats a named link that is absent from
the core-role list as an automatic false positive: such links are exported to
``judge_queue.jsonl`` as possible extra real parts.

All writable paths are under /mnt/zsn/lyb/arti-skill/exp.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import re
import statistics
import xml.etree.ElementTree as ET
from collections import Counter
from itertools import combinations
from pathlib import Path
from typing import Any, Iterable


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
ASSET_DOC = EXP_ROOT / "Nano3dasset.md"
INPUT_URDF_DIR = EXP_ROOT / "runtime/nano3d_naming/input_urdf"
CROSS_SEED_URDF_DIR = EXP_ROOT / "runtime/nano3d_naming/cross_seed_input_urdf"
GOLD_PATH = EXP_ROOT / "reference/naming_gold_v2.json"
PROTOCOL_PATH = EXP_ROOT / "reference/naming_protocol_v2.json"
CROSS_SEED_PATH = EXP_ROOT / "runtime/nano3d_low_medium/cross_seed_records.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_naming"

PLACEHOLDER_RE = re.compile(r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$", re.I)
TOKEN_RE = re.compile(r"[a-z0-9]+")
INSTANCE_MARKERS = {
    "left", "right", "front", "rear", "upper", "lower", "top", "bottom",
    "nose", "main", "inboard", "outboard", "inner", "outer", "near", "far",
    "needle", "lead", "driver", "driven",
}


def tokens(value: str) -> tuple[str, ...]:
    return tuple(TOKEN_RE.findall(str(value).lower()))


def contains_sequence(haystack: tuple[str, ...], needle: tuple[str, ...]) -> bool:
    if not needle or len(needle) > len(haystack):
        return False
    return any(haystack[i : i + len(needle)] == needle for i in range(len(haystack) - len(needle) + 1))


def pattern_match(pattern: str, name: str) -> bool:
    wanted = set(tokens(pattern))
    observed = set(tokens(name))
    return bool(wanted) and wanted.issubset(observed)


def pattern_score(pattern: str, link_name: str, role_name: str) -> int:
    """Rank only the pattern actually matched, avoiding generic-pattern theft."""
    pts = tokens(pattern)
    nts = tokens(link_name)
    rts = set(tokens(role_name))
    if not pts or not set(pts).issubset(set(nts)):
        return -1
    score = 100 * len(set(pts))
    score += 20 if contains_sequence(nts, pts) else 0
    score += 10 if tuple(nts[-len(pts) :]) == pts else 0
    score += 5 if tuple(nts[: len(pts)]) == pts else 0
    score += len(rts & set(nts))
    return score


def role_candidate(link_name: str, role: dict[str, Any]) -> dict[str, Any] | None:
    matched = []
    for pattern in role.get("patterns", []):
        score = pattern_score(str(pattern), link_name, str(role["name"]))
        if score >= 0:
            matched.append((score, str(pattern)))
    if not matched:
        return None
    score, pattern = max(matched, key=lambda item: (item[0], len(item[1]), item[1]))
    return {"role": str(role["name"]), "score": score, "pattern": pattern}


def all_role_candidates(link_name: str, roles: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = [candidate for role in roles if (candidate := role_candidate(link_name, role)) is not None]
    return sorted(rows, key=lambda item: (-item["score"], item["role"]))


def best_role(link_name: str, roles: list[dict[str, Any]]) -> str | None:
    candidates = all_role_candidates(link_name, roles)
    if not candidates:
        return None
    top = candidates[0]["score"]
    winners = [row["role"] for row in candidates if row["score"] == top]
    return winners[0] if len(winners) == 1 else None


def read_selected_assets() -> list[dict[str, Any]]:
    text = ASSET_DOC.read_text(encoding="utf-8")
    paths = re.findall(r"\]\((/mnt/zsn/lyb/arti-skill/(?:seed_exports|seed_exports_physics_10)/[^)]+)\)", text)
    rows = []
    for raw in paths:
        path = Path(raw)
        rows.append(
            {
                "asset_id": f"{path.parent.name}__{path.name}",
                "slug": path.parent.name,
                "seed": int(path.name.removeprefix("seed_")),
                "source": "seed_exports_physics_10" if "seed_exports_physics_10" in path.parts else "seed_exports",
                "snapshot": str(INPUT_URDF_DIR / f"{path.parent.name}__{path.name}.urdf"),
            }
        )
    if len(rows) != 33:
        raise RuntimeError(f"expected 33 selected assets, found {len(rows)}")
    return rows


def load_inputs() -> tuple[dict[str, Any], dict[str, Any]]:
    gold = json.loads(GOLD_PATH.read_text(encoding="utf-8"))
    protocol = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
    missing = sorted(set(row["slug"] for row in read_selected_assets()) - set(gold.get("assets", {})))
    if missing:
        raise RuntimeError(f"missing naming gold entries: {missing}")
    return gold, protocol


def parse_links(path: Path) -> list[dict[str, Any]]:
    root = ET.parse(path).getroot()
    rows = []
    for node in root.findall("link"):
        name = node.attrib.get("name", "")
        if not name:
            continue
        geometry = []
        for visual in node.findall("visual"):
            geometry_node = visual.find("geometry")
            if geometry_node is None or not list(geometry_node):
                continue
            shape = list(geometry_node)[0]
            item = {"type": shape.tag}
            if shape.tag == "mesh":
                item["filename"] = shape.attrib.get("filename")
            geometry.append(item)
        rows.append(
            {
                "name": name,
                "visual_tag_count": len(node.findall("visual")),
                "has_visual": bool(geometry),
                "parsed_visual_geometry_count": len(geometry),
                "has_collision": bool(node.findall("collision")),
                "geometry": geometry,
            }
        )
    return rows


def expand_role_slots(roles: list[dict[str, Any]]) -> list[dict[str, Any]]:
    slots = []
    for role in roles:
        for index in range(int(role.get("min_count", 1))):
            slots.append(
                {
                    "role": str(role["name"]),
                    "instance_index": index,
                    "functional": bool(role.get("functional")),
                }
            )
    return slots


def assign_required_roles(link_names: list[str], roles: list[dict[str, Any]]) -> dict[str, Any]:
    """Maximum-cardinality then maximum-specificity bipartite assignment."""
    # URDF XML order and glTF node order are representation details. Freeze a
    # lexical order so equal-score assignments are artifact-order invariant.
    link_names = sorted(link_names)
    slots = expand_role_slots(roles)
    candidates = [all_role_candidates(name, roles) for name in link_names]
    role_scores = [{row["role"]: row for row in rows} for rows in candidates]

    # mask -> (specificity score, [(link index, slot index, matched pattern, edge score)])
    states: dict[int, tuple[int, tuple[tuple[int, int, str, int], ...]]] = {0: (0, ())}
    for link_index, score_map in enumerate(role_scores):
        updated = dict(states)
        for mask, (total_score, assignments) in states.items():
            for slot_index, slot in enumerate(slots):
                if mask & (1 << slot_index):
                    continue
                candidate = score_map.get(slot["role"])
                if candidate is None:
                    continue
                new_mask = mask | (1 << slot_index)
                new_score = total_score + int(candidate["score"])
                previous = updated.get(new_mask)
                row = (link_index, slot_index, str(candidate["pattern"]), int(candidate["score"]))
                if previous is None or new_score > previous[0]:
                    updated[new_mask] = (new_score, assignments + (row,))
        states = updated

    best_mask, (specificity, assigned) = max(
        states.items(), key=lambda item: (item[0].bit_count(), item[1][0])
    )
    assignment_rows = []
    for link_index, slot_index, pattern, score in assigned:
        link_name = link_names[link_index]
        role_name = slots[slot_index]["role"]
        pattern_token_count = len(set(tokens(pattern)))
        canonical_role_covered = set(tokens(role_name)).issubset(set(tokens(link_name)))
        other_scores = [
            int(item["score"])
            for item in candidates[link_index]
            if item["role"] != role_name
        ]
        runner_up_score = max(other_scores, default=None)
        assignment_rows.append(
            {
                "link": link_name,
                "role": role_name,
                "role_instance_index": slots[slot_index]["instance_index"],
                "functional": slots[slot_index]["functional"],
                "matched_pattern": pattern,
                "matched_pattern_token_count": pattern_token_count,
                "canonical_role_covered": canonical_role_covered,
                "evidence_strength": "strong" if canonical_role_covered or pattern_token_count >= 2 else "single_token_alias",
                "specificity_score": score,
                "runner_up_role_score": runner_up_score,
                "specificity_margin": score - runner_up_score if runner_up_score is not None else None,
            }
        )
    return {
        "required_slots": slots,
        "assignments": sorted(assignment_rows, key=lambda row: (row["role"], row["role_instance_index"])),
        "matched_required_instances": best_mask.bit_count(),
        "specificity_score": specificity,
        "link_candidates": {name: rows for name, rows in zip(link_names, candidates)},
    }


def instance_key(name: str) -> str:
    ts = tokens(name)
    markers = [token for token in ts if token.isdigit() or token in INSTANCE_MARKERS]
    return "/".join(markers)


def instance_rows(link_names: list[str], roles: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for role in roles:
        required = int(role.get("min_count", 1))
        if required <= 1:
            continue
        names = []
        for name in link_names:
            candidates = all_role_candidates(name, roles)
            if not candidates:
                continue
            top = candidates[0]["score"]
            if any(row["role"] == role["name"] and row["score"] == top for row in candidates):
                names.append(name)
        keys = sorted({key for name in names if (key := instance_key(name))})
        distinguishable = min(required, len(keys))
        rows.append(
            {
                "role": role["name"],
                "required_count": required,
                "candidate_names": names,
                "instance_keys": keys,
                "distinguishable_count": distinguishable,
                "score": distinguishable / required,
            }
        )
    return rows


def semantic_reference_eligible(spec: dict[str, Any]) -> bool:
    return "fallback" not in str(spec.get("evidence", "")).lower()


def evaluate_asset(row: dict[str, Any], gold: dict[str, Any]) -> dict[str, Any]:
    links = parse_links(Path(row["snapshot"]))
    names = [link["name"] for link in links]
    mesh_links = [link for link in links if link["has_visual"]]
    mesh_names = [link["name"] for link in mesh_links]
    spec = gold["assets"][row["slug"]]
    roles = spec["roles"]
    assignment = assign_required_roles(mesh_names, roles)
    assigned_links = {item["link"] for item in assignment["assignments"]}
    required_count = len(assignment["required_slots"])
    functional_required = sum(bool(slot["functional"]) for slot in assignment["required_slots"])
    functional_matched = sum(bool(item["functional"]) for item in assignment["assignments"])
    named_links = [link for link in links if not PLACEHOLDER_RE.fullmatch(link["name"])]
    named_mesh_links = [link for link in mesh_links if not PLACEHOLDER_RE.fullmatch(link["name"])]
    repeated = instance_rows(mesh_names, roles)
    strong_assignments = [item for item in assignment["assignments"] if item["evidence_strength"] == "strong"]

    return {
        **row,
        "evidence": spec.get("evidence"),
        "semantic_reference_eligible": semantic_reference_eligible(spec),
        "links": links,
        "link_count": len(links),
        "mesh_bearing_link_count": len(mesh_links),
        "named_link_count": len(named_links),
        "named_mesh_link_count": len(named_mesh_links),
        "placeholder_link_count": len(links) - len(named_links),
        "nameability": len(named_mesh_links) / len(mesh_links) if mesh_links else None,
        "required_spec_instance_count": required_count,
        "matched_required_instance_count": assignment["matched_required_instances"],
        "source_role_recall": assignment["matched_required_instances"] / required_count if required_count else None,
        "strong_match_count": len(strong_assignments),
        "strong_match_sensitivity": len(strong_assignments) / required_count if required_count else None,
        "functional_required_instance_count": functional_required,
        "functional_matched_instance_count": functional_matched,
        "functional_core_coverage": functional_matched / functional_required if functional_required else None,
        "paper_aligned_richness_candidate": len(named_mesh_links) / required_count if required_count else None,
        "reference_roles": [
            {
                "role": str(role["name"]),
                "min_count": int(role.get("min_count", 1)),
                "functional": bool(role.get("functional")),
            }
            for role in roles
        ],
        "required_role_assignment": assignment["assignments"],
        "link_role_candidates": assignment["link_candidates"],
        "extra_real_part_candidates": [
            link["name"] for link in named_mesh_links if link["name"] not in assigned_links
        ],
        "instance_rows": repeated,
        "instance_discriminability": (
            sum(item["distinguishable_count"] for item in repeated)
            / sum(item["required_count"] for item in repeated)
            if repeated else None
        ),
        "semantic_precision": None,
        "semantic_judge_recall": None,
        "over_segmentation_rate": None,
        "over_segmentation_status": "unsupported_without_part_to_role_decomposition_gold",
    }


def raw_jaccard(a: set[str], b: set[str]) -> float:
    union = a | b
    return len(a & b) / len(union) if union else 1.0


def weighted_jaccard(a: dict[str, int], b: dict[str, int]) -> float:
    keys = set(a) | set(b)
    denominator = sum(max(a.get(key, 0), b.get(key, 0)) for key in keys)
    return sum(min(a.get(key, 0), b.get(key, 0)) for key in keys) / denominator if denominator else 1.0


def capped_role_signature(names: list[str], roles: list[dict[str, Any]]) -> dict[str, int]:
    assignment = assign_required_roles(names, roles)
    counts = Counter(item["role"] for item in assignment["assignments"])
    return {str(role["name"]): min(int(role.get("min_count", 1)), counts.get(str(role["name"]), 0)) for role in roles}


def cross_seed_eval(gold: dict[str, Any]) -> dict[str, Any]:
    payload = json.loads(CROSS_SEED_PATH.read_text(encoding="utf-8"))
    cohorts = []
    raw_scores: list[float] = []
    set_scores: list[float] = []
    multiset_scores: list[float] = []
    exact_rates: list[float] = []
    missing_snapshots = []
    for cohort in payload:
        seeds = cohort.get("seed_records", [])
        if len(seeds) < 2:
            continue
        slug = cohort["slug"]
        roles = gold["assets"][slug]["roles"]
        parsed = []
        for seed in seeds:
            path = CROSS_SEED_URDF_DIR / f"{seed['asset_id']}.urdf"
            if not path.exists():
                missing_snapshots.append(str(path))
                continue
            names = [row["name"] for row in parse_links(path) if row["has_visual"]]
            parsed.append((seed["asset_id"], names))
        if len(parsed) < 2:
            continue
        raw_pairwise = [raw_jaccard(set(a), set(b)) for (_, a), (_, b) in combinations(parsed, 2)]
        signatures = [capped_role_signature(names, roles) for _, names in parsed]
        role_sets = [{role for role, count in signature.items() if count > 0} for signature in signatures]
        set_pairwise = [raw_jaccard(a, b) for a, b in combinations(role_sets, 2)]
        multiset_pairwise = [weighted_jaccard(a, b) for a, b in combinations(signatures, 2)]
        signature_counts = Counter(tuple(sorted(signature.items())) for signature in signatures)
        exact_rate = max(signature_counts.values()) / len(signatures)
        raw_scores.extend(raw_pairwise)
        set_scores.extend(set_pairwise)
        multiset_scores.extend(multiset_pairwise)
        exact_rates.append(exact_rate)
        cohorts.append(
            {
                "slug": slug,
                "selected_asset_id": cohort.get("selected_asset_id"),
                "seed_count": len(parsed),
                "raw_pairwise_name_jaccard_mean": statistics.mean(raw_pairwise),
                "source_role_set_jaccard_mean": statistics.mean(set_pairwise),
                "source_role_count_weighted_jaccard_mean": statistics.mean(multiset_pairwise),
                "source_role_count_signature_mode_rate": exact_rate,
                "source_role_count_signatures": {
                    "|".join(f"{key}:{value}" for key, value in signature): count
                    for signature, count in signature_counts.items()
                },
            }
        )
    if missing_snapshots:
        raise RuntimeError(
            f"missing {len(missing_snapshots)} frozen cross-seed URDFs; populate {CROSS_SEED_URDF_DIR} first"
        )
    return {
        "multi_seed_cohort_count": len(cohorts),
        "seed_records": sum(row["seed_count"] for row in cohorts),
        "raw_pairwise_name_jaccard_mean": statistics.mean(raw_scores) if raw_scores else None,
        "source_role_set_jaccard_mean": statistics.mean(set_scores) if set_scores else None,
        "source_role_count_weighted_jaccard_mean": statistics.mean(multiset_scores) if multiset_scores else None,
        "source_role_count_signature_mode_rate_mean": statistics.mean(exact_rates) if exact_rates else None,
        "cohorts": cohorts,
        "single_seed_cohorts_excluded": 10,
        "input_scope": "exp-local frozen URDF snapshots",
    }


def enrich_cross_seed_summary(cross: dict[str, Any], protocol: dict[str, Any]) -> None:
    cohorts = cross["cohorts"]
    bootstrap = protocol["bootstrap"]
    metrics = {
        "raw_pairwise_name_jaccard": "raw_pairwise_name_jaccard_mean",
        "source_role_set_jaccard": "source_role_set_jaccard_mean",
        "source_role_count_weighted_jaccard": "source_role_count_weighted_jaccard_mean",
    }
    for offset, (prefix, key) in enumerate(metrics.items(), 10):
        values = numeric_values(cohorts, key)
        cross[f"{prefix}_cohort_macro"] = statistics.mean(values) if values else None
        cross[f"{prefix}_cohort_median"] = statistics.median(values) if values else None
        cross[f"{prefix}_cohort_macro_95ci"] = bootstrap_mean_ci(
            values,
            samples=int(bootstrap["resamples"]),
            seed=int(bootstrap["seed"]) + offset,
        )


def numeric_values(rows: Iterable[dict[str, Any]], key: str) -> list[float]:
    return [float(row[key]) for row in rows if isinstance(row.get(key), (int, float)) and not isinstance(row.get(key), bool)]


def mean(rows: Iterable[dict[str, Any]], key: str) -> float | None:
    values = numeric_values(rows, key)
    return statistics.mean(values) if values else None


def micro_ratio(rows: Iterable[dict[str, Any]], numerator: str, denominator: str) -> float | None:
    nums = sum(int(row.get(numerator, 0)) for row in rows)
    dens = sum(int(row.get(denominator, 0)) for row in rows)
    return nums / dens if dens else None


def percentile(sorted_values: list[float], quantile: float) -> float:
    if not sorted_values:
        raise ValueError("cannot take percentile of empty values")
    position = (len(sorted_values) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    fraction = position - lower
    return sorted_values[lower] * (1 - fraction) + sorted_values[upper] * fraction


def bootstrap_mean_ci(values: list[float], *, samples: int, seed: int) -> list[float] | None:
    if not values:
        return None
    rng = random.Random(seed)
    boot = sorted(statistics.mean(rng.choice(values) for _ in values) for _ in range(samples))
    return [percentile(boot, 0.025), percentile(boot, 0.975)]


def summarize(records: list[dict[str, Any]], cross: dict[str, Any], protocol: dict[str, Any]) -> dict[str, Any]:
    eligible = [row for row in records if row["semantic_reference_eligible"]]
    repeated = [item for row in eligible for item in row["instance_rows"]]
    bootstrap = protocol["bootstrap"]
    return {
        "protocol": protocol["protocol"],
        "asset_count_direct": len(records),
        "asset_count_source_semantic": len(eligible),
        "source_semantic_exclusions": [row["asset_id"] for row in records if not row["semantic_reference_eligible"]],
        "independent_hidden_gold": False,
        "three_judge_complete": False,
        "link_count_total": sum(row["link_count"] for row in records),
        "mesh_bearing_link_count_total": sum(row["mesh_bearing_link_count"] for row in records),
        "parts_per_asset_mean": mean(records, "mesh_bearing_link_count"),
        "parts_per_asset_95ci": bootstrap_mean_ci(
            numeric_values(records, "mesh_bearing_link_count"), samples=int(bootstrap["resamples"]), seed=int(bootstrap["seed"])
        ),
        "nameability_micro": micro_ratio(records, "named_mesh_link_count", "mesh_bearing_link_count"),
        "paper_aligned_richness_candidate_mean": mean(eligible, "paper_aligned_richness_candidate"),
        "paper_aligned_richness_candidate_micro": micro_ratio(
            eligible, "named_mesh_link_count", "required_spec_instance_count"
        ),
        "paper_aligned_richness_candidate_95ci": bootstrap_mean_ci(
            numeric_values(eligible, "paper_aligned_richness_candidate"), samples=int(bootstrap["resamples"]), seed=int(bootstrap["seed"]) + 1
        ),
        "source_role_recall_macro": mean(eligible, "source_role_recall"),
        "source_role_recall_micro": micro_ratio(eligible, "matched_required_instance_count", "required_spec_instance_count"),
        "strong_match_count": sum(row["strong_match_count"] for row in eligible),
        "single_token_alias_count": sum(
            row["matched_required_instance_count"] - row["strong_match_count"] for row in eligible
        ),
        "strong_match_sensitivity_macro": mean(eligible, "strong_match_sensitivity"),
        "strong_match_sensitivity_micro": micro_ratio(eligible, "strong_match_count", "required_spec_instance_count"),
        "assignment_exact_tie_count": sum(
            item.get("specificity_margin") == 0
            for row in eligible
            for item in row["required_role_assignment"]
        ),
        "assignment_low_margin_count": sum(
            item.get("specificity_margin") is not None and item["specificity_margin"] <= 5
            for row in eligible
            for item in row["required_role_assignment"]
        ),
        "source_role_recall_95ci": bootstrap_mean_ci(
            numeric_values(eligible, "source_role_recall"), samples=int(bootstrap["resamples"]), seed=int(bootstrap["seed"]) + 2
        ),
        "functional_core_coverage_macro": mean(eligible, "functional_core_coverage"),
        "functional_core_coverage_micro": micro_ratio(
            eligible, "functional_matched_instance_count", "functional_required_instance_count"
        ),
        "instance_discriminability": (
            sum(item["distinguishable_count"] for item in repeated)
            / sum(item["required_count"] for item in repeated)
            if repeated else None
        ),
        "instance_distinguishable_count": sum(item["distinguishable_count"] for item in repeated),
        "instance_required_count": sum(item["required_count"] for item in repeated),
        "instance_applicable_groups": len(repeated),
        "core_role_assigned_links": sum(row["matched_required_instance_count"] for row in eligible),
        "extra_real_part_candidates": sum(len(row["extra_real_part_candidates"]) for row in eligible),
        "semantic_precision": None,
        "semantic_precision_status": "pending_three_independent_judges",
        "semantic_judge_recall": None,
        "semantic_judge_recall_status": "pending_three_independent_judges",
        "over_segmentation_rate": None,
        "over_segmentation_status": "unsupported_without_part_to_role_decomposition_gold",
        "cross_seed": cross,
        "limitations": protocol["limitations"],
    }


def make_judge_queue(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    queue = []
    for record in records:
        if not record["semantic_reference_eligible"]:
            continue
        assigned = {row["link"]: row["role"] for row in record["required_role_assignment"]}
        for link in record["links"]:
            if not link["has_visual"]:
                continue
            if PLACEHOLDER_RE.fullmatch(link["name"]):
                continue
            queue.append(
                {
                    "item_id": hashlib.sha256(f"{record['asset_id']}\0{link['name']}".encode()).hexdigest()[:20],
                    "asset_id": record["asset_id"],
                    "category": record["slug"],
                    "node_name": link["name"],
                    "has_visual": link["has_visual"],
                    "geometry": link["geometry"],
                    "assigned_source_role": assigned.get(link["name"]),
                    "deterministic_role_candidates": record["link_role_candidates"].get(link["name"], []),
                    "reference_roles": record["reference_roles"],
                    "reference_evidence": record["evidence"],
                    "judge_verdict": None,
                    "judge_matched_role": None,
                    "judge_reason": None,
                }
            )
    return queue


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def input_manifest(selected: list[dict[str, Any]]) -> dict[str, Any]:
    cross_files = sorted(CROSS_SEED_URDF_DIR.glob("*.urdf"))
    return {
        "gold": {"path": str(GOLD_PATH), "sha256": sha256(GOLD_PATH)},
        "protocol": {"path": str(PROTOCOL_PATH), "sha256": sha256(PROTOCOL_PATH)},
        "harness": {"path": str(Path(__file__).resolve()), "sha256": sha256(Path(__file__).resolve())},
        "selected_urdfs": [
            {"asset_id": row["asset_id"], "path": row["snapshot"], "sha256": sha256(Path(row["snapshot"]))}
            for row in selected
        ],
        "cross_seed_urdf_count": len(cross_files),
        "cross_seed_aggregate_sha256": hashlib.sha256(
            "".join(f"{path.name}:{sha256(path)}\n" for path in cross_files).encode()
        ).hexdigest(),
    }


def fmt(value: Any) -> str:
    return "N/A" if value is None else f"{float(value):.3f}"


def fmt_ci(value: Any, ci: list[float] | None) -> str:
    return f"{fmt(value)} [{fmt(ci[0])}, {fmt(ci[1])}]" if ci else fmt(value)


def write_markdown(summary: dict[str, Any], records: list[dict[str, Any]], output: Path) -> None:
    lines = [
        "# Nano3D Table 2 Naming 评测报告（paper-aligned v2）",
        "",
        "流程按论文拆成 Parts → Named → Richness 三个直接/候选 gate，再单独处理 semantic judges。源文档角色只用于 count-aware recall；未命中 core-role 的命名节点进入 extra-real-part judge queue，不再自动算作 precision 假阳性。",
        "",
        "## 汇总",
        "",
        "| Metric | Result | Status |",
        "|---|---:|---|",
        f"| Parts（mesh-bearing URDF links/asset） | {fmt_ci(summary['parts_per_asset_mean'], summary['parts_per_asset_95ci'])} | direct GLB-node proxy; N=33 |",
        f"| Named / Nameability | {fmt(summary['nameability_micro'])} | direct; mesh-bearing links |",
        f"| Paper-aligned Naming Richness | {fmt_ci(summary['paper_aligned_richness_candidate_mean'], summary['paper_aligned_richness_candidate_95ci'])} | candidate proxy; named mesh links/spec instances; N=32 |",
        f"| Naming Richness micro | {fmt(summary['paper_aligned_richness_candidate_micro'])} | supplementary; pooled 233/149 |",
        f"| Source-role Recall | {fmt_ci(summary['source_role_recall_macro'], summary['source_role_recall_95ci'])} | source-derived, count-aware macro; micro={fmt(summary['source_role_recall_micro'])} |",
        f"| Strong-match sensitivity | {fmt(summary['strong_match_sensitivity_micro'])} | conservative; {summary['strong_match_count']}/149 without single-token aliases |",
        f"| Functional Core Coverage | {fmt(summary['functional_core_coverage_macro'])} | source-derived macro; micro={fmt(summary['functional_core_coverage_micro'])} |",
        f"| Instance Discriminability | {fmt(summary['instance_discriminability'])} | {summary['instance_distinguishable_count']}/{summary['instance_required_count']} instances across {summary['instance_applicable_groups']} groups |",
        "| Semantic Precision | N/A | pending three independent judges |",
        "| Semantic Judge Recall | N/A | pending three independent judges |",
        f"| Cross-Seed raw name Jaccard | pair-micro {fmt(summary['cross_seed']['raw_pairwise_name_jaccard_mean'])}; cohort-macro {fmt(summary['cross_seed']['raw_pairwise_name_jaccard_cohort_macro'])} | supplementary; {summary['cross_seed']['multi_seed_cohort_count']} cohorts |",
        f"| Cross-Seed role-count Jaccard | pair-micro {fmt(summary['cross_seed']['source_role_count_weighted_jaccard_mean'])}; cohort-macro {fmt(summary['cross_seed']['source_role_count_weighted_jaccard_cohort_macro'])} | source-derived supplementary proxy |",
        "| Over-Segmentation Rate | N/A | missing part-to-role decomposition gold; not a paper Naming metric |",
        "",
        f"Semantic source subset excludes `{summary['source_semantic_exclusions'][0]}` because its v1 role list was output-derived fallback rather than copied independent source evidence.",
        f"The judge queue contains {summary['core_role_assigned_links']} assigned required-role links and {summary['extra_real_part_candidates']} additional named-part candidates. Additional candidates are not false positives until judges decide that they are invalid or hallucinated.",
        "",
        "## 资产级明细",
        "",
        "| Asset | Mesh Parts | Named | Richness candidate* | Source Recall* | Strong sensitivity* | Functional Coverage* | Instance* | Extra judge candidates | Eligible |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in records:
        lines.append(
            f"| {row['asset_id']} | {row['mesh_bearing_link_count']} | {row['named_mesh_link_count']} | "
            f"{fmt(row['paper_aligned_richness_candidate'])} | {fmt(row['source_role_recall'])} | {fmt(row['strong_match_sensitivity'])} | "
            f"{fmt(row['functional_core_coverage'])} | {fmt(row['instance_discriminability'])} | "
            f"{len(row['extra_real_part_candidates'])} | {'yes' if row['semantic_reference_eligible'] else 'no'} |"
        )
    lines.extend(
        [
            "",
            "`*` 表示 source-derived/candidate proxy，不是论文三 judge 的独立 semantic 结果。",
            "",
            "## 关键口径修正",
            "",
            "- 原 `matched core-role links/asset = 6.818` 不再作为 Part Exists；论文的 Parts gate 统计产出的部件节点，因此本地改为 mesh-bearing URDF link/asset。",
            "- 原 `matched core-role links / all links = 0.934` 不再称为 Semantic Precision；core-role list 不覆盖额外真实部件，未命中不等于 hallucination。",
            "- Recall 展开 `min_count` 后做一对一最大匹配，不再只按 role 是否出现计分。",
            "- Richness 使用论文方向的 `named parts / spec part instances`，但在 judge 完成前明确标为 candidate proxy。",
            "- Instance 指标按 required instance 加权，不再使用每个 role group 等权的 15/18 布尔通过率。",
            "- 另报 strong-match sensitivity：只接受 canonical role token 或至少两 token 的已冻结 pattern，用于显示单 token aliases 对 1.000 Recall 的影响。",
            "- 跨 seed 同时报 pair-micro 与 cohort-macro，避免 seed 较多的 cohort 在唯一汇总值中占更高权重。",
            "- 10,000 次 bootstrap 以 asset 为重采样单元，随机种子冻结在 protocol 文件中。",
            "",
            "## 仍不能证明",
            "",
            "- 未完成三独立 judge，因此不能报告论文同口径 Semantic Precision/Recall 或 judge-validated Richness。",
            "- 没有 point/mesh-level semantic masks，因此不是 segmentation IoU，也不能测严格 Over-Segmentation。",
            "- URDF link 与论文 GLB mesh node 仅为表示层代理，不应直接横向排名。",
        ]
    )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    if EXP_ROOT not in output.parents and output != EXP_ROOT:
        raise RuntimeError(f"output must remain under {EXP_ROOT}")
    output.mkdir(parents=True, exist_ok=True)
    gold, protocol = load_inputs()
    selected = read_selected_assets()
    records = [evaluate_asset(row, gold) for row in selected]
    cross = cross_seed_eval(gold)
    enrich_cross_seed_summary(cross, protocol)
    summary = summarize(records, cross, protocol)
    judge_queue = make_judge_queue(records)
    manifest = input_manifest(selected)

    (output / "asset_records.json").write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "cross_seed_records.json").write_text(json.dumps(cross, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "input_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "judge_queue.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in judge_queue), encoding="utf-8"
    )
    write_markdown(summary, records, output / "report.md")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"judge_queue_items={len(judge_queue)}")
    print(f"outputs={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
