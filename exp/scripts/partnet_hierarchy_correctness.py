#!/usr/bin/env python3
"""PartNet-ontology semantic hierarchy alignment scorer for final URDF artifacts."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import re
from typing import Any
import xml.etree.ElementTree as ET


CATEGORY_ALIASES = {
    "storage_furniture_cabinet": "storage_furniture",
    "storage_furniture": "storage_furniture",
    "table": "table",
    "refrigerator": "refrigerator",
    "dishwasher": "dishwasher",
    "microwave": "microwave",
}


def normalize_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def load_protocol(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def map_role(name: str, category: str, protocol: dict[str, Any]) -> dict[str, Any]:
    normalized = normalize_name(name)
    rules = protocol["categories"][category]["ordered_role_rules"]
    matches = []
    for index, rule in enumerate(rules):
        if re.search(str(rule["pattern"]), normalized):
            matches.append(
                {
                    "rule_index": index,
                    "pattern": rule["pattern"],
                    "role": rule["role"],
                }
            )
    selected = matches[0] if matches else None
    return {
        "link_name": name,
        "normalized_name": normalized,
        "mapped_role": selected["role"] if selected else None,
        "selected_rule_index": selected["rule_index"] if selected else None,
        "selected_pattern": selected["pattern"] if selected else None,
        "all_matching_roles": list(dict.fromkeys(match["role"] for match in matches)),
        "ambiguous_role_match": len({match["role"] for match in matches}) > 1,
    }


def evaluate_urdf(
    urdf_path: Path,
    category: str,
    protocol: dict[str, Any],
    link_labels: dict[str, str] | None = None,
) -> dict[str, Any]:
    category = CATEGORY_ALIASES[category]
    xml_root = ET.parse(urdf_path).getroot()
    if xml_root.tag != "robot":
        raise ValueError(f"expected robot root, found {xml_root.tag!r}")
    link_names = [node.attrib.get("name", "") for node in xml_root.findall("link")]
    if not link_names or any(not name for name in link_names):
        raise ValueError("URDF contains no links or unnamed links")
    if len(set(link_names)) != len(link_names):
        raise ValueError("URDF contains duplicate link names")
    link_set = set(link_names)
    parent_by_child: dict[str, str] = {}
    malformed_edges = []
    multi_parent_links = set()
    for joint in xml_root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set:
            malformed_edges.append(joint.attrib.get("name", f"{parent}->{child}"))
            continue
        if child in parent_by_child:
            multi_parent_links.add(child)
        parent_by_child[child] = parent

    link_labels = link_labels or {}
    mappings = {}
    for name in link_names:
        mapping_input = link_labels.get(name, name)
        mapping = map_role(mapping_input, category, protocol)
        mapping["link_name"] = name
        mapping["mapping_input"] = mapping_input
        mapping["mapping_source"] = (
            "package_part_label" if name in link_labels else "urdf_link_name"
        )
        mappings[name] = mapping
    role_by_link = {
        name: str(mapping["mapped_role"])
        for name, mapping in mappings.items()
        if mapping["mapped_role"] is not None
    }
    present_roles = set(role_by_link.values())
    ancestor_distance = {
        (row["ancestor"], row["descendant"]): int(row["distance"])
        for row in protocol["categories"][category]["ontology"]["ancestor_relations"]
    }

    def nearest_predicted_parent(link: str) -> str | None:
        seen = set()
        current = parent_by_child.get(link)
        while current is not None and current not in seen:
            seen.add(current)
            if current in role_by_link:
                return current
            current = parent_by_child.get(current)
        return None

    edge_records = []
    tp = 0
    predicted_edge_count = 0
    expected_edge_count = 0
    for link, child_role in role_by_link.items():
        predicted_parent_link = nearest_predicted_parent(link)
        predicted_parent_role = (
            role_by_link[predicted_parent_link] if predicted_parent_link is not None else None
        )
        candidates = {
            role: ancestor_distance[(role, child_role)]
            for role in present_roles
            if (role, child_role) in ancestor_distance
        }
        min_distance = min(candidates.values()) if candidates else None
        expected_parent_roles = sorted(
            role for role, distance in candidates.items() if distance == min_distance
        )
        has_expected = bool(expected_parent_roles)
        has_predicted = predicted_parent_role is not None
        correct = bool(has_expected and predicted_parent_role in expected_parent_roles)
        expected_edge_count += int(has_expected)
        predicted_edge_count += int(has_predicted)
        tp += int(correct)
        edge_records.append(
            {
                "child_link": link,
                "child_role": child_role,
                "predicted_parent_link": predicted_parent_link,
                "predicted_parent_role": predicted_parent_role,
                "expected_parent_roles": expected_parent_roles,
                "ontology_distance": min_distance,
                "correct": correct,
                "scorable": has_expected,
            }
        )
    fp = predicted_edge_count - tp
    fn = expected_edge_count - tp
    precision = tp / predicted_edge_count if predicted_edge_count else None
    recall = tp / expected_edge_count if expected_edge_count else None
    f1 = (
        2 * precision * recall / (precision + recall)
        if precision is not None and recall is not None and precision + recall > 0
        else 0.0 if expected_edge_count or predicted_edge_count else None
    )
    scorable = expected_edge_count > 0
    exact = bool(scorable and fp == 0 and fn == 0)
    nesting_accuracy = tp / expected_edge_count if expected_edge_count else None
    return {
        "category": category,
        "link_count": len(link_names),
        "mapped_link_count": len(role_by_link),
        "semantic_role_coverage": len(role_by_link) / len(link_names),
        "package_part_label_count": sum(name in link_labels for name in link_names),
        "distinct_mapped_role_count": len(present_roles),
        "mapped_role_counts": dict(sorted(Counter(role_by_link.values()).items())),
        "ambiguous_role_match_count": sum(
            bool(mapping["ambiguous_role_match"]) for mapping in mappings.values()
        ),
        "unmapped_link_names": sorted(set(link_names) - set(role_by_link)),
        "mapping_records": [mappings[name] for name in link_names],
        "malformed_edge_count": len(malformed_edges),
        "multi_parent_link_count": len(multi_parent_links),
        "scorable": scorable,
        "expected_edge_count": expected_edge_count,
        "predicted_edge_count": predicted_edge_count,
        "true_positive_edge_count": tp,
        "false_positive_edge_count": fp,
        "false_negative_edge_count": fn,
        "parent_child_edge_precision": precision,
        "parent_child_edge_recall": recall,
        "parent_child_edge_f1": f1,
        "induced_parent_child_edge_f1": f1,
        "hierarchy_exact_match": exact,
        "induced_hierarchy_exact_match": exact,
        "semantic_nesting_accuracy": nesting_accuracy,
        "semantic_parent_alignment": nesting_accuracy,
        "edge_records": edge_records,
    }


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    requested_count = len(records)
    available = [row for row in records if row.get("available")]
    evaluated = [row for row in available if row.get("evaluation_complete")]
    scorable = [row for row in evaluated if row.get("scorable")]
    parse_failures = [row for row in available if not row.get("evaluation_complete")]
    no_mapped_role = [
        row for row in evaluated if int(row.get("mapped_link_count", 0)) == 0
    ]
    mapped_without_induced_edge = [
        row
        for row in evaluated
        if int(row.get("mapped_link_count", 0)) > 0 and not row.get("scorable")
    ]
    requested_f1 = [float(row.get("parent_child_edge_f1") or 0.0) for row in records]
    requested_coverage_weighted_induced_f1 = [
        float(row.get("parent_child_edge_f1") or 0.0)
        * float(row.get("semantic_role_coverage") or 0.0)
        for row in records
    ]
    requested_nesting = [
        float(row.get("semantic_nesting_accuracy") or 0.0) for row in records
    ]
    tp = sum(int(row["true_positive_edge_count"]) for row in scorable)
    predicted = sum(int(row["predicted_edge_count"]) for row in scorable)
    expected = sum(int(row["expected_edge_count"]) for row in scorable)
    micro_precision = tp / predicted if predicted else None
    micro_recall = tp / expected if expected else None
    micro_f1 = (
        2 * micro_precision * micro_recall / (micro_precision + micro_recall)
        if micro_precision is not None
        and micro_recall is not None
        and micro_precision + micro_recall > 0
        else None
    )
    return {
        "requested_count": requested_count,
        "available_count": len(available),
        "evaluated_count": len(evaluated),
        "scorable_count": len(scorable),
        "unscorable_reason_counts": {
            "unavailable_no_final_urdf": requested_count - len(available),
            "available_parse_failure": len(parse_failures),
            "evaluated_no_mapped_role": len(no_mapped_role),
            "evaluated_mapped_roles_no_induced_edge": len(
                mapped_without_induced_edge
            ),
        },
        "ambiguous_mapping_asset_count": sum(
            int(row.get("ambiguous_role_match_count", 0)) > 0 for row in evaluated
        ),
        "ambiguous_role_match_count": sum(
            int(row.get("ambiguous_role_match_count", 0)) for row in evaluated
        ),
        "scorable_asset_coverage_requested": len(scorable) / requested_count,
        "semantic_role_coverage_link_micro": (
            sum(int(row["mapped_link_count"]) for row in evaluated)
            / sum(int(row["link_count"]) for row in evaluated)
            if evaluated
            else None
        ),
        "semantic_role_coverage_asset_macro": (
            sum(float(row["semantic_role_coverage"]) for row in evaluated) / len(evaluated)
            if evaluated
            else None
        ),
        "semantic_role_coverage_requested_macro": (
            sum(float(row.get("semantic_role_coverage") or 0.0) for row in records)
            / requested_count
        ),
        "parent_child_edge_f1_requested_macro": sum(requested_f1) / requested_count,
        "coverage_weighted_induced_edge_f1_requested_macro": (
            sum(requested_coverage_weighted_induced_f1) / requested_count
        ),
        "parent_child_edge_f1_conditional_macro": (
            sum(float(row["parent_child_edge_f1"]) for row in scorable) / len(scorable)
            if scorable
            else None
        ),
        "parent_child_edge_precision_conditional_micro": micro_precision,
        "parent_child_edge_recall_conditional_micro": micro_recall,
        "parent_child_edge_f1_conditional_micro": micro_f1,
        "hierarchy_exact_match_requested_count": sum(
            bool(row.get("hierarchy_exact_match")) for row in records
        ),
        "hierarchy_exact_match_requested_rate": sum(
            bool(row.get("hierarchy_exact_match")) for row in records
        )
        / requested_count,
        "hierarchy_exact_match_conditional_rate": (
            sum(bool(row["hierarchy_exact_match"]) for row in scorable) / len(scorable)
            if scorable
            else None
        ),
        "semantic_nesting_accuracy_requested_macro": sum(requested_nesting)
        / requested_count,
        "semantic_nesting_accuracy_conditional_micro": (
            tp / expected if expected else None
        ),
        "true_positive_edge_count": tp,
        "predicted_edge_count_scorable": predicted,
        "expected_edge_count_scorable": expected,
    }
