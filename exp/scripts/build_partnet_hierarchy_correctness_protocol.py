#!/usr/bin/env python3
"""Freeze the official PartNet hierarchy ontology and lexical role mapper."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = EXP_ROOT.parent
WORKSPACE = REPO_ROOT.parent
PARTNET = EXP_ROOT / "baselines/partnet_dataset-official"
LABEL_ROOT = PARTNET / "stats/after_merging_label_ids"
OUTPUT = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"

CATEGORIES = {
    "storage_furniture": "StorageFurniture",
    "table": "Table",
    "refrigerator": "Refrigerator",
    "dishwasher": "Dishwasher",
    "microwave": "Microwave",
}

ROLE_RULES: dict[str, list[tuple[str, str]]] = {
    "dishwasher": [
        (r"door[_ ]?frame", "door_frame"),
        (r"handle|pull", "handle"),
        (r"door", "door"),
        (r"foot", "foot"),
        (r"surface", "surface"),
        (r"base", "base"),
        (r"frame", "frame"),
        (r"body|shell|housing|cabinet|chassis", "body"),
        (r"dishwasher", "dishwasher"),
    ],
    "refrigerator": [
        (r"door[_ ]?frame", "door_frame"),
        (r"handle|pull", "handle"),
        (r"door", "door"),
        (r"interior|cavity", "body_interior"),
        (r"shelf", "shelf"),
        (r"foot", "foot"),
        (r"surface", "surface"),
        (r"base", "base"),
        (r"frame", "frame"),
        (r"body|shell|housing|cabinet|chassis|fridge", "body"),
        (r"refrigerator", "refrigerator"),
    ],
    "microwave": [
        (r"turntable|glass[_ ]?plate|rotating[_ ]?plate|tray", "tray"),
        (r"door[_ ]?frame", "door_frame"),
        (r"handle|pull", "handle"),
        (r"door", "door"),
        (r"interior|cavity", "body_interior"),
        (r"foot", "foot"),
        (r"base", "base"),
        (r"frame", "frame"),
        (r"body|shell|housing|cabinet|chassis", "body"),
        (r"microwave", "microwave"),
    ],
    "storage_furniture": [
        (r"drawer[_ ]?back", "drawer_back"),
        (r"drawer[_ ]?bottom|drawer[_ ]?floor", "drawer_bottom"),
        (r"drawer[_ ]?side", "drawer_side"),
        (r"drawer[_ ]?front", "drawer_front"),
        (r"drawer[_ ]?box", "drawer_box"),
        (r"door[_ ]?surface", "cabinet_door_surface"),
        (r"drawer.*handle|drawer.*pull", "handle"),
        (r"door.*handle|door.*pull", "handle"),
        (r"handle|pull", "handle"),
        (r"cabinet[_ ]?door|door", "cabinet_door"),
        (r"drawer", "drawer"),
        (r"counter[_ ]?top|countertop", "countertop"),
        (r"shelf", "shelf"),
        (r"cabinet[_ ]?frame|frame", "cabinet_frame"),
        (r"caster[_ ]?stem", "caster_stem"),
        (r"caster", "caster"),
        (r"wheel", "wheel"),
        (r"hinge", "hinge"),
        (r"foot", "foot"),
        (r"cabinet[_ ]?base|base", "cabinet_base"),
        (r"body|shell|housing|cabinet|chassis", "cabinet"),
        (r"storage[_ ]?furniture", "storage_furniture"),
    ],
    "table": [
        (r"drawer[_ ]?back", "drawer_back"),
        (r"drawer[_ ]?bottom|drawer[_ ]?floor", "drawer_bottom"),
        (r"drawer[_ ]?side", "drawer_side"),
        (r"drawer[_ ]?front", "drawer_front"),
        (r"drawer[_ ]?box", "drawer_box"),
        (r"drawer.*handle|drawer.*pull", "handle"),
        (r"handle|pull", "handle"),
        (r"table[_ ]?top.*drop|drop[_ ]?leaf", "tabletop_dropleaf"),
        (r"table[_ ]?top[_ ]?surface", "tabletop_surface"),
        (r"table[_ ]?top[_ ]?frame", "tabletop_frame"),
        (r"table[_ ]?top|tabletop", "tabletop"),
        (r"table[_ ]?base|base", "table_base"),
        (r"cabinet[_ ]?door|door", "cabinet_door"),
        (r"drawer", "drawer"),
        (r"keyboard[_ ]?tray", "keyboard_tray"),
        (r"shelf", "shelf"),
        (r"caster[_ ]?stem", "caster_stem"),
        (r"caster", "caster"),
        (r"wheel", "wheel"),
        (r"central[_ ]?support", "central_support"),
        (r"pedestal", "pedestal"),
        (r"bar[_ ]?stretcher|stretcher", "bar_stretcher"),
        (r"runner", "runner"),
        (r"foot", "foot"),
        (r"leg", "leg"),
        (r"bench", "bench"),
        (r"body|shell|housing|chassis|table", "table"),
    ],
}


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


def parse_hierarchy_file(path: Path) -> dict[str, Any]:
    entries = []
    paths = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        label_id, semantic_path, node_kind = line.strip().split(maxsplit=2)
        roles = semantic_path.split("/")
        entries.append(
            {
                "label_id": int(label_id),
                "path": semantic_path,
                "roles": roles,
                "node_kind": node_kind.strip(),
            }
        )
        paths.append(roles)
    ancestor_distances: dict[tuple[str, str], int] = {}
    for roles in paths:
        for child_index, child in enumerate(roles):
            for parent_index, parent in enumerate(roles[:child_index]):
                key = (parent, child)
                distance = child_index - parent_index
                ancestor_distances[key] = min(ancestor_distances.get(key, distance), distance)
    return {
        "entries": entries,
        "roles": sorted({role for roles in paths for role in roles}),
        "ancestor_relations": [
            {"ancestor": parent, "descendant": child, "distance": distance}
            for (parent, child), distance in sorted(ancestor_distances.items())
        ],
    }


def main() -> int:
    repo = contained(PARTNET)
    commit = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "-C", str(repo), "status", "--porcelain"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if status:
        raise RuntimeError("PartNet checkout is not clean")
    categories = {}
    for category_id, official_name in CATEGORIES.items():
        source = contained(LABEL_ROOT / f"{official_name}-hier.txt")
        ontology = parse_hierarchy_file(source)
        ontology_roles = set(ontology["roles"])
        unknown_rule_roles = sorted(
            {role for _, role in ROLE_RULES[category_id]} - ontology_roles
        )
        if unknown_rule_roles:
            raise RuntimeError(
                f"{category_id} role rules reference roles absent from the official "
                f"ontology: {unknown_rule_roles}"
            )
        categories[category_id] = {
            "official_category": official_name,
            "source_file": str(source.relative_to(repo)),
            "source_sha256": sha256(source),
            "ontology": ontology,
            "ordered_role_rules": [
                {"pattern": pattern, "role": role}
                for pattern, role in ROLE_RULES[category_id]
            ],
        }
    payload = {
        "protocol_id": "partnet_hierarchy_correctness_v1",
        "claim_boundary": (
            "Prediction-side lexical role recovery plus category-level PartNet semantic "
            "ontology alignment. PartNet is not URDF kinematic-tree ground truth, and this "
            "protocol is not instance-matched PartNet geometry or human hierarchy annotation."
        ),
        "source": {
            "repository": "https://github.com/daerduoCarey/partnet_dataset",
            "commit": commit,
            "license": "MIT",
            "license_sha256": sha256(contained(repo / "LICENSE")),
        },
        "categories": categories,
        "mapping": {
            "normalization": "lowercase; non-alphanumeric runs become underscore; numeric suffixes retained but do not affect regex token matches",
            "selection": "first matching ordered category rule; one role maximum per URDF link",
            "unmapped_policy": "retain as unmapped and include in semantic role coverage denominator",
            "ambiguous_policy": "do not infer geometry or use tested source hierarchy; only an explicit ordered rule may assign a role",
        },
        "scoring": {
            "induced_reference_parent": "nearest PartNet ancestor role present in the same predicted asset",
            "induced_edge_f1": "node-instance edge TP/FP/FN after collapsing unmapped URDF wrappers; repeated role instances remain separate child-link records",
            "induced_hierarchy_exact_match": "all ontology-induced expected edges recovered with no extra mapped-role edges; unscorable assets fail requested-denominator exact match",
            "semantic_parent_alignment": "mapped child instances whose nearest mapped predicted parent is a nearest present PartNet ancestor",
            "coverage_weighted_induced_edge_f1": "per-asset induced Edge F1 multiplied by mapped predicted links divided by all final URDF links; unavailable or unscorable requested assets contribute zero",
            "required_companion_metrics": [
                "semantic_role_coverage",
                "scorable_asset_coverage",
                "conditional_and_requested_denominators",
            ],
        },
    }
    contained(OUTPUT.parent, exists=False).mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(OUTPUT), "sha256": sha256(OUTPUT)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
