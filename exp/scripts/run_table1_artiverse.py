#!/usr/bin/env python3
"""Evaluate Artiverse Table 1 scale and structural-diversity metrics."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import copy
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import shlex
from statistics import mean, median
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_ARTIVERSE_ROOT = REPO / "exp/artiverse"
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_artiverse"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
SELECTION_PROTOCOL = "artiverse-table1-global-sample-v1"
TOPOLOGY_PROTOCOL = "rooted-joint-tree-v1"
FINGERPRINT_PROTOCOL = "simulation-package-fingerprint-v2"
MTL_TEXTURE_DIRECTIVES = {
    "bump",
    "decal",
    "disp",
    "map_bump",
    "map_d",
    "map_ka",
    "map_kd",
    "map_ke",
    "map_ks",
    "map_ns",
    "norm",
    "refl",
}
ORDER_INSENSITIVE_CHILD_TAGS = {
    "actuator",
    "collision",
    "geometry",
    "inertial",
    "joint",
    "link",
    "material",
    "sensor",
    "transmission",
    "visual",
}
NUMERIC_ATTRIBUTES: dict[str, set[str]] = {
    "axis": {"xyz"},
    "box": {"size"},
    "calibration": {"falling", "rising"},
    "color": {"rgba"},
    "cylinder": {"length", "radius"},
    "dynamics": {"damping", "friction"},
    "inertia": {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"},
    "limit": {"effort", "lower", "upper", "velocity"},
    "mass": {"value"},
    "mesh": {"scale"},
    "mimic": {"multiplier", "offset"},
    "origin": {"rpy", "xyz"},
    "safety_controller": {
        "k_position",
        "k_velocity",
        "soft_lower_limit",
        "soft_upper_limit",
    },
    "sphere": {"radius"},
}
NUMERIC_TEXT_TAGS = {
    "dampingFactor",
    "fdir1",
    "friction",
    "kd",
    "kp",
    "maxContacts",
    "maxVel",
    "minDepth",
    "mu1",
    "mu2",
    "selfCollide",
}


def freeze_selection(
    identities: list[dict[str, Any]],
    *,
    sample_size: int,
    seed: str,
    release_manifest_sha256: str,
) -> list[dict[str, Any]]:
    if sample_size <= 0:
        raise ValueError("sample size must be positive")
    if sample_size > len(identities):
        raise ValueError(
            f"sample size {sample_size} exceeds release size {len(identities)}"
        )
    asset_ids = [str(row["asset_id"]) for row in identities]
    if len(set(asset_ids)) != len(asset_ids):
        raise ValueError("release identities are not unique")

    ranked: list[dict[str, Any]] = []
    for row in identities:
        asset_id = str(row["asset_id"])
        payload = "\0".join(
            (SELECTION_PROTOCOL, release_manifest_sha256, str(seed), asset_id)
        ).encode("utf-8")
        ranked.append(
            {
                **row,
                "selection_hash": hashlib.sha256(payload).hexdigest(),
            }
        )
    ranked.sort(key=lambda row: (row["selection_hash"], row["asset_id"]))
    return [
        {**row, "selection_rank": rank}
        for rank, row in enumerate(ranked[:sample_size], start=1)
    ]


def _canonical_tree(
    node: str,
    adjacency: dict[str, list[tuple[str, str]]],
) -> str:
    children = sorted(
        f"{joint_type}:{_canonical_tree(child, adjacency)}"
        for child, joint_type in adjacency[node]
    )
    return f"({','.join(children)})"


def analyze_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected robot root, found {root.tag!r}")

    link_nodes = root.findall("link")
    joint_nodes = root.findall("joint")
    link_names = [node.attrib.get("name", "").strip() for node in link_nodes]
    links = set(link_names)
    names_valid = bool(link_names) and "" not in links and len(links) == len(link_names)
    adjacency: dict[str, list[tuple[str, str]]] = {name: [] for name in links if name}
    indegree: Counter[str] = Counter()
    endpoints_valid = names_valid

    for joint in joint_nodes:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "").strip() if parent_node is not None else ""
        child = child_node.attrib.get("link", "").strip() if child_node is not None else ""
        joint_type = joint.attrib.get("type", "").strip().lower()
        if parent not in links or child not in links or parent == child or not joint_type:
            endpoints_valid = False
            continue
        adjacency[parent].append((child, joint_type))
        indegree[child] += 1

    roots = sorted(name for name in adjacency if indegree[name] == 0)
    multi_parent = any(count > 1 for count in indegree.values())
    reached: set[str] = set()
    active: set[str] = set()
    cyclic = False

    def visit(node: str) -> None:
        nonlocal cyclic
        if node in active:
            cyclic = True
            return
        if node in reached:
            return
        active.add(node)
        for child, _ in adjacency[node]:
            visit(child)
        active.remove(node)
        reached.add(node)

    if len(roots) == 1:
        visit(roots[0])
    valid_tree = (
        names_valid
        and endpoints_valid
        and len(roots) == 1
        and not multi_parent
        and not cyclic
        and len(joint_nodes) == len(link_nodes) - 1
        and len(reached) == len(link_nodes)
    )
    canonical = _canonical_tree(roots[0], adjacency) if valid_tree else None
    topology_hash = (
        hashlib.sha256(f"{TOPOLOGY_PROTOCOL}\0{canonical}".encode("utf-8")).hexdigest()
        if canonical is not None
        else None
    )
    joint_type_counts = Counter(
        joint.attrib.get("type", "").strip().lower() for joint in joint_nodes
    )
    return {
        "link_count": len(link_nodes),
        "joint_count": len(joint_nodes),
        "joint_type_counts": dict(sorted(joint_type_counts.items())),
        "non_fixed_joint_count": sum(
            count for joint_type, count in joint_type_counts.items() if joint_type != "fixed"
        ),
        "valid_tree": valid_tree,
        "topology_hash": topology_hash,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _contained_resource(package_root: Path, parent: Path, reference: str) -> Path | None:
    if not reference or "://" in reference:
        return None
    candidate = (parent / reference).resolve(strict=False)
    try:
        candidate.relative_to(package_root)
    except ValueError:
        return None
    return candidate


def _resource_references(path: Path) -> list[str]:
    suffix = path.suffix.lower()
    if suffix not in {".obj", ".mtl"}:
        return []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    references: list[str] = []
    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            fields = shlex.split(line, comments=True, posix=True)
        except ValueError:
            fields = line.split()
        if not fields:
            continue
        directive = fields[0].lower()
        if suffix == ".obj" and directive == "mtllib":
            references.extend(fields[1:])
        elif suffix == ".mtl" and directive in MTL_TEXTURE_DIRECTIVES and len(fields) >= 2:
            references.append(fields[-1])
    return references


def _replace_resource_references(
    path: Path,
    dependency_digests: list[str | None],
) -> bytes:
    suffix = path.suffix.lower()
    if suffix not in {".obj", ".mtl"}:
        return path.read_bytes()
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    digest_index = 0
    canonical_lines: list[str] = []
    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            canonical_lines.append(raw_line)
            continue
        try:
            fields = shlex.split(stripped, comments=True, posix=True)
        except ValueError:
            fields = stripped.split()
        if not fields:
            canonical_lines.append(raw_line)
            continue
        directive = fields[0]
        lowered = directive.lower()
        reference_count = 0
        if suffix == ".obj" and lowered == "mtllib":
            reference_count = max(0, len(fields) - 1)
            canonical_fields = [directive]
        elif suffix == ".mtl" and lowered in MTL_TEXTURE_DIRECTIVES and len(fields) >= 2:
            reference_count = 1
            canonical_fields = fields[:-1]
        else:
            canonical_lines.append(raw_line)
            continue
        replacements = dependency_digests[digest_index : digest_index + reference_count]
        digest_index += reference_count
        canonical_fields.extend(
            f"sha256:{digest}" if digest is not None else "MISSING"
            for digest in replacements
        )
        canonical_lines.append(" ".join(canonical_fields))
    if digest_index != len(dependency_digests):
        raise ValueError(f"resource reference accounting mismatch: {path}")
    return ("\n".join(canonical_lines) + "\n").encode("utf-8")


def _resource_digest(
    path: Path,
    *,
    package_root: Path,
    cache: dict[Path, str | None],
    active: set[Path],
    resources: set[Path],
    missing: set[str],
) -> str | None:
    resolved = path.resolve(strict=False)
    try:
        relative = resolved.relative_to(package_root)
    except ValueError:
        missing.add(f"OUTSIDE_PACKAGE:{path}")
        return None
    resources.add(resolved)
    if resolved in cache:
        return cache[resolved]
    if not resolved.is_file():
        missing.add(relative.as_posix())
        cache[resolved] = None
        return None
    if resolved in active:
        missing.add(f"RESOURCE_CYCLE:{relative.as_posix()}")
        cache[resolved] = None
        return None

    active.add(resolved)
    dependency_digests: list[str | None] = []
    complete = True
    for reference in _resource_references(resolved):
        dependency = _contained_resource(package_root, resolved.parent, reference)
        if dependency is None:
            missing.add(f"UNRESOLVED:{reference}")
            complete = False
            dependency_digests.append(None)
            continue
        dependency_digest = _resource_digest(
            dependency,
            package_root=package_root,
            cache=cache,
            active=active,
            resources=resources,
            missing=missing,
        )
        if dependency_digest is None:
            complete = False
        dependency_digests.append(dependency_digest)
    active.remove(resolved)
    if not complete:
        cache[resolved] = None
        return None

    payload = "\0".join(
        (
            FINGERPRINT_PROTOCOL,
            resolved.suffix.lower(),
            hashlib.sha256(
                _replace_resource_references(resolved, dependency_digests)
            ).hexdigest(),
        )
    ).encode("utf-8")
    cache[resolved] = hashlib.sha256(payload).hexdigest()
    return cache[resolved]


def _normalize_numeric_attribute(value: str) -> str:
    fields = value.split()
    if not fields:
        return value
    normalized: list[str] = []
    try:
        for field in fields:
            number = Decimal(field)
            if not number.is_finite():
                return value
            if number == 0:
                normalized.append("0")
                continue
            normalized.append(format(number.normalize(), "f"))
    except InvalidOperation:
        return value
    return " ".join(normalized)


def _canonical_xml(element: ET.Element) -> str:
    numeric_attributes = NUMERIC_ATTRIBUTES.get(element.tag, set())
    attributes = "".join(
        f" {key}={(_normalize_numeric_attribute(value) if key in numeric_attributes else value)!r}"
        for key, value in sorted(element.attrib.items())
    )
    raw_text = (element.text or "").strip()
    text = (
        _normalize_numeric_attribute(raw_text)
        if element.tag in NUMERIC_TEXT_TAGS
        else raw_text
    )
    children = [_canonical_xml(child) for child in element]
    if element.tag in ORDER_INSENSITIVE_CHILD_TAGS:
        children.sort()
    return f"<{element.tag}{attributes}>{text}{''.join(children)}</{element.tag}>"


def _joint_without_identity(joint: ET.Element) -> ET.Element:
    normalized = copy.deepcopy(joint)
    normalized.attrib.pop("name", None)
    for endpoint in (normalized.find("parent"), normalized.find("child")):
        if endpoint is not None:
            endpoint.attrib.pop("link", None)
    mimic = normalized.find("mimic")
    if mimic is not None:
        mimic.attrib.pop("joint", None)
    return normalized


def _canonical_urdf(root: ET.Element) -> str:
    if root.tag != "robot":
        raise ValueError(f"expected robot root, found {root.tag!r}")
    link_nodes = root.findall("link")
    joint_nodes = root.findall("joint")
    links: dict[str, ET.Element] = {}
    for link in link_nodes:
        name = link.attrib.get("name", "").strip()
        if not name or name in links:
            raise ValueError("URDF links must have unique nonempty names")
        links[name] = link

    adjacency: dict[str, list[tuple[ET.Element, str]]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    for joint in joint_nodes:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "").strip() if parent_node is not None else ""
        child = child_node.attrib.get("link", "").strip() if child_node is not None else ""
        if parent not in links or child not in links or parent == child:
            raise ValueError("URDF joint has invalid parent or child endpoint")
        adjacency[parent].append((joint, child))
        indegree[child] += 1
    roots = [name for name in links if indegree[name] == 0]
    if (
        len(roots) != 1
        or any(count > 1 for count in indegree.values())
        or len(joint_nodes) != len(link_nodes) - 1
    ):
        raise ValueError("URDF is not a single rooted tree")

    material_definitions: dict[str, str] = {}
    for material in root.findall("material"):
        name = material.attrib.get("name", "").strip()
        if not name:
            continue
        definition = copy.deepcopy(material)
        definition.attrib.pop("name", None)
        material_definitions[name] = hashlib.sha256(
            _canonical_xml(definition).encode("utf-8")
        ).hexdigest()

    preliminary_cache: dict[str, str] = {}
    preliminary_active: set[str] = set()

    def preliminary_node(name: str) -> str:
        if name in preliminary_cache:
            return preliminary_cache[name]
        if name in preliminary_active:
            raise ValueError("URDF joint graph contains a cycle")
        preliminary_active.add(name)
        link = copy.deepcopy(links[name])
        link.attrib.pop("name", None)
        for material in link.iter("material"):
            material_name = material.attrib.get("name", "").strip()
            if material_name in material_definitions:
                material.attrib["name"] = f"material@{material_definitions[material_name]}"
            else:
                material.attrib.pop("name", None)
        child_payloads = []
        for joint_node, child_name in adjacency[name]:
            joint = _joint_without_identity(joint_node)
            child_payloads.append(
                f"EDGE[{_canonical_xml(joint)}]{preliminary_node(child_name)}"
            )
        preliminary_active.remove(name)
        payload = f"NODE[{_canonical_xml(link)}][{''.join(sorted(child_payloads))}]"
        preliminary_cache[name] = payload
        return payload

    preliminary_node(roots[0])
    link_tokens = {
        name: "link@"
        + hashlib.sha256(preliminary_node(name).encode("utf-8")).hexdigest()
        for name in links
    }
    joint_tokens: dict[str, str] = {}
    for parent_name, child_edges in adjacency.items():
        for joint, child_name in child_edges:
            joint_name = joint.attrib.get("name", "").strip()
            if not joint_name:
                continue
            payload = "\0".join(
                (
                    preliminary_node(parent_name),
                    _canonical_xml(_joint_without_identity(joint)),
                    preliminary_node(child_name),
                )
            ).encode("utf-8")
            joint_tokens[joint_name] = "joint@" + hashlib.sha256(payload).hexdigest()

    def normalize_known_references(element: ET.Element) -> None:
        if element.tag == "link" and element.attrib.get("name") in link_tokens:
            element.attrib["name"] = link_tokens[element.attrib["name"]]
        if element.tag == "joint" and element.attrib.get("name") in joint_tokens:
            element.attrib["name"] = joint_tokens[element.attrib["name"]]
        if element.tag == "material" and element.attrib.get("name") in material_definitions:
            element.attrib["name"] = f"material@{material_definitions[element.attrib['name']]}"
        if element.tag == "transmission":
            element.attrib.pop("name", None)
        joint_reference = element.attrib.get("joint")
        if joint_reference in joint_tokens:
            element.attrib["joint"] = joint_tokens[joint_reference]
        for attribute in ("link", "link1", "link2", "parent", "child"):
            reference = element.attrib.get(attribute)
            if reference in link_tokens:
                element.attrib[attribute] = link_tokens[reference]
        reference = element.attrib.get("reference")
        if reference in link_tokens:
            element.attrib["reference"] = link_tokens[reference]
        elif reference in joint_tokens:
            element.attrib["reference"] = joint_tokens[reference]
        for child in element:
            normalize_known_references(child)

    visited: set[str] = set()
    active: set[str] = set()

    def canonical_node(name: str) -> str:
        if name in active:
            raise ValueError("URDF joint graph contains a cycle")
        if name in visited:
            raise ValueError("URDF joint graph reuses a child link")
        active.add(name)
        link = copy.deepcopy(links[name])
        normalize_known_references(link)
        link.attrib.pop("name", None)
        link_payload = _canonical_xml(link)
        child_payloads: list[str] = []
        for joint_node, child_name in adjacency[name]:
            joint = copy.deepcopy(joint_node)
            normalize_known_references(joint)
            joint.attrib.pop("name", None)
            for endpoint in (joint.find("parent"), joint.find("child")):
                if endpoint is not None:
                    endpoint.attrib.pop("link", None)
            child_payloads.append(
                f"EDGE[{_canonical_xml(joint)}]{canonical_node(child_name)}"
            )
        active.remove(name)
        visited.add(name)
        return f"NODE[{link_payload}][{''.join(sorted(child_payloads))}]"

    tree_payload = canonical_node(roots[0])
    if len(visited) != len(links):
        raise ValueError("URDF joint graph is disconnected")

    robot_attributes = {
        key: value for key, value in root.attrib.items() if key != "name"
    }
    attributes = "".join(
        f" {key}={_normalize_numeric_attribute(value)!r}"
        for key, value in sorted(robot_attributes.items())
    )
    standard_extras: list[str] = []
    extension_extras: list[str] = []
    for element in root:
        if element.tag in {"link", "joint"}:
            continue
        extra = copy.deepcopy(element)
        normalize_known_references(extra)
        if extra.tag == "material" and extra.attrib.get("name", "").startswith("material@"):
            extra.attrib.pop("name", None)
        canonical_extra = _canonical_xml(extra)
        if extra.tag in {"material", "transmission"}:
            standard_extras.append(canonical_extra)
        else:
            extension_extras.append(canonical_extra)
    return (
        f"ROBOT[{attributes}][{tree_payload}]"
        f"[STANDARD:{''.join(sorted(standard_extras))}]"
        f"[EXTENSIONS:{''.join(extension_extras)}]"
    )


def fingerprint_package(
    urdf_path: Path,
    package_root: Path | None = None,
) -> dict[str, Any]:
    """Fingerprint URDF resources under the declared package root.

    Most releases store each URDF beside its resources.  Shared-container
    releases such as PhysX-Mobility keep URDFs below ``urdf/`` while mesh
    paths resolve from the release root, so callers may provide that frozen
    package root explicitly.
    """

    package_root = (package_root or urdf_path.parent).resolve()
    try:
        urdf_path.resolve().relative_to(package_root)
    except ValueError as error:
        raise ValueError("URDF is outside declared package root") from error
    root = copy.deepcopy(ET.parse(urdf_path).getroot())
    resources: set[Path] = set()
    missing: set[str] = set()
    cache: dict[Path, str | None] = {}

    for resource_node in [*root.findall(".//mesh"), *root.findall(".//texture")]:
        reference = resource_node.attrib.get("filename", "")
        # URDF mesh/texture references are relative to the URDF's directory,
        # while ``package_root`` may be a shared release container (PhysX).
        resource = _contained_resource(
            package_root,
            urdf_path.resolve().parent,
            reference,
        )
        if resource is None:
            missing.add(f"UNRESOLVED:{reference}")
            continue
        digest = _resource_digest(
            resource,
            package_root=package_root,
            cache=cache,
            active=set(),
            resources=resources,
            missing=missing,
        )
        if digest is not None:
            resource_node.attrib["filename"] = f"sha256:{digest}"

    if missing:
        return {
            "complete": False,
            "fingerprint": None,
            "resource_count": len(resources),
            "missing_resources": sorted(missing),
        }

    canonical = _canonical_urdf(root)
    fingerprint = hashlib.sha256(
        f"{FINGERPRINT_PROTOCOL}\0{canonical}".encode("utf-8")
    ).hexdigest()
    return {
        "complete": True,
        "fingerprint": fingerprint,
        "resource_count": len(resources),
        "missing_resources": [],
    }


def _distribution(values: list[int]) -> dict[str, Any]:
    if not values:
        return {
            "denominator": 0,
            "mean": None,
            "median": None,
            "p90_nearest_rank": None,
        }
    ordered = sorted(values)
    p90 = ordered[math.ceil(0.9 * len(ordered)) - 1]
    return {
        "denominator": len(ordered),
        "mean": mean(ordered),
        "median": median(ordered),
        "p90_nearest_rank": p90,
    }


def _category_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    parsed = [row for row in records if row.get("parse_success")]
    movable = [int(row["non_fixed_joint_count"]) for row in parsed]
    topology_hashes = [
        str(row["topology_hash"])
        for row in records
        if row.get("valid_tree") and row.get("topology_hash")
    ]
    fingerprints = [
        str(row["package_fingerprint"])
        for row in records
        if row.get("fingerprint_complete") and row.get("package_fingerprint")
    ]
    fingerprint_counts = Counter(fingerprints)
    duplicate_excess = len(fingerprints) - len(fingerprint_counts)
    multi_joint = sum(value >= 2 for value in movable)
    return {
        "N_eval": len(records),
        "N_parse": len(parsed),
        "links_per_asset": _distribution([int(row["link_count"]) for row in parsed]),
        "movable_joints_per_asset": _distribution(movable),
        "multi_joint_assets": {
            "numerator": multi_joint,
            "denominator": len(records),
            "rate": multi_joint / len(records) if records else None,
        },
        "unique_topologies": {
            "unique": len(set(topology_hashes)),
            "denominator": len(topology_hashes),
            "rate": len(set(topology_hashes)) / len(topology_hashes)
            if topology_hashes
            else None,
        },
        "exact_duplicate_rate": {
            "duplicate_excess": duplicate_excess,
            "denominator": len(fingerprints),
            "rate": duplicate_excess / len(fingerprints) if fingerprints else None,
        },
    }


def aggregate_records(
    records: list[dict[str, Any]],
    *,
    release_asset_count: int,
    release_category_count: int,
) -> dict[str, Any]:
    n_eval = len(records)
    parsed = [row for row in records if row.get("parse_success")]
    links = [int(row["link_count"]) for row in parsed]
    movable = [int(row["non_fixed_joint_count"]) for row in parsed]
    topology_hashes = [
        str(row["topology_hash"])
        for row in records
        if row.get("valid_tree") and row.get("topology_hash")
    ]
    fingerprints = [
        str(row["package_fingerprint"])
        for row in records
        if row.get("fingerprint_complete") and row.get("package_fingerprint")
    ]
    fingerprint_counts = Counter(fingerprints)
    duplicate_counts = [count for count in fingerprint_counts.values() if count > 1]
    multi_joint = sum(value >= 2 for value in movable)
    joint_type_counts: Counter[str] = Counter()
    for row in parsed:
        joint_type_counts.update(row.get("joint_type_counts") or {})
    records_by_category: dict[str, list[dict[str, Any]]] = {}
    for row in records:
        records_by_category.setdefault(str(row["raw_category"]), []).append(row)
    category_breakdown = {
        category: _category_metrics(records_by_category[category])
        for category in sorted(records_by_category)
    }
    category_topology_rates = [
        row["unique_topologies"]["rate"]
        for row in category_breakdown.values()
        if row["unique_topologies"]["rate"] is not None
    ]
    category_duplicate_rates = [
        row["exact_duplicate_rate"]["rate"]
        for row in category_breakdown.values()
        if row["exact_duplicate_rate"]["rate"] is not None
    ]

    return {
        "cohort": {
            "N_release": release_asset_count,
            "N_eval": n_eval,
            "N_parse": len(parsed),
            "release_raw_categories": release_category_count,
            "eval_raw_categories": len({str(row["raw_category"]) for row in records}),
            "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        },
        "links_per_asset": _distribution(links),
        "movable_joints_per_asset": _distribution(movable),
        "declared_joint_type_counts": dict(sorted(joint_type_counts.items())),
        "multi_joint_assets": {
            "numerator": multi_joint,
            "denominator": n_eval,
            "rate": multi_joint / n_eval if n_eval else None,
            "valid_only_denominator": len(parsed),
            "valid_only_rate": multi_joint / len(parsed) if parsed else None,
        },
        "unique_topologies": {
            "unique": len(set(topology_hashes)),
            "denominator": len(topology_hashes),
            "rate": len(set(topology_hashes)) / len(topology_hashes)
            if topology_hashes
            else None,
            "coverage_denominator": n_eval,
            "coverage_rate": len(topology_hashes) / n_eval if n_eval else None,
        },
        "exact_duplicate_rate": {
            "duplicate_excess": len(fingerprints) - len(fingerprint_counts),
            "unique": len(fingerprint_counts),
            "denominator": len(fingerprints),
            "rate": (len(fingerprints) - len(fingerprint_counts)) / len(fingerprints)
            if fingerprints
            else None,
            "assets_in_duplicate_clusters": sum(duplicate_counts),
            "assets_in_duplicate_clusters_rate": sum(duplicate_counts) / len(fingerprints)
            if fingerprints
            else None,
            "duplicate_cluster_count": len(duplicate_counts),
            "max_cluster_size": max(duplicate_counts, default=1 if fingerprints else 0),
            "coverage_denominator": n_eval,
            "coverage_rate": len(fingerprints) / n_eval if n_eval else None,
        },
        "category_macro": {
            "category_count": len(category_breakdown),
            "multi_joint_assets_rate": mean(
                row["multi_joint_assets"]["rate"] for row in category_breakdown.values()
            )
            if category_breakdown
            else None,
            "unique_topologies_evaluable_categories": len(category_topology_rates),
            "unique_topologies_rate": mean(category_topology_rates)
            if category_topology_rates
            else None,
            "exact_duplicate_evaluable_categories": len(category_duplicate_rates),
            "exact_duplicate_rate": mean(category_duplicate_rates)
            if category_duplicate_rates
            else None,
        },
        "category_breakdown": category_breakdown,
    }


def load_release_manifest(artiverse_root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest_path = artiverse_root / "dataset_chunks/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("format") != "artiverse-data-tar-gz-chunks-v1":
        raise ValueError(f"unexpected release manifest format: {manifest.get('format')!r}")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or len(chunks) != manifest.get("chunk_count"):
        raise ValueError("release manifest chunk count mismatch")

    identities: list[dict[str, Any]] = []
    for chunk in chunks:
        roots = chunk.get("roots")
        if not isinstance(roots, list) or len(roots) != chunk.get("model_count"):
            raise ValueError(f"chunk root count mismatch: {chunk.get('archive')!r}")
        for root_text in roots:
            parts = Path(str(root_text)).parts
            if len(parts) != 4 or parts[0] != "data" or Path(str(root_text)).is_absolute():
                raise ValueError(f"invalid manifest root: {root_text!r}")
            identities.append(
                {
                    "asset_id": str(root_text),
                    "manifest_root": str(root_text),
                    "raw_category": parts[1],
                    "source": parts[2],
                    "model_id": parts[3],
                    "chunk_archive": str(chunk.get("archive", "")),
                }
            )
    if len(identities) != manifest.get("model_count"):
        raise ValueError("release manifest model count mismatch")
    if len({row["asset_id"] for row in identities}) != len(identities):
        raise ValueError("release manifest contains duplicate asset identities")
    return manifest, identities


def _atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def write_json(path: Path, value: Any) -> None:
    _atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    _atomic_write_text(
        path,
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
    )


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def evaluate_asset(artiverse_root: Path, identity: dict[str, Any]) -> dict[str, Any]:
    record: dict[str, Any] = {
        **identity,
        "status": None,
        "parse_success": False,
        "link_count": None,
        "joint_count": None,
        "joint_type_counts": None,
        "non_fixed_joint_count": None,
        "valid_tree": False,
        "topology_hash": None,
        "fingerprint_complete": False,
        "package_fingerprint": None,
        "referenced_resource_count": None,
        "missing_resources": [],
        "error": None,
    }
    model_root = (artiverse_root / str(identity["manifest_root"])).resolve(strict=False)
    try:
        model_root.relative_to(artiverse_root.resolve())
    except ValueError:
        record["status"] = "INVALID_MANIFEST_ROOT"
        record["error"] = f"asset root escapes Artiverse root: {model_root}"
        return record
    if not model_root.is_dir():
        record["status"] = "MISSING_ASSET_ROOT"
        record["error"] = f"missing asset root: {identity['manifest_root']}"
        return record

    package = model_root / "urdf_w_collider"
    if not package.is_dir():
        record["status"] = "MISSING_URDF_PACKAGE"
        record["error"] = "missing urdf_w_collider directory"
        return record
    candidates = sorted(path for path in package.glob("*.urdf") if path.is_file())
    record["primary_urdf_candidates"] = [
        path.relative_to(artiverse_root).as_posix() for path in candidates
    ]
    if len(candidates) == 0:
        record["status"] = "MISSING_PRIMARY_URDF"
        record["error"] = "expected exactly one top-level URDF; found 0"
        return record
    if len(candidates) != 1:
        record["status"] = "AMBIGUOUS_PRIMARY_URDF"
        record["error"] = f"expected exactly one top-level URDF; found {len(candidates)}"
        return record

    urdf_path = candidates[0]
    record["primary_urdf"] = urdf_path.relative_to(artiverse_root).as_posix()
    try:
        record["primary_urdf_sha256"] = sha256_file(urdf_path)
    except OSError as error:
        record["status"] = "URDF_READ_FAILED"
        record["error"] = _error_text(error)
        return record
    try:
        analysis = analyze_urdf(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "URDF_PARSE_FAILED"
        record["error"] = _error_text(error)
        return record
    record.update(analysis)
    record["parse_success"] = True

    try:
        fingerprint = fingerprint_package(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "FINGERPRINT_FAILED"
        record["error"] = _error_text(error)
        return record
    record["fingerprint_complete"] = bool(fingerprint["complete"])
    record["package_fingerprint"] = fingerprint["fingerprint"]
    record["referenced_resource_count"] = fingerprint["resource_count"]
    record["missing_resources"] = fingerprint["missing_resources"]
    if fingerprint["complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced package resources are unavailable"
    return record


def _evaluate_asset_fail_closed(
    artiverse_root: Path,
    identity: dict[str, Any],
) -> dict[str, Any]:
    try:
        return evaluate_asset(artiverse_root, identity)
    except Exception as error:
        return {
            **identity,
            "status": "ASSET_EVALUATION_FAILED",
            "parse_success": False,
            "link_count": None,
            "joint_count": None,
            "joint_type_counts": None,
            "non_fixed_joint_count": None,
            "valid_tree": False,
            "topology_hash": None,
            "fingerprint_complete": False,
            "package_fingerprint": None,
            "referenced_resource_count": None,
            "missing_resources": [],
            "error": _error_text(error),
        }


def _report(summary: dict[str, Any], run_manifest: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]
    macro = summary["category_macro"]

    def percentage(value: float | None) -> str:
        return "N/E" if value is None else f"{100.0 * value:.2f}%"

    def statistic(value: float | int | None, *, decimals: int | None = None) -> str:
        if value is None:
            return "N/E"
        if decimals is None:
            return str(value)
        return f"{value:.{decimals}f}"

    return "\n".join(
        (
            "# Artiverse Table 1: Dataset Scale and Structural Diversity",
            "",
            "## Frozen cohort",
            "",
            f"- Release snapshot: pre-release manifest `{run_manifest['release_manifest_sha256']}`.",
            f"- `N_release`: {cohort['N_release']} assets across {cohort['release_raw_categories']} raw categories.",
            f"- `N_eval`: {cohort['N_eval']} globally sampled assets across {cohort['eval_raw_categories']} raw categories.",
            f"- Selection: deterministic salted SHA-256 rank, seed `{run_manifest['seed']}`; no replacement or outcome filtering.",
            "- This is not the shared-category balanced cohort.",
            "",
            "## Table 1 result",
            "",
            "| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| Artiverse | 5,402 | {cohort['N_release']} | {cohort['N_eval']} | "
                f"{cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | "
                f"{statistic(links['mean'], decimals=3)} / "
                f"{statistic(links['median'], decimals=3)} / "
                f"{statistic(links['p90_nearest_rank'])} "
                f"(n={links['denominator']}) | "
                f"{statistic(movable['mean'], decimals=3)} / "
                f"{statistic(movable['median'], decimals=3)} / "
                f"{statistic(movable['p90_nearest_rank'])} "
                f"(n={movable['denominator']}) | "
                f"{multi['numerator']} / {multi['denominator']} ({percentage(multi['rate'])}) | "
                f"{topology['unique']} / {topology['denominator']} ({percentage(topology['rate'])}); "
                f"coverage {topology['denominator']} / {topology['coverage_denominator']} "
                f"({percentage(topology['coverage_rate'])}) | "
                f"{duplicate['duplicate_excess']} / {duplicate['denominator']} "
                f"({percentage(duplicate['rate'])}); coverage {duplicate['denominator']} / "
                f"{duplicate['coverage_denominator']} ({percentage(duplicate['coverage_rate'])}) |"
            ),
            "",
            "## Diagnostics",
            "",
            f"- XML parse coverage: {cohort['N_parse']} / {cohort['N_eval']} ({percentage(cohort['N_parse'] / cohort['N_eval'] if cohort['N_eval'] else None)}).",
            f"- Category macro over {macro['category_count']} sampled raw categories: multi-joint {percentage(macro['multi_joint_assets_rate'])}; unique topologies {percentage(macro['unique_topologies_rate'])} over {macro['unique_topologies_evaluable_categories']} evaluable categories; exact duplicate rate {percentage(macro['exact_duplicate_rate'])} over {macro['exact_duplicate_evaluable_categories']} evaluable categories.",
            f"- Assets in duplicate clusters: {duplicate['assets_in_duplicate_clusters']} / {duplicate['denominator']} ({percentage(duplicate['assets_in_duplicate_clusters_rate'])}); {duplicate['duplicate_cluster_count']} clusters, maximum size {duplicate['max_cluster_size']}.",
            "- Topology hashes describe URDF representation structure, not semantic joint correctness.",
            "- Movable-joint counts include all declared XML joints except literal `fixed`, including exporter extension types; this is not a runtime-valid DoF count.",
            "- Unique-topology rate is defined over valid rooted trees only; coverage against `N_eval` is reported separately.",
            "- Exact duplicate rate uses canonicalized URDF plus the recursively resolved simulation resource closure; incomplete closures are not treated as unique.",
            "",
        )
    )


@contextmanager
def _output_lock(output: Path):
    output = output.absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.parent / f".{output.name}.lock"
    handle = lock_path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f"output is locked by another run: {output}") from error
        handle.seek(0)
        handle.truncate()
        handle.write(
            json.dumps(
                {
                    "pid": os.getpid(),
                    "output": str(output),
                    "acquired_at_utc": datetime.now(timezone.utc).isoformat(),
                },
                sort_keys=True,
            )
            + "\n"
        )
        handle.flush()
        yield
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def _verify_staged_artifacts(output: Path) -> None:
    manifest_path = output / "artifact_manifest.json"
    artifact_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    files = artifact_manifest.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("artifact manifest has no files")
    for name, expected in files.items():
        path = output / name
        if not path.is_file():
            raise ValueError(f"staged artifact is missing: {name}")
        if path.stat().st_size != expected.get("bytes"):
            raise ValueError(f"staged artifact byte count mismatch: {name}")
        if sha256_file(path) != expected.get("sha256"):
            raise ValueError(f"staged artifact hash mismatch: {name}")


def _publish_staged_output(staging: Path, output: Path) -> None:
    runs_root = output.parent / f".{output.name}.runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    old_version: Path | None = None
    if output.is_symlink():
        old_version = output.resolve(strict=True)
        try:
            old_version.relative_to(runs_root.resolve())
        except ValueError as error:
            raise RuntimeError(
                f"existing output pointer is outside its managed run directory: {output}"
            ) from error
    elif output.exists():
        raise RuntimeError(
            f"existing output is not an atomic run pointer; choose a new output path: {output}"
        )

    version = runs_root / f"run.{uuid.uuid4().hex}"
    staging.replace(version)
    temporary_pointer = output.parent / f".{output.name}.pointer.{uuid.uuid4().hex}"
    try:
        temporary_pointer.symlink_to(os.path.relpath(version, output.parent))
        os.replace(temporary_pointer, output)
    except BaseException:
        if temporary_pointer.is_symlink():
            temporary_pointer.unlink()
        if version.exists():
            shutil.rmtree(version)
        raise
    if old_version is not None and old_version != version:
        shutil.rmtree(old_version)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    artiverse_root = args.artiverse_root.resolve()
    output = output.resolve()
    protocol = args.protocol.resolve()
    try:
        output.relative_to(artiverse_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the Artiverse dataset root")

    release_manifest_path = artiverse_root / "dataset_chunks/manifest.json"
    release_manifest_sha256 = sha256_file(release_manifest_path)
    release_manifest, identities = load_release_manifest(artiverse_root)
    universe_bytes = "".join(
        f"{asset_id}\n" for asset_id in sorted(row["asset_id"] for row in identities)
    ).encode("utf-8")
    universe_sha256 = hashlib.sha256(universe_bytes).hexdigest()
    selected = freeze_selection(
        identities,
        sample_size=args.sample_size,
        seed=str(args.seed),
        release_manifest_sha256=release_manifest_sha256,
    )
    started_at = datetime.now(timezone.utc).isoformat()
    run_manifest: dict[str, Any] = {
        "schema_version": 1,
        "dataset": "Artiverse",
        "release_status": "PRE_RELEASE_SUBSET",
        "paper_reported_assets": 5402,
        "paper_reported_categories": 88,
        "N_release": len(identities),
        "release_raw_category_count": len({row["raw_category"] for row in identities}),
        "N_eval": len(selected),
        "seed": str(args.seed),
        "selection_protocol": SELECTION_PROTOCOL,
        "selection_policy": (
            "SHA256(protocol_id + NUL + release_manifest_sha256 + NUL + seed + "
            "NUL + asset_id), ascending by (digest, asset_id); no replacement or outcome filtering"
        ),
        "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        "missing_or_failed_assets_retained": True,
        "release_manifest": release_manifest_path.relative_to(artiverse_root).as_posix(),
        "release_manifest_sha256": release_manifest_sha256,
        "release_manifest_created_utc": release_manifest.get("created_utc"),
        "release_manifest_declared_file_count": release_manifest.get("file_count"),
        "release_manifest_declared_input_bytes": release_manifest.get("input_bytes"),
        "release_universe_sha256": universe_sha256,
        "protocol": str(protocol),
        "protocol_sha256": sha256_file(protocol),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256_file(Path(__file__).resolve()),
        "topology_protocol": TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": FINGERPRINT_PROTOCOL,
        "movable_joint_policy": (
            "all declared XML joint elements whose normalized type is not literal fixed; "
            "includes exporter extension types and does not imply runtime-valid DoFs"
        ),
        "topology_denominator_policy": (
            "unique topology hashes divided by assets with valid rooted joint trees; "
            "coverage is reported against N_eval"
        ),
        "duplicate_denominator_policy": (
            "duplicate excess divided by assets with complete simulation-package fingerprints; "
            "coverage is reported against N_eval"
        ),
        "p90_definition": "nearest-rank: sorted_values[ceil(0.90 * n) - 1]",
        "workers": args.workers,
        "python_executable": sys.executable,
        "python_version": sys.version,
        "started_at_utc": started_at,
        "assets": selected,
    }
    write_json(output / "manifest.json", run_manifest)

    if args.workers == 1:
        records = [_evaluate_asset_fail_closed(artiverse_root, row) for row in selected]
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            records = list(
                executor.map(
                    lambda row: _evaluate_asset_fail_closed(artiverse_root, row),
                    selected,
                )
            )
    write_jsonl(output / "asset_records.jsonl", records)

    summary = aggregate_records(
        records,
        release_asset_count=len(identities),
        release_category_count=len({row["raw_category"] for row in identities}),
    )
    summary["status_counts"] = dict(sorted(Counter(row["status"] for row in records).items()))
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(output / "summary.json", summary)
    _atomic_write_text(output / "report.md", _report(summary, run_manifest))

    artifact_files = ("manifest.json", "asset_records.jsonl", "summary.json", "report.md")
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    artiverse_root = args.artiverse_root.resolve()
    try:
        output.relative_to(artiverse_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the Artiverse dataset root")

    with _output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            _verify_staged_artifacts(staging)
            _publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artiverse-root", type=Path, default=DEFAULT_ARTIVERSE_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--sample-size", type=int, default=800)
    parser.add_argument("--seed", default="20260813")
    parser.add_argument("--workers", type=int, default=4)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "N_release": summary["cohort"]["N_release"],
                "N_eval": summary["cohort"]["N_eval"],
                "N_parse": summary["cohort"]["N_parse"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
