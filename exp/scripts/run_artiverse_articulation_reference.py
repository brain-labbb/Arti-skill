#!/usr/bin/env python3
"""Fail-closed Artiverse release audit and Table 6 static adapter.

This runner never treats Artiverse annotations as independent semantic gold.
The static phase reports release/export self-consistency and freezes a
content-independent N=100 motion-intent cohort. Motion and collision scoring
remain separate downstream phases under the frozen protocol.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Iterable
import xml.etree.ElementTree as ET


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_ARTIVERSE = EXP_ROOT / "artiverse"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/table6_artiverse_articulation_reference_v1.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/table6_artiverse_reference_v1"

FROZEN_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
FROZEN_MODEL_COUNT = 3544
FROZEN_FILE_COUNT = 531_937
FROZEN_INPUT_BYTES = 86_992_752_890
FROZEN_CATEGORY_COUNT = 84
FROZEN_SOURCE_COUNT = 10
FROZEN_CHUNK_COUNT = 2
SELECTION_SALT = "nano3d-table6-artiverse-reference-v1"
MOTION_COHORT_SIZE = 100

TYPE_ALIASES = {"rotation": "revolute", "translation": "prismatic"}
MOTION_TYPES = {"revolute", "continuous", "prismatic", "screw", "cylindrical", "universal"}
COMPOSITE_PRECEDENCE = ("screw", "cylindrical", "universal")
EXPECTED_URDF_TYPES = {
    "revolute": ("revolute",),
    "continuous": ("continuous",),
    "prismatic": ("prismatic",),
    "screw": ("revolute",),
    "cylindrical": ("prismatic", "continuous"),
    "universal": ("revolute", "revolute"),
}
SEMANTIC_DOF = {
    "revolute": 1,
    "continuous": 1,
    "prismatic": 1,
    "screw": 1,
    "cylindrical": 2,
    "universal": 2,
}
PID_FROM_NAME = re.compile(r"^joint_([^_]+)_to_")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("preflight", "static"), default="preflight")
    parser.add_argument("--artiverse-root", type=Path, default=DEFAULT_ARTIVERSE)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--verify-archives",
        action="store_true",
        help="hash both 65 GB source archives; mandatory for --phase static",
    )
    return parser.parse_args()


def contained(path: Path, *, must_exist: bool) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def identity_hash(identity: str) -> str:
    return hashlib.sha256(f"{SELECTION_SALT}|{identity}".encode()).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    os.replace(temporary, path)


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def manifest_identities(manifest: dict[str, Any]) -> tuple[list[dict[str, str]], list[str]]:
    identities: list[dict[str, str]] = []
    errors: list[str] = []
    for chunk in manifest.get("chunks", []):
        archive = str(chunk.get("archive", ""))
        roots = chunk.get("roots")
        if not isinstance(roots, list):
            errors.append(f"chunk {archive!r} roots is not a list")
            continue
        if int(chunk.get("model_count", -1)) != len(roots):
            errors.append(f"chunk {archive!r} model_count does not match roots")
        for root in roots:
            parts = Path(str(root)).parts
            if len(parts) != 4 or parts[0] != "data":
                errors.append(f"invalid manifest root: {root!r}")
                continue
            identities.append(
                {
                    "manifest_root": str(root),
                    "category": parts[1],
                    "source": parts[2],
                    "model_id": parts[3],
                    "identity": "/".join(parts[1:]),
                    "archive": archive,
                }
            )
    roots = [row["manifest_root"] for row in identities]
    if len(set(roots)) != len(roots):
        errors.append("manifest roots are not unique")
    return identities, errors


def actual_model_roots(data_root: Path) -> tuple[set[str], list[str]]:
    roots: set[str] = set()
    errors: list[str] = []
    if not data_root.is_dir():
        return roots, [f"missing data directory: {data_root}"]
    for category in sorted(data_root.iterdir()):
        if category.is_symlink() or not category.is_dir():
            errors.append(f"unexpected category entry: {category.name}")
            continue
        for source in sorted(category.iterdir()):
            if source.is_symlink() or not source.is_dir():
                errors.append(f"unexpected source entry: {category.name}/{source.name}")
                continue
            for model in sorted(source.iterdir()):
                if model.is_symlink() or not model.is_dir():
                    errors.append(
                        f"unexpected model entry: {category.name}/{source.name}/{model.name}"
                    )
                    continue
                roots.add(f"data/{category.name}/{source.name}/{model.name}")
    return roots, errors


def scan_tree(data_root: Path) -> dict[str, Any]:
    file_count = 0
    input_bytes = 0
    symlinks: list[str] = []
    nonregular: list[str] = []
    for dirpath, dirnames, filenames in os.walk(data_root, followlinks=False):
        base = Path(dirpath)
        kept_dirs = []
        for dirname in dirnames:
            path = base / dirname
            if path.is_symlink():
                if len(symlinks) < 100:
                    symlinks.append(path.relative_to(data_root.parent).as_posix())
            else:
                kept_dirs.append(dirname)
        dirnames[:] = kept_dirs
        for filename in filenames:
            path = base / filename
            if path.is_symlink():
                if len(symlinks) < 100:
                    symlinks.append(path.relative_to(data_root.parent).as_posix())
                continue
            if not path.is_file():
                if len(nonregular) < 100:
                    nonregular.append(path.relative_to(data_root.parent).as_posix())
                continue
            stat = path.stat()
            file_count += 1
            input_bytes += stat.st_size
    return {
        "file_count": file_count,
        "input_bytes": input_bytes,
        "symlink_count": len(symlinks),
        "symlink_examples": symlinks,
        "nonregular_count": len(nonregular),
        "nonregular_examples": nonregular,
    }


def run_preflight(
    artiverse: Path,
    protocol_path: Path,
    *,
    verify_archives: bool,
) -> tuple[dict[str, Any], list[dict[str, str]]]:
    protocol_hash = sha256_file(protocol_path)
    protocol = read_json(protocol_path)
    manifest_path = artiverse / "dataset_chunks/manifest.json"
    manifest_hash = sha256_file(manifest_path) if manifest_path.is_file() else None
    manifest = read_json(manifest_path) if manifest_hash else {}
    identities, manifest_errors = manifest_identities(manifest)
    expected_roots = {row["manifest_root"] for row in identities}
    categories = {row["category"] for row in identities}
    sources = {row["source"] for row in identities}

    invariant_checks = {
        "manifest_sha256": manifest_hash == FROZEN_MANIFEST_SHA256,
        "model_count": manifest.get("model_count") == FROZEN_MODEL_COUNT,
        "file_count": manifest.get("file_count") == FROZEN_FILE_COUNT,
        "input_bytes": manifest.get("input_bytes") == FROZEN_INPUT_BYTES,
        "chunk_count": manifest.get("chunk_count") == FROZEN_CHUNK_COUNT,
        "root_count": len(identities) == FROZEN_MODEL_COUNT,
        "unique_root_count": len(expected_roots) == FROZEN_MODEL_COUNT,
        "category_count": len(categories) == FROZEN_CATEGORY_COUNT,
        "source_count": len(sources) == FROZEN_SOURCE_COUNT,
        "manifest_schema": not manifest_errors,
        "protocol_id": protocol.get("protocol_id")
        == "nano3d_table6_artiverse_articulation_reference_v1",
    }

    unpack_log = artiverse / "unpack.log"
    unpack_marker = f"[UNPACK] reconstructed {artiverse / 'data'}"
    unpack_complete = (
        unpack_log.is_file()
        and unpack_marker in unpack_log.read_text(encoding="utf-8", errors="replace")
    )
    invariant_checks["unpack_completion_marker"] = unpack_complete

    actual_roots, layout_errors = actual_model_roots(artiverse / "data")
    missing_roots = sorted(expected_roots - actual_roots)
    extra_roots = sorted(actual_roots - expected_roots)
    root_gate = (
        all(invariant_checks.values())
        and not layout_errors
        and not missing_roots
        and not extra_roots
    )

    tree_stats: dict[str, Any] | None = None
    if root_gate:
        tree_stats = scan_tree(artiverse / "data")
        invariant_checks["extracted_file_count"] = tree_stats["file_count"] == FROZEN_FILE_COUNT
        invariant_checks["extracted_input_bytes"] = tree_stats["input_bytes"] == FROZEN_INPUT_BYTES
        invariant_checks["no_symlinks"] = tree_stats["symlink_count"] == 0
        invariant_checks["regular_files_only"] = tree_stats["nonregular_count"] == 0
        roots_after, layout_after = actual_model_roots(artiverse / "data")
        invariant_checks["stable_root_snapshot"] = roots_after == actual_roots and not layout_after
    else:
        invariant_checks.update(
            {
                "extracted_file_count": False,
                "extracted_input_bytes": False,
                "no_symlinks": False,
                "regular_files_only": False,
                "stable_root_snapshot": False,
            }
        )

    archive_checks: dict[str, Any] = {}
    for chunk in manifest.get("chunks", []):
        name = str(chunk.get("archive", ""))
        archive_path = artiverse / "dataset_chunks" / name
        item = {
            "exists": archive_path.is_file(),
            "declared_sha256": chunk.get("sha256"),
            "declared_bytes": chunk.get("archive_bytes"),
            "actual_bytes": archive_path.stat().st_size if archive_path.is_file() else None,
            "actual_sha256": None,
            "sha256_state": "NOT_CHECKED",
        }
        if verify_archives and archive_path.is_file():
            item["actual_sha256"] = sha256_file(archive_path)
            item["sha256_state"] = (
                "PASS" if item["actual_sha256"] == item["declared_sha256"] else "FAIL"
            )
        archive_checks[name] = item
    archives_ready = (
        verify_archives
        and len(archive_checks) == FROZEN_CHUNK_COUNT
        and all(
            item["exists"]
            and item["actual_bytes"] == item["declared_bytes"]
            and item["sha256_state"] == "PASS"
            for item in archive_checks.values()
        )
    )
    extraction_ready = all(invariant_checks.values())
    paper_ready = extraction_ready and archives_ready
    blockers = [key for key, value in invariant_checks.items() if not value]
    if not archives_ready:
        blockers.append("archive_sha256" if verify_archives else "archive_sha256_not_checked")
    readiness = {
        "protocol_id": protocol.get("protocol_id"),
        "protocol_sha256": protocol_hash,
        "manifest_path": manifest_path.relative_to(WORKSPACE).as_posix(),
        "manifest_sha256": manifest_hash,
        "state": "READY" if paper_ready else "BLOCKED",
        "extraction_ready": extraction_ready,
        "paper_ready_inputs": paper_ready,
        "static_audit_authorized": paper_ready,
        "blockers": blockers,
        "invariant_checks": invariant_checks,
        "declared_root_count": len(expected_roots),
        "actual_root_count": len(actual_roots),
        "missing_root_count": len(missing_roots),
        "missing_root_examples": missing_roots[:25],
        "extra_root_count": len(extra_roots),
        "extra_root_examples": extra_roots[:25],
        "layout_error_count": len(layout_errors),
        "layout_error_examples": layout_errors[:25],
        "manifest_errors": manifest_errors[:25],
        "tree_stats": tree_stats,
        "archive_checks": archive_checks,
        "fail_closed_action": (
            "STATIC_AUDIT_ALLOWED" if paper_ready else "READINESS_EVIDENCE_ONLY"
        ),
    }
    return readiness, identities


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def finite_vector(value: Any, *, nonzero: bool = False) -> bool:
    if not isinstance(value, list) or len(value) != 3 or not all(finite_number(v) for v in value):
        return False
    return not nonzero or math.sqrt(sum(float(v) ** 2 for v in value)) > 1e-12


def normalize_type(value: Any) -> str:
    raw = str(value).strip().lower()
    return TYPE_ALIASES.get(raw, raw)


def semantic_type(records: list[dict[str, Any]]) -> str | None:
    normalized = [normalize_type(item.get("type")) for item in records]
    types = set(normalized)
    for joint_type in COMPOSITE_PRECEDENCE:
        if joint_type in types:
            return joint_type
    if "revolute" in types and "prismatic" in types:
        return "cylindrical"
    if sum(joint_type in {"revolute", "continuous"} for joint_type in normalized) >= 2:
        return "universal"
    for joint_type in ("continuous", "revolute", "prismatic"):
        if joint_type in types:
            return joint_type
    return None


def audit_annotations(path: Path) -> dict[str, Any]:
    try:
        value = read_json(path)
        raw = value.get("articulations")
        if not isinstance(raw, list) or not all(isinstance(item, dict) for item in raw):
            raise ValueError("articulations must be a list of objects")
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        raw_types: Counter[str] = Counter()
        missing_pid = 0
        for item in raw:
            raw_types[normalize_type(item.get("type"))] += 1
            if item.get("pid") is None:
                missing_pid += 1
                continue
            grouped[str(item["pid"])].append(item)

        semantic: list[dict[str, Any]] = []
        unsupported_types = sorted(set(raw_types) - MOTION_TYPES - {"fixed", "free"})
        for pid, items in sorted(grouped.items()):
            joint_type = semantic_type(items)
            if joint_type is None:
                continue
            by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
            for item in items:
                by_type[normalize_type(item.get("type"))].append(item)
            representative = next(
                (item for item in items if normalize_type(item.get("type")) == joint_type),
                items[0],
            )
            if joint_type == "universal":
                revolute_records = [
                    item
                    for item in items
                    if normalize_type(item.get("type")) in {"revolute", "continuous"}
                ]
                if len(revolute_records) >= 2 and normalize_type(
                    representative.get("type")
                ) != "universal":
                    first, second = revolute_records[:2]
                    axes_valid = finite_vector(first.get("axis"), nonzero=True) and finite_vector(
                        second.get("axis"), nonzero=True
                    )
                    origins_valid = finite_vector(first.get("origin")) and finite_vector(
                        second.get("origin")
                    )
                    ranges = (
                        (first.get("rangeMin"), first.get("rangeMax")),
                        (second.get("rangeMin"), second.get("rangeMax")),
                    )
                else:
                    axes_valid = finite_vector(
                        representative.get("axis", representative.get("axis1")), nonzero=True
                    ) and finite_vector(
                        representative.get("axis2"), nonzero=True
                    )
                    origins_valid = finite_vector(
                        representative.get("origin", representative.get("origin1"))
                    ) and finite_vector(
                        representative.get("origin2")
                    )
                    ranges = (
                        (
                            representative.get("rangeMin", representative.get("rot1RangeMin")),
                            representative.get("rangeMax", representative.get("rot1RangeMax")),
                        ),
                        (
                            representative.get("rot2RangeMin", representative.get("range2Min")),
                            representative.get("rot2RangeMax", representative.get("range2Max")),
                        ),
                    )
            elif joint_type == "cylindrical" and "cylindrical" not in by_type:
                revolute = by_type["revolute"][0]
                prismatic = by_type["prismatic"][0]
                axes_valid = finite_vector(revolute.get("axis"), nonzero=True) and finite_vector(
                    prismatic.get("axis"), nonzero=True
                )
                origins_valid = finite_vector(revolute.get("origin")) and finite_vector(
                    prismatic.get("origin")
                )
                ranges = (
                    (revolute.get("rangeMin"), revolute.get("rangeMax")),
                    (
                        prismatic.get("prismatic_rangeMin", prismatic.get("rangeMin")),
                        prismatic.get("prismatic_rangeMax", prismatic.get("rangeMax")),
                    ),
                )
            else:
                axes_valid = finite_vector(representative.get("axis"), nonzero=True)
                origins_valid = finite_vector(representative.get("origin"))
                if joint_type == "cylindrical":
                    ranges = (
                        (
                            representative.get(
                                "rangeMin", representative.get("rotRangeMin", -math.pi)
                            ),
                            representative.get(
                                "rangeMax", representative.get("rotRangeMax", math.pi)
                            ),
                        ),
                        (
                            representative.get("prismatic_rangeMin"),
                            representative.get("prismatic_rangeMax"),
                        ),
                    )
                elif joint_type == "screw":
                    ranges = (
                        (
                            representative.get("rangeMin", representative.get("rotRangeMin")),
                            representative.get("rangeMax", representative.get("rotRangeMax")),
                        ),
                        (
                            representative.get(
                                "transRangeMin", representative.get("prismatic_rangeMin")
                            ),
                            representative.get(
                                "transRangeMax", representative.get("prismatic_rangeMax")
                            ),
                        ),
                    )
                elif joint_type == "continuous":
                    ranges = ((-math.pi, math.pi),)
                elif joint_type == "prismatic":
                    ranges = (
                        (
                            representative.get(
                                "rangeMin", representative.get("prismatic_rangeMin")
                            ),
                            representative.get(
                                "rangeMax", representative.get("prismatic_rangeMax")
                            ),
                        ),
                    )
                else:
                    ranges = (
                        (
                            representative.get("rangeMin", representative.get("rotRangeMin")),
                            representative.get("rangeMax", representative.get("rotRangeMax")),
                        ),
                    )
            ranges_valid = all(
                finite_number(lower) and finite_number(upper) and float(lower) <= float(upper)
                for lower, upper in ranges
            )
            semantic.append(
                {
                    "pid": pid,
                    "type": joint_type,
                    "semantic_dof": SEMANTIC_DOF[joint_type],
                    "axis_metadata_valid": axes_valid,
                    "origin_metadata_valid": origins_valid,
                    "range_metadata_valid": ranges_valid,
                    "coordinate_ranges": [
                        [float(lower), float(upper)]
                        if finite_number(lower) and finite_number(upper)
                        else None
                        for lower, upper in ranges
                    ],
                    "raw_record_count": len(items),
                }
            )
        return {
            "valid": True,
            "error": None,
            "raw_record_count": len(raw),
            "raw_type_counts": dict(sorted(raw_types.items())),
            "missing_pid_count": missing_pid,
            "unsupported_types": unsupported_types,
            "semantic_joint_count": len(semantic),
            "semantic_dof_count": sum(item["semantic_dof"] for item in semantic),
            "semantic_type_counts": dict(sorted(Counter(item["type"] for item in semantic).items())),
            "semantic_joints": semantic,
            "graph_parent_by_pid_present": isinstance(value.get("graphParentByPid"), dict),
            "object_base_id_present": value.get("objectBaseId") is not None,
        }
    except Exception as exc:
        return {
            "valid": False,
            "error": f"{type(exc).__name__}: {exc}",
            "raw_record_count": None,
            "raw_type_counts": {},
            "missing_pid_count": None,
            "unsupported_types": [],
            "semantic_joint_count": None,
            "semantic_dof_count": None,
            "semantic_type_counts": {},
            "semantic_joints": [],
            "graph_parent_by_pid_present": False,
            "object_base_id_present": False,
        }


def resolve_mesh(urdf_path: Path, filename: str) -> tuple[bool, str, Path | None]:
    if Path(filename).is_absolute():
        return False, "absolute", None
    normalized = filename.removeprefix("package://")
    candidate = (urdf_path.parent / normalized).resolve()
    try:
        candidate.relative_to(urdf_path.parent.resolve())
    except ValueError:
        return False, "escape", None
    return candidate.is_file() and candidate.stat().st_size > 0, "relative", candidate


def audit_urdf(path: Path, semantic_joints: list[dict[str, Any]]) -> dict[str, Any]:
    try:
        robot = ET.parse(path).getroot()
        if robot.tag != "robot":
            raise ValueError(f"root tag is {robot.tag!r}")
        links = robot.findall("link")
        joints = robot.findall("joint")
        link_names = [str(link.get("name")) for link in links]
        link_set = set(link_names)
        children: list[str] = []
        graph: dict[str, list[str]] = defaultdict(list)
        bad_endpoints = 0
        movable: list[dict[str, Any]] = []
        for joint in joints:
            parent = joint.find("parent")
            child = joint.find("child")
            parent_name = parent.get("link") if parent is not None else None
            child_name = child.get("link") if child is not None else None
            if parent_name not in link_set or child_name not in link_set:
                bad_endpoints += 1
            elif parent_name is not None and child_name is not None:
                graph[parent_name].append(child_name)
                children.append(child_name)
            joint_type = str(joint.get("type", ""))
            if joint_type != "fixed":
                explicit_pid = joint.get("artiverse_pid")
                match = PID_FROM_NAME.match(str(joint.get("name", "")))
                heuristic_pid = match.group(1) if match else None
                movable.append(
                    {
                        "name": joint.get("name"),
                        "type": joint_type,
                        "pid": str(explicit_pid) if explicit_pid is not None else heuristic_pid,
                        "mapping_mode": "explicit" if explicit_pid is not None else "joint_name_heuristic",
                        "original_type": joint.get("artiverse_original_type"),
                        "component": joint.get("artiverse_component"),
                        "parent": parent_name,
                        "child": child_name,
                        "lower": (
                            float(joint.find("limit").get("lower"))
                            if joint.find("limit") is not None
                            and joint.find("limit").get("lower") is not None
                            else (-math.pi if joint_type == "continuous" else None)
                        ),
                        "upper": (
                            float(joint.find("limit").get("upper"))
                            if joint.find("limit") is not None
                            and joint.find("limit").get("upper") is not None
                            else (math.pi if joint_type == "continuous" else None)
                        ),
                        "screw_pitch": joint.get("artiverse_screw_pitch"),
                        "trans_min": joint.get("artiverse_trans_min"),
                        "trans_max": joint.get("artiverse_trans_max"),
                    }
                )

        roots = sorted(link_set - set(children))
        duplicate_children = len(children) != len(set(children))
        visited: set[str] = set()
        active: set[str] = set()

        def visit(node: str) -> bool:
            if node in active:
                return False
            if node in visited:
                return True
            active.add(node)
            for child_name in graph.get(node, []):
                if not visit(child_name):
                    return False
            active.remove(node)
            visited.add(node)
            return True

        acyclic = all(visit(root) for root in roots) and len(visited) == len(link_set)
        tree_valid = (
            len(link_names) == len(link_set)
            and len(roots) == 1
            and not duplicate_children
            and not bad_endpoints
            and acyclic
            and len(joints) == max(0, len(links) - 1)
        )

        visual_links = [link for link in links if link.findall("visual")]
        visual_with_collision = sum(bool(link.findall("collision")) for link in visual_links)
        collision_elements = robot.findall(".//collision")
        collision_meshes = [mesh for collision in collision_elements for mesh in collision.findall(".//mesh")]
        missing_collision_meshes: list[str] = []
        invalid_collision_meshes: list[str] = []
        collision_mesh_inventory: dict[str, dict[str, Any]] = {}
        for mesh in collision_meshes:
            filename = mesh.get("filename")
            if not filename:
                invalid_collision_meshes.append("[missing filename]")
                continue
            valid, reference_kind, candidate = resolve_mesh(path, filename)
            if not valid:
                missing_collision_meshes.append(filename)
            elif candidate is not None:
                relative = candidate.relative_to(WORKSPACE).as_posix()
                collision_mesh_inventory[relative] = {
                    "path": relative,
                    "bytes": candidate.stat().st_size,
                    "urdf_reference_kind": reference_kind,
                }

        positive_mass = 0
        positive_inertia_diagonal = 0
        for link in links:
            mass = link.find("./inertial/mass")
            inertia = link.find("./inertial/inertia")
            if mass is not None:
                try:
                    value = float(str(mass.get("value")))
                    positive_mass += int(math.isfinite(value) and value > 0)
                except (TypeError, ValueError):
                    pass
            if inertia is not None:
                try:
                    values = [float(str(inertia.get(key))) for key in ("ixx", "iyy", "izz")]
                    positive_inertia_diagonal += int(all(math.isfinite(v) and v > 0 for v in values))
                except (TypeError, ValueError):
                    pass

        by_pid: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for item in movable:
            if item["pid"] is not None:
                by_pid[item["pid"]].append(item)
        mapping_rows = []
        for semantic in semantic_joints:
            exported = by_pid.get(semantic["pid"], [])
            expected = Counter(EXPECTED_URDF_TYPES[semantic["type"]])
            actual = Counter(item["type"] for item in exported)
            mapping_rows.append(
                {
                    "pid": semantic["pid"],
                    "semantic_type": semantic["type"],
                    "semantic_dof": semantic["semantic_dof"],
                    "expected_urdf_types": dict(sorted(expected.items())),
                    "actual_urdf_types": dict(sorted(actual.items())),
                    "mapped_element_count": len(exported),
                    "exported_coordinates": exported,
                    "mapping_complete": actual == expected,
                    "all_explicit": bool(exported)
                    and all(item["mapping_mode"] == "explicit" for item in exported),
                }
            )
        mapping_complete = all(item["mapping_complete"] for item in mapping_rows)
        semantic_pids = {item["pid"] for item in semantic_joints}
        mapped_joint_names = {
            item["name"]
            for pid, pid_items in by_pid.items()
            if pid in semantic_pids
            for item in pid_items
            if item["name"] is not None
        }
        unmapped_movable = [item["name"] for item in movable if item["name"] not in mapped_joint_names]
        return {
            "valid": True,
            "error": None,
            "link_count": len(links),
            "joint_element_count": len(joints),
            "movable_joint_element_count": len(movable),
            "tree_valid": tree_valid,
            "root_links": roots,
            "visual_link_count": len(visual_links),
            "visual_links_with_collision_count": visual_with_collision,
            "visual_collision_coverage_complete": visual_with_collision == len(visual_links),
            "collision_element_count": len(collision_elements),
            "collision_mesh_count": len(collision_meshes),
            "missing_collision_mesh_count": len(missing_collision_meshes),
            "missing_collision_mesh_examples": missing_collision_meshes[:20],
            "invalid_collision_mesh_count": len(invalid_collision_meshes),
            "collision_mesh_inventory": [
                collision_mesh_inventory[key] for key in sorted(collision_mesh_inventory)
            ],
            "positive_mass_link_count": positive_mass,
            "positive_inertia_diagonal_link_count": positive_inertia_diagonal,
            "all_links_positive_mass": positive_mass == len(links),
            "all_links_positive_inertia_diagonal": positive_inertia_diagonal == len(links),
            "mapping_complete": mapping_complete,
            "mapping_rows": mapping_rows,
            "unmapped_movable_joint_count": len(unmapped_movable),
            "unmapped_movable_joint_examples": unmapped_movable[:20],
        }
    except Exception as exc:
        return {
            "valid": False,
            "error": f"{type(exc).__name__}: {exc}",
            "link_count": None,
            "joint_element_count": None,
            "movable_joint_element_count": None,
            "tree_valid": False,
            "visual_link_count": None,
            "visual_links_with_collision_count": None,
            "visual_collision_coverage_complete": False,
            "collision_element_count": None,
            "collision_mesh_count": None,
            "missing_collision_mesh_count": None,
            "missing_collision_mesh_examples": [],
            "invalid_collision_mesh_count": None,
            "collision_mesh_inventory": [],
            "positive_mass_link_count": None,
            "positive_inertia_diagonal_link_count": None,
            "all_links_positive_mass": False,
            "all_links_positive_inertia_diagonal": False,
            "mapping_complete": False,
            "mapping_rows": [],
            "unmapped_movable_joint_count": None,
            "unmapped_movable_joint_examples": [],
        }


def audit_asset(artiverse: Path, identity: dict[str, str]) -> dict[str, Any]:
    model_root = artiverse / identity["manifest_root"]
    model_id = identity["model_id"]
    articulation_path = model_root / f"{model_id}.articulations.json"
    glb_path = model_root / f"{model_id}.segmented.glb"
    material_path = model_root / "material.json"
    mass_path = model_root / "mass_furniture_heuristic.json"
    urdf_dir = model_root / "urdf_w_collider"
    urdf_paths = sorted(urdf_dir.glob("*.urdf")) if urdf_dir.is_dir() else []
    annotation_hash = sha256_file(articulation_path) if articulation_path.is_file() else None
    urdf_hash = sha256_file(urdf_paths[0]) if len(urdf_paths) == 1 else None
    core = {
        "segmented_glb": glb_path.is_file() and glb_path.stat().st_size > 0,
        "articulation_json": articulation_path.is_file(),
        "material_json": material_path.is_file(),
        "urdf_directory": urdf_dir.is_dir(),
        "exactly_one_urdf": len(urdf_paths) == 1,
    }
    annotation = (
        audit_annotations(articulation_path)
        if core["articulation_json"]
        else {"valid": False, "semantic_joint_count": None, "semantic_dof_count": None, "semantic_joints": [], "error": "missing"}
    )
    urdf = (
        audit_urdf(urdf_paths[0], annotation.get("semantic_joints", []))
        if len(urdf_paths) == 1 and annotation.get("valid")
        else {"valid": False, "mapping_complete": False, "error": "missing prerequisite"}
    )
    material_valid = False
    material_error = None
    if material_path.is_file():
        try:
            read_json(material_path)
            material_valid = True
        except Exception as exc:
            material_error = f"{type(exc).__name__}: {exc}"
    core_complete = all(core.values()) and annotation.get("valid") and urdf.get("valid") and material_valid
    eligible = bool(
        core_complete
        and annotation.get("semantic_joint_count", 0) > 0
        and urdf.get("movable_joint_element_count", 0) > 0
        and urdf.get("tree_valid")
        and urdf.get("visual_collision_coverage_complete")
        and urdf.get("collision_element_count", 0) > 0
        and urdf.get("collision_mesh_count", 0) > 0
        and urdf.get("missing_collision_mesh_count") == 0
        and urdf.get("invalid_collision_mesh_count") == 0
        and urdf.get("mapping_complete")
        and urdf.get("unmapped_movable_joint_count") == 0
    )
    return {
        **identity,
        "selection_hash": identity_hash(identity["identity"]),
        "articulation_path": (
            articulation_path.relative_to(WORKSPACE).as_posix()
            if articulation_path.is_file()
            else None
        ),
        "articulation_sha256": annotation_hash,
        "urdf_path": (
            urdf_paths[0].relative_to(WORKSPACE).as_posix() if len(urdf_paths) == 1 else None
        ),
        "urdf_sha256": urdf_hash,
        "core": core,
        "core_complete": core_complete,
        "annotation": annotation,
        "urdf": urdf,
        "material_json_valid": material_valid,
        "material_json_error": material_error,
        "mass_furniture_heuristic_present": mass_path.is_file(),
        "static_motion_eligible": eligible,
    }


def hamilton_quotas(records: list[dict[str, Any]], total: int) -> dict[str, int]:
    counts = Counter(record["source"] for record in records)
    population = sum(counts.values())
    exact = {source: total * count / population for source, count in counts.items()}
    quotas = {source: math.floor(value) for source, value in exact.items()}
    remainder = total - sum(quotas.values())
    order = sorted(counts, key=lambda source: (-(exact[source] - quotas[source]), source))
    for source in order[:remainder]:
        quotas[source] += 1
    return dict(sorted(quotas.items()))


def freeze_motion_cohort(records: list[dict[str, Any]]) -> dict[str, Any]:
    eligible = [record for record in records if record["static_motion_eligible"]]
    if len(eligible) < MOTION_COHORT_SIZE:
        raise ValueError(f"only {len(eligible)} assets pass static eligibility; need 100")
    quotas = hamilton_quotas(eligible, MOTION_COHORT_SIZE)
    selected: list[dict[str, Any]] = []
    selected_ids: set[str] = set()
    selected_sources: Counter[str] = Counter()
    selected_categories: Counter[str] = Counter()
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in eligible:
        by_category[record["category"]].append(record)

    # Category coverage is deterministic and precedes any load/collision outcome.
    categories = sorted(by_category)
    if len(categories) > MOTION_COHORT_SIZE:
        categories = sorted(categories, key=identity_hash)[:MOTION_COHORT_SIZE]
    for category in categories:
        ranked = sorted(
            by_category[category],
            key=lambda record: (
                -(quotas[record["source"]] - selected_sources[record["source"]]),
                record["selection_hash"],
                record["identity"],
            ),
        )
        choice = ranked[0]
        selected.append(choice)
        selected_ids.add(choice["identity"])
        selected_sources[choice["source"]] += 1
        selected_categories[choice["category"]] += 1

    while len(selected) < MOTION_COHORT_SIZE:
        candidates = [record for record in eligible if record["identity"] not in selected_ids]
        choice = min(
            candidates,
            key=lambda record: (
                -(quotas[record["source"]] - selected_sources[record["source"]]),
                selected_categories[record["category"]],
                record["selection_hash"],
                record["identity"],
            ),
        )
        selected.append(choice)
        selected_ids.add(choice["identity"])
        selected_sources[choice["source"]] += 1
        selected_categories[choice["category"]] += 1

    cohort = []
    for index, record in enumerate(selected, 1):
        semantic_types = record["annotation"].get("semantic_type_counts", {})
        collision_inventory = []
        for item in record["urdf"]["collision_mesh_inventory"]:
            mesh_path = WORKSPACE / item["path"]
            collision_inventory.append({**item, "sha256": sha256_file(mesh_path)})
        inventory_payload = json.dumps(
            collision_inventory, sort_keys=True, separators=(",", ":")
        ).encode()
        cohort.append(
            {
                "cohort_index": index,
                "identity": record["identity"],
                "manifest_root": record["manifest_root"],
                "category": record["category"],
                "source": record["source"],
                "model_id": record["model_id"],
                "selection_hash": record["selection_hash"],
                "articulation_path": record["articulation_path"],
                "articulation_sha256": record["articulation_sha256"],
                "urdf_path": record["urdf_path"],
                "urdf_sha256": record["urdf_sha256"],
                "raw_annotation_record_count": record["annotation"]["raw_record_count"],
                "semantic_joint_count": record["annotation"]["semantic_joint_count"],
                "semantic_dof_count": record["annotation"]["semantic_dof_count"],
                "exported_movable_joint_element_count": record["urdf"][
                    "movable_joint_element_count"
                ],
                "semantic_type_counts": semantic_types,
                "semantic_joints": record["annotation"]["semantic_joints"],
                "unsupported_annotation_types": record["annotation"]["unsupported_types"],
                "annotation_missing_pid_count": record["annotation"]["missing_pid_count"],
                "exported_coordinates": [
                    coordinate
                    for mapping in record["urdf"]["mapping_rows"]
                    for coordinate in mapping["exported_coordinates"]
                ],
                "collision_mesh_inventory": collision_inventory,
                "collision_inventory_sha256": hashlib.sha256(inventory_payload).hexdigest(),
                "motion_state": "INTENT_FROZEN_NOT_EXECUTED",
            }
        )
    return {
        "protocol_id": "nano3d_table6_artiverse_motion_cohort_v1",
        "selection_salt": SELECTION_SALT,
        "selection_uses_motion_or_collision_outcome": False,
        "failure_replacement": False,
        "candidate_count": len(eligible),
        "intent_size": len(cohort),
        "eligible_category_count": len(by_category),
        "hamilton_source_quotas": quotas,
        "selected_source_counts": dict(sorted(selected_sources.items())),
        "selected_category_counts": dict(sorted(selected_categories.items())),
        "assets": cohort,
    }


def aggregate_static(records: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(records)
    valid_annotations = [record for record in records if record["annotation"].get("valid")]
    valid_urdfs = [record for record in records if record["urdf"].get("valid")]
    semantic_type_counts: Counter[str] = Counter()
    raw_type_counts: Counter[str] = Counter()
    for record in valid_annotations:
        semantic_type_counts.update(record["annotation"].get("semantic_type_counts", {}))
        raw_type_counts.update(record["annotation"].get("raw_type_counts", {}))
    semantic_rows = [
        joint
        for record in valid_annotations
        for joint in record["annotation"].get("semantic_joints", [])
    ]
    totals = {
        "annotation_records": sum(record["annotation"]["raw_record_count"] for record in valid_annotations),
        "semantic_joints": sum(record["annotation"]["semantic_joint_count"] for record in valid_annotations),
        "semantic_dofs": sum(record["annotation"]["semantic_dof_count"] for record in valid_annotations),
        "exported_movable_joint_elements": sum(
            record["urdf"]["movable_joint_element_count"] for record in valid_urdfs
        ),
    }

    def coverage(predicate: Any) -> dict[str, Any]:
        count = sum(bool(predicate(record)) for record in records)
        return {"count": count, "denominator": n, "rate": rate(count, n)}

    return {
        "role": "real-data release/export reference; excluded from generated-method rankings",
        "asset_denominator": n,
        "category_count": len({record["category"] for record in records}),
        "source_count": len({record["source"] for record in records}),
        "totals_by_distinct_denominator": totals,
        "means_per_release_asset": {key: value / n for key, value in totals.items()},
        "raw_annotation_type_counts": dict(sorted(raw_type_counts.items())),
        "semantic_joint_type_counts": dict(sorted(semantic_type_counts.items())),
        "semantic_metadata_validity": {
            field: {
                "count": sum(bool(joint[field]) for joint in semantic_rows),
                "denominator": len(semantic_rows),
                "rate": rate(sum(bool(joint[field]) for joint in semantic_rows), len(semantic_rows)),
                "interpretation": "metadata validity, not accuracy",
            }
            for field in (
                "axis_metadata_valid",
                "origin_metadata_valid",
                "range_metadata_valid",
            )
        },
        "coverage": {
            "core_complete": coverage(lambda row: row["core_complete"]),
            "segmented_glb": coverage(lambda row: row["core"]["segmented_glb"]),
            "articulation_json": coverage(lambda row: row["core"]["articulation_json"]),
            "material_json": coverage(lambda row: row["core"]["material_json"]),
            "exactly_one_urdf": coverage(lambda row: row["core"]["exactly_one_urdf"]),
            "annotation_parseable": coverage(lambda row: row["annotation"].get("valid")),
            "urdf_parseable": coverage(lambda row: row["urdf"].get("valid")),
            "urdf_tree_valid": coverage(lambda row: row["urdf"].get("tree_valid")),
            "mapping_complete": coverage(lambda row: row["urdf"].get("mapping_complete")),
            "visual_collision_complete": coverage(
                lambda row: row["urdf"].get("visual_collision_coverage_complete")
            ),
            "collision_mesh_refs_resolved": coverage(
                lambda row: row["urdf"].get("valid")
                and row["urdf"].get("missing_collision_mesh_count") == 0
                and row["urdf"].get("invalid_collision_mesh_count") == 0
            ),
            "material_json_valid": coverage(lambda row: row["material_json_valid"]),
            "all_links_positive_mass": coverage(
                lambda row: row["urdf"].get("all_links_positive_mass")
            ),
            "all_links_positive_inertia_diagonal": coverage(
                lambda row: row["urdf"].get("all_links_positive_inertia_diagonal")
            ),
            "mass_furniture_heuristic_present": coverage(
                lambda row: row["mass_furniture_heuristic_present"]
            ),
            "static_motion_eligible": coverage(lambda row: row["static_motion_eligible"]),
        },
        "accuracy_boundary": {
            "joint_type_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "joint_recall": "N/A_NO_INDEPENDENT_GOLD",
            "parent_child_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "axis_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "origin_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "limit_accuracy": "N/A_NO_INDEPENDENT_GOLD",
        },
        "motion_state": "NOT_EXECUTED",
    }


def static_report(summary: dict[str, Any], cohort: dict[str, Any]) -> str:
    totals = summary["totals_by_distinct_denominator"]
    coverage = summary["coverage"]
    lines = [
        "# Artiverse Table 6 release reference",
        "",
        "Status: STATIC_COMPLETE; MOTION_INTENT_FROZEN_NOT_EXECUTED",
        "",
        (
            f"The frozen release contains {summary['asset_denominator']} assets across "
            f"{summary['category_count']} categories and {summary['source_count']} sources."
        ),
        "",
        "## Distinct denominators",
        "",
        f"- raw annotation records: {totals['annotation_records']}",
        f"- semantic joints (unique motion-bearing pid): {totals['semantic_joints']}",
        f"- semantic scalar DoFs: {totals['semantic_dofs']}",
        f"- exported movable URDF elements: {totals['exported_movable_joint_elements']}",
        "",
        "## Static release audit",
        "",
        f"- core-complete packages: {coverage['core_complete']['count']}/{summary['asset_denominator']}",
        f"- parseable annotation JSON: {coverage['annotation_parseable']['count']}/{summary['asset_denominator']}",
        f"- parseable URDF: {coverage['urdf_parseable']['count']}/{summary['asset_denominator']}",
        f"- complete semantic-to-export mapping: {coverage['mapping_complete']['count']}/{summary['asset_denominator']}",
        f"- resolved native collision mesh references: {coverage['collision_mesh_refs_resolved']['count']}/{summary['asset_denominator']}",
        f"- heuristic mass sidecar present: {coverage['mass_furniture_heuristic_present']['count']}/{summary['asset_denominator']}",
        "",
        "## Frozen motion intent",
        "",
        f"The content-hashed motion cohort has {cohort['intent_size']} assets from {cohort['eligible_category_count']} eligible categories. It was selected before any load, contact, or FCL result; failures are never replaced.",
        "",
        "## Claim boundary",
        "",
        "Artiverse is a real-data release reference, not a generated method. Annotation-to-URDF agreement is export self-consistency rather than independent semantic accuracy. Joint type accuracy, recall, parent-child accuracy, and semantic axis/origin/limit accuracy remain N/A without independent gold. Motion and collision have not yet been run.",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    artiverse = contained(args.artiverse_root, must_exist=True)
    protocol = contained(args.protocol, must_exist=True)
    output = contained(args.output, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)

    readiness, identities = run_preflight(
        artiverse,
        protocol,
        verify_archives=args.verify_archives,
    )
    write_json(output / "readiness.json", readiness)
    print(json.dumps({"state": readiness["state"], "blockers": readiness["blockers"]}))
    if readiness["state"] != "READY":
        return 2
    if args.phase == "preflight":
        return 0
    if not args.verify_archives:
        print("error: --phase static requires --verify-archives", file=sys.stderr)
        return 2

    records = [audit_asset(artiverse, identity) for identity in identities]
    summary = aggregate_static(records)
    write_jsonl(output / "static_asset_records.jsonl", records)
    write_json(output / "static_summary.json", summary)
    cohort = freeze_motion_cohort(records)
    cohort["source_static_asset_records_sha256"] = sha256_file(
        output / "static_asset_records.jsonl"
    )
    cohort["source_static_summary_sha256"] = sha256_file(output / "static_summary.json")
    cohort["source_readiness_sha256"] = sha256_file(output / "readiness.json")
    cohort["source_protocol_sha256"] = sha256_file(protocol)
    cohort["source_release_manifest_sha256"] = FROZEN_MANIFEST_SHA256
    write_json(output / "motion_cohort_manifest.json", cohort)
    report_path = output / "report.md"
    temporary_report = report_path.with_suffix(".md.tmp")
    temporary_report.write_text(static_report(summary, cohort), encoding="utf-8")
    os.replace(temporary_report, report_path)
    print(json.dumps({"state": "STATIC_COMPLETE", "assets": len(records), "cohort": len(cohort["assets"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
