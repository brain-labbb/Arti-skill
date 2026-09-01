#!/usr/bin/env python3
"""Independently verify a LAM Supplementary Table S1 run."""

from __future__ import annotations

import argparse
from collections import deque
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
from pathlib import PurePosixPath, PureWindowsPath
import platform
import re
import shlex
import sys
import tempfile
from typing import Any, Mapping
import xml.etree.ElementTree as ET


RECEIPT_NAME_RE = re.compile(
    r"(?:^|[-_])(?:mechanical[-_])?receipt(?:[-_]|\.|$)", re.IGNORECASE
)
ALLOWANCE_NAME_RE = re.compile(
    r"allow(?:ance|list|ed)|exclu(?:de|sion)", re.IGNORECASE
)
REBUILD_RECIPE_NAMES = frozenset(
    {
        "build_recipe.json",
        "build-recipe.json",
        "rebuild_recipe.json",
        "rebuild-recipe.json",
        "deterministic_rebuild.json",
    }
)
GENERATION_CONFIG_RE = re.compile(r"^generation_config(?:\..+)?$", re.IGNORECASE)

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
RUNNER_PATH = SCRIPT.with_name("run_s1_lam.py")
STATIC_ATOMS_PATH = SCRIPT.with_name("lam_supplementary_static.py")
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
TABLE3_RUN = (
    REPO
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3"
)
TABLE3_MANIFEST = TABLE3_RUN / "manifest.json"
TABLE3_RECORDS = TABLE3_RUN / "asset_records.jsonl"
RELEASE_ROOT = REPO / "exp/Articulated-Object-Code/released_outputs"
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_lam_n800_20260814"
TABLE4_MANIFEST = TABLE4_RUN / "frozen_manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_VERIFICATION = TABLE4_RUN / "verification.json"
EXPECTED_N_EVAL = 800
EXPECTED_TABLE3_MANIFEST_SHA256 = (
    "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
)
EXPECTED_TABLE3_CONTENT_SHA256 = (
    "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
)
EXPECTED_TABLE3_RECORDS_SHA256 = (
    "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
)
EXPECTED_ORDERED_KEYS_SHA256 = (
    "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
)
EXPECTED_TABLE4_MANIFEST_SHA256 = (
    "8adc7d8698eaeab5ee5a62d881ed50d4e65c5dc80c9d1d8ae0f4a4a204474594"
)
EXPECTED_TABLE4_CONTENT_SHA256 = (
    "9a46a1cb7668666cf3c485cc35086cdd79a113d23a8b00625ede012c8b039d2d"
)
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = (
    "15423f8646be26dd01fe9d1ca5c0a1b7b1f454349e77d1562827e719c0d1d014"
)
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = (
    "ac62b73d71530982a63c1e8cf345cfda126608aa6e42ce9710383daace2af257"
)
EXPECTED_TABLE4_VERIFICATION_SHA256 = (
    "e74ed91dca984af8aba900cf3915b490fb1298e5c2bc539af7ade43570edbc51"
)
EXPECTED_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_lam_n800_v1"
EXPECTED_FORMAL_STRICT_PASSED = 91
EXPECTED_FORMAL_ALLOWANCE_MEASURED_ASSETS = 770
EXPECTED_FORMAL_ALLOWANCE_ELIGIBLE_PAIRS = 17_939
PROTOCOL_ID = "s1_lam_table3cohort_n800_seed20260813_v1"
SCHEMA_VERSION = "supplementary-s1-lam/v1"
DATASET = "LAM released outputs"
EXPECTED_PROTOCOL_SNAPSHOT_SHA256 = (
    "7e4558dee779b39759a4e622c6ff412d9ebd0f6c94e4905d27b9dc4021fbf70b"
)
TABLE4_IDENTITY_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    "asset_key",
    "category",
    "input_identity_sha256",
    "selection_rank",
    "selection_hash",
    "tier",
    "rel_path",
    "object_release_id",
    "package_relpath",
    "model_urdf_sha256",
    "package_content_manifest_sha256",
    "source_record_sha256",
    "source_manifest_record_sha256",
)
SINGLE_JOINT_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
ZERO_WIDTH_TOLERANCE = 1e-12
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_binding(package: Path) -> dict[str, Any]:
    lexical = Path(os.path.abspath(str(package)))
    if lexical.is_symlink():
        raise ValueError(f"package root is a symlink: {lexical}")
    root = lexical.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"package is not a directory: {root}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(root, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {path.relative_to(root)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            resolved.relative_to(root)
            if not resolved.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            files.append(
                {
                    "path": relative,
                    "bytes": resolved.stat().st_size,
                    "sha256": sha256_file(resolved),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def resolve_release_package(release_root: Path, rel_path: str) -> tuple[Path, Path]:
    root_lexical = Path(os.path.abspath(str(release_root)))
    if root_lexical.is_symlink():
        raise ValueError(f"release root is a symlink: {root_lexical}")
    root_resolved = root_lexical.resolve(strict=True)
    relative = Path(rel_path)
    if relative.is_absolute() or not relative.parts or any(
        part in {"", ".", ".."} for part in relative.parts
    ):
        raise ValueError(f"invalid package relative path: {rel_path!r}")

    package_lexical = root_lexical
    for part in relative.parts:
        package_lexical /= part
        if package_lexical.is_symlink():
            raise ValueError(
                f"package path contains symlink: {package_lexical.relative_to(root_lexical)}"
            )
    package = package_lexical.resolve(strict=True)
    try:
        package.relative_to(root_resolved)
    except ValueError as exc:
        raise ValueError(f"package escapes release root: {rel_path}") from exc
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {rel_path}")

    urdf_lexical = package_lexical / "generated.urdf"
    if urdf_lexical.is_symlink():
        raise ValueError(f"generated.urdf is a symlink: {rel_path}")
    urdf_path = urdf_lexical.resolve(strict=True)
    if urdf_path.parent != package or not urdf_path.is_file():
        raise ValueError(f"invalid generated.urdf: {rel_path}")
    return package, urdf_path


def _local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1]


def _children(node: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in node if _local_tag(child) == tag]


def _descendants(node: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in node.iter() if _local_tag(child) == tag]


def _safe_relative_path(raw: str, *, field: str) -> PurePosixPath:
    if not isinstance(raw, str) or not raw or "\\" in raw or "\x00" in raw:
        raise ValueError(f"invalid_{field}: {raw!r}")
    if raw.startswith("/") or PureWindowsPath(raw).is_absolute():
        raise ValueError(f"unsafe_{field}: {raw!r}")
    parts: list[str] = []
    for part in raw.split("/"):
        if part == "":
            raise ValueError(f"noncanonical_{field}: {raw!r}")
        if part == ".":
            continue
        if part == "..":
            if parts and parts[-1] != "..":
                parts.pop()
            else:
                parts.append(part)
        else:
            parts.append(part)
    if not parts:
        raise ValueError(f"invalid_{field}: {raw!r}")
    return PurePosixPath(*parts)


def _resolve_resource(
    package: Path,
    declaring_file: Path,
    raw: str,
    *,
    field: str,
) -> tuple[Path | None, str | None]:
    try:
        relative = _safe_relative_path(raw.strip(), field=field)
    except (AttributeError, ValueError) as exc:
        return None, str(exc)
    if GENERATION_CONFIG_RE.fullmatch(relative.name):
        return None, f"generation_config_not_readable_as_resource: {relative.as_posix()}"
    candidate = declaring_file.parent.joinpath(*relative.parts)
    if candidate.is_symlink():
        return None, f"symlink_{field}: {relative.as_posix()}"
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(package)
    except FileNotFoundError:
        return None, f"missing_{field}: {relative.as_posix()}"
    except (OSError, ValueError):
        return None, f"escaping_{field}: {relative.as_posix()}"
    if not resolved.is_file():
        return None, f"not_file_{field}: {relative.as_posix()}"
    try:
        if resolved.stat().st_size <= 0:
            return None, f"empty_{field}: {relative.as_posix()}"
    except OSError as exc:
        return None, f"unreadable_{field}: {type(exc).__name__}: {exc}"
    return resolved, None


def _nested_resource_specs(path: Path) -> tuple[list[tuple[str, str]], str | None]:
    try:
        if path.suffix.lower() == ".obj":
            specs: list[tuple[str, str]] = []
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if tokens and tokens[0].lower() == "mtllib":
                    specs.extend(("obj_mtl", token) for token in tokens[1:])
            return specs, None
        if path.suffix.lower() == ".mtl":
            specs = []
            texture_keys = {
                "map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump",
                "map_bump", "disp", "decal", "norm",
            }
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if len(tokens) >= 2 and tokens[0].lower() in texture_keys:
                    specs.append(("mtl_resource", tokens[-1]))
            return specs, None
        if path.suffix.lower() == ".gltf":
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                return [], "gltf_root_not_object"
            specs = []
            for section in ("buffers", "images"):
                entries = payload.get(section, [])
                if not isinstance(entries, list):
                    return [], f"gltf_{section}_not_list"
                for entry in entries:
                    uri = entry.get("uri") if isinstance(entry, dict) else None
                    if isinstance(uri, str) and not uri.startswith("data:"):
                        specs.append((f"gltf_{section[:-1]}", uri))
            return specs, None
        if path.suffix.lower() == ".dae":
            root = ET.parse(path).getroot()
            specs = []
            for image in _descendants(root, "image"):
                for node in image.iter():
                    if (
                        _local_tag(node) == "init_from"
                        and node.text
                        and node.text.strip()
                        and not node.text.strip().startswith("#")
                    ):
                        specs.append(("dae_image", node.text.strip()))
            return specs, None
    except Exception as exc:  # noqa: BLE001
        return [], f"nested_resource_parse_failed: {type(exc).__name__}: {exc}"
    return [], None


def _resource_closure(root: ET.Element, package: Path, urdf: Path) -> dict[str, Any]:
    queue: deque[tuple[str, str, Path]] = deque()
    queue.extend(
        ("urdf_mesh", node.attrib.get("filename", ""), urdf)
        for node in _descendants(root, "mesh")
    )
    queue.extend(
        ("urdf_texture", node.attrib.get("filename", ""), urdf)
        for node in _descendants(root, "texture")
    )
    relative_urdf = urdf.relative_to(package).as_posix()
    records: dict[str, dict[str, Any]] = {
        relative_urdf: {"path": relative_urdf, "sha256": sha256_file(urdf)}
    }
    issues: list[str] = []
    visited_edges: set[tuple[str, str, str]] = set()
    expanded: set[Path] = set()
    while queue:
        kind, raw, declaring = queue.popleft()
        edge = (kind, raw, declaring.relative_to(package).as_posix())
        if edge in visited_edges:
            continue
        visited_edges.add(edge)
        resolved, issue = _resolve_resource(package, declaring, raw, field=kind)
        if issue is not None:
            issues.append(f"{edge[2]}:{kind}: {issue}")
            continue
        assert resolved is not None
        relative = resolved.relative_to(package).as_posix()
        records.setdefault(relative, {"path": relative, "sha256": sha256_file(resolved)})
        if resolved in expanded:
            continue
        expanded.add(resolved)
        nested, nested_issue = _nested_resource_specs(resolved)
        if nested_issue:
            issues.append(f"{relative}: {nested_issue}")
        else:
            queue.extend(
                (nested_kind, nested_raw, resolved)
                for nested_kind, nested_raw in nested
            )
    ordered = [records[key] for key in sorted(records)]
    complete = not issues
    return {
        "status": "COMPLETE" if complete else "PARTIAL",
        "complete": complete,
        "file_count": len(ordered),
        "sha256": canonical_sha256(ordered) if complete else None,
        "files": ordered,
        "issues": issues,
    }


def _metadata_only(binding_row: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "path": binding_row["path"],
        "size_bytes": binding_row["bytes"],
        "sha256": binding_row["sha256"],
    }


def audit_package(package: Path) -> dict[str, Any]:
    root_lexical = Path(os.path.abspath(str(package)))
    binding = package_binding(root_lexical)
    root_path = root_lexical.resolve(strict=True)
    urdf_path = (root_path / "generated.urdf").resolve(strict=True)
    urdf_path.relative_to(root_path)
    if not urdf_path.is_file():
        raise ValueError("generated.urdf is not a regular file")
    robot = ET.parse(urdf_path).getroot()
    if _local_tag(robot) != "robot":
        raise ValueError("URDF root is not robot")
    links = _children(robot, "link")
    names = [link.attrib.get("name", "").strip() for link in links]
    topology_issues: list[str] = []
    if any(not name for name in names):
        topology_issues.append("unnamed_link")
    if len(names) != len(set(names)):
        topology_issues.append("duplicate_link_name")
    adjacent: set[tuple[str, str]] = set()
    if topology_issues:
        eligible: set[tuple[str, str]] = set()
    else:
        for joint in _children(robot, "joint"):
            parents = _children(joint, "parent")
            children = _children(joint, "child")
            if len(parents) != 1 or len(children) != 1:
                topology_issues.append("joint_parent_child_cardinality")
                continue
            parent = parents[0].attrib.get("link", "").strip()
            child = children[0].attrib.get("link", "").strip()
            if parent not in names or child not in names or parent == child:
                topology_issues.append("joint_parent_child_invalid")
                continue
            adjacent.add(tuple(sorted((parent, child))))
        collision_names = [
            link.attrib.get("name", "").strip()
            for link in links
            if _children(link, "collision")
        ]
        eligible = {
            tuple(sorted((collision_names[left], collision_names[right])))
            for left in range(len(collision_names))
            for right in range(left + 1, len(collision_names))
        } - adjacent

    file_names = [str(row["path"]) for row in binding["files"]]
    receipt_candidates = [
        name
        for name in file_names
        if Path(name).suffix.lower() == ".json"
        and RECEIPT_NAME_RE.search(Path(name).name.lower())
    ]
    rebuild_candidates = [
        name for name in file_names if Path(name).name.lower() in REBUILD_RECIPE_NAMES
    ]
    allowance_candidates = [
        name
        for name in file_names
        if Path(name).suffix.lower() == ".json"
        and ALLOWANCE_NAME_RE.search(Path(name).name.lower())
    ]
    generation_configs = [
        name for name in file_names if GENERATION_CONFIG_RE.fullmatch(Path(name).name)
    ]
    binding_rows = [row for row in binding["files"] if isinstance(row, Mapping)]
    generation_metadata = [
        _metadata_only(row)
        for row in binding_rows
        if GENERATION_CONFIG_RE.fullmatch(Path(str(row["path"])).name)
    ]
    supporting_metadata = [
        _metadata_only(row)
        for row in binding_rows
        if Path(str(row["path"])).name.lower() in {"workflow.json", "export.js"}
    ]
    resource_closure = _resource_closure(robot, root_path, urdf_path)
    allowance_complete = not topology_issues
    s1_evidence = {
        "receipt": {
            "candidate_count": 0,
            "valid_mechanical_receipt_count": 0,
            "receipt_bound_asset": 0,
            "records": [],
            "issues": [],
        },
        "receipt_replay": {
            "eligible_receipt_count": 0,
            "attempted": 0,
            "passed": False,
            "status": "NO_VALID_RECEIPT",
        },
        "rebuild": {
            "status": "N/E",
            "eligible_asset": 0,
            "candidate_recipe_count": 0,
            "valid_recipe_count": 0,
            "recipes": [],
            "generation_config_metadata_only": generation_metadata,
            "supporting_artifacts_metadata_only": supporting_metadata,
            "privacy_note": (
                "generation_config.* was hashed only; content was not parsed or returned"
            ),
        },
        "allowance": {
            "status": "COMPLETE" if allowance_complete else "NOT_EVALUABLE",
            "candidate_file_count": 0,
            "valid_file_count": 0,
            "registered_excluded_pair_count": 0 if allowance_complete else None,
            "eligible_nonadjacent_pair_count": (
                len(eligible) if allowance_complete else None
            ),
            "records": [],
            "issues": list(topology_issues),
        },
    }
    return {
        "package_binding": binding,
        "urdf_sha256": sha256_file(urdf_path),
        "eligible_nonadjacent_pair_count": len(eligible),
        "eligible_pairs_sha256": canonical_sha256(
            [list(pair) for pair in sorted(eligible)]
        ),
        "receipt_candidate_count": len(receipt_candidates),
        "rebuild_recipe_candidate_count": len(rebuild_candidates),
        "allowance_candidate_count": len(allowance_candidates),
        "topology_issues": topology_issues,
        "resource_closure": resource_closure,
        "s1_evidence": s1_evidence,
        "generation_config_count": len(generation_configs),
        "generation_configs": generation_configs,
        "receipt_candidates": receipt_candidates,
        "rebuild_recipe_candidates": rebuild_candidates,
        "allowance_candidates": allowance_candidates,
    }


def _ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    rate = passed / denominator if denominator else None
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": rate,
        "percentage": None if rate is None else rate * 100.0,
    }


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def _joint_interval(row: Mapping[str, Any]) -> tuple[float, float]:
    if row.get("type") == "continuous":
        return -math.pi, math.pi
    lower = row.get("lower")
    upper = row.get("upper")
    if (
        isinstance(lower, bool)
        or isinstance(upper, bool)
        or not isinstance(lower, (int, float))
        or not isinstance(upper, (int, float))
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has no finite range")
    lower_value = float(lower)
    upper_value = float(upper)
    if (
        not math.isfinite(lower_value)
        or not math.isfinite(upper_value)
        or upper_value - lower_value <= ZERO_WIDTH_TOLERANCE
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid range")
    return lower_value, upper_value


def expected_state_identity_rows(item: Mapping[str, Any]) -> list[dict[str, Any]]:
    joints_raw = item.get("joint_specs")
    if not isinstance(joints_raw, list) or any(
        not isinstance(row, Mapping) for row in joints_raw
    ):
        raise ValueError("Table 4 joint_specs are invalid")
    joints = [dict(row) for row in joints_raw]
    if canonical_sha256(joints) != item.get("joint_specs_sha256"):
        raise ValueError("Table 4 joint_specs hash mismatch")
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = _nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF count"
    )
    names = [row.get("name") for row in joints]
    if (
        len(joints) != movable
        or sum(row.get("range_evaluable") is True for row in joints) != range_evaluable
        or len(set(names)) != len(names)
        or any(not isinstance(name, str) or not name for name in names)
    ):
        raise ValueError("Table 4 joint specification closure mismatch")
    expected_denominators = {
        "rest_state_expected": 1,
        "single_state_expected": SINGLE_JOINT_SAMPLES * movable,
        "sobol_state_expected": SOBOL_SAMPLES if movable > 0 else 0,
    }
    for field, expected in expected_denominators.items():
        if item.get(field) != expected:
            raise ValueError(f"Table 4 {field} protocol mismatch")

    rest_values = [0.0] * movable
    identities = [{
        "phase": "rest",
        "joint_name": None,
        "sample_index": 0,
        "joint_values_sha256": canonical_sha256(rest_values),
    }]
    for position, joint in enumerate(joints):
        if joint.get("range_evaluable") is not True:
            continue
        lower, upper = _joint_interval(joint)
        for sample_index in range(SINGLE_JOINT_SAMPLES):
            values = list(rest_values)
            values[position] = lower + sample_index * (upper - lower) / (
                SINGLE_JOINT_SAMPLES - 1
            )
            identities.append({
                "phase": "single_joint_sweep",
                "joint_name": str(joint["name"]),
                "sample_index": sample_index,
                "joint_values_sha256": canonical_sha256(values),
            })
    if joints and range_evaluable == movable:
        from scipy.stats import qmc

        intervals = [_joint_interval(joint) for joint in joints]
        unit = qmc.Sobol(
            d=movable,
            scramble=True,
            seed=SOBOL_SEED,
        ).random_base2(m=6)
        if len(unit) != SOBOL_SAMPLES:
            raise ValueError("Sobol generator returned an unexpected state count")
        for sample_index, vector in enumerate(unit):
            values = [
                float(lower + scalar * (upper - lower))
                for scalar, (lower, upper) in zip(vector, intervals)
            ]
            identities.append({
                "phase": "multi_joint_sobol",
                "joint_name": None,
                "sample_index": sample_index,
                "joint_values_sha256": canonical_sha256(values),
            })
    return identities


def recompute_metrics(
    records: list[Mapping[str, Any]],
    *,
    intended_assets: int,
) -> dict[str, Any]:
    if intended_assets < 0 or len(records) != intended_assets:
        raise ValueError("records do not cover the intended denominator")
    receipt_bound = 0
    receipt_replayed = 0
    rebuild_eligible = 0
    rebuild_matched = 0
    rebuild_complete = True
    registered_pairs = 0
    eligible_pairs = 0
    allowance_complete = True
    allowance_measured_assets = 0
    strict_passed = 0
    registered_passed = 0
    registered_outcomes_complete = True

    for record in records:
        binding = record.get("binding")
        result_eligible = bool(
            record.get("status") == "completed"
            and isinstance(binding, Mapping)
            and binding.get("verified") is True
        )
        evidence = (
            record.get("s1_evidence")
            if isinstance(record.get("s1_evidence"), Mapping)
            else {}
        )
        receipt = evidence.get("receipt") if isinstance(evidence.get("receipt"), Mapping) else {}
        replay = (
            evidence.get("receipt_replay")
            if isinstance(evidence.get("receipt_replay"), Mapping)
            else {}
        )
        rebuild = evidence.get("rebuild") if isinstance(evidence.get("rebuild"), Mapping) else {}
        allowance = (
            evidence.get("allowance")
            if isinstance(evidence.get("allowance"), Mapping)
            else {}
        )
        receipt_bound += int(result_eligible and bool(receipt.get("receipt_bound_asset")))
        receipt_replayed += int(result_eligible and bool(replay.get("passed")))
        eligible = int(result_eligible and bool(rebuild.get("eligible_asset")))
        rebuild_eligible += eligible
        rebuild_matched += int(bool(record.get("deterministic_rebuild_match"))) if eligible else 0
        if eligible and record.get("rebuild_replay_status") != "COMPLETE":
            rebuild_complete = False
        if not result_eligible or allowance.get("status") != "COMPLETE":
            allowance_complete = False
        else:
            allowance_measured_assets += 1
            registered_pairs += _nonnegative_int(
                allowance.get("registered_excluded_pair_count"),
                "registered allowance pair count",
            )
            eligible_pairs += _nonnegative_int(
                allowance.get("eligible_nonadjacent_pair_count"),
                "eligible pair count",
            )
        strict_passed += int(
            result_eligible and bool(record.get("strict_pass_no_method_allowance"))
        )
        registered = record.get("registered_allowance_strict_pass")
        if result_eligible and isinstance(registered, bool):
            registered_passed += int(registered)
        else:
            registered_outcomes_complete = False

    rebuild_metric = {
        "status": (
            "N/E"
            if rebuild_eligible == 0
            else ("COMPLETE" if rebuild_complete else "NOT_EVALUABLE")
        ),
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rebuild_matched / rebuild_eligible,
        "percentage": (
            None
            if rebuild_eligible == 0
            else 100.0 * rebuild_matched / rebuild_eligible
        ),
        "eligible_assets": rebuild_eligible,
        "asset_denominator": intended_assets,
    }
    allowance_rate = (
        registered_pairs / eligible_pairs
        if allowance_complete and eligible_pairs > 0
        else None
    )
    allowance_metric = {
        "status": (
            "PARTIAL"
            if not allowance_complete
            else ("N/E" if eligible_pairs == 0 else "COMPLETE")
        ),
        "registered_pairs": registered_pairs,
        "eligible_pairs": eligible_pairs,
        "rate": allowance_rate,
        "percentage": None if allowance_rate is None else allowance_rate * 100.0,
        "measured_assets": allowance_measured_assets,
        "intended_assets": intended_assets,
    }
    strict_metric = _ratio(strict_passed, intended_assets)
    if registered_pairs == 0 and allowance_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 0.0,
            "registered_passed": strict_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    elif registered_outcomes_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 100.0 * (registered_passed - strict_passed) / intended_assets,
            "registered_passed": registered_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    else:
        gain_metric = {
            "status": "NOT_EVALUABLE",
            "value": None,
            "registered_passed": None,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
            "reason": "registered allowance exists but no frozen pair-specific replay is available",
        }
    return {
        "receipt_bound_assets": _ratio(receipt_bound, intended_assets),
        "receipt_replay_pass": _ratio(receipt_replayed, intended_assets),
        "deterministic_rebuild_match": rebuild_metric,
        "allowance_density": allowance_metric,
        "strict_pass_no_method_allowance": strict_metric,
        "registered_allowance_gain_pp": gain_metric,
    }


def registered_allowance_outcome(
    strict_pass: bool,
    allowance: Mapping[str, Any],
) -> bool | None:
    registered_pairs = allowance.get("registered_excluded_pair_count")
    no_named_candidate = allowance.get("candidate_file_count") == 0
    if registered_pairs == 0 or (registered_pairs is None and no_named_candidate):
        return strict_pass
    return None


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                row = json.loads(stripped, parse_constant=_reject_json_constant)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"JSONL row is not an object at {path}:{line_number}")
            rows.append(row)
    return rows


def _require_identity(
    expected: Mapping[str, Any],
    observed: Mapping[str, Any],
    *,
    label: str,
) -> None:
    for field in TABLE4_IDENTITY_FIELDS:
        if observed.get(field) != expected.get(field):
            raise ValueError(f"{label} {field} mismatch")


def _validate_state_observation(state: Mapping[str, Any]) -> None:
    observations: dict[str, tuple[int, int, float]] = {}
    for prefix in ("all_pair", "non_adjacent"):
        contacts = _nonnegative_int(state.get(f"{prefix}_contact_count"), "contact count")
        illegal = _nonnegative_int(
            state.get(f"{prefix}_illegal_penetration_count"),
            "illegal penetration count",
        )
        maximum_raw = state.get(f"{prefix}_max_penetration_m")
        if (
            isinstance(maximum_raw, bool)
            or not isinstance(maximum_raw, (int, float))
            or not math.isfinite(float(maximum_raw))
            or float(maximum_raw) < 0.0
            or illegal > contacts
        ):
            raise ValueError("collision observation counters are invalid")
        maximum = float(maximum_raw)
        if (illegal > 0) != (maximum > PENETRATION_THRESHOLD_M):
            raise ValueError("penetration threshold evidence is inconsistent")
        if maximum > 0.0 and contacts == 0:
            raise ValueError("penetration exists without a contact")
        observations[prefix] = (contacts, illegal, maximum)
    all_pair = observations["all_pair"]
    non_adjacent = observations["non_adjacent"]
    if (
        non_adjacent[0] > all_pair[0]
        or non_adjacent[1] > all_pair[1]
        or non_adjacent[2] > all_pair[2] + 1e-15
    ):
        raise ValueError("non-adjacent observation is not an all-pair subset")
    expected_metric = observations[
        "all_pair" if state.get("phase") == "rest" else "non_adjacent"
    ][2]
    metric = state.get("metric_max_penetration_m")
    if (
        isinstance(metric, bool)
        or not isinstance(metric, (int, float))
        or not math.isclose(float(metric), expected_metric, rel_tol=0.0, abs_tol=1e-15)
    ):
        raise ValueError("state metric penetration policy mismatch")
    readback = state.get("reset_readback_max_abs_error")
    if (
        isinstance(readback, bool)
        or not isinstance(readback, (int, float))
        or not math.isfinite(float(readback))
        or not 0.0 <= float(readback) <= RESET_TOLERANCE
    ):
        raise ValueError("reset readback evidence is invalid")


def reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    asset_key = str(item["asset_key"])
    _require_identity(item, asset, label=f"asset {asset_key}")
    state_hash = canonical_sha256(states)
    if state_hash != asset.get("state_records_sha256"):
        raise ValueError(f"state hash mismatch: {asset_key}")
    expected_state_identities = expected_state_identity_rows(item)
    observed_state_identities = [
        {
            "phase": state.get("phase"),
            "joint_name": state.get("joint_name"),
            "sample_index": state.get("sample_index"),
            "joint_values_sha256": state.get("joint_values_sha256"),
        }
        for state in states
    ]
    identity_keys = [
        (row["phase"], row["joint_name"], row["sample_index"])
        for row in observed_state_identities
    ]
    if len(identity_keys) != len(set(identity_keys)):
        raise ValueError(f"duplicate state identity: {asset_key}")
    if observed_state_identities != expected_state_identities[: len(states)]:
        raise ValueError(
            f"state identity does not match frozen sampling sequence: {asset_key}"
        )
    expected = {
        "rest": _nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": _nonnegative_int(
            item.get("single_state_expected"), "single expected"
        ),
        "multi_joint_sobol": _nonnegative_int(
            item.get("sobol_state_expected"), "sobol expected"
        ),
    }
    by_phase: dict[str, list[dict[str, Any]]] = {key: [] for key in expected}
    for state in states:
        _require_identity(item, state, label=f"state {asset_key}")
        phase = state.get("phase")
        if phase not in by_phase:
            raise ValueError(f"unknown state phase: {phase!r}")
        _validate_state_observation(state)
        by_phase[str(phase)].append(state)
    executed = {phase: len(rows) for phase, rows in by_phase.items()}
    free = {
        phase: sum(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows)
        for phase, rows in by_phase.items()
    }
    load_success = asset.get("load_success") is True
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF")
    range_evaluable = _nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF"
    )
    rest_all_pair = bool(
        load_success
        and executed["rest"] == expected["rest"]
        and all(row["all_pair_illegal_penetration_count"] == 0 for row in by_phase["rest"])
    )
    rest = bool(
        load_success
        and executed["rest"] == expected["rest"]
        and free["rest"] == expected["rest"]
    )
    single = bool(
        load_success
        and executed["single_joint_sweep"] == expected["single_joint_sweep"]
        and free["single_joint_sweep"] == expected["single_joint_sweep"]
    )
    sobol = bool(
        movable > 0
        and range_evaluable == movable
        and executed["multi_joint_sobol"] == expected["multi_joint_sobol"]
        and free["multi_joint_sobol"] == expected["multi_joint_sobol"]
    )
    complete = bool(
        load_success
        and range_evaluable == movable
        and sum(executed.values()) == sum(expected.values())
    )
    strict = bool(complete and rest and single and sobol)
    single_by_joint: dict[str, list[dict[str, Any]]] = {}
    for state in by_phase["single_joint_sweep"]:
        name = state.get("joint_name")
        if not isinstance(name, str) or not name:
            raise ValueError("single-joint state has invalid joint name")
        single_by_joint.setdefault(name, []).append(state)
    joint_passed = sum(
        len(rows) == SINGLE_JOINT_SAMPLES
        and all(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows)
        for rows in single_by_joint.values()
    )
    fields = {
        "rest_state_executed": executed["rest"],
        "rest_non_adjacent_free": free["rest"],
        "rest_all_pair_cf": rest_all_pair,
        "rest_non_adjacent_cf": rest,
        "single_state_executed": executed["single_joint_sweep"],
        "single_non_adjacent_free": free["single_joint_sweep"],
        "joint_single_sweep_cf_passed": joint_passed,
        "single_joint_sweep_cf": single,
        "sobol_state_executed": executed["multi_joint_sobol"],
        "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol,
        "measurement_complete": complete,
        "strict_collision_pass": strict,
    }
    for field, value in fields.items():
        if asset.get(field) != value:
            raise ValueError(f"Table 4 asset {field} mismatch: {asset_key}")
    return {
        "strict_collision_pass": strict,
        "measurement_complete": complete,
        "state_record_count": len(states),
        "state_records_sha256": state_hash,
        "table4_asset_record_sha256": canonical_sha256(asset),
    }


def load_authority(n: int) -> list[dict[str, Any]]:
    expected_hashes = (
        (TABLE3_MANIFEST, EXPECTED_TABLE3_MANIFEST_SHA256),
        (TABLE3_RECORDS, EXPECTED_TABLE3_RECORDS_SHA256),
        (TABLE4_MANIFEST, EXPECTED_TABLE4_MANIFEST_SHA256),
        (TABLE4_ASSET_RECORDS, EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
        (TABLE4_STATE_RECORDS, EXPECTED_TABLE4_STATE_RECORDS_SHA256),
        (TABLE4_VERIFICATION, EXPECTED_TABLE4_VERIFICATION_SHA256),
    )
    for path, expected in expected_hashes:
        if sha256_file(path) != expected:
            raise ValueError(f"frozen authority hash mismatch: {path}")
    table3 = json.loads(TABLE3_MANIFEST.read_text(encoding="utf-8"))
    table3_content = {key: value for key, value in table3.items() if key != "manifest_content_sha256"}
    if (
        table3.get("manifest_content_sha256") != EXPECTED_TABLE3_CONTENT_SHA256
        or canonical_sha256(table3_content) != EXPECTED_TABLE3_CONTENT_SHA256
    ):
        raise ValueError("Table 3 content hash mismatch")
    manifest_rows = table3.get("records")
    jsonl_rows = load_jsonl(TABLE3_RECORDS)
    if not isinstance(manifest_rows, list) or len(manifest_rows) != EXPECTED_N_EVAL:
        raise ValueError("Table 3 manifest row count mismatch")
    if len(jsonl_rows) != EXPECTED_N_EVAL:
        raise ValueError("Table 3 JSONL row count mismatch")
    jsonl_by_rank = {row.get("selection_rank"): row for row in jsonl_rows}
    if set(jsonl_by_rank) != set(range(1, EXPECTED_N_EVAL + 1)):
        raise ValueError("Table 3 JSONL ranks are not exactly 1..800")
    ordered_keys = [str(row["asset_key"]) for row in manifest_rows]
    if canonical_sha256(ordered_keys) != EXPECTED_ORDERED_KEYS_SHA256:
        raise ValueError("Table 3 ordered key hash mismatch")

    table4_manifest = json.loads(TABLE4_MANIFEST.read_text(encoding="utf-8"))
    table4_content = {
        key: value
        for key, value in table4_manifest.items()
        if key != "manifest_content_sha256"
    }
    items = table4_manifest.get("items")
    assets = json.loads(TABLE4_ASSET_RECORDS.read_text(encoding="utf-8"))
    verification = json.loads(TABLE4_VERIFICATION.read_text(encoding="utf-8"))
    if (
        table4_manifest.get("protocol_id") != EXPECTED_TABLE4_PROTOCOL_ID
        or table4_manifest.get("manifest_content_sha256") != EXPECTED_TABLE4_CONTENT_SHA256
        or canonical_sha256(table4_content) != EXPECTED_TABLE4_CONTENT_SHA256
        or not isinstance(items, list)
        or len(items) != EXPECTED_N_EVAL
        or not isinstance(assets, list)
        or len(assets) != EXPECTED_N_EVAL
    ):
        raise ValueError("Table 4 manifest or asset identity mismatch")
    artifacts = verification.get("artifact_sha256", {})
    if (
        verification.get("status") != "PASS"
        or artifacts.get("frozen_manifest.json") != EXPECTED_TABLE4_MANIFEST_SHA256
        or artifacts.get("asset_records.json") != EXPECTED_TABLE4_ASSET_RECORDS_SHA256
        or artifacts.get("state_records.jsonl") != EXPECTED_TABLE4_STATE_RECORDS_SHA256
    ):
        raise ValueError("Table 4 verification receipt mismatch")
    states_by_id: dict[str, list[dict[str, Any]]] = {}
    for state in load_jsonl(TABLE4_STATE_RECORDS):
        dataset_id = state.get("dataset_id")
        if not isinstance(dataset_id, str):
            raise ValueError("Table 4 state dataset_id is invalid")
        states_by_id.setdefault(dataset_id, []).append(state)

    authority: list[dict[str, Any]] = []
    cross_fields = (
        "asset_key",
        "category",
        "object_release_id",
        "rel_path",
        "selection_hash",
        "selection_rank",
        "tier",
        "urdf_sha256",
    )
    for index in range(n):
        manifest_row = manifest_rows[index]
        jsonl_row = jsonl_by_rank[index + 1]
        if any(manifest_row.get(field) != jsonl_row.get(field) for field in cross_fields):
            raise ValueError(f"Table 3 identity mismatch at rank {index + 1}")
        item = items[index]
        asset = assets[index]
        expected_source = {
            "order": index,
            "asset_key": manifest_row.get("asset_key"),
            "selection_rank": manifest_row.get("selection_rank"),
            "selection_hash": manifest_row.get("selection_hash"),
            "tier": manifest_row.get("tier"),
            "rel_path": manifest_row.get("rel_path"),
            "object_release_id": manifest_row.get("object_release_id"),
            "model_urdf_sha256": manifest_row.get("urdf_sha256"),
            "package_relpath": f"released_outputs/{manifest_row.get('rel_path')}",
            "source_record_sha256": canonical_sha256(jsonl_row),
            "source_manifest_record_sha256": canonical_sha256(manifest_row),
        }
        for field, expected in expected_source.items():
            if item.get(field) != expected or asset.get(field) != expected:
                raise ValueError(f"Table 3/Table 4 {field} mismatch at order {index}")
        table4_result = reaggregate_table4_asset(
            item,
            asset,
            states_by_id.get(str(item["dataset_id"]), []),
        )
        package, _ = resolve_release_package(
            RELEASE_ROOT,
            str(manifest_row["rel_path"]),
        )
        package_audit = audit_package(package)
        if package_audit["package_binding"] != item.get("package_binding"):
            raise ValueError(f"live package binding mismatch at order {index}")
        if any(
            package_audit[field] != 0
            for field in (
                "receipt_candidate_count",
                "rebuild_recipe_candidate_count",
                "allowance_candidate_count",
            )
        ):
            raise ValueError(f"unexpected named S1 evidence candidate at order {index}")
        authority.append(
            {
                "source": manifest_row,
                "jsonl": jsonl_row,
                "item": item,
                "table4": table4_result,
                "package": package,
                "package_audit": package_audit,
            }
        )
    return authority


def _expected_output_record(
    authority: Mapping[str, Any],
    index: int,
) -> dict[str, Any]:
    source = authority["source"]
    item = authority["item"]
    table4 = authority["table4"]
    audit = authority["package_audit"]
    evidence_raw = audit.get("s1_evidence")
    if not isinstance(evidence_raw, Mapping):
        raise ValueError("independent S1 evidence is missing")
    evidence = dict(evidence_raw)
    rebuild = evidence.get("rebuild")
    rebuild = rebuild if isinstance(rebuild, Mapping) else {}
    allowance = evidence.get("allowance")
    allowance = allowance if isinstance(allowance, Mapping) else {}
    registered_pairs = (
        allowance.get("registered_excluded_pair_count")
        if allowance.get("status") == "COMPLETE"
        else None
    )
    strict_pass = table4.get("strict_collision_pass") is True
    identity = {
        "selection_index": index,
        "asset_key": source.get("asset_key"),
        "selection_rank": source.get("selection_rank"),
        "selection_hash": source.get("selection_hash"),
        "tier": source.get("tier"),
        "rel_path": source.get("rel_path"),
        "object_release_id": source.get("object_release_id"),
        "category": source.get("category"),
        "package": str(authority["package"]),
        "primary_urdf_relative_path": "generated.urdf",
        "model_urdf_sha256": audit["urdf_sha256"],
        "package_content_manifest_sha256": audit["package_binding"][
            "content_manifest_sha256"
        ],
        "table4_input_identity_sha256": item.get("input_identity_sha256"),
    }
    return {
        **identity,
        "s1_input_identity_sha256": canonical_sha256(identity),
        "status": "completed",
        "binding": {"verified": True, "issues": []},
        "resource_closure": audit.get("resource_closure"),
        "s1_evidence": evidence,
        "deterministic_rebuild_match": None,
        "rebuild_replay_status": (
            "ELIGIBLE_NOT_RUN" if bool(rebuild.get("eligible_asset")) else "N/E"
        ),
        "strict_pass_no_method_allowance": strict_pass,
        "registered_allowance_strict_pass": registered_allowance_outcome(
            strict_pass,
            allowance,
        ),
        "table4_measurement_complete": bool(table4.get("measurement_complete")),
        "table4_state_record_count": table4.get("state_record_count"),
        "table4_state_records_sha256": table4.get("state_records_sha256"),
        "table4_asset_record_sha256": table4.get("table4_asset_record_sha256"),
    }


def _atomic_projection_matches(
    record: Mapping[str, Any],
    authority: Mapping[str, Any],
    index: int,
) -> tuple[bool, bool]:
    expected = _expected_output_record(authority, index)
    table4_fields = {
        "selection_index",
        "asset_key",
        "selection_rank",
        "selection_hash",
        "tier",
        "rel_path",
        "object_release_id",
        "category",
        "package",
        "primary_urdf_relative_path",
        "model_urdf_sha256",
        "package_content_manifest_sha256",
        "table4_input_identity_sha256",
        "s1_input_identity_sha256",
        "binding",
        "strict_pass_no_method_allowance",
        "registered_allowance_strict_pass",
        "table4_measurement_complete",
        "table4_state_record_count",
        "table4_state_records_sha256",
        "table4_asset_record_sha256",
    }
    table4_ok = all(record.get(key) == expected.get(key) for key in table4_fields)
    static_fields = set(expected) - table4_fields
    static_ok = bool(
        set(record) == set(expected)
        and all(record.get(key) == expected.get(key) for key in static_fields)
    )
    return table4_ok, static_ok


def _expected_frozen_config(
    *,
    n_eval: int,
    formal: bool,
    workers: int,
    created_at: str,
    protocol_snapshot_sha256: str,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": "FORMAL" if formal else "SMOKE",
        "created_at": created_at,
        "n_eval": n_eval,
        "workers": workers,
        "cohort": {
            "source_manifest": str(TABLE3_MANIFEST),
            "source_manifest_file_sha256": EXPECTED_TABLE3_MANIFEST_SHA256,
            "source_manifest_content_sha256": EXPECTED_TABLE3_CONTENT_SHA256,
            "source_asset_records": str(TABLE3_RECORDS),
            "source_asset_records_file_sha256": EXPECTED_TABLE3_RECORDS_SHA256,
            "ordered_asset_keys_sha256": EXPECTED_ORDERED_KEYS_SHA256,
            "selection": "first N records in frozen selection_rank 1..800 order",
        },
        "table4": {
            "protocol_id": EXPECTED_TABLE4_PROTOCOL_ID,
            "manifest_sha256": EXPECTED_TABLE4_MANIFEST_SHA256,
            "manifest_content_sha256": EXPECTED_TABLE4_CONTENT_SHA256,
            "asset_records_sha256": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
            "state_records_sha256": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
            "verification_sha256": EXPECTED_TABLE4_VERIFICATION_SHA256,
        },
        "pair_policy": {
            "eligible_pairs": "distinct source-URDF links with collision geometry",
            "shared_topology_exclusion": "exclude_direct_parent_child",
            "surface_contact_allowed": True,
            "illegal_penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "registered_method_specific_allowance_registry": [],
        },
        "privacy": {
            "generation_config": "hash metadata only; contents are not parsed or emitted",
            "released_code_execution": "disabled",
            "network_access": "disabled",
        },
        "code": {
            "runner": str(RUNNER_PATH),
            "runner_sha256": sha256_file(RUNNER_PATH),
            "verifier": str(SCRIPT),
            "verifier_sha256": sha256_file(SCRIPT),
            "static_atoms": str(STATIC_ATOMS_PATH),
            "static_atoms_sha256": sha256_file(STATIC_ATOMS_PATH),
        },
        "protocol_snapshot_sha256": protocol_snapshot_sha256,
    }


def _candidate_counts(records: list[Mapping[str, Any]]) -> dict[str, int]:
    def atom(record: Mapping[str, Any], name: str) -> Mapping[str, Any]:
        evidence = record.get("s1_evidence")
        if not isinstance(evidence, Mapping):
            raise ValueError("record s1_evidence is not an object")
        value = evidence.get(name)
        if not isinstance(value, Mapping):
            raise ValueError(f"record S1 {name} atom is not an object")
        return value

    return {
        "receipt": sum(
            _nonnegative_int(
                atom(record, "receipt").get("candidate_count"),
                "receipt candidate count",
            )
            for record in records
        ),
        "rebuild_recipe": sum(
            _nonnegative_int(
                atom(record, "rebuild").get("candidate_recipe_count"),
                "rebuild candidate count",
            )
            for record in records
        ),
        "allowance": sum(
            _nonnegative_int(
                atom(record, "allowance").get("candidate_file_count"),
                "allowance candidate count",
            )
            for record in records
        ),
    }


def _render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        percentage = metric.get("percentage")
        if percentage is None:
            return f"{metric.get('passed')} / {metric.get('denominator')} (N/E)"
        return (
            f"{metric.get('passed')} / {metric.get('denominator')} "
            f"({float(percentage):.2f}%)"
        )

    rebuild = metrics["deterministic_rebuild_match"]
    rebuild_text = (
        f"N/E ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible)"
        if rebuild["status"] == "N/E"
        else fraction(rebuild)
    )
    allowance = metrics["allowance_density"]
    if allowance["status"] == "PARTIAL":
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} "
            f"(PARTIAL; {allowance['measured_assets']} / "
            f"{allowance['intended_assets']} assets)"
        )
    elif allowance["percentage"] is None:
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} (N/E)"
        )
    else:
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} "
            f"({float(allowance['percentage']):.2f}%)"
        )
    gain = metrics["registered_allowance_gain_pp"]
    gain_text = "N/E" if gain["value"] is None else f"{float(gain['value']):.2f} pp"
    return "\n".join(
        [
            f"# Supplementary Table S1 - {summary['dataset']}",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- Classification: `{summary['classification']}`",
            f"- N_eval: {summary['n_eval']}",
            f"- Status counts: {summary['status_counts']}",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            f"| Deterministic Rebuild Match | {rebuild_text} |",
            f"| Allowance Density | {allowance_text} |",
            "| Strict Pass (No Method-specific Allowance) | "
            f"{fraction(metrics['strict_pass_no_method_allowance'])} |",
            f"| Registered-allowance Gain | {gain_text} |",
            "",
        ]
    )


def _atomic_write_json(path: Path, value: Any) -> None:
    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(
                value,
                handle,
                indent=2,
                sort_keys=True,
                ensure_ascii=True,
                allow_nan=False,
            )
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _verify_run(
    run: Path,
    *,
    formal: bool,
    write_result: bool = True,
) -> dict[str, Any]:
    run_lexical = Path(os.path.abspath(str(run)))
    if run_lexical.is_symlink():
        raise ValueError(f"run directory is a symlink: {run_lexical}")
    run = run_lexical.resolve(strict=True)
    if not run.is_dir():
        raise ValueError(f"run path is not a directory: {run}")
    required = (
        "asset_records.jsonl",
        "environment.json",
        "frozen_config.json",
        "manifest.json",
        "protocol_snapshot.md",
        "summary.json",
        "summary.md",
    )
    checks: dict[str, bool] = {}
    errors: list[str] = []
    allowed_entries = set(required) | {"verification.json"}
    observed_entries = {path.name for path in run.iterdir()}
    checks["required_artifacts"] = all(
        (run / name).is_file() and not (run / name).is_symlink()
        for name in required
    )
    checks["artifact_set"] = bool(
        set(required) <= observed_entries <= allowed_entries
        and all(not path.is_symlink() and path.is_file() for path in run.iterdir())
    )
    if not checks["required_artifacts"]:
        errors.append("required artifacts are missing")
        result = {
            "schema_version": "s1-lam-verification/v1",
            "status": "FAIL",
            "checked_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "checks": checks,
            "errors": errors,
        }
        if write_result:
            _atomic_write_json(run / "verification.json", result)
        return result

    try:
        manifest = json.loads(
            (run / "manifest.json").read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
        summary = json.loads(
            (run / "summary.json").read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
        config = json.loads(
            (run / "frozen_config.json").read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
        environment = json.loads(
            (run / "environment.json").read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
        records = load_jsonl(run / "asset_records.jsonl")
        summary_markdown = (run / "summary.md").read_text(encoding="utf-8")
        if not all(
            isinstance(value, dict)
            for value in (manifest, summary, config, environment)
        ):
            raise ValueError("JSON artifact roots must be objects")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        errors.append(f"output parse failed: {type(exc).__name__}: {exc}")
        manifest, summary, config, environment, records = {}, {}, {}, {}, []
        summary_markdown = ""

    declared_manifest_hash = manifest.get("manifest_content_sha256")
    manifest_body = {
        key: value for key, value in manifest.items() if key != "manifest_content_sha256"
    }
    checks["manifest_self_hash"] = bool(
        declared_manifest_hash and declared_manifest_hash == canonical_sha256(manifest_body)
    )
    artifact_rows = manifest.get("artifacts")
    expected_artifact_names = set(required) - {"manifest.json"}
    checks["manifest_artifact_hashes"] = bool(
        isinstance(artifact_rows, Mapping)
        and set(artifact_rows) == expected_artifact_names
        and all(
            isinstance(artifact_rows.get(name), Mapping)
            and set(artifact_rows[name]) == {"bytes", "sha256"}
            and artifact_rows[name].get("bytes") == (run / name).stat().st_size
            and artifact_rows[name].get("sha256") == sha256_file(run / name)
            for name in expected_artifact_names
        )
    )
    n_eval = summary.get("n_eval")
    n_eval_valid = bool(
        isinstance(n_eval, int)
        and not isinstance(n_eval, bool)
        and 1 <= n_eval <= EXPECTED_N_EVAL
    )
    checks["n_eval_range"] = n_eval_valid
    checks["record_count_and_order"] = bool(
        n_eval_valid
        and len(records) == n_eval
        and [row.get("selection_index") for row in records] == list(range(n_eval))
        and [row.get("selection_rank") for row in records]
        == list(range(1, n_eval + 1))
    )
    expected_classification = "FORMAL" if formal else "SMOKE"
    expected_snapshot_sha256 = (
        EXPECTED_PROTOCOL_SNAPSHOT_SHA256
        if formal
        else sha256_file(PROTOCOL_DOCUMENT)
    )
    checks["verification_mode"] = bool(
        manifest.get("classification") == expected_classification
        and summary.get("classification") == expected_classification
        and config.get("classification") == expected_classification
    )
    summary_keys = {
        "schema_version", "protocol_id", "dataset", "classification", "status",
        "started_at", "completed_at", "wall_seconds", "n_eval",
        "full_frozen_cohort_size", "status_counts", "evidence_candidate_counts",
        "metrics",
    }
    wall_seconds = summary.get("wall_seconds")
    checks["summary_identity"] = bool(
        set(summary) == summary_keys
        and summary.get("schema_version") == SCHEMA_VERSION
        and summary.get("protocol_id") == PROTOCOL_ID
        and summary.get("dataset") == DATASET
        and summary.get("classification") == expected_classification
        and summary.get("status") == "completed"
        and summary.get("full_frozen_cohort_size") == EXPECTED_N_EVAL
        and isinstance(summary.get("started_at"), str)
        and isinstance(summary.get("completed_at"), str)
        and isinstance(wall_seconds, (int, float))
        and not isinstance(wall_seconds, bool)
        and math.isfinite(float(wall_seconds))
        and float(wall_seconds) >= 0.0
    )
    expected_manifest_body = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": expected_classification,
        "status": "completed",
        "created_at": summary.get("completed_at"),
        "n_eval": n_eval,
        "artifacts": artifact_rows,
    }
    checks["manifest_identity"] = manifest_body == expected_manifest_body
    workers = config.get("workers")
    workers_valid = bool(
        isinstance(workers, int)
        and not isinstance(workers, bool)
        and workers > 0
        and (not formal or workers == 4)
    )
    expected_config = (
        _expected_frozen_config(
            n_eval=n_eval,
            formal=formal,
            workers=workers,
            created_at=summary.get("started_at"),
            protocol_snapshot_sha256=expected_snapshot_sha256,
        )
        if n_eval_valid and workers_valid and isinstance(summary.get("started_at"), str)
        else None
    )
    checks["frozen_config"] = bool(expected_config is not None and config == expected_config)
    expected_environment = {
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "workers": workers,
        "thread_environment": {
            name: os.environ.get(name)
            for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
        },
    }
    checks["environment_binding"] = environment == expected_environment
    checks["run_identity"] = bool(
        checks["verification_mode"]
        and checks["summary_identity"]
        and checks["manifest_identity"]
        and checks["frozen_config"]
        and config.get("created_at") == summary.get("started_at")
    )
    code = config.get("code") if isinstance(config.get("code"), Mapping) else {}
    checks["code_identity"] = bool(
        code.get("runner_sha256") == sha256_file(RUNNER_PATH)
        and code.get("verifier_sha256") == sha256_file(SCRIPT)
        and code.get("static_atoms_sha256") == sha256_file(STATIC_ATOMS_PATH)
    )
    checks["protocol_snapshot"] = bool(
        config.get("protocol_snapshot_sha256") == expected_snapshot_sha256
        and sha256_file(run / "protocol_snapshot.md")
        == expected_snapshot_sha256
    )
    try:
        if not n_eval_valid:
            raise ValueError("n_eval is outside the frozen cohort")
        observed_metrics = recompute_metrics(records, intended_assets=n_eval)
        checks["summary_metrics"] = summary.get("metrics") == observed_metrics
        observed_status: dict[str, int] = {}
        for record in records:
            status = str(record.get("status", "missing"))
            observed_status[status] = observed_status.get(status, 0) + 1
        checks["summary_status_counts"] = summary.get("status_counts") == dict(
            sorted(observed_status.items())
        )
        checks["summary_candidate_counts"] = (
            summary.get("evidence_candidate_counts") == _candidate_counts(records)
        )
        checks["summary_markdown"] = summary_markdown == _render_summary(summary)
    except (KeyError, TypeError, ValueError) as exc:
        checks["summary_metrics"] = False
        checks["summary_status_counts"] = False
        checks["summary_candidate_counts"] = False
        checks["summary_markdown"] = False
        errors.append(f"aggregate recomputation failed: {type(exc).__name__}: {exc}")

    try:
        if not n_eval_valid or len(records) != n_eval:
            raise ValueError("record denominator is invalid")
        authority = load_authority(n_eval)
        table4_checks: list[bool] = []
        static_checks: list[bool] = []
        for index, (record, expected) in enumerate(zip(records, authority, strict=True)):
            table4_ok, static_ok = _atomic_projection_matches(record, expected, index)
            table4_checks.append(table4_ok)
            static_checks.append(static_ok)
        checks["atomic_table4_projection"] = all(table4_checks)
        checks["atomic_static_evidence"] = all(static_checks)
    except (IndexError, KeyError, OSError, TypeError, ValueError, ET.ParseError) as exc:
        checks["atomic_table4_projection"] = False
        checks["atomic_static_evidence"] = False
        errors.append(f"authority recomputation failed: {type(exc).__name__}: {exc}")

    pair_policy = (
        config.get("pair_policy")
        if isinstance(config.get("pair_policy"), Mapping)
        else {}
    )
    checks["allowance_registry_frozen_empty"] = (
        pair_policy.get("registered_method_specific_allowance_registry") == []
    )
    if formal:
        metrics = summary.get("metrics") if isinstance(summary.get("metrics"), Mapping) else {}
        strict = (
            metrics.get("strict_pass_no_method_allowance")
            if isinstance(metrics.get("strict_pass_no_method_allowance"), Mapping)
            else {}
        )
        receipt = metrics.get("receipt_bound_assets", {})
        replay = metrics.get("receipt_replay_pass", {})
        rebuild = metrics.get("deterministic_rebuild_match", {})
        allowance = metrics.get("allowance_density", {})
        gain = metrics.get("registered_allowance_gain_pp", {})
        checks["formal_invariants"] = bool(
            n_eval == EXPECTED_N_EVAL
            and summary.get("classification") == "FORMAL"
            and workers == 4
            and summary.get("status_counts") == {"completed": EXPECTED_N_EVAL}
            and receipt == _ratio(0, EXPECTED_N_EVAL)
            and replay == _ratio(0, EXPECTED_N_EVAL)
            and isinstance(rebuild, Mapping)
            and rebuild.get("status") == "N/E"
            and rebuild.get("eligible_assets") == 0
            and rebuild.get("passed") is None
            and isinstance(allowance, Mapping)
            and allowance.get("status") == "PARTIAL"
            and allowance.get("registered_pairs") == 0
            and allowance.get("eligible_pairs")
            == EXPECTED_FORMAL_ALLOWANCE_ELIGIBLE_PAIRS
            and allowance.get("measured_assets")
            == EXPECTED_FORMAL_ALLOWANCE_MEASURED_ASSETS
            and allowance.get("intended_assets") == EXPECTED_N_EVAL
            and allowance.get("rate") is None
            and allowance.get("percentage") is None
            and strict.get("passed") == EXPECTED_FORMAL_STRICT_PASSED
            and isinstance(gain, Mapping)
            and gain.get("status") == "COMPLETE"
            and gain.get("value") == 0.0
            and gain.get("registered_passed") == EXPECTED_FORMAL_STRICT_PASSED
            and gain.get("no_allowance_passed") == EXPECTED_FORMAL_STRICT_PASSED
            and summary.get("evidence_candidate_counts")
            == {"receipt": 0, "rebuild_recipe": 0, "allowance": 0}
        )
    else:
        checks["formal_invariants"] = True

    for name, passed in checks.items():
        if not passed:
            errors.append(f"check failed: {name}")
    result = {
        "schema_version": "s1-lam-verification/v1",
        "protocol_id": PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checked_at": dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z"),
        "formal": formal,
        "n_eval": n_eval,
        "checks": checks,
        "errors": errors,
        "artifact_sha256": {
            name: sha256_file(run / name) for name in required
        },
        "manifest_file_sha256": sha256_file(run / "manifest.json"),
        "manifest_content_sha256": declared_manifest_hash,
    }
    if write_result:
        _atomic_write_json(run / "verification.json", result)
    return result


def verify_run(
    run: Path,
    *,
    formal: bool,
    write_result: bool = True,
) -> dict[str, Any]:
    try:
        return _verify_run(run, formal=formal, write_result=write_result)
    except Exception as exc:  # noqa: BLE001
        result = {
            "schema_version": "s1-lam-verification/v1",
            "protocol_id": PROTOCOL_ID,
            "status": "FAIL",
            "checked_at": dt.datetime.now(dt.timezone.utc)
            .isoformat()
            .replace("+00:00", "Z"),
            "formal": formal,
            "n_eval": None,
            "checks": {"fail_closed_validation": False},
            "errors": [f"validation exception: {type(exc).__name__}: {exc}"],
        }
        run_lexical = Path(os.path.abspath(str(run)))
        if (
            write_result
            and not run_lexical.is_symlink()
            and run_lexical.is_dir()
        ):
            _atomic_write_json(run_lexical / "verification.json", result)
        return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--formal", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = verify_run(args.run, formal=args.formal, write_result=True)
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
