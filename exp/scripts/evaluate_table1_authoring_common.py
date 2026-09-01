#!/usr/bin/env python3
"""Evaluate one normalized Table 1 authoring package against the frozen spec.

The authoring process never receives the evaluator-only spec path.  Method
adapters emit a package manifest after an attempt; this program independently
re-hashes and reparses the saved source, URDF, and referenced meshes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPO_ROOT / "exp/reference/table1_reliability_common_authoring_v1.json"
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table1_reliability_protocol_v1.json"
DEFAULT_HIDDEN_SPECS = REPO_ROOT / "exp/reference/table1_reliability_hidden_specs_v1.json"
DEFAULT_PACKAGE_SCHEMA = REPO_ROOT / "exp/reference/table1_authoring_package_schema_v1.json"
ALLOWED_OUTPUT_ROOT = REPO_ROOT / "exp/runtime/table1_reliability"
HEX64 = re.compile(r"^[0-9a-f]{64}$")


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return payload


def contained(path: Path, root: Path) -> Path:
    resolved = path.resolve()
    resolved.relative_to(root.resolve())
    return resolved


def resolve_artifact(raw: str, package_path: Path, run_root: Path) -> Path:
    candidate = Path(raw)
    if not candidate.is_absolute():
        candidate = package_path.parent / candidate
    return contained(candidate, run_root)


def dump_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def normalized_name(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


def role_rows(spec: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for raw in spec.get("required_parts", []):
        if isinstance(raw, str):
            rows.append({"role_id": raw, "aliases": [raw], "min_count": 1})
            continue
        if not isinstance(raw, dict):
            continue
        role_id = str(raw.get("role_id", "")).strip()
        aliases = [str(item).strip() for item in raw.get("aliases", []) if str(item).strip()]
        if role_id and role_id not in aliases:
            aliases.append(role_id)
        rows.append(
            {
                "role_id": role_id,
                "aliases": aliases,
                "min_count": int(raw.get("min_count", 1)),
            }
        )
    return rows


def alias_match(name: str, aliases: list[str]) -> bool:
    normalized = normalized_name(name)
    return any(
        (token := normalized_name(alias))
        and (normalized == token or normalized.startswith(token + "_") or token in normalized.split("_"))
        for alias in aliases
    )


def parse_vector(raw: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not raw:
        return default
    values = tuple(float(item) for item in raw.split())
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError(f"invalid axis vector: {raw!r}")
    norm = math.sqrt(sum(item * item for item in values))
    if norm <= 1e-12:
        raise ValueError("zero-length joint axis")
    return tuple(item / norm for item in values)  # type: ignore[return-value]


def axis_in_parent_frame(
    axis: tuple[float, float, float], rpy_raw: str | None
) -> tuple[float, float, float]:
    if not rpy_raw:
        return axis
    roll, pitch, yaw = (float(item) for item in rpy_raw.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF uses the fixed-axis Rz(yaw) * Ry(pitch) * Rx(roll) convention.
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return tuple(sum(rotation[i][j] * axis[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def parse_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("URDF root element must be <robot>")
    links = [str(node.attrib.get("name", "")).strip() for node in root.findall("link")]
    if not links or any(not item for item in links) or len(links) != len(set(links)):
        raise ValueError("URDF links must be nonempty and unique")
    joints: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        parent_node = node.find("parent")
        child_node = node.find("child")
        if parent_node is None or child_node is None:
            raise ValueError("each URDF joint requires parent and child")
        limit = node.find("limit")
        mimic = node.find("mimic")
        axis = node.find("axis")
        origin = node.find("origin")
        local_axis = parse_vector(
            axis.attrib.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)
        )
        lower = float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None
        upper = float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None
        if any(value is not None and not math.isfinite(value) for value in (lower, upper)):
            raise ValueError("URDF joint limits must be finite")
        joints.append(
            {
                "name": str(node.attrib.get("name", "")).strip(),
                "type": str(node.attrib.get("type", "")).strip().lower(),
                "parent": str(parent_node.attrib.get("link", "")).strip(),
                "child": str(child_node.attrib.get("link", "")).strip(),
                "axis_parent_frame": axis_in_parent_frame(
                    local_axis, origin.attrib.get("rpy") if origin is not None else None
                ),
                "lower": lower,
                "upper": upper,
                "mimic": dict(mimic.attrib) if mimic is not None else None,
            }
        )
    joint_names = [row["name"] for row in joints]
    if any(not item for item in joint_names) or len(joint_names) != len(set(joint_names)):
        raise ValueError("URDF joints must be nonempty and unique")
    mesh_refs = [
        str(node.attrib.get("filename", "")).strip()
        for node in root.findall(".//mesh")
        if str(node.attrib.get("filename", "")).strip()
    ]
    return {"links": links, "joints": joints, "mesh_refs": mesh_refs}


def tree_check(links: list[str], joints: list[dict[str, Any]]) -> dict[str, Any]:
    link_set = set(links)
    endpoints_valid = all(row["parent"] in link_set and row["child"] in link_set for row in joints)
    parent_count: dict[str, int] = {name: 0 for name in links}
    adjacency: dict[str, set[str]] = {name: set() for name in links}
    for row in joints:
        if row["child"] in parent_count:
            parent_count[row["child"]] += 1
        if row["parent"] in adjacency and row["child"] in adjacency:
            adjacency[row["parent"]].add(row["child"])
            adjacency[row["child"]].add(row["parent"])
    roots = [name for name, count in parent_count.items() if count == 0]
    visited: set[str] = set()
    pending = [roots[0]] if roots else []
    while pending:
        name = pending.pop()
        if name in visited:
            continue
        visited.add(name)
        pending.extend(adjacency[name] - visited)
    passed = (
        endpoints_valid
        and len(roots) == 1
        and all(count <= 1 for count in parent_count.values())
        and len(visited) == len(links)
        and len(joints) == len(links) - 1
    )
    return {
        "pass": passed,
        "link_count": len(links),
        "joint_count": len(joints),
        "root_count": len(roots),
        "connected_link_count": len(visited),
        "endpoints_valid": endpoints_valid,
    }


def resolve_mesh_ref(ref: str, urdf: Path, run_root: Path) -> Path | None:
    candidates: list[Path] = []
    if ref.startswith("file://"):
        candidates.append(Path(ref[7:]))
    elif ref.startswith("package://"):
        tail = ref[len("package://") :]
        candidates.extend([run_root / tail, run_root / "/".join(tail.split("/")[1:])])
    else:
        raw = Path(ref)
        candidates.append(raw if raw.is_absolute() else urdf.parent / raw)
    for candidate in candidates:
        try:
            resolved = contained(candidate, run_root)
        except ValueError:
            continue
        if resolved.is_file():
            return resolved
    basename = Path(ref).name
    matches = [item for item in run_root.rglob(basename) if item.is_file()]
    return matches[0].resolve() if len(matches) == 1 else None


def mesh_load_check(paths: list[Path]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    try:
        import trimesh  # type: ignore[import-not-found]
    except ImportError as exc:
        return {"pass": False, "error": f"trimesh unavailable: {exc}", "records": []}
    for path in paths:
        row: dict[str, Any] = {"path": str(path), "sha256": sha256_file(path), "pass": False}
        try:
            loaded = trimesh.load(path, process=False)
            geometries = list(loaded.geometry.values()) if isinstance(loaded, trimesh.Scene) else [loaded]
            vertices = sum(len(getattr(item, "vertices", ())) for item in geometries)
            faces = sum(len(getattr(item, "faces", ())) for item in geometries)
            row.update({"pass": vertices > 0 and faces > 0, "vertices": vertices, "faces": faces})
        except Exception as exc:  # noqa: BLE001
            row["error"] = f"{type(exc).__name__}: {exc}"
        rows.append(row)
    return {"pass": bool(rows) and all(row["pass"] for row in rows), "records": rows}


def semantic_check(links: list[str], roles: list[dict[str, Any]]) -> dict[str, Any]:
    slots: list[tuple[int, int]] = []
    candidates: dict[tuple[int, int], list[str]] = {}
    for role_index, role in enumerate(roles):
        hits = sorted(name for name in links if alias_match(name, role["aliases"]))
        for slot_index in range(role["min_count"]):
            slot = (role_index, slot_index)
            slots.append(slot)
            candidates[slot] = hits

    link_to_slot: dict[str, tuple[int, int]] = {}

    def assign(slot: tuple[int, int], seen: set[str]) -> bool:
        for link in candidates[slot]:
            if link in seen:
                continue
            seen.add(link)
            previous = link_to_slot.get(link)
            if previous is None or assign(previous, seen):
                link_to_slot[link] = slot
                return True
        return False

    for slot in sorted(slots, key=lambda value: (len(candidates[value]), value)):
        assign(slot, set())
    slot_to_link = {slot: link for link, slot in link_to_slot.items()}

    records = []
    for role_index, role in enumerate(roles):
        candidate_hits = sorted(name for name in links if alias_match(name, role["aliases"]))
        matched = sorted(
            slot_to_link[slot]
            for slot in slots
            if slot[0] == role_index and slot in slot_to_link
        )
        records.append(
            {
                "role_id": role["role_id"],
                "minimum": role["min_count"],
                "candidate_links": candidate_hits,
                "matched_links": matched,
                "pass": len(matched) >= role["min_count"],
            }
        )
    return {"pass": bool(records) and all(row["pass"] for row in records), "records": records}


def joint_check(joints: list[dict[str, Any]], roles: list[dict[str, Any]], spec: dict[str, Any]) -> dict[str, Any]:
    role_by_id = {row["role_id"]: row for row in roles}
    axis_tolerance_deg = float(spec.get("axis_tolerance_degrees", 12.0))
    limit_tolerance = float(spec.get("limit_tolerance", 0.05))

    expected_rows = [row for row in spec.get("required_joints", []) if isinstance(row, dict)]

    def core_reasons(expected: dict[str, Any], actual: dict[str, Any]) -> list[str]:
        reasons: list[str] = []
        expected_type = str(expected.get("type", "")).lower()
        if expected_type not in {"fixed", "floating", "planar"}:
            expected_axis = parse_vector(
                " ".join(str(item) for item in expected.get("axis", (1, 0, 0))),
                (1.0, 0.0, 0.0),
            )
            dot = max(
                -1.0,
                min(
                    1.0,
                    sum(a * b for a, b in zip(expected_axis, actual["axis_parent_frame"])),
                ),
            )
            axis_error = math.degrees(math.acos(dot))
            if axis_error > axis_tolerance_deg:
                reasons.append(
                    f"axis error {axis_error:.3f} deg exceeds {axis_tolerance_deg:.3f}"
                )
        if expected_type in {"revolute", "prismatic"}:
            for key, relation in (("lower", "at_most"), ("upper", "at_least")):
                wanted = expected.get(key)
                actual_value = actual.get(key)
                if wanted is None:
                    continue
                if actual_value is None:
                    reasons.append(f"missing {key} limit")
                elif relation == "at_most" and actual_value > float(wanted) + limit_tolerance:
                    reasons.append(f"lower limit {actual_value} does not cover {wanted}")
                elif relation == "at_least" and actual_value < float(wanted) - limit_tolerance:
                    reasons.append(f"upper limit {actual_value} does not cover {wanted}")
        return reasons

    candidate_indexes: dict[int, list[int]] = {}
    candidate_failures: dict[tuple[int, int], list[str]] = {}
    eligible_indexes: dict[int, list[int]] = {}
    for expected_index, expected in enumerate(expected_rows):
        parent_role = role_by_id.get(str(expected.get("parent_role", "")), {"aliases": []})
        child_role = role_by_id.get(str(expected.get("child_role", "")), {"aliases": []})
        expected_type = str(expected.get("type", "")).lower()
        indexes = [
            index
            for index, row in enumerate(joints)
            if alias_match(row["parent"], parent_role["aliases"])
            and alias_match(row["child"], child_role["aliases"])
            and row["type"] == expected_type
        ]
        candidate_indexes[expected_index] = indexes
        for joint_index in indexes:
            candidate_failures[(expected_index, joint_index)] = core_reasons(
                expected, joints[joint_index]
            )
        eligible_indexes[expected_index] = [
            joint_index
            for joint_index in indexes
            if not candidate_failures[(expected_index, joint_index)]
        ]

    joint_to_expected: dict[int, int] = {}

    def assign(expected_index: int, seen: set[int]) -> bool:
        for joint_index in eligible_indexes[expected_index]:
            if joint_index in seen:
                continue
            seen.add(joint_index)
            previous = joint_to_expected.get(joint_index)
            if previous is None or assign(previous, seen):
                joint_to_expected[joint_index] = expected_index
                return True
        return False

    for expected_index in sorted(
        range(len(expected_rows)), key=lambda value: (len(eligible_indexes[value]), value)
    ):
        assign(expected_index, set())
    expected_to_joint = {
        expected_index: joint_index for joint_index, expected_index in joint_to_expected.items()
    }
    expected_id_to_index = {
        str(expected.get("joint_id", "")): index
        for index, expected in enumerate(expected_rows)
    }

    records: list[dict[str, Any]] = []
    for expected_index, expected in enumerate(expected_rows):
        joint_index = expected_to_joint.get(expected_index)
        matched = joints[joint_index] if joint_index is not None else None
        reasons: list[str] = []
        if matched is None:
            indexes = candidate_indexes[expected_index]
            if not indexes:
                reasons.append("missing parent/child/type match")
            elif eligible_indexes[expected_index]:
                reasons.append("no distinct joint candidate available")
            else:
                best = min(indexes, key=lambda value: len(candidate_failures[(expected_index, value)]))
                matched = joints[best]
                reasons.extend(candidate_failures[(expected_index, best)])

        expected_mimic = expected.get("mimic") if expected.get("mimic_required") else None
        if matched is not None and isinstance(expected_mimic, dict):
            actual_mimic = matched.get("mimic")
            if not isinstance(actual_mimic, dict):
                reasons.append("required mimic relationship missing")
            else:
                target_index = expected_id_to_index.get(str(expected_mimic.get("joint", "")))
                target_joint_index = expected_to_joint.get(target_index) if target_index is not None else None
                target_name = joints[target_joint_index]["name"] if target_joint_index is not None else None
                if not target_name or actual_mimic.get("joint") != target_name:
                    reasons.append("mimic target does not match the required joint")
                for key, default in (("multiplier", 1.0), ("offset", 0.0)):
                    wanted = float(expected_mimic.get(key, default))
                    try:
                        actual_value = float(actual_mimic.get(key, default))
                    except (TypeError, ValueError):
                        reasons.append(f"invalid mimic {key}")
                        continue
                    if not math.isfinite(actual_value) or abs(actual_value - wanted) > 1e-6:
                        reasons.append(f"mimic {key} does not match")
        records.append(
            {
                "joint_id": expected.get("joint_id"),
                "parent_role": expected.get("parent_role"),
                "child_role": expected.get("child_role"),
                "type": expected_type,
                "matched_joint": matched["name"] if matched else None,
                "pass": not reasons,
                "failure_codes": reasons,
            }
        )
    return {"pass": bool(records) and all(row["pass"] for row in records), "records": records}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-manifest", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--hidden-specs", type=Path, default=DEFAULT_HIDDEN_SPECS)
    parser.add_argument("--package-schema", type=Path, default=DEFAULT_PACKAGE_SCHEMA)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    package_path = contained(args.package_manifest, ALLOWED_OUTPUT_ROOT)
    output_path = contained(args.output, ALLOWED_OUTPUT_ROOT)
    manifest_path = contained(args.manifest, REPO_ROOT)
    protocol_path = contained(args.protocol, REPO_ROOT)
    hidden_path = contained(args.hidden_specs, REPO_ROOT)
    package_schema_path = contained(args.package_schema, REPO_ROOT)
    package = read_object(package_path)
    manifest = read_object(manifest_path)
    protocol = read_object(protocol_path)
    hidden = read_object(hidden_path)
    package_schema = read_object(package_schema_path)
    try:
        import jsonschema  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(f"jsonschema is required for package validation: {exc}") from exc
    jsonschema.Draft202012Validator(package_schema).validate(package)
    run_root_raw = Path(str(package["run_root"]))
    if not run_root_raw.is_absolute():
        run_root_raw = package_path.parent / run_root_raw
    run_root = contained(run_root_raw, ALLOWED_OUTPUT_ROOT)
    package_path.relative_to(run_root)
    output_path.relative_to(run_root)

    bindings = {
        "manifest_sha256": sha256_file(manifest_path),
        "protocol_sha256": sha256_file(protocol_path),
        "hidden_specs_sha256": sha256_file(hidden_path),
        "common_evaluator_sha256": sha256_file(Path(__file__).resolve()),
        "package_schema_sha256": sha256_file(package_schema_path),
        "package_manifest_sha256": sha256_file(package_path),
    }
    declared = package.get("bindings", {})
    binding_checks = {
        key: isinstance(declared, dict) and declared.get(key) == value
        for key, value in bindings.items()
        if key not in {"package_manifest_sha256"}
    }
    protocol_checks = {
        "manifest": protocol.get("manifest", {}).get("sha256") == bindings["manifest_sha256"],
        "hidden_specs": protocol.get("hidden_specs", {}).get("sha256") == bindings["hidden_specs_sha256"],
        "evaluator": protocol.get("common_evaluator", {}).get("sha256")
        == bindings["common_evaluator_sha256"],
        "package_schema": protocol.get("package_schema", {}).get("sha256")
        == bindings["package_schema_sha256"],
    }

    task_id = str(package.get("task_id", ""))
    repeat_id = str(package.get("repeat_id", ""))
    attempt_index = package.get("attempt_index")
    public_task = next((row for row in manifest.get("tasks", []) if row.get("task_id") == task_id), None)
    hidden_spec = next((row for row in hidden.get("specs", []) if row.get("task_id") == task_id), None)
    if public_task is None or hidden_spec is None:
        raise ValueError(f"task not found in both frozen inputs: {task_id!r}")
    spec_for_hash = {key: value for key, value in hidden_spec.items() if key != "hidden_spec_sha256"}
    spec_hash = hashlib.sha256(canonical_bytes(spec_for_hash)).hexdigest()
    task_checks = {
        "repeat_id_frozen": repeat_id in manifest.get("repeat_ids", []),
        "attempt_index_valid": isinstance(attempt_index, int)
        and 0 <= attempt_index <= int(protocol.get("max_common_repair_turns", -1)),
        "public_hidden_spec_hash": public_task.get("hidden_spec_sha256") == spec_hash,
        "private_hidden_spec_hash": hidden_spec.get("hidden_spec_sha256") == spec_hash,
    }

    source_info = package.get("artifacts", {}).get("source", {})
    urdf_info = package.get("artifacts", {}).get("urdf", {})
    source = resolve_artifact(str(source_info.get("path", "")), package_path, run_root)
    urdf = resolve_artifact(str(urdf_info.get("path", "")), package_path, run_root)
    source_ok = source.is_file() and source.stat().st_size > 0
    urdf_ok = urdf.is_file() and urdf.stat().st_size > 0
    declared_hash_checks = {
        "source": source_ok and source_info.get("sha256") == sha256_file(source),
        "urdf": urdf_ok and urdf_info.get("sha256") == sha256_file(urdf),
    }

    parsed: dict[str, Any] = {"links": [], "joints": [], "mesh_refs": []}
    parse_error = None
    if urdf_ok:
        try:
            parsed = parse_urdf(urdf)
        except Exception as exc:  # noqa: BLE001
            parse_error = f"{type(exc).__name__}: {exc}"
    mesh_paths: list[Path] = []
    unresolved_mesh_refs: list[str] = []
    for ref in parsed["mesh_refs"]:
        resolved = resolve_mesh_ref(ref, urdf, run_root)
        if resolved is None:
            unresolved_mesh_refs.append(ref)
        elif resolved not in mesh_paths:
            mesh_paths.append(resolved)
    mesh_check = mesh_load_check(mesh_paths) if mesh_paths else {"pass": False, "records": []}
    topology = tree_check(parsed["links"], parsed["joints"]) if not parse_error else {"pass": False}
    roles = role_rows(hidden_spec)
    semantics = semantic_check(parsed["links"], roles) if not parse_error else {"pass": False, "records": []}
    joints = joint_check(parsed["joints"], roles, hidden_spec) if not parse_error else {"pass": False, "records": []}

    probe = package.get("execution_probe", {})
    execution_timeout = float(protocol.get("timeouts", {}).get("execution_seconds_per_attempt", 0))
    probe_wall_time = probe.get("wall_time_s") if isinstance(probe, dict) else None
    executable = (
        source_ok
        and isinstance(probe, dict)
        and probe.get("exit_code") == 0
        and probe.get("timed_out") is False
        and probe.get("source_sha256") == sha256_file(source)
        and isinstance(probe_wall_time, (int, float))
        and 0 <= probe_wall_time <= execution_timeout
    )
    artifact_saved = (
        source_ok
        and urdf_ok
        and parse_error is None
        and bool(parsed["mesh_refs"])
        and not unresolved_mesh_refs
        and mesh_check["pass"]
        and all(declared_hash_checks.values())
    )
    common_qc_pass = (
        executable
        and artifact_saved
        and topology["pass"]
        and semantics["pass"]
        and joints["pass"]
        and all(binding_checks.values())
        and all(protocol_checks.values())
        and all(task_checks.values())
    )
    feedback = {
        "schema_version": 1,
        "task_id": task_id,
        "attempt_index": attempt_index,
        "failure_codes": [
            name
            for name, passed in {
                "EXECUTION_PROBE_FAILED": executable,
                "ARTIFACT_PACKAGE_FAILED": artifact_saved,
                "URDF_TREE_FAILED": bool(topology.get("pass")),
                "SEMANTIC_ROLES_FAILED": bool(semantics.get("pass")),
                "JOINT_SPEC_FAILED": bool(joints.get("pass")),
                "INPUT_BINDING_FAILED": all(binding_checks.values()) and all(protocol_checks.values()) and all(task_checks.values()),
            }.items()
            if not passed
        ],
        "bounded_diagnostics": {
            "unresolved_mesh_reference_count": len(unresolved_mesh_refs),
            "urdf_parse_error_class": parse_error.split(":", 1)[0] if parse_error else None,
            "tree_root_count": topology.get("root_count"),
            "tree_connected_link_count": topology.get("connected_link_count"),
            "tree_link_count": topology.get("link_count"),
        },
        "policy": (
            "Only common gate identifiers and output-derived bounded diagnostics are exposed; "
            "expected roles, joints, axes, limits, thresholds, and reserved constraints remain withheld."
        ),
    }
    report = {
        "schema_version": 1,
        "evaluator_id": "nano3d_table1_common_package_evaluator_v1",
        "evaluated_at_utc": datetime.now(timezone.utc).isoformat(),
        "method_id": package.get("method_id"),
        "run_id": package.get("run_id"),
        "task_id": task_id,
        "repeat_id": repeat_id,
        "attempt_index": attempt_index,
        "bindings": bindings,
        "binding_checks": binding_checks,
        "protocol_checks": protocol_checks,
        "task_checks": task_checks,
        "verdicts": {
            "executable": executable,
            "artifact_saved": artifact_saved,
            "urdf_tree_pass": bool(topology.get("pass")),
            "semantic_roles_pass": bool(semantics.get("pass")),
            "joint_spec_pass": bool(joints.get("pass")),
            "common_qc_pass": common_qc_pass,
        },
        "urdf": {
            "parse_error": parse_error,
            "topology": topology,
            "semantics": semantics,
            "joints": joints,
            "mesh_reference_count": len(parsed["mesh_refs"]),
            "unresolved_mesh_refs": unresolved_mesh_refs,
        },
        "mesh_load": mesh_check,
        "declared_artifact_hash_checks": declared_hash_checks,
        "feedback": feedback,
        "claim_boundary": (
            "Table 1 gate covers execution, saved package, URDF tree, semantic roles, and "
            "declared joint topology/type/axis/limits. Evaluator-private H1/H2 geometric "
            "constraints are reserved for later benchmark axes and do not affect this verdict."
        ),
    }
    dump_json(output_path, report)
    print(json.dumps({"output": str(output_path), "common_qc_pass": common_qc_pass}, sort_keys=True))
    return 2 if args.require_pass and not common_qc_pass else 0


if __name__ == "__main__":
    raise SystemExit(main())
