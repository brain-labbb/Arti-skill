#!/usr/bin/env python3
"""Run the frozen PV-A arm of the Nano3D Table 5 editability protocol.

The task-to-template mapping is data, not code.  ``--preflight`` validates the
mapping against the current parent manifest without compiling artifacts.
``--run`` additionally requires the caller to provide the exact frozen parent
manifest SHA-256 and compiles the parent phase before the edited phase.

All generated files stay below ``exp/runtime/nano3d_editability_v2/pva``.
Templates and Articraft records are read-only inputs.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import hashlib
import importlib
import json
import math
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from statistics import mean, median
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = EXP_ROOT.parent
WORKSPACE = PROJECT_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"
DEFAULT_MANIFEST = EXP_ROOT / "reference/editability_v2/articraft_parent_manifest.json"
DEFAULT_MAPPING = EXP_ROOT / "reference/editability_v2/pva_task_mapping.json"
DEFAULT_OUT = EXP_ROOT / "runtime/nano3d_editability_v2/pva"
PROTOCOL_ID = "nano3d_table5_pva_representation_matched_v2"
FROZEN_SEEDS = tuple(range(16))
MAIN_SEED = 0

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

import run_nano3d_editability as local  # noqa: E402


def contained(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    contained(path.parent, must_exist=False).mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def normalize(value: Any) -> Any:
    try:
        import numpy as np

        if isinstance(value, np.ndarray):
            array = np.ascontiguousarray(value)
            return {
                "ndarray_shape": list(array.shape),
                "ndarray_dtype": str(array.dtype),
                "ndarray_sha256": hashlib.sha256(array.tobytes()).hexdigest(),
            }
    except ImportError:
        pass
    if hasattr(value, "vertices") and hasattr(value, "faces"):
        return {
            "mesh_vertices": normalize(value.vertices),
            "mesh_faces": normalize(value.faces),
        }
    if dataclasses.is_dataclass(value):
        return {
            field.name: normalize(getattr(value, field.name))
            for field in dataclasses.fields(value)
            if field.name not in {"assets", "materialized_path"}
        }
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def replace_names(value: str, replacements: dict[str, str]) -> str:
    result = value
    for old, new in replacements.items():
        result = result.replace(old, new)
    return result


def signature(value: Any, replacements: dict[str, str] | None = None) -> str:
    encoded_text = json.dumps(normalize(value), sort_keys=True, default=str)
    encoded_text = replace_names(encoded_text, replacements or {})
    encoded = encoded_text.encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def manifest_binding_sha256(task: dict[str, Any]) -> str:
    fields = {
        key: task.get(key)
        for key in ("edit_class", "category", "subcategory", "normalized_instruction")
    }
    return hashlib.sha256(
        json.dumps(fields, ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()


def task_module(mapping: dict[str, Any]) -> Any:
    return importlib.import_module(f"agent.templates.{mapping['slug']}")


def apply_overrides(config: Any, overrides: dict[str, Any]) -> Any:
    fields = {field.name for field in dataclasses.fields(config)}
    unknown = sorted(set(overrides) - fields)
    if unknown:
        raise ValueError(f"unknown config fields: {unknown}")
    return dataclasses.replace(config, **overrides)


def config_pair(module: Any, mapping: dict[str, Any], seed: int) -> tuple[Any, Any]:
    sampled = module.config_from_seed(seed)
    shared = dict(mapping.get("shared_overrides") or {})
    parent = apply_overrides(sampled, {**shared, **mapping["parent_overrides"]})
    edited = apply_overrides(sampled, {**shared, **mapping["edited_overrides"]})
    return parent, edited


def resolved_metrics(module: Any, resolved: Any) -> dict[str, float]:
    metrics = {
        field.name: float(getattr(resolved, field.name))
        for field in dataclasses.fields(resolved)
        if isinstance(getattr(resolved, field.name), (int, float))
        and not isinstance(getattr(resolved, field.name), bool)
    }
    if module.__name__.endswith("pictureX_0611_hydraulic_jack2"):
        length = float(resolved.chassis_len_m)
        if resolved.chassis_style == "low_profile":
            metrics["chassis_front_extent_m"] = 0.66 * length - 0.010
            metrics["chassis_front_side_height_m"] = 0.66 * float(module.CHASSIS_H)
        else:
            metrics["chassis_front_extent_m"] = 0.50 * length
            metrics["chassis_front_side_height_m"] = float(module.CHASSIS_H)
    return metrics


def _op_pass(actual: float, op: str, expected: float, tolerance: float) -> bool:
    if op == "eq":
        return abs(actual - expected) <= tolerance
    if op == "ge":
        return actual + tolerance >= expected
    if op == "gt":
        return actual > expected + tolerance
    if op == "le":
        return actual - tolerance <= expected
    if op == "lt":
        return actual < expected - tolerance
    raise ValueError(f"unsupported predicate op: {op}")


def score_numeric_predicates(
    module: Any,
    parent: Any,
    edited: Any,
    predicates: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    parent_resolved = module.resolve_config(parent)
    edited_resolved = module.resolve_config(edited)
    parent_metrics = resolved_metrics(module, parent_resolved)
    edited_metrics = resolved_metrics(module, edited_resolved)
    rows: list[dict[str, Any]] = []
    for rule in predicates:
        source = rule.get("source", "resolved")
        if source == "resolved":
            left = edited_metrics[rule["edited_field"]]
            right_values = parent_metrics
        else:
            left = float(getattr(edited, rule["edited_field"]))
            right_values = {
                field.name: float(getattr(parent, field.name))
                for field in dataclasses.fields(parent)
                if isinstance(getattr(parent, field.name), (int, float))
            }
        tolerance = float(rule.get("tolerance", 1.0e-9))
        if "parent_field" in rule:
            right = right_values[rule["parent_field"]]
            expected = right * float(rule.get("factor", 1.0)) + float(rule.get("offset", 0.0))
        else:
            right = None
            expected = float(rule["value"])
        passed = _op_pass(left, str(rule["op"]), expected, tolerance)
        rows.append(
            {
                **rule,
                "edited_actual": left,
                "parent_actual": right,
                "expected_boundary": expected,
                "passed": passed,
            }
        )
    return rows


def source_graph(model: Any, mapping: dict[str, Any]) -> dict[str, Any]:
    replacements = dict(mapping["locality"].get("canonical_replacements") or {})
    parts = {
        replace_names(part.name, replacements): signature(part, replacements)
        for part in model.parts
    }
    part_declarations = {
        replace_names(part.name, replacements): replace_names(
            json.dumps(normalize(part), sort_keys=True, default=str), replacements
        )
        for part in model.parts
    }
    joints: dict[str, dict[str, Any]] = {}
    for joint in model.articulations:
        joint_type = getattr(joint.articulation_type, "value", joint.articulation_type)
        joint_name = replace_names(joint.name, replacements)
        joints[joint_name] = {
            "type": str(joint_type).lower(),
            "parent": replace_names(str(joint.parent), replacements),
            "child": replace_names(str(joint.child), replacements),
            "signature": signature(joint, replacements),
            "declaration": replace_names(
                json.dumps(normalize(joint), sort_keys=True, default=str), replacements
            ),
        }
    return {
        "parts": parts,
        "part_declarations": part_declarations,
        "joints": joints,
    }


def searchable_graph(graph: dict[str, Any]) -> dict[str, list[dict[str, str]]]:
    parts = [
        {
            "name": name,
            "search": str(declaration).lower(),
        }
        for name, declaration in graph["part_declarations"].items()
    ]
    joints = [
        {
            "name": name,
            "type": row["type"],
            "search": str(row["declaration"]).lower(),
        }
        for name, row in graph["joints"].items()
    ]
    return {"part": parts, "joint": joints}


def rule_matches(rule: dict[str, Any], parent_graph: dict[str, Any], edit_graph: dict[str, Any]) -> dict[str, Any]:
    scope = str(rule.get("scope", "edited"))
    entity = str(rule["entity"])
    tokens = tuple(str(item).lower() for item in rule.get("tokens", ()))
    joint_type = str(rule.get("joint_type", "")).lower()
    source = searchable_graph(parent_graph if scope == "parent" else edit_graph)[entity]
    matches = [
        row["name"]
        for row in source
        if all(token in row["search"] for token in tokens)
        and (not joint_type or row.get("type") == joint_type)
    ]
    minimum = int(rule.get("min_count", 1))
    maximum = rule.get("max_count")
    passed = len(matches) >= minimum and (maximum is None or len(matches) <= int(maximum))
    if scope == "delta_added":
        parent_names = {
            row["name"]
            for row in searchable_graph(parent_graph)[entity]
            if all(token in row["search"] for token in tokens)
            and (not joint_type or row.get("type") == joint_type)
        }
        matches = sorted(set(matches) - parent_names)
        passed = len(matches) >= minimum and (maximum is None or len(matches) <= int(maximum))
    return {**rule, "matches": sorted(matches), "passed": passed}


def tokens_match(name: str, tokens: tuple[str, ...]) -> bool:
    lowered = name.lower()
    return any(token.lower() in lowered for token in tokens)


def partition_graphs(
    parent_graph: dict[str, Any],
    edit_graph: dict[str, Any],
    mapping: dict[str, Any],
) -> dict[str, Any]:
    part_tokens = tuple(mapping["locality"]["allowed_changed_part_tokens"])
    joint_tokens = tuple(mapping["locality"]["allowed_changed_joint_tokens"])
    part_names = set(parent_graph["parts"]) | set(edit_graph["parts"])
    joint_names = set(parent_graph["joints"]) | set(edit_graph["joints"])
    changed_parts = {
        name
        for name in part_names
        if parent_graph["parts"].get(name) != edit_graph["parts"].get(name)
    }
    changed_joints = {
        name
        for name in joint_names
        if parent_graph["joints"].get(name, {}).get("signature")
        != edit_graph["joints"].get(name, {}).get("signature")
    }
    allowed_parts = {name for name in part_names if tokens_match(name, part_tokens)}
    allowed_joints = {name for name in joint_names if tokens_match(name, joint_tokens)}
    for name, row in {**parent_graph["joints"], **edit_graph["joints"]}.items():
        if row["parent"] in allowed_parts or row["child"] in allowed_parts:
            allowed_joints.add(name)
    return {
        "changed_parts": sorted(changed_parts),
        "changed_joints": sorted(changed_joints),
        "allowed_changed_parts": sorted(allowed_parts),
        "allowed_changed_joints": sorted(allowed_joints),
        "unexpected_changed_parts": sorted(changed_parts - allowed_parts),
        "unexpected_changed_joints": sorted(changed_joints - allowed_joints),
        "true_non_target_parts": sorted(part_names - allowed_parts),
        "true_non_target_joints": sorted(joint_names - allowed_joints),
        "geometry_locality": changed_parts <= allowed_parts,
        "structural_locality": changed_joints <= allowed_joints,
    }


def build_pair(module: Any, mapping: dict[str, Any], parent: Any, edited: Any, task_root: Path) -> tuple[Any, Any]:
    from sdk import AssetContext

    parent_source = task_root / "preflight_parent/source.py"
    edit_source = task_root / "preflight_edited/source.py"
    parent_source.parent.mkdir(parents=True, exist_ok=True)
    edit_source.parent.mkdir(parents=True, exist_ok=True)
    parent_source.touch(exist_ok=True)
    edit_source.touch(exist_ok=True)
    build = getattr(module, f"build_{mapping['stem']}")
    parent_model = build(parent, assets=AssetContext.from_script(parent_source))
    edit_model = build(edited, assets=AssetContext.from_script(edit_source))
    return parent_model, edit_model


def preflight_one(mapping: dict[str, Any], seed: int, output: Path, *, build_models: bool) -> dict[str, Any]:
    module = task_module(mapping)
    parent, edited = config_pair(module, mapping, seed)
    parent_payload = normalize(parent)
    edit_payload = normalize(edited)
    changed_fields = sorted(
        field.name
        for field in dataclasses.fields(parent)
        if getattr(parent, field.name) != getattr(edited, field.name)
    )
    declared_fields = sorted(set(mapping["target_fields"]))
    numeric = score_numeric_predicates(
        module,
        parent,
        edited,
        list(mapping.get("numeric_predicates") or ()),
    )
    row: dict[str, Any] = {
        "task_id": mapping["task_id"],
        "seed": seed,
        "slug": mapping["slug"],
        "parent_config": parent_payload,
        "edited_config": edit_payload,
        "changed_fields": changed_fields,
        "declared_target_fields": declared_fields,
        "field_locality": changed_fields == declared_fields,
        "numeric_predicates": numeric,
        "numeric_pass": all(item["passed"] for item in numeric),
        "model_build_performed": build_models,
    }
    if build_models:
        parent_model, edit_model = build_pair(module, mapping, parent, edited, output)
        parent_graph = source_graph(parent_model, mapping)
        edit_graph = source_graph(edit_model, mapping)
        rules = [
            rule_matches(rule, parent_graph, edit_graph)
            for rule in mapping.get("source_rules") or ()
        ]
        partition = partition_graphs(parent_graph, edit_graph, mapping)
        row.update(
            {
                "source_rules": rules,
                "source_rules_pass": all(item["passed"] for item in rules),
                "source_partition": partition,
                "source_locality_pass": bool(
                    partition["geometry_locality"] and partition["structural_locality"]
                ),
                "parent_part_names": sorted(parent_graph["parts"]),
                "edited_part_names": sorted(edit_graph["parts"]),
                "parent_joint_rows": parent_graph["joints"],
                "edited_joint_rows": edit_graph["joints"],
            }
        )
    else:
        row["source_rules_pass"] = None
        row["source_locality_pass"] = None
    row["preflight_pass"] = bool(
        row["field_locality"]
        and row["numeric_pass"]
        and (row["source_rules_pass"] is not False)
        and (row["source_locality_pass"] is not False)
    )
    return row


def validate_inputs(manifest: dict[str, Any], mapping: dict[str, Any], manifest_sha: str) -> None:
    if mapping.get("protocol_id") != PROTOCOL_ID:
        raise RuntimeError("mapping protocol_id mismatch")
    if mapping.get("source_manifest_sha256") != manifest_sha:
        raise RuntimeError("mapping was not frozen against this parent manifest SHA-256")
    if mapping.get("source_scorer_sha256") != manifest.get("scorer_sha256"):
        raise RuntimeError("mapping scorer SHA-256 differs from the parent manifest")
    if mapping.get("source_prompt_contract_sha256") != manifest.get("prompt_contract_sha256"):
        raise RuntimeError("mapping prompt-contract SHA-256 differs from the parent manifest")
    if mapping.get("source_budget_guard_sha256") != manifest.get("budget_guard_sha256"):
        raise RuntimeError("mapping budget-guard SHA-256 differs from the parent manifest")
    if tuple(mapping.get("seeds") or ()) != FROZEN_SEEDS:
        raise RuntimeError("mapping seeds must be exactly 0..15")
    if mapping.get("main_seed") != MAIN_SEED:
        raise RuntimeError("mapping main_seed must be 0")
    tasks = manifest.get("tasks") or ()
    mapped = mapping.get("tasks") or ()
    if len(tasks) != 18 or len(mapped) != 18:
        raise RuntimeError("both manifests must contain exactly 18 tasks")
    manifest_ids = [row["task_id"] for row in tasks]
    mapping_ids = [row["task_id"] for row in mapped]
    if manifest_ids != mapping_ids:
        raise RuntimeError("mapping task order/IDs differ from parent manifest")
    blocked = [row["task_id"] for row in mapped if row.get("status") != "ready"]
    if blocked:
        raise RuntimeError(f"mapping contains blocked tasks: {blocked}")
    for source_task, mapped_task in zip(tasks, mapped):
        expected = mapped_task.get("manifest_binding_sha256")
        actual = manifest_binding_sha256(source_task)
        if expected != actual:
            raise RuntimeError(f"{mapped_task['task_id']}: manifest binding SHA-256 mismatch")


def write_wrapper(path: Path, mapping: dict[str, Any], seed: int, overrides: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = local.wrapper_text(mapping["slug"], mapping["stem"], seed, "pva", overrides)
    if path.is_file() and path.read_text(encoding="utf-8") != text:
        raise RuntimeError(f"existing frozen wrapper differs: {path}")
    if not path.exists():
        path.write_text(text, encoding="utf-8")


def compile_one(wrapper: Path, result: Path, timeout: float) -> dict[str, Any]:
    if result.is_file():
        return json.loads(result.read_text(encoding="utf-8"))
    started = time.monotonic()
    try:
        payload = local.run_one(wrapper, result, timeout)
    except BaseException as exc:  # noqa: BLE001
        payload = {
            "verdict": "fail",
            "error": f"worker_launch_or_run_exception: {type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc()[-6000:],
            "elapsed_s": time.monotonic() - started,
            "wrapper": str(wrapper),
        }
    # local.run_one returns timeout/crash payloads without necessarily writing
    # them. Every submitted case must have durable evidence before phase return.
    if not result.is_file():
        dump_json(result, payload)
    return payload


def compile_phase(
    jobs: list[tuple[Path, Path]], workers: int, timeout: float, label: str
) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        futures = {
            pool.submit(compile_one, wrapper, result, timeout): (wrapper, result)
            for wrapper, result in jobs
        }
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            wrapper, result = futures[future]
            try:
                payload = future.result()
            except BaseException as exc:  # noqa: BLE001
                payload = {
                    "verdict": "fail",
                    "error": f"future_result_exception: {type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc()[-6000:],
                    "elapsed_s": None,
                    "wrapper": str(wrapper),
                }
                dump_json(result, payload)
            results[str(wrapper)] = payload
            if index % 16 == 0 or index == len(futures):
                print(f"{label} {index}/{len(futures)}", flush=True)
    return results


def canonical(node: ET.Element) -> str:
    clone = ET.fromstring(ET.tostring(node, encoding="unicode"))
    return ET.tostring(clone, encoding="unicode", short_empty_elements=True)


def referenced_hashes(
    node: ET.Element, package: Path, replacements: dict[str, str]
) -> dict[str, str | None]:
    hashes: dict[str, str | None] = {}
    for element in node.iter():
        for field in ("filename", "texture"):
            reference = element.attrib.get(field)
            if not reference:
                continue
            candidate = Path(reference)
            if not candidate.is_absolute():
                candidate = package / candidate
            try:
                candidate = contained(candidate)
            except (FileNotFoundError, ValueError):
                hashes[replace_names(reference, replacements)] = None
            else:
                hashes[replace_names(reference, replacements)] = (
                    sha256(candidate) if candidate.is_file() else None
                )
    return hashes


def artifact_graph(path: Path, mapping: dict[str, Any]) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    package = path.parent
    replacements = dict(mapping["locality"].get("canonical_replacements") or {})
    links = {}
    for node in root.findall("link"):
        payload = {
            "xml": replace_names(canonical(node), replacements),
            "referenced_hashes": referenced_hashes(node, package, replacements),
        }
        name = replace_names(node.attrib.get("name", ""), replacements)
        links[name] = hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode()
        ).hexdigest()
    joints = {}
    for node in root.findall("joint"):
        parent = node.find("parent")
        child = node.find("child")
        name = replace_names(node.attrib.get("name", ""), replacements)
        joints[name] = {
            "type": node.attrib.get("type", ""),
            "parent": replace_names(
                parent.attrib.get("link", "") if parent is not None else "", replacements
            ),
            "child": replace_names(
                child.attrib.get("link", "") if child is not None else "", replacements
            ),
            "signature": hashlib.sha256(
                replace_names(canonical(node), replacements).encode()
            ).hexdigest(),
        }
    return {"links": links, "joints": joints}


def _floats(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(item) for item in value.split()) if value else default


def _transform_points(
    points: list[tuple[float, float, float]],
    xyz: tuple[float, ...],
    rpy: tuple[float, ...],
) -> list[tuple[float, float, float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return [
        tuple(
            sum(rotation[axis][column] * point[column] for column in range(3))
            + xyz[axis]
            for axis in range(3)
        )
        for point in points
    ]


def _box_corners(low: tuple[float, ...], high: tuple[float, ...]) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]


def _geometry_bounds(geometry: ET.Element, package: Path) -> tuple[tuple[float, ...], tuple[float, ...]]:
    box = geometry.find("box")
    if box is not None:
        size = _floats(box.attrib.get("size"), (0.0, 0.0, 0.0))
        return tuple(-item / 2.0 for item in size), tuple(item / 2.0 for item in size)
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.attrib["radius"])
        half = float(cylinder.attrib["length"]) / 2.0
        return (-radius, -radius, -half), (radius, radius, half)
    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.attrib["radius"])
        return (-radius, -radius, -radius), (radius, radius, radius)
    mesh = geometry.find("mesh")
    if mesh is None:
        raise ValueError("unsupported empty URDF geometry")
    import trimesh

    mesh_path = Path(mesh.attrib["filename"])
    if not mesh_path.is_absolute():
        mesh_path = package / mesh_path
    loaded = trimesh.load(contained(mesh_path), force="scene")
    bounds = loaded.bounds
    scale = _floats(mesh.attrib.get("scale"), (1.0, 1.0, 1.0))
    scaled = [
        tuple(float(bounds[row][axis]) * scale[axis] for axis in range(3))
        for row in range(2)
    ]
    return tuple(scaled[0]), tuple(scaled[1])


def urdf_visual_bounds(path: Path, mapping: dict[str, Any]) -> dict[str, dict[str, dict[str, list[float]]]]:
    root = ET.parse(path).getroot()
    package = path.parent
    replacements = dict(mapping["locality"].get("canonical_replacements") or {})
    result: dict[str, dict[str, dict[str, list[float]]]] = {}
    for link in root.findall("link"):
        link_name = replace_names(link.attrib.get("name", ""), replacements)
        visuals: dict[str, dict[str, list[float]]] = {}
        for index, visual in enumerate(link.findall("visual")):
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            low, high = _geometry_bounds(geometry, package)
            origin = visual.find("origin")
            xyz = _floats(None if origin is None else origin.attrib.get("xyz"), (0.0, 0.0, 0.0))
            rpy = _floats(None if origin is None else origin.attrib.get("rpy"), (0.0, 0.0, 0.0))
            points = _transform_points(_box_corners(low, high), xyz, rpy)
            mins = [min(point[axis] for point in points) for axis in range(3)]
            maxs = [max(point[axis] for point in points) for axis in range(3)]
            visual_name = replace_names(visual.attrib.get("name", f"visual_{index}"), replacements)
            visuals[visual_name] = {
                "min": mins,
                "max": maxs,
                "extent": [maxs[axis] - mins[axis] for axis in range(3)],
            }
        result[link_name] = visuals
    return result


def _select_compiled_measure(
    bounds: dict[str, dict[str, dict[str, list[float]]]], rule: dict[str, Any]
) -> tuple[float, list[str]]:
    link_tokens = tuple(str(item).lower() for item in rule.get("link_tokens", ()))
    visual_tokens = tuple(str(item).lower() for item in rule.get("visual_tokens", ()))
    selected: list[tuple[str, dict[str, list[float]]]] = []
    for link_name, visuals in bounds.items():
        if link_tokens and not all(token in link_name.lower() for token in link_tokens):
            continue
        for visual_name, visual_bounds in visuals.items():
            if visual_tokens and not all(token in visual_name.lower() for token in visual_tokens):
                continue
            selected.append((f"{link_name}/{visual_name}", visual_bounds))
    if not selected:
        raise ValueError(f"compiled predicate selected no visuals: {rule}")
    axis = {"x": 0, "y": 1, "z": 2}[str(rule["axis"]).lower()]
    low = min(item["min"][axis] for _, item in selected)
    high = max(item["max"][axis] for _, item in selected)
    measure = str(rule.get("measure", "extent"))
    value = {"min": low, "max": high, "extent": high - low}[measure]
    return value, [name for name, _ in selected]


def score_compiled_predicates(
    parent_path: Path,
    edit_path: Path,
    mapping: dict[str, Any],
) -> list[dict[str, Any]]:
    rules = list(mapping.get("compiled_predicates") or ())
    if not rules:
        return []
    parent_bounds = urdf_visual_bounds(parent_path, mapping)
    edit_bounds = urdf_visual_bounds(edit_path, mapping)
    rows = []
    for rule in rules:
        parent_value, parent_matches = _select_compiled_measure(parent_bounds, rule)
        edit_value, edit_matches = _select_compiled_measure(edit_bounds, rule)
        comparison = str(rule.get("comparison", "parent_factor"))
        evidence: dict[str, Any] = {}
        if comparison == "edited_ratio":
            denominator = dict(rule["denominator"])
            denominator_value, denominator_matches = _select_compiled_measure(
                edit_bounds, denominator
            )
            actual = edit_value / denominator_value
            expected = float(rule["value"])
            evidence = {
                "edited_numerator": edit_value,
                "edited_denominator": denominator_value,
                "denominator_matches": denominator_matches,
            }
        elif comparison == "ratio_difference":
            second = dict(rule["second_selector"])
            second_parent, second_parent_matches = _select_compiled_measure(
                parent_bounds, second
            )
            second_edit, second_edit_matches = _select_compiled_measure(edit_bounds, second)
            first_ratio = edit_value / parent_value
            second_ratio = second_edit / second_parent
            actual = abs(first_ratio - second_ratio)
            expected = float(rule["value"])
            evidence = {
                "first_ratio": first_ratio,
                "second_ratio": second_ratio,
                "second_parent_actual": second_parent,
                "second_edited_actual": second_edit,
                "second_parent_matches": second_parent_matches,
                "second_edited_matches": second_edit_matches,
            }
        else:
            actual = edit_value
            expected = parent_value * float(rule.get("factor", 1.0)) + float(
                rule.get("offset", 0.0)
            )
        tolerance = float(rule.get("tolerance", 1.0e-6))
        rows.append(
            {
                **rule,
                "parent_actual": parent_value,
                "edited_actual": edit_value,
                "comparison_actual": actual,
                "expected_boundary": expected,
                "parent_matches": parent_matches,
                "edited_matches": edit_matches,
                **evidence,
                "passed": _op_pass(actual, str(rule["op"]), expected, tolerance),
            }
        )
    return rows


def score_compiled_case(
    preflight: dict[str, Any],
    mapping: dict[str, Any],
    parent_run: dict[str, Any],
    edit_run: dict[str, Any],
    template_hash_before: str,
    template_hash_after: str,
) -> dict[str, Any]:
    parent_path = Path(parent_run.get("urdf_path") or "")
    edit_path = Path(edit_run.get("urdf_path") or "")
    parent_graph = (
        artifact_graph(parent_path, mapping)
        if parent_path.is_file()
        else {"links": {}, "joints": {}}
    )
    edit_graph = (
        artifact_graph(edit_path, mapping)
        if edit_path.is_file()
        else {"links": {}, "joints": {}}
    )
    compiled_predicates = (
        score_compiled_predicates(parent_path, edit_path, mapping)
        if parent_path.is_file() and edit_path.is_file()
        else []
    )
    part_tokens = tuple(mapping["locality"]["allowed_changed_part_tokens"])
    joint_tokens = tuple(mapping["locality"]["allowed_changed_joint_tokens"])
    link_names = set(parent_graph["links"]) | set(edit_graph["links"])
    joint_names = set(parent_graph["joints"]) | set(edit_graph["joints"])
    changed_links = {
        name
        for name in link_names
        if parent_graph["links"].get(name) != edit_graph["links"].get(name)
    }
    changed_joints = {
        name
        for name in joint_names
        if parent_graph["joints"].get(name, {}).get("signature")
        != edit_graph["joints"].get(name, {}).get("signature")
    }
    allowed_links = {name for name in link_names if tokens_match(name, part_tokens)}
    allowed_joints = {name for name in joint_names if tokens_match(name, joint_tokens)}
    for name, row in {**parent_graph["joints"], **edit_graph["joints"]}.items():
        if row["parent"] in allowed_links or row["child"] in allowed_links:
            allowed_joints.add(name)
    compile_pass = parent_run.get("verdict") == "pass" and edit_run.get("verdict") == "pass"
    target_fulfilled = bool(
        compile_pass
        and preflight["field_locality"]
        and preflight["numeric_pass"]
        and preflight["source_rules_pass"]
        and preflight["source_locality_pass"]
        and all(row["passed"] for row in compiled_predicates)
    )
    non_target_preserved = all(
        parent_graph["links"].get(name) == edit_graph["links"].get(name)
        for name in link_names - allowed_links
    )
    structural_locality = changed_joints <= allowed_joints
    geometry_locality = changed_links <= allowed_links
    anchor = bool(target_fulfilled and structural_locality)
    scale = bool(
        target_fulfilled
        and all(row["passed"] for row in preflight["numeric_predicates"])
        and all(row["passed"] for row in compiled_predicates)
    )
    regression = bool(compile_pass and template_hash_before == template_hash_after)
    gates = {
        "target_fulfilled": target_fulfilled,
        "anchor": anchor,
        "scale": scale,
        "non_target_preserved": non_target_preserved,
        "geometry_locality": geometry_locality,
        "structural_locality": structural_locality,
        "post_edit_constraint_pass": edit_run.get("verdict") == "pass",
        "regression_preservation": regression,
    }
    gates["final_pass"] = all(gates.values())
    return {
        "task_id": mapping["task_id"],
        "seed": preflight["seed"],
        "main_asset": preflight["seed"] == MAIN_SEED,
        "gates": gates,
        "changed_artifact_links": sorted(changed_links),
        "changed_artifact_joints": sorted(changed_joints),
        "unexpected_changed_links": sorted(changed_links - allowed_links),
        "unexpected_changed_joints": sorted(changed_joints - allowed_joints),
        "compiled_predicates": compiled_predicates,
        "parent_compile": parent_run,
        "edited_compile": edit_run,
        "preflight": preflight,
        "cost": {
            "api_tokens": 0,
            "api_cost": 0.0,
            "parent_wall_time_s": parent_run.get("elapsed_s"),
            "edited_wall_time_s": edit_run.get("elapsed_s"),
            "operation": "frozen PV-A dataclass field/component edit",
        },
    }


def run_preflight(mapping: dict[str, Any], output: Path, *, all_seeds: bool) -> dict[str, Any]:
    seeds = FROZEN_SEEDS if all_seeds else (MAIN_SEED,)
    rows = []
    for task in mapping["tasks"]:
        for seed in seeds:
            case_root = output / "preflight" / task["task_id"] / f"seed_{seed:02d}"
            row = preflight_one(task, seed, case_root, build_models=all_seeds or seed == MAIN_SEED)
            dump_json(case_root / "preflight.json", row)
            rows.append(row)
    summary = {
        "protocol_id": PROTOCOL_ID,
        "source_manifest_sha256": mapping["source_manifest_sha256"],
        "source_scorer_sha256": mapping["source_scorer_sha256"],
        "source_prompt_contract_sha256": mapping["source_prompt_contract_sha256"],
        "source_budget_guard_sha256": mapping["source_budget_guard_sha256"],
        "mapping_freeze_state": mapping.get("freeze_state"),
        "task_count": len(mapping["tasks"]),
        "seed_count": len(seeds),
        "case_count": len(rows),
        "preflight_pass_count": sum(row["preflight_pass"] for row in rows),
        "all_pass": all(row["preflight_pass"] for row in rows),
        "failed_cases": [
            {"task_id": row["task_id"], "seed": row["seed"]}
            for row in rows
            if not row["preflight_pass"]
        ],
    }
    dump_json(output / "preflight/summary.json", summary)
    return {"rows": rows, "summary": summary}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--run", action="store_true")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--expect-manifest-sha256")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=900.0)
    args = parser.parse_args()

    manifest_path = contained(args.manifest)
    mapping_path = contained(args.mapping)
    output = contained(args.out, must_exist=False)
    output.relative_to(EXP_ROOT.resolve(strict=True))
    output.mkdir(parents=True, exist_ok=True)
    manifest_sha = sha256(manifest_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    validate_inputs(manifest, mapping, manifest_sha)

    if args.run and args.expect_manifest_sha256 != manifest_sha:
        raise RuntimeError(
            "formal run requires --expect-manifest-sha256 equal to the current frozen manifest"
        )
    if args.run:
        if mapping.get("freeze_state") != "final":
            raise RuntimeError("formal run requires mapping freeze_state=final")
        if manifest.get("selection_frozen_before_new_runs") is not True:
            raise RuntimeError("formal run requires selection_frozen_before_new_runs=true")
        if not manifest.get("frozen_at_utc"):
            raise RuntimeError("formal run requires a non-empty frozen_at_utc")
    preflight = run_preflight(mapping, output, all_seeds=args.run)
    if not preflight["summary"]["all_pass"]:
        raise RuntimeError("PV-A preflight failed; formal compilation is fail-closed")
    if args.preflight:
        print(json.dumps(preflight["summary"], indent=2, sort_keys=True))
        return 0

    frozen = {
        "protocol_id": PROTOCOL_ID,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "source_manifest": str(manifest_path),
        "source_manifest_sha256": manifest_sha,
        "mapping": str(mapping_path),
        "mapping_sha256": sha256(mapping_path),
        "main_seed": MAIN_SEED,
        "seeds": list(FROZEN_SEEDS),
        "task_count": 18,
        "case_count": 18 * 16,
        "phase_order": ["parent", "edited"],
        "workers": args.workers,
        "per_case_timeout_s": args.timeout,
        "child_thread_env": {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        },
    }
    dump_json(output / "frozen_run_manifest.json", frozen)

    parent_jobs: list[tuple[Path, Path]] = []
    edited_jobs: list[tuple[Path, Path]] = []
    paths: dict[tuple[str, int], tuple[Path, Path]] = {}
    hashes_before: dict[str, str] = {}
    for task in mapping["tasks"]:
        template = contained(TEMPLATE_ROOT / "agent/templates" / f"{task['slug']}.py")
        hashes_before[task["task_id"]] = sha256(template)
        for seed in FROZEN_SEEDS:
            case_root = output / "cases" / task["task_id"] / f"seed_{seed:02d}"
            parent_wrapper = case_root / "parent/source.py"
            edit_wrapper = case_root / "edited/source.py"
            shared = dict(task.get("shared_overrides") or {})
            write_wrapper(
                parent_wrapper,
                task,
                seed,
                {**shared, **task["parent_overrides"]},
            )
            write_wrapper(
                edit_wrapper,
                task,
                seed,
                {**shared, **task["edited_overrides"]},
            )
            parent_jobs.append((parent_wrapper, parent_wrapper.parent / "compile_result.json"))
            edited_jobs.append((edit_wrapper, edit_wrapper.parent / "compile_result.json"))
            paths[(task["task_id"], seed)] = (parent_wrapper, edit_wrapper)

    parent_results = compile_phase(parent_jobs, args.workers, args.timeout, "parent")
    parent_finished = datetime.now(timezone.utc).isoformat()
    edit_results = compile_phase(edited_jobs, args.workers, args.timeout, "edited")
    edited_finished = datetime.now(timezone.utc).isoformat()

    records = []
    for task in mapping["tasks"]:
        template = contained(TEMPLATE_ROOT / "agent/templates" / f"{task['slug']}.py")
        after = sha256(template)
        for seed in FROZEN_SEEDS:
            parent_wrapper, edit_wrapper = paths[(task["task_id"], seed)]
            preflight_path = output / "preflight" / task["task_id"] / f"seed_{seed:02d}/preflight.json"
            row = score_compiled_case(
                json.loads(preflight_path.read_text(encoding="utf-8")),
                task,
                parent_results[str(parent_wrapper)],
                edit_results[str(edit_wrapper)],
                hashes_before[task["task_id"]],
                after,
            )
            record_path = output / "records" / task["task_id"] / f"seed_{seed:02d}.json"
            dump_json(record_path, row)
            records.append(row)

    gate_names = tuple(records[0]["gates"])
    task_results = []
    for task in mapping["tasks"]:
        subset = [row for row in records if row["task_id"] == task["task_id"]]
        task_results.append(
            {
                "task_id": task["task_id"],
                "main_seed_pass": next(
                    row["gates"]["final_pass"] for row in subset if row["seed"] == MAIN_SEED
                ),
                "final_pass_count": sum(row["gates"]["final_pass"] for row in subset),
                "propagation_16_of_16": all(row["gates"]["final_pass"] for row in subset),
            }
        )
    edit_times = [
        row["cost"]["edited_wall_time_s"]
        for row in records
        if isinstance(row["cost"]["edited_wall_time_s"], (int, float))
    ]
    summary = {
        **frozen,
        "parent_phase_finished_at": parent_finished,
        "edited_phase_finished_at": edited_finished,
        "phase_order_valid": frozen["started_at"] <= parent_finished <= edited_finished,
        "main_asset": {
            "pass": sum(row["main_seed_pass"] for row in task_results),
            "total": len(task_results),
        },
        "propagation_16_of_16": {
            "pass": sum(row["propagation_16_of_16"] for row in task_results),
            "total": len(task_results),
        },
        "gates": {
            gate: {
                "pass": sum(row["gates"][gate] for row in records),
                "total": len(records),
                "rate": mean(row["gates"][gate] for row in records),
            }
            for gate in gate_names
        },
        "task_results": task_results,
        "edit_cost": {
            "api_tokens": 0,
            "api_cost": 0.0,
            "mean_wall_time_s": mean(edit_times) if edit_times else None,
            "median_wall_time_s": median(edit_times) if edit_times else None,
        },
    }
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--worker":
        raise SystemExit(local.worker_main(Path(sys.argv[2]), Path(sys.argv[3])))
    raise SystemExit(main())
