#!/usr/bin/env python3
"""Freeze source-program joint gold and score the 33 exported URDFs against it.

The exported ``model.py`` wrappers encode the original seed and call the checked-in
procedural template.  This harness rebuilds each wrapper in an exp-local copy, freezes
the SDK articulation declarations before URDF parsing, and verifies that the existing
frozen URDF preserves those declarations.  It does not infer semantics from the URDF
under test and refuses to hide source/URDF mismatches.
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import hashlib
import io
import json
import math
import runpy
import shutil
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_INPUT = EXP_ROOT / "runtime/nano3d_glb_n33/input_packages/input_manifest.json"
DEFAULT_GLB_RECORDS = EXP_ROOT / "runtime/nano3d_glb_n33/output/records.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/source_program_gold"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def vec(node: ET.Element | None, attr: str, default: tuple[float, float, float]) -> list[float]:
    if node is None or attr not in node.attrib:
        return list(default)
    values = [float(value) for value in node.attrib[attr].split()]
    if len(values) != 3:
        raise ValueError(f"expected vec3 for {attr}: {values}")
    return values


def parse_urdf_joints(path: Path) -> list[dict[str, Any]]:
    root = ET.parse(path).getroot()
    rows: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        if node.attrib.get("type") == "fixed":
            continue
        origin = node.find("origin")
        axis = node.find("axis")
        limit = node.find("limit")
        parent = node.find("parent")
        child = node.find("child")
        rows.append(
            {
                "name": node.attrib["name"],
                "type": node.attrib["type"],
                "parent": parent.attrib["link"] if parent is not None else None,
                "child": child.attrib["link"] if child is not None else None,
                "axis": vec(axis, "xyz", (0.0, 0.0, 0.0)),
                "origin_xyz": vec(origin, "xyz", (0.0, 0.0, 0.0)),
                "origin_rpy": vec(origin, "rpy", (0.0, 0.0, 0.0)),
                "lower": float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
                "upper": float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
            }
        )
    return rows


def normalize(values: list[float] | tuple[float, float, float]) -> list[float]:
    magnitude = math.sqrt(sum(float(value) ** 2 for value in values))
    return [float(value) / magnitude for value in values] if magnitude > 1e-12 else list(values)


def expected_joints(object_model: Any) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for joint in object_model.articulations:
        limits = joint.motion_limits
        joint_type = getattr(joint.articulation_type, "value", str(joint.articulation_type))
        if joint_type == "fixed":
            continue
        rows.append(
            {
                "name": joint.name,
                "type": joint_type,
                "parent": joint.parent,
                "child": joint.child,
                "axis": normalize(joint.axis) if joint_type != "fixed" else [0.0, 0.0, 0.0],
                "origin_xyz": [float(value) for value in joint.origin.xyz],
                "origin_rpy": [float(value) for value in joint.origin.rpy],
                "lower": float(limits.lower) if limits is not None and limits.lower is not None else None,
                "upper": float(limits.upper) if limits is not None and limits.upper is not None else None,
            }
        )
    return rows


def imported_template_path(wrapper: Path, template_root: Path) -> Path:
    tree = ast.parse(wrapper.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Import):
            for alias in node.names:
                prefix = "agent.templates."
                if alias.name.startswith(prefix):
                    return template_root / "agent/templates" / f"{alias.name[len(prefix):]}.py"
        if isinstance(node, ast.ImportFrom) and node.module:
            prefix = "agent.templates."
            if node.module.startswith(prefix):
                return template_root / "agent/templates" / f"{node.module[len(prefix):]}.py"
    raise ValueError(f"no template import in {wrapper}")


def diagonal(bounds: list[list[float]]) -> float:
    return math.sqrt(sum((float(high) - float(low)) ** 2 for low, high in zip(bounds[0], bounds[1])))


def axis_error_degrees(expected: list[float], actual: list[float]) -> float:
    left = normalize(expected)
    right = normalize(actual)
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(left, right))))
    return math.degrees(math.acos(dot))


def euclidean(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def limit_error(expected: dict[str, Any], actual: dict[str, Any]) -> float | None:
    if expected["type"] not in {"revolute", "prismatic"}:
        return None
    if None in (expected["lower"], expected["upper"], actual["lower"], actual["upper"]):
        return None
    span = max(abs(expected["upper"] - expected["lower"]), 1e-12)
    return (abs(actual["lower"] - expected["lower"]) + abs(actual["upper"] - expected["upper"])) / (2.0 * span)


def copy_frozen_package(source: Path, target: Path) -> None:
    if target.exists():
        for name in ("model.py", "model.urdf"):
            if sha256(source / name) != sha256(target / name):
                raise RuntimeError(f"resume refused; working copy differs: {target / name}")
        return
    shutil.copytree(source, target)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def run_asset(
    row: dict[str, Any], glb_row: dict[str, Any], work_root: Path, template_root: Path
) -> dict[str, Any]:
    asset_id = row["asset_id"]
    source = Path(row["copied_package"])
    work = work_root / asset_id
    copy_frozen_package(source, work)
    wrapper = work / "model.py"
    template = imported_template_path(wrapper, template_root)
    if not template.is_file():
        raise FileNotFoundError(template)
    namespace: dict[str, Any] | None = None
    error: str | None = None
    captured = io.StringIO()
    try:
        with contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
            namespace = runpy.run_path(str(wrapper))
    except Exception as exc:  # noqa: BLE001
        error = f"{type(exc).__name__}: {exc}"
    if namespace is None:
        return {
            "asset_id": asset_id,
            "rebuild_success": False,
            "rebuild_error": error,
            "captured_output": captured.getvalue()[-4000:],
            "wrapper_sha256": sha256(wrapper),
            "template_path": display_path(template),
            "template_sha256": sha256(template),
            "frozen_urdf_sha256": sha256(source / "model.urdf"),
        }

    expected = expected_joints(namespace["object_model"])
    actual = parse_urdf_joints(source / "model.urdf")
    expected_by_name = {joint["name"]: joint for joint in expected}
    actual_by_name = {joint["name"]: joint for joint in actual}
    bbox_diagonal = diagonal(glb_row["urdf_scene_bounds"])
    comparisons: list[dict[str, Any]] = []
    for name in sorted(set(expected_by_name) | set(actual_by_name)):
        gold = expected_by_name.get(name)
        observed = actual_by_name.get(name)
        if gold is None or observed is None:
            comparisons.append(
                {"name": name, "expected_present": gold is not None, "actual_present": observed is not None}
            )
            continue
        comparisons.append(
            {
                "name": name,
                "expected_present": True,
                "actual_present": True,
                "type_correct": observed["type"] == gold["type"],
                "parent_child_correct": (observed["parent"], observed["child"]) == (gold["parent"], gold["child"]),
                "axis_error_degrees": axis_error_degrees(gold["axis"], observed["axis"]),
                "origin_error_bbox_normalized": euclidean(gold["origin_xyz"], observed["origin_xyz"]) / max(bbox_diagonal, 1e-12),
                "origin_rpy_error_radians": euclidean(gold["origin_rpy"], observed["origin_rpy"]),
                "limit_error_range_normalized": limit_error(gold, observed),
                "expected": gold,
                "actual": observed,
            }
        )
    matched = [item for item in comparisons if item.get("expected_present") and item.get("actual_present")]
    return {
        "asset_id": asset_id,
        "rebuild_success": True,
        "rebuild_error": None,
        "captured_output": captured.getvalue()[-4000:],
        "wrapper_sha256": sha256(wrapper),
        "template_path": display_path(template),
        "template_sha256": sha256(template),
        "frozen_urdf_sha256": sha256(source / "model.urdf"),
        "bbox_diagonal": bbox_diagonal,
        "expected_joint_count": len(expected),
        "actual_joint_count": len(actual),
        "matched_joint_count": len(matched),
        "expected_joints": expected,
        "comparisons": comparisons,
    }


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    rebuilt = [record for record in records if record["rebuild_success"]]
    comparisons = [item for record in rebuilt for item in record["comparisons"]]
    expected_total = sum(record["expected_joint_count"] for record in rebuilt)
    actual_total = sum(record["actual_joint_count"] for record in rebuilt)
    matched = [item for item in comparisons if item.get("expected_present") and item.get("actual_present")]
    axis_errors = [item["axis_error_degrees"] for item in matched if "axis_error_degrees" in item]
    origin_errors = [item["origin_error_bbox_normalized"] for item in matched if "origin_error_bbox_normalized" in item]
    limit_errors = [item["limit_error_range_normalized"] for item in matched if item.get("limit_error_range_normalized") is not None]
    exact_assets = sum(
        record["expected_joint_count"] == record["actual_joint_count"] == record["matched_joint_count"]
        and all(item.get("type_correct") and item.get("parent_child_correct") for item in record["comparisons"])
        for record in rebuilt
    )
    return {
        "schema_version": 1,
        "protocol": "t5_source_program_joint_gold_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "gold_definition": (
            "SDK articulation declarations rebuilt from each frozen seed wrapper and the recorded current template source; "
            "the existing URDF is the independently parsed output under test."
        ),
        "asset_count": len(records),
        "rebuild_success": sum(record["rebuild_success"] for record in records),
        "exact_asset_joint_graphs": exact_assets,
        "expected_joint_count": expected_total,
        "actual_joint_count": actual_total,
        "matched_joint_count": len(matched),
        "joint_recall": len(matched) / expected_total if expected_total else None,
        "joint_type_accuracy": sum(item.get("type_correct", False) for item in matched) / len(matched) if matched else None,
        "parent_child_accuracy": sum(item.get("parent_child_correct", False) for item in matched) / len(matched) if matched else None,
        "axis_error_degrees_mean": mean(axis_errors) if axis_errors else None,
        "axis_error_degrees_max": max(axis_errors) if axis_errors else None,
        "origin_error_bbox_normalized_mean": mean(origin_errors) if origin_errors else None,
        "origin_error_bbox_normalized_max": max(origin_errors) if origin_errors else None,
        "limit_error_range_normalized_mean": mean(limit_errors) if limit_errors else None,
        "limit_error_range_normalized_max": max(limit_errors) if limit_errors else None,
        "limitations": [
            "This evaluates compiler/export preservation against source-program intent, not human judgment of whether the source program chose the semantically ideal joint.",
            "A template source hash is frozen per asset; any source/URDF mismatch remains visible and is not imputed.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--glb-records", type=Path, default=DEFAULT_GLB_RECORDS)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--template-root", type=Path, default=TEMPLATE_ROOT)
    parser.add_argument("--asset-id", action="append", help="restrict to an asset id; repeatable")
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    template_root = args.template_root.resolve()
    sys.path.insert(0, str(template_root))
    inputs = json.loads(args.input_manifest.read_text(encoding="utf-8"))
    requested = set(args.asset_id or [])
    if requested:
        available = {row["asset_id"] for row in inputs}
        unknown = requested - available
        if unknown:
            raise RuntimeError(f"unknown asset ids: {sorted(unknown)}")
        inputs = [row for row in inputs if row["asset_id"] in requested]
    glb_rows = json.loads(args.glb_records.read_text(encoding="utf-8"))
    glb_by_id = {row["asset_id"]: row for row in glb_rows}
    if not {row["asset_id"] for row in inputs}.issubset(glb_by_id):
        raise RuntimeError("input assets are missing from GLB records")
    records = [
        run_asset(row, glb_by_id[row["asset_id"]], output / "work_packages", template_root)
        for row in inputs
    ]
    dump_json(output / "asset_records.json", records)
    summary = summarize(records)
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["rebuild_success"] == summary["asset_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
