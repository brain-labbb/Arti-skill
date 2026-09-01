#!/usr/bin/env python3
"""Run Nano3D low/medium-difficulty checks inside the exp directory.

This harness never edits source assets.  It reuses the existing pilot parser,
scans sibling seeds only under the selected asset's original source root, and
performs representation-level articulation checks.  It deliberately does not
claim a physics sweep when no physics engine is available.
"""

from __future__ import annotations

import itertools
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from contextlib import suppress
from pathlib import Path
from statistics import mean, median
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_nano3d_asset_pilot import (  # noqa: E402
    DEFAULT_OUTPUT,
    asset_identity,
    parse_urdf,
    read_asset_paths,
    static_record,
)


OUTPUT = DEFAULT_OUTPUT.parent / "nano3d_low_medium"


def optional_physics_imports() -> tuple[Any | None, Any | None, str | None]:
    """Load pybullet and urdfpy with a local NumPy compatibility shim."""
    try:
        import numpy as np

        # urdfpy 0.0.22 predates NumPy 1.24 aliases.  The aliases are scoped
        # to this process and do not modify the installed package or source.
        for name, value in {"float": float, "int": int}.items():
            if not hasattr(np, name):
                setattr(np, name, value)
        from urdfpy import URDF
    except Exception as exc:  # noqa: BLE001
        return None, None, f"urdfpy_unavailable: {type(exc).__name__}: {exc}"
    try:
        import pybullet
    except Exception as exc:  # noqa: BLE001
        return URDF, None, f"pybullet_unavailable: {type(exc).__name__}: {exc}"
    return URDF, pybullet, None


URDF_CLASS, PYBULLET, PHYSICS_IMPORT_ERROR = optional_physics_imports()


def normalized_names(urdf_path: Path) -> tuple[str, ...]:
    root = ET.parse(urdf_path).getroot()
    names = [node.attrib.get("name", "") for node in root.findall("link")]
    names.extend(node.attrib.get("name", "") for node in root.findall("joint"))
    return tuple(sorted(name.strip().lower() for name in names if name.strip()))


def joint_type_signature(urdf_path: Path) -> tuple[tuple[str, int], ...]:
    root = ET.parse(urdf_path).getroot()
    counts = Counter(node.attrib.get("type", "") for node in root.findall("joint"))
    return tuple(sorted(counts.items()))


def mode_rate(values: list[Any]) -> float | None:
    if not values:
        return None
    return Counter(values).most_common(1)[0][1] / len(values)


def jaccard(left: set[str], right: set[str]) -> float:
    union = left | right
    return len(left & right) / len(union) if union else 1.0


def sibling_cohort(asset_dir: Path) -> list[Path]:
    """Return all parseable sibling seeds under the asset's own source root."""
    source_root = asset_dir.parent.parent
    slug_dir = source_root / asset_dir.parent.name
    if not slug_dir.is_dir():
        return [asset_dir]
    return sorted(
        seed_dir
        for seed_dir in slug_dir.iterdir()
        if seed_dir.is_dir()
        and seed_dir.name.startswith("seed_")
        and (seed_dir / "model.urdf").is_file()
    )


def cross_seed_record(selected_asset: Path) -> dict[str, Any]:
    cohort = sibling_cohort(selected_asset)
    rows: list[dict[str, Any]] = []
    name_sets: list[set[str]] = []
    name_signatures: list[tuple[str, ...]] = []
    link_counts: list[int] = []
    joint_counts: list[int] = []
    joint_types: list[tuple[tuple[str, int], ...]] = []
    for asset_dir in cohort:
        urdf_path = asset_dir / "model.urdf"
        identity = asset_identity(asset_dir)
        try:
            parsed = parse_urdf(asset_dir, urdf_path)
            signature = normalized_names(urdf_path)
            type_signature = joint_type_signature(urdf_path)
            name_set = set(signature)
            error = None
        except Exception as exc:  # noqa: BLE001
            parsed = {"valid_urdf": False, "links": 0, "joints": 0}
            signature = ()
            type_signature = ()
            name_set = set()
            error = f"{type(exc).__name__}: {exc}"
        row = {
            **identity,
            "valid_urdf": bool(parsed.get("valid_urdf")),
            "links": int(parsed.get("links", 0) or 0),
            "joints": int(parsed.get("joints", 0) or 0),
            "raw_name_coverage": parsed.get("raw_name_coverage"),
            "name_signature": list(signature),
            "joint_type_signature": [list(item) for item in type_signature],
            "error": error,
        }
        rows.append(row)
        if row["valid_urdf"]:
            name_sets.append(name_set)
            name_signatures.append(signature)
            link_counts.append(row["links"])
            joint_counts.append(row["joints"])
            joint_types.append(type_signature)

    pairwise = [jaccard(left, right) for left, right in itertools.combinations(name_sets, 2)]
    exact_mode_rate = mode_rate(name_signatures)
    return {
        "slug": selected_asset.parent.name,
        "selected_asset_id": asset_identity(selected_asset)["asset_id"],
        "source": asset_identity(selected_asset)["source"],
        "selected_seed": asset_identity(selected_asset)["seed"],
        "cohort_size": len(cohort),
        "valid_urdf_count": sum(row["valid_urdf"] for row in rows),
        "valid_urdf_rate": sum(row["valid_urdf"] for row in rows) / len(rows) if rows else None,
        "name_signature_mode_rate": exact_mode_rate,
        "pairwise_name_jaccard_mean": mean(pairwise) if pairwise else None,
        "pairwise_name_jaccard_median": median(pairwise) if pairwise else None,
        "link_count_mode_rate": mode_rate(link_counts),
        "joint_count_mode_rate": mode_rate(joint_counts),
        "joint_type_signature_mode_rate": mode_rate(joint_types),
        "raw_name_coverage_mean": mean(
            row["raw_name_coverage"] for row in rows if isinstance(row.get("raw_name_coverage"), (int, float))
        )
        if any(isinstance(row.get("raw_name_coverage"), (int, float)) for row in rows)
        else None,
        "seed_records": rows,
    }


def finite_vector(text: str | None, expected: int = 3) -> bool:
    if not text:
        return True
    try:
        values = [float(value) for value in text.split()]
    except ValueError:
        return False
    return len(values) == expected and all(math.isfinite(value) for value in values)


def vector_norm(text: str | None) -> float | None:
    if not text:
        return None
    try:
        values = [float(value) for value in text.split()]
    except ValueError:
        return None
    return math.sqrt(sum(value * value for value in values)) if values else None


def articulation_smoke_record(asset_dir: Path) -> dict[str, Any]:
    identity = asset_identity(asset_dir)
    result: dict[str, Any] = {
        **identity,
        "physics_engine_available": PYBULLET is not None,
        "urdfpy_available": URDF_CLASS is not None,
        "urdfpy_load_success": False,
        "pybullet_load_success": False,
        "physics_sweep": "not_run",
        "rest_step_success": False,
        "boundary_state_count": 0,
        "boundary_step_success": 0,
        "rest_self_contact_points": None,
        "boundary_self_contact_points": [],
        "valid_urdf": False,
        "movable_joints": 0,
        "joint_axis_present": 0,
        "joint_axis_nonzero_finite": 0,
        "joint_origin_finite": 0,
        "bounded_joint_count": 0,
        "valid_joint_bounds": 0,
        "visual_links": 0,
        "collision_links": 0,
        "rest_pose_parse_ready": False,
        "boundary_pose_parse_ready": False,
        "representation_smoke_pass": False,
        "issues": [],
    }
    try:
        root = ET.parse(asset_dir / "model.urdf").getroot()
        result["valid_urdf"] = True
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"urdf_parse: {type(exc).__name__}: {exc}")
        return result

    urdf_path = asset_dir / "model.urdf"
    if URDF_CLASS is not None:
        try:
            URDF_CLASS.load(str(urdf_path))
            result["urdfpy_load_success"] = True
        except Exception as exc:  # noqa: BLE001
            result["issues"].append(f"urdfpy_load: {type(exc).__name__}: {exc}")

    for link in root.findall("link"):
        if link.find("visual") is not None:
            result["visual_links"] += 1
        if link.find("collision") is not None:
            result["collision_links"] += 1

    movable = [joint for joint in root.findall("joint") if joint.attrib.get("type") != "fixed"]
    result["movable_joints"] = len(movable)
    axis_ok = True
    origin_ok = True
    bounds_ok = True
    for joint in movable:
        axis = joint.find("axis")
        if axis is not None and axis.attrib.get("xyz"):
            result["joint_axis_present"] += 1
            norm = vector_norm(axis.attrib.get("xyz"))
            if finite_vector(axis.attrib.get("xyz")) and norm is not None and norm > 1e-9:
                result["joint_axis_nonzero_finite"] += 1
            else:
                axis_ok = False
        else:
            axis_ok = False
        origin = joint.find("origin")
        if origin is None or (finite_vector(origin.attrib.get("xyz")) and finite_vector(origin.attrib.get("rpy"))):
            result["joint_origin_finite"] += 1
        else:
            origin_ok = False
        if joint.attrib.get("type") in {"revolute", "prismatic"}:
            result["bounded_joint_count"] += 1
            limit = joint.find("limit")
            try:
                lower = float(limit.attrib["lower"]) if limit is not None else math.nan
                upper = float(limit.attrib["upper"]) if limit is not None else math.nan
                valid = math.isfinite(lower) and math.isfinite(upper) and lower <= upper
            except (AttributeError, KeyError, TypeError, ValueError):
                valid = False
            if valid:
                result["valid_joint_bounds"] += 1
            else:
                bounds_ok = False
    if not axis_ok:
        result["issues"].append("one or more movable joints lack a finite nonzero axis")
    if not origin_ok:
        result["issues"].append("one or more joint origins are not finite")
    if not bounds_ok:
        result["issues"].append("one or more bounded joints lack finite ordered limits")
    result["rest_pose_parse_ready"] = result["valid_urdf"] and axis_ok and origin_ok
    result["boundary_pose_parse_ready"] = result["rest_pose_parse_ready"] and bounds_ok
    if PYBULLET is not None and result["boundary_pose_parse_ready"]:
        client = PYBULLET.connect(PYBULLET.DIRECT)
        body = None
        try:
            body = PYBULLET.loadURDF(
                str(urdf_path),
                useFixedBase=True,
                flags=PYBULLET.URDF_USE_INERTIA_FROM_FILE,
            )
            result["pybullet_load_success"] = True
            joint_state = {}
            movable_indices = []
            boundary_states: list[tuple[int, float]] = []
            continuous_type = getattr(PYBULLET, "JOINT_CONTINUOUS", None)
            for index in range(PYBULLET.getNumJoints(body)):
                info = PYBULLET.getJointInfo(body, index)
                joint_type = info[2]
                if joint_type in {PYBULLET.JOINT_REVOLUTE, PYBULLET.JOINT_PRISMATIC}:
                    lower, upper = float(info[8]), float(info[9])
                    value = (lower + upper) / 2.0 if lower <= upper else 0.0
                    joint_state[index] = value
                    movable_indices.append(index)
                    if math.isfinite(lower) and math.isfinite(upper) and lower <= upper:
                        boundary_states.extend([(index, lower), (index, upper)])
                elif continuous_type is not None and joint_type == continuous_type:
                    joint_state[index] = 0.0
                    movable_indices.append(index)
                    boundary_states.extend([(index, -math.pi), (index, math.pi)])
            for index, value in joint_state.items():
                PYBULLET.resetJointState(body, index, value)
            for _ in range(10):
                PYBULLET.stepSimulation()
            result["rest_step_success"] = True
            result["rest_self_contact_points"] = len(PYBULLET.getContactPoints(bodyA=body, bodyB=body))
            for index, value in boundary_states:
                for reset_index, reset_value in joint_state.items():
                    PYBULLET.resetJointState(body, reset_index, reset_value)
                PYBULLET.resetJointState(body, index, value)
                for _ in range(5):
                    PYBULLET.stepSimulation()
                result["boundary_state_count"] += 1
                result["boundary_step_success"] += 1
                result["boundary_self_contact_points"].append(
                    {
                        "joint_index": index,
                        "value": value,
                        "self_contact_points": len(PYBULLET.getContactPoints(bodyA=body, bodyB=body)),
                    }
                )
            result["physics_sweep"] = "rest_and_individual_boundary_smoke"
        except Exception as exc:  # noqa: BLE001
            result["physics_sweep"] = "failed_during_load_or_step"
            result["issues"].append(f"pybullet_smoke: {type(exc).__name__}: {exc}")
        finally:
            if body is not None:
                with suppress(Exception):
                    PYBULLET.removeBody(body)
            with suppress(Exception):
                PYBULLET.disconnect(client)
    elif PHYSICS_IMPORT_ERROR:
        result["physics_sweep"] = f"not_run: {PHYSICS_IMPORT_ERROR}"
    result["representation_smoke_pass"] = result["boundary_pose_parse_ready"]
    return result


def main() -> int:
    output = OUTPUT.resolve()
    output.mkdir(parents=True, exist_ok=True)
    selected = read_asset_paths()

    # Low-level results are already produced by the existing pilot; verify and
    # carry its static records into this run without changing source assets.
    pilot_output = DEFAULT_OUTPUT
    static_path = pilot_output / "static_records.json"
    summary_path = pilot_output / "summary.json"
    low_summary: dict[str, Any] = {}
    if static_path.exists() and summary_path.exists():
        low_summary = {
            "source": str(pilot_output),
            "summary": json.loads(summary_path.read_text(encoding="utf-8")),
            "static_records_available": len(json.loads(static_path.read_text(encoding="utf-8"))),
        }
    else:
        low_summary = {"source": str(pilot_output), "summary": None, "static_records_available": 0}

    cross_seed = [cross_seed_record(path) for path in selected]
    articulation = [articulation_smoke_record(path) for path in selected]
    static = [static_record(path) for path in selected]

    multi = [row for row in cross_seed if row["cohort_size"] > 1]
    summary = {
        "protocol": "nano3d_low_medium_v1",
        "selected_asset_count": len(selected),
        "low_pilot_summary": low_summary,
        "cross_seed": {
            "cohort_count": len(cross_seed),
            "multi_seed_cohort_count": len(multi),
            "single_seed_cohort_count": len(cross_seed) - len(multi),
            "total_seed_records": sum(row["cohort_size"] for row in cross_seed),
            "valid_urdf_rate_over_seed_records": (
                sum(row["valid_urdf_count"] for row in cross_seed)
                / sum(row["cohort_size"] for row in cross_seed)
            ),
            "name_signature_mode_rate_mean_multi_seed": mean(
                row["name_signature_mode_rate"] for row in multi if row["name_signature_mode_rate"] is not None
            )
            if multi
            else None,
            "pairwise_name_jaccard_mean_multi_seed": mean(
                row["pairwise_name_jaccard_mean"]
                for row in multi
                if row["pairwise_name_jaccard_mean"] is not None
            )
            if any(row["pairwise_name_jaccard_mean"] is not None for row in multi)
            else None,
            "link_count_mode_rate_mean_multi_seed": mean(
                row["link_count_mode_rate"] for row in multi if row["link_count_mode_rate"] is not None
            )
            if multi
            else None,
            "joint_count_mode_rate_mean_multi_seed": mean(
                row["joint_count_mode_rate"] for row in multi if row["joint_count_mode_rate"] is not None
            )
            if multi
            else None,
            "single_seed_note": "single-seed cohorts are not used to claim cross-seed consistency",
        },
        "articulation_smoke": {
            "asset_count": len(articulation),
            "representation_smoke_pass": sum(row["representation_smoke_pass"] for row in articulation),
            "rest_pose_parse_ready": sum(row["rest_pose_parse_ready"] for row in articulation),
            "boundary_pose_parse_ready": sum(row["boundary_pose_parse_ready"] for row in articulation),
            "physics_engine_available": PYBULLET is not None,
            "urdfpy_available": URDF_CLASS is not None,
            "urdfpy_load_success": sum(row["urdfpy_load_success"] for row in articulation),
            "pybullet_load_success": sum(row["pybullet_load_success"] for row in articulation),
            "rest_step_success": sum(row["rest_step_success"] for row in articulation),
            "boundary_state_count": sum(row["boundary_state_count"] for row in articulation),
            "boundary_step_success": sum(row["boundary_step_success"] for row in articulation),
            "physics_sweep": "rest_and_individual_boundary_smoke",
            "collision_free_claim": "unsupported; contact points are diagnostic only",
        },
        "mesh_static": {
            "asset_count": len(static),
            "mesh_readable_files": sum(int(row.get("readable_mesh_files", 0)) for row in static),
            "watertight_files": sum(int(row.get("watertight_mesh_files", 0)) for row in static),
            "winding_consistent_files": sum(int(row.get("winding_consistent_mesh_files", 0)) for row in static),
            "open_edges": sum(int(row.get("open_edges", 0)) for row in static),
            "degenerate_faces": sum(int(row.get("degenerate_faces", 0)) for row in static),
            "mesh_errors": sum(len(row.get("mesh_errors", [])) for row in static),
        },
    }
    (output / "cross_seed_records.json").write_text(
        json.dumps(cross_seed, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (output / "articulation_smoke_records.json").write_text(
        json.dumps(articulation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (output / "low_medium_static_records.json").write_text(
        json.dumps(static, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"outputs={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
