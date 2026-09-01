#!/usr/bin/env python3
"""Run the frozen URDF Sim-Ready Table 4 protocol on PartNet-Mobility.

This evaluator measures discrete self-collision and mechanical clearance. It
does not run continuous collision detection, infer semantic joint correctness,
or estimate exact overlap volume. Selected package, load, child-process, and
timeout failures remain in the intent-to-evaluate denominators.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
DEFAULT_DATASET = REPO / "exp/PartNet-Mobility/data/dataset"
DEFAULT_ARCHIVE = REPO / "exp/PartNet-Mobility/partnet-mobility-v0.zip"
DEFAULT_OUTPUT = REPO / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813"

PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
QUALIFICATION_PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_qualification_v1"
SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
SAMPLE_SIZE = 800
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
ZERO_WIDTH_TOLERANCE = 1e-12
# Table 4 v2 samples the independent configuration space.  The historical v1
# protocol is kept available for validating old receipts, but new runs should
# bind this version explicitly in their job identity.
SAMPLING_PROTOCOL_V1 = "independent_sampling_v1"
SAMPLING_PROTOCOL_V2 = "mimic_aware_independent_sampling_v2"
EXPECTED_RELEASE_ASSETS = 2347
EXPECTED_ARCHIVE_SHA256 = (
    "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
)
EXPECTED_CANDIDATE_POOL_SHA256 = (
    "0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d"
)
EXPECTED_SELECTED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
CORE_FILES = (
    "meta.json",
    "mobility.urdf",
    "mobility_v2.json",
    "semantics.txt",
    "result.json",
    "bounding_box.json",
)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(canonical_bytes(value))


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(
                json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n"
            )
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def penetration_is_illegal(contact_distance_m: float) -> bool:
    return max(0.0, -float(contact_distance_m)) > PENETRATION_THRESHOLD_M


def normalize_executable_path(path: Path, cwd: Path) -> Path:
    candidate = path if path.is_absolute() else cwd / path
    absolute = Path(os.path.abspath(candidate))
    if not absolute.is_file():
        raise FileNotFoundError(f"executable path does not exist: {absolute}")
    return absolute


def selection_digest(dataset_id: str, salt: str = SELECTION_SALT) -> str:
    return sha256_bytes(f"{salt}\0{dataset_id}".encode("utf-8"))


def select_candidates(
    candidates: list[dict[str, Any]], sample_size: int, salt: str
) -> list[dict[str, Any]]:
    if sample_size <= 0:
        raise ValueError("sample_size must be positive")
    identities = [str(row["dataset_id"]) for row in candidates]
    if len(identities) != len(set(identities)):
        raise ValueError("candidate dataset IDs are not unique")
    if sample_size > len(candidates):
        raise ValueError(
            f"sample_size {sample_size} exceeds candidate count {len(candidates)}"
        )
    ranked = sorted(
        candidates,
        key=lambda row: (
            selection_digest(str(row["dataset_id"]), salt),
            int(str(row["dataset_id"])),
        ),
    )
    return [dict(row) for row in ranked[:sample_size]]


def _joint_interval(row: dict[str, Any]) -> tuple[float, float]:
    # ``sampling_lower``/``sampling_upper`` are the effective interval after
    # propagating bounded mimic followers back to their independent driver.
    if row.get("sampling_lower") is not None or row.get("sampling_upper") is not None:
        lower = row.get("sampling_lower")
        upper = row.get("sampling_upper")
        if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
            raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid effective range")
        lower_value = float(lower)
        upper_value = float(upper)
        if (
            not math.isfinite(lower_value)
            or not math.isfinite(upper_value)
            or upper_value - lower_value <= ZERO_WIDTH_TOLERANCE
        ):
            raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid effective range")
        return lower_value, upper_value
    if row["type"] == "continuous":
        return -math.pi, math.pi
    lower = row.get("lower")
    upper = row.get("upper")
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
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


def _joint_constraint_interval(row: dict[str, Any]) -> tuple[float, float]:
    """Return a finite joint constraint, including a degenerate point range."""

    if row.get("sampling_lower") is not None or row.get("sampling_upper") is not None:
        lower = row.get("sampling_lower")
        upper = row.get("sampling_upper")
        label = "effective range"
    elif row.get("type") == "continuous":
        return -math.pi, math.pi
    else:
        lower = row.get("lower")
        upper = row.get("upper")
        label = "range"
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has no finite {label}")
    lower_value = float(lower)
    upper_value = float(upper)
    width = upper_value - lower_value
    if (
        not math.isfinite(lower_value)
        or not math.isfinite(upper_value)
        or width < -ZERO_WIDTH_TOLERANCE
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid {label}")
    if width <= ZERO_WIDTH_TOLERANCE:
        fixed = (lower_value + upper_value) / 2.0
        return fixed, fixed
    return lower_value, upper_value


def single_joint_values(row: dict[str, Any]) -> list[float]:
    lower, upper = _joint_interval(row)
    return [
        lower + index * (upper - lower) / (SINGLE_SAMPLES - 1)
        for index in range(SINGLE_SAMPLES)
    ]


def sobol_joint_values(
    rows: list[dict[str, Any]], seed: int = SOBOL_SEED
) -> list[list[float]]:
    if not rows:
        return []
    from scipy.stats import qmc

    intervals = [_joint_interval(row) for row in rows]
    unit = qmc.Sobol(d=len(rows), scramble=True, seed=seed).random_base2(m=6)
    if len(unit) != SOBOL_SAMPLES:
        raise RuntimeError(f"Sobol generator returned {len(unit)} states")
    return [
        [
            float(lower + scalar * (upper - lower))
            for scalar, (lower, upper) in zip(vector, intervals)
        ]
        for vector in unit
    ]


def _finite_attribute(
    node: ET.Element,
    attribute: str,
    default: float,
    *,
    context: str,
) -> float:
    raw = node.get(attribute)
    if raw is None or raw == "":
        return float(default)
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context} mimic {attribute} is not numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{context} mimic {attribute} is not finite")
    return value


def parse_urdf_joints(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root is {root.tag!r}, expected 'robot'")
    rows: list[dict[str, Any]] = []
    for xml_index, node in enumerate(root.findall("joint")):
        joint_type = str(node.get("type", ""))
        if joint_type == "fixed":
            continue
        limit = node.find("limit")
        lower: float | None = None
        upper: float | None = None
        if joint_type in {"revolute", "prismatic"} and limit is not None:
            try:
                lower = float(limit.get("lower", ""))
                upper = float(limit.get("upper", ""))
            except ValueError:
                lower = upper = None
        row = {
            "xml_index": xml_index,
            "name": node.get("name", f"joint_{xml_index}"),
            "type": joint_type,
            "lower": lower,
            "upper": upper,
            "mimic": None,
        }
        mimic = node.find("mimic")
        if mimic is not None:
            target = str(mimic.get("joint") or "").strip()
            if not target:
                raise ValueError(f"joint {row['name']} has an empty mimic target")
            row["mimic"] = {
                "joint": target,
                "multiplier": _finite_attribute(
                    mimic, "multiplier", 1.0, context=str(row["name"])
                ),
                "offset": _finite_attribute(
                    mimic, "offset", 0.0, context=str(row["name"])
                ),
            }
        try:
            _joint_interval(row)
            row["range_evaluable"] = True
        except ValueError:
            row["range_evaluable"] = False
        rows.append(row)
    return rows


def _apply_external_joint_constraints(
    joints: Sequence[Mapping[str, Any]],
    external_joint_constraints: Sequence[Mapping[str, Any]] | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    """Apply reviewed affine constraints without mutating parsed URDF rows.

    External constraints use the same affine convention as URDF ``mimic``:
    ``follower = multiplier * driver + offset``.  They are deliberately
    explicit and fail closed; category names or joint-name heuristics never
    create a constraint inside the sampling compiler.
    """

    rows: list[dict[str, Any]] = []
    for source in joints:
        row = dict(source)
        mimic = row.get("mimic")
        row["mimic"] = dict(mimic) if isinstance(mimic, Mapping) else mimic
        rows.append(row)
    native_mimic_count = sum(row.get("mimic") is not None for row in rows)
    if external_joint_constraints is None:
        return rows, [], native_mimic_count
    if isinstance(external_joint_constraints, (str, bytes)) or not isinstance(
        external_joint_constraints, Sequence
    ):
        raise ValueError("external joint constraints must be a sequence")

    names = [str(row.get("name") or "") for row in rows]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("movable joint names must be non-empty and unique")
    by_name = {name: index for index, name in enumerate(names)}
    normalized: list[dict[str, Any]] = []
    external_followers: set[str] = set()
    for position, raw in enumerate(external_joint_constraints):
        if not isinstance(raw, Mapping):
            raise ValueError(
                f"external joint constraint {position} must be an object"
            )
        constraint_id = str(raw.get("constraint_id") or "").strip()
        driver = str(raw.get("driver_joint") or "").strip()
        follower = str(raw.get("follower_joint") or "").strip()
        if not constraint_id:
            raise ValueError(
                f"external joint constraint {position} has no constraint_id"
            )
        if driver not in by_name:
            raise ValueError(
                f"external joint constraint {constraint_id!r} references "
                f"missing driver joint {driver!r}"
            )
        if follower not in by_name:
            raise ValueError(
                f"external joint constraint {constraint_id!r} references "
                f"missing follower joint {follower!r}"
            )
        if driver == follower:
            raise ValueError(
                f"external joint constraint {constraint_id!r} is self-referential"
            )
        if follower in external_followers:
            raise ValueError(
                f"multiple external joint constraints target follower {follower!r}"
            )
        follower_row = rows[by_name[follower]]
        if follower_row.get("mimic") is not None:
            raise ValueError(
                f"external joint constraint {constraint_id!r} would override "
                f"native mimic joint {follower!r}"
            )
        try:
            multiplier = float(raw["multiplier"])
            offset = float(raw["offset"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"external joint constraint {constraint_id!r} has invalid coefficients"
            ) from exc
        if not math.isfinite(multiplier) or not math.isfinite(offset):
            raise ValueError(
                f"external joint constraint {constraint_id!r} has non-finite coefficients"
            )
        follower_row["mimic"] = {
            "joint": driver,
            "multiplier": multiplier,
            "offset": offset,
        }
        external_followers.add(follower)
        normalized.append(
            {
                "constraint_id": constraint_id,
                "driver_joint": driver,
                "follower_joint": follower,
                "multiplier": multiplier,
                "offset": offset,
            }
        )
    return rows, normalized, native_mimic_count


def compile_joint_sampling_plan(
    joints: list[dict[str, Any]],
    *,
    external_joint_constraints: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Compile a deterministic affine map from independent to full joint state.

    URDF's mimic relation is an equality constraint, not another free DoF.
    The compiler also intersects bounded follower limits with the driver's
    interval, so every generated state is inside all declared limits.
    """

    joints, normalized_external, native_mimic_count = (
        _apply_external_joint_constraints(joints, external_joint_constraints)
    )
    names = [str(row.get("name") or "") for row in joints]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("movable joint names must be non-empty and unique")
    by_name = {name: index for index, name in enumerate(names)}
    for index, row in enumerate(joints):
        mimic = row.get("mimic")
        if mimic is None:
            continue
        if not isinstance(mimic, dict):
            raise ValueError(f"joint {names[index]} has malformed mimic metadata")
        target = str(mimic.get("joint") or "")
        if target not in by_name:
            raise ValueError(
                f"joint {names[index]} mimics missing joint {target!r}"
            )
        if target == names[index]:
            raise ValueError(f"joint {names[index]} cannot mimic itself")

    roots = [index for index, row in enumerate(joints) if row.get("mimic") is None]
    visiting: set[int] = set()
    resolved: dict[int, tuple[int, float, float]] = {}

    def resolve(index: int) -> tuple[int, float, float]:
        if index in resolved:
            return resolved[index]
        if index in visiting:
            raise ValueError(f"mimic cycle detected at joint {names[index]!r}")
        visiting.add(index)
        mimic = joints[index].get("mimic")
        if mimic is None:
            result = (index, 1.0, 0.0)
        else:
            target_index = by_name[str(mimic["joint"])]
            target_root, target_a, target_b = resolve(target_index)
            multiplier = float(mimic.get("multiplier", 1.0))
            offset = float(mimic.get("offset", 0.0))
            if not math.isfinite(multiplier) or not math.isfinite(offset):
                raise ValueError(f"joint {names[index]} has non-finite mimic coefficients")
            result = (
                target_root,
                multiplier * target_a,
                multiplier * target_b + offset,
            )
        visiting.remove(index)
        resolved[index] = result
        return result

    bindings = [resolve(index) for index in range(len(joints))]
    intervals: dict[int, list[float]] = {}
    for root_index in roots:
        root_row = joints[root_index]
        try:
            lower, upper = _joint_constraint_interval(root_row)
        except ValueError:
            intervals[root_index] = [math.nan, math.nan]
            continue
        intervals[root_index] = [lower, upper]

    # Every finite bounded follower contributes an inverse affine constraint.
    for index, row in enumerate(joints):
        if row.get("type") not in {"revolute", "prismatic"}:
            continue
        lower = row.get("lower")
        upper = row.get("upper")
        root_index, coefficient, intercept = bindings[index]
        if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
            intervals[root_index] = [math.nan, math.nan]
            continue
        lower_value, upper_value = float(lower), float(upper)
        if (
            not math.isfinite(lower_value)
            or not math.isfinite(upper_value)
            or upper_value - lower_value < -ZERO_WIDTH_TOLERANCE
        ):
            intervals[root_index] = [math.nan, math.nan]
            continue
        if upper_value - lower_value <= ZERO_WIDTH_TOLERANCE:
            fixed = (lower_value + upper_value) / 2.0
            lower_value = fixed
            upper_value = fixed
        if root_index not in intervals or not math.isfinite(coefficient):
            continue
        if abs(coefficient) <= ZERO_WIDTH_TOLERANCE:
            if not (lower_value - ZERO_WIDTH_TOLERANCE <= intercept <= upper_value + ZERO_WIDTH_TOLERANCE):
                intervals[root_index] = [math.nan, math.nan]
            continue
        bound_a = (lower_value - intercept) / coefficient
        bound_b = (upper_value - intercept) / coefficient
        low, high = sorted((bound_a, bound_b))
        intervals[root_index][0] = max(intervals[root_index][0], low)
        intervals[root_index][1] = min(intervals[root_index][1], high)

    independent_rows: list[dict[str, Any]] = []
    sampled_roots: list[int] = []
    fixed_root_values: dict[int, float] = {}
    for root_index in roots:
        row = dict(joints[root_index])
        low, high = intervals[root_index]
        if (
            math.isfinite(low)
            and math.isfinite(high)
            and high - low >= -ZERO_WIDTH_TOLERANCE
            and high - low <= ZERO_WIDTH_TOLERANCE
        ):
            fixed_root_values[root_index] = (low + high) / 2.0
            continue
        sampled_roots.append(root_index)
        row["sampling_lower"] = low
        row["sampling_upper"] = high
        try:
            _joint_interval(row)
            row["sampling_range_evaluable"] = True
        except ValueError:
            row["sampling_range_evaluable"] = False
        row["zero_baseline_evaluable"] = bool(
            row["sampling_range_evaluable"]
            and low - ZERO_WIDTH_TOLERANCE <= 0.0 <= high + ZERO_WIDTH_TOLERANCE
        )
        # The frozen rest state and all inactive sweep dimensions are q=0.
        # If that baseline violates a propagated mimic limit, retaining the
        # asset as incomplete is safer than sampling an invalid configuration.
        if not row["zero_baseline_evaluable"]:
            row["sampling_range_evaluable"] = False
        independent_rows.append(row)

    root_position = {
        joint_index: position for position, joint_index in enumerate(sampled_roots)
    }

    binding_rows = [
        {
            "joint_index": index,
            "joint_name": names[index],
            "root_index": root_index,
            "root_name": names[root_index],
            "multiplier": coefficient,
            "offset": intercept,
        }
        for index, (root_index, coefficient, intercept) in enumerate(bindings)
    ]
    plan_payload = {
        "protocol": SAMPLING_PROTOCOL_V2,
        "joint_names": names,
        "independent_joint_names": [names[index] for index in sampled_roots],
        "bindings": binding_rows,
        "independent_intervals": [
            {
                "name": row["name"],
                "lower": row.get("sampling_lower"),
                "upper": row.get("sampling_upper"),
                "range_evaluable": bool(row.get("sampling_range_evaluable")),
                "zero_baseline_evaluable": bool(
                    row.get("zero_baseline_evaluable")
                ),
            }
            for row in independent_rows
        ],
    }
    if external_joint_constraints is not None:
        plan_payload["external_joint_constraints"] = normalized_external
    if fixed_root_values:
        plan_payload["fixed_roots"] = [
            {"name": names[index], "value": fixed_root_values[index]}
            for index in roots
            if index in fixed_root_values
        ]
    result = {
        "protocol": SAMPLING_PROTOCOL_V2,
        "joints": joints,
        "independent_joints": independent_rows,
        "bindings": bindings,
        "binding_rows": binding_rows,
        "root_position_by_index": root_position,
        "fixed_root_values": fixed_root_values,
        "fixed_root_joint_count": len(fixed_root_values),
        "independent_dof_count": len(sampled_roots),
        "range_evaluable_independent_dof_count": sum(
            bool(row.get("sampling_range_evaluable")) for row in independent_rows
        ),
        "mimic_joint_count": len(joints) - len(roots),
        "plan_sha256": canonical_sha256(plan_payload),
    }
    if external_joint_constraints is not None:
        result.update(
            {
                "native_mimic_joint_count": native_mimic_count,
                "external_joint_constraint_count": len(normalized_external),
                "external_joint_constraints": normalized_external,
            }
        )
    return result


def expand_joint_values(
    plan: dict[str, Any], independent_values: list[float]
) -> list[float]:
    expected = int(plan.get("independent_dof_count", -1))
    if len(independent_values) != expected:
        raise ValueError(
            f"independent state has {len(independent_values)} values, expected {expected}"
        )
    values: list[float] = []
    root_positions = plan.get("root_position_by_index", {})
    fixed_root_values = plan.get("fixed_root_values", {})
    for root_index, coefficient, intercept in plan["bindings"]:
        if root_index in fixed_root_values:
            root_value = float(fixed_root_values[root_index])
        else:
            try:
                root_position = int(root_positions[root_index])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"sampling plan has no root position for {root_index}") from exc
            root_value = float(independent_values[root_position])
        value = coefficient * root_value + intercept
        if not math.isfinite(value):
            raise ValueError("expanded joint state is non-finite")
        values.append(value)
    return values


def sampling_plan_metadata(
    urdf_path: Path,
    *,
    declared_dof: int | None = None,
    expected_sha256: str | None = None,
    external_joint_constraints: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return frozen v2 sampling metadata for a URDF.

    This helper is used while freezing jobs, before any child process runs, so
    retained preload/timeout failures still carry the correct independent-DoF
    denominator.  Invalid plans are represented explicitly and remain
    fail-closed at execution time.
    """

    try:
        if expected_sha256 is not None and sha256_file(urdf_path) != str(
            expected_sha256
        ):
            raise ValueError("primary URDF hash drift before sampling-plan freeze")
        joints = parse_urdf_joints(urdf_path)
        if declared_dof is not None and len(joints) != int(declared_dof):
            raise ValueError(
                f"declared joint count mismatch: {len(joints)} != {declared_dof}"
            )
        plan = compile_joint_sampling_plan(
            joints,
            external_joint_constraints=external_joint_constraints,
        )
        metadata = {
            "independent_dof_count": int(plan["independent_dof_count"]),
            "range_evaluable_independent_dof_count": int(
                plan["range_evaluable_independent_dof_count"]
            ),
            "mimic_joint_count": int(plan["mimic_joint_count"]),
            "fixed_root_joint_count": int(plan["fixed_root_joint_count"]),
            "joint_sampling_plan_sha256": str(plan["plan_sha256"]),
            "sampling_plan_error": None,
        }
        if external_joint_constraints is not None:
            metadata.update(
                {
                    "native_mimic_joint_count": int(
                        plan["native_mimic_joint_count"]
                    ),
                    "external_joint_constraint_count": int(
                        plan["external_joint_constraint_count"]
                    ),
                }
            )
        return metadata
    except (OSError, ET.ParseError, TypeError, ValueError) as exc:
        fallback = int(declared_dof or 0)
        metadata = {
            "independent_dof_count": fallback,
            "range_evaluable_independent_dof_count": 0,
            "mimic_joint_count": 0,
            "fixed_root_joint_count": 0,
            "joint_sampling_plan_sha256": None,
            "sampling_plan_error": f"{type(exc).__name__}: {exc}",
        }
        if external_joint_constraints is not None:
            metadata.update(
                {
                    "native_mimic_joint_count": 0,
                    "external_joint_constraint_count": 0,
                }
            )
        return metadata


def collision_mesh_inventory(asset_dir: Path, urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory = []
    for reference in references:
        path = asset_dir / reference
        exists = path.is_file()
        inventory.append(
            {
                "path": reference,
                "exists": exists,
                "size_bytes": path.stat().st_size if exists else None,
                "sha256": sha256_file(path) if exists else None,
            }
        )
    return inventory


def validate_frozen_asset_files(item: dict[str, Any], asset_dir: Path) -> None:
    urdf_path = asset_dir / "mobility.urdf"
    if sha256_file(urdf_path) != item.get("urdf_sha256"):
        raise RuntimeError("URDF content drift after freeze")
    bounding_box_hash = item.get("bounding_box_sha256")
    if bounding_box_hash is not None and sha256_file(
        asset_dir / "bounding_box.json"
    ) != bounding_box_hash:
        raise RuntimeError("bounding-box content drift after freeze")
    observed = collision_mesh_inventory(asset_dir, urdf_path)
    if (
        canonical_sha256(observed)
        != item.get("collision_mesh_inventory_sha256")
        or observed != item.get("collision_mesh_files")
    ):
        raise RuntimeError("collision mesh inventory drift after freeze")


def validate_frozen_archive(
    frozen: dict[str, Any], archive: Path, *, expected_sha256: str
) -> None:
    if archive.stat().st_size != int(frozen["size_bytes"]):
        raise RuntimeError("archive size drift after freeze")
    observed = sha256_file(archive)
    if observed != frozen.get("sha256") or observed != expected_sha256:
        raise RuntimeError("archive SHA256 drift after freeze")


def current_runtime_identity() -> dict[str, Any]:
    import numpy
    import pybullet as bullet
    import scipy
    from scipy.stats import _qmc

    python_executable = Path(sys.executable)
    bullet_module = Path(bullet.__file__).resolve(strict=True)
    qmc_module = Path(_qmc.__file__).resolve(strict=True)
    return {
        "python_executable": os.path.abspath(python_executable),
        "python_executable_sha256": sha256_file(python_executable),
        "python_version": sys.version,
        "python_cache_tag": sys.implementation.cache_tag,
        "pybullet_api_version": bullet.getAPIVersion(),
        "pybullet_module": str(bullet_module),
        "pybullet_module_sha256": sha256_file(bullet_module),
        "scipy_version": scipy.__version__,
        "scipy_qmc_module": str(qmc_module),
        "scipy_qmc_module_sha256": sha256_file(qmc_module),
        "numpy_version": numpy.__version__,
    }


def require_runtime_match(
    expected: dict[str, Any], observed: dict[str, Any]
) -> None:
    if expected != observed:
        differing = sorted(
            key
            for key in set(expected) | set(observed)
            if expected.get(key) != observed.get(key)
        )
        raise RuntimeError(
            f"child runtime identity mismatch in fields: {differing}"
        )


def probe_runtime_identity(python: Path, result_path: Path) -> dict[str, Any]:
    result_path.unlink(missing_ok=True)
    command = [
        str(python),
        str(SCRIPT),
        "--phase",
        "runtime",
        "--runtime-result",
        str(result_path),
    ]
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120,
        check=False,
    )
    if completed.returncode != 0 or not result_path.is_file():
        raise RuntimeError(
            "child runtime probe failed: "
            f"returncode={completed.returncode}, output={completed.stdout[-2000:]}"
        )
    return read_json(result_path)


def package_audit(asset_dir: Path) -> dict[str, Any]:
    missing_core = [name for name in CORE_FILES if not (asset_dir / name).is_file()]
    result: dict[str, Any] = {
        "package_audit_success": False,
        "missing_core_files": missing_core,
        "category": None,
        "movable_dof_count": 0,
        "range_evaluable_dof_count": 0,
        "independent_dof_count": 0,
        "range_evaluable_independent_dof_count": 0,
        "mimic_joint_count": 0,
        "fixed_root_joint_count": 0,
        "joint_sampling_plan_sha256": None,
        "sampling_plan_error": None,
        "joint_specs": [],
        "joint_specs_sha256": canonical_sha256([]),
        "missing_collision_mesh_reference_count": None,
        "object_bbox_diagonal_m": None,
        "urdf_sha256": None,
        "bounding_box_sha256": None,
        "collision_mesh_files": [],
        "collision_mesh_inventory_sha256": None,
        "audit_issue": None,
    }
    try:
        if missing_core:
            raise FileNotFoundError(f"missing core files: {missing_core}")
        meta = read_json(asset_dir / "meta.json")
        category = meta.get("model_cat")
        if not isinstance(category, str) or not category:
            raise ValueError("meta.json has no nonempty model_cat")
        urdf_path = asset_dir / "mobility.urdf"
        joints = parse_urdf_joints(urdf_path)
        plan = compile_joint_sampling_plan(joints)
        mesh_inventory = collision_mesh_inventory(asset_dir, urdf_path)
        missing_mesh = [row for row in mesh_inventory if not row["exists"]]
        bbox = read_json(asset_dir / "bounding_box.json")
        minimum = [float(value) for value in bbox["min"]]
        maximum = [float(value) for value in bbox["max"]]
        if len(minimum) != 3 or len(maximum) != 3:
            raise ValueError("bounding_box.json is not 3D")
        diagonal = math.sqrt(
            sum((high - low) ** 2 for low, high in zip(minimum, maximum))
        )
        if not math.isfinite(diagonal) or diagonal <= 0:
            raise ValueError("object bounding-box diagonal is not positive finite")
        result.update(
            {
                "package_audit_success": True,
                "category": category,
                "movable_dof_count": len(joints),
                "range_evaluable_dof_count": sum(
                    bool(row["range_evaluable"]) for row in joints
                ),
                "independent_dof_count": int(plan["independent_dof_count"]),
                "range_evaluable_independent_dof_count": int(
                    plan["range_evaluable_independent_dof_count"]
                ),
                "mimic_joint_count": int(plan["mimic_joint_count"]),
                "fixed_root_joint_count": int(plan["fixed_root_joint_count"]),
                "joint_sampling_plan_sha256": str(plan["plan_sha256"]),
                "sampling_plan_error": None,
                "joint_specs": joints,
                "joint_specs_sha256": canonical_sha256(joints),
                "missing_collision_mesh_reference_count": len(missing_mesh),
                "object_bbox_diagonal_m": diagonal,
                "urdf_sha256": sha256_file(urdf_path),
                "bounding_box_sha256": sha256_file(
                    asset_dir / "bounding_box.json"
                ),
                "collision_mesh_files": mesh_inventory,
                "collision_mesh_inventory_sha256": canonical_sha256(
                    mesh_inventory
                ),
            }
        )
    except Exception as exc:  # noqa: BLE001
        result["audit_issue"] = f"{type(exc).__name__}: {exc}"
    return result


def discover_release_ids(dataset_root: Path) -> list[str]:
    return sorted(
        (
            path.name
            for path in dataset_root.iterdir()
            if path.is_dir() and path.name.isdigit()
        ),
        key=int,
    )


def selection_contract(
    release_ids: list[str],
    *,
    sample_size: int,
    qualification_smoke: bool,
) -> dict[str, Any]:
    if len(release_ids) != EXPECTED_RELEASE_ASSETS:
        raise RuntimeError(
            f"release has {len(release_ids)} numeric IDs, expected {EXPECTED_RELEASE_ASSETS}"
        )
    pool_hash = canonical_sha256(release_ids)
    if pool_hash != EXPECTED_CANDIDATE_POOL_SHA256:
        raise RuntimeError(
            f"candidate pool identity mismatch: {pool_hash}"
        )
    if not qualification_smoke and sample_size != SAMPLE_SIZE:
        raise ValueError(
            f"formal protocol requires sample_size={SAMPLE_SIZE}, got {sample_size}"
        )
    selected = select_candidates(
        [{"dataset_id": dataset_id} for dataset_id in release_ids],
        sample_size,
        SELECTION_SALT,
    )
    selected_hash = canonical_sha256(
        [row["dataset_id"] for row in selected]
    )
    if not qualification_smoke and selected_hash != EXPECTED_SELECTED_IDS_SHA256:
        raise RuntimeError(
            f"formal ordered selection identity mismatch: {selected_hash}"
        )
    return {
        "protocol_id": (
            QUALIFICATION_PROTOCOL_ID if qualification_smoke else PROTOCOL_ID
        ),
        "cohort_label": (
            f"PartNet-Mobility qualification smoke N={sample_size}"
            if qualification_smoke
            else "PartNet-Mobility N=800 sampled release cohort"
        ),
        "qualification_smoke": qualification_smoke,
        "candidate_pool_identity_sha256": pool_hash,
        "selected": selected,
        "ordered_selected_ids_sha256": selected_hash,
    }


def build_manifest(
    dataset_root: Path,
    archive: Path,
    sample_size: int = SAMPLE_SIZE,
    qualification_smoke: bool = False,
    child_runtime: dict[str, Any] | None = None,
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    archive = archive.resolve(strict=True)
    release_ids = discover_release_ids(dataset_root)
    contract = selection_contract(
        release_ids,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
    )
    selected = contract["selected"]
    frozen_child_runtime = child_runtime or current_runtime_identity()
    require_runtime_match(current_runtime_identity(), frozen_child_runtime)
    items = []
    for order, selected_row in enumerate(selected):
        dataset_id = str(selected_row["dataset_id"])
        audit = package_audit(dataset_root / dataset_id)
        item = {
            "protocol_id": contract["protocol_id"],
            "order": order,
            "dataset_id": dataset_id,
            "selection_digest": selection_digest(dataset_id, SELECTION_SALT),
            **audit,
            "runtime_identity": frozen_child_runtime,
            "runtime_identity_sha256": canonical_sha256(frozen_child_runtime),
        }
        item["rest_state_expected"] = 1
        item["single_state_expected"] = SINGLE_SAMPLES * int(
            item["movable_dof_count"]
        )
        item["sobol_state_expected"] = (
            SOBOL_SAMPLES if int(item["movable_dof_count"]) > 0 else 0
        )
        item["input_identity_sha256"] = canonical_sha256(
            {
                key: item[key]
                for key in (
                    "protocol_id",
                    "order",
                    "dataset_id",
                    "selection_digest",
                    "category",
                    "movable_dof_count",
                    "range_evaluable_dof_count",
                    "joint_specs_sha256",
                    "runtime_identity_sha256",
                    "urdf_sha256",
                    "bounding_box_sha256",
                    "collision_mesh_inventory_sha256",
                    "object_bbox_diagonal_m",
                    "rest_state_expected",
                    "single_state_expected",
                    "sobol_state_expected",
                )
            }
        )
        items.append(item)
    archive_sha256 = sha256_file(archive)
    manifest = {
        "protocol_id": contract["protocol_id"],
        "status": "FROZEN",
        "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "cohort_label": contract["cohort_label"],
        "qualification_smoke": qualification_smoke,
        "cohort_boundary": {
            "is_full_release_cohort": False,
            "is_shared_category_balanced_cohort": False,
            "paper_table_role": (
                "qualification only"
                if qualification_smoke
                else "sampled release diagnostic"
            ),
        },
        "dataset_root": str(dataset_root),
        "archive": {
            "path": str(archive),
            "size_bytes": archive.stat().st_size,
            "sha256": archive_sha256,
            "matches_expected_sha256": archive_sha256
            == EXPECTED_ARCHIVE_SHA256,
        },
        "release_asset_count": len(release_ids),
        "candidate_pool_identity_sha256": contract[
            "candidate_pool_identity_sha256"
        ],
        "sample_size": sample_size,
        "selection_policy": {
            "algorithm": "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)",
            "salt": SELECTION_SALT,
            "identity_fields_used": ["dataset_id"],
            "outcome_based_filtering": False,
            "selected_failures_retained_without_replacement": True,
        },
        "sampling": {
            "rest_state": "native URDF/PyBullet q=0",
            "single_joint_states_per_declared_nonfixed_joint": SINGLE_SAMPLES,
            "single_joint_other_joint_state": 0.0,
            "continuous_joint_interval": [-math.pi, math.pi],
            "sobol_states_per_asset_with_nonfixed_joint": SOBOL_SAMPLES,
            "sobol_scramble": True,
            "sobol_seed": SOBOL_SEED,
        },
        "collision_policy": {
            "penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "surface_contact_allowed": True,
            "rest_panels": ["all_pair", "exclude_direct_parent_child"],
            "sweep_sobol_strict_policy": "exclude_direct_parent_child",
            "continuous_collision_detection": "not_run",
            "aor": "N/E: no stable exact overlap-volume implementation",
            "object_scale": "diagonal from release bounding_box.json",
        },
        "runtime": {
            "runner_sha256": sha256_file(SCRIPT),
            "manifest_builder": current_runtime_identity(),
            "child": frozen_child_runtime,
        },
        "items": items,
    }
    manifest["ordered_selected_ids_sha256"] = contract[
        "ordered_selected_ids_sha256"
    ]
    manifest["items_sha256"] = canonical_sha256(items)
    return manifest


def prepare(
    dataset_root: Path,
    archive: Path,
    output: Path,
    sample_size: int,
    qualification_smoke: bool,
    child_python: Path,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    runtime_path = output / "child_runtime_probe.json"
    child_runtime = probe_runtime_identity(child_python, runtime_path)
    require_runtime_match(current_runtime_identity(), child_runtime)
    manifest_path = output / "frozen_manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
        validate_manifest(
            manifest,
            dataset_root,
            archive,
            qualification_smoke=qualification_smoke,
            child_runtime=child_runtime,
        )
        return manifest
    manifest = build_manifest(
        dataset_root,
        archive,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
        child_runtime=child_runtime,
    )
    if not manifest["archive"]["matches_expected_sha256"]:
        raise RuntimeError("PartNet-Mobility archive hash does not match the frozen pin")
    atomic_json(manifest_path, manifest)
    return manifest


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    archive: Path,
    *,
    qualification_smoke: bool,
    child_runtime: dict[str, Any] | None = None,
) -> None:
    release_ids = discover_release_ids(dataset_root)
    contract = selection_contract(
        release_ids,
        sample_size=int(manifest.get("sample_size", -1)),
        qualification_smoke=qualification_smoke,
    )
    if manifest.get("protocol_id") != contract["protocol_id"]:
        raise RuntimeError("manifest protocol mismatch")
    if manifest.get("qualification_smoke") is not qualification_smoke:
        raise RuntimeError("manifest qualification mode mismatch")
    if Path(manifest["dataset_root"]).resolve() != dataset_root.resolve():
        raise RuntimeError("manifest dataset root mismatch")
    if Path(manifest["archive"]["path"]).resolve() != archive.resolve():
        raise RuntimeError("manifest archive path mismatch")
    validate_frozen_archive(
        manifest["archive"], archive, expected_sha256=EXPECTED_ARCHIVE_SHA256
    )
    items = manifest.get("items", [])
    if len(items) != int(manifest.get("sample_size", -1)):
        raise RuntimeError("manifest selected item count mismatch")
    if manifest.get("candidate_pool_identity_sha256") != contract[
        "candidate_pool_identity_sha256"
    ]:
        raise RuntimeError("manifest candidate pool hash mismatch")
    if [item["dataset_id"] for item in items] != [
        row["dataset_id"] for row in contract["selected"]
    ]:
        raise RuntimeError("manifest selected ID order mismatch")
    if manifest.get("ordered_selected_ids_sha256") != contract[
        "ordered_selected_ids_sha256"
    ]:
        raise RuntimeError("manifest selected ID hash mismatch")
    if canonical_sha256(items) != manifest.get("items_sha256"):
        raise RuntimeError("manifest items hash mismatch")
    if sha256_file(SCRIPT) != manifest["runtime"]["runner_sha256"]:
        raise RuntimeError("runner changed after cohort freeze")
    observed_runtime = child_runtime or current_runtime_identity()
    require_runtime_match(manifest["runtime"]["child"], observed_runtime)


PAIR_POLICY_FIXTURE = """<?xml version="1.0"?>
<robot name="table4_pair_policy_smoke">
  <link name="base">
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="child">
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="child_joint" type="fixed">
    <parent link="base"/><child link="child"/>
  </joint>
</robot>
"""


def _direct_parent_pairs(bullet: Any, body: int, client: int) -> set[frozenset[int]]:
    result: set[frozenset[int]] = set()
    for child_index in range(bullet.getNumJoints(body, physicsClientId=client)):
        info = bullet.getJointInfo(body, child_index, physicsClientId=client)
        result.add(frozenset((int(info[16]), child_index)))
    return result


def _collision_observation(
    bullet: Any,
    body: int,
    client: int,
    direct_parent_pairs: set[frozenset[int]],
) -> dict[str, Any]:
    contacts = bullet.getContactPoints(
        bodyA=body, bodyB=body, physicsClientId=client
    )
    all_depths = [max(0.0, -float(contact[8])) for contact in contacts]
    non_adjacent = [
        contact
        for contact in contacts
        if frozenset((int(contact[3]), int(contact[4]))) not in direct_parent_pairs
    ]
    non_adjacent_depths = [
        max(0.0, -float(contact[8])) for contact in non_adjacent
    ]
    return {
        "all_pair_contact_count": len(contacts),
        "all_pair_illegal_penetration_count": sum(
            penetration_is_illegal(float(contact[8])) for contact in contacts
        ),
        "all_pair_max_penetration_m": max(all_depths or [0.0]),
        "non_adjacent_contact_count": len(non_adjacent),
        "non_adjacent_illegal_penetration_count": sum(
            penetration_is_illegal(float(contact[8]))
            for contact in non_adjacent
        ),
        "non_adjacent_max_penetration_m": max(non_adjacent_depths or [0.0]),
    }


def run_pair_policy_smoke(output: Path) -> dict[str, Any]:
    import pybullet as bullet

    output.mkdir(parents=True, exist_ok=True)
    fixture = output / "pair_policy_smoke.urdf"
    atomic_text(fixture, PAIR_POLICY_FIXTURE)
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        )
        body = bullet.loadURDF(
            str(fixture), useFixedBase=True, flags=flags, physicsClientId=client
        )
        bullet.performCollisionDetection(physicsClientId=client)
        observation = _collision_observation(
            bullet, body, client, _direct_parent_pairs(bullet, body, client)
        )
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    passed = (
        observation["all_pair_illegal_penetration_count"] > 0
        and observation["non_adjacent_illegal_penetration_count"] == 0
    )
    result = {
        "protocol_id": "urdf_table4_pybullet_pair_policy_smoke_v1",
        "status": "PASS" if passed else "FAIL",
        "pybullet_api_version": bullet.getAPIVersion(),
        "load_flags": flags,
        **observation,
    }
    atomic_json(output / "pair_policy_smoke.json", result)
    if not passed:
        raise RuntimeError(f"pair-policy smoke failed: {result}")
    return result


def _reset_and_observe(
    bullet: Any,
    body: int,
    client: int,
    joint_indices: list[int],
    values: list[float],
    direct_parent_pairs: set[frozenset[int]],
) -> tuple[dict[str, Any], float]:
    for index, value in zip(joint_indices, values):
        bullet.resetJointState(
            body, index, value, targetVelocity=0.0, physicsClientId=client
        )
    bullet.performCollisionDetection(physicsClientId=client)
    readback_error = max(
        [
            abs(
                float(bullet.getJointState(body, index, physicsClientId=client)[0])
                - value
            )
            for index, value in zip(joint_indices, values)
        ]
        or [0.0]
    )
    if readback_error > RESET_TOLERANCE:
        raise RuntimeError(
            f"reset/readback error {readback_error} exceeds {RESET_TOLERANCE}"
        )
    return (
        _collision_observation(
            bullet, body, client, direct_parent_pairs
        ),
        readback_error,
    )


def _reset_and_readback(
    bullet: Any,
    body: int,
    client: int,
    joint_indices: list[int],
    values: list[float],
) -> float:
    """Reset one state without taking a collision-detection snapshot."""

    for index, value in zip(joint_indices, values):
        bullet.resetJointState(
            body, index, value, targetVelocity=0.0, physicsClientId=client
        )
    readback_error = max(
        [
            abs(
                float(bullet.getJointState(body, index, physicsClientId=client)[0])
                - value
            )
            for index, value in zip(joint_indices, values)
        ]
        or [0.0]
    )
    if readback_error > RESET_TOLERANCE:
        raise RuntimeError(
            f"reset/readback error {readback_error} exceeds {RESET_TOLERANCE}"
        )
    return readback_error


def evaluate_asset(item: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    import pybullet as bullet

    result = failure_record(item, "evaluation_not_completed")
    result["issues"] = []
    result["state_records"] = []
    result["input_identity_sha256"] = item["input_identity_sha256"]
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["runtime_identity"] = current_runtime_identity()
    result["sampling_protocol"] = str(
        item.get("sampling_protocol", SAMPLING_PROTOCOL_V1)
    )
    asset_dir = dataset_root / str(item["dataset_id"])
    urdf_path = asset_dir / "mobility.urdf"
    if not item.get("package_audit_success"):
        result["issues"].append(
            str(item.get("audit_issue") or "package_audit_failed")
        )
        return result
    missing_collision_meshes = int(
        item.get("missing_collision_mesh_reference_count") or 0
    )
    if missing_collision_meshes:
        result["issues"].append(
            f"missing_collision_mesh_references:{missing_collision_meshes}"
        )
        return result
    try:
        validate_frozen_asset_files(item, asset_dir)
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
        return result
    try:
        joints = parse_urdf_joints(urdf_path)
        sampling_protocol = result["sampling_protocol"]
        plan: dict[str, Any] | None = None
        if sampling_protocol == SAMPLING_PROTOCOL_V2:
            plan = compile_joint_sampling_plan(joints)
            expected_independent = int(plan["independent_dof_count"])
            expected_range_independent = int(
                plan["range_evaluable_independent_dof_count"]
            )
            if item.get("joint_sampling_plan_sha256") not in {
                None,
                plan["plan_sha256"],
            }:
                raise RuntimeError("joint sampling plan hash mismatch")
            result.update(
                {
                    "independent_dof_count": expected_independent,
                    "range_evaluable_independent_dof_count": expected_range_independent,
                    "mimic_joint_count": int(plan["mimic_joint_count"]),
                    "fixed_root_joint_count": int(plan["fixed_root_joint_count"]),
                    "joint_sampling_plan_sha256": str(plan["plan_sha256"]),
                }
            )
        elif sampling_protocol == SAMPLING_PROTOCOL_V1:
            expected_independent = len(joints)
            expected_range_independent = sum(
                bool(row["range_evaluable"]) for row in joints
            )
        else:
            raise ValueError(f"unknown sampling protocol: {sampling_protocol}")
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
        return result
    result["movable_dof_count"] = len(joints)
    result["range_evaluable_dof_count"] = sum(
        bool(row["range_evaluable"]) for row in joints
    )
    result["independent_dof_count"] = expected_independent
    result["range_evaluable_independent_dof_count"] = expected_range_independent
    if sampling_protocol == SAMPLING_PROTOCOL_V2:
        result["single_state_expected"] = int(item.get(
            "single_state_expected", SINGLE_SAMPLES * expected_independent
        ))
        result["sobol_state_expected"] = int(item.get(
            "sobol_state_expected", SOBOL_SAMPLES if expected_independent else 0
        ))
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = index
        missing_names = [
            row["name"] for row in joints if row["name"] not in simulator_by_name
        ]
        if missing_names:
            raise RuntimeError(f"simulator joint mapping missing: {missing_names}")
        joint_indices = [simulator_by_name[row["name"]] for row in joints]
        for index in joint_indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        direct_pairs = _direct_parent_pairs(bullet, body, client)
        if plan is None:
            rest_values = [0.0] * len(joints)
            independent_rows = joints
        else:
            independent_rows = list(plan["independent_joints"])
            rest_values = expand_joint_values(
                plan, [0.0] * int(plan["independent_dof_count"])
            )
        scale = float(item["object_bbox_diagonal_m"])
        max_penetration_m = 0.0
        max_readback_error = 0.0

        def observe(
            values: list[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> dict[str, Any]:
            nonlocal max_penetration_m, max_readback_error
            collision, readback_error = _reset_and_observe(
                bullet,
                body,
                client,
                joint_indices,
                values,
                direct_pairs,
            )
            max_readback_error = max(max_readback_error, readback_error)
            metric_max_penetration_m = float(
                collision[
                    "all_pair_max_penetration_m"
                    if phase == "rest"
                    else "non_adjacent_max_penetration_m"
                ]
            )
            max_penetration_m = max(
                max_penetration_m,
                metric_max_penetration_m,
            )
            state = {
                "dataset_id": item["dataset_id"],
                "category": item.get("category"),
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(values),
                "sampling_protocol": sampling_protocol,
                "joint_sampling_plan_sha256": result.get(
                    "joint_sampling_plan_sha256"
                ),
                "reset_readback_max_abs_error": readback_error,
                "metric_max_penetration_m": metric_max_penetration_m,
                **collision,
            }
            result["state_records"].append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["rest_all_pair_cf"] = (
            rest["all_pair_illegal_penetration_count"] == 0
        )
        result["rest_non_adjacent_cf"] = (
            rest["non_adjacent_illegal_penetration_count"] == 0
        )
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])

        joint_sweep_passes = 0
        for independent_position, row in zip(
            range(len(independent_rows)), independent_rows
        ):
            range_ok = bool(
                row.get("sampling_range_evaluable", row.get("range_evaluable"))
            )
            if not range_ok:
                result["issues"].append(
                    f"joint_range_not_evaluable:{row['name']}"
                )
                continue
            joint_free = True
            for sample_index, value in enumerate(single_joint_values(row)):
                if plan is None:
                    values = list(rest_values)
                    values[independent_position] = value
                else:
                    independent_values = [0.0] * len(independent_rows)
                    independent_values[independent_position] = float(value)
                    values = expand_joint_values(plan, independent_values)
                state = observe(
                    values,
                    "single_joint_sweep",
                    sample_index,
                    str(row["name"]),
                )
                result["single_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["single_non_adjacent_free"] += int(free)
                joint_free = joint_free and free
            joint_sweep_passes += int(joint_free)
        result["joint_single_sweep_cf_passed"] = joint_sweep_passes

        evaluable_positions = [
            index
            for index, row in enumerate(independent_rows)
            if bool(row.get("sampling_range_evaluable", row.get("range_evaluable")))
        ]
        evaluable_rows = [independent_rows[index] for index in evaluable_positions]
        if evaluable_rows and len(evaluable_rows) == len(independent_rows):
            for sample_index, sampled in enumerate(
                sobol_joint_values(evaluable_rows, seed=SOBOL_SEED)
            ):
                if plan is None:
                    values = list(rest_values)
                    for position, value in zip(evaluable_positions, sampled):
                        values[position] = value
                else:
                    values = expand_joint_values(plan, [float(value) for value in sampled])
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["sobol_non_adjacent_free"] += int(free)

        result["single_joint_sweep_cf"] = (
            result["single_state_executed"] == result["single_state_expected"]
            and result["single_non_adjacent_free"]
            == result["single_state_expected"]
        )
        result["multi_joint_sobol_cf"] = (
            expected_range_independent == expected_independent
            and expected_independent > 0
            and
            result["sobol_state_executed"] == result["sobol_state_expected"]
            and result["sobol_non_adjacent_free"]
            == result["sobol_state_expected"]
        )
        result["measurement_complete"] = (
            expected_range_independent == expected_independent
            and
            result["rest_state_executed"] == result["rest_state_expected"]
            and result["single_state_executed"] == result["single_state_expected"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
        if result["issues"] == ["evaluation_not_completed"]:
            result["issues"] = []
        else:
            result["issues"] = [
                issue
                for issue in result["issues"]
                if issue != "evaluation_not_completed"
            ]
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    states = result["state_records"]
    if states:
        result["max_penetration_m"] = max(
            float(state["metric_max_penetration_m"]) for state in states
        )
        result["max_penetration_normalized"] = (
            float(result["max_penetration_m"])
            / float(result["object_bbox_diagonal_m"])
        )
        result["max_reset_readback_error"] = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
    result["state_records_sha256"] = canonical_sha256(states)
    return result


def failure_record(
    item: dict[str, Any], issue: str, *, timed_out: bool = False
) -> dict[str, Any]:
    movable = int(item.get("movable_dof_count", 0))
    sampling_protocol = str(item.get("sampling_protocol", SAMPLING_PROTOCOL_V1))
    independent = (
        int(
            item.get(
                "independent_dof_count",
                item.get("expected_independent_joints", movable),
            )
        )
        if sampling_protocol == SAMPLING_PROTOCOL_V2
        else movable
    )
    rest_expected = int(item.get("rest_state_expected", 1))
    single_expected = int(
        item.get("single_state_expected", SINGLE_SAMPLES * independent)
    )
    sobol_expected = int(
        item.get("sobol_state_expected", SOBOL_SAMPLES if independent > 0 else 0)
    )
    return {
        "protocol_id": item.get("protocol_id", PROTOCOL_ID),
        "sampling_protocol": sampling_protocol,
        "order": item.get("order"),
        "dataset_id": str(item["dataset_id"]),
        "category": item.get("category"),
        "input_identity_sha256": item.get("input_identity_sha256"),
        "load_success": False,
        "measurement_complete": False,
        "movable_dof_count": movable,
        "range_evaluable_dof_count": int(
            item.get("range_evaluable_dof_count", 0)
        ),
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            item.get("range_evaluable_independent_dof_count", 0)
        ),
        "mimic_joint_count": int(item.get("mimic_joint_count", 0)),
        "fixed_root_joint_count": int(item.get("fixed_root_joint_count", 0)),
        "joint_sampling_plan_sha256": item.get("joint_sampling_plan_sha256"),
        "sampling_plan_error": item.get("sampling_plan_error"),
        "rest_state_expected": rest_expected,
        "rest_state_executed": 0,
        "rest_non_adjacent_free": 0,
        "rest_all_pair_cf": False,
        "rest_non_adjacent_cf": False,
        "single_state_expected": single_expected,
        "single_state_executed": 0,
        "single_non_adjacent_free": 0,
        "joint_single_sweep_cf_passed": 0,
        "single_joint_sweep_cf": False,
        "sobol_state_expected": sobol_expected,
        "sobol_state_executed": 0,
        "sobol_non_adjacent_free": 0,
        "multi_joint_sobol_cf": False,
        "strict_collision_pass": False,
        "max_penetration_m": None,
        "max_penetration_normalized": None,
        "max_reset_readback_error": None,
        "object_bbox_diagonal_m": item.get("object_bbox_diagonal_m"),
        "child_timed_out": timed_out,
        "child_returncode": -9 if timed_out else None,
        "issues": [issue],
        "state_records": [],
        "state_records_sha256": canonical_sha256([]),
        "runtime_identity": item.get("runtime_identity"),
    }


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> None:
    item = read_json(item_path)
    result = evaluate_asset(item, dataset_root)
    atomic_json(result_path, result)


def expected_state_identity_rows(item: dict[str, Any]) -> list[dict[str, Any]]:
    joints = list(item.get("joint_specs", []))
    sampling_protocol = str(item.get("sampling_protocol", SAMPLING_PROTOCOL_V1))
    plan = (
        compile_joint_sampling_plan(joints)
        if sampling_protocol == SAMPLING_PROTOCOL_V2
        else None
    )
    independent_rows = joints if plan is None else list(plan["independent_joints"])
    rest_values = (
        [0.0] * len(joints)
        if plan is None
        else expand_joint_values(plan, [0.0] * len(independent_rows))
    )
    identities = [
        {
            "phase": "rest",
            "joint_name": None,
            "sample_index": 0,
            "joint_values_sha256": canonical_sha256(rest_values),
        }
    ]
    for position, joint in enumerate(independent_rows):
        if not joint.get("sampling_range_evaluable", joint.get("range_evaluable")):
            continue
        for sample_index, value in enumerate(single_joint_values(joint)):
            if plan is None:
                values = list(rest_values)
                values[position] = value
            else:
                independent_values = [0.0] * len(independent_rows)
                independent_values[position] = value
                values = expand_joint_values(plan, independent_values)
            identities.append(
                {
                    "phase": "single_joint_sweep",
                    "joint_name": str(joint["name"]),
                    "sample_index": sample_index,
                    "joint_values_sha256": canonical_sha256(values),
                }
            )
    if independent_rows and all(
        joint.get("sampling_range_evaluable", joint.get("range_evaluable"))
        for joint in independent_rows
    ):
        for sample_index, sampled in enumerate(sobol_joint_values(independent_rows)):
            values = (
                sampled
                if plan is None
                else expand_joint_values(plan, [float(value) for value in sampled])
            )
            identities.append(
                {
                    "phase": "multi_joint_sobol",
                    "joint_name": None,
                    "sample_index": sample_index,
                    "joint_values_sha256": canonical_sha256(values),
                }
            )
    return identities


def validate_state_closure(
    record: dict[str, Any],
    state_records: list[dict[str, Any]] | None = None,
    item: dict[str, Any] | None = None,
) -> None:
    states = record.get("state_records", []) if state_records is None else state_records
    if not isinstance(states, list):
        raise RuntimeError("state records are not a list")
    executed_total = sum(
        int(record[f"{phase}_state_executed"])
        for phase in ("rest", "single", "sobol")
    )
    if len(states) != executed_total:
        raise RuntimeError(
            f"state record count {len(states)} != executed count {executed_total}"
        )
    phase_names = {
        "rest": "rest",
        "single": "single_joint_sweep",
        "sobol": "multi_joint_sobol",
    }
    for phase, phase_name in phase_names.items():
        observed = sum(state.get("phase") == phase_name for state in states)
        expected = int(record[f"{phase}_state_executed"])
        if observed != expected:
            raise RuntimeError(
                f"state phase count mismatch for {phase_name}: {observed} != {expected}"
            )
    keys = [
        (state.get("phase"), state.get("joint_name"), state.get("sample_index"))
        for state in states
    ]
    if len(keys) != len(set(keys)):
        raise RuntimeError("duplicate state identity")
    if any(str(state.get("dataset_id")) != str(record["dataset_id"]) for state in states):
        raise RuntimeError("state dataset identity mismatch")
    expected_category = record.get("category") if item is None else item.get("category")
    if any(state.get("category") != expected_category for state in states):
        raise RuntimeError("state category identity mismatch")
    if item is not None:
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
        if observed_state_identities != expected_state_identities[: len(states)]:
            raise RuntimeError("state identity does not match frozen sampling sequence")
    for state in states:
        phase = state.get("phase")
        for prefix in ("all_pair", "non_adjacent"):
            contact_count = int(state[f"{prefix}_contact_count"])
            illegal_count = int(state[f"{prefix}_illegal_penetration_count"])
            maximum = float(state[f"{prefix}_max_penetration_m"])
            if not 0 <= illegal_count <= contact_count or maximum < 0.0:
                raise RuntimeError("invalid collision observation counters")
        expected_metric = float(
            state[
                "all_pair_max_penetration_m"
                if phase == "rest"
                else "non_adjacent_max_penetration_m"
            ]
        )
        if not math.isclose(
            float(state["metric_max_penetration_m"]),
            expected_metric,
            rel_tol=0.0,
            abs_tol=1e-15,
        ):
            raise RuntimeError("state metric penetration policy mismatch")
    free_counts = {
        "rest": sum(
            state["non_adjacent_illegal_penetration_count"] == 0
            for state in states
            if state["phase"] == "rest"
        ),
        "single": sum(
            state["non_adjacent_illegal_penetration_count"] == 0
            for state in states
            if state["phase"] == "single_joint_sweep"
        ),
        "sobol": sum(
            state["non_adjacent_illegal_penetration_count"] == 0
            for state in states
            if state["phase"] == "multi_joint_sobol"
        ),
    }
    for phase, observed in free_counts.items():
        key = (
            "rest_non_adjacent_free"
            if phase == "rest"
            else f"{phase}_non_adjacent_free"
        )
        if observed != int(record[key]):
            raise RuntimeError(f"state free count mismatch for {phase}")
    digest = canonical_sha256(states)
    if digest != record.get("state_records_sha256"):
        raise RuntimeError("state record digest mismatch")
    if states:
        observed_max = max(float(state["metric_max_penetration_m"]) for state in states)
        if not math.isclose(
            observed_max,
            float(record["max_penetration_m"]),
            rel_tol=0.0,
            abs_tol=1e-15,
        ):
            raise RuntimeError("state maximum penetration mismatch")
        scale = float(record["object_bbox_diagonal_m"])
        if not math.isclose(
            observed_max / scale,
            float(record["max_penetration_normalized"]),
            rel_tol=1e-12,
            abs_tol=1e-15,
        ):
            raise RuntimeError("normalized maximum penetration mismatch")
        observed_readback = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
        if not math.isclose(
            observed_readback,
            float(record["max_reset_readback_error"]),
            rel_tol=0.0,
            abs_tol=1e-15,
        ):
            raise RuntimeError("state maximum reset readback mismatch")
    elif record.get("max_penetration_m") is not None:
        raise RuntimeError("empty state records have a maximum penetration")


def _result_counters_valid(
    result: dict[str, Any],
    state_records: list[dict[str, Any]] | None = None,
    item: dict[str, Any] | None = None,
) -> bool:
    try:
        for phase in ("rest", "single", "sobol"):
            expected = int(result[f"{phase}_state_expected"])
            executed = int(result[f"{phase}_state_executed"])
            free_key = (
                "rest_non_adjacent_free"
                if phase == "rest"
                else f"{phase}_non_adjacent_free"
            )
            free = int(result[free_key])
            if not 0 <= free <= executed <= expected:
                return False
        expected_total = sum(
            int(result[f"{phase}_state_expected"])
            for phase in ("rest", "single", "sobol")
        )
        executed_total = sum(
            int(result[f"{phase}_state_executed"])
            for phase in ("rest", "single", "sobol")
        )
        sampling_protocol = str(
            result.get("sampling_protocol", SAMPLING_PROTOCOL_V1)
        )
        if sampling_protocol == SAMPLING_PROTOCOL_V2:
            evaluable_count = int(
                result["range_evaluable_independent_dof_count"]
            )
            sampled_count = int(result["independent_dof_count"])
        else:
            evaluable_count = int(result["range_evaluable_dof_count"])
            sampled_count = int(result["movable_dof_count"])
        measurement_expected = bool(
            result["load_success"]
            and evaluable_count == sampled_count
            and executed_total == expected_total
        )
        if bool(result["measurement_complete"]) != measurement_expected:
            return False
        if bool(result["strict_collision_pass"]) and not bool(
            result["measurement_complete"]
        ):
            return False
        rest_all_pair_cf = bool(
            int(result["rest_state_executed"]) == int(result["rest_state_expected"])
            and all(
                int(state["all_pair_illegal_penetration_count"]) == 0
                for state in (
                    result.get("state_records", [])
                    if state_records is None
                    else state_records
                )
                if state.get("phase") == "rest"
            )
        )
        expected_flags = {
            "rest_all_pair_cf": bool(result["load_success"] and rest_all_pair_cf),
            "rest_non_adjacent_cf": (
                bool(result["load_success"])
                and int(result["rest_state_executed"])
                == int(result["rest_state_expected"])
                and int(result["rest_non_adjacent_free"])
                == int(result["rest_state_expected"])
            ),
            "single_joint_sweep_cf": (
                bool(result["load_success"])
                and int(result["single_state_executed"])
                == int(result["single_state_expected"])
                and int(result["single_non_adjacent_free"])
                == int(result["single_state_expected"])
            ),
            "multi_joint_sobol_cf": (
                evaluable_count == sampled_count
                and sampled_count > 0
                and int(result["sobol_state_executed"])
                == int(result["sobol_state_expected"])
                and int(result["sobol_non_adjacent_free"])
                == int(result["sobol_state_expected"])
            ),
        }
        for key, expected_flag in expected_flags.items():
            if bool(result[key]) != bool(expected_flag):
                return False
        states = (
            result.get("state_records", [])
            if state_records is None
            else state_records
        )
        single_by_joint: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for state in states:
            if state.get("phase") == "single_joint_sweep":
                single_by_joint[str(state.get("joint_name"))].append(state)
        joint_passes = sum(
            len(joint_states) == SINGLE_SAMPLES
            and all(
                int(state["non_adjacent_illegal_penetration_count"]) == 0
                for state in joint_states
            )
            for joint_states in single_by_joint.values()
        )
        if int(result["joint_single_sweep_cf_passed"]) != joint_passes:
            return False
        strict_expected = bool(
            result["measurement_complete"]
            and expected_flags["rest_non_adjacent_cf"]
            and expected_flags["single_joint_sweep_cf"]
            and expected_flags["multi_joint_sobol_cf"]
        )
        if bool(result["strict_collision_pass"]) != strict_expected:
            return False
        validate_state_closure(result, state_records, item)
    except (KeyError, RuntimeError, TypeError, ValueError):
        return False
    return True


def result_matches_item(
    result: dict[str, Any],
    item: dict[str, Any],
    runner_hash: str,
    state_records: list[dict[str, Any]] | None = None,
) -> bool:
    frozen_fields = (
        "protocol_id",
        "order",
        "dataset_id",
        "category",
        "input_identity_sha256",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
        "object_bbox_diagonal_m",
        "runtime_identity",
    )
    return bool(
        all(result.get(key) == item.get(key) for key in frozen_fields)
        and result.get("runner_sha256") == runner_hash
        and _result_counters_valid(result, state_records, item)
    )


def _valid_cached_child(path: Path, item: dict[str, Any], runner_hash: str) -> bool:
    if not path.is_file():
        return False
    try:
        result = read_json(path)
    except (OSError, json.JSONDecodeError):
        return False
    return result_matches_item(result, item, runner_hash)


def run_one_subprocess(
    item_path: Path,
    item: dict[str, Any],
    dataset_root: Path,
    child_result: Path,
    child_log: Path,
    timeout: int,
    python: Path,
    runner_hash: str,
) -> dict[str, Any]:
    if _valid_cached_child(child_result, item, runner_hash):
        asset_dir = dataset_root / str(item["dataset_id"])
        try:
            validate_frozen_asset_files(item, asset_dir)
        except Exception as exc:  # noqa: BLE001
            result = failure_record(
                item, f"frozen_asset_files_drift:{type(exc).__name__}: {exc}"
            )
            result["runner_sha256"] = runner_hash
            result["child_returncode"] = None
            result["child_timed_out"] = False
            result["child_log"] = str(child_log)
            result["cache_reused"] = False
            atomic_json(child_result, result)
            return result
        result = read_json(child_result)
        result["cache_reused"] = True
        return result
    command = [
        str(python),
        str(SCRIPT),
        "--phase",
        "child",
        "--dataset-root",
        str(dataset_root),
        "--child-item",
        str(item_path),
        "--child-result",
        str(child_result),
    ]
    environment = dict(os.environ)
    environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    child_log.parent.mkdir(parents=True, exist_ok=True)
    timed_out = False
    with child_log.open("wb") as log:
        try:
            completed = subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                env=environment,
                check=False,
            )
            returncode = completed.returncode
        except subprocess.TimeoutExpired:
            returncode = -9
            timed_out = True
    if returncode == 0 and _valid_cached_child(child_result, item, runner_hash):
        result = read_json(child_result)
    else:
        result = failure_record(
            item,
            "child_timeout" if timed_out else f"child_exit_{returncode}",
            timed_out=timed_out,
        )
        result["runner_sha256"] = runner_hash
        atomic_json(child_result, result)
    result["child_returncode"] = returncode
    result["child_timed_out"] = timed_out
    result["child_log"] = str(child_log)
    result["cache_reused"] = False
    atomic_json(child_result, result)
    return result


def execute(
    manifest: dict[str, Any],
    dataset_root: Path,
    output: Path,
    workers: int,
    timeout: int,
    python: Path,
) -> list[dict[str, Any]]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    run_pair_policy_smoke(output)
    child_runtime = probe_runtime_identity(
        python, output / "child_runtime_probe.json"
    )
    require_runtime_match(manifest["runtime"]["child"], child_runtime)
    inputs = output / "inputs"
    children = output / "children"
    logs = output / "child_logs"
    for directory in (inputs, children, logs):
        directory.mkdir(parents=True, exist_ok=True)
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    jobs = []
    for item in manifest["items"]:
        prefix = f"{int(item['order']):04d}_{item['dataset_id']}"
        item_path = inputs / f"{prefix}.json"
        child_result = children / f"{prefix}.json"
        child_log = logs / f"{prefix}.log"
        atomic_json(item_path, item)
        jobs.append((item, item_path, child_result, child_log))
    by_order: dict[int, dict[str, Any]] = {}
    started = time.time()
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                run_one_subprocess,
                item_path,
                item,
                dataset_root,
                child_result,
                child_log,
                timeout,
                python,
                runner_hash,
            ): item
            for item, item_path, child_result, child_log in jobs
        }
        for completed, future in enumerate(as_completed(futures), 1):
            item = futures[future]
            result = future.result()
            by_order[int(item["order"])] = result
            progress = {
                "protocol_id": manifest["protocol_id"],
                "status": "RUNNING" if completed < len(jobs) else "CHILDREN_COMPLETE",
                "completed_assets": completed,
                "total_assets": len(jobs),
                "load_successes": sum(row["load_success"] for row in by_order.values()),
                "measurement_complete_assets": sum(
                    row["measurement_complete"] for row in by_order.values()
                ),
                "timeouts": sum(row["child_timed_out"] for row in by_order.values()),
                "elapsed_seconds": time.time() - started,
                "last_completed_dataset_id": item["dataset_id"],
            }
            atomic_json(output / "progress.json", progress)
            print(
                f"table4 {completed}/{len(jobs)} id={item['dataset_id']} "
                f"load={int(result['load_success'])} complete={int(result['measurement_complete'])}",
                flush=True,
            )
    return [by_order[index] for index in range(len(jobs))]


def _pass_metric(records: list[dict[str, Any]], key: str) -> dict[str, Any]:
    passed = sum(bool(row.get(key)) for row in records)
    return {"passed": passed, "denominator": len(records), "rate": rate(passed, len(records))}


def summarize_records(
    manifest: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    if len(records) != int(manifest["sample_size"]):
        raise RuntimeError(
            f"record count {len(records)} != frozen sample {manifest['sample_size']}"
        )
    rest_expected = sum(int(row.get("rest_state_expected", 1)) for row in records)
    single_expected = sum(
        int(
            row.get(
                "single_state_expected",
                SINGLE_SAMPLES * int(row.get("movable_dof_count", 0)),
            )
        )
        for row in records
    )
    sobol_expected = sum(
        int(
            row.get(
                "sobol_state_expected",
                SOBOL_SAMPLES if int(row.get("movable_dof_count", 0)) > 0 else 0,
            )
        )
        for row in records
    )
    total_expected = rest_expected + single_expected + sobol_expected
    rest_free = sum(int(row["rest_non_adjacent_free"]) for row in records)
    single_free = sum(int(row["single_non_adjacent_free"]) for row in records)
    sobol_free = sum(int(row["sobol_non_adjacent_free"]) for row in records)
    free_total = rest_free + single_free + sobol_free
    executed_total = sum(
        int(row["rest_state_executed"])
        + int(row["single_state_executed"])
        + int(row["sobol_state_executed"])
        for row in records
    )
    observed_collision = sum(
        int(row["rest_state_executed"])
        + int(row["single_state_executed"])
        + int(row["sobol_state_executed"])
        - int(row["rest_non_adjacent_free"])
        - int(row["single_non_adjacent_free"])
        - int(row["sobol_non_adjacent_free"])
        for row in records
    )
    max_values = [
        float(row["max_penetration_normalized"])
        for row in records
        if row.get("max_penetration_normalized") is not None
    ]
    fully_measured_max_values = [
        float(row["max_penetration_normalized"])
        for row in records
        if row.get("max_penetration_normalized") is not None
        and bool(row.get("measurement_complete"))
    ]
    metrics: dict[str, Any] = {
        "rest_all_pair_cf": _pass_metric(records, "rest_all_pair_cf"),
        "rest_non_adjacent_cf": _pass_metric(records, "rest_non_adjacent_cf"),
        "single_joint_sweep_cf": _pass_metric(records, "single_joint_sweep_cf"),
        "multi_joint_sobol_cf": _pass_metric(records, "multi_joint_sobol_cf"),
        "collision_state_rate": {
            "collision_states": total_expected - free_total,
            "denominator": total_expected,
            "rate": rate(total_expected - free_total, total_expected),
            "definition": "fail-closed collision-or-unexecuted configurations / frozen expected configurations",
            "observed_collision_states": observed_collision,
            "executed_states": executed_total,
            "unexecuted_states": total_expected - executed_total,
            "observed_collision_rate_executed": rate(observed_collision, executed_total),
        },
        "aor": {
            "status": "N/E",
            "reason": "no stable exact overlap-volume implementation; bounding-box overlap is not substituted",
        },
        "max_penetration": {
            "maximum_observed_normalized": max(max_values) if max_values else None,
            "observed_assets": len(max_values),
            "fully_measured_assets": len(fully_measured_max_values),
            "denominator": len(records),
            "status": (
                "COMPLETE"
                if len(fully_measured_max_values) == len(records)
                else "PARTIAL"
            ),
            "normalization": "release bounding_box.json diagonal",
        },
        "collision_free_range": {
            "passed_states": single_free,
            "denominator": single_expected,
            "rate": rate(single_free, single_expected),
        },
        "strict_collision_pass": _pass_metric(records, "strict_collision_pass"),
    }
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        by_category[str(row.get("category") or "__PACKAGE_AUDIT_FAILED__")].append(row)
    category_results = {
        category: summarize_records(
            {"sample_size": len(rows)}, rows
        )["metrics"]
        for category, rows in sorted(by_category.items())
    } if len(by_category) > 1 else {}
    macro_keys = (
        "rest_all_pair_cf",
        "rest_non_adjacent_cf",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
        "strict_collision_pass",
    )
    category_macro = {}
    if category_results:
        for key in macro_keys:
            rates = [
                value[key]["rate"]
                for value in category_results.values()
                if value[key]["rate"] is not None
            ]
            category_macro[key] = sum(rates) / len(rates) if rates else None
        range_rates = [
            value["collision_free_range"]["rate"]
            for value in category_results.values()
            if value["collision_free_range"]["rate"] is not None
        ]
        category_macro["collision_free_range"] = (
            sum(range_rates) / len(range_rates) if range_rates else None
        )
    return {
        "protocol_id": manifest.get("protocol_id", PROTOCOL_ID),
        "status": (
            "COMPLETE"
            if all(row["measurement_complete"] for row in records)
            else "COMPLETE_WITH_RETAINED_FAILURES"
        ),
        "cohort": {
            "label": manifest.get(
                "cohort_label", "PartNet-Mobility N=800 sampled release cohort"
            ),
            "selected": len(records),
            "load_success": sum(row["load_success"] for row in records),
            "measurement_complete": sum(
                row["measurement_complete"] for row in records
            ),
            "category_count": len(by_category),
            "child_timeouts": sum(row["child_timed_out"] for row in records),
        },
        "metrics": metrics,
        "category_macro": category_macro,
        "category_results": category_results,
        "claim_boundary": {
            "continuous_collision_detection": "not_run",
            "semantic_joint_correctness": "not_evaluated",
            "physical_dynamics_validity": "not_evaluated",
            "full_release_result": False,
            "shared_category_balanced_result": False,
        },
    }


def summarize(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    records = []
    state_records = []
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    for item in manifest["items"]:
        path = output / "children" / f"{int(item['order']):04d}_{item['dataset_id']}.json"
        if not _valid_cached_child(path, item, runner_hash):
            raise RuntimeError(f"missing or stale child result: {path}")
        row = read_json(path)
        asset_states = row.pop("state_records", [])
        if not result_matches_item(row, item, runner_hash, asset_states):
            raise RuntimeError(f"child state closure mismatch: {path}")
        state_records.extend(asset_states)
        records.append(row)
    summary = summarize_records(manifest, records)
    atomic_json(output / "asset_records.json", records)
    atomic_jsonl(output / "state_records.jsonl", state_records)
    atomic_json(output / "summary.json", summary)
    render_report(summary, output)
    return summary


def _format_metric(row: dict[str, Any], numerator_key: str = "passed") -> str:
    numerator = row[numerator_key]
    denominator = row["denominator"]
    percentage = 100.0 * row["rate"] if row["rate"] is not None else float("nan")
    return f"{numerator}/{denominator} ({percentage:.3f}%)"


def report_text(summary: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    cohort_label = str(summary["cohort"]["label"])
    maximum = metrics["max_penetration"]
    lines = [
        f"# {cohort_label}: URDF Sim-Ready Table 4",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is an outcome-independent frozen cohort diagnostic. It is not the 2,347-asset Full Release panel and not the six-method shared-category balanced panel.",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Rest All-pair CF | {_format_metric(metrics['rest_all_pair_cf'])} |",
        f"| Rest Non-adjacent CF | {_format_metric(metrics['rest_non_adjacent_cf'])} |",
        f"| Single-joint Sweep CF | {_format_metric(metrics['single_joint_sweep_cf'])} |",
        f"| Multi-joint Sobol CF | {_format_metric(metrics['multi_joint_sobol_cf'])} |",
        f"| Collision-state Rate | {_format_metric(metrics['collision_state_rate'], 'collision_states')} |",
        "| AOR | N/E |",
        f"| Max Penetration | {maximum['maximum_observed_normalized']} (fully measured {maximum['fully_measured_assets']}/{maximum['denominator']}; observed {maximum['observed_assets']}/{maximum['denominator']}; {maximum['status']}) |",
        f"| Collision-free Range | {_format_metric(metrics['collision_free_range'], 'passed_states')} |",
        f"| Strict Collision Pass | {_format_metric(metrics['strict_collision_pass'])} |",
        "",
        "Collision-state Rate is fail-closed: unexecuted configurations caused by package, load, child, or timeout failures remain in the denominator and count as non-free. The report separately preserves observed collisions, executed states, and unexecuted states.",
        "",
        "AOR is N/E because no stable exact overlap-volume calculation was run; bounding-box overlap is not used as a substitute. All sweeps are discrete, with no CCD claim.",
    ]
    return "\n".join(lines) + "\n"


def render_report(summary: dict[str, Any], output: Path) -> None:
    atomic_text(output / "report.md", report_text(summary))


def verify(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    summary = read_json(output / "summary.json")
    records = read_json(output / "asset_records.json")
    state_records = []
    with (output / "state_records.jsonl").open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise RuntimeError(
                    f"blank line in state_records.jsonl at line {line_number}"
                )
            state_records.append(json.loads(line))
    states_by_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for state in state_records:
        states_by_id[str(state.get("dataset_id"))].append(state)
    manifest_ids = {str(item["dataset_id"]) for item in manifest["items"]}
    unknown_state_ids = sorted(set(states_by_id) - manifest_ids)
    state_closure_matches = not unknown_state_ids
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    for item, record in zip(manifest["items"], records):
        asset_states = states_by_id.get(str(item["dataset_id"]), [])
        state_closure_matches = state_closure_matches and result_matches_item(
            record, item, runner_hash, asset_states
        )
    expected_state_order = [
        state
        for item in manifest["items"]
        for state in states_by_id.get(str(item["dataset_id"]), [])
    ]
    state_closure_matches = (
        state_closure_matches and expected_state_order == state_records
    )
    frozen_asset_files_match = True
    dataset_root = Path(manifest["dataset_root"])
    for item in manifest["items"]:
        try:
            validate_frozen_asset_files(
                item, dataset_root / str(item["dataset_id"])
            )
        except (OSError, RuntimeError, ValueError, ET.ParseError):
            frozen_asset_files_match = False
            break
    recomputed = summarize_records(manifest, records)
    runtime_probe = read_json(output / "child_runtime_probe.json")
    checks = {
        "protocol_matches": summary.get("protocol_id") == manifest["protocol_id"],
        "sample_size_exact": len(records) == manifest["sample_size"],
        "record_order_matches_manifest": [row["dataset_id"] for row in records]
        == [item["dataset_id"] for item in manifest["items"]],
        "summary_recomputes_exactly": summary == recomputed,
        "report_recomputes_exactly": (output / "report.md").read_text(
            encoding="utf-8"
        )
        == report_text(summary),
        "state_records_close_against_assets": state_closure_matches,
        "state_record_count_matches_assets": len(state_records)
        == sum(
            int(record[f"{phase}_state_executed"])
            for record in records
            for phase in ("rest", "single", "sobol")
        ),
        "frozen_asset_files_match": frozen_asset_files_match,
        "child_runtime_matches_manifest": runtime_probe
        == manifest["runtime"]["child"],
        "single_state_denominator_frozen": sum(
            row["single_state_expected"] for row in records
        )
        == SINGLE_SAMPLES
        * sum(item["movable_dof_count"] for item in manifest["items"]),
        "sobol_state_denominator_frozen": sum(
            row["sobol_state_expected"] for row in records
        )
        == SOBOL_SAMPLES
        * sum(item["movable_dof_count"] > 0 for item in manifest["items"]),
        "aor_remains_not_evaluable": summary["metrics"]["aor"]["status"] == "N/E",
        "no_full_release_claim": summary["claim_boundary"]["full_release_result"] is False,
        "pair_policy_smoke_pass": read_json(output / "pair_policy_smoke.json")["status"]
        == "PASS",
    }
    artifacts = (
        "frozen_manifest.json",
        "child_runtime_probe.json",
        "pair_policy_smoke.json",
        "asset_records.json",
        "state_records.jsonl",
        "summary.json",
        "report.md",
    )
    receipt = {
        "protocol_id": "urdf_sim_ready_table4_partnet_mobility_verify_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "artifact_sha256": {
            name: sha256_file(output / name) for name in artifacts
        },
        "runner_sha256": sha256_file(SCRIPT),
    }
    atomic_json(output / "verification.json", receipt)
    if receipt["status"] != "PASS":
        raise RuntimeError(f"verification failed: {checks}")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "prepare", "run", "summarize", "verify", "child", "runtime"),
        default="all",
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sample-size", type=int, default=SAMPLE_SIZE)
    parser.add_argument("--qualification-smoke", action="store_true")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--child-item", type=Path)
    parser.add_argument("--child-result", type=Path)
    parser.add_argument("--runtime-result", type=Path)
    args = parser.parse_args()

    if args.phase == "runtime":
        if args.runtime_result is None:
            parser.error("runtime phase requires --runtime-result")
        atomic_json(args.runtime_result, current_runtime_identity())
        return 0

    dataset_root = args.dataset_root.resolve(strict=True)
    if args.phase == "child":
        if args.child_item is None or args.child_result is None:
            parser.error("child phase requires --child-item and --child-result")
        run_child(args.child_item, dataset_root, args.child_result)
        return 0

    archive = args.archive.resolve(strict=True)
    output = args.output.resolve()
    child_python = normalize_executable_path(args.python, Path.cwd())
    child_runtime = probe_runtime_identity(
        child_python, output / "child_runtime_probe.json"
    )
    manifest: dict[str, Any]
    if args.phase in {"all", "prepare"}:
        manifest = prepare(
            dataset_root,
            archive,
            output,
            args.sample_size,
            args.qualification_smoke,
            child_python,
        )
        print(
            json.dumps(
                {
                    "manifest": str(output / "frozen_manifest.json"),
                    "sample_size": manifest["sample_size"],
                    "selected_ids_sha256": manifest["ordered_selected_ids_sha256"],
                },
                indent=2,
            ),
            flush=True,
        )
        if args.phase == "prepare":
            return 0
    else:
        manifest = read_json(output / "frozen_manifest.json")
        validate_manifest(
            manifest,
            dataset_root,
            archive,
            qualification_smoke=args.qualification_smoke,
            child_runtime=child_runtime,
        )

    if args.phase in {"all", "run"}:
        execute(
            manifest,
            dataset_root,
            output,
            args.workers,
            args.timeout,
            child_python,
        )
        if args.phase == "run":
            return 0
    if args.phase in {"all", "summarize"}:
        summary = summarize(manifest, output)
        print(json.dumps(summary["cohort"], indent=2), flush=True)
        if args.phase == "summarize":
            return 0
    if args.phase in {"all", "verify"}:
        receipt = verify(manifest, output)
        print(json.dumps(receipt, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
