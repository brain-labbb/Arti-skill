#!/usr/bin/env python3
"""Method-agnostic Table 2 supplementary static evaluator.

Implements the four frozen Table 2 supplementary metrics from
``exp/URDF-Sim-Ready-Automatic-Evaluation.md``:

- ``Visual-bearing Collision Coverage``
- ``Joint-limit Portability``
- ``Joint Dynamics Coverage``
- ``Placeholder-mass Incidence``

The atom implementations are imported from ``lam_supplementary_static`` so that
every dataset is scored by exactly the same code version (the protocol requires
a single evaluator version for all methods).  This wrapper removes the
LAM-specific S1 evidence atoms and keeps only the Table 2 supplementary atoms,
so the same auditor can be reused for Artiverse, Articraft-10K,
PartNet-Mobility, PhysX-Mobility and Ours-500K without modification.

Fail-closed policy: package preflight failures, XML parse failures, link or
joint extraction problems and missing/unloadable resources all remain asset-
or joint-level failures and are never dropped from the denominator.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

SCHEMA_VERSION = "table2-supplementary-static/v1"


def _load_lam_static():
    """Load the frozen LAM static atom module (single source of truth)."""

    try:
        from exp.scripts import lam_supplementary_static as module  # noqa: PLC0415

        return module
    except Exception:  # noqa: BLE001 - fall back to direct file loading
        target = SCRIPT.with_name("lam_supplementary_static.py")
        spec = importlib.util.spec_from_file_location("lam_supplementary_static", target)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load lam_supplementary_static from {target}") from None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


lam_static = _load_lam_static()

# Re-exported helpers (identical code paths for every method).
sha256_file = lam_static.sha256_file
canonical_sha256 = lam_static.canonical_sha256
normalize_placeholder_registry = lam_static.normalize_placeholder_registry
safe_package_relative_path = lam_static.safe_package_relative_path


def _failed_record(
    *,
    package: Path,
    asset_id: str,
    urdf_relative_path: str,
    expected_movable_joints: int | None,
    issue: str,
) -> dict[str, Any]:
    intended_joints = expected_movable_joints if expected_movable_joints is not None else 0
    return {
        "schema_version": SCHEMA_VERSION,
        "asset_id": asset_id,
        "package": str(package),
        "urdf_relative_path": urdf_relative_path,
        "urdf_sha256": None,
        "status": "error",
        "parse": {"success": False, "issues": [issue]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE",
                "asset_intended": 1,
                "asset_passed": 0,
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [],
                "issues": [issue],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended_joints,
                "joints_extracted": 0,
                "joints_passed": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended_joints,
                "joints_extracted": 0,
                "joints_covered": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0,
                "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None,
                "incidence_numerator": None,
                "incidence_denominator": 0,
                "registry_ids": [],
                "link_records": [],
                "incomplete_inertial_links": [],
                "issues": [issue],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE",
            "complete": False,
            "file_count": 0,
            "sha256": None,
            "files": [],
            "issues": [issue],
        },
        "issues": [issue],
    }


def audit_package(
    package: Path | str,
    *,
    urdf_relative_path: str,
    asset_id: str | None = None,
    expected_movable_joints: int | None = None,
    placeholder_registry: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return Table 2 supplementary static atoms for one URDF package.

    The caller owns the frozen cohort, the expected movable-joint denominator
    (``J_eval`` contribution of this asset) and the placeholder registry.  Any
    path, XML or extraction failure remains an asset-level failure.
    """

    package_raw = Path(package)
    resolved_asset_id = asset_id if asset_id is not None else package_raw.name
    try:
        if expected_movable_joints is not None and (
            isinstance(expected_movable_joints, bool)
            or not isinstance(expected_movable_joints, int)
            or expected_movable_joints < 0
        ):
            raise ValueError("expected_movable_joints must be a non-negative integer or None")
        registry = normalize_placeholder_registry(placeholder_registry)
        if package_raw.is_symlink():
            raise ValueError("package_is_symlink")
        package_resolved = package_raw.resolve(strict=True)
        if not package_resolved.is_dir():
            raise NotADirectoryError(package_resolved)
        urdf_path = lam_static._primary_urdf(package_resolved, urdf_relative_path)
    except Exception as exc:  # noqa: BLE001
        return _failed_record(
            package=package_raw,
            asset_id=resolved_asset_id,
            urdf_relative_path=urdf_relative_path,
            expected_movable_joints=expected_movable_joints,
            issue=f"package_preflight_failed: {type(exc).__name__}: {exc}",
        )

    urdf_sha256 = sha256_file(urdf_path)
    try:
        root = ET.parse(urdf_path).getroot()
        if lam_static.local_tag(root) != "robot":
            raise ValueError(f"root_element_not_robot: {lam_static.local_tag(root)}")
    except Exception as exc:  # noqa: BLE001
        record = _failed_record(
            package=package_resolved,
            asset_id=resolved_asset_id,
            urdf_relative_path=urdf_relative_path,
            expected_movable_joints=expected_movable_joints,
            issue=f"xml_parse_failed: {type(exc).__name__}: {exc}",
        )
        record["urdf_sha256"] = urdf_sha256
        return record

    link_names = [link.attrib.get("name", "").strip() for link in lam_static.children(root, "link")]
    link_issues: list[str] = []
    if not link_names:
        link_issues.append("no_declared_links")
    if any(not name for name in link_names):
        link_issues.append("unnamed_link")
    if len(set(link_names)) != len(link_names):
        link_issues.append("duplicate_link_name")
    link_extraction_complete = not link_issues

    visual_collision = lam_static._visual_collision_atoms(
        root,
        package_resolved,
        urdf_path,
        link_extraction_complete=link_extraction_complete,
    )
    joint_limit, joint_dynamics, joint_extraction_issues = lam_static._joint_atoms(
        root,
        expected_movable_joints=expected_movable_joints,
    )
    placeholder = lam_static._placeholder_mass_atoms(root, registry)
    resource_closure = lam_static._resource_closure(root, package_resolved, urdf_path)
    issues = [
        *link_issues,
        *joint_extraction_issues,
        *resource_closure["issues"],
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "asset_id": resolved_asset_id,
        "package": str(package_resolved),
        "urdf_relative_path": urdf_relative_path,
        "urdf_sha256": urdf_sha256,
        "status": "completed",
        "parse": {"success": True, "issues": []},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": visual_collision,
            "joint_limit_portability": joint_limit,
            "joint_dynamics_coverage": joint_dynamics,
            "placeholder_mass_incidence": placeholder,
        },
        "resource_closure": resource_closure,
        "issues": issues,
    }


def audit_worker(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Stable, importable worker entry point used by the multiprocessing pool."""

    import time as _time

    started = _time.monotonic()
    record = audit_package(
        payload["package"],
        urdf_relative_path=payload["primary_urdf_relative_path"],
        asset_id=payload["asset_id"],
        expected_movable_joints=payload["expected_declared_joint_count"],
        placeholder_registry=payload.get("placeholder_registry") or [],
    )
    record["worker_elapsed_seconds"] = _time.monotonic() - started
    return record


__all__ = [
    "SCHEMA_VERSION",
    "audit_worker",
    "audit_package",
    "canonical_sha256",
    "normalize_placeholder_registry",
    "safe_package_relative_path",
    "sha256_file",
]
