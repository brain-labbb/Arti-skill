#!/usr/bin/env python3
"""Build the frozen PV-A N=200 Table 5 manifest with physics.json overlays."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO_ROOT = SCRIPT_PATH.parents[2]
RUN_ROOT = REPO_ROOT / "exp/runtime/table5_pva_n200_physics_v2_20260828"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_pva_n200_manifest as _base  # noqa: E402
from table5_pva_physics import (  # noqa: E402
    PLAN_SCHEMA,
    POLICY_ID,
    SIDECAR_SCHEMA,
    build_injected_asset,
    sha256_file,
)


MANIFEST_SCHEMA = "table5_pva_physics_n200_manifest_v2"
PROTOCOL_SCHEMA = "table5_pva_physics_protocol_v2"
PROTOCOL_ID = "table5-pva-prefix-physics-injection-v2"
SAMPLE_SIZE = _base.SAMPLE_SIZE
ROSTER_PATH = _base.ROSTER_PATH
EVALUATION_ROOT = _base.EVALUATION_ROOT


class ManifestError(ValueError):
    """Raised when the physics-bound manifest cannot be built exactly."""


def _implementation_receipt() -> dict[str, Any]:
    runtime = SCRIPT_PATH.with_name("table5_pva_physics_n200_runtime.py")
    helper = SCRIPT_PATH.with_name("table5_pva_physics.py")
    generic_runtime = SCRIPT_PATH.with_name("table5_n200_runtime.py")
    evaluator = SCRIPT_PATH.with_name("run_table5_sketch_mobility.py")
    for path in (runtime, helper, generic_runtime, evaluator):
        if not path.is_file():
            raise ManifestError(f"physics Table 5 implementation is missing: {path}")
    return {
        "runtime_script": str(generic_runtime),
        "runtime_script_sha256": sha256_file(generic_runtime),
        "evaluator_script": str(evaluator),
        "evaluator_script_sha256": sha256_file(evaluator),
        "physics_runtime_script": str(runtime),
        "physics_runtime_script_sha256": sha256_file(runtime),
        "physics_overlay_script": str(helper),
        "physics_overlay_script_sha256": sha256_file(helper),
        "reuse": [
            "table5_n200_runtime.run_manifest",
            "evaluate_asset",
            "PyBulletAdapter",
            "MuJoCoAdapter",
            "DynamicGenesisAdapter",
        ],
    }


def _physics_protocol(base_protocol: dict[str, Any]) -> dict[str, Any]:
    protocol = copy.deepcopy(base_protocol)
    protocol.update(
        {
            "schema_version": PROTOCOL_SCHEMA,
            "protocol_id": PROTOCOL_ID,
            "physics_injection": {
                "required_sidecar_schema": SIDECAR_SCHEMA,
                "policy_id": POLICY_ID,
                "source_binding": "model_urdf_sha256 must exactly match source URDF",
                "surface_binding": (
                    "exact one-to-one coverage of named visual and collision surface keys"
                ),
                "inertial_policy": {
                    "valid_source": "preserve without density override",
                    "missing_or_invalid": (
                        "sum collision solid mass properties using bound density and "
                        "the parallel-axis theorem"
                    ),
                    "primitive_geometry": "analytic solid mass properties",
                    "watertight_mesh": "exact signed-volume mass properties",
                    "negative_mesh_winding": "invert before mass property use",
                    "non_watertight_mesh": "convex hull fallback for mass properties",
                    "mesh_contact_area": "original scaled collision triangle surface area",
                },
                "common_contact_policy": {
                    "field": "dynamic_friction_coefficient",
                    "reduction": "collision surface-area weighted mean per URDF link",
                    "granularity": "link in all three simulators",
                    "pybullet": "changeDynamics lateralFriction",
                    "genesis": "RigidLink.set_friction",
                    "mujoco": "geom_friction sliding component for every collision geom",
                },
                "unsupported_common_fields": {
                    "youngs_modulus_pa": "no exact common rigid contact API",
                    "poissons_ratio": "no exact common rigid contact API",
                    "static_friction_coefficient": "no separate common static coefficient API",
                    "restitution_coefficient": "no exact common scalar semantics",
                },
                "joint_dynamics": (
                    "physics.json contains no joint damping or joint friction; no value invented"
                ),
                "derived_plan_schema": PLAN_SCHEMA,
            },
        }
    )
    for simulator in ("pybullet", "genesis", "mujoco"):
        protocol["adapters"][simulator]["inertials"] = (
            "physics overlay: preserve valid source, derive only missing or invalid"
        )
        protocol["adapters"][simulator]["contact_friction"] = (
            "physics plan link-level dynamic coefficient"
        )
    protocol["implementation"] = _implementation_receipt()
    resume = protocol["artifacts"].setdefault("resume_binding", [])
    for field in (
        "identity.source_urdf_sha256",
        "identity.physics_sha256",
        "identity.physics_plan_sha256",
        "identity.physics_policy_id",
    ):
        if field not in resume:
            resume.append(field)
    protocol["protocol_sha256"] = _base.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def build_manifest(
    *,
    out_path: Path,
    roster_path: Path = ROSTER_PATH,
    evaluation_root: Path = EVALUATION_ROOT,
    sample_size: int = SAMPLE_SIZE,
) -> dict[str, Any]:
    if sample_size != SAMPLE_SIZE:
        raise ManifestError("the physics-bound PV-A cohort is fixed at N=200")
    base = _base.build_manifest(
        roster_path=roster_path,
        evaluation_root=evaluation_root,
        sample_size=sample_size,
    )
    rows = base["datasets"][0]["rows"]
    injected_root = out_path.resolve(strict=False).parent / "injected_assets"
    totals = {
        "binding_count": 0,
        "link_count": 0,
        "preserved_inertial_link_count": 0,
        "derived_inertial_link_count": 0,
        "convex_hull_fallback_collision_count": 0,
    }
    for row in rows:
        source_urdf = Path(row["urdf_path"]).resolve(strict=True)
        source_hash = row["urdf_sha256"]
        physics_path = (Path(row["package_root"]) / "physics.json").resolve(strict=True)
        asset_root = injected_root / row["dataset_id"]
        injected_urdf = asset_root / "model.physics.urdf"
        plan_path = asset_root / "physics_plan.json"
        plan = build_injected_asset(
            source_urdf=source_urdf,
            physics_path=physics_path,
            destination_urdf=injected_urdf,
            plan_path=plan_path,
        )
        if plan["source_urdf_sha256"] != source_hash:
            raise ManifestError(f"source identity drift for {row['dataset_id']}")
        row["source_urdf"] = {
            "path": str(source_urdf),
            "sha256": source_hash,
        }
        row["physics_sidecar"] = {
            "path": str(physics_path),
            "sha256": plan["physics_sha256"],
            "schema_version": plan["physics_schema_version"],
            "model_urdf_sha256": plan["physics_model_urdf_sha256"],
            "binding_count": plan["binding_count"],
        }
        row["physics_injection"] = {
            "policy_id": plan["policy_id"],
            "plan_schema_version": plan["schema_version"],
            "plan_path": str(plan_path.resolve()),
            "plan_sha256": plan["plan_sha256"],
            "injected_urdf_path": str(injected_urdf.resolve()),
            "injected_urdf_sha256": plan["injected_urdf_sha256"],
            "link_count": plan["link_count"],
            "preserved_inertial_link_count": plan[
                "preserved_inertial_link_count"
            ],
            "derived_inertial_link_count": plan["derived_inertial_link_count"],
            "convex_hull_fallback_collision_count": plan[
                "convex_hull_fallback_collision_count"
            ],
        }
        row["urdf_path"] = str(injected_urdf.resolve())
        row["urdf_sha256"] = plan["injected_urdf_sha256"]
        row["row_sha256"] = _base.canonical_sha256(
            row, exclude_fields=("row_sha256",)
        )
        for field in totals:
            totals[field] += int(plan[field])

    protocol = _physics_protocol(base["protocol"])
    base.update(
        {
            "schema_version": MANIFEST_SCHEMA,
            "manifest_title": (
                "PV-A formal full-release prefix Table 5 N=200 with physics.json injection"
            ),
            "protocol": protocol,
            "protocol_sha256": protocol["protocol_sha256"],
            "physics_injection": {
                "policy_id": POLICY_ID,
                "sidecar_schema": SIDECAR_SCHEMA,
                "plan_schema": PLAN_SCHEMA,
                "injected_asset_root": str(injected_root.resolve()),
                "totals": totals,
            },
            "generation": {
                "script": str(SCRIPT_PATH),
                "script_sha256": sha256_file(SCRIPT_PATH),
                "base_manifest_script": str(_base.SCRIPT_PATH),
                "base_manifest_script_sha256": sha256_file(_base.SCRIPT_PATH),
                "static_parser_issue_rows": base.get("generation", {}).get(
                    "static_parser_issue_rows"
                ),
            },
        }
    )
    base["manifest_sha256"] = _base.canonical_sha256(
        base, exclude_fields=("manifest_sha256",)
    )
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=RUN_ROOT / "manifest.json")
    parser.add_argument("--roster", type=Path, default=ROSTER_PATH)
    parser.add_argument("--evaluation-root", type=Path, default=EVALUATION_ROOT)
    args = parser.parse_args()
    output = args.out.resolve(strict=False)
    manifest = build_manifest(
        out_path=output,
        roster_path=args.roster.resolve(),
        evaluation_root=args.evaluation_root.resolve(),
    )
    _base._atomic_write_json(output, manifest)
    print(
        json.dumps(
            {
                "manifest": str(output),
                "manifest_sha256": manifest["manifest_sha256"],
                "protocol_sha256": manifest["protocol_sha256"],
                "rows": manifest["total_rows"],
                "physics_totals": manifest["physics_injection"]["totals"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ManifestError, _base.ManifestError) as error:
        print(f"table5_pva_physics_n200_manifest: {error}", file=sys.stderr)
        raise SystemExit(2)
