#!/usr/bin/env python3
"""Build a standardized, fully inertial T5 evaluation copy without changing source assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_INPUT = EXP_ROOT / "runtime/t5_formal_v1/gold_then_compile/formal_input_manifest.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/simulation_ready"
ALUMINUM = "freecad:9bf060e9-1663-44a2-88e2-2ff6ee858efe"

sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(TEMPLATE_ROOT))

import run_nano3d_editability as compiler  # noqa: E402
from run_nano3d_articulation_paper import parse_joint_metadata, parse_physical_metadata  # noqa: E402


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


def wrapper_text(source_wrapper: Path) -> str:
    return f'''from __future__ import annotations

import runpy
from sdk import AssetContext, ArticulationType, BulkMaterialSlot, MotionProperties

_source_namespace = runpy.run_path({str(source_wrapper)!r})
_template_module = _source_namespace.get("_template_module")
CONFIG = _source_namespace.get("CONFIG")
SEED = _source_namespace.get("SEED", 0)
object_model = _source_namespace["object_model"]
object_model.set_assets(AssetContext.from_script(__file__))

# Standardized evaluation prior: every rigid link is recomputed atomically from
# its current geometry with one documented aluminum density. This is an explicit
# simulator-control copy, not a claim about the object's real-world material.
for _part in object_model.parts:
    _part.inertial = None
    _part.bulk_material = BulkMaterialSlot(
        material_id={ALUMINUM!r},
        geometry_mode="solid",
    )

for _joint in object_model.articulations:
    if _joint.articulation_type != ArticulationType.FIXED:
        _joint.motion_properties = MotionProperties(damping=0.05, friction=0.01)

def run_tests():
    return _source_namespace["run_tests"]()
'''


def stage(row: dict[str, Any], output: Path) -> dict[str, Any]:
    asset_id = row["asset_id"]
    source = Path(row["copied_package"])
    package = output / "packages" / asset_id
    package.mkdir(parents=True, exist_ok=True)
    assets = source / "assets"
    if assets.is_dir() and not (package / "assets").exists():
        shutil.copytree(assets, package / "assets")
    wrapper = package / "model.py"
    expected = wrapper_text(source / "model.py")
    if wrapper.exists() and wrapper.read_text(encoding="utf-8") != expected:
        raise RuntimeError(f"resume refused; simulation wrapper differs: {wrapper}")
    if not wrapper.exists():
        wrapper.write_text(expected, encoding="utf-8")
    return {
        "asset_id": asset_id,
        "source_package": row["source_package"],
        "source_formal_package": str(source),
        "copied_package": str(package),
        "wrapper_sha256": sha256(wrapper),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    rows = json.loads(args.input_manifest.read_text(encoding="utf-8"))
    staged = [stage(row, output) for row in rows]

    results: dict[str, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {}
        for row in staged:
            package = Path(row["copied_package"])
            result_path = package / "compile_result.json"
            if result_path.is_file():
                results[row["asset_id"]] = json.loads(result_path.read_text(encoding="utf-8"))
                continue
            future = executor.submit(
                compiler.run_one, package / "model.py", result_path, args.timeout
            )
            futures[future] = row["asset_id"]
        for future in as_completed(futures):
            asset_id = futures[future]
            results[asset_id] = future.result()
            print(asset_id, results[asset_id].get("verdict"), flush=True)

    records: list[dict[str, Any]] = []
    final_manifest: list[dict[str, Any]] = []
    for row in staged:
        asset_id = row["asset_id"]
        package = Path(row["copied_package"])
        urdf = package / "model.urdf"
        physical = parse_physical_metadata(package) if urdf.is_file() else {}
        joints = parse_joint_metadata(package) if urdf.is_file() else []
        record = {
            **row,
            "compile_result": results[asset_id],
            "model_urdf_sha256": sha256(urdf) if urdf.is_file() else None,
            "movable_joint_count": len(joints),
            **physical,
        }
        records.append(record)
        if urdf.is_file():
            final_manifest.append(
                {
                    "asset_id": asset_id,
                    "source_package": row["source_package"],
                    "copied_package": str(package),
                    "model_urdf_sha256": sha256(urdf),
                }
            )
    dump_json(output / "asset_records.json", records)
    dump_json(output / "simulation_input_manifest.json", final_manifest)
    summary = {
        "schema_version": 1,
        "protocol": "t5_standardized_simulation_material_prior_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "asset_count": len(records),
        "compile_success": sum(row["compile_result"].get("verdict") == "pass" for row in records),
        "link_count": sum(row.get("link_count", 0) for row in records),
        "valid_inertial_links": sum(row.get("valid_inertial_link_count", 0) for row in records),
        "collision_links": sum(row.get("collision_link_count", 0) for row in records),
        "physical_metadata_complete_links": sum(row.get("physical_metadata_complete_link_count", 0) for row in records),
        "physical_metadata_complete_assets": sum(row.get("physical_metadata_complete_asset", False) for row in records),
        "movable_joint_count": sum(row.get("movable_joint_count", 0) for row in records),
        "material_prior": {
            "bulk_material_id": ALUMINUM,
            "geometry_mode": "solid",
            "joint_damping": 0.05,
            "joint_friction": 0.01,
            "scope": "standardized simulator-control copy only",
        },
    }
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["compile_success"] == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
