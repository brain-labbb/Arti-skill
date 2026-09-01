#!/usr/bin/env python3
"""Backfill the Table 7 edge-manifold proxy for the frozen PV-A N=33 pilot.

The original pilot predates the shared Table 7 protocol and did not record the
edge-incidence manifold proxy.  Ten original package paths are no longer
present.  This audit therefore reads the preserved N=33 package copies used by
the frozen URDF-to-GLB evaluation, but accepts them only after their identities
and URDF hashes match the original pilot manifest and the legacy geometry
statistics reproduce exactly.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from statistics import fmean
from typing import Any

import numpy as np
import trimesh


REPO = Path("/mnt/zsn/lyb/arti-skill").resolve()
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
LEGACY_ROOT = REPO / "exp/runtime/nano3d_asset_pilot"
LEGACY_MANIFEST = LEGACY_ROOT / "asset_manifest.jsonl"
LEGACY_RECORDS = LEGACY_ROOT / "static_records.json"
LEGACY_SUMMARY = LEGACY_ROOT / "summary.json"
COPY_ROOT = REPO / "exp/runtime/nano3d_glb_n33/input_packages"
COPY_MANIFEST = COPY_ROOT / "input_manifest.json"
RUNNER = Path(__file__).resolve()
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_pva_existing_export"
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".glb", ".gltf", ".off", ".dae"}
REQUIRED_OUTPUTS = (
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
)
EXPECTED_INPUT_HASHES = {
    "protocol": "5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c",
    "legacy_manifest": "9b30419bbe877fd4463684fd8e6d956e54fdc388bb058e2ca9c96c792c5062a5",
    "legacy_records": "a886151c49577f9f3932a433d3b3fb3a7735af1103d72e9b310fa0456b6e5a24",
    "legacy_summary": "fa1adb1f7546b98ce48bb676570bbbd670b232a43746fd624250f064294ac3ee",
    "copy_manifest": "4ca61644aa96f158fdee70271e218312f4e6fadb2df948692d8b1698f266b80d",
}
EXPECTED_LEGACY = {
    "asset_count": 33,
    "readable_geometries": 387,
    "watertight_geometries": 145,
    "winding_consistent_geometries": 379,
    "open_edges": 290335,
    "degenerate_faces": 51,
}
EXPECTED_MESH_INVENTORY_SHA256 = "3e93abfb654af37b4bcaa70859d41d737dcb09c72863b2471cfd2fa7039b4c94"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def input_hashes() -> dict[str, str]:
    return {
        "protocol": sha256_file(PROTOCOL),
        "legacy_manifest": sha256_file(LEGACY_MANIFEST),
        "legacy_records": sha256_file(LEGACY_RECORDS),
        "legacy_summary": sha256_file(LEGACY_SUMMARY),
        "copy_manifest": sha256_file(COPY_MANIFEST),
    }


def mesh_inventory(copy_manifest: list[dict[str, Any]]) -> dict[str, Any]:
    entries: list[dict[str, Any]] = []
    for copied in copy_manifest:
        package = Path(copied["copied_package"]).resolve()
        for mesh_path in sorted(
            path
            for path in (package / "assets").rglob("*")
            if path.is_file() and path.suffix.lower() in MESH_SUFFIXES
        ):
            if mesh_path.is_symlink():
                raise RuntimeError(f"mesh inventory contains symlink: {mesh_path}")
            entries.append(
                {
                    "asset_id": package.name,
                    "bytes": mesh_path.stat().st_size,
                    "path": mesh_path.relative_to(package).as_posix(),
                    "sha256": sha256_file(mesh_path),
                }
            )
    canonical = json.dumps(
        entries, ensure_ascii=False, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    return {
        "algorithm": "sha256(canonical compact JSON of ordered asset_id/path/bytes/file_sha256 records)",
        "entries": entries,
        "file_count": len(entries),
        "sha256": hashlib.sha256(canonical).hexdigest(),
    }


def validate_inputs() -> tuple[
    list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]
]:
    observed_hashes = input_hashes()
    if observed_hashes != EXPECTED_INPUT_HASHES:
        raise RuntimeError(f"frozen input hash mismatch: {observed_hashes}")

    legacy_manifest = load_jsonl(LEGACY_MANIFEST)
    legacy_records = load_json(LEGACY_RECORDS)
    copy_manifest = load_json(COPY_MANIFEST)
    if not (len(legacy_manifest) == len(legacy_records) == len(copy_manifest) == 33):
        raise RuntimeError("expected exactly 33 rows in all frozen inputs")

    ids = [row["asset_id"] for row in legacy_manifest]
    if len(ids) != len(set(ids)):
        raise RuntimeError("legacy manifest identities are not unique")
    if ids != [row["asset_id"] for row in legacy_records]:
        raise RuntimeError("legacy records do not preserve manifest order")
    if ids != [Path(row["copied_package"]).name for row in copy_manifest]:
        raise RuntimeError("copy manifest does not preserve legacy identity order")

    for legacy, copied in zip(legacy_manifest, copy_manifest, strict=True):
        if legacy["model_urdf_sha256"] != copied["model_urdf_sha256"]:
            raise RuntimeError(f"URDF provenance mismatch for {legacy['asset_id']}")
        package = Path(copied["copied_package"]).resolve()
        if package.parent != COPY_ROOT or package.name != legacy["asset_id"]:
            raise RuntimeError(f"copy path escaped frozen root for {legacy['asset_id']}")
        if sha256_file(package / "model.urdf") != legacy["model_urdf_sha256"]:
            raise RuntimeError(f"live copied URDF hash mismatch for {legacy['asset_id']}")
    inventory = mesh_inventory(copy_manifest)
    if inventory["file_count"] != 387 or inventory["sha256"] != EXPECTED_MESH_INVENTORY_SHA256:
        raise RuntimeError(
            "mesh inventory hash mismatch: "
            f"count={inventory['file_count']} sha256={inventory['sha256']}"
        )
    return legacy_manifest, legacy_records, copy_manifest, inventory


def audit_geometry(mesh: Any) -> dict[str, Any]:
    faces = np.asarray(mesh.faces)
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"expected triangle faces, got {faces.shape}")
    edges = np.sort(
        np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]), axis=0),
        axis=1,
    )
    _unique_edges, incidence = np.unique(edges, axis=0, return_counts=True)
    repeated_vertex = (
        (faces[:, 0] == faces[:, 1])
        | (faces[:, 1] == faces[:, 2])
        | (faces[:, 2] == faces[:, 0])
    )
    area = np.asarray(mesh.area_faces)
    degenerate = int(np.count_nonzero(repeated_vertex | (area <= 1e-12)))
    return {
        "degenerate_faces": degenerate,
        "edge_manifold_proxy": bool(np.all(incidence <= 2)),
        "face_count": int(len(faces)),
        "nonmanifold_edges": int(np.count_nonzero(incidence > 2)),
        "open_edges": int(np.count_nonzero(incidence == 1)),
        "vertex_count": int(len(mesh.vertices)),
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
    }


def audit_asset(
    legacy: dict[str, Any], legacy_record: dict[str, Any], copied: dict[str, Any]
) -> dict[str, Any]:
    package = Path(copied["copied_package"]).resolve()
    mesh_paths = sorted(
        path
        for path in (package / "assets").rglob("*")
        if path.is_file() and path.suffix.lower() in MESH_SUFFIXES
    )
    geometries: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    for mesh_path in mesh_paths:
        try:
            loaded = trimesh.load(mesh_path, force="scene", process=False)
            scene_geometries = (
                list(loaded.geometry.items())
                if hasattr(loaded, "geometry")
                else [("geometry_0", loaded)]
            )
            readable = [
                (name, mesh)
                for name, mesh in scene_geometries
                if hasattr(mesh, "faces") and hasattr(mesh, "vertices")
            ]
            if not readable:
                raise ValueError("no readable triangle geometry")
            for geometry_name, mesh in readable:
                geometries.append(
                    {
                        "geometry_name": str(geometry_name),
                        "mesh_path": mesh_path.relative_to(package).as_posix(),
                        **audit_geometry(mesh),
                    }
                )
        except Exception as exc:  # noqa: BLE001
            errors.append(
                {
                    "mesh_path": mesh_path.relative_to(package).as_posix(),
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )

    count = len(geometries)
    manifold = sum(row["edge_manifold_proxy"] for row in geometries)
    observed_legacy = {
        "mesh_files": len(mesh_paths),
        "readable_mesh_files": count,
        "watertight_mesh_files": sum(row["watertight"] for row in geometries),
        "winding_consistent_mesh_files": sum(row["winding_consistent"] for row in geometries),
        "open_edges": sum(row["open_edges"] for row in geometries),
        "degenerate_faces": sum(row["degenerate_faces"] for row in geometries),
    }
    expected_legacy = {key: legacy_record[key] for key in observed_legacy}
    return {
        "asset_id": legacy["asset_id"],
        "copied_package": str(package),
        "geometry": {
            "edge_manifold_fraction": manifold / count if count else None,
            "edge_manifold_geometry_count": manifold,
            "geometry_count": count,
            "load_errors": errors,
            "nonmanifold_edges": sum(row["nonmanifold_edges"] for row in geometries),
            "records": geometries,
        },
        "legacy_metrics_match": observed_legacy == expected_legacy,
        "legacy_observed": observed_legacy,
        "legacy_expected": expected_legacy,
        "model_urdf_sha256": legacy["model_urdf_sha256"],
        "original_package_present": Path(legacy["asset_path"]).is_dir(),
    }


def build_records(
    legacy_manifest: list[dict[str, Any]],
    legacy_records: list[dict[str, Any]],
    copy_manifest: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for index, (legacy, legacy_record, copied) in enumerate(
        zip(legacy_manifest, legacy_records, copy_manifest, strict=True)
    ):
        row = audit_asset(legacy, legacy_record, copied)
        row["selection_index"] = index
        records.append(row)
    return records


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    geometry_rows = [geometry for row in records for geometry in row["geometry"]["records"]]
    manifold = sum(row["edge_manifold_proxy"] for row in geometry_rows)
    legacy_reproduction = {
        "asset_count": len(records),
        "readable_geometries": len(geometry_rows),
        "watertight_geometries": sum(row["watertight"] for row in geometry_rows),
        "winding_consistent_geometries": sum(row["winding_consistent"] for row in geometry_rows),
        "open_edges": sum(row["open_edges"] for row in geometry_rows),
        "degenerate_faces": sum(row["degenerate_faces"] for row in geometry_rows),
    }
    return {
        "audit_id": "pva_existing_export_table7_manifold_backfill_v1",
        "cohort": {
            "available_assets": len(records),
            "geometry_evaluable_assets": sum(bool(row["geometry"]["geometry_count"]) for row in records),
            "requested_assets": 33,
            "selection": "original frozen Nano3D existing-export pilot manifest; no outcome filtering",
        },
        "geometry": {
            "edge_manifold_proxy": {
                "all_geometries_pass_assets": {
                    "denominator": len(records),
                    "numerator": sum(
                        row["geometry"]["edge_manifold_geometry_count"]
                        == row["geometry"]["geometry_count"]
                        for row in records
                    ),
                },
                "definition": "every undirected edge has at most two incident faces; vertex-manifold is not claimed",
                "geometry_level": {
                    "denominator": len(geometry_rows),
                    "numerator": manifold,
                    "rate": manifold / len(geometry_rows) if geometry_rows else None,
                },
                "per_asset_mean_fraction": fmean(
                    row["geometry"]["edge_manifold_fraction"] for row in records
                ),
            },
            "load_error_count": sum(len(row["geometry"]["load_errors"]) for row in records),
            "nonmanifold_edges_total": sum(row["nonmanifold_edges"] for row in geometry_rows),
            "readable_geometries": len(geometry_rows),
        },
        "legacy_reproduction": {
            "expected": EXPECTED_LEGACY,
            "observed": legacy_reproduction,
            "pass": legacy_reproduction == EXPECTED_LEGACY
            and all(row["legacy_metrics_match"] for row in records),
        },
        "protocol_id": "nano3d_table7_production_readiness_v1",
        "status": "COMPLETE",
    }


def build_manifest(records: list[dict[str, Any]], inventory: dict[str, Any]) -> dict[str, Any]:
    return {
        "audit_id": "pva_existing_export_table7_manifold_backfill_v1",
        "cohort_size": 33,
        "geometry_scope": "all mesh payloads under each frozen copied package assets/ directory",
        "input_hashes": input_hashes(),
        "loader": "trimesh.load(path, force='scene', process=False); every readable triangle geometry",
        "mesh_inventory": inventory,
        "original_package_availability": {
            "present": sum(row["original_package_present"] for row in records),
            "missing": sum(not row["original_package_present"] for row in records),
            "policy": "preserved package copies accepted only after identity, URDF hash, and legacy geometry metric reproduction gates",
        },
        "protocol_sha256": sha256_file(PROTOCOL),
        "runner_sha256": sha256_file(RUNNER),
        "selection": [
            {
                "asset_id": row["asset_id"],
                "copied_package": row["copied_package"],
                "model_urdf_sha256": row["model_urdf_sha256"],
                "selection_index": row["selection_index"],
            }
            for row in records
        ],
    }


def build_report(summary: dict[str, Any]) -> str:
    manifold = summary["geometry"]["edge_manifold_proxy"]
    level = manifold["geometry_level"]
    return f"""# PV-A existing-export Table 7 manifold backfill

Status: **{summary['status']}**

The cohort is the original frozen N=33 existing-export pilot. Ten original
`seed_exports_physics_10` paths are no longer present, so the audit uses the
preserved N=33 input-package copies. Identity and URDF hashes match the legacy
manifest 33/33, and the old geometry statistics reproduce exactly before the
new metric is accepted.

## Result

| Method | Manifold |
|---|---:|
| PV-A existing-export pilot (N=33) | {manifold['per_asset_mean_fraction']:.6f} edge-manifold proxy mean/asset; {level['numerator']}/{level['denominator']} geometries |

- Assets whose every geometry passes: {manifold['all_geometries_pass_assets']['numerator']}/{manifold['all_geometries_pass_assets']['denominator']}.
- Nonmanifold edges (>2 incident faces): {summary['geometry']['nonmanifold_edges_total']} total.
- Load errors: {summary['geometry']['load_error_count']}.
- Definition: every undirected edge has at most two incident faces. Boundary
  edges are allowed; vertex-manifold is not claimed.
- Legacy reproduction gate: **{'PASS' if summary['legacy_reproduction']['pass'] else 'FAIL'}**.
"""


def build_outputs() -> dict[str, Any]:
    legacy_manifest, legacy_records, copy_manifest, inventory = validate_inputs()
    records = build_records(legacy_manifest, legacy_records, copy_manifest)
    summary = aggregate(records)
    if not summary["legacy_reproduction"]["pass"]:
        raise RuntimeError(f"legacy geometry metrics did not reproduce: {summary['legacy_reproduction']}")
    if summary["geometry"]["load_error_count"] != 0:
        raise RuntimeError("mesh load errors prevent a complete backfill")
    manifest = build_manifest(records, inventory)
    summary["hashes"] = {
        "manifest_sha256": hashlib.sha256(
            (json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
        ).hexdigest(),
        "protocol_sha256": sha256_file(PROTOCOL),
        "runner_sha256": sha256_file(RUNNER),
    }
    return {
        "asset_records.json": records,
        "manifest.json": manifest,
        "protocol_snapshot.json": load_json(PROTOCOL),
        "report.md": build_report(summary),
        "summary.json": summary,
    }


def write_outputs(output: Path, outputs: dict[str, Any]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for name, payload in outputs.items():
        path = output / name
        if name == "protocol_snapshot.json":
            path.write_bytes(PROTOCOL.read_bytes())
        elif name.endswith(".json"):
            dump_json(path, payload)
        else:
            path.write_text(payload, encoding="utf-8")
    hashes = {name: sha256_file(output / name) for name in outputs}
    checks = {
        "all_asset_legacy_metrics_match": all(
            row["legacy_metrics_match"] for row in outputs["asset_records.json"]
        ),
        "all_required_outputs_exist": True,
        "cohort_is_33_unique_assets": len(
            {row["asset_id"] for row in outputs["asset_records.json"]}
        )
        == 33,
        "geometry_denominator_is_387": outputs["summary.json"]["geometry"]["readable_geometries"]
        == 387,
        "input_hashes_frozen": input_hashes() == EXPECTED_INPUT_HASHES,
        "legacy_reproduction_pass": outputs["summary.json"]["legacy_reproduction"]["pass"],
        "load_errors_zero": outputs["summary.json"]["geometry"]["load_error_count"] == 0,
        "mesh_inventory_frozen": outputs["manifest.json"]["mesh_inventory"]["file_count"] == 387
        and outputs["manifest.json"]["mesh_inventory"]["sha256"]
        == EXPECTED_MESH_INVENTORY_SHA256,
        "protocol_snapshot_exact": (output / "protocol_snapshot.json").read_bytes()
        == PROTOCOL.read_bytes(),
        "runner_hash_recorded": outputs["manifest.json"]["runner_sha256"] == sha256_file(RUNNER),
        "vertex_manifold_not_claimed": "vertex-manifold is not claimed"
        in outputs["summary.json"]["geometry"]["edge_manifold_proxy"]["definition"],
    }
    self_check = {
        "artifact_hashes": hashes,
        "checks": checks,
        "status": "PASS" if all(checks.values()) else "FAIL",
    }
    dump_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {checks}")


def verify(output: Path) -> list[str]:
    errors: list[str] = []
    for name in REQUIRED_OUTPUTS:
        if not (output / name).is_file():
            errors.append(f"missing required output: {name}")
    if errors:
        return errors
    self_check = load_json(output / "self_check.json")
    if self_check.get("status") != "PASS" or not all(self_check.get("checks", {}).values()):
        errors.append("recorded self-check is not PASS")
    for name, expected_hash in self_check.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected_hash:
            errors.append(f"artifact hash mismatch: {name}")
    try:
        expected = build_outputs()
        for name, payload in expected.items():
            if name.endswith(".json"):
                observed = load_json(output / name)
            else:
                observed = (output / name).read_text(encoding="utf-8")
            if observed != payload:
                errors.append(f"live recomputation mismatch: {name}")
    except Exception as exc:  # noqa: BLE001
        errors.append(f"live recomputation failed: {type(exc).__name__}: {exc}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    if args.verify_only:
        errors = verify(output)
    else:
        outputs = build_outputs()
        write_outputs(output, outputs)
        errors = verify(output)
    summary = load_json(output / "summary.json") if (output / "summary.json").is_file() else {}
    manifold = summary.get("geometry", {}).get("edge_manifold_proxy", {})
    print(
        json.dumps(
            {
                "errors": errors,
                "geometry_level": manifold.get("geometry_level"),
                "output": str(output.relative_to(REPO.parent)),
                "per_asset_mean_fraction": manifold.get("per_asset_mean_fraction"),
                "status": "PASS" if not errors else "FAIL",
            },
            sort_keys=True,
        )
    )
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
