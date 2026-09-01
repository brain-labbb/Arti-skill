#!/usr/bin/env python3
"""Emit the frozen Table 7 PhysX-Omni N=0 production-readiness audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve(strict=True)
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
TABLE6 = REPO / "exp/runtime/table6_physx_omni/preflight.json"
SOURCE = REPO / ".cache/table6_sources/physx_omni/source"
METADATA = REPO / ".cache/table6_sources/physx_omni/metadata"
OUTPUT = REPO / "exp/runtime/table7_physx_omni"

EXPECTED_SOURCE_COMMIT = "46fa1cd0b6883d4d14431d51c3326ef80a85ef64"
EXPECTED_MODEL_REVISION = "765cd275839f88333cb754f1c6c0b8d3887a3b2c"
EXPECTED_TRELLIS_REVISION = "25e0d31ffbebe4b5a97464dd851910efc3002d96"
STATUS = "BLOCKED_MISSING_MODEL_OUTPUTS"
INTERPRETATION = "NOT_AN_EXPERIMENTAL_ZERO"
REQUIRED = [
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
]
METRICS = [
    "watertight",
    "manifold",
    "open_edges",
    "degenerate_faces",
    "self_intersection",
    "source_kb",
    "urdf_kb",
    "mesh_kb",
    "portable_package",
    "deterministic_build",
    "semantic_complete",
    "kinematic_complete",
    "physical_complete",
]
CODE_EVIDENCE = [
    "README.md",
    "download.py",
    "1vlm_demo.py",
    "2infer_geo.py",
    "decoder_each.py",
    "3jsongen_update.py",
]


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def rel(path: Path) -> str:
    return safe(path, must_exist=False).relative_to(WORKSPACE).as_posix()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def write_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def file_row(path: Path, root: Path) -> dict[str, Any]:
    path = safe(path)
    root = safe(root)
    return {
        "path": path.relative_to(root).as_posix(),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def tree_inventory(root: Path) -> list[dict[str, Any]]:
    root = safe(root)
    rows: list[dict[str, Any]] = []
    for current_raw, directories, files in os.walk(root, followlinks=False):
        current = safe(Path(current_raw))
        directories[:] = sorted(directories)
        for name in sorted(files):
            rows.append(file_row(current / name, root))
    return sorted(rows, key=lambda row: row["path"])


def output_candidates() -> list[dict[str, Any]]:
    demo = safe(SOURCE / "demo")
    candidates: dict[str, Path] = {}
    for suffix in ("*.glb", "*.gltf", "*.obj", "*.urdf", "*.xml"):
        for raw in demo.rglob(suffix):
            path = safe(raw)
            candidates[rel(path)] = path
    scene = safe(SOURCE / "scene.xml", must_exist=False)
    if scene.is_file():
        candidates[rel(scene)] = scene
    return [
        {"path": name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
        for name, path in sorted(candidates.items())
    ]


def evidence(table6: dict[str, Any]) -> dict[str, Any]:
    official = table6["official_source"]
    remote = table6["official_remote_artifacts"]
    archive_relative = Path(official["archive_path"])
    if archive_relative.is_absolute() or ".." in archive_relative.parts:
        raise RuntimeError("unsafe archive path in Table 6 preflight")
    archive = safe(REPO / archive_relative)
    code = [file_row(SOURCE / name, SOURCE) for name in CODE_EVIDENCE]
    metadata = tree_inventory(METADATA)
    candidates = output_candidates()
    model = remote["physx_omni_model"]
    trellis = remote["trellis_image_large"]
    return {
        "official_code": {
            "available": True,
            "source_commit": official["commit"],
            "source_tree": official["tree"],
            "archive_path": rel(archive),
            "archive_bytes": archive.stat().st_size,
            "archive_sha256": sha256_file(archive),
            "archive_sha256_expected": official["archive_sha256"],
            "code_file_inventory": code,
            "table6_cpu_tiny_smoke_status": table6["official_tiny_smoke"]["status"],
            "tiny_smoke_is_method_inference": False,
            "tiny_smoke_is_final_package_evidence": False,
        },
        "metadata_inventory": {
            "root": rel(METADATA),
            "file_count": len(metadata),
            "files": metadata,
            "canonical_inventory_sha256": sha256_json(metadata),
        },
        "pinned_remote_revisions": {
            "physx_omni_model": model["revision"],
            "trellis_image_large": trellis["revision"],
            "physxverse": remote["physxverse"]["revision"],
            "physx_mobility": remote["physx_mobility"]["revision"],
            "physx_bench": remote["physx_bench"]["revision"],
        },
        "model_inputs": {
            "physx_omni_8b": {
                "parameters": model["parameters"],
                "weight_shards": model["weight_shards"],
                "weight_bytes": model["weight_bytes"],
                "downloaded": model["downloaded"],
            },
            "trellis_image_large": {
                "repository_bytes": trellis["repository_bytes"],
                "downloaded": trellis["downloaded"],
            },
            "minimum_model_repository_bytes_not_downloaded": remote[
                "minimum_model_repository_bytes_not_downloaded"
            ],
        },
        "production_outputs": {
            "available": bool(candidates),
            "search_scope": rel(SOURCE / "demo") + "/** plus source/scene.xml",
            "candidate_count": len(candidates),
            "candidates": candidates,
            "common_table7_output_adapter_present": False,
            "distinction": (
                "Official source availability and a CPU benchmark-plumbing smoke do not "
                "constitute a method-generated final production package."
            ),
        },
    }


def not_evaluable(reason: str) -> dict[str, Any]:
    return {
        "state": "not_evaluable",
        "numerator": 0,
        "denominator": 0,
        "value": None,
        "reason": reason,
    }


def make_manifest(protocol: dict[str, Any], table6: dict[str, Any], audit: dict[str, Any]) -> dict[str, Any]:
    blockers = [
        {
            "id": "PHYSX_OMNI_8B_WEIGHTS_MISSING",
            "evidence": "The pinned four-shard 8B checkpoint (16,584,414,544 bytes) is not downloaded.",
        },
        {
            "id": "TRELLIS_DECODER_WEIGHTS_MISSING",
            "evidence": "The pinned TRELLIS-image-large repository (3,300,497,168 bytes) is not downloaded.",
        },
        {
            "id": "METHOD_GENERATED_FINAL_OUTPUTS_MISSING",
            "evidence": "No GLB/OBJ/URDF/MJCF output exists in the official default output scope.",
        },
        {
            "id": "TABLE7_OUTPUT_ADAPTER_MISSING",
            "evidence": "No frozen adapter maps a PhysX-Omni run into a complete attributable Table 7 package and dependency manifest.",
        },
    ]
    evidence_files = [
        rel(PROTOCOL),
        rel(TABLE6),
        audit["official_code"]["archive_path"],
        *[rel(SOURCE / name) for name in CODE_EVIDENCE],
        *[rel(METADATA / row["path"]) for row in audit["metadata_inventory"]["files"]],
    ]
    return {
        "schema_version": "1.0.0",
        "manifest_id": "table7_physx_omni_local_n0_v1",
        "method": "PhysX-Omni",
        "table": "Table 7: Production Readiness",
        "status": STATUS,
        "interpretation": INTERPRETATION,
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": rel(OUTPUT / "protocol_snapshot.json"),
        "protocol_snapshot_sha256": sha256_file(OUTPUT / "protocol_snapshot.json"),
        "frozen_inputs": {
            "table6_preflight": rel(TABLE6),
            "table6_preflight_sha256": sha256_file(TABLE6),
            "runner": rel(SCRIPT),
            "runner_sha256": sha256_file(SCRIPT),
        },
        "selection_policy": {
            "requested_assets": 0,
            "available_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "outcome_based_reselection": False,
            "reason": "No frozen request cohort was run because required model inputs and final-output adapter are absent.",
        },
        "release_evidence": audit,
        "blockers": blockers,
        "paper_value_policy": "No paper claim is substituted for a local Table 7 measurement.",
        "evidence_files": sorted(set(evidence_files)),
        "assets": [],
    }


def make_summary(protocol: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    reason = (
        "No PhysX-Omni-attributable final package was released into or generated in "
        "the frozen local output scope; required model inputs and a Table 7 adapter are absent."
    )
    return {
        "schema_version": "1.0.0",
        "method": "PhysX-Omni",
        "status": STATUS,
        "interpretation": INTERPRETATION,
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": sha256_file(OUTPUT / "protocol_snapshot.json"),
        "manifest_sha256": sha256_file(OUTPUT / "manifest.json"),
        "generation_run_started": False,
        "local_evaluated_asset_count": 0,
        "paper_numbers_used_as_local_results": False,
        "cohort": {
            "requested_assets": 0,
            "available_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "pass_assets": 0,
            "fail_assets": 0,
            "unavailable_assets": 0,
        },
        "performance_columns": {name: not_evaluable(reason) for name in METRICS},
        "blockers": manifest["blockers"],
    }


def report_text(manifest: dict[str, Any], summary: dict[str, Any]) -> str:
    audit = manifest["release_evidence"]
    pins = audit["pinned_remote_revisions"]
    cells = " | ".join("not_evaluable" for _ in METRICS)
    return f"""# Table 7 PhysX-Omni Production Readiness audit

Status: **{STATUS}**  
Interpretation: **{INTERPRETATION}**  
Local evaluated final packages: **N=0**

Official code is locally available at source commit
`{audit['official_code']['source_commit']}` (tree
`{audit['official_code']['source_tree']}`). The fixed source archive hash matches
the Table 6 pin, and `{audit['metadata_inventory']['file_count']}` local metadata
files are inventoried with byte counts and SHA-256 values in `manifest.json`.

This code-availability evidence is distinct from production-output evidence. The
prior CPU tiny smoke exercised benchmark manifest/aggregation plumbing only; it
did not run PhysX-Omni inference and did not produce a final asset package.

## Pinned model inputs and blockers

| Input | Frozen revision | Local state |
|---|---|---|
| PhysX-Omni 8B checkpoint | `{pins['physx_omni_model']}` | missing; 4 shards / 16,584,414,544 bytes not downloaded |
| TRELLIS-image-large | `{pins['trellis_image_large']}` | missing; 3,300,497,168-byte repository not downloaded |
| Method-generated final package | source default output scope | missing |
| Common Table 7 output adapter | frozen local harness | missing |

No network, download, GPU, inference, or generation was performed by this
runner. The local `N=0` is an availability result, not evidence that the method
generated zero successful assets.

## Common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| PhysX-Omni (official code available; local final-package N=0) | {cells} |

All 13 cells are `not_evaluable`. Geometry, size, portability, rebuild, and
completeness tests require a method-attributable final package; none exists
locally. Paper claims and code/tiny-smoke evidence were not converted into
local scores.

Protocol SHA-256: `{summary['protocol_sha256']}`  
Manifest SHA-256: `{summary['manifest_sha256']}`
"""


def workspace_evidence_exists(relative: str) -> bool:
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        return False
    return safe(WORKSPACE / path, must_exist=False).is_file()


def current_checks(
    protocol: dict[str, Any], manifest: dict[str, Any], records: list[Any], summary: dict[str, Any]
) -> dict[str, bool]:
    release = manifest["release_evidence"]
    cohort = summary["cohort"]
    columns = summary["performance_columns"]
    return {
        "required_outputs_match_protocol": protocol["required_outputs"] == REQUIRED,
        "all_required_outputs_exist": all(safe(OUTPUT / name, must_exist=False).is_file() for name in REQUIRED),
        "protocol_snapshot_exact": sha256_file(OUTPUT / "protocol_snapshot.json") == sha256_file(PROTOCOL),
        "protocol_hash_recorded": manifest["protocol_snapshot_sha256"] == summary["protocol_sha256"],
        "manifest_hash_recorded": sha256_file(OUTPUT / "manifest.json") == summary["manifest_sha256"],
        "manifest_identities_unique": len(records) == len({row.get("asset_id") for row in records}) == 0,
        "asset_records_match_manifest": records == manifest["assets"] == [],
        "requested_denominator_accounted": cohort["requested_assets"] == cohort["pass_assets"] + cohort["fail_assets"] + cohort["unavailable_assets"],
        "all_denominators_zero": all(cohort[name] == 0 for name in ("requested_assets", "available_assets", "geometry_evaluable_assets", "package_evaluable_assets")),
        "all_13_columns_present": set(columns) == set(METRICS) and len(columns) == 13,
        "all_13_columns_not_evaluable": all(value["state"] == "not_evaluable" for value in columns.values()),
        "all_rates_explicit_n0": all(value["numerator"] == 0 and value["denominator"] == 0 and value["value"] is None for value in columns.values()),
        "status_is_blocked_missing_model_outputs": manifest["status"] == summary["status"] == STATUS,
        "interpretation_is_not_experimental_zero": manifest["interpretation"] == summary["interpretation"] == INTERPRETATION,
        "source_commit_pinned": release["official_code"]["source_commit"] == EXPECTED_SOURCE_COMMIT,
        "source_archive_hash_matches_pin": release["official_code"]["archive_sha256"] == release["official_code"]["archive_sha256_expected"],
        "model_revision_pinned": release["pinned_remote_revisions"]["physx_omni_model"] == EXPECTED_MODEL_REVISION,
        "trellis_revision_pinned": release["pinned_remote_revisions"]["trellis_image_large"] == EXPECTED_TRELLIS_REVISION,
        "official_code_available": release["official_code"]["available"] is True,
        "model_inputs_absent": release["model_inputs"]["physx_omni_8b"]["downloaded"] is False and release["model_inputs"]["trellis_image_large"]["downloaded"] is False,
        "no_method_generated_outputs": release["production_outputs"]["available"] is False and release["production_outputs"]["candidate_count"] == 0,
        "output_adapter_absent": release["production_outputs"]["common_table7_output_adapter_present"] is False,
        "paper_values_not_used": summary["paper_numbers_used_as_local_results"] is False,
        "all_referenced_evidence_exists": all(workspace_evidence_exists(path) for path in manifest["evidence_files"]),
    }


def verify(output: Path) -> dict[str, Any]:
    global OUTPUT
    OUTPUT = safe(output)
    protocol = read_json(OUTPUT / "protocol_snapshot.json")
    manifest = read_json(OUTPUT / "manifest.json")
    records = read_json(OUTPUT / "asset_records.json")
    summary = read_json(OUTPUT / "summary.json")
    recorded = read_json(OUTPUT / "self_check.json")
    checks = current_checks(protocol, manifest, records, summary)
    hashes = {
        name: sha256_file(OUTPUT / name) == expected
        for name, expected in recorded["artifact_hashes"].items()
    }
    failures = [name for name, passed in checks.items() if not passed]
    failures.extend(f"artifact_hash:{name}" for name, passed in hashes.items() if not passed)
    if recorded.get("status") != "PASS":
        failures.append("recorded_self_check_status")
    return {
        "status": "PASS" if not failures else "FAIL",
        "checks": checks,
        "artifact_hash_matches": hashes,
        "failures": failures,
    }


def run(output: Path) -> dict[str, Any]:
    global OUTPUT
    OUTPUT = safe(output, must_exist=False)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    safe(OUTPUT / "protocol_snapshot.json", must_exist=False).write_bytes(safe(PROTOCOL).read_bytes())
    protocol = read_json(OUTPUT / "protocol_snapshot.json")
    if protocol["required_outputs"] != REQUIRED:
        raise RuntimeError("shared protocol required_outputs changed")
    table6 = read_json(TABLE6)
    audit = evidence(table6)
    if audit["production_outputs"]["available"]:
        raise RuntimeError("final-output candidates appeared; implement and freeze the output adapter before scoring")
    manifest = make_manifest(protocol, table6, audit)
    write_json(OUTPUT / "manifest.json", manifest)
    records: list[Any] = []
    write_json(OUTPUT / "asset_records.json", records)
    summary = make_summary(protocol, manifest)
    write_json(OUTPUT / "summary.json", summary)
    write_text(OUTPUT / "report.md", report_text(manifest, summary))
    write_json(OUTPUT / "self_check.json", {"status": "PROVISIONAL", "artifact_hashes": {}})
    checks = current_checks(protocol, manifest, records, summary)
    artifact_hashes = {
        name: sha256_file(OUTPUT / name) for name in REQUIRED if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "protocol_sha256": sha256_file(OUTPUT / "protocol_snapshot.json"),
        "manifest_sha256": sha256_file(OUTPUT / "manifest.json"),
        "artifact_hash_scope": "all required outputs except self_check.json to avoid circular hashing",
        "artifact_hashes": artifact_hashes,
    }
    write_json(OUTPUT / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {[name for name, passed in checks.items() if not passed]}")
    return verify(OUTPUT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if Path.cwd().resolve(strict=True) != WORKSPACE:
        raise RuntimeError(f"run from authorized workspace root: {WORKSPACE}")
    result = verify(args.output) if args.verify_only else run(args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
