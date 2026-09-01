#!/usr/bin/env python3
"""Produce the Table 7 Nova3D release audit from the frozen public checkout.

The public repository contains clients and integrations, not the hosted generation
backend or attributable final assets.  This runner therefore performs no generation
and no mesh scoring.  Local N stays zero and the N=54 paper row remains separate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
TABLE6_PREFLIGHT = REPO / "exp/runtime/table6_nova3d/preflight.json"
TABLE_DOCUMENT = REPO / "exp/Nano3dresults.md"
CHECKOUT = REPO / ".cache/table6_sources/nova3d/code"
OUTPUT = REPO / "exp/runtime/table7_nova3d"

EXPECTED_COMMIT = "042ee613aa2fb745d287261eab029d42c704646e"
FINAL_ASSET_SUFFIXES = {
    ".blend", ".fbx", ".glb", ".gltf", ".obj", ".off", ".ply", ".stl",
    ".urdf", ".usd", ".usda", ".usdc",
}
LOCAL_COLUMNS = (
    "watertight", "manifold", "open_edges", "degenerate_faces",
    "self_intersection", "source_bytes", "urdf_bytes", "mesh_bytes",
    "portable_package", "deterministic_build", "semantic_complete",
    "kinematic_complete", "physical_complete",
)
BLOCKERS = ("CLOSED_BACKEND", "PAPER_ASSETS_UNRELEASED")


def safe(path: Path, *, must_exist: bool = True) -> Path:
    """Resolve and enforce the run-start workspace boundary before file access."""
    root = WORKSPACE.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def rel(path: Path) -> str:
    return safe(path, must_exist=False).relative_to(WORKSPACE).as_posix()


def read_text(path: Path) -> str:
    return safe(path).read_text(encoding="utf-8")


def read_json(path: Path) -> Any:
    return json.loads(read_text(path))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def checkout_commit() -> tuple[str, str]:
    git_dir = safe(CHECKOUT / ".git")
    head = read_text(git_dir / "HEAD").strip()
    if not head.startswith("ref: "):
        commit = head
        branch = "detached"
    else:
        ref = head[5:].strip()
        if ".." in Path(ref).parts or not re.fullmatch(r"refs/[A-Za-z0-9._/-]+", ref):
            raise RuntimeError(f"unsafe git HEAD ref: {ref!r}")
        ref_path = safe(git_dir / ref, must_exist=False)
        if ref_path.is_file():
            commit = read_text(ref_path).strip()
        else:
            commit = ""
            for line in read_text(git_dir / "packed-refs").splitlines():
                fields = line.split()
                if len(fields) == 2 and fields[1] == ref:
                    commit = fields[0]
                    break
        branch = ref.removeprefix("refs/heads/")
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise RuntimeError("could not resolve a 40-hex checkout commit")
    return branch, commit


def checkout_inventory() -> dict[str, Any]:
    root = safe(CHECKOUT)
    top_level: list[dict[str, str]] = []
    for candidate in sorted(root.iterdir(), key=lambda item: item.name):
        resolved = safe(candidate)
        kind = "directory" if resolved.is_dir() else "file"
        top_level.append({"path": candidate.name, "kind": kind})

    assets: list[dict[str, Any]] = []
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        current = safe(Path(directory))
        if current == root and ".git" in directory_names:
            directory_names.remove(".git")
        directory_names.sort()
        for name in sorted(file_names):
            raw = current / name
            if raw.suffix.lower() not in FINAL_ASSET_SUFFIXES:
                continue
            path = safe(raw)
            assets.append(
                {
                    "path": path.relative_to(root).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    counts = {suffix: 0 for suffix in sorted(FINAL_ASSET_SUFFIXES)}
    for item in assets:
        counts[Path(item["path"]).suffix.lower()] += 1
    examples = safe(root / "examples", must_exist=False)
    return {
        "scope": "official public checkout; .git excluded from asset scan",
        "top_level_entries": top_level,
        "final_asset_suffix_counts": counts,
        "final_asset_count": len(assets),
        "final_assets": assets,
        "examples_directory_present": examples.is_dir(),
    }


def find_line(text: str, needle: str) -> int:
    for number, line in enumerate(text.splitlines(), start=1):
        if needle in line:
            return number
    raise RuntimeError(f"required evidence text is absent: {needle}")


def paper_evidence() -> dict[str, Any]:
    text = read_text(TABLE_DOCUMENT)
    prefix = "| Nova3D (paper, N=54 GLB assets) |"
    expected_fragments = (
        "0.89 per asset", "0.91 per asset", "42/asset mean",
        "18.7 KB source/asset", "1.16 MB GLB/asset",
        "54/54 runtime-ready GLB", "deterministic headless build reported",
        "named/tree claims reported",
    )
    matches = [
        (number, line)
        for number, line in enumerate(text.splitlines(), start=1)
        if line.startswith(prefix) and all(fragment in line for fragment in expected_fragments)
    ]
    if len(matches) != 1:
        raise RuntimeError("the frozen Nova3D paper-only Table 7 row drifted")
    line_number, row = matches[0]
    return {
        "evidence_class": "paper_only_transcription",
        "local_reproduction": False,
        "must_not_be_merged_with_local_results": True,
        "source": rel(TABLE_DOCUMENT),
        "source_sha256": sha256_file(TABLE_DOCUMENT),
        "source_line": line_number,
        "cohort_assets": 54,
        "representation": "GLB plus per-asset executable source",
        "values": {
            "watertight_per_asset": 0.89,
            "manifold_per_asset": 0.91,
            "open_edges_per_asset_mean": 42,
            "degenerate_faces": None,
            "paper_auxiliary_triangles_per_asset": 39000,
            "self_intersection": None,
            "source_kb_per_asset": 18.7,
            "urdf_kb_per_asset": None,
            "glb_mb_per_asset": 1.16,
            "portable_package_claim": "54/54 runtime-ready GLB",
            "deterministic_build_claim": "deterministic headless build reported",
            "semantic_claim": "named/tree claims reported",
            "kinematic_complete": None,
            "physical_complete": None,
        },
        "raw_markdown_row": row,
    }


def not_evaluable(reason: str) -> dict[str, Any]:
    return {
        "state": "not_evaluable",
        "reason": reason,
        "numerator": 0,
        "denominator": 0,
        "rate": None,
    }


def build_local_results() -> dict[str, Any]:
    reason = (
        "No Nova3D-attributable final GLB or source package is present in the "
        "official public checkout; no hosted or paid generation was run."
    )
    results = {name: not_evaluable(reason) for name in LOCAL_COLUMNS}
    for name in ("open_edges", "degenerate_faces"):
        results[name].update({"total": None, "mean_per_asset": None})
    for name in ("source_bytes", "urdf_bytes", "mesh_bytes"):
        results[name].update({"total_bytes": None, "mean_bytes_per_asset": None})
    return results


def report_text(summary: dict[str, Any]) -> str:
    paper = summary["paper_reported_only"]["values"]
    return f"""# Table 7: Nova3D production-readiness audit

- Status: **BLOCKED / NOT_EVALUABLE**.
- Local evaluable cohort: **N=0**.
- Official checkout commit: `{summary['official_checkout']['commit']}`.
- Final asset files in the checkout: **0**.
- Hosted API, paid generation, network, secret, and GPU use in this runner: **none**.

## Local common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Nova3D (official public checkout; local N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

`N=0` is an availability result, not a zero performance score. The public repository
contains open clients and integrations, while its README identifies the hosted generation
backend as closed-source and marks generated examples plus source programs as coming soon.
No final package exists on which geometry, copying, rebuilding, or field completeness can
be measured.

## Separate paper-only evidence

The existing Table 7 transcription reports N=54 GLB assets with watertight
`{paper['watertight_per_asset']}`, manifold `{paper['manifold_per_asset']}`, mean open edges
`{paper['open_edges_per_asset_mean']}`, source `{paper['source_kb_per_asset']} KB/asset`, and
GLB size `{paper['glb_mb_per_asset']} MB/asset`. It also transcribes the claims
`{paper['portable_package_claim']}`, `{paper['deterministic_build_claim']}`, and
`{paper['semantic_claim']}`. These values were not reproduced from this checkout and are
never used as local measurements.

## Blockers and denominators

- `CLOSED_BACKEND`: generation is dispatched to the proprietary hosted service.
- `PAPER_ASSETS_UNRELEASED`: no paper GLB/source packages or examples are in the checkout.
- requested assets: 0; available assets: 0; geometry-evaluable assets: 0;
  package-evaluable assets: 0.
- Every local Table 7 column is `not_evaluable`; paper-only N=54 remains a separate
  evidence layer.
"""


def create(output: Path) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    safe(output.parent).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = safe(output)

    protocol = read_json(PROTOCOL)
    preflight = read_json(TABLE6_PREFLIGHT)
    if protocol.get("protocol_id") != "nano3d_table7_production_readiness_v1":
        raise RuntimeError("unexpected shared Table 7 protocol")
    branch, commit = checkout_commit()
    inventory = checkout_inventory()
    if commit != EXPECTED_COMMIT or preflight["official_source"]["commit"] != commit:
        raise RuntimeError("Nova3D checkout commit differs from the frozen Table 6 evidence")
    if inventory["final_asset_count"] != 0 or inventory["examples_directory_present"]:
        raise RuntimeError("checkout now contains candidate released assets; N=0 audit is stale")
    if preflight["blocked_reason_codes"] != list(BLOCKERS):
        raise RuntimeError("Table 6 blocker evidence drifted")

    snapshot = output / "protocol_snapshot.json"
    temporary = safe(output / "protocol_snapshot.json.tmp", must_exist=False)
    shutil.copyfile(safe(PROTOCOL), temporary)
    temporary.replace(snapshot)
    protocol_hash = sha256_file(snapshot)

    paper = paper_evidence()
    evidence = {
        "table6_preflight": {"path": rel(TABLE6_PREFLIGHT), "sha256": sha256_file(TABLE6_PREFLIGHT)},
        "official_readme": {"path": rel(CHECKOUT / "README.md"), "sha256": sha256_file(CHECKOUT / "README.md")},
        "official_app_readme": {"path": rel(CHECKOUT / "app/README.md"), "sha256": sha256_file(CHECKOUT / "app/README.md")},
        "official_license": {"path": rel(CHECKOUT / "LICENSE"), "sha256": sha256_file(CHECKOUT / "LICENSE")},
        "table_document": {"path": rel(TABLE_DOCUMENT), "sha256": sha256_file(TABLE_DOCUMENT)},
    }
    readme = read_text(CHECKOUT / "README.md")
    manifest = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": rel(snapshot),
        "protocol_sha256": protocol_hash,
        "method": "Nova3D",
        "axis": "Table 7 Production Readiness",
        "manifest_frozen_before_scoring": True,
        "selection_policy": {
            "requested_assets": 0,
            "paper_cohort_is_not_a_local_cohort": True,
            "outcome_based_filtering": False,
            "reason": "no released or locally generated attributable final package",
        },
        "official_checkout": {
            "repository_url": preflight["official_source"]["url"],
            "path": rel(CHECKOUT),
            "branch": branch,
            "commit": commit,
            "commit_sha256_role": "canonical checkout identity",
            "inventory": inventory,
            "readme_backend_line": find_line(readme, "hosted generation backend is (currently) closed-source"),
            "readme_examples_line": find_line(readme, "coming soon"),
        },
        "blocker_codes": list(BLOCKERS),
        "assets": [],
        "input_evidence": evidence,
    }
    write_json(output / "manifest.json", manifest)
    manifest_hash = sha256_file(output / "manifest.json")
    write_json(output / "asset_records.json", [])

    local_results = build_local_results()
    summary = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": "BLOCKED",
        "evidence_state": "not_evaluable",
        "blocker_codes": list(BLOCKERS),
        "cohort": {
            "requested_assets": 0,
            "available_assets": 0,
            "unavailable_requested_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "paper_only_assets_separate": 54,
        },
        "official_checkout": {
            "commit": commit,
            "final_asset_count": inventory["final_asset_count"],
            "examples_directory_present": inventory["examples_directory_present"],
            "final_asset_suffix_counts": inventory["final_asset_suffix_counts"],
        },
        "local_results": local_results,
        "paper_reported_only": paper,
        "safety": {
            "network_accessed": False,
            "hosted_api_called": False,
            "paid_generation_called": False,
            "secret_read": False,
            "gpu_job_started": False,
        },
        "hashes": {
            "protocol_sha256": protocol_hash,
            "manifest_sha256": manifest_hash,
        },
    }
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})

    state_accounting = {
        name: {
            "pass": 0,
            "fail": 0,
            "not_evaluable_assets": 0,
            "denominator": 0,
            "conserved": 0 == summary["cohort"]["requested_assets"],
        }
        for name in LOCAL_COLUMNS
    }
    required = protocol["required_outputs"]
    required_paths = [safe(output / name, must_exist=False) for name in required]
    checks = {
        "manifest_identities_unique": len(manifest["assets"]) == len({row.get("asset_id") for row in manifest["assets"]}),
        "manifest_and_records_match": manifest["assets"] == read_json(output / "asset_records.json") == [],
        "requested_equals_available_plus_unavailable": 0 == 0 + 0,
        "required_denominators_are_zero_and_explicit": all(summary["cohort"][key] == 0 for key in (
            "requested_assets", "available_assets", "geometry_evaluable_assets", "package_evaluable_assets"
        )),
        "all_local_columns_not_evaluable": all(local_results[name]["state"] == "not_evaluable" for name in LOCAL_COLUMNS),
        "all_local_rates_have_explicit_zero_denominators": all(local_results[name]["denominator"] == 0 and local_results[name]["numerator"] == 0 for name in LOCAL_COLUMNS),
        "state_partitions_conserve_requested_denominator": all(row["conserved"] for row in state_accounting.values()),
        "paper_only_layer_is_separate": paper["cohort_assets"] == 54 and paper["local_reproduction"] is False,
        "checkout_commit_matches_table6_freeze": commit == EXPECTED_COMMIT,
        "checkout_contains_no_final_asset": inventory["final_asset_count"] == 0,
        "protocol_snapshot_hash_matches_source": protocol_hash == sha256_file(PROTOCOL),
        "manifest_sha256_recorded": summary["hashes"]["manifest_sha256"] == sha256_file(output / "manifest.json"),
        "all_referenced_evidence_exists_inside_workspace": all(path.is_file() for path in required_paths)
        and all(safe(WORKSPACE / item["path"]).is_file() for item in evidence.values()),
    }
    artifact_hashes = {
        name: sha256_file(output / name)
        for name in required
        if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "state_accounting": state_accounting,
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {[key for key, value in checks.items() if not value]}")
    return verify(output)


def verify(output: Path) -> dict[str, Any]:
    output = safe(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    self_check = read_json(output / "self_check.json")
    errors: list[str] = []
    for name in protocol.get("required_outputs", []):
        if not safe(output / name, must_exist=False).is_file():
            errors.append(f"missing required output: {name}")
    for name, expected in self_check.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact hash mismatch: {name}")
    if self_check.get("status") != "PASS":
        errors.append("self_check is not PASS")
    if manifest.get("assets") != records or records != []:
        errors.append("local asset records are not the frozen empty cohort")
    if summary.get("cohort", {}).get("available_assets") != 0:
        errors.append("local available-assets denominator is not zero")
    if any(summary.get("local_results", {}).get(name, {}).get("state") != "not_evaluable" for name in LOCAL_COLUMNS):
        errors.append("one or more local Table 7 columns are not not_evaluable")
    if summary.get("paper_reported_only", {}).get("cohort_assets") != 54:
        errors.append("paper-only N=54 evidence is absent")
    if sha256_file(output / "manifest.json") != self_check.get("manifest_sha256"):
        errors.append("manifest SHA-256 mismatch")
    if sha256_file(output / "protocol_snapshot.json") != self_check.get("protocol_sha256"):
        errors.append("protocol SHA-256 mismatch")
    return {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "local_n": len(records),
        "paper_only_n": summary.get("paper_reported_only", {}).get("cohort_assets"),
        "output": rel(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    cwd = safe(Path.cwd())
    output = safe(args.output if args.output.is_absolute() else cwd / args.output, must_exist=False)
    result = verify(output) if args.verify_only else create(output)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
