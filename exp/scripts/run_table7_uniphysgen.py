#!/usr/bin/env python3
"""Audit the frozen UniPhysGen release for Table 7 Production Readiness.

This runner is intentionally a release audit, not an inference harness.  The
frozen official checkout contains no runnable implementation, checkpoint,
output contract, or final simulation package, so it emits an explicit N=0
not-evaluable result without converting release absence into a method failure.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table7_production_readiness_protocol_v1.json"
DEFAULT_TABLE6_PREFLIGHT = REPO_ROOT / "exp/runtime/table6_uniphysgen/preflight.json"
DEFAULT_SOURCE = REPO_ROOT / ".cache/table6_sources/uniphysgen/repo"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table7_uniphysgen"
EXPECTED_PLACEHOLDER_FILES = [".gitignore", "LICENSE", "README.md"]
TABLE7_COLUMNS = [
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
CODE_SUFFIXES = {".py", ".sh", ".cpp", ".cc", ".c", ".cu", ".h", ".hpp", ".rs"}
CHECKPOINT_SUFFIXES = {".ckpt", ".pt", ".pth", ".safetensors", ".onnx"}
PACKAGE_SUFFIXES = {
    ".urdf", ".sdf", ".mjcf", ".usd", ".usda", ".usdc", ".glb", ".gltf",
    ".obj", ".ply", ".stl", ".dae", ".blend",
}
REQUIRED_OUTPUTS = [
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
]


def contained(path: Path, *, must_exist: bool = True) -> Path:
    root = WORKSPACE_ROOT.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def workspace_relative(path: Path) -> str:
    return contained(path).relative_to(contained(WORKSPACE_ROOT)).as_posix()


def read_json(path: Path) -> Any:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    contained(path, must_exist=False).write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, value: str) -> None:
    contained(path, must_exist=False).write_text(value, encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(source: Path, *args: str) -> str:
    source = contained(source)
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LC_ALL": "C",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_OPTIONAL_LOCKS": "0",
    }
    result = subprocess.run(
        ["git", "-C", str(source), *args],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return result.stdout


def tracked_inventory(source: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    raw = git(source, "ls-files", "--stage", "-z")
    for entry in raw.split("\0"):
        if not entry:
            continue
        metadata, relative = entry.split("\t", 1)
        mode, blob_oid, stage = metadata.split()
        if stage != "0" or mode == "120000":
            raise RuntimeError(f"unsupported tracked entry: {relative} mode={mode} stage={stage}")
        path = contained(source / relative)
        rows.append(
            {
                "path": path.relative_to(contained(source)).as_posix(),
                "git_mode": mode,
                "git_blob_oid": blob_oid,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return sorted(rows, key=lambda row: row["path"])


def worktree_inventory(source: Path) -> list[dict[str, Any]]:
    source = contained(source)
    rows: list[dict[str, Any]] = []
    for current_raw, directories, files in os.walk(source, followlinks=False):
        current = contained(Path(current_raw))
        directories[:] = sorted(name for name in directories if name != ".git")
        for name in sorted(files):
            path = contained(current / name)
            rows.append(
                {
                    "path": path.relative_to(source).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return sorted(rows, key=lambda row: row["path"])


def source_audit(source: Path, table6_preflight: dict[str, Any]) -> dict[str, Any]:
    source = contained(source)
    tracked = tracked_inventory(source)
    worktree = worktree_inventory(source)
    tracked_paths = [row["path"] for row in tracked]
    worktree_paths = [row["path"] for row in worktree]
    head = git(source, "rev-parse", "HEAD").strip()
    branch = git(source, "symbolic-ref", "--short", "HEAD").strip()
    status_lines = [line for line in git(source, "status", "--porcelain=v1", "--untracked-files=all").splitlines() if line]
    code = [path for path in worktree_paths if Path(path).suffix.lower() in CODE_SUFFIXES]
    checkpoints = [path for path in worktree_paths if Path(path).suffix.lower() in CHECKPOINT_SUFFIXES]
    packages = [path for path in worktree_paths if Path(path).suffix.lower() in PACKAGE_SUFFIXES]
    output_contract = [
        path for path in worktree_paths
        if "output_contract" in path.lower() or "output-schema" in path.lower()
    ]
    pinned_head = table6_preflight["official_source"]["head"]
    release_placeholder = (
        tracked_paths == EXPECTED_PLACEHOLDER_FILES
        and worktree_paths == EXPECTED_PLACEHOLDER_FILES
        and not code
        and not checkpoints
        and not packages
        and not output_contract
    )
    return {
        "local_path": workspace_relative(source),
        "head_commit": head,
        "branch": branch,
        "table6_pinned_head": pinned_head,
        "head_matches_table6_pin": head == pinned_head,
        "clean_worktree": not status_lines,
        "git_status_porcelain": status_lines,
        "tracked_file_count": len(tracked),
        "tracked_files": tracked,
        "worktree_file_count_excluding_git": len(worktree),
        "worktree_files_excluding_git": worktree,
        "expected_placeholder_files": EXPECTED_PLACEHOLDER_FILES,
        "release_placeholder": release_placeholder,
        "runnable_code_candidates": code,
        "checkpoint_candidates": checkpoints,
        "output_contract_candidates": output_contract,
        "simulation_package_candidates": packages,
        "attributable_final_simulation_packages": [],
    }


def not_evaluable(reason: str) -> dict[str, Any]:
    return {
        "state": "not_evaluable",
        "numerator": 0,
        "denominator": 0,
        "value": None,
        "reason": reason,
    }


def make_manifest(
    protocol: dict[str, Any],
    protocol_sha: str,
    table6_preflight: dict[str, Any],
    table6_sha: str,
    audit: dict[str, Any],
) -> dict[str, Any]:
    blockers = [
        {
            "id": "RUNNABLE_CODE_UNRELEASED",
            "evidence": "The exhaustive checkout inventory contains no executable method source or inference entrypoint.",
        },
        {
            "id": "COMPATIBLE_CHECKPOINT_UNRELEASED",
            "evidence": "The exhaustive checkout inventory contains no checkpoint or weight payload.",
        },
        {
            "id": "OUTPUT_PACKAGE_CONTRACT_UNRELEASED",
            "evidence": "The checkout defines no prediction/final-package schema or dependency contract.",
        },
        {
            "id": "FINAL_SIMULATION_PACKAGE_UNAVAILABLE",
            "evidence": "No tracked or local checkout file is a candidate URDF/USD/SDF/MJCF or mesh-bearing final simulation package.",
        },
    ]
    return {
        "schema_version": "1.0.0",
        "method": "UniPhysGen",
        "evaluation_axis": "Table 7 Production Readiness",
        "audit_type": "frozen_official_release_static_audit",
        "status": "RELEASE_BLOCKED",
        "interpretation": "NOT_AN_EXPERIMENTAL_ZERO",
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": "exp/runtime/table7_uniphysgen/protocol_snapshot.json",
        "protocol_snapshot_sha256": protocol_sha,
        "frozen_input": {
            "table6_preflight": workspace_relative(DEFAULT_TABLE6_PREFLIGHT),
            "table6_preflight_sha256": table6_sha,
        },
        "selection_policy": {
            "requested_assets": 0,
            "available_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "outcome_based_reselection": False,
            "reason": "No official final simulation package was released or locally generated.",
        },
        "official_checkout": audit,
        "blockers": blockers,
        "paper_value_policy": "No paper-reported value is substituted for a local Table 7 measurement.",
        "assets": [],
    }


def make_summary(protocol: dict[str, Any], manifest_sha: str, blockers: list[dict[str, str]]) -> dict[str, Any]:
    reason = "No released or locally generated attributable final simulation package exists."
    return {
        "schema_version": "1.0.0",
        "method": "UniPhysGen",
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": sha256_file(DEFAULT_OUTPUT / "protocol_snapshot.json"),
        "manifest_sha256": manifest_sha,
        "status": "RELEASE_BLOCKED",
        "interpretation": "NOT_AN_EXPERIMENTAL_ZERO",
        "run_started": False,
        "paper_numbers_used_as_local_results": False,
        "cohort": {
            "requested_assets": 0,
            "available_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "unavailable_assets": 0,
        },
        "artifact_availability": {
            "released_final_packages": 0,
            "locally_generated_final_packages": 0,
            "recovered_final_packages": 0,
            "attributable_final_packages": 0,
        },
        "performance_columns": {column: not_evaluable(reason) for column in TABLE7_COLUMNS},
        "blockers": blockers,
    }


def report_text(manifest: dict[str, Any], summary: dict[str, Any]) -> str:
    audit = manifest["official_checkout"]
    cells = " | ".join("not_evaluable" for _ in TABLE7_COLUMNS)
    return f"""# Table 7 UniPhysGen Production Readiness Release Audit

Status: **RELEASE_BLOCKED**  
Interpretation: **NOT_AN_EXPERIMENTAL_ZERO**  
Evaluated final packages: **N=0**

The frozen official checkout remains a release placeholder at commit
`{audit['head_commit']}`. Its exhaustive non-`.git` inventory contains only
`.gitignore`, `LICENSE`, and `README.md`; their byte sizes, Git blob IDs, and
SHA-256 hashes are recorded in `manifest.json`. There is no runnable code,
compatible checkpoint, output package contract, or attributable final
simulation package.

This is a release-readiness blocker, not a failed UniPhysGen experiment and not
evidence that the method generated zero successful assets. No inference,
generation, geometry scoring, package test, or deterministic rebuild was run,
and no paper value was substituted for a local measurement.

## Denominators

- requested assets: 0
- available assets: 0
- geometry-evaluable assets: 0
- package-evaluable assets: 0

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| UniPhysGen (official release placeholder; N=0) | {cells} |

Every performance cell is `not_evaluable`: there is no final artifact on which
to measure a pass, failure, byte size, or error count.

## Blocking release items

1. Runnable official inference code and environment specification.
2. Compatible official checkpoint with provenance and license.
3. Official output/final-package schema and dependency contract.
4. At least one attributable final simulation package, released or generated.

Protocol SHA-256: `{summary['protocol_sha256']}`  
Manifest SHA-256: `{summary['manifest_sha256']}`
"""


def checks(
    output: Path,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records: list[Any],
    summary: dict[str, Any],
    live_audit: dict[str, Any],
) -> dict[str, bool]:
    denominators = summary["cohort"]
    return {
        "required_outputs_exactly_declared": protocol["required_outputs"] == REQUIRED_OUTPUTS,
        "all_required_outputs_exist": all(contained(output / name).is_file() for name in REQUIRED_OUTPUTS),
        "protocol_snapshot_exact": sha256_file(output / "protocol_snapshot.json") == sha256_file(DEFAULT_PROTOCOL),
        "manifest_hash_recorded_and_exact": sha256_file(output / "manifest.json") == summary["manifest_sha256"],
        "manifest_identities_unique": not records and manifest["assets"] == [],
        "checkout_matches_recorded_inventory": live_audit == manifest["official_checkout"],
        "checkout_matches_table6_pin": live_audit["head_matches_table6_pin"],
        "release_placeholder_reconfirmed": live_audit["release_placeholder"],
        "no_attributable_final_package": not live_audit["attributable_final_simulation_packages"],
        "denominators_all_zero": all(denominators[key] == 0 for key in denominators),
        "requested_denominator_accounted": denominators["requested_assets"] == len(records) == 0,
        "all_performance_columns_present": set(summary["performance_columns"]) == set(TABLE7_COLUMNS),
        "all_performance_columns_not_evaluable": all(
            value["state"] == "not_evaluable" for value in summary["performance_columns"].values()
        ),
        "all_rates_have_explicit_zero_denominators": all(
            value["numerator"] == 0 and value["denominator"] == 0 and value["value"] is None
            for value in summary["performance_columns"].values()
        ),
        "status_is_release_blocked": summary["status"] == manifest["status"] == "RELEASE_BLOCKED",
        "interpretation_is_not_experimental_zero": (
            summary["interpretation"] == manifest["interpretation"] == "NOT_AN_EXPERIMENTAL_ZERO"
        ),
        "paper_values_not_used": summary["paper_numbers_used_as_local_results"] is False,
    }


def verify(output: Path) -> dict[str, Any]:
    output = contained(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    self_check = read_json(output / "self_check.json")
    table6 = read_json(DEFAULT_TABLE6_PREFLIGHT)
    live_audit = source_audit(DEFAULT_SOURCE, table6)
    current = checks(output, protocol, manifest, records, summary, live_audit)
    hash_matches = {
        name: sha256_file(output / name) == expected
        for name, expected in self_check["artifact_hashes"].items()
    }
    failures = [name for name, passed in current.items() if not passed]
    failures.extend(f"artifact_hash:{name}" for name, passed in hash_matches.items() if not passed)
    if self_check.get("status") != "PASS":
        failures.append("recorded_self_check_status")
    return {
        "status": "PASS" if not failures else "FAIL",
        "checks": current,
        "artifact_hash_matches": hash_matches,
        "failures": failures,
    }


def run(output: Path) -> dict[str, Any]:
    output = contained(output, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)
    protocol_source = contained(DEFAULT_PROTOCOL)
    snapshot = contained(output / "protocol_snapshot.json", must_exist=False)
    snapshot.write_bytes(protocol_source.read_bytes())
    protocol = read_json(snapshot)
    if protocol["required_outputs"] != REQUIRED_OUTPUTS:
        raise RuntimeError("shared protocol required_outputs changed")
    table6 = read_json(DEFAULT_TABLE6_PREFLIGHT)
    audit = source_audit(DEFAULT_SOURCE, table6)
    if not audit["head_matches_table6_pin"] or not audit["release_placeholder"]:
        raise RuntimeError("frozen checkout changed; a new representation-aware audit is required")
    manifest = make_manifest(
        protocol,
        sha256_file(snapshot),
        table6,
        sha256_file(DEFAULT_TABLE6_PREFLIGHT),
        audit,
    )
    write_json(output / "manifest.json", manifest)
    records: list[Any] = []
    write_json(output / "asset_records.json", records)
    summary = make_summary(protocol, sha256_file(output / "manifest.json"), manifest["blockers"])
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(manifest, summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})
    current_checks = checks(output, protocol, manifest, records, summary, audit)
    artifact_hashes = {
        name: sha256_file(output / name)
        for name in REQUIRED_OUTPUTS
        if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "status": "PASS" if all(current_checks.values()) else "FAIL",
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "checks": current_checks,
        "protocol_sha256": sha256_file(snapshot),
        "manifest_sha256": sha256_file(output / "manifest.json"),
        "artifact_hash_scope": "all required outputs except self_check.json to avoid circular hashing",
        "artifact_hashes": artifact_hashes,
        "denominator_self_check": {
            "requested_assets": 0,
            "manifest_assets": len(manifest["assets"]),
            "asset_records": len(records),
            "available_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
        },
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        failed = [name for name, passed in current_checks.items() if not passed]
        raise RuntimeError(f"self-check failed: {failed}")
    return verify(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if Path.cwd().resolve(strict=True) != WORKSPACE_ROOT.resolve(strict=True):
        raise RuntimeError(f"run from authorized workspace root: {WORKSPACE_ROOT}")
    output = contained(args.output, must_exist=args.verify_only)
    result = verify(output) if args.verify_only else run(output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
