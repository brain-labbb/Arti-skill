#!/usr/bin/env python3
"""Emit the offline Table 7 PartCrafter representation-control audit."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
PAPER_TABLE = REPO / "exp/Nano3dresults.md"
TABLE6_PREFLIGHT = REPO / "exp/runtime/table6_mesh_cad_segmentation/preflight.json"
OUTPUT = REPO / "exp/runtime/table7_partcrafter"

STATUS = "PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE"
INTERPRETATION = "NOT_AN_EXPERIMENTAL_ZERO"
METRICS = (
    "watertight", "manifold", "open_edges", "degenerate_faces",
    "self_intersection", "source_kb", "urdf_kb", "mesh_kb",
    "portable_package", "deterministic_build", "semantic_complete",
    "kinematic_complete", "physical_complete",
)
REQUIRED = (
    "protocol_snapshot.json", "manifest.json", "asset_records.json",
    "summary.json", "self_check.json", "report.md",
)
KNOWN_METHOD_LOCATIONS = (
    REPO / "PartCrafter",
    REPO / ".cache/PartCrafter",
    REPO / ".cache/table6_sources/partcrafter",
    REPO / ".cache/table6_sources/PartCrafter",
    REPO / "exp/baselines/PartCrafter",
    REPO / "exp/baselines/PartCrafter-official",
    REPO / "exp/runtime/table6_partcrafter",
    REPO / "exp/runtime/partcrafter",
    REPO / "exp/runtime/partcrafter_outputs",
)


def safe(path: Path, *, must_exist: bool = True) -> Path:
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


def paper_evidence() -> dict[str, Any]:
    text = read_text(PAPER_TABLE)
    prefix = "| PartCrafter (paper, N=30 GLB assets) |"
    fragments = (
        "0.73 per asset", "0.76 per asset", "53/asset mean",
        "N/A (1,394k triangles)", "27.9 MB GLB/asset",
        "N/A (output artifact only)",
    )
    matches = [
        (number, line)
        for number, line in enumerate(text.splitlines(), start=1)
        if line.startswith(prefix) and all(fragment in line for fragment in fragments)
    ]
    if len(matches) != 1:
        raise RuntimeError("frozen PartCrafter Table 7 paper row drifted")
    line_number, row = matches[0]
    return {
        "evidence_class": "paper_only_transcription",
        "local_reproduction": False,
        "must_not_be_merged_with_local_results": True,
        "source": rel(PAPER_TABLE),
        "source_sha256": sha256_file(PAPER_TABLE),
        "source_line": line_number,
        "cohort_assets": 30,
        "representation": "GLB output artifacts",
        "values": {
            "watertight_per_asset": 0.73,
            "manifold_per_asset": 0.76,
            "open_edges_per_asset_mean": 53,
            "degenerate_faces": None,
            "paper_auxiliary_triangles_per_asset": 1394000,
            "self_intersection": None,
            "source_kb": None,
            "urdf_kb": None,
            "mesh_mb_per_asset": 27.9,
            "portable_package": None,
            "deterministic_build": None,
            "semantic_complete": None,
            "kinematic_complete": None,
            "physical_complete": None,
        },
        "raw_markdown_row": row,
    }


def bounded_inventory() -> dict[str, Any]:
    existing_roots = [rel(path) for path in KNOWN_METHOD_LOCATIONS if safe(path, must_exist=False).exists()]
    scripts = safe(REPO / "exp/scripts")
    method_runners = sorted(
        entry.name
        for entry in scripts.iterdir()
        if "partcrafter" in entry.name.lower() and entry.name != SCRIPT.name
    )
    runtime = safe(REPO / "exp/runtime")
    runtime_entries = sorted(
        entry.name
        for entry in runtime.iterdir()
        if "partcrafter" in entry.name.lower() and entry.name != OUTPUT.name
    )
    return {
        "scope": "bounded top-level checks only; no recursive workspace scan",
        "checked_exact_method_locations": [rel(path) for path in KNOWN_METHOD_LOCATIONS],
        "existing_method_locations": existing_roots,
        "method_execution_runner_candidates": method_runners,
        "method_runtime_candidates": runtime_entries,
        "attributable_final_asset_count": 0,
    }


def not_evaluable(reason: str) -> dict[str, Any]:
    return {
        "state": "not_evaluable",
        "reason": reason,
        "numerator": 0,
        "denominator": 0,
        "rate": None,
    }


def local_results() -> dict[str, dict[str, Any]]:
    reason = (
        "No PartCrafter-attributable final GLB package is present in the bounded "
        "local evidence scope; the N=30 row is paper-only and was not reproduced."
    )
    results = {name: not_evaluable(reason) for name in METRICS}
    for name in ("open_edges", "degenerate_faces"):
        results[name].update({"total": None, "mean_per_asset": None})
    for name in ("source_kb", "urdf_kb", "mesh_kb"):
        results[name].update({"total_kb": None, "mean_kb_per_asset": None})
    return results


def evidence_files() -> dict[str, dict[str, str]]:
    paths = {
        "table6_representation_preflight": TABLE6_PREFLIGHT,
        "paper_table": PAPER_TABLE,
    }
    return {
        name: {"path": rel(path), "sha256": sha256_file(path)}
        for name, path in paths.items()
    }


def validate_inputs() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    preflight = read_json(TABLE6_PREFLIGHT)
    paper = paper_evidence()
    inventory = bounded_inventory()
    local = preflight.get("local_common_articulated_outputs", {})
    if preflight.get("status") != "PAPER_ONLY" or preflight.get("execution_status") != "NOT_RUN_NOT_APPLICABLE":
        raise RuntimeError("Table 6 representation-control status drifted")
    if local.get("found") is not False or local.get("attributable_to_control") is not False:
        raise RuntimeError("Table 6 local-output evidence drifted")
    if inventory["existing_method_locations"] or inventory["method_execution_runner_candidates"] or inventory["method_runtime_candidates"]:
        raise RuntimeError("a local PartCrafter method source, runner, or runtime now exists; N=0 audit is stale")
    return preflight, paper, inventory


def report_text(summary: dict[str, Any]) -> str:
    paper = summary["paper_reported_only"]["values"]
    return f"""# Table 7: PartCrafter representation-control audit

- Status: **{STATUS} / {INTERPRETATION}**.
- Local attributable final assets: **N=0**.
- Paper-only cohort kept separate: **N=30 GLB assets**.
- Network, download, generation, and GPU use by this runner: **none**.

## Local common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| PartCrafter (local N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

Local `N=0` means no attributable output is available, not measured zero
performance and not a failed experimental run.

## Separate paper-only transcription

The frozen paper row reports N=30 GLB assets: watertight
`{paper['watertight_per_asset']}`, manifold `{paper['manifold_per_asset']}`,
mean open edges `{paper['open_edges_per_asset_mean']}`, and GLB size
`{paper['mesh_mb_per_asset']} MB/asset`. The reported 1,394k triangles are
auxiliary triangle count, not degenerate faces. These values are never used as
local measurements.
"""


def current_checks(
    output: Path,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records: list[Any],
    summary: dict[str, Any],
) -> dict[str, bool]:
    _, live_paper, live_inventory = validate_inputs()
    cohort = summary["cohort"]
    columns = summary["local_results"]
    evidence = manifest["input_evidence"]
    return {
        "required_outputs_match_protocol": tuple(protocol.get("required_outputs", [])) == REQUIRED,
        "all_required_outputs_exist": all(safe(output / name, must_exist=False).is_file() for name in REQUIRED),
        "protocol_snapshot_exact": sha256_file(output / "protocol_snapshot.json") == sha256_file(PROTOCOL),
        "protocol_hash_recorded": manifest["protocol_sha256"] == summary["hashes"]["protocol_sha256"],
        "manifest_hash_recorded": sha256_file(output / "manifest.json") == summary["hashes"]["manifest_sha256"],
        "manifest_identities_unique": len(records) == len({row.get("asset_id") for row in records}) == 0,
        "asset_records_match_manifest": records == manifest["assets"] == [],
        "requested_denominator_accounted": cohort["requested_assets"] == cohort["pass_assets"] + cohort["fail_assets"] + cohort["unavailable_assets"],
        "all_local_denominators_zero": all(cohort[name] == 0 for name in ("requested_assets", "available_assets", "geometry_evaluable_assets", "package_evaluable_assets")),
        "all_13_columns_present": set(columns) == set(METRICS) and len(columns) == 13,
        "all_13_columns_not_evaluable": all(row["state"] == "not_evaluable" for row in columns.values()),
        "all_rates_explicit_n0": all(row["numerator"] == 0 and row["denominator"] == 0 and row["rate"] is None for row in columns.values()),
        "paper_only_n30_is_separate": cohort["paper_only_assets_separate"] == 30 and summary["paper_reported_only"]["local_reproduction"] is False,
        "paper_values_match_frozen_row": summary["paper_reported_only"] == live_paper,
        "paper_values_not_local_scores": summary["paper_values_used_as_local_results"] is False,
        "status_exact": manifest["status"] == summary["status"] == STATUS,
        "interpretation_exact": manifest["interpretation"] == summary["interpretation"] == INTERPRETATION,
        "bounded_inventory_still_empty": live_inventory["attributable_final_asset_count"] == 0 and not live_inventory["existing_method_locations"] and not live_inventory["method_execution_runner_candidates"] and not live_inventory["method_runtime_candidates"],
        "evidence_hashes_match": all(safe(WORKSPACE / row["path"]).is_file() and sha256_file(WORKSPACE / row["path"]) == row["sha256"] for row in evidence.values()),
        "runner_safety_flags_clear": not any(summary["runner_safety"].values()),
    }


def create(output: Path) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    safe(output.parent).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = safe(output)
    protocol = read_json(PROTOCOL)
    preflight, paper, inventory = validate_inputs()
    if protocol.get("protocol_id") != "nano3d_table7_production_readiness_v1" or tuple(protocol.get("required_outputs", [])) != REQUIRED:
        raise RuntimeError("shared Table 7 protocol drifted")

    snapshot = safe(output / "protocol_snapshot.json", must_exist=False)
    temporary = safe(output / "protocol_snapshot.json.tmp", must_exist=False)
    shutil.copyfile(safe(PROTOCOL), temporary)
    temporary.replace(snapshot)
    protocol_hash = sha256_file(snapshot)
    manifest = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": rel(snapshot),
        "protocol_sha256": protocol_hash,
        "method": "PartCrafter",
        "role": "supplementary representation control",
        "axis": "Table 7 Production Readiness",
        "status": STATUS,
        "interpretation": INTERPRETATION,
        "manifest_frozen_before_scoring": True,
        "selection_policy": {
            "requested_assets": 0,
            "paper_cohort_is_not_a_local_cohort": True,
            "outcome_based_filtering": False,
            "reason": "no attributable released or locally generated PartCrafter final package",
        },
        "bounded_local_inventory": inventory,
        "table6_evidence_status": preflight["status"],
        "assets": [],
        "input_evidence": evidence_files(),
    }
    write_json(output / "manifest.json", manifest)
    write_json(output / "asset_records.json", [])
    manifest_hash = sha256_file(output / "manifest.json")
    summary = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": STATUS,
        "interpretation": INTERPRETATION,
        "evidence_state": "not_evaluable",
        "cohort": {
            "requested_assets": 0,
            "available_assets": 0,
            "pass_assets": 0,
            "fail_assets": 0,
            "unavailable_assets": 0,
            "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0,
            "paper_only_assets_separate": 30,
        },
        "local_results": local_results(),
        "paper_reported_only": paper,
        "paper_values_used_as_local_results": False,
        "runner_safety": {
            "network_accessed": False,
            "download_started": False,
            "generation_started": False,
            "gpu_job_started": False,
        },
        "hashes": {"protocol_sha256": protocol_hash, "manifest_sha256": manifest_hash},
    }
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})
    records = read_json(output / "asset_records.json")
    checks = current_checks(output, protocol, manifest, records, summary)
    artifact_hashes = {
        name: sha256_file(output / name)
        for name in REQUIRED
        if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "state_accounting": {
            name: {"pass": 0, "fail": 0, "not_evaluable_assets": 0, "denominator": 0, "conserved": True}
            for name in METRICS
        },
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {[name for name, passed in checks.items() if not passed]}")
    return verify(output)


def verify(output: Path) -> dict[str, Any]:
    output = safe(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    recorded = read_json(output / "self_check.json")
    checks = current_checks(output, protocol, manifest, records, summary)
    errors = [name for name, passed in checks.items() if not passed]
    for name, expected in recorded.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact_hash:{name}")
    if recorded.get("status") != "PASS":
        errors.append("recorded_self_check_status")
    return {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "audit_status": summary.get("status"),
        "interpretation": summary.get("interpretation"),
        "local_n": len(records),
        "paper_only_n": summary.get("cohort", {}).get("paper_only_assets_separate"),
        "not_evaluable_columns": sum(row.get("state") == "not_evaluable" for row in summary.get("local_results", {}).values()),
        "output": rel(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    cwd = safe(Path.cwd())
    if cwd != WORKSPACE:
        raise RuntimeError(f"run from authorized workspace root: {WORKSPACE}")
    output = safe(args.output if args.output.is_absolute() else cwd / args.output, must_exist=False)
    result = verify(output) if args.verify_only else create(output)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
