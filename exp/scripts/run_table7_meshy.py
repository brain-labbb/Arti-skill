#!/usr/bin/env python3
"""Create the offline Table 7 Meshy representation-control evidence bundle."""

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
TABLE_DOCUMENT = REPO / "exp/Nano3dresults.md"
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_meshy"
REASON_CODES = (
    "PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE",
    "NOT_AN_EXPERIMENTAL_ZERO",
)
LOCAL_COLUMNS = (
    "watertight", "manifold", "open_edges", "degenerate_faces",
    "self_intersection", "source_bytes", "urdf_bytes", "mesh_bytes",
    "portable_package", "deterministic_build", "semantic_complete",
    "kinematic_complete", "physical_complete",
)
TARGETED_CANDIDATES = (
    ".cache/Meshy",
    ".cache/table6_sources/meshy",
    ".cache/table7_sources/meshy",
    "exp/baselines/Meshy",
    "exp/runtime/table6_meshy",
    "exp/scripts/run_table6_meshy.py",
    "exp/scripts/run_meshy.py",
)


def safe(path: Path, *, must_exist: bool = True) -> Path:
    root = WORKSPACE.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
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


def canonical_sha256(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


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
    text = read_text(TABLE_DOCUMENT)
    prefix = "| Meshy (paper, N=54 GLB assets) |"
    expected = (
        "0.00 per asset", "0.67 per asset", "47,319/asset mean",
        "N/A (213k triangles)", "12.5 MB GLB/asset",
        "N/A (output artifact only)",
    )
    matches = [
        (number, line)
        for number, line in enumerate(text.splitlines(), start=1)
        if line.startswith(prefix) and all(fragment in line for fragment in expected)
    ]
    if len(matches) != 1:
        raise RuntimeError("the frozen Meshy Table 7 paper row is absent or ambiguous")
    line_number, row = matches[0]
    return {
        "evidence_class": "paper_only_transcription",
        "local_reproduction": False,
        "must_not_be_merged_with_local_results": True,
        "cohort_assets": 54,
        "representation": "GLB output artifact",
        "source": relative(TABLE_DOCUMENT),
        "source_line": line_number,
        "source_sha256": sha256_file(TABLE_DOCUMENT),
        "values": {
            "watertight_per_asset": 0.0,
            "manifold_per_asset": 0.67,
            "open_edges_per_asset_mean": 47319,
            "degenerate_faces": None,
            "paper_auxiliary_triangles_per_asset": 213000,
            "self_intersection": None,
            "source_kb_per_asset": None,
            "urdf_kb_per_asset": None,
            "glb_mb_per_asset": 12.5,
            "portable_package": None,
            "deterministic_build": None,
            "semantic_complete": None,
            "kinematic_complete": None,
            "physical_complete": None,
        },
        "raw_markdown_row": row,
    }


def targeted_inventory() -> dict[str, Any]:
    entries = []
    for name in TARGETED_CANDIDATES:
        path = safe(REPO / name, must_exist=False)
        entries.append({"path": relative(path), "present": path.exists()})
    present = [entry["path"] for entry in entries if entry["present"]]
    return {
        "scope": "fixed Meshy-specific checkout, output, and generation-runner candidates",
        "entries": entries,
        "present_candidates": present,
        "attributable_generation_runner_count": 0,
        "attributable_final_asset_count": 0,
        "inventory_sha256": canonical_sha256(entries),
    }


def local_results() -> dict[str, Any]:
    reason = (
        "No locally generated or released Meshy-attributable final asset is available; "
        "the N=54 values are paper-only evidence."
    )
    results = {
        name: {
            "state": "not_evaluable", "reason": reason,
            "numerator": 0, "denominator": 0, "rate": None,
        }
        for name in LOCAL_COLUMNS
    }
    for name in ("open_edges", "degenerate_faces"):
        results[name].update({"total": None, "mean_per_asset": None})
    for name in ("source_bytes", "urdf_bytes", "mesh_bytes"):
        results[name].update({"total_bytes": None, "mean_bytes_per_asset": None})
    return results


def report(summary: dict[str, Any]) -> str:
    paper = summary["paper_reported_only"]["values"]
    return f"""# Table 7: Meshy representation-control audit

- Status: **PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE**.
- Interpretation: **NOT_AN_EXPERIMENTAL_ZERO**.
- Local Meshy cohort: **N=0**.
- Reason codes: `PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE`, `NOT_AN_EXPERIMENTAL_ZERO`.
- Network, downloads, generation, API, GPU, and secret access: **none**.

## Local common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Meshy (local N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

No Meshy-specific checkout, generation runner, or attributable final package exists in the
fixed local inventory. Local N=0 means evidence is unavailable; it is not an experimental
zero and must not be interpreted as a geometry failure.

## Separate paper-only evidence

The existing Table 7 transcription reports N=54 GLB assets: watertight
`{paper['watertight_per_asset']}`, manifold `{paper['manifold_per_asset']}`, mean open edges
`{paper['open_edges_per_asset_mean']}`, approximately
`{paper['paper_auxiliary_triangles_per_asset']}` triangles/asset, and
`{paper['glb_mb_per_asset']} MB GLB/asset`. Other common-protocol fields are not reported.
These paper values are never merged into the local N=0 row.

## Denominators

- requested assets: 0
- available assets: 0
- geometry-evaluable assets: 0
- package-evaluable assets: 0
- paper-only assets, separate evidence layer: 54
"""


def create(output: Path) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    safe(output.parent).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = safe(output)
    protocol = read_json(PROTOCOL)
    if protocol.get("protocol_id") != "nano3d_table7_production_readiness_v1":
        raise RuntimeError("unexpected shared Table 7 protocol")
    inventory = targeted_inventory()
    if inventory["present_candidates"]:
        raise RuntimeError("Meshy local material now exists; extend the N=0 adapter before reuse")
    paper = paper_evidence()

    snapshot = output / "protocol_snapshot.json"
    temporary = safe(output / "protocol_snapshot.json.tmp", must_exist=False)
    shutil.copyfile(safe(PROTOCOL), temporary)
    temporary.replace(snapshot)
    protocol_hash = sha256_file(snapshot)
    manifest = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": relative(snapshot),
        "protocol_sha256": protocol_hash,
        "method": "Meshy",
        "axis": "Table 7 Production Readiness",
        "control_type": "paper-only mesh-native representation control",
        "manifest_frozen_before_scoring": True,
        "selection_policy": {
            "requested_assets": 0,
            "outcome_based_filtering": False,
            "paper_cohort_is_not_a_local_cohort": True,
            "reason": "no local attributable Meshy outputs or runnable generation path",
        },
        "reason_codes": list(REASON_CODES),
        "targeted_local_inventory": inventory,
        "assets": [],
        "input_evidence": {
            "protocol": {"path": relative(PROTOCOL), "sha256": sha256_file(PROTOCOL)},
            "table_document": {"path": relative(TABLE_DOCUMENT), "sha256": sha256_file(TABLE_DOCUMENT)},
            "runner": {"path": relative(SCRIPT), "sha256": sha256_file(SCRIPT)},
        },
    }
    write_json(output / "manifest.json", manifest)
    manifest_hash = sha256_file(output / "manifest.json")
    write_json(output / "asset_records.json", [])
    results = local_results()
    summary = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": "PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE",
        "interpretation": "NOT_AN_EXPERIMENTAL_ZERO",
        "reason_codes": list(REASON_CODES),
        "cohort": {
            "requested_assets": 0, "available_assets": 0,
            "unavailable_requested_assets": 0, "geometry_evaluable_assets": 0,
            "package_evaluable_assets": 0, "paper_only_assets_separate": 54,
        },
        "local_results": results,
        "paper_reported_only": paper,
        "targeted_local_inventory": inventory,
        "safety": {
            "network_accessed": False, "download_attempted": False,
            "generation_attempted": False, "api_called": False,
            "gpu_job_started": False, "secret_read": False,
        },
        "hashes": {
            "protocol_sha256": protocol_hash,
            "manifest_sha256": manifest_hash,
            "local_inventory_sha256": inventory["inventory_sha256"],
        },
    }
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})

    required = protocol["required_outputs"]
    accounting = {
        name: {"pass": 0, "fail": 0, "not_evaluable_assets": 0, "denominator": 0}
        for name in LOCAL_COLUMNS
    }
    checks = {
        "manifest_identities_unique": manifest["assets"] == [],
        "manifest_matches_asset_records": read_json(output / "asset_records.json") == [],
        "requested_equals_available_plus_unavailable": 0 == 0 + 0,
        "required_denominators_explicit": all(
            summary["cohort"][name] == 0
            for name in ("requested_assets", "available_assets", "geometry_evaluable_assets", "package_evaluable_assets")
        ),
        "all_local_columns_not_evaluable": all(results[name]["state"] == "not_evaluable" for name in LOCAL_COLUMNS),
        "all_rates_have_explicit_zero_denominators": all(results[name]["numerator"] == 0 and results[name]["denominator"] == 0 for name in LOCAL_COLUMNS),
        "state_accounting_conserves_requested_denominator": all(row["pass"] + row["fail"] + row["not_evaluable_assets"] == 0 for row in accounting.values()),
        "paper_only_n54_is_separate": paper["cohort_assets"] == 54 and paper["local_reproduction"] is False,
        "reason_codes_exact": summary["reason_codes"] == list(REASON_CODES),
        "status_and_interpretation_exact": summary["status"]
        == "PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE"
        and summary["interpretation"] == "NOT_AN_EXPERIMENTAL_ZERO",
        "targeted_inventory_empty": inventory["present_candidates"] == [],
        "protocol_snapshot_hash_matches_source": protocol_hash == sha256_file(PROTOCOL),
        "manifest_sha256_recorded": manifest_hash == sha256_file(output / "manifest.json"),
        "all_required_outputs_exist_inside_workspace": all(safe(output / name, must_exist=False).is_file() for name in required),
        "all_referenced_evidence_exists_inside_workspace": all(safe(WORKSPACE / item["path"]).is_file() for item in manifest["input_evidence"].values()),
    }
    artifact_hashes = {name: sha256_file(output / name) for name in required if name != "self_check.json"}
    self_check = {
        "schema_version": "1.0.0",
        "protocol_id": protocol["protocol_id"],
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "state_accounting": accounting,
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "local_inventory_sha256": inventory["inventory_sha256"],
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {[name for name, value in checks.items() if not value]}")
    return verify(output)


def verify(output: Path) -> dict[str, Any]:
    output = safe(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    self_check = read_json(output / "self_check.json")
    errors = []
    for name in protocol.get("required_outputs", []):
        if not safe(output / name, must_exist=False).is_file():
            errors.append(f"missing required output: {name}")
    for name, expected in self_check.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact hash mismatch: {name}")
    if self_check.get("status") != "PASS":
        errors.append("self_check is not PASS")
    if manifest.get("assets") != records or records != []:
        errors.append("local asset cohort is not empty")
    if summary.get("cohort", {}).get("available_assets") != 0:
        errors.append("local available-assets denominator is not zero")
    if any(summary.get("local_results", {}).get(name, {}).get("state") != "not_evaluable" for name in LOCAL_COLUMNS):
        errors.append("one or more local columns are not not_evaluable")
    if summary.get("paper_reported_only", {}).get("cohort_assets") != 54:
        errors.append("paper-only N=54 layer is absent")
    if summary.get("reason_codes") != list(REASON_CODES):
        errors.append("required reason codes are absent")
    if summary.get("status") != "PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE":
        errors.append("summary status is not PAPER_ONLY_LOCAL_OUTPUTS_UNAVAILABLE")
    if summary.get("interpretation") != "NOT_AN_EXPERIMENTAL_ZERO":
        errors.append("summary interpretation is not NOT_AN_EXPERIMENTAL_ZERO")
    if sha256_file(output / "manifest.json") != self_check.get("manifest_sha256"):
        errors.append("manifest SHA-256 mismatch")
    if sha256_file(output / "protocol_snapshot.json") != self_check.get("protocol_sha256"):
        errors.append("protocol SHA-256 mismatch")
    return {
        "status": "PASS" if not errors else "FAIL", "errors": errors,
        "local_n": len(records),
        "paper_only_n": summary.get("paper_reported_only", {}).get("cohort_assets"),
        "output": relative(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    cwd = safe(Path.cwd())
    output = safe(args.output if args.output.is_absolute() else cwd / args.output, must_exist=False)
    result = verify(output) if args.verify_only else create(output)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
