#!/usr/bin/env python3
"""Independent verifier for Table 2 supplementary runs.

Recomputes every aggregate from the child receipts and frozen manifest, and
checks denominator closure, cohort identity and artifact integrity.  The
verifier never trusts ``summary.json`` values without recomputation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]

TABLE1_MANIFEST = REPO / "exp/runtime/table1_artiverse/manifest.json"
EXPECTED_TABLE1_MANIFEST_SHA256 = "f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c"
ALLOWED_STATUSES = {"completed", "error", "timeout", "binding_failed"}


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


class CheckLog:
    def __init__(self) -> None:
        self.checks: list[dict[str, Any]] = []

    def add(self, name: str, passed: bool, detail: str = "") -> None:
        self.checks.append({"name": name, "passed": bool(passed), "detail": detail})

    @property
    def ok(self) -> bool:
        return all(check["passed"] for check in self.checks)


DEFAULT_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "manifest_root", "model_id",
    "raw_category", "source", "chunk_archive", "selection_rank",
    "selection_hash", "package", "primary_urdf_relative_path",
    "expected_declared_joint_count", "model_urdf_sha256_expected",
    "package_content_manifest_sha256_expected",
    "package_binding_files_expected",
)


def verify_run(
    output_root: Path,
    *,
    table1_manifest: Path | None = None,
    expected_table1_sha256: str | None = None,
    identity_fields: tuple[str, ...] | None = None,
    table1_id_key: str = "manifest_root",
) -> dict[str, Any]:
    log = CheckLog()
    output_root = Path(output_root)
    table1_manifest_path = Path(table1_manifest) if table1_manifest is not None else TABLE1_MANIFEST
    table1_expected_sha = expected_table1_sha256 if expected_table1_sha256 is not None else EXPECTED_TABLE1_MANIFEST_SHA256
    resolved_identity_fields = tuple(identity_fields) if identity_fields is not None else DEFAULT_IDENTITY_FIELDS

    manifest_path = output_root / "frozen_manifest.json"
    summary_path = output_root / "summary.json"
    records_path = output_root / "asset_records.jsonl"
    if not (manifest_path.exists() and summary_path.exists() and records_path.exists()):
        log.add("required_artifacts_present", False, "frozen_manifest/summary/asset_records missing")
        return {"status": "FAIL", "checks": log.checks}
    log.add("required_artifacts_present", True)

    manifest = load_json(manifest_path)
    declared_hash = manifest.get("manifest_content_sha256")
    body = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    computed_hash = canonical_sha256(body)
    log.add(
        "frozen_manifest_self_hash",
        declared_hash == computed_hash,
        f"declared={declared_hash} computed={computed_hash}",
    )

    items = manifest.get("items", [])
    provenance = manifest.get("provenance", {})
    n_eval = provenance.get("n_eval")
    j_eval = provenance.get("j_eval")
    log.add("cohort_size", len(items) == n_eval, f"items={len(items)} n_eval={n_eval}")
    log.add(
        "j_eval_consistency",
        sum(item.get("expected_declared_joint_count", 0) for item in items) == j_eval,
        f"j_eval={j_eval}",
    )

    # Cohort identity against the frozen Table 1 manifest (byte-level hash).
    if table1_manifest_path.exists():
        observed = sha256_file(table1_manifest_path)
        log.add(
            "table1_manifest_hash",
            observed == table1_expected_sha,
            f"observed={observed}",
        )
        table1 = load_json(table1_manifest_path)
        table1_order = [str(asset[table1_id_key]) for asset in table1.get("assets", [])][: len(items)]
        item_order = [item["asset_id"] for item in items]
        log.add(
            "cohort_order_matches_table1",
            table1_order == item_order,
            f"first={item_order[0] if item_order else None}",
        )
    else:
        log.add("table1_manifest_hash", False, "Table 1 manifest not found")

    # Unique selection indices and input identity hashes.
    seen_indices = set()
    identity_ok = True
    for item in items:
        if item["selection_index"] in seen_indices:
            identity_ok = False
            break
        seen_indices.add(item["selection_index"])
        expected_identity = canonical_sha256({
            field: item.get(field) for field in resolved_identity_fields
        })
        if item.get("input_identity_sha256") != expected_identity:
            identity_ok = False
            break
    log.add("input_identity_hashes", identity_ok)

    # Child receipts: presence, identity, allowed statuses.
    receipts: dict[int, Mapping[str, Any]] = {}
    receipts_ok = True
    status_ok = True
    for item in items:
        index = item["selection_index"]
        path = output_root / "children" / f"rank_{index:04d}.json"
        if not path.exists():
            receipts_ok = False
            break
        receipt = load_json(path)
        if receipt.get("input_identity_sha256") != item["input_identity_sha256"]:
            receipts_ok = False
            break
        if receipt.get("asset_id") != item["asset_id"]:
            receipts_ok = False
            break
        if receipt.get("status") not in ALLOWED_STATUSES:
            status_ok = False
        receipts[index] = receipt
    log.add("child_receipts_present_and_bound", receipts_ok)
    log.add("child_statuses_allowed", status_ok)

    # Recompute aggregates.
    asset_pass = 0
    link_declared = 0
    link_covered = 0
    link_complete_assets = 0
    portability_passed = 0
    portability_intended = 0
    dynamics_covered = 0
    dynamics_intended = 0
    complete_inertial = 0
    dynamic_links = 0
    parse_success = 0
    status_counts: dict[str, int] = {}
    for item in items:
        receipt = receipts[item["selection_index"]]
        audit = receipt["audit"]
        status_counts[receipt["status"]] = status_counts.get(receipt["status"], 0) + 1
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})
        asset_pass += int(visual.get("asset_passed", 0))
        link_declared += int(visual.get("visual_bearing_links_declared", 0))
        link_covered += int(visual.get("covered_visual_bearing_links", 0))
        link_complete_assets += int(bool(visual.get("link_extraction_complete")))
        portability_passed += int(portability.get("joints_passed", 0))
        portability_intended += int(portability.get("joints_intended", 0))
        dynamics_covered += int(dynamics.get("joints_covered", 0))
        dynamics_intended += int(dynamics.get("joints_intended", 0))
        complete_inertial += int(placeholder.get("complete_inertial_links", 0))
        dynamic_links += int(placeholder.get("dynamic_links", 0))
        parse_success += int(bool(audit.get("parse", {}).get("success")))

    # Denominator closure: intended joint counts must equal frozen J_eval even
    # when assets fail preflight (fail-closed).
    log.add(
        "joint_denominator_closure",
        portability_intended == j_eval and dynamics_intended == j_eval,
        f"portability_intended={portability_intended} dynamics_intended={dynamics_intended} j_eval={j_eval}",
    )

    summary = load_json(summary_path)
    metrics = summary.get("metrics", {})
    visual_summary = metrics.get("visual_bearing_collision_coverage", {})
    portability_summary = metrics.get("joint_limit_portability", {})
    dynamics_summary = metrics.get("joint_dynamics_coverage", {})
    placeholder_summary = metrics.get("placeholder_mass_incidence", {})

    def matches(cell: Mapping[str, Any], numerator: int, denominator: int) -> bool:
        return int(cell.get("numerator", -1)) == numerator and int(cell.get("denominator", -1)) == denominator

    log.add(
        "summary_visual_asset_level",
        matches(visual_summary.get("asset_level", {}), asset_pass, n_eval),
        f"recomputed={asset_pass}/{n_eval}",
    )
    log.add(
        "summary_visual_link_micro",
        matches(visual_summary.get("link_micro", {}), link_covered, link_declared),
        f"recomputed={link_covered}/{link_declared}",
    )
    log.add(
        "summary_link_extraction_coverage",
        matches(visual_summary.get("link_extraction_coverage", {}), link_complete_assets, n_eval),
        f"recomputed={link_complete_assets}/{n_eval}",
    )
    log.add(
        "summary_portability",
        matches(portability_summary.get("joint_level", {}), portability_passed, j_eval),
        f"recomputed={portability_passed}/{j_eval}",
    )
    log.add(
        "summary_dynamics",
        matches(dynamics_summary.get("joint_level", {}), dynamics_covered, j_eval),
        f"recomputed={dynamics_covered}/{j_eval}",
    )
    log.add(
        "summary_placeholder_coverage",
        matches(placeholder_summary.get("complete_inertial_coverage", {}), complete_inertial, dynamic_links),
        f"recomputed={complete_inertial}/{dynamic_links}",
    )
    log.add(
        "summary_n_eval_j_eval",
        summary.get("n_eval") == n_eval and summary.get("j_eval") == j_eval,
    )
    log.add(
        "summary_status_counts",
        summary.get("status_counts") == dict(sorted(status_counts.items())),
        f"recomputed={dict(sorted(status_counts.items()))}",
    )
    log.add(
        "summary_parse_success",
        summary.get("parse_success_assets") == parse_success,
        f"recomputed={parse_success}",
    )
    log.add(
        "summary_manifest_hash_binding",
        summary.get("frozen_manifest_sha256") == declared_hash,
    )

    # Protocol snapshot integrity.
    snapshot_path = output_root / "protocol_snapshot.md"
    if snapshot_path.exists():
        log.add(
            "protocol_snapshot_hash",
            sha256_file(snapshot_path) == summary.get("protocol_snapshot_sha256"),
        )
    else:
        log.add("protocol_snapshot_hash", False, "protocol_snapshot.md missing")

    # asset_records.jsonl integrity: selection order + per-asset pass fields.
    record_lines = [line for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    records_ok = len(record_lines) == len(items)
    if records_ok:
        for line, item in zip(record_lines, items):
            record = json.loads(line)
            receipt = receipts[item["selection_index"]]
            audit = receipt["audit"]
            table2 = audit.get("table2_supplementary", {})
            visual = table2.get("visual_bearing_collision_coverage", {})
            portability = table2.get("joint_limit_portability", {})
            dynamics = table2.get("joint_dynamics_coverage", {})
            if (
                record.get("asset_id") != item["asset_id"]
                or record.get("selection_index") != item["selection_index"]
                or record.get("status") != receipt["status"]
                or bool(record.get("visual_bearing_collision_coverage_asset_pass")) != bool(visual.get("asset_pass"))
                or int(record.get("joint_limit_portability_passed", -1)) != int(portability.get("joints_passed", 0))
                or int(record.get("joint_dynamics_covered", -1)) != int(dynamics.get("joints_covered", 0))
            ):
                records_ok = False
                break
    log.add("asset_records_consistent", records_ok)

    return {
        "status": "PASS" if log.ok else "FAIL",
        "output_root": str(output_root),
        "recomputed": {
            "asset_pass": asset_pass,
            "n_eval": n_eval,
            "link_micro": f"{link_covered}/{link_declared}",
            "portability": f"{portability_passed}/{j_eval}",
            "dynamics": f"{dynamics_covered}/{j_eval}",
            "complete_inertial": f"{complete_inertial}/{dynamic_links}",
            "status_counts": dict(sorted(status_counts.items())),
        },
        "checks": log.checks,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args(argv)
    result = verify_run(args.output_root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
