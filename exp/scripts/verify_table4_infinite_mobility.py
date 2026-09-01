#!/usr/bin/env python3
"""Independent verifier for the blocked Infinite Mobility Table 4 receipt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))
import infinite_mobility_table123_common as cohort_common  # noqa: E402


BLOCKED = "BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT"
DATASET = "Infinite Mobility"
N_EVAL = 720
J_EVAL = 4723
COHORT_SHA256 = "cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08"
COHORT_CONTENT_SHA256 = "f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23"
TABLE2_SHA256 = "3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290"
TABLE2_RECORDS_SHA256 = "d488501734a41d4b814c294f7ad94ed529df72b0f99cfaed8b1d19a3bf1c2ada"
TABLE3_SHA256 = "52d03061d150e23f5f97e0227931047379969a5518c5448a14e7062a3ed6d611"
TABLE3_RECORDS_SHA256 = "e1ebf268e6839869e9d7e8d98e2ae0411e4ed17dea28b5c7692bef326b6f4113"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
    return hashlib.sha256(payload).hexdigest()


def manifest_self_hash(manifest: dict[str, object]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def read_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError(f"non-object row in {path}")
                rows.append(value)
    return rows


def source_identity_matches(
    source_row: object, record: object, *, expected_order: int
) -> bool:
    """Check that a receipt row is still bound to its frozen cohort row."""

    if not isinstance(source_row, dict) or not isinstance(record, dict):
        return False
    expected_binding = source_row.get("package_binding")
    if not isinstance(expected_binding, dict):
        return False
    scalar_fields = (
        "asset_id",
        "factory",
        "raw_category",
        "seed",
        "source",
        "original_status",
        "recovery_used",
        "urdf_relpath",
        "package_path",
    )
    if any(source_row.get(field) != record.get(field) for field in scalar_fields):
        return False
    if source_row.get("selection_index") != expected_order:
        return False
    if record.get("order") != expected_order or record.get("selection_index") != expected_order:
        return False
    if record.get("asset_key") != source_row.get("asset_id"):
        return False
    if record.get("urdf_sha256") != source_row.get("primary_urdf_sha256"):
        return False
    return (
        record.get("package_content_manifest_sha256")
        == expected_binding.get("content_manifest_sha256")
        and record.get("package_file_count") == expected_binding.get("file_count")
        and record.get("package_total_bytes") == expected_binding.get("total_bytes")
    )


def verify(output: Path) -> dict[str, object]:
    output = output.resolve(strict=True)
    manifest = read_json(output / "manifest.json")
    summary = read_json(output / "summary.json")
    records = read_jsonl(output / "asset_records.jsonl")
    if not isinstance(manifest, dict) or not isinstance(summary, dict):
        raise ValueError("manifest or summary is not an object")
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise ValueError("manifest source section missing")
    checks: dict[str, bool] = {}

    checks["manifest_self_hash"] = manifest.get("manifest_content_sha256") == manifest_self_hash(manifest)
    checks["dataset_and_status"] = (
        manifest.get("dataset") == DATASET
        and manifest.get("classification") == "FORMAL"
        and summary.get("dataset") == DATASET
        and summary.get("status") == "BLOCKED"
    )
    cohort_path = Path(str(source["cohort_manifest"]))
    table2_path = Path(str(source["table2_manifest"]))
    table3_root = Path(str(source["table3_receipt"]))
    table2_records_path = table2_path.parent / "records.jsonl"
    table3_manifest_path = table3_root / "manifest.json"
    table3_records_path = table3_root / "asset_records.jsonl"
    checks["upstream_hashes"] = (
        sha256_file(cohort_path) == COHORT_SHA256
        and sha256_file(table2_path) == TABLE2_SHA256
        and sha256_file(table2_records_path) == TABLE2_RECORDS_SHA256
        and sha256_file(table3_manifest_path) == TABLE3_SHA256
        and sha256_file(table3_records_path) == TABLE3_RECORDS_SHA256
    )
    cohort = read_json(cohort_path)
    if not isinstance(cohort, dict):
        raise ValueError("cohort manifest is not an object")
    checks["cohort_content_hash"] = cohort.get("manifest_content_sha256") == COHORT_CONTENT_SHA256
    assets = cohort.get("assets")
    checks["record_count"] = isinstance(assets, list) and len(assets) == len(records) == N_EVAL
    checks["record_order"] = [row.get("order") for row in records] == list(range(1, N_EVAL + 1))
    checks["source_identity_binding"] = (
        isinstance(assets, list)
        and len(assets) == len(records) == N_EVAL
        and all(
            source_identity_matches(source_row, record, expected_order=index)
            for index, (source_row, record) in enumerate(zip(assets, records), 1)
        )
    )
    checks["zero_collision_gate"] = (
        sum(int(row.get("native_collision_element_count", -1)) for row in records) == 0
        and all(row.get("collision_metric_status") == BLOCKED for row in records)
    )
    checks["no_state_queries"] = (
        (output / "state_records.jsonl").read_text(encoding="utf-8") == ""
        and all(not row.get("state_records") for row in records)
    )
    checks["denominators"] = (
        sum(int(row.get("movable_dof_count", -1)) for row in records) == J_EVAL
        and int(summary.get("n_eval", -1)) == N_EVAL
        and int(summary.get("j_eval", -1)) == J_EVAL
        and summary.get("state_plan", {}).get("total_expected") == 142463
    )
    metrics = summary.get("metrics", {})
    checks["all_metrics_n_e"] = all(
        isinstance(metrics.get(name), dict) and metrics[name].get("status") == "N/E"
        for name in (
            "rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf",
            "multi_joint_sobol_cf", "collision_state_rate", "aor", "max_penetration",
            "collision_free_range", "strict_collision_pass",
        )
    )
    claim = summary.get("claim_boundary", {})
    checks["claim_boundary"] = (
        claim.get("native_collision_element_total") == 0
        and claim.get("collision_queries_executed") is False
        and claim.get("vacuous_no_contact_is_not_pass") is True
    )

    # Rebind every package and URDF independently of the producer's compact
    # asset record. This is intentionally the expensive part of final review.
    package_rebind = True
    if isinstance(assets, list) and len(assets) == len(records):
        for source_row, record in zip(assets, records, strict=True):
            try:
                package = Path(str(source_row["package_path"])).resolve(strict=True)
                observed = cohort_common.package_binding(package)
                urdf = (package / str(source_row["urdf_relpath"])).resolve(strict=True)
                root = ET.parse(urdf).getroot()
                expected_binding = source_row["package_binding"]
                if observed != expected_binding:
                    package_rebind = False
                    break
                if observed.get("content_manifest_sha256") != record.get("package_content_manifest_sha256"):
                    package_rebind = False
                    break
                observed_urdf_sha = sha256_file(urdf)
                if observed_urdf_sha != source_row.get("primary_urdf_sha256"):
                    package_rebind = False
                    break
                if observed_urdf_sha != record.get("urdf_sha256"):
                    package_rebind = False
                    break
                if len(root.findall(".//collision")) != 0:
                    package_rebind = False
                    break
            except Exception:
                package_rebind = False
                break
    else:
        package_rebind = False
    checks["package_and_urdf_rebind"] = package_rebind

    artifact_manifest = read_json(output / "artifact_manifest.json")
    artifact_ok = isinstance(artifact_manifest, dict) and isinstance(artifact_manifest.get("files"), dict)
    if artifact_ok:
        for name, entry in artifact_manifest["files"].items():
            path = output / str(name)
            if not path.is_file() or path.stat().st_size != int(entry["bytes"]) or sha256_file(path) != entry["sha256"]:
                artifact_ok = False
                break
    checks["artifact_closure"] = bool(artifact_ok)
    report = (output / "report.md").read_text(encoding="utf-8")
    checks["report_claim"] = BLOCKED in report and "N/E" in report and "vacuous" in report

    receipt = {
        "schema_version": 1,
        "protocol_id": "urdf_sim_ready_table4_infinite_mobility_verify_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checked_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "checks": checks,
        "artifact_sha256": {
            name: sha256_file(output / name)
            for name in (
                "manifest.json", "asset_records.jsonl", "state_records.jsonl", "summary.json",
                "report.md", "protocol_snapshot.md", "environment.json", "artifact_manifest.json",
            )
        },
    }
    temporary = output / ".verification.json.tmp"
    temporary.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output / "verification.json")
    if receipt["status"] != "PASS":
        raise RuntimeError(f"verification failed: {checks}")
    return receipt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    receipt = verify(args.output)
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
