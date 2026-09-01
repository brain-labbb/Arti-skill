#!/usr/bin/env python3
"""One-shot derivation of frozen constants for the PartNet-Mobility Table 5 toolchain.

Read-only over sources; prints a JSON constant block that is pasted into
table5_partnet_mobility_common.py before any simulator access.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from table5_partnet_mobility_common import (  # noqa: E402
    canonical_sha256,
    collect_artifact_set,
    package_binding,
    sha256_file,
)

EXP = Path(__file__).resolve().parents[1]
DATASET_ROOT = EXP / "PartNet-Mobility/data/dataset"
TABLE4_MANIFEST = EXP / "runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
UPSTREAM = {
    "table2": EXP / "runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z",
    "table3": EXP / "runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z",
    "table4": EXP / "runtime/urdf_table4_partnet_mobility_n800_20260813",
}
EXPECTED_TABLE4_MANIFEST_SHA256 = "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
EXPECTED_ORDERED_IDS_SHA256 = "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"


def main() -> int:
    payload = TABLE4_MANIFEST.read_bytes()
    import hashlib

    if hashlib.sha256(payload).hexdigest() != EXPECTED_TABLE4_MANIFEST_SHA256:
        print("table4 manifest sha mismatch", file=sys.stderr)
        return 2
    manifest = json.loads(payload)
    items = manifest["items"]
    assert len(items) == 800
    ids = [it["dataset_id"] for it in items]
    ordered_ids = hashlib.sha256(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode()
    ).hexdigest()
    assert ordered_ids == EXPECTED_ORDERED_IDS_SHA256, ordered_ids

    package_meta = []
    urdf_meta = []
    file_count = 0
    total_bytes = 0
    for index, item in enumerate(items):
        assert item["order"] == index
        dataset_id = str(item["dataset_id"])
        package = DATASET_ROOT / dataset_id
        binding = package_binding(package)
        package_meta.append(
            {
                "dataset_id": int(dataset_id),
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
            }
        )
        urdf_path = package / "mobility.urdf"
        urdf_hash = sha256_file(urdf_path)
        assert urdf_hash == item["urdf_sha256"], f"urdf drift {dataset_id}"
        urdf_meta.append(
            {
                "dataset_id": int(dataset_id),
                "urdf_relpath": f"{dataset_id}/mobility.urdf",
                "sha256": urdf_hash,
            }
        )
        file_count += binding["file_count"]
        total_bytes += binding["total_bytes"]
        if (index + 1) % 100 == 0:
            print(f"packages bound: {index + 1}/800", file=sys.stderr)

    # upstream artifact sets
    artifact_sets = {}
    for name, root in UPSTREAM.items():
        aset = collect_artifact_set(root)
        artifact_sets[name] = {
            "artifact_set": {
                "artifact_set_sha256": aset["artifact_set_sha256"],
                "file_count": aset["file_count"],
                "total_bytes": aset["total_bytes"],
            },
            "root": str(root),
        }
        print(f"artifact set {name}: {aset['file_count']} files", file=sys.stderr)

    # summary expectations (actual frozen summaries)
    t2 = json.loads((UPSTREAM["table2"] / "summary.json").read_text())
    t3 = json.loads((UPSTREAM["table3"] / "summary.json").read_text())
    t4 = json.loads((UPSTREAM["table4"] / "summary.json").read_text())
    t2_expect = {
        "schema_version": t2["schema_version"],
        "status": t2["status"],
        "dataset": t2["dataset"],
        "mode": t2["mode"],
        "n_eval": t2["n_eval"],
        "records_present": t2["records_present"],
        "error_count": t2["error_count"],
        "status_counts": t2["status_counts"],
        "strict_passed": t2["strict_urdf_pass"]["passed"],
        "metric_pass_counts": {
            key: t2["metrics"][key]["passed"]
            for key in (
                "collision_coverage",
                "finite_fields",
                "inertia_validity",
                "inertial_coverage",
                "parse_rate",
                "resource_resolution",
                "strict_urdf_pass",
                "valid_joint_spec",
                "valid_tree",
            )
        },
    }
    t3_expect = {
        "schema_version": t3["schema_version"],
        "status": t3["status"],
        "dataset": t3["dataset"],
        "n_eval": t3["n_eval"],
        "j_eval": t3["j_eval"],
        "parse_success": t3["parse_success"],
        "valid_tree": t3["valid_tree"],
        "status_counts": t3["status_counts"],
        "strict_passed": t3["metrics"]["strict_kinematic_pass"]["passed"],
    }
    t4_expect = {
        "status": t4["status"],
        "protocol_id": t4["protocol_id"],
        "selected": t4["cohort"]["selected"],
        "category_count": t4["cohort"]["category_count"],
        "load_success": t4["cohort"]["load_success"],
        "measurement_complete": t4["cohort"]["measurement_complete"],
        "strict_passed": t4["metrics"]["strict_collision_pass"]["passed"],
        "verification_status": json.loads(
            (UPSTREAM["table4"] / "verification.json").read_text()
        ).get("status"),
    }

    constants = {
        "FORMAL_TABLE4_MANIFEST_SHA256": EXPECTED_TABLE4_MANIFEST_SHA256,
        "FORMAL_ORDERED_IDS_SHA256": EXPECTED_ORDERED_IDS_SHA256,
        "FORMAL_ORDERED_PACKAGE_BINDING_SHA256": canonical_sha256(package_meta),
        "FORMAL_ORDERED_URDF_BINDING_SHA256": canonical_sha256(urdf_meta),
        "FORMAL_PACKAGE_FILE_COUNT": file_count,
        "FORMAL_PACKAGE_TOTAL_BYTES": total_bytes,
        "FORMAL_UPSTREAM_ARTIFACT_SETS": artifact_sets,
        "FORMAL_T2_EXPECTATIONS": t2_expect,
        "FORMAL_T3_EXPECTATIONS": t3_expect,
        "FORMAL_T4_EXPECTATIONS": t4_expect,
    }
    print(json.dumps(constants, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
