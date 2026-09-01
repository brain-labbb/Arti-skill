#!/usr/bin/env python3
"""Run frozen Table 4a mechanical-safety evaluation on Ours-500K.

The execution engine and fail-closed aggregation are shared with the frozen
Articraft-10K Table 4a runner. This adapter binds that implementation to the
complete 500-asset Brain cohort and its existing Ours Table 2/3/4 receipts.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4a_articraft10k as shared  # noqa: E402


DATASET_ROOT = REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
SOURCE_MANIFEST = (
    REPO
    / "exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/frozen_manifest.json"
)
TABLE4_STATE_RECORDS = SOURCE_MANIFEST.with_name("state_records.jsonl")
TABLE4_ASSET_RECORDS = SOURCE_MANIFEST.with_name("asset_records.jsonl")
TABLE3_RECORDS = (
    REPO
    / "exp/runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/asset_records.jsonl"
)
TABLE2_COHORT_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/manifest.json"
)

EXPECTED_SOURCE_MANIFEST_FILE_SHA256 = (
    "1b29d868112dcda326a08f8e3439d6b96c65833b99cc33af3bfcdb58fb4c2e24"
)
EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = (
    "19f3c5e6063864f4506a9c1fd9817c183ffb59bb559c644ef570b7a647d3b735"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "dcd19530ff3a3546fa149db58f331a042d5c3326f1b4fa1e5580914952e79289"
)
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = (
    "1fcefb39b36db33ce48fa8f52b161ac87eb2e81e0599dfd38c429e173b9b922e"
)
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = (
    "7095ca2c75b1fb7f9a108ad0ef6451dbf85847a2c5401202e87d1c8e09f9012d"
)
EXPECTED_TABLE3_RECORDS_SHA256 = (
    "0c96ff4cdb4d65fc4e275eb9addcb7e4a464256f495244a7157c991c5778da9a"
)
EXPECTED_TABLE2_COHORT_FILE_SHA256 = (
    "f6f2eb2e9a5a0b257d2843674e987946a9d014274348784018540772f2660b71"
)
EXPECTED_TABLE2_COHORT_CONTENT_SHA256 = (
    "8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa"
)
EXPECTED_CATEGORY_MAPPING_SHA256 = (
    "ef1b865bf7b90ad4694e8598a2e8e382058bcdb9cbea5e87cef351f072258e6b"
)
EXPECTED_CATEGORY_REVISION = "embedded_in_frozen_table4_manifest"

N_EVAL = 500
J_EVAL = 2467
EXPECTED_CATEGORY_COUNT = 12


def _joint_specs(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    specs: list[dict[str, Any]] = []
    for xml_index, joint in enumerate(root.findall("joint")):
        joint_type = str(joint.get("type", "")).strip()
        if joint_type == "fixed":
            continue
        name = str(joint.get("name", "")).strip()
        if not name or joint_type not in {"continuous", "prismatic", "revolute"}:
            raise SystemExit(f"unsupported movable joint in {urdf_path}: {name!r} {joint_type!r}")
        lower: float | None = None
        upper: float | None = None
        if joint_type != "continuous":
            limit = joint.find("limit")
            if limit is None:
                raise SystemExit(f"bounded joint has no limit in {urdf_path}: {name}")
            try:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
            except (KeyError, ValueError) as exc:
                raise SystemExit(f"invalid joint limit in {urdf_path}: {name}") from exc
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise SystemExit(f"non-evaluable joint range in {urdf_path}: {name}")
        specs.append(
            {
                "name": name,
                "type": joint_type,
                "lower": lower,
                "upper": upper,
                "xml_index": xml_index,
            }
        )
    return specs


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    if shared.sha256_bytes(payload) != EXPECTED_SOURCE_MANIFEST_FILE_SHA256:
        raise SystemExit("source manifest file sha256 mismatch")
    manifest = json.loads(payload)
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256:
        raise SystemExit("source manifest content sha256 mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} items")
    ids = [str(item["dataset_id"]) for item in items]
    ordered = shared.sha256_bytes(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode()
    )
    if ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(f"ordered asset id sha256 mismatch: {ordered}")

    total_joints = 0
    for index, item in enumerate(items):
        if int(item.get("order", -1)) != index:
            raise SystemExit(f"item order field mismatch at index {index}")
        if str(item.get("asset_id")) != str(item.get("dataset_id")):
            raise SystemExit(f"asset/dataset identity mismatch at index {index}")
        urdf_path = DATASET_ROOT / str(item["primary_urdf_relpath"])
        specs = _joint_specs(urdf_path)
        if len(specs) != int(item["movable_dof_count"]):
            raise SystemExit(f"movable joint count mismatch: {item['dataset_id']}")
        item["joint_specs"] = specs
        item["package_relpath"] = str(item["asset_root_relpath"])
        total_joints += len(specs)
    if total_joints != J_EVAL:
        raise SystemExit(f"frozen J_eval mismatch: {total_joints}")
    return manifest


def load_table4_strict_pass() -> dict[str, bool]:
    if shared.lam4a.sha256_file(TABLE4_ASSET_RECORDS) != EXPECTED_TABLE4_ASSET_RECORDS_SHA256:
        raise SystemExit("Table 4 asset records sha256 mismatch")
    result: dict[str, bool] = {}
    with TABLE4_ASSET_RECORDS.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            result[str(record["dataset_id"])] = bool(record["strict_collision_pass"])
    if len(result) != N_EVAL:
        raise SystemExit(f"Table 4 strict-pass record count mismatch: {len(result)}")
    return result


def hash_cross_check_covers_executed_states(aggregates: Mapping[str, Any]) -> bool:
    state_counts = aggregates["state_counts"]
    cross = state_counts["hash_cross_check"]
    classified = sum(int(cross[key]) for key in ("verified", "mismatch", "no_reference"))
    return classified == int(state_counts["executed"])


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
    table4_strict: Mapping[str, bool],
    category_info: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(passed), "detail": detail})

    items = manifest["items"]
    table2 = json.loads(TABLE2_COHORT_MANIFEST.read_text(encoding="utf-8"))
    table2_ids = [str(row["asset_id"]) for row in table2["assets"]]
    table4_ids = [str(row["asset_id"]) for row in items]
    check("source_manifest_file_sha256", shared.lam4a.sha256_file(SOURCE_MANIFEST) == EXPECTED_SOURCE_MANIFEST_FILE_SHA256)
    check("table2_cohort_file_sha256", shared.lam4a.sha256_file(TABLE2_COHORT_MANIFEST) == EXPECTED_TABLE2_COHORT_FILE_SHA256)
    check("cohort_is_exact_table2_sample", table2_ids == table4_ids)
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record["selection_index"]) == index
            and str(record["dataset_id"]) == str(items[index]["dataset_id"])
            for index, record in enumerate(records)
        ),
    )
    check("j_eval_denominator", aggregates["joint_level_full_range_cf"]["denominator"] == J_EVAL)
    check("retention_denominator", aggregates["collision_safe_dof_retention"]["denominator"] == J_EVAL)
    check(
        "urdf_identity_matches_frozen_manifest",
        all(
            (record.get("urdf_sha256") is None and record.get("status") != "completed")
            or record.get("urdf_sha256") == str(items[int(record["selection_index"])]["urdf_sha256"])
            for record in records
        ),
    )
    cross = aggregates["state_counts"]["hash_cross_check"]
    check("state_hash_cross_check_no_mismatch", cross["mismatch"] == 0, str(cross))
    check("state_hash_cross_check_complete", hash_cross_check_covers_executed_states(aggregates), str(cross))
    check("state_intended_count", aggregates["state_counts"]["intended"] == shared.SINGLE_SAMPLES * J_EVAL)
    check("category_mapping_sha256", category_info.get("category_mapping_sha256") == EXPECTED_CATEGORY_MAPPING_SHA256)
    check("category_count", category_info.get("eval_category_count") == EXPECTED_CATEGORY_COUNT)
    recomputed = shared.aggregate(records, table4_strict)
    check(
        "aggregate_recomputation_matches",
        shared.lam4a.canonical_sha256(recomputed)
        == shared.lam4a.canonical_sha256(dict(aggregates)),
    )
    return {
        "all_pass": all(row["pass"] for row in checks),
        "check_count": len(checks),
        "checks": checks,
    }


def configure_shared_runner() -> None:
    shared.SCRIPT = SCRIPT
    shared.SCHEMA_VERSION = "table4a-ours-500k/v1"
    shared.PROTOCOL_ID = "table4a_ours_500k_table2cohort_n500_v1"
    shared.DATASET = "Ours-500K"
    shared.DATASET_ROOT = DATASET_ROOT
    shared.SOURCE_MANIFEST = SOURCE_MANIFEST
    shared.EXPECTED_SOURCE_MANIFEST_FILE_SHA256 = EXPECTED_SOURCE_MANIFEST_FILE_SHA256
    shared.EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256
    shared.EXPECTED_ORDERED_IDS_SHA256 = EXPECTED_ORDERED_IDS_SHA256
    shared.TABLE4_STATE_RECORDS = TABLE4_STATE_RECORDS
    shared.EXPECTED_TABLE4_STATE_RECORDS_SHA256 = EXPECTED_TABLE4_STATE_RECORDS_SHA256
    shared.TABLE4_ASSET_RECORDS = TABLE4_ASSET_RECORDS
    shared.EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = EXPECTED_TABLE4_ASSET_RECORDS_SHA256
    shared.TABLE3_RECORDS = TABLE3_RECORDS
    shared.EXPECTED_TABLE3_RECORDS_SHA256 = EXPECTED_TABLE3_RECORDS_SHA256
    shared.TABLE2_COHORT_MANIFEST = TABLE2_COHORT_MANIFEST
    shared.EXPECTED_TABLE2_COHORT_FILE_SHA256 = EXPECTED_TABLE2_COHORT_FILE_SHA256
    shared.EXPECTED_TABLE2_COHORT_CONTENT_SHA256 = EXPECTED_TABLE2_COHORT_CONTENT_SHA256
    shared.CATEGORY_RECORDS_ROOT = DATASET_ROOT
    shared.EXPECTED_CATEGORY_MAPPING_SHA256 = EXPECTED_CATEGORY_MAPPING_SHA256
    shared.EXPECTED_CATEGORY_RECORDS_REVISION = EXPECTED_CATEGORY_REVISION
    shared.N_EVAL = N_EVAL
    shared.J_EVAL = J_EVAL
    shared.EXPECTED_CATEGORY_COUNT = EXPECTED_CATEGORY_COUNT
    shared.CHILD_TIMEOUT_SECONDS = 1800
    shared.WORKERS = 16
    shared.PRIVATE_GENESIS_CACHES = True
    shared.GENESIS_CACHE_POLICY = (
        "per-rank private GS_CACHE_FILE_PATH under output root; rank 1 warmup cache is copied "
        "as a read template into each later rank cache to prevent concurrent-write SIGABRTs"
    )
    shared.SELECTION_POLICY = (
        "all 500 entries from the frozen Ours Table 2 cohort in existing order; "
        "full acquired release, no resampling or result-based filtering"
    )
    shared.RUN_NOTES = [
        "Joint-level Full-range CF reuses all 51,807 frozen Ours Table 4 single-joint sweep states (K=21, endpoints included, other joints q=0); every q-vector hash is cross-checked.",
        "State collision oracle = Genesis contact-penetration backend (genesis_contact_penetration_v1), direct kinematic detection only, illegal iff eligible-pair penetration > 1e-6 m.",
        "Headline pair policy = distinct source links excluding direct parent-child; no method-specific allowance and no visual fallback.",
        "Normalized Clearance P5 is N/E under this oracle because separated-pair signed clearance is unavailable and no independent exact-distance backend is registered.",
        "Existing Strict Collision Pass values in DoF bins are historical Ours Table 4 PyBullet results reported alongside, not re-executed.",
    ]
    operationalization = dict(shared.OPERATIONALIZATION)
    operationalization.update(
        {
            "state_plan": (
                "Exact frozen Ours Table 4 K=21 sweep per movable joint; bounded limits from the "
                "source URDF, continuous interval [-pi, pi], all other joints q=0. All 51,807 "
                "full q-vector hashes are bound to frozen Table 4 state_records."
            ),
            "table3_joint_pass_source": (
                "Ours Table 3 asset_records joints[].joint_level_pass from frozen run "
                "urdf_table3_ours_500k_table2_n500_20260819T101000Z."
            ),
            "existing_strict_collision_pass_source": (
                "Ours Table 4 asset_records strict_collision_pass from frozen run "
                "urdf_table4_ours_500k_table2_n500_20260819T104011Z."
            ),
            "category_source": "raw_category embedded in the frozen Ours Table 4 manifest.",
            "percentile_policy": (
                "median/P90 of per-asset safe-DoF counts with linear interpolation over all "
                "500 assets, including fail-closed zeros."
            ),
        }
    )
    shared.OPERATIONALIZATION = operationalization
    shared.load_source_manifest = load_source_manifest
    shared.load_table4_strict_pass = load_table4_strict_pass
    shared._read_category = lambda asset_id: asset_id.split("/", 1)[0]
    shared._category_revision = lambda: EXPECTED_CATEGORY_REVISION
    shared.verify_run = verify_run


def main() -> int:
    configure_shared_runner()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return shared.run_child(args.job, args.result)
    if args.mode is None:
        raise SystemExit("--mode is required unless --child is given")
    if args.output_dir is None:
        timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        if args.mode == "formal":
            name = f"table4a_urdf_ours_500k_table2cohort_n500_{timestamp}"
        else:
            name = f"table4a_urdf_ours_500k_smoke_n{args.n}_{timestamp}"
        args.output_dir = REPO / "exp/runtime" / name
    return shared.run_scope(args)


if __name__ == "__main__":
    raise SystemExit(main())
