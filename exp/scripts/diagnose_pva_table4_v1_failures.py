#!/usr/bin/env python3
"""Read-only diagnosis of failures in the sealed PV-A v1 Table 4 release."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import tempfile
from typing import Any
import zlib


SCRIPT = Path(__file__).resolve()
EXP = SCRIPT.parent.parent
DEFAULT_ROOT = EXP / "runtime/pva_table1234_full_release_20260826/evaluation"
DEFAULT_OUTPUT = EXP / "runtime/pva_table4_v1_failure_diagnosis_20260827.json"
SCHEMA = "pva_table4_v1_failure_diagnosis_v1"
OURS_CATEGORIES = (
    "Science_First_aid_cabinet", "desk_with_drawer_card_catalog",
    "drawer_cabinet_with_sliding_drawers", "office_table_with_doors_or_drawers",
    "pictureX_0611_Cabinet_with_doors", "pictureX_0611_Cabinet_with_doors_and_drawers",
    "pictureX_0611_Desk_with_drawers_no_door", "pictureX_0611_Dressing_table",
    "pictureX_0611_Hutch_Cabinet", "pictureX_0611_Industrial_rolling_work_table",
    "pictureX_0611_Locker_box", "pictureX_0611_kitchen_cabinet",
)


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def metric_rows(connection: sqlite3.Connection, where: str = "1") -> list[dict[str, Any]]:
    query = f"""
      WITH t AS (
        SELECT a.category, COALESCE(json_extract(r.table4_json,'$.strict_collision_pass'),0) strict
        FROM results r JOIN assets a USING(ordinal) WHERE {where}
      )
      SELECT category,count(*),sum(strict),count(*)-sum(strict),1.0*sum(strict)/count(*)
      FROM t GROUP BY category ORDER BY count(*)-sum(strict) DESC, category
    """
    return [dict(zip(("category", "denominator", "passed", "failed", "pass_rate"), row)) for row in connection.execute(query)]


def build(root: Path) -> dict[str, Any]:
    root = root.resolve(strict=True)
    database = (root / "results.sqlite3").resolve(strict=True)
    receipt_path = (root / "full_release_receipt.json").resolve(strict=True)
    receipt = json.loads(receipt_path.read_text())
    database_sha = sha256_file(database)
    if receipt.get("result_database_sha256") != database_sha:
        raise RuntimeError("sealed database SHA256 mismatch")
    connection = sqlite3.connect(f"{database.as_uri()}?mode=ro&immutable=1", uri=True)
    connection.execute("PRAGMA query_only=ON")
    try:
        total, passed = connection.execute(
            "SELECT count(*),sum(COALESCE(json_extract(table4_json,'$.strict_collision_pass'),0)) FROM results"
        ).fetchone()
        overlap = []
        for rest, single, sobol, count in connection.execute("""
          WITH t AS (SELECT
            COALESCE(json_extract(table4_json,'$.rest_non_adjacent_cf'),0) r,
            COALESCE(json_extract(table4_json,'$.single_joint_sweep_cf'),0) s,
            COALESCE(json_extract(table4_json,'$.multi_joint_sobol_cf'),0) b,
            COALESCE(json_extract(table4_json,'$.strict_collision_pass'),0) strict
            FROM results)
          SELECT r,s,b,count(*) FROM t WHERE strict=0 GROUP BY r,s,b ORDER BY count(*) DESC
        """):
            overlap.append({"rest_pass": bool(rest), "single_pass": bool(single), "sobol_pass": bool(sobol), "assets": count})
        issues = []
        for worker, issue_json, count in connection.execute("""
          SELECT worker_status,json_extract(table4_json,'$.issues'),count(*) FROM results
          WHERE COALESCE(json_extract(table4_json,'$.measurement_complete'),0)=0
          GROUP BY worker_status,json_extract(table4_json,'$.issues') ORDER BY count(*) DESC
        """):
            examples = [dict(zip(("ordinal", "asset_id", "category"), row)) for row in connection.execute(
                """SELECT r.ordinal,a.asset_id,a.category FROM results r JOIN assets a USING(ordinal)
                   WHERE r.worker_status=? AND json_extract(r.table4_json,'$.issues')=? ORDER BY r.ordinal LIMIT 5""",
                (worker, issue_json),
            )]
            issues.append({"worker_status": worker, "issues": json.loads(issue_json), "assets": count, "examples": examples})
        placeholders = ",".join("?" for _ in OURS_CATEGORIES)
        query = f"""WITH t AS (SELECT a.category,COALESCE(json_extract(r.table4_json,'$.strict_collision_pass'),0) strict
          FROM results r JOIN assets a USING(ordinal) WHERE a.category IN ({placeholders}))
          SELECT category,count(*),sum(strict),count(*)-sum(strict),1.0*sum(strict)/count(*) FROM t GROUP BY category ORDER BY count(*)-sum(strict) DESC,category"""
        ours = [dict(zip(("category", "denominator", "passed", "failed", "pass_rate"), row)) for row in connection.execute(query, OURS_CATEGORIES)]
        all_categories = metric_rows(connection)
        samples: dict[str, list[dict[str, Any]]] = {phase: [] for phase in ("rest", "single_joint_sweep", "multi_joint_sobol")}
        sampled_assets: dict[str, set[str]] = {phase: set() for phase in samples}
        cursor = connection.execute("""SELECT r.ordinal,a.asset_id,a.category,r.table4_states_zlib
          FROM results r JOIN assets a USING(ordinal)
          WHERE COALESCE(json_extract(r.table4_json,'$.strict_collision_pass'),0)=0 AND r.table4_state_count>0 ORDER BY r.ordinal""")
        for ordinal, asset_id, category, blob in cursor:
            states = [json.loads(line) for line in zlib.decompress(blob).splitlines()]
            for state in states:
                phase = state.get("phase")
                if (phase in samples and len(samples[phase]) < 5
                        and asset_id not in sampled_assets[phase]
                        and int(state.get("non_adjacent_illegal_penetration_count", 0)) > 0):
                    samples[phase].append({
                        "ordinal": ordinal, "asset_id": asset_id, "category": category,
                        "phase": phase, "sample_index": state.get("sample_index"),
                        "joint_name": state.get("joint_name"),
                        "non_adjacent_contact_count": state.get("non_adjacent_contact_count"),
                        "non_adjacent_illegal_penetration_count": state.get("non_adjacent_illegal_penetration_count"),
                        "non_adjacent_max_penetration_m": state.get("non_adjacent_max_penetration_m"),
                    })
                    sampled_assets[phase].add(asset_id)
            if all(len(rows) >= 5 for rows in samples.values()):
                break
        result = {
            "schema_version": SCHEMA,
            "inputs": {"database": str(database), "database_sha256": database_sha, "receipt": str(receipt_path), "sqlite": "mode=ro&immutable=1; query_only=ON"},
            "strict_summary": {"denominator": total, "passed": passed, "failed": total - passed, "pass_rate": passed / total},
            "incomplete_issue_taxonomy": issues,
            "strict_failure_phase_overlap": overlap,
            "categories": {
                "ours_500k_same_12": ours,
                "lowest_pass_rate": sorted(all_categories, key=lambda row: (row["pass_rate"], -row["failed"], row["category"]))[:25],
                "largest_failure_contributors": all_categories[:25],
            },
            "collision_state_examples": samples,
            "collision_pair_evidence": {
                "link_pair_identity_available": False,
                "reason": "sealed v1 state schema stores aggregate contact/illegal counts and maximum depth, not link-pair identities",
                "required_for_pairs": "separate diagnostic replay of selected hash-bound asset/state with the frozen runtime and contact-query instrumentation",
            },
            "interpretation_boundaries": {
                "geometry_penetration": "states with non-adjacent illegal count > 0 and depth > 1e-6 are observed geometric penetrations",
                "functional_contact_without_metadata": "cannot be distinguished from unintended penetration because v1 has no semantic contact allowlist or link-pair metadata",
                "runtime_error": "reported separately by incomplete issue taxonomy and not classified as geometric penetration",
            },
            "implementation": {"script": str(SCRIPT), "script_sha256": sha256_file(SCRIPT)},
        }
        result["artifact_content_sha256"] = hashlib.sha256(canonical(result).encode()).hexdigest()
        return result
    finally:
        connection.close()


def write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="ascii") as stream:
            stream.write(canonical(value) + "\n")
        os.replace(name, path)
    finally:
        try: os.unlink(name)
        except FileNotFoundError: pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    value = build(args.root)
    write(args.output, value)
    print(canonical(value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
