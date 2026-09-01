#!/usr/bin/env python3
"""Verify frozen identities and recompute aggregates for an Ours-500K S1 run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_s1_ours_500k as runner  # noqa: E402


def verify_run(run_dir: Path, *, formal: bool = True) -> dict:
    config = runner.base.load_json_object(run_dir / "frozen_config.json", "frozen config")
    summary = runner.base.load_json_object(run_dir / "summary.json", "summary")
    records = runner.base.load_jsonl(run_dir / "asset_records.jsonl")
    checks = []
    def check(name: str, passed: bool) -> None:
        checks.append({"check": name, "pass": bool(passed)})
    check("protocol_identity", config.get("protocol_id") == runner.PROTOCOL_ID == summary.get("protocol_id"))
    check("record_count", len(records) == summary.get("n_eval") and (not formal or len(records) == runner.N_EVAL))
    check("aggregate_recomputation", runner.base.aggregate_s1(records) == summary.get("metrics"))
    check("ordered_identity", runner.base.ordered_ids_sha256([row["dataset_id"] for row in records]) == config["source"]["ordered_dataset_ids_sha256"])
    check("source_hashes", config["source"]["cohort_manifest_sha256"] == runner.EXPECTED_SOURCE_MANIFEST_SHA256
          and config["source"]["table4_asset_records_sha256"] == runner.EXPECTED_ASSET_RECORDS_SHA256
          and config["source"]["table4_state_records_sha256"] == runner.EXPECTED_STATE_RECORDS_SHA256)
    result = {"status": "PASS" if all(row["pass"] for row in checks) else "FAIL",
              "all_pass": all(row["pass"] for row in checks), "check_count": len(checks), "checks": checks}
    (run_dir / "verification.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    result = verify_run(args.run_dir, formal=not args.smoke)
    print(json.dumps(result, indent=2))
    return 0 if result["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
