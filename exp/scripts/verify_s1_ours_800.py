#!/usr/bin/env python3
"""Independently verify an Ours-800 Supplementary Table S1 run."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_s1_ours_800 as runner  # noqa: E402


def verify_run(run_dir: Path, *, formal: bool = True) -> dict[str, Any]:
    run_dir = run_dir.resolve(strict=True)
    config = runner.base.load_json_object(
        run_dir / "frozen_config.json", "frozen config"
    )
    summary = runner.base.load_json_object(run_dir / "summary.json", "summary")
    manifest = runner.base.load_json_object(run_dir / "manifest.json", "manifest")
    records = runner.base.load_jsonl(run_dir / "asset_records.jsonl")
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool) -> None:
        checks.append({"check": name, "pass": bool(passed)})

    check(
        "protocol_identity",
        config.get("protocol_id")
        == runner.PROTOCOL_ID
        == summary.get("protocol_id")
        == manifest.get("protocol_id"),
    )
    check(
        "record_count",
        len(records) == summary.get("n_eval") == manifest.get("n_eval")
        and (not formal or len(records) == runner.N_EVAL),
    )
    check(
        "aggregate_recomputation",
        runner.base.aggregate_s1(records) == summary.get("metrics"),
    )
    check(
        "ordered_identity",
        runner.base.ordered_ids_sha256(
            [row["dataset_id"] for row in records]
        )
        == config["source"]["ordered_dataset_ids_sha256"]
        == runner.EXPECTED_ORDERED_IDS_SHA256,
    )
    check(
        "source_hashes",
        config["source"]["cohort_manifest_sha256"]
        == runner.EXPECTED_SOURCE_MANIFEST_SHA256
        and config["source"]["table4_asset_records_sha256"]
        == runner.EXPECTED_ASSET_RECORDS_SHA256
        and config["source"]["table4_state_records_sha256"]
        == runner.EXPECTED_STATE_RECORDS_SHA256
        and runner.base.sha256_file(runner.TABLE4_VERIFICATION)
        == runner.EXPECTED_TABLE4_VERIFICATION_SHA256,
    )
    artifact_checks = []
    for name, binding in manifest.get("artifacts", {}).items():
        path = run_dir / name
        artifact_checks.append(
            path.is_file()
            and runner.base.sha256_file(path) == binding.get("sha256")
            and path.stat().st_size == binding.get("bytes")
        )
    check("artifact_bindings", len(artifact_checks) == 6 and all(artifact_checks))
    manifest_payload = dict(manifest)
    observed_content_hash = manifest_payload.pop("manifest_content_sha256", None)
    check(
        "manifest_self_hash",
        observed_content_hash == runner.base.canonical_sha256(manifest_payload),
    )
    check(
        "formal_headline_invariants",
        not formal
        or (
            summary["metrics"]["receipt_bound_assets"]["passed"] == 0
            and summary["metrics"]["deterministic_rebuild_match"][
                "eligible_assets"
            ]
            == 0
            and summary["metrics"]["allowance_density"]["registered_pairs"]
            == 0
            and summary["metrics"]["strict_pass_no_method_allowance"][
                "passed"
            ]
            == runner.EXPECTED_FORMAL_STRICT_PASSED
        ),
    )
    result = {
        "status": "PASS" if all(row["pass"] for row in checks) else "FAIL",
        "all_pass": all(row["pass"] for row in checks),
        "check_count": len(checks),
        "checks": checks,
    }
    runner.base.atomic_write_json(run_dir / "verification.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    result = verify_run(args.run_dir, formal=not args.smoke)
    print(runner.base.canonical_json(result))
    return 0 if result["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
