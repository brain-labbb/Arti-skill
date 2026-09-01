#!/usr/bin/env python3
"""Run project-native corner evaluation for the frozen Nano3D 33-template cohort.

The template repository is imported read-only.  Existing random-36 outcomes are
seeded into an exp-local cache; only corner cases absent from the base sweep are
compiled.  All temporary files, artifacts, and reports stay below ``exp``.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import statistics
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
RELIABILITY_ROOT = EXP_ROOT / "runtime" / "nano3d_seed_reliability"
DEFAULT_OUT = EXP_ROOT / "runtime" / "nano3d_corner"
BASE_SEEDS = list(range(36))

sys.path.insert(0, str(TEMPLATE_ROOT))

from agent.template_check import (  # noqa: E402
    _CACHE_SCHEMA,
    _domain_corner_stage,
    _domain_report,
    _legacy_corner_stage,
    mechanical_hash,
)
from agent.template_policy import rigid_seed_detail  # noqa: E402
from agent.template_sweep import SeedOutcome, compiled_artifact_is_valid  # noqa: E402


NUMERICAL_THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "VECLIB_MAXIMUM_THREADS": "1",
}


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def seed_base_cache(
    *, slug: str, stem: str, source_payload: dict[str, Any], cache_base: Path
) -> tuple[str, list[SeedOutcome], Path]:
    mechanical = mechanical_hash(TEMPLATE_ROOT, slug, stem)
    cache_root = cache_base / slug / mechanical
    outcomes: list[SeedOutcome] = []
    by_seed = {int(row["seed"]): row for row in source_payload["outcomes"]}
    if sorted(by_seed) != BASE_SEEDS:
        raise RuntimeError(f"{slug}: expected base seeds 0-35, got {sorted(by_seed)}")
    for seed in BASE_SEEDS:
        outcome = SeedOutcome.from_dict(by_seed[seed]["qc"])
        if outcome.verdict != "pass":
            raise RuntimeError(f"{slug} seed {seed}: base reliability outcome is not pass")
        if not compiled_artifact_is_valid(outcome.artifact_dir):
            raise RuntimeError(f"{slug} seed {seed}: base artifact is absent or hash-invalid")
        outcomes.append(outcome)
        dump_json(
            cache_root / "outcomes" / f"seed_{seed}.json",
            {
                "schema_version": _CACHE_SCHEMA,
                "mechanical_hash": mechanical,
                "outcome": outcome.to_dict(),
                "seeded_from": str(
                    RELIABILITY_ROOT / "templates" / slug / "outcomes.json"
                ),
            },
        )
    return mechanical, outcomes, cache_root


def normalize_cases(corner: dict[str, Any]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for index, raw in enumerate(corner.get("cases") or []):
        row = dict(raw)
        status = row.get("status") or row.get("verdict") or "fail"
        artifact_dir = row.get("artifact_dir")
        rigid_error = None
        artifact_valid = False
        if status == "pass":
            artifact_valid = compiled_artifact_is_valid(artifact_dir)
            if not artifact_valid:
                status = "fail"
                rigid_error = "corner artifact is absent or hash-invalid"
            else:
                urdf = (Path(str(artifact_dir)) / "model.urdf").read_text(encoding="utf-8")
                rigid_error = rigid_seed_detail(urdf)
                if rigid_error:
                    status = "fail"
        normalized.append(
            {
                "index": index,
                "name": row.get("name"),
                "seed": row.get("seed"),
                "status": status,
                "config": row.get("config") or {},
                "overrides": row.get("overrides") or {},
                "reason": row.get("reason"),
                "failure_type": row.get("failure_type_normalized")
                or row.get("failure_type"),
                "error": rigid_error or row.get("error") or row.get("failure_details"),
                "failures": row.get("failures") or [],
                "artifact_dir": artifact_dir,
                "artifact_valid": artifact_valid,
                "elapsed_s": float(row.get("elapsed_s") or 0.0),
                "motion_qc_report": row.get("motion_qc_report"),
                "joint_motion_coverage": row.get("joint_motion_coverage"),
            }
        )
    return normalized


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--slug", action="append", help="Run only selected cohort slug(s)")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    out_root = args.out.resolve()
    try:
        out_root.relative_to(EXP_ROOT.resolve())
    except ValueError as exc:
        raise SystemExit("--out must be inside /mnt/zsn/lyb/arti-skill/exp") from exc
    out_root.mkdir(parents=True, exist_ok=True)
    temp_root = out_root / "tmp"
    temp_root.mkdir(parents=True, exist_ok=True)
    os.environ.update(NUMERICAL_THREAD_ENV)
    os.environ["TMPDIR"] = str(temp_root)
    tempfile.tempdir = str(temp_root)

    source_manifest = json.loads(
        (RELIABILITY_ROOT / "manifest.json").read_text(encoding="utf-8")
    )
    cohort = list(source_manifest["templates"])
    requested = set(args.slug or [])
    if requested:
        unknown = requested - {row["slug"] for row in cohort}
        if unknown:
            raise SystemExit(f"--slug not in frozen cohort: {sorted(unknown)}")
        cohort = [row for row in cohort if row["slug"] in requested]

    manifest = {
        "schema_version": 1,
        "protocol": "nano3d_project_native_corner_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "template_root": str(TEMPLATE_ROOT),
        "base_seed_manifest": str(RELIABILITY_ROOT / "manifest.json"),
        "base_seeds": BASE_SEEDS,
        "domain_policy": "all authored TEMPLATE_CORNERS plus missing domain candidate/edge cases",
        "legacy_policy": "select_corner_seeds probes 0-511 and picks up to 12 reachable seeds absent from base 0-35",
        "strict_case_policy": "project native full compile + author tests + overlap/disconnected + motion QC; harness also rejects rigid URDF",
        "workers": max(1, args.workers),
        "timeout_s": args.timeout,
        "numerical_thread_env": NUMERICAL_THREAD_ENV,
        "templates": cohort,
    }
    dump_json(out_root / "manifest.json", manifest)

    cache_base = out_root / "cache"
    template_records: list[dict[str, Any]] = []
    started = time.monotonic()
    for position, template_row in enumerate(cohort, 1):
        slug = template_row["slug"]
        stem = template_row["stem"]
        result_path = out_root / "templates" / slug / "corner_result.json"
        if result_path.is_file() and not args.force:
            record = json.loads(result_path.read_text(encoding="utf-8"))
            if record.get("protocol") == manifest["protocol"]:
                print(f"[{position}/{len(cohort)}] {slug}: resume", flush=True)
                template_records.append(record)
                continue

        print(f"[{position}/{len(cohort)}] {slug}: corner", flush=True)
        source_path = RELIABILITY_ROOT / "templates" / slug / "outcomes.json"
        source_payload = json.loads(source_path.read_text(encoding="utf-8"))
        mechanical, base_outcomes, cache_root = seed_base_cache(
            slug=slug,
            stem=stem,
            source_payload=source_payload,
            cache_base=cache_base,
        )
        module = importlib.import_module(f"agent.templates.{slug}")
        domain, domain_audit = _domain_report(module)
        template_started = time.monotonic()
        if domain is not None:
            native = _domain_corner_stage(
                repo_root=TEMPLATE_ROOT,
                slug=slug,
                stem=stem,
                module=module,
                domain=domain,
                random_outcomes=base_outcomes,
                cache_root=cache_root,
                compile_timeout_s=args.timeout,
            )
        else:
            native = _legacy_corner_stage(
                repo_root=TEMPLATE_ROOT,
                slug=slug,
                stem=stem,
                mechanical=mechanical,
                cache_root=cache_root,
                max_workers=max(1, args.workers),
                compile_timeout_s=args.timeout,
                progress=None,
            )
        cases = normalize_cases(native)
        passed = sum(row["status"] == "pass" for row in cases)
        failed = len(cases) - passed
        record = {
            "schema_version": 1,
            "protocol": manifest["protocol"],
            "slug": slug,
            "stem": stem,
            "template_sha256": template_row["template_sha256"],
            "mechanical_hash": mechanical,
            "mode": "domain_cases" if domain is not None else "reachable_seed_corners",
            "domain_audit": domain_audit,
            "native_status": native.get("status"),
            "strict_status": "pass" if failed == 0 else "fail",
            "case_count": len(cases),
            "passed": passed,
            "failed": failed,
            "cases": cases,
            "selection_plan": native.get("plan"),
            "coverage": native.get("coverage"),
            "authored_case_count": native.get("authored_case_count"),
            "supplemental_cases": native.get("supplemental_cases") or [],
            "native_repair_findings": native.get("repair_findings") or [],
            "elapsed_wall_s": time.monotonic() - template_started,
        }
        dump_json(result_path, record)
        template_records.append(record)
        print(
            f"  mode={record['mode']} cases={len(cases)} pass={passed} fail={failed}",
            flush=True,
        )

    cases = [case for row in template_records for case in row["cases"]]
    elapsed = [float(case["elapsed_s"]) for case in cases]
    total_cases = len(cases)
    passed_cases = sum(case["status"] == "pass" for case in cases)
    summary = {
        "schema_version": 1,
        "protocol": manifest["protocol"],
        "status": "complete" if len(template_records) == len(cohort) else "partial",
        "template_count": len(template_records),
        "domain_template_count": sum(row["mode"] == "domain_cases" for row in template_records),
        "legacy_template_count": sum(
            row["mode"] == "reachable_seed_corners" for row in template_records
        ),
        "corner_case_count": total_cases,
        "corner_cases_passed": passed_cases,
        "corner_cases_failed": total_cases - passed_cases,
        "corner_case_pass_rate": passed_cases / total_cases if total_cases else None,
        "strict_all_corner_templates_passed": sum(
            row["strict_status"] == "pass" for row in template_records
        ),
        "native_corner_gates_passed": sum(
            row["native_status"] == "pass" for row in template_records
        ),
        "case_elapsed_s": {
            "total": sum(elapsed),
            "mean": statistics.mean(elapsed) if elapsed else None,
            "median": statistics.median(elapsed) if elapsed else None,
            "p95": percentile(elapsed, 0.95),
        },
        "harness_wall_s": time.monotonic() - started,
        "template_records": [
            {
                "slug": row["slug"],
                "mode": row["mode"],
                "case_count": row["case_count"],
                "passed": row["passed"],
                "failed": row["failed"],
                "strict_status": row["strict_status"],
                "native_status": row["native_status"],
            }
            for row in template_records
        ],
    }
    dump_json(out_root / "records.json", template_records)
    dump_json(out_root / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0 if summary["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
