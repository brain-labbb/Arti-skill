#!/usr/bin/env python3
"""Run the frozen 33-template x 36-seed Nano3D reliability cohort.

The procedural-template repository is imported read-only.  Every manifest,
temporary file, compiled artifact, and report is written below ``exp``.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
ASSET_LIST = EXP_ROOT / "Nano3dasset.md"
DEFAULT_OUT = EXP_ROOT / "runtime" / "nano3d_seed_reliability"
SEEDS = list(range(36))
ASSET_RE = re.compile(
    r"\]\((/mnt/zsn/lyb/arti-skill/(seed_exports(?:_physics_10)?)/([^/]+)/seed_(-?\d+))\)"
)

sys.path.insert(0, str(TEMPLATE_ROOT))

from agent.template_registry import TEMPLATE_REGISTRY  # noqa: E402
from agent.template_sweep import (  # noqa: E402
    compiled_artifact_is_valid,
    run_seed_outcomes,
)


COMPILE_PROBE = r'''
import hashlib
import json
import shutil
import sys
import tempfile
import traceback
from pathlib import Path

from agent.compiler import compile_urdf_report
from agent.template_sweep import _GENERIC_MODEL_TEMPLATE, _resolve_config_from_seed, _config_to_dict

slug, stem, seed_text, work_parent_text, publish_text = sys.argv[1:]
seed = int(seed_text)
work_parent = Path(work_parent_text)
publish = Path(publish_text)
work = Path(tempfile.mkdtemp(prefix=f".compile_{slug}_{seed}_", dir=work_parent))
started = __import__("time").monotonic()
try:
    config = _config_to_dict(_resolve_config_from_seed(slug, seed))
    script = work / "model.py"
    script.write_text(_GENERIC_MODEL_TEMPLATE.format(slug=slug, stem=stem, seed=seed), encoding="utf-8")
    report = compile_urdf_report(
        script,
        sdk_package="sdk",
        run_checks=False,
        target="full",
        rewrite_visual_glb=False,
        motion_qc=False,
    )
    urdf = report.urdf_xml or ""
    if not urdf.strip():
        raise RuntimeError("compiler returned empty URDF")
    (work / "model.urdf").write_text(urdf, encoding="utf-8")
    (work / "artifact.json").write_text(json.dumps({
        "schema_version": 1,
        "slug": slug,
        "seed": seed,
        "config": config,
        "urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
        "probe": "compile_only_run_checks_false_motion_qc_false",
    }, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    if publish.exists():
        shutil.rmtree(publish)
    publish.parent.mkdir(parents=True, exist_ok=True)
    work.replace(publish)
    payload = {"verdict": "pass", "elapsed_s": __import__("time").monotonic() - started,
               "artifact_dir": str(publish.resolve()), "config": config, "error": None}
except BaseException as exc:
    payload = {"verdict": "fail", "elapsed_s": __import__("time").monotonic() - started,
               "artifact_dir": None, "config": locals().get("config", {}),
               "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()[-4000:]}
finally:
    if work.exists():
        shutil.rmtree(work, ignore_errors=True)
print(json.dumps(payload, default=str))
'''


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def cohort_from_asset_list() -> list[dict[str, Any]]:
    text = ASSET_LIST.read_text(encoding="utf-8-sig")
    seen: set[str] = set()
    rows: list[dict[str, Any]] = []
    for path_text, source_root, slug, selected_seed in ASSET_RE.findall(text):
        if slug in seen:
            continue
        seen.add(slug)
        if slug not in TEMPLATE_REGISTRY:
            raise RuntimeError(f"selected asset slug is absent from TEMPLATE_REGISTRY: {slug}")
        source_path = Path(path_text)
        template_path = TEMPLATE_ROOT / "agent" / "templates" / f"{slug}.py"
        rows.append({
            "slug": slug,
            "stem": TEMPLATE_REGISTRY[slug],
            "selected_asset_seed": int(selected_seed),
            "selected_asset_source": source_root,
            "selected_asset_path": str(source_path),
            "template_path": str(template_path),
            "template_sha256": sha256(template_path),
        })
    if len(rows) != 33:
        raise RuntimeError(f"expected 33 unique selected slugs, found {len(rows)}")
    return rows


def run_compile_probe(row: dict[str, Any], seed: int, out_root: Path, timeout: float) -> dict[str, Any]:
    publish = out_root / "compile_artifacts" / row["slug"] / f"seed_{seed}"
    work_parent = out_root / "compile_work"
    work_parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    try:
        proc = subprocess.run(
            [sys.executable, "-c", COMPILE_PROBE, row["slug"], row["stem"], str(seed),
             str(work_parent), str(publish)],
            cwd=TEMPLATE_ROOT,
            text=True,
            capture_output=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"verdict": "fail", "elapsed_s": time.monotonic() - started,
                "artifact_dir": None, "config": {}, "error": f"compile_timeout({timeout:.0f}s)"}
    if proc.returncode != 0:
        return {"verdict": "fail", "elapsed_s": time.monotonic() - started,
                "artifact_dir": None, "config": {},
                "error": f"compile_probe_crash(rc={proc.returncode}): {proc.stderr[-2000:]}"}
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as exc:
        return {"verdict": "fail", "elapsed_s": time.monotonic() - started,
                "artifact_dir": None, "config": {},
                "error": f"compile_probe_invalid_json: {exc}; stdout={proc.stdout[-1000:]!r}"}


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=180.0, help="Hard timeout per strict/probe compile")
    parser.add_argument("--slug", action="append", help="Run only selected slug(s); repeatable")
    parser.add_argument("--force", action="store_true", help="Re-run completed template records")
    args = parser.parse_args()

    # Geometry workers already provide process-level parallelism.  Letting each
    # NumPy/OpenBLAS import create dozens of inner threads can turn a valid seed
    # into a pthread_create crash, so freeze nested numerical parallelism.
    numerical_thread_env = {
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
        "VECLIB_MAXIMUM_THREADS": "1",
    }
    os.environ.update(numerical_thread_env)

    out_root = args.out.resolve()
    try:
        out_root.relative_to(EXP_ROOT.resolve())
    except ValueError as exc:
        raise SystemExit("--out must be inside /mnt/zsn/lyb/arti-skill/exp") from exc
    out_root.mkdir(parents=True, exist_ok=True)

    cohort = cohort_from_asset_list()
    selected = set(args.slug or [])
    if selected:
        unknown = selected - {row["slug"] for row in cohort}
        if unknown:
            raise SystemExit(f"--slug not in frozen cohort: {sorted(unknown)}")
        cohort = [row for row in cohort if row["slug"] in selected]

    manifest = {
        "schema_version": 1,
        "protocol": "nano3d_seed_distribution_reliability_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_asset_list": str(ASSET_LIST),
        "source_asset_list_sha256": sha256(ASSET_LIST),
        "template_repo": str(TEMPLATE_ROOT),
        "seeds": SEEDS,
        "strict_gate": "template_sweep._compile_one: run_checks=True,target=full,motion_qc=True + overlap/rigid/disconnected policies",
        "compile_gate": "compile_urdf_report: run_checks=False,target=full,motion_qc=False; only run after strict failure",
        "numerical_thread_env": numerical_thread_env,
        "templates": cohort,
    }
    dump_json(out_root / "manifest.json", manifest)

    all_records: list[dict[str, Any]] = []
    run_started = time.monotonic()
    for index, row in enumerate(cohort, 1):
        record_path = out_root / "templates" / row["slug"] / "outcomes.json"
        if record_path.is_file() and not args.force:
            payload = json.loads(record_path.read_text(encoding="utf-8"))
            if payload.get("seeds") == SEEDS and len(payload.get("outcomes", [])) == len(SEEDS):
                print(f"[{index}/{len(cohort)}] {row['slug']}: resume", flush=True)
                all_records.extend(payload["outcomes"])
                continue

        print(f"[{index}/{len(cohort)}] {row['slug']}: strict sweep 36 seeds", flush=True)
        strict_started = time.monotonic()
        outcomes = run_seed_outcomes(
            slug=row["slug"], stem=row["stem"], seeds=SEEDS, sdk_package="sdk",
            max_workers=max(1, args.workers), repo_root=TEMPLATE_ROOT,
            compile_timeout_s=args.timeout, motion_qc=True,
            artifact_root=out_root / "qc_artifacts" / row["slug"],
        )
        failed_seeds = [outcome.seed for outcome in outcomes if outcome.verdict != "pass"]
        probe_results: dict[int, dict[str, Any]] = {}
        if failed_seeds:
            print(f"  compile-only probes for {len(failed_seeds)} strict failures", flush=True)
            by_seed = {outcome.seed: outcome for outcome in outcomes}
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(max(1, args.workers), len(failed_seeds))
            ) as executor:
                futures = {
                    executor.submit(run_compile_probe, row, seed, out_root, args.timeout): seed
                    for seed in failed_seeds
                }
                for future in concurrent.futures.as_completed(futures):
                    seed = futures[future]
                    try:
                        probe_results[seed] = future.result()
                    except Exception as exc:  # defensive harness isolation
                        probe_results[seed] = {
                            "verdict": "fail", "elapsed_s": 0.0, "artifact_dir": None,
                            "config": by_seed[seed].config,
                            "error": f"probe_thread_error:{type(exc).__name__}: {exc}",
                        }

        records: list[dict[str, Any]] = []
        for outcome in outcomes:
            strict = outcome.to_dict()
            compile_result = {
                "verdict": "pass", "elapsed_s": strict["elapsed_s"],
                "artifact_dir": strict.get("artifact_dir"), "config": strict.get("config", {}),
                "error": None, "inferred_from_strict_pass": True,
            }
            if strict["verdict"] != "pass":
                compile_result = probe_results[outcome.seed]
                compile_result["inferred_from_strict_pass"] = False
            artifact_dir = strict.get("artifact_dir") or compile_result.get("artifact_dir")
            artifact_valid = compiled_artifact_is_valid(artifact_dir)
            records.append({
                "slug": row["slug"], "stem": row["stem"], "seed": outcome.seed,
                "config": strict.get("config") or compile_result.get("config") or {},
                "compile": compile_result, "qc": strict,
                "artifact_dir": artifact_dir, "artifact_valid": artifact_valid,
            })
        payload = {
            "slug": row["slug"], "stem": row["stem"], "seeds": SEEDS,
            "elapsed_wall_s": time.monotonic() - strict_started,
            "run_config": {
                "max_workers": max(1, args.workers),
                "per_seed_timeout_s": args.timeout,
                "numerical_thread_env": numerical_thread_env,
            },
            "compile_pass": sum(r["compile"]["verdict"] == "pass" for r in records),
            "qc_pass": sum(r["qc"]["verdict"] == "pass" for r in records),
            "artifact_saved": sum(r["artifact_valid"] for r in records),
            "outcomes": records,
        }
        dump_json(record_path, payload)
        all_records.extend(records)
        print(f"  compile={payload['compile_pass']}/36 qc={payload['qc_pass']}/36 artifacts={payload['artifact_saved']}/36", flush=True)

    template_rows = []
    for row in cohort:
        records = [item for item in all_records if item["slug"] == row["slug"]]
        template_rows.append({
            "slug": row["slug"], "stem": row["stem"], "seeds": len(records),
            "compile_pass": sum(item["compile"]["verdict"] == "pass" for item in records),
            "qc_pass": sum(item["qc"]["verdict"] == "pass" for item in records),
            "artifact_saved": sum(item["artifact_valid"] for item in records),
            "all_36_compile": len(records) == 36 and all(item["compile"]["verdict"] == "pass" for item in records),
            "all_36_qc": len(records) == 36 and all(item["qc"]["verdict"] == "pass" for item in records),
        })
    elapsed = [float(item["qc"].get("elapsed_s") or 0.0) for item in all_records]
    failures = Counter(
        item["qc"].get("failure_type_normalized") or item["qc"].get("failure_type") or "unknown"
        for item in all_records if item["qc"]["verdict"] != "pass"
    )
    summary = {
        "schema_version": 1,
        "protocol": manifest["protocol"],
        "status": "complete" if len(all_records) == len(cohort) * 36 else "partial",
        "template_count": len(cohort), "seed_count": len(all_records),
        "compile_pass": sum(item["compile"]["verdict"] == "pass" for item in all_records),
        "qc_pass": sum(item["qc"]["verdict"] == "pass" for item in all_records),
        "artifact_saved": sum(item["artifact_valid"] for item in all_records),
        "templates_36_of_36_compile": sum(row["all_36_compile"] for row in template_rows),
        "templates_36_of_36_qc": sum(row["all_36_qc"] for row in template_rows),
        "failure_types": dict(failures.most_common()),
        "strict_seed_elapsed_s": {
            "total": sum(elapsed), "mean": statistics.mean(elapsed) if elapsed else None,
            "median": statistics.median(elapsed) if elapsed else None,
            "p95": percentile(elapsed, 0.95),
        },
        "harness_wall_s": time.monotonic() - run_started,
        "template_records": template_rows,
        "unsupported": ["first_shot", "final_success_after_repair", "corner_pass",
                        "regression_retention", "repair_turns", "tokens", "api_cost"],
    }
    dump_json(out_root / "records.json", all_records)
    dump_json(out_root / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0 if summary["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
