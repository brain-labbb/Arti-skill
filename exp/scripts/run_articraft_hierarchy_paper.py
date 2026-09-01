#!/usr/bin/env python3
"""Evaluate a frozen Articraft-10K cohort with the shared Nano3D tree parser."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
from statistics import mean
import time
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_SKILL = WORKSPACE / "arti-skill"
CURRENT_REPO = ARTI_SKILL / "exp/baselines/Articraft-official"
PAPER_HARNESS = ARTI_SKILL / "exp/baselines/Articraft-paper-harness"
DATASET = ARTI_SKILL / "exp/baselines/Articraft-10K-official"
DATASET_MANIFEST = DATASET / "records_manifest.jsonl"
SHARED_EVALUATOR = ARTI_SKILL / "exp/scripts/run_nano3d_hierarchy.py"
DEFAULT_OUTPUT = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_paper/articraft"
PROTOCOL = "articraft_10k_hierarchy_equal_category_hash6_v1"
RANK_NAMESPACE = "nano3d_articraft_paper_hierarchy_v1"

CATEGORY_MAP = {
    "storage_furniture_cabinet": "hingeddoor_cabinet",
    "table": "overbed_table",
    "refrigerator": "refrigerator_with_hinged_doors",
    "dishwasher": "dishwasher_with_dropdown_door_and_sliding_racks",
    "oven": "conventional_oven_with_dropdown_door",
}


def contained(path: Path) -> Path:
    resolved = path.resolve(strict=False)
    if not resolved.is_relative_to(WORKSPACE):
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def git_commit(repo: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    return completed.stdout.strip()


def load_shared_parser() -> tuple[Any, str]:
    spec = importlib.util.spec_from_file_location("nano3d_shared_hierarchy", SHARED_EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load shared evaluator: {SHARED_EVALUATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.parse_hierarchy, sha256(SHARED_EVALUATOR)


def rank_key(cohort: str, record_id: str) -> str:
    payload = f"{RANK_NAMESPACE}\0{cohort}\0{record_id}".encode()
    return hashlib.sha256(payload).hexdigest()


def source_paths(row: dict[str, Any]) -> dict[str, Path]:
    record_id = str(row["record_id"])
    revision_id = str(row.get("active_revision_id") or "")
    record_dir = contained(DATASET / "records" / record_id)
    revision_dir = contained(record_dir / "revisions" / revision_id)
    return {
        "record_dir": record_dir,
        "record_json": contained(record_dir / "record.json"),
        "revision_dir": revision_dir,
        "model_py": contained(revision_dir / "model.py"),
        "revision_json": contained(revision_dir / "revision.json"),
        "provenance_json": contained(revision_dir / "provenance.json"),
        "prompt_txt": contained(revision_dir / "prompt.txt"),
        "inputs_dir": contained(revision_dir / "inputs"),
    }


def inspect_source(row: dict[str, Any]) -> dict[str, Any]:
    paths = source_paths(row)
    required = ("record_json", "model_py", "revision_json", "provenance_json", "prompt_txt")
    missing = [name for name in required if not paths[name].is_file()]
    result: dict[str, Any] = {
        "record_id": row["record_id"],
        "active_revision_id": row.get("active_revision_id"),
        "source_complete": not missing,
        "source_missing": missing,
        "source_paths": {name: str(path) for name, path in paths.items()},
    }
    if missing:
        return result
    hashes = {name + "_sha256": sha256(paths[name]) for name in required}
    try:
        record = load_json(paths["record_json"])
        revision = load_json(paths["revision_json"])
        provenance = load_json(paths["provenance_json"])
    except Exception as exc:  # noqa: BLE001
        result["source_complete"] = False
        result["source_missing"] = [f"invalid_json:{type(exc).__name__}"]
        return result
    result.update(hashes)
    result.update(
        {
            "record_declared_model_sha256": (record.get("hashes") or {}).get("model_py_sha256"),
            "record_declared_model_hash_matches": (
                (record.get("hashes") or {}).get("model_py_sha256") == hashes["model_py_sha256"]
            ),
            "revision_record_id": revision.get("record_id"),
            "provenance_record_id": provenance.get("record_id"),
            "provenance_git_commit": (provenance.get("environment") or {}).get("git_commit"),
            "provenance_provider": (provenance.get("generation") or {}).get("provider"),
            "provenance_model_id": (provenance.get("generation") or {}).get("model_id"),
        }
    )
    return result


def copy_selected_record(source: dict[str, Any], data_root: Path) -> None:
    record_id = str(source["record_id"])
    revision_id = str(source["active_revision_id"])
    source_record = Path(source["source_paths"]["record_dir"])
    source_revision = Path(source["source_paths"]["revision_dir"])
    target_record = contained(data_root / "records" / record_id)
    if target_record.exists():
        shutil.rmtree(target_record)
    target_revision = target_record / "revisions" / revision_id
    target_revision.mkdir(parents=True)
    shutil.copy2(source_record / "record.json", target_record / "record.json")
    for optional in ("picture.json", "collections"):
        source_path = source_record / optional
        target_path = target_record / optional
        if source_path.is_dir():
            shutil.copytree(source_path, target_path)
        elif source_path.is_file():
            shutil.copy2(source_path, target_path)
    for name in (
        "model.py",
        "revision.json",
        "provenance.json",
        "prompt.txt",
        "prompt_series.json",
        "cost.json",
    ):
        source_path = source_revision / name
        if source_path.is_file():
            shutil.copy2(source_path, target_revision / name)
    inputs = source_revision / "inputs"
    if inputs.is_dir():
        shutil.copytree(inputs, target_revision / "inputs")


def compile_record(
    selection: dict[str, Any],
    *,
    data_root: Path,
    output: Path,
    timeout_seconds: int,
) -> dict[str, Any]:
    record_id = str(selection["record_id"])
    result: dict[str, Any] = {
        "cohort": selection["cohort"],
        "category_slug": selection["category_slug"],
        "record_id": record_id,
        "rating": selection["rating"],
        "rank": selection["rank"],
        "rank_sha256": selection["rank_sha256"],
        "source_complete": selection["source_complete"],
        "compile_status": "not_run",
        "compile_returncode": None,
        "compile_elapsed_seconds": None,
        "final_urdf_present": False,
        "parseable_final_urdf": False,
        "error": None,
    }
    if not selection["source_complete"]:
        result["compile_status"] = "preflight_failed"
        result["error"] = ",".join(selection["source_missing"])
        return result

    copy_selected_record(selection, data_root)
    package = contained(data_root / "cache/record_materialization" / record_id)
    if package.exists():
        shutil.rmtree(package)
    logs = contained(output / "logs")
    logs.mkdir(parents=True, exist_ok=True)
    cli = contained(PAPER_HARNESS / ".venv/bin/articraft")
    command = [
        str(cli),
        "compile",
        "--data-dir",
        str(data_root),
        record_id,
        "--target",
        "full",
        "--validate",
        "--strict-geom-qc",
    ]
    env = {
        "PATH": str(PAPER_HARNESS / ".venv/bin") + ":/usr/local/bin:/usr/bin:/bin",
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
    }
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            command,
            cwd=PAPER_HARNESS,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        result["compile_status"] = "timeout"
        result["compile_elapsed_seconds"] = time.perf_counter() - started
        result["error"] = f"timeout_after_{timeout_seconds}s"
        (logs / f"{record_id}.stdout.txt").write_text(exc.stdout or "", encoding="utf-8")
        (logs / f"{record_id}.stderr.txt").write_text(exc.stderr or "", encoding="utf-8")
        return result
    result["compile_elapsed_seconds"] = time.perf_counter() - started
    result["compile_returncode"] = completed.returncode
    (logs / f"{record_id}.stdout.txt").write_text(completed.stdout, encoding="utf-8")
    (logs / f"{record_id}.stderr.txt").write_text(completed.stderr, encoding="utf-8")

    report_path = contained(package / "compile_report.json")
    urdf_path = contained(package / "model.urdf")
    report: dict[str, Any] = {}
    if report_path.is_file():
        try:
            payload = load_json(report_path)
            report = payload if isinstance(payload, dict) else {}
        except Exception as exc:  # noqa: BLE001
            result["error"] = f"invalid_compile_report:{type(exc).__name__}"
    signals = (report.get("signal_bundle") or {}).get("signals") or []
    result.update(
        {
            "package_dir": str(package),
            "model_urdf": str(urdf_path),
            "compile_report_json": str(report_path),
            "reported_compile_status": report.get("status"),
            "reported_blocking_signal_codes": [
                str(signal.get("code"))
                for signal in signals
                if isinstance(signal, dict) and signal.get("blocking")
            ],
            "compile_status": (
                "success"
                if completed.returncode == 0 and report.get("status") == "success"
                else "failed"
            ),
            "final_urdf_present": urdf_path.is_file(),
        }
    )
    if urdf_path.is_file():
        result["model_urdf_sha256"] = sha256(urdf_path)
    if result["compile_status"] != "success" and result["error"] is None:
        result["error"] = (
            f"returncode={completed.returncode};reported_status={report.get('status')}"
        )
    return result


def build_report(summary: dict[str, Any]) -> str:
    selected = summary["selected_cohort"]
    metrics = summary["hierarchy_metrics"]
    lines = [
        "# Articraft-10K paper-ready hierarchy audit",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is a deterministic re-evaluation of the officially released Articraft-10K source records. It is not a new common-prompt generation rerun and does not measure semantic hierarchy correctness.",
        "",
        "## Official provenance",
        "",
        f"- Current supported code: `articraftresearch/Articraft@{summary['provenance']['current_repo_commit']}` (Apache-2.0)",
        f"- Paper harness: `mattzh72/articraft@{summary['provenance']['paper_harness_commit']}` (Apache-2.0)",
        f"- Released data: `articraftresearch/Articraft-10K@{summary['provenance']['dataset_commit']}` (CC BY 4.0)",
        f"- Released manifest: {summary['inventory']['released_manifest_count']} records; {summary['inventory']['category_directory_count']} category folders",
        "",
        "## Frozen cohort",
        "",
        f"- Protocol: `{summary['protocol']}`",
        f"- Evaluation unit: {summary['evaluation_unit']}",
        f"- Selection: {selected['selection_rule']}",
        f"- Requested denominator: {selected['selected_count']} records ({selected['per_category_n']} per category)",
        "",
        "| Cohort | Official category | Released N | Paper-retained N (rating 4-5) | Selected N | Parseable N |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for row in summary["category_audit"]:
        result = summary["per_category_results"][row["cohort"]]
        lines.append(
            f"| {row['cohort']} | `{row['category_slug']}` | {row['released_count']} | {row['paper_retained_count']} | {result['selected_count']} | {result['parseable_count']} |"
        )
    lines.extend(
        [
            "",
            "## Materialization",
            "",
            f"- Compile success: {selected['compile_success_count']}/{selected['selected_count']}",
            f"- Final URDF present: {selected['final_urdf_count']}/{selected['selected_count']}",
            f"- Parseable final URDF: {selected['parseable_count']}/{selected['selected_count']}",
            f"- Failures: `{json.dumps(summary['failure_counts'], sort_keys=True)}`",
            "",
            "## Shared-tree metrics",
            "",
            f"- Valid Tree: {metrics['valid_tree_count']}/{selected['selected_count']} end-to-end; {metrics['valid_tree_count']}/{selected['parseable_count']} conditional",
            f"- Has Hierarchy: {metrics['has_tree_count']}/{selected['selected_count']} end-to-end; {metrics['has_tree_count']}/{selected['parseable_count']} conditional",
            f"- Semantic Depth: {metrics['semantic_depth_mean']:.3f} mean [{metrics['semantic_depth_min']}, {metrics['semantic_depth_max']}]",
            f"- Named Groups: {metrics['named_groups_mean']:.3f} mean; {metrics['named_groups_total']} total",
            f"- Pivots: {metrics['pivots_mean']:.3f} mean; {metrics['pivots_total']} total",
            "- Parent-Child Edge F1: N/A (no independent hierarchy gold)",
            "- Hierarchy Exact Match: N/A (no independent hierarchy gold)",
            "- Semantic Nesting Accuracy: N/A (paper protocol does not score correctness)",
            "- Cross-Seed Consistency: N/A (per-asset release records)",
            "",
            "The release's pre-existing ratings define the paper-retained set before this experiment; compile and hierarchy outcomes never affect membership. Selection is hash-ranked within each mapped category, and failed records are not replaced. Depth, group count, and pivots are descriptors rather than quality scores.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--per-category", type=int, default=6)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)
    data_root = contained(output / "selected_data")
    data_root.mkdir(parents=True, exist_ok=True)

    manifest_rows = [
        json.loads(line)
        for line in DATASET_MANIFEST.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(manifest_rows) != 10_788:
        raise ValueError(f"expected released manifest N=10788, found {len(manifest_rows)}")
    if len({str(row["record_id"]) for row in manifest_rows}) != len(manifest_rows):
        raise ValueError("duplicate record_id in released manifest")

    category_audit: list[dict[str, Any]] = []
    selections: list[dict[str, Any]] = []
    for cohort, category_slug in CATEGORY_MAP.items():
        released = [row for row in manifest_rows if row.get("category_slug") == category_slug]
        retained = [row for row in released if row.get("rating") in (4, 5)]
        if len(retained) < args.per_category:
            raise ValueError(f"{cohort} has only {len(retained)} retained candidates")
        inspected = [(row, inspect_source(row)) for row in retained]
        complete = sum(bool(source["source_complete"]) for _, source in inspected)
        category_audit.append(
            {
                "cohort": cohort,
                "category_slug": category_slug,
                "released_count": len(released),
                "paper_retained_count": len(retained),
                "excluded_below_rating_4_count": len(released) - len(retained),
                "retained_source_complete_count": complete,
                "retained_source_incomplete_count": len(retained) - complete,
                "released_run_statuses": sorted({str(row.get("run_status")) for row in released}),
                "released_sdk_packages": sorted({str(row.get("sdk_package")) for row in released}),
            }
        )
        ranked = sorted(
            retained,
            key=lambda row: (rank_key(cohort, str(row["record_id"])), str(row["record_id"])),
        )
        source_by_id = {str(row["record_id"]): source for row, source in inspected}
        for rank, row in enumerate(ranked[: args.per_category], start=1):
            record_id = str(row["record_id"])
            selections.append(
                {
                    "cohort": cohort,
                    "category_slug": category_slug,
                    "rank": rank,
                    "rank_sha256": rank_key(cohort, record_id),
                    "record_id": record_id,
                    "rating": row.get("rating"),
                    "provider": row.get("provider"),
                    "model_id": row.get("model_id"),
                    "run_status": row.get("run_status"),
                    **source_by_id[record_id],
                }
            )

    selection_path = contained(output / "selection_manifest.jsonl")
    selection_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in selections),
        encoding="utf-8",
    )
    inventory = {
        "released_manifest_count": len(manifest_rows),
        "category_directory_count": sum(
            1 for path in (DATASET / "categories").iterdir() if path.is_dir()
        ),
        "records_manifest_sha256": sha256(DATASET_MANIFEST),
        "category_audit": category_audit,
        "selection_manifest_sha256": sha256(selection_path),
        "selection_rule": (
            f"within each frozen category, retain published rating 4-5 objects, sort by "
            f"SHA256({RANK_NAMESPACE!r} + NUL + cohort + NUL + record_id), tie-break by "
            f"record_id, take first {args.per_category}; no replacement after failure"
        ),
    }
    (output / "inventory.json").write_text(
        json.dumps(inventory, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.preflight_only:
        print(json.dumps(inventory, indent=2, sort_keys=True))
        return 0

    parse_hierarchy, shared_hash = load_shared_parser()
    records: list[dict[str, Any]] = []
    for index, selection in enumerate(selections, start=1):
        print(
            f"[{index}/{len(selections)}] {selection['cohort']} {selection['record_id']}",
            flush=True,
        )
        result = compile_record(
            selection,
            data_root=data_root,
            output=output,
            timeout_seconds=args.timeout_seconds,
        )
        if result["final_urdf_present"]:
            try:
                result.update(parse_hierarchy(Path(str(result["model_urdf"]))))
                result["parseable_final_urdf"] = True
            except Exception as exc:  # noqa: BLE001
                result["error"] = f"hierarchy_parse:{type(exc).__name__}:{exc}"
        records.append(result)

    records_path = contained(output / "records.jsonl")
    records_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in records),
        encoding="utf-8",
    )
    parseable = [row for row in records if row["parseable_final_urdf"]]
    failure_labels: list[str] = []
    for row in records:
        if row["compile_status"] != "success":
            failure_labels.append(str(row["compile_status"]))
        elif not row["final_urdf_present"]:
            failure_labels.append("missing_final_urdf")
        elif not row["parseable_final_urdf"]:
            failure_labels.append("hierarchy_parse_failure")
    failure_counts = Counter(failure_labels)
    per_category_results: dict[str, dict[str, int]] = {}
    for cohort in CATEGORY_MAP:
        cohort_rows = [row for row in records if row["cohort"] == cohort]
        per_category_results[cohort] = {
            "selected_count": len(cohort_rows),
            "compile_success_count": sum(row["compile_status"] == "success" for row in cohort_rows),
            "parseable_count": sum(bool(row["parseable_final_urdf"]) for row in cohort_rows),
            "valid_tree_count": sum(bool(row.get("valid_tree")) for row in cohort_rows),
            "has_tree_count": sum(bool(row.get("has_tree")) for row in cohort_rows),
        }
    metrics = {
        "valid_tree_count": sum(bool(row.get("valid_tree")) for row in parseable),
        "has_tree_count": sum(bool(row.get("has_tree")) for row in parseable),
        "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in parseable) if parseable else None,
        "semantic_depth_min": min(int(row["semantic_depth"]) for row in parseable) if parseable else None,
        "semantic_depth_max": max(int(row["semantic_depth"]) for row in parseable) if parseable else None,
        "named_groups_mean": mean(int(row["named_group_count"]) for row in parseable) if parseable else None,
        "named_groups_total": sum(int(row["named_group_count"]) for row in parseable),
        "pivots_mean": mean(int(row["pivot_count"]) for row in parseable) if parseable else None,
        "pivots_total": sum(int(row["pivot_count"]) for row in parseable),
        "unsupported": {
            "parent_child_edge_f1": "N/A: no independent hierarchy gold",
            "hierarchy_exact_match": "N/A: no independent hierarchy gold",
            "semantic_nesting_accuracy": "N/A: paper protocol does not score correctness",
            "cross_seed_consistency": "N/A: per-asset release records",
        },
    }
    status = "COMPLETE" if len(parseable) == len(records) else (
        "COMPLETE_WITH_FAILURES" if parseable else "BLOCKED"
    )
    summary = {
        "status": status,
        "protocol": PROTOCOL,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "evaluation_unit": "one officially released, paper-retained Articraft record -> freshly materialized URDF package",
        "claim_boundary": "official Articraft-10K deterministic audit; not a new common-prompt generation rerun",
        "inventory": inventory,
        "category_audit": category_audit,
        "selected_cohort": {
            "per_category_n": args.per_category,
            "selected_count": len(records),
            "selection_rule": inventory["selection_rule"],
            "selection_manifest_sha256": inventory["selection_manifest_sha256"],
            "source_complete_count": sum(bool(row["source_complete"]) for row in records),
            "compile_success_count": sum(row["compile_status"] == "success" for row in records),
            "final_urdf_count": sum(bool(row["final_urdf_present"]) for row in records),
            "parseable_count": len(parseable),
        },
        "failure_counts": dict(sorted(failure_counts.items())),
        "per_category_results": per_category_results,
        "hierarchy_metrics": metrics,
        "provenance": {
            "current_repo_url": "https://github.com/articraftresearch/Articraft",
            "current_repo_commit": git_commit(CURRENT_REPO),
            "current_repo_license": "Apache-2.0",
            "current_repo_license_sha256": sha256(CURRENT_REPO / "LICENSE"),
            "paper_url": "https://arxiv.org/abs/2605.15187",
            "paper_harness_url": "https://github.com/mattzh72/articraft",
            "paper_harness_commit": git_commit(PAPER_HARNESS),
            "paper_harness_license": "Apache-2.0",
            "paper_harness_license_sha256": sha256(PAPER_HARNESS / "LICENSE"),
            "paper_harness_uv_lock_sha256": sha256(PAPER_HARNESS / "uv.lock"),
            "dataset_url": "https://github.com/articraftresearch/Articraft-10K",
            "dataset_commit": git_commit(DATASET),
            "dataset_license": "CC-BY-4.0",
            "dataset_license_sha256": sha256(DATASET / "LICENSE"),
            "dataset_manifest_sha256": sha256(DATASET_MANIFEST),
            "shared_evaluator": str(SHARED_EVALUATOR),
            "shared_evaluator_sha256": shared_hash,
            "runner_sha256": sha256(Path(__file__)),
            "compile_contract": "paper harness; target=full; validate=true; strict_geom_qc=true",
            "timeout_seconds": args.timeout_seconds,
        },
        "artifacts": {
            "inventory": str(output / "inventory.json"),
            "selection_manifest": str(selection_path),
            "records": str(records_path),
            "report": str(output / "report.md"),
            "selected_data": str(data_root),
        },
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if parseable:
        (output / "report.md").write_text(build_report(summary), encoding="utf-8")
    else:
        (output / "report.md").write_text(
            "# Articraft-10K paper-ready hierarchy audit\n\nStatus: **BLOCKED**\n\n"
            "No selected record produced a parseable final URDF. See `summary.json` and `records.jsonl`.\n",
            encoding="utf-8",
        )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if parseable else 2


if __name__ == "__main__":
    raise SystemExit(main())
