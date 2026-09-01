#!/usr/bin/env python3
"""Deterministic existing-dataset hierarchy audit for Articraft.

This is not a common-prompt rerun. It freezes a result-independent sample from
the complete Articraft dataset manifest, materializes each sampled source in an
output-local snapshot, and applies the shared Nano3D URDF tree parser.
"""

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
import sys
import time
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_SKILL_ROOT = WORKSPACE / "arti-skill"
ARTICRAFT_ROOT = ARTI_SKILL_ROOT / "articraft_data"
DATASET_MANIFEST = ARTICRAFT_ROOT / "data/cache/manifests/dataset.json"
RECORDS_INDEX = ARTICRAFT_ROOT / "data/records_index.jsonl"
SHARED_EVALUATOR = ARTI_SKILL_ROOT / "exp/scripts/run_nano3d_hierarchy.py"
DEFAULT_OUTPUT = ARTI_SKILL_ROOT / "exp/runtime/nano3d_hierarchy_baselines/articraft"
PROTOCOL_ID = "articraft_hierarchy_existing_dataset_hash54_v1"
RANK_NAMESPACE = "articraft_hierarchy_v1"


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


def json_load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_shared_parser() -> tuple[Any, str]:
    spec = importlib.util.spec_from_file_location("nano3d_shared_hierarchy", SHARED_EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load shared evaluator: {SHARED_EVALUATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.parse_hierarchy, sha256(SHARED_EVALUATOR)


def rank_key(record_id: str) -> str:
    payload = f"{RANK_NAMESPACE}\0{record_id}".encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def inspect_source(record_id: str) -> dict[str, Any]:
    record_dir = contained(ARTICRAFT_ROOT / "data/records" / record_id)
    record_path = contained(record_dir / "record.json")
    row: dict[str, Any] = {
        "record_id": record_id,
        "record_dir": str(record_dir),
        "record_json": str(record_path),
        "source_eligible": False,
        "preflight_reasons": [],
    }
    if not record_path.is_file():
        row["preflight_reasons"].append("missing_record_json")
        return row
    try:
        record = json_load(record_path)
    except Exception as exc:  # noqa: BLE001
        prefix = record_path.read_text(encoding="utf-8", errors="replace")[:80]
        reason = (
            "git_lfs_pointer_record_json"
            if prefix.startswith("version https://git-lfs.github.com/spec/v1")
            else f"invalid_record_json:{type(exc).__name__}"
        )
        row["preflight_reasons"].append(reason)
        return row
    if not isinstance(record, dict):
        row["preflight_reasons"].append("record_json_not_object")
        return row

    revision_id = str(record.get("active_revision_id") or "")
    row["active_revision_id"] = revision_id or None
    if not revision_id:
        row["preflight_reasons"].append("missing_active_revision_id")
        return row
    revision_dir = contained(record_dir / "revisions" / revision_id)
    model_path = contained(revision_dir / "model.py")
    revision_path = contained(revision_dir / "revision.json")
    provenance_path = contained(revision_dir / "provenance.json")
    row.update(
        {
            "revision_dir": str(revision_dir),
            "model_py": str(model_path),
            "revision_json": str(revision_path),
            "provenance_json": str(provenance_path),
        }
    )
    for label, path in (
        ("model_py", model_path),
        ("revision_json", revision_path),
        ("provenance_json", provenance_path),
    ):
        if not path.is_file():
            row["preflight_reasons"].append(f"missing_{label}")
    if row["preflight_reasons"]:
        return row

    try:
        revision = json_load(revision_path)
        provenance = json_load(provenance_path)
    except Exception as exc:  # noqa: BLE001
        pointer_paths = [
            path.name
            for path in (revision_path, provenance_path)
            if path.read_text(encoding="utf-8", errors="replace").startswith(
                "version https://git-lfs.github.com/spec/v1"
            )
        ]
        reason = (
            "git_lfs_pointer_revision_or_provenance:" + ",".join(pointer_paths)
            if pointer_paths
            else f"invalid_revision_provenance_json:{type(exc).__name__}"
        )
        row["preflight_reasons"].append(reason)
        return row
    if not isinstance(revision, dict) or not isinstance(provenance, dict):
        row["preflight_reasons"].append("revision_or_provenance_not_object")
        return row

    current_model_hash = sha256(model_path)
    frozen_hash = str(record.get("hashes", {}).get("model_py_sha256") or "")
    row.update(
        {
            "model_py_sha256": current_model_hash,
            "record_declared_model_py_sha256": frozen_hash or None,
            "record_declared_hash_matches": bool(frozen_hash and frozen_hash == current_model_hash),
            "provider": record.get("provider"),
            "model_id": record.get("model_id"),
            "source_run_id": (record.get("source") or {}).get("run_id"),
            "run_status": (revision.get("run_summary") or {}).get("final_status"),
            "historical_external_compile_status": (
                (revision.get("external_quality") or {}).get("compile") or {}
            ).get("status"),
            "historical_external_validation_passed": (
                (revision.get("external_quality") or {}).get("validation") or {}
            ).get("passed"),
        }
    )
    row["source_eligible"] = True
    return row


def copy_record_snapshot(source: dict[str, Any], snapshot_root: Path) -> Path:
    record_id = str(source["record_id"])
    source_record_dir = Path(str(source["record_dir"]))
    target_record_dir = contained(snapshot_root / "data/records" / record_id)
    if target_record_dir.exists():
        shutil.rmtree(target_record_dir)
    target_record_dir.mkdir(parents=True)
    shutil.copy2(source_record_dir / "record.json", target_record_dir / "record.json")
    for optional in ("picture.json", "collections"):
        source_path = source_record_dir / optional
        target_path = target_record_dir / optional
        if source_path.is_dir():
            shutil.copytree(source_path, target_path)
        elif source_path.is_file():
            shutil.copy2(source_path, target_path)

    revision_id = str(source["active_revision_id"])
    source_revision = source_record_dir / "revisions" / revision_id
    target_revision = target_record_dir / "revisions" / revision_id
    target_revision.mkdir(parents=True)
    for name in ("model.py", "revision.json", "provenance.json", "prompt.txt", "prompt_series.json"):
        source_path = source_revision / name
        if source_path.is_file():
            shutil.copy2(source_path, target_revision / name)
    inputs = source_revision / "inputs"
    if inputs.is_dir():
        shutil.copytree(inputs, target_revision / "inputs")
    return target_record_dir


def compile_one(
    source: dict[str, Any],
    *,
    snapshot_root: Path,
    articraft_cli: Path,
    timeout_seconds: int,
) -> dict[str, Any]:
    record_id = str(source["record_id"])
    result: dict[str, Any] = {
        "record_id": record_id,
        "rank_sha256": source["rank_sha256"],
        "source_eligible": bool(source["source_eligible"]),
        "compile_returncode": None,
        "compile_status": "not_run",
        "compile_elapsed_seconds": None,
        "final_urdf_present": False,
        "parseable_final_urdf": False,
        "error": None,
    }
    if not source["source_eligible"]:
        result["compile_status"] = "preflight_failed"
        result["error"] = ";".join(source["preflight_reasons"])
        return result

    copy_record_snapshot(source, snapshot_root)
    log_dir = contained(snapshot_root.parent / "logs")
    log_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(articraft_cli),
        "compile",
        record_id,
        "--repo-root",
        str(snapshot_root),
        "--target",
        "full",
        "--validate",
        "--strict-geom-qc",
    ]
    started = time.perf_counter()
    package_dir = contained(snapshot_root / "data/cache/record_materialization" / record_id)
    if package_dir.exists():
        shutil.rmtree(package_dir)
    env = {
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
        "PATH": str(ARTICRAFT_ROOT / ".venv/bin") + ":/usr/local/bin:/usr/bin:/bin",
    }
    try:
        completed = subprocess.run(
            command,
            cwd=ARTICRAFT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
        result["compile_returncode"] = completed.returncode
        (log_dir / f"{record_id}.stdout.txt").write_text(completed.stdout, encoding="utf-8")
        (log_dir / f"{record_id}.stderr.txt").write_text(completed.stderr, encoding="utf-8")
    except subprocess.TimeoutExpired as exc:
        result["compile_status"] = "timeout"
        result["error"] = f"timeout_after_{timeout_seconds}s"
        (log_dir / f"{record_id}.stdout.txt").write_text(exc.stdout or "", encoding="utf-8")
        (log_dir / f"{record_id}.stderr.txt").write_text(exc.stderr or "", encoding="utf-8")
        result["compile_elapsed_seconds"] = time.perf_counter() - started
        return result
    result["compile_elapsed_seconds"] = time.perf_counter() - started

    urdf_path = contained(package_dir / "model.urdf")
    report_path = contained(package_dir / "compile_report.json")
    result["package_dir"] = str(package_dir)
    result["model_urdf"] = str(urdf_path)
    result["compile_report_json"] = str(report_path)
    report: dict[str, Any] = {}
    if report_path.is_file():
        try:
            payload = json_load(report_path)
            report = payload if isinstance(payload, dict) else {}
        except Exception as exc:  # noqa: BLE001
            result["error"] = f"invalid_compile_report:{type(exc).__name__}"
    result["reported_compile_status"] = report.get("status")
    result["reported_validation_passed"] = (report.get("validation") or {}).get("passed")
    signals = (report.get("signal_bundle") or {}).get("signals") or []
    result["reported_blocking_signal_codes"] = [
        str(signal.get("code"))
        for signal in signals
        if isinstance(signal, dict) and signal.get("blocking")
    ]
    result["compile_status"] = (
        "success" if completed.returncode == 0 and report.get("status") == "success" else "failed"
    )
    result["final_urdf_present"] = urdf_path.is_file()
    if urdf_path.is_file():
        result["model_urdf_sha256"] = sha256(urdf_path)
    if result["compile_status"] != "success" and result["error"] is None:
        result["error"] = f"returncode={completed.returncode};reported_status={report.get('status')}"
    return result


def markdown_report(summary: dict[str, Any]) -> str:
    universe = summary["universe"]
    selected = summary["selected_cohort"]
    metrics = summary["hierarchy_metrics"]
    failures = summary["failure_counts"]
    lines = [
        "# Articraft hierarchy existing-dataset audit",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is an existing-dataset/package audit. It is not a common-prompt rerun and does not establish hierarchy correctness.",
        "",
        "## Frozen cohort",
        "",
        f"- Protocol: `{summary['protocol']}`",
        f"- Evaluation unit: {summary['evaluation_unit']}",
        f"- Dataset manifest records: {universe['manifest_record_count']}",
        f"- Source-eligible records: {universe['source_eligible_count']}/{universe['manifest_record_count']}",
        f"- Selected records: {selected['selected_count']} (no replacement after failures)",
        f"- Selection: {selected['selection_rule']}",
        f"- Dataset manifest SHA-256: `{summary['provenance']['dataset_manifest_sha256']}`",
        f"- Shared evaluator SHA-256: `{summary['provenance']['shared_evaluator_sha256']}`",
        "",
        "## Materialization",
        "",
        f"- Strict compile success: {selected['compile_success_count']}/{selected['selected_count']}",
        f"- Final URDF present: {selected['final_urdf_count']}/{selected['selected_count']}",
        f"- Parseable final URDF: {selected['parseable_final_urdf_count']}/{selected['selected_count']}",
        f"- Failure counts: `{json.dumps(failures, sort_keys=True)}`",
        "",
        "## Shared-tree hierarchy metrics",
        "",
        f"- Valid Tree: {metrics['valid_tree_count']}/{selected['selected_count']} end-to-end; {metrics['valid_tree_count']}/{selected['parseable_final_urdf_count']} conditional on parseable final URDF",
        f"- Has Hierarchy: {metrics['has_tree_count']}/{selected['selected_count']} end-to-end; {metrics['has_tree_count']}/{selected['parseable_final_urdf_count']} conditional",
        f"- Semantic Depth: {metrics['semantic_depth_mean']:.3f} mean [{metrics['semantic_depth_min']}, {metrics['semantic_depth_max']}] over parseable final URDFs",
        f"- Named Groups: {metrics['named_groups_mean']:.3f} mean; {metrics['named_groups_total']} total",
        f"- Pivots: {metrics['pivots_mean']:.3f} mean; {metrics['pivots_total']} total",
        "- Parent-Child Edge F1: N/A (no independent hierarchy gold)",
        "- Hierarchy Exact Match: N/A (no independent hierarchy gold)",
        "- Semantic Nesting Accuracy: N/A (paper protocol does not score correctness)",
        "- Cross-Seed Consistency: N/A (per-asset method; no seed interface)",
        "",
        "## Provenance and limitations",
        "",
        "The sample is selected solely from record IDs before compilation; compile or hierarchy outcomes never affect membership. Current sources are snapshotted into this output, and final packages are materialized there. Historical compile flags are provenance only and are not counted as current packages.",
        "",
        "The repository's source materialization cache is not inspected because its canonical symlink target is outside the authorized workspace. Most ineligible records are Git LFS pointer stubs in this checkout, not observed hierarchy failures.",
        "",
        "Depth, named groups, and pivots are structural descriptors, not quality scores. The evaluator maps named URDF links to scene nodes, counts a root link at depth 1, treats visual-free internal links as named groups, and counts non-fixed joints as pivots. This audit does not share prompts or assets with Nova3D/PV-A and cannot support a direct quality ranking.",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sample-size", type=int, default=54)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)
    snapshot_root = contained(output / "snapshot_repo")
    snapshot_root.mkdir(parents=True, exist_ok=True)
    articraft_cli = contained(ARTICRAFT_ROOT / ".venv/bin/articraft")
    if not articraft_cli.is_file():
        raise FileNotFoundError(f"Articraft CLI not found: {articraft_cli}")

    manifest = json_load(DATASET_MANIFEST)
    generated = manifest.get("generated") if isinstance(manifest, dict) else None
    if not isinstance(generated, list):
        raise ValueError("dataset manifest lacks generated list")
    record_ids = [str(row["record_id"]) for row in generated]
    if len(record_ids) != len(set(record_ids)):
        raise ValueError("duplicate record IDs in dataset manifest")
    if args.sample_size <= 0 or args.sample_size > len(record_ids):
        raise ValueError("sample size must be within dataset manifest size")

    universe_rows = [inspect_source(record_id) for record_id in record_ids]
    universe_by_id = {str(row["record_id"]): row for row in universe_rows}
    ranked = sorted(record_ids, key=lambda record_id: (rank_key(record_id), record_id))
    selected_ids = ranked[: args.sample_size]
    selection_rows: list[dict[str, Any]] = []
    for rank, record_id in enumerate(selected_ids, start=1):
        row = dict(universe_by_id[record_id])
        row["rank"] = rank
        row["rank_sha256"] = rank_key(record_id)
        selection_rows.append(row)

    selection_path = contained(output / "selection_manifest.jsonl")
    selection_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in selection_rows),
        encoding="utf-8",
    )
    reason_counts = Counter(
        reason.split(":", 1)[0]
        for row in universe_rows
        for reason in row["preflight_reasons"]
    )
    preflight = {
        "protocol": PROTOCOL_ID,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "manifest_record_count": len(record_ids),
        "source_eligible_count": sum(bool(row["source_eligible"]) for row in universe_rows),
        "source_ineligible_count": sum(not bool(row["source_eligible"]) for row in universe_rows),
        "source_ineligible_reason_counts": dict(sorted(reason_counts.items())),
        "source_materialization_cache": "NOT_INSPECTED_WORKSPACE_BOUNDARY_SYMLINK",
        "declared_model_hash_match_count": sum(
            bool(row.get("record_declared_hash_matches")) for row in universe_rows
        ),
        "declared_model_hash_mismatch_or_missing_count": sum(
            row["source_eligible"] and not row.get("record_declared_hash_matches")
            for row in universe_rows
        ),
        "selected_count": len(selection_rows),
        "selected_source_eligible_count": sum(bool(row["source_eligible"]) for row in selection_rows),
        "selection_rule": (
            f"sort all manifest record_ids by SHA256({RANK_NAMESPACE!r} + NUL + record_id), "
            f"tie-break by record_id, take first {args.sample_size}; no replacement"
        ),
        "dataset_manifest_sha256": sha256(DATASET_MANIFEST),
        "records_index_sha256": sha256(RECORDS_INDEX),
        "selection_manifest_sha256": sha256(selection_path),
    }
    (output / "preflight.json").write_text(
        json.dumps(preflight, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.preflight_only:
        print(json.dumps(preflight, indent=2, sort_keys=True))
        return 0

    parse_hierarchy, shared_evaluator_hash = load_shared_parser()
    records: list[dict[str, Any]] = []
    for index, source in enumerate(selection_rows, start=1):
        print(f"[{index}/{len(selection_rows)}] {source['record_id']}", flush=True)
        result = compile_one(
            source,
            snapshot_root=snapshot_root,
            articraft_cli=articraft_cli,
            timeout_seconds=args.timeout_seconds,
        )
        if result["final_urdf_present"]:
            try:
                hierarchy = parse_hierarchy(Path(str(result["model_urdf"])))
                result.update(hierarchy)
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
    valid = [row for row in parseable if row.get("valid_tree")]
    failure_counts = Counter(
        row["compile_status"] if not row["parseable_final_urdf"] else "parseable"
        for row in records
    )
    status = "COMPLETE" if len(parseable) == len(records) else (
        "COMPLETE_WITH_FAILURES" if parseable else "BLOCKED"
    )
    metrics = {
        "metric_denominator": "parseable materialized final URDF; end-to-end rates also use selected N",
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
            "cross_seed_consistency": "N/A: per-asset method without a seed interface",
        },
    }
    summary = {
        "status": status,
        "protocol": PROTOCOL_ID,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "evaluation_unit": "one frozen Articraft dataset record -> freshly materialized final URDF package",
        "claim_boundary": "existing-dataset/package audit; not a common-prompt rerun",
        "universe": preflight,
        "selected_cohort": {
            "selected_count": len(records),
            "selection_rule": preflight["selection_rule"],
            "selection_manifest_sha256": preflight["selection_manifest_sha256"],
            "source_eligible_count": sum(bool(row["source_eligible"]) for row in records),
            "compile_success_count": sum(row["compile_status"] == "success" for row in records),
            "final_urdf_count": sum(bool(row["final_urdf_present"]) for row in records),
            "parseable_final_urdf_count": len(parseable),
        },
        "failure_counts": dict(sorted(failure_counts.items())),
        "hierarchy_metrics": metrics,
        "provenance": {
            "articraft_dataset_manifest": str(DATASET_MANIFEST),
            "dataset_manifest_sha256": preflight["dataset_manifest_sha256"],
            "records_index": str(RECORDS_INDEX),
            "records_index_sha256": preflight["records_index_sha256"],
            "shared_evaluator": str(SHARED_EVALUATOR),
            "shared_evaluator_sha256": shared_evaluator_hash,
            "runner_sha256": sha256(Path(__file__)),
            "articraft_cli": str(articraft_cli),
            "compile_contract": "target=full, validate=true, strict_geom_qc=true, timeout per item",
            "timeout_seconds": args.timeout_seconds,
            "snapshot_repo": str(snapshot_root),
        },
        "artifacts": {
            "selection_manifest": str(selection_path),
            "preflight": str(output / "preflight.json"),
            "records": str(records_path),
            "report": str(output / "report.md"),
        },
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(markdown_report(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if parseable else 2


if __name__ == "__main__":
    raise SystemExit(main())
