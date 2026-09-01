#!/usr/bin/env python3
"""Fail-closed verification and isolated replay for PV-A expanded N=150."""

from __future__ import annotations

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/pva"
DEFAULT_REPLAY = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/pva_replay"
RUNNER = Path(__file__).with_name("run_pva_hierarchy_expanded_n150.py")
TREE = Path(__file__).with_name("run_nano3d_hierarchy.py")
EXTENDED = Path(__file__).with_name("hierarchy_extended_metrics.py")
PARTNET = Path(__file__).with_name("partnet_hierarchy_correctness.py")
ONTOLOGY = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
CATEGORIES = {
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
}
PER_CATEGORY = 30
REQUESTED = 150
EXPECTED_FAILURES = 25
TIME_KEYS = {
    "created_at_utc",
    "generated_at",
    "started_at_utc",
    "finished_at_utc",
    "updated_at_utc",
    "verified_at_utc",
}
DERIVED_HASH_KEYS = {
    "records_sha256",
    "selection_manifest_sha256",
    "evaluation_manifest",
    "extended_records",
    "extended_summary",
    "alignment_records",
    "alignment_summary",
}


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    if not resolved.is_relative_to(WORKSPACE.resolve(strict=True)):
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


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def atomic_json(path: Path, payload: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check(condition: bool, message: str, checks: list[str]) -> None:
    if not condition:
        raise ValueError(message)
    checks.append(message)


def record_key(row: dict[str, Any]) -> tuple[str, int]:
    return str(row["category"]), int(row["seed"])


def rewrite_terminal_for_replay(
    row: dict[str, Any], reference: Path, replay: Path
) -> dict[str, Any]:
    rewritten = copy.deepcopy(row)
    reference_text = str(reference)
    replay_text = str(replay)
    for field in ("artifact_dir", "model_urdf"):
        value = rewritten.get(field)
        if isinstance(value, str):
            if not value.startswith(reference_text + os.sep):
                raise ValueError(f"terminal {field} is outside reference root: {value}")
            rewritten[field] = replay_text + value[len(reference_text) :]
    return rewritten


def normalize_replay_payload(payload: Any, source_root: str, replay_root: str) -> Any:
    if isinstance(payload, dict):
        return {
            key: normalize_replay_payload(value, source_root, replay_root)
            for key, value in payload.items()
            if key not in TIME_KEYS
            and key not in DERIVED_HASH_KEYS
            and not (key == "hashes" and isinstance(value, dict))
        }
    if isinstance(payload, list):
        return [normalize_replay_payload(value, source_root, replay_root) for value in payload]
    if isinstance(payload, str):
        return payload.replace(source_root, replay_root)
    return payload


def core_equal(reference: Path, replay: Path, relative: str) -> bool:
    left_path = reference / relative
    right_path = replay / relative
    if relative.endswith(".jsonl"):
        left: Any = load_jsonl(left_path)
        right: Any = load_jsonl(right_path)
    elif relative.endswith(".json"):
        left = load_json(left_path)
        right = load_json(right_path)
    else:
        left = left_path.read_text(encoding="utf-8")
        right = right_path.read_text(encoding="utf-8")
    return normalize_replay_payload(left, str(reference), str(replay)) == normalize_replay_payload(
        right, str(replay), str(replay)
    )


def compare_metric_dict(
    expected: dict[str, Any], observed: dict[str, Any], label: str, checks: list[str]
) -> None:
    for field, value in expected.items():
        check(field in observed, f"{label} contains {field}", checks)
        check(observed[field] == value, f"{label} recomputes {field}", checks)


def validate_reference(reference: Path) -> dict[str, Any]:
    runner = load_module("verify_pva_expanded_runner", RUNNER)
    tree = load_module("verify_pva_expanded_tree", TREE)
    extended = load_module("verify_pva_expanded_extended", EXTENDED)
    partnet = load_module("verify_pva_expanded_partnet", PARTNET)
    protocol = partnet.load_protocol(ONTOLOGY)
    checks: list[str] = []

    manifest_path = contained(reference / "frozen_manifest.jsonl")
    records_path = contained(reference / "records.jsonl")
    progress_path = contained(reference / "records.progress.jsonl")
    evaluation_path = contained(reference / "evaluation_manifest.jsonl")
    extended_records_path = contained(reference / "extended_structure_records.jsonl")
    extended_summary_path = contained(reference / "extended_structure_summary.json")
    scores_path = contained(reference / "partnet_scores/records.jsonl")
    score_summary_path = contained(reference / "partnet_scores/summary.json")
    summary_path = contained(reference / "summary.json")
    preflight_path = contained(reference / "preflight.json")
    gate_path = contained(reference / "old_n30_compatibility_gate.json")
    nested_path = contained(reference / "old_n30_nested_validation.json")
    state_path = contained(reference / "state.json")

    manifest = load_jsonl(manifest_path)
    deterministic_manifest = runner.selection()
    check(len(manifest) == REQUESTED, "manifest has exactly 150 rows", checks)
    check(manifest == deterministic_manifest, "manifest equals deterministic selection", checks)
    check(
        Counter(str(row["category"]) for row in manifest)
        == Counter({category: PER_CATEGORY for category in CATEGORIES}),
        "manifest has five categories x 30",
        checks,
    )
    check(
        len({record_key(row) for row in manifest}) == REQUESTED,
        "manifest category-seed identities are unique",
        checks,
    )
    for category in CATEGORIES:
        seeds = sorted(int(row["seed"]) for row in manifest if row["category"] == category)
        check(seeds == list(range(PER_CATEGORY)), f"{category} seeds are exactly 0..29", checks)
        check(seeds[:6] == list(range(6)), f"{category} old six are exact prefix", checks)

    terminal_paths = sorted((reference / "terminal").glob("*/seed_*.json"))
    terminal = [load_json(path) for path in terminal_paths]
    records = load_jsonl(records_path)
    progress = load_jsonl(progress_path)
    check(len(terminal) == REQUESTED, "terminal directory has 150 rows", checks)
    check(len({record_key(row) for row in terminal}) == REQUESTED, "terminal identities are unique", checks)
    check(
        sorted(terminal, key=record_key) == sorted(records, key=record_key),
        "terminal rows equal records.jsonl",
        checks,
    )
    check(records == progress, "final records and progress journal are byte-semantic equal", checks)
    check(
        {record_key(row) for row in records} == {record_key(row) for row in manifest},
        "terminal identities equal frozen manifest",
        checks,
    )

    gate = load_json(gate_path)
    check(gate["status"] == "PASS", "old compatibility gate status is PASS", checks)
    check(gate["checked_count"] == 30, "old compatibility gate checks 30", checks)
    check(gate["match_count"] == 30, "old compatibility gate matches 30/30", checks)
    check(gate["mismatch_count"] == 0, "old compatibility gate has zero mismatch", checks)
    check(
        all(bool(row["match"]) for row in gate["checks"]),
        "all old compatibility rows match",
        checks,
    )

    failures = [row for row in records if row["compile_status"] != "PASS"]
    available = [row for row in records if row.get("parseable_final_urdf")]
    check(len(failures) == EXPECTED_FAILURES, "exactly 25 failures are retained", checks)
    check(len(available) == REQUESTED - EXPECTED_FAILURES, "exactly 125 assets are available", checks)
    check(
        Counter(row["failure_type"] for row in failures)
        == Counter(
            {
                "fail_if_isolated_parts()": 8,
                "fail_if_parts_overlap_in_current_pose()": 17,
            }
        ),
        "failure types are 8 isolated plus 17 overlap",
        checks,
    )
    check(
        Counter(row["category"] for row in failures)
        == Counter({"microwave": 8, "refrigerator": 17}),
        "failure category counts are preserved",
        checks,
    )

    evaluation = load_jsonl(evaluation_path)
    check(len(evaluation) == REQUESTED, "evaluation manifest has 150 rows", checks)
    evaluation_by_id = {str(row["sample_id"]): row for row in evaluation}
    check(len(evaluation_by_id) == REQUESTED, "evaluation sample IDs are unique", checks)
    extended_records = load_jsonl(extended_records_path)
    extended_by_id = {str(row["sample_id"]): row for row in extended_records}
    check(len(extended_by_id) == len(available), "extended records cover every available asset", checks)
    scores = load_jsonl(scores_path)
    score_by_id = {str(row["sample_id"]): row for row in scores}
    check(len(score_by_id) == REQUESTED, "PartNet records cover all requested assets", checks)

    tree_fields: set[str] | None = None
    extended_fields: set[str] | None = None
    partnet_fields: set[str] | None = None
    for row in records:
        sample_id = f"{row['category']}/seed_{row['seed']}"
        evaluated = evaluation_by_id[sample_id]
        expected_available = bool(row.get("parseable_final_urdf"))
        check(bool(evaluated["available"]) == expected_available, f"{sample_id} availability agrees", checks)
        check(int(evaluated["selection_rank"]) == int(row["seed"]) + 1, f"{sample_id} rank agrees", checks)
        score = score_by_id[sample_id]
        check(bool(score["available"]) == expected_available, f"{sample_id} PartNet availability agrees", checks)
        if not expected_available:
            check(not score.get("evaluation_complete"), f"{sample_id} failure remains unevaluated", checks)
            continue
        urdf = contained(Path(str(row["model_urdf"])))
        digest = sha256(urdf)
        check(digest == row["model_urdf_sha256"], f"{sample_id} terminal URDF hash matches", checks)
        check(digest == evaluated["urdf_sha256"], f"{sample_id} manifest URDF hash matches", checks)
        check(digest == score["urdf_sha256"], f"{sample_id} PartNet URDF hash matches", checks)

        recomputed_tree = tree.parse_hierarchy(urdf)
        tree_fields = tree_fields or set(recomputed_tree)
        compare_metric_dict(recomputed_tree, row, f"{sample_id} shared tree", checks)

        recomputed_extended = extended.analyze_urdf(urdf)
        extended_fields = extended_fields or set(recomputed_extended)
        compare_metric_dict(
            recomputed_extended, extended_by_id[sample_id], f"{sample_id} extended", checks
        )

        recomputed_partnet = partnet.evaluate_urdf(urdf, str(row["category"]), protocol)
        partnet_fields = partnet_fields or set(recomputed_partnet)
        compare_metric_dict(recomputed_partnet, score, f"{sample_id} PartNet", checks)
        check(score["evaluation_complete"] is True, f"{sample_id} PartNet evaluation complete", checks)

    extended_summary = load_json(extended_summary_path)
    recomputed_extended_overall = extended.aggregate(extended_records, REQUESTED)
    check(
        extended_summary["overall"] == recomputed_extended_overall,
        "extended overall aggregate recomputes",
        checks,
    )
    for category in CATEGORIES:
        category_rows = [row for row in extended_records if row["category"] == category]
        check(
            extended_summary["per_category"][category]["metrics"]
            == extended.aggregate(category_rows, PER_CATEGORY),
            f"{category} extended aggregate recomputes",
            checks,
        )
        check(
            extended_summary["per_category"][category]["topology"]
            == extended.topology_consistency(category_rows),
            f"{category} topology aggregate recomputes",
            checks,
        )

    score_summary = load_json(score_summary_path)
    check(score_summary["overall"] == partnet.aggregate(scores), "PartNet overall aggregate recomputes", checks)
    for category in CATEGORIES:
        check(
            score_summary["per_category"][category]
            == partnet.aggregate([row for row in scores if row["category"] == category]),
            f"{category} PartNet aggregate recomputes",
            checks,
        )
    check(score_summary["bootstrap"]["replicates"] == 10_000, "bootstrap has 10000 replicates", checks)
    check(score_summary["bootstrap"]["seed"] == 20260812, "bootstrap seed is pinned", checks)
    check(
        score_summary["bootstrap"]
        == runner.stratified_bootstrap(
            scores,
            {
                "role_coverage_requested": lambda row: float(row.get("semantic_role_coverage") or 0.0),
                "scorable_coverage_requested": lambda row: float(bool(row.get("scorable"))),
                "induced_edge_f1_requested": lambda row: float(row.get("parent_child_edge_f1") or 0.0),
                "coverage_weighted_induced_edge_f1": lambda row: float(row.get("parent_child_edge_f1") or 0.0)
                * float(row.get("semantic_role_coverage") or 0.0),
                "induced_exact_requested": lambda row: float(bool(row.get("hierarchy_exact_match"))),
                "semantic_parent_alignment_requested": lambda row: float(row.get("semantic_nesting_accuracy") or 0.0),
            },
        ),
        "PartNet category-stratified bootstrap recomputes exactly",
        checks,
    )

    summary = load_json(summary_path)
    preflight = load_json(preflight_path)
    nested = load_json(nested_path)
    state = load_json(state_path)
    check(summary["status"] == "COMPLETE_WITH_FAILURES", "summary status preserves failures", checks)
    check(summary["selection"]["requested_count"] == REQUESTED, "summary requested denominator is 150", checks)
    check(summary["selection"]["failures_not_replaced"] is True, "summary declares failures not replaced", checks)
    check(summary["materialization"]["overall"] == runner.summarize_rows(records, REQUESTED), "materialization overall recomputes", checks)
    for category in CATEGORIES:
        check(
            summary["materialization"]["per_category"][category]
            == runner.summarize_rows([row for row in records if row["category"] == category], PER_CATEGORY),
            f"{category} materialization summary recomputes",
            checks,
        )
    check(summary["structure"] == extended_summary, "summary embeds exact extended summary", checks)
    check(summary["alignment"] == score_summary, "summary embeds exact PartNet summary", checks)
    check(nested["status"] == "PASS" and nested["nested_count"] == 30, "old N30 nested validation passes", checks)
    check(state["phase"] == "complete" and state["terminal_count"] == 150, "state is complete at 150", checks)
    check(state["failure_count"] == EXPECTED_FAILURES, "state reports 25 failures", checks)

    expected_hashes = {
        "records": sha256(records_path),
        "evaluation_manifest": sha256(evaluation_path),
        "extended_records": sha256(extended_records_path),
        "extended_summary": sha256(extended_summary_path),
        "alignment_records": sha256(scores_path),
        "alignment_summary": sha256(score_summary_path),
        "runner": sha256(RUNNER),
    }
    check(summary["hashes"] == expected_hashes, "summary hashes pin all core artifacts and runner", checks)
    check(preflight == runner.preflight(), "preflight recomputes exactly", checks)
    for module_name, path in {
        "tree": TREE,
        "extended": EXTENDED,
        "partnet": PARTNET,
        "ontology": ONTOLOGY,
    }.items():
        check(
            preflight["shared_modules"][module_name]["sha256"] == sha256(path),
            f"preflight pins current {module_name} hash",
            checks,
        )
    for template in preflight["templates"]:
        check(
            template["template_sha256"] == sha256(Path(template["template_path"])),
            f"preflight template hash matches {template['category']}",
            checks,
        )

    result = {
        "status": "PASS",
        "protocol_id": runner.PROTOCOL_ID,
        "requested_count": REQUESTED,
        "available_count": len(available),
        "failure_count": len(failures),
        "per_category_requested": PER_CATEGORY,
        "check_count": len(checks),
        "recomputed_field_counts": {
            "shared_tree": len(tree_fields or ()),
            "extended_structure": len(extended_fields or ()),
            "partnet_alignment": len(partnet_fields or ()),
        },
        "artifact_hashes": expected_hashes,
        "checks": checks,
    }
    atomic_json(reference / "verification.json", result)
    return result


def replay_preparation_mode(replay: Path, *, exists: bool, reuse: bool) -> str:
    del replay
    if exists:
        return "reuse" if reuse else "reject"
    return "create"


def prepare_replay(reference: Path, replay: Path, *, reuse: bool = False) -> None:
    mode = replay_preparation_mode(replay, exists=replay.exists(), reuse=reuse)
    if mode == "reject":
        raise FileExistsError(f"refusing to overwrite replay: {replay}")
    if mode == "reuse":
        required = (
            "packages",
            "terminal",
            "frozen_manifest.jsonl",
            "records.progress.jsonl",
        )
        missing = [relative for relative in required if not (replay / relative).exists()]
        if missing:
            raise ValueError(f"existing replay is incomplete: {missing}")
        return
    replay.mkdir(parents=True)
    # Hard-link the immutable package files into an isolated namespace.
    shutil.copytree(reference / "packages", replay / "packages", copy_function=os.link)
    for relative in ("preflight.json", "frozen_manifest.jsonl"):
        shutil.copy2(reference / relative, replay / relative)
    terminal_rows: list[dict[str, Any]] = []
    for source in sorted((reference / "terminal").glob("*/seed_*.json")):
        row = rewrite_terminal_for_replay(load_json(source), reference, replay)
        destination = replay / source.relative_to(reference)
        destination.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(destination, row)
        terminal_rows.append(row)
    terminal_rows.sort(key=record_key)
    temporary = replay / "records.progress.jsonl"
    temporary.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in terminal_rows),
        encoding="utf-8",
    )


def replay_and_compare(reference: Path, replay: Path, *, reuse_replay: bool = False) -> dict[str, Any]:
    prepare_replay(reference, replay, reuse=reuse_replay)
    command = [sys.executable, str(RUNNER), "--output", str(replay), "--finalize-only"]
    completed = subprocess.run(command, cwd=WORKSPACE / "arti-skill", text=True, capture_output=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay finalize failed\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )
    replay_verification = validate_reference(replay)
    comparable = (
        "frozen_manifest.jsonl",
        "records.jsonl",
        "evaluation_manifest.jsonl",
        "extended_structure_records.jsonl",
        "extended_structure_summary.json",
        "partnet_scores/records.jsonl",
        "partnet_scores/summary.json",
        "old_n30_compatibility_gate.json",
        "old_n30_nested_validation.json",
        "report.md",
        "summary.json",
    )
    comparisons = {
        relative: {
            "reference_sha256": sha256(reference / relative),
            "replay_sha256": sha256(replay / relative),
            "core_equal_after_declared_path_time_normalization": core_equal(
                reference, replay, relative
            ),
        }
        for relative in comparable
    }
    reference_records = load_jsonl(reference / "records.jsonl")
    replay_records = load_jsonl(replay / "records.jsonl")
    replay_paths_isolated = all(
        not row.get("parseable_final_urdf")
        or (
            str(row["artifact_dir"]).startswith(str(replay) + os.sep)
            and str(row["model_urdf"]).startswith(str(replay) + os.sep)
        )
        for row in replay_records
    )
    package_file_count = sum(path.is_file() for path in replay.joinpath("packages").rglob("*"))
    linked_file_count = sum(
        source.stat().st_ino == (replay / source.relative_to(reference)).stat().st_ino
        for source in reference.joinpath("packages").rglob("*")
        if source.is_file()
    )
    checks = {
        "replay_finalize_exit_zero": completed.returncode == 0,
        "replay_verification_passed": replay_verification["status"] == "PASS",
        "replay_terminal_paths_are_isolated": replay_paths_isolated,
        "replay_has_150_terminal_rows": len(replay_records) == REQUESTED,
        "replay_has_125_available_rows": sum(
            bool(row.get("parseable_final_urdf")) for row in replay_records
        )
        == 125,
        "replay_preserves_25_failures": sum(
            row["compile_status"] != "PASS" for row in replay_records
        )
        == EXPECTED_FAILURES,
        "replay_package_files_are_hardlinked": package_file_count > 0
        and linked_file_count == package_file_count,
        "all_core_artifacts_equal": all(
            row["core_equal_after_declared_path_time_normalization"]
            for row in comparisons.values()
        ),
        "terminal_core_records_equal": normalize_replay_payload(
            reference_records, str(reference), str(replay)
        )
        == normalize_replay_payload(replay_records, str(replay), str(replay)),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "reference": str(reference),
        "replay": str(replay),
        "replay_mode": (
            "hard-linked immutable package tree plus rebased terminal journal; "
            "runner --finalize-only; no compile"
        ),
        "replay_command": command,
        "package_file_count": package_file_count,
        "hardlinked_package_file_count": linked_file_count,
        "checks": checks,
        "file_comparisons": comparisons,
        "runner_stdout": completed.stdout,
        "runner_stderr": completed.stderr,
    }
    atomic_json(reference / "determinism_verification.json", result)
    if result["status"] != "PASS":
        failed = [name for name, passed in checks.items() if not passed]
        raise ValueError(f"determinism replay failed: {failed}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--reuse-replay", action="store_true")
    args = parser.parse_args()
    reference = contained(args.reference)
    replay = contained(args.replay, exists=False)
    verification = validate_reference(reference)
    determinism = (
        None
        if args.skip_replay
        else replay_and_compare(reference, replay, reuse_replay=args.reuse_replay)
    )
    print(
        json.dumps(
            {
                "status": verification["status"],
                "check_count": verification["check_count"],
                "failure_count": verification["failure_count"],
                "determinism_status": determinism["status"] if determinism else "SKIPPED",
                "reference": str(reference),
                "replay": str(replay) if determinism else None,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
