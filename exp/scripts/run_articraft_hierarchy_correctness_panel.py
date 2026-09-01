#!/usr/bin/env python3
"""Materialize the frozen Articraft PartNet hierarchy correctness panel."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from statistics import mean
import subprocess
from typing import Any

import run_articraft_hierarchy_paper as base


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_SKILL = WORKSPACE / "arti-skill"
OLD_PANEL = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_paper/articraft"
DEFAULT_OUTPUT = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_correctness/articraft"
PARTNET = ARTI_SKILL / "exp/baselines/partnet_dataset-official"
ONTOLOGY = ARTI_SKILL / "exp/reference/partnet_hierarchy_correctness_v1.json"
SHARED_SCORER = ARTI_SKILL / "exp/scripts/partnet_hierarchy_correctness.py"
SHARED_SCORER_RUNNER = ARTI_SKILL / "exp/scripts/run_partnet_hierarchy_correctness.py"
OLD_SELECTION_SHA256 = "dea74ac6b6007a9c63be914de814c907f60e96ca0fa44ba03d654481536f732b"
ONTOLOGY_SHA256 = "d63e02ba5cf11cf0e0b37acf61451fe9e6877060d82913f2e0f816e77aa81b85"
PROTOCOL = "nano3d_hierarchy_partnet_correctness_five_category_v1"
INHERITED_COHORTS = (
    "storage_furniture_cabinet",
    "table",
    "refrigerator",
    "dishwasher",
)
EXPECTED_COHORTS = (*INHERITED_COHORTS, "microwave")


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


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def git_commit(repo: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    return completed.stdout.strip()


def select_cohort() -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    old_selection_path = contained(OLD_PANEL / "selection_manifest.jsonl")
    if sha256(old_selection_path) != OLD_SELECTION_SHA256:
        raise ValueError("old frozen selection hash changed")
    old_rows = load_jsonl(old_selection_path)
    inherited = [row for row in old_rows if row["cohort"] in INHERITED_COHORTS]
    if len(inherited) != 24 or Counter(row["cohort"] for row in inherited) != Counter(
        {cohort: 6 for cohort in INHERITED_COHORTS}
    ):
        raise ValueError("expected four inherited cohorts x 6")

    manifest = load_jsonl(contained(base.DATASET_MANIFEST))
    released = [row for row in manifest if row.get("category_slug") == "microwave_oven"]
    retained = [row for row in released if row.get("rating") in (4, 5)]
    inspected = [(row, base.inspect_source(row)) for row in retained]
    ranked = sorted(
        retained,
        key=lambda row: (
            base.rank_key("microwave", str(row["record_id"])),
            str(row["record_id"]),
        ),
    )
    source_by_id = {str(row["record_id"]): source for row, source in inspected}
    candidate_rows: list[dict[str, Any]] = []
    for rank, row in enumerate(ranked, start=1):
        record_id = str(row["record_id"])
        candidate_rows.append(
            {
                "cohort": "microwave",
                "category_slug": "microwave_oven",
                "rank": rank,
                "rank_sha256": base.rank_key("microwave", record_id),
                "selected": rank <= 6,
                "record_id": record_id,
                "rating": row.get("rating"),
                "provider": row.get("provider"),
                "model_id": row.get("model_id"),
                "run_status": row.get("run_status"),
                **source_by_id[record_id],
            }
        )
    selected_microwave = [row for row in candidate_rows if row["selected"]]
    if len(selected_microwave) != 6:
        raise ValueError("microwave selection did not produce six records")
    selection = inherited + selected_microwave
    audit = {
        "released_microwave_count": len(released),
        "paper_retained_rating_4_5_count": len(retained),
        "retained_source_complete_count": sum(
            bool(source["source_complete"]) for _, source in inspected
        ),
        "retained_source_incomplete_count": sum(
            not bool(source["source_complete"]) for _, source in inspected
        ),
        "selected_microwave_count": len(selected_microwave),
        "old_selection_sha256": OLD_SELECTION_SHA256,
        "inherited_record_count": len(inherited),
        "selection_rule": (
            "inherit the exact first four frozen Articraft cohorts; for official microwave_oven, "
            "retain release rating 4-5, sort by SHA256('nano3d_articraft_paper_hierarchy_v1' "
            "+ NUL + 'microwave' + NUL + record_id), tie-break by record_id, take first 6; "
            "identity-only and no replacement after materialization or evaluation failure"
        ),
    }
    return selection, candidate_rows, audit


def reuse_frozen_record(
    selection: dict[str, Any],
    *,
    data_root: Path,
    old_by_id: dict[str, dict[str, Any]],
    parse_hierarchy: Any,
) -> dict[str, Any]:
    record_id = str(selection["record_id"])
    source = old_by_id[record_id]
    source_urdf = contained(Path(str(source["model_urdf"])))
    if sha256(source_urdf) != source["model_urdf_sha256"]:
        raise ValueError(f"old frozen URDF hash changed: {record_id}")
    base.copy_selected_record(selection, data_root)
    source_package = contained(Path(str(source["package_dir"])))
    target_package = contained(data_root / "cache/record_materialization" / record_id)
    target_package.parent.mkdir(parents=True, exist_ok=True)
    if target_package.exists():
        shutil.rmtree(target_package)
    shutil.copytree(source_package, target_package)
    target_urdf = contained(target_package / "model.urdf")
    copied_hash = sha256(target_urdf)
    if copied_hash != source["model_urdf_sha256"]:
        raise ValueError(f"copied frozen URDF hash mismatch: {record_id}")
    result: dict[str, Any] = {
        "cohort": selection["cohort"],
        "category_slug": selection["category_slug"],
        "record_id": record_id,
        "rating": selection["rating"],
        "rank": selection["rank"],
        "rank_sha256": selection["rank_sha256"],
        "source_complete": selection["source_complete"],
        "materialization_mode": "reused_verified_frozen_output",
        "compile_status": "reused_frozen_success",
        "compile_returncode": source["compile_returncode"],
        "compile_elapsed_seconds": None,
        "final_urdf_present": True,
        "parseable_final_urdf": False,
        "error": None,
        "package_dir": str(target_package),
        "model_urdf": str(target_urdf),
        "model_urdf_sha256": copied_hash,
        "origin_model_urdf": str(source_urdf),
    }
    try:
        result.update(parse_hierarchy(target_urdf))
        result["parseable_final_urdf"] = True
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"hierarchy_parse:{type(exc).__name__}:{exc}"
    return result


def fresh_compile_record(
    selection: dict[str, Any],
    *,
    data_root: Path,
    output: Path,
    parse_hierarchy: Any,
    timeout_seconds: int,
) -> dict[str, Any]:
    result = base.compile_record(
        selection,
        data_root=data_root,
        output=output,
        timeout_seconds=timeout_seconds,
    )
    result["materialization_mode"] = "fresh_compile"
    if result["final_urdf_present"]:
        try:
            result.update(parse_hierarchy(Path(str(result["model_urdf"]))))
            result["parseable_final_urdf"] = True
        except Exception as exc:  # noqa: BLE001
            result["error"] = f"hierarchy_parse:{type(exc).__name__}:{exc}"
    return result


def build_report(summary: dict[str, Any]) -> str:
    selected = summary["selected_cohort"]
    return "\n".join(
        [
            "# Articraft hierarchy correctness panel materialization",
            "",
            f"Status: **{summary['status']}**",
            "",
            f"- Requested records: {selected['selected_count']} (6 per category)",
            f"- Reused verified frozen outputs: {selected['reused_frozen_count']}/24",
            f"- Fresh microwave compile success: {selected['fresh_compile_success_count']}/6",
            f"- Parseable final URDF: {selected['parseable_count']}/30",
            f"- Valid tree: {selected['valid_tree_count']}/30",
            f"- Cohort manifest SHA-256: `{selected['cohort_manifest_sha256']}`",
            f"- Evaluation manifest SHA-256: `{selected['evaluation_manifest_sha256']}`",
            "- Selection is identity-only and failures are retained without replacement.",
            "- Correctness scoring is delegated to the frozen shared PartNet scorer; this runner does not infer roles.",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if sha256(contained(ONTOLOGY)) != ONTOLOGY_SHA256:
        raise ValueError("frozen PartNet correctness ontology hash mismatch")
    selection, microwave_candidates, selection_audit = select_cohort()
    manifest_path = contained(output / "cohort_manifest.jsonl")
    candidates_path = contained(output / "microwave_candidates.jsonl")
    write_jsonl(manifest_path, selection)
    write_jsonl(candidates_path, microwave_candidates)
    selection_audit.update(
        {
            "cohort_manifest_sha256": sha256(manifest_path),
            "microwave_candidates_sha256": sha256(candidates_path),
            "category_counts": dict(sorted(Counter(row["cohort"] for row in selection).items())),
            "partnet_ontology": str(ONTOLOGY),
            "partnet_ontology_sha256": ONTOLOGY_SHA256,
        }
    )
    (output / "selection_audit.json").write_text(
        json.dumps(selection_audit, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if args.preflight_only:
        print(json.dumps(selection_audit, indent=2, sort_keys=True))
        return 0

    parse_hierarchy, shared_hash = base.load_shared_parser()
    old_results = load_jsonl(contained(OLD_PANEL / "records.jsonl"))
    old_by_id = {str(row["record_id"]): row for row in old_results}
    data_root = contained(output / "selected_data")
    data_root.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    for index, selected in enumerate(selection, start=1):
        print(f"[{index}/30] {selected['cohort']} {selected['record_id']}", flush=True)
        if selected["cohort"] in INHERITED_COHORTS:
            result = reuse_frozen_record(
                selected,
                data_root=data_root,
                old_by_id=old_by_id,
                parse_hierarchy=parse_hierarchy,
            )
        else:
            result = fresh_compile_record(
                selected,
                data_root=data_root,
                output=output,
                parse_hierarchy=parse_hierarchy,
                timeout_seconds=args.timeout_seconds,
            )
        results.append(result)
    results_path = contained(output / "results.jsonl")
    write_jsonl(results_path, results)
    evaluation_rows: list[dict[str, Any]] = []
    for result in results:
        available = bool(result["final_urdf_present"] and result["parseable_final_urdf"])
        row = {
            "method": "Articraft",
            "sample_id": f"articraft__{result['cohort']}__{result['record_id']}",
            "category": result["cohort"],
            "urdf_path": result.get("model_urdf"),
            "urdf_sha256": result.get("model_urdf_sha256"),
            "available": available,
            "selection_rank": result["rank"],
        }
        if not available:
            row["unavailable_reason"] = result.get("error") or result["compile_status"]
        evaluation_rows.append(row)
    evaluation_manifest_path = contained(output / "evaluation_manifest.jsonl")
    write_jsonl(evaluation_manifest_path, evaluation_rows)

    parseable = [row for row in results if row["parseable_final_urdf"]]
    failure_counts = Counter(
        row["compile_status"] for row in results if not row["parseable_final_urdf"]
    )
    status = "COMPLETE" if len(parseable) == 30 else (
        "COMPLETE_WITH_FAILURES" if parseable else "BLOCKED"
    )
    runner_path = contained(Path(__file__))
    summary = {
        "status": status,
        "protocol": PROTOCOL,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "claim_boundary": (
            "official-release correctness-panel materialization; role correctness is scored "
            "only by the separately frozen shared PartNet scorer"
        ),
        "evaluation_unit": "one requested Articraft record and its final URDF",
        "selection_audit": selection_audit,
        "selected_cohort": {
            "selected_count": len(results),
            "per_category_n": 6,
            "cohort_manifest_sha256": sha256(manifest_path),
            "evaluation_manifest_sha256": sha256(evaluation_manifest_path),
            "reused_frozen_count": sum(
                row["materialization_mode"] == "reused_verified_frozen_output"
                for row in results
            ),
            "fresh_compile_count": sum(
                row["materialization_mode"] == "fresh_compile" for row in results
            ),
            "fresh_compile_success_count": sum(
                row["materialization_mode"] == "fresh_compile"
                and row["compile_status"] == "success"
                for row in results
            ),
            "final_urdf_count": sum(bool(row["final_urdf_present"]) for row in results),
            "parseable_count": len(parseable),
            "valid_tree_count": sum(bool(row.get("valid_tree")) for row in parseable),
        },
        "hierarchy_descriptors": {
            "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in parseable) if parseable else None,
            "semantic_depth_min": min(int(row["semantic_depth"]) for row in parseable) if parseable else None,
            "semantic_depth_max": max(int(row["semantic_depth"]) for row in parseable) if parseable else None,
            "pivot_count_mean": mean(int(row["pivot_count"]) for row in parseable) if parseable else None,
            "pivot_count_total": sum(int(row["pivot_count"]) for row in parseable),
        },
        "failure_counts": dict(sorted(failure_counts.items())),
        "correctness_scorer": {
            "status": "READY_FOR_SHARED_SCORER",
            "evaluation_manifest": str(evaluation_manifest_path),
            "ontology": str(ONTOLOGY),
            "ontology_sha256": ONTOLOGY_SHA256,
            "shared_scorer": str(SHARED_SCORER),
            "shared_scorer_sha256": sha256(contained(SHARED_SCORER)),
            "shared_scorer_runner": str(SHARED_SCORER_RUNNER),
            "shared_scorer_runner_sha256": sha256(contained(SHARED_SCORER_RUNNER)),
        },
        "provenance": {
            "runner": str(runner_path),
            "runner_sha256": sha256(runner_path),
            "dataset_commit": git_commit(base.DATASET),
            "paper_harness_commit": git_commit(base.PAPER_HARNESS),
            "partnet_meta_commit": git_commit(PARTNET),
            "shared_hierarchy_evaluator": str(base.SHARED_EVALUATOR),
            "shared_hierarchy_evaluator_sha256": shared_hash,
            "old_panel_selection_sha256": OLD_SELECTION_SHA256,
            "old_panel_records_sha256": sha256(contained(OLD_PANEL / "records.jsonl")),
        },
        "artifacts": {
            "cohort_manifest": str(manifest_path),
            "microwave_candidates": str(candidates_path),
            "results": str(results_path),
            "evaluation_manifest": str(evaluation_manifest_path),
        },
    }
    summary_path = contained(output / "summary.json")
    report_path = contained(output / "report.md")
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report_path.write_text(build_report(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if parseable else 2


if __name__ == "__main__":
    raise SystemExit(main())
