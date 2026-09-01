#!/usr/bin/env python3
"""Run the nested Articraft-10K Table 3 expansion (30/category, N=150)."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import random
from statistics import mean
import subprocess
from typing import Any

import hierarchy_extended_metrics as extended
from partnet_hierarchy_correctness import aggregate as alignment_aggregate
from partnet_hierarchy_correctness import evaluate_urdf, load_protocol
import run_articraft_hierarchy_paper as base


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_SKILL = WORKSPACE / "arti-skill"
DEFAULT_OUTPUT = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_expanded_n150/articraft"
OLD_FROZEN = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_correctness/articraft/cohort_manifest.jsonl"
ONTOLOGY = ARTI_SKILL / "exp/reference/partnet_hierarchy_correctness_v1.json"
SCORER_RUNNER = ARTI_SKILL / "exp/scripts/run_partnet_hierarchy_correctness.py"
PROTOCOL = "articraft_10k_hierarchy_expanded_hash30_v1"
PER_CATEGORY = 30
OLD_PER_CATEGORY = 6
EXPECTED_MANIFEST_COUNT = 10_788
EXPECTED_OLD_FROZEN_SHA256 = "6ee21cb5ce0ed18ab760e35592fb2a165e883a0b67c70d78006107d499d8023f"
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812
CATEGORY_MAP = {
    "storage_furniture_cabinet": "hingeddoor_cabinet",
    "table": "overbed_table",
    "refrigerator": "refrigerator_with_hinged_doors",
    "dishwasher": "dishwasher_with_dropdown_door_and_sliding_racks",
    "microwave": "microwave_oven",
}


def contained(path: Path, *, exists: bool = False) -> Path:
    resolved = path.resolve(strict=exists)
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


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def load_shared_tree_parser() -> tuple[Any, str]:
    parser_path = contained(base.SHARED_EVALUATOR, exists=True)
    spec = importlib.util.spec_from_file_location("expanded_shared_tree_parser", parser_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load shared tree parser: {parser_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.parse_hierarchy, sha256(parser_path)


def frozen_selection() -> tuple[list[dict[str, Any]], dict[str, Any]]:
    manifest = load_jsonl(contained(base.DATASET_MANIFEST, exists=True))
    if len(manifest) != EXPECTED_MANIFEST_COUNT:
        raise ValueError(f"expected manifest N={EXPECTED_MANIFEST_COUNT}, found {len(manifest)}")
    if len({str(row["record_id"]) for row in manifest}) != len(manifest):
        raise ValueError("released manifest contains duplicate record IDs")

    old_path = contained(OLD_FROZEN, exists=True)
    if sha256(old_path) != EXPECTED_OLD_FROZEN_SHA256:
        raise ValueError("the original frozen-6 manifest hash changed")
    old_rows = load_jsonl(old_path)
    old_by_category = {
        category: sorted(
            (row for row in old_rows if row["cohort"] == category),
            key=lambda row: int(row["rank"]),
        )
        for category in CATEGORY_MAP
    }

    selected: list[dict[str, Any]] = []
    category_audit: list[dict[str, Any]] = []
    nesting_checks: dict[str, Any] = {}
    for cohort, category_slug in CATEGORY_MAP.items():
        released = [row for row in manifest if row.get("category_slug") == category_slug]
        retained = [row for row in released if row.get("rating") in (4, 5)]
        if len(retained) < PER_CATEGORY:
            raise ValueError(f"{cohort}: only {len(retained)} rating-4/5 candidates")
        ranked = sorted(
            retained,
            key=lambda row: (
                base.rank_key(cohort, str(row["record_id"])),
                str(row["record_id"]),
            ),
        )
        old_category = old_by_category[cohort]
        if len(old_category) != OLD_PER_CATEGORY:
            raise ValueError(f"{cohort}: expected {OLD_PER_CATEGORY} original frozen rows")
        expected_prefix = [str(row["record_id"]) for row in ranked[:OLD_PER_CATEGORY]]
        observed_prefix = [str(row["record_id"]) for row in old_category]
        if expected_prefix != observed_prefix:
            raise ValueError(
                f"{cohort}: expanded ranking is not nested over the original frozen-6"
            )
        if any(
            str(old["rank_sha256"])
            != base.rank_key(cohort, str(old["record_id"]))
            for old in old_category
        ):
            raise ValueError(f"{cohort}: original frozen rank hash mismatch")

        inspected = {str(row["record_id"]): base.inspect_source(row) for row in retained}
        cohort_selection: list[dict[str, Any]] = []
        for rank, row in enumerate(ranked[:PER_CATEGORY], start=1):
            record_id = str(row["record_id"])
            selection = {
                "cohort": cohort,
                "ontology_category": (
                    "storage_furniture" if cohort == "storage_furniture_cabinet" else cohort
                ),
                "category_slug": category_slug,
                "rank": rank,
                "rank_sha256": base.rank_key(cohort, record_id),
                "record_id": record_id,
                "rating": row.get("rating"),
                "provider": row.get("provider"),
                "model_id": row.get("model_id"),
                "run_status": row.get("run_status"),
                **inspected[record_id],
            }
            cohort_selection.append(selection)
            selected.append(selection)
        category_audit.append(
            {
                "cohort": cohort,
                "category_slug": category_slug,
                "released_count": len(released),
                "retained_rating_4_5_count": len(retained),
                "retained_source_complete_count": sum(
                    bool(source["source_complete"]) for source in inspected.values()
                ),
                "selected_count": len(cohort_selection),
                "selected_source_complete_count": sum(
                    bool(row["source_complete"]) for row in cohort_selection
                ),
            }
        )
        nesting_checks[cohort] = {
            "original_frozen_count": len(old_category),
            "original_is_exact_prefix": True,
            "original_record_ids": observed_prefix,
            "expanded_prefix_record_ids": expected_prefix,
        }

    if Counter(row["cohort"] for row in selected) != Counter(
        {category: PER_CATEGORY for category in CATEGORY_MAP}
    ):
        raise ValueError("expanded category counts are not exactly 30 each")
    audit = {
        "protocol_id": PROTOCOL,
        "selection_namespace": base.RANK_NAMESPACE,
        "selection_rule": (
            "Within each exact frozen category slug, retain release records with rating in "
            "{4,5}; sort by SHA256(namespace + NUL + cohort + NUL + record_id), "
            "tie-break by record_id, and take ranks 1..30. The unchanged namespace and "
            "cohort keys make the original ranks 1..6 an exact prefix. Source, compile, "
            "parse, structure, and alignment outcomes never affect membership; failures "
            "remain in the requested denominator without replacement."
        ),
        "released_manifest_count": len(manifest),
        "released_manifest_sha256": sha256(contained(base.DATASET_MANIFEST, exists=True)),
        "per_category_requested": PER_CATEGORY,
        "requested_count": len(selected),
        "original_frozen_manifest": str(old_path),
        "original_frozen_manifest_sha256": sha256(old_path),
        "category_audit": category_audit,
        "nesting_checks": nesting_checks,
    }
    return selected, audit


def result_is_resumable(row: dict[str, Any]) -> bool:
    if not row.get("final_urdf_present"):
        return True
    path = Path(str(row.get("model_urdf") or ""))
    return path.is_file() and sha256(path) == row.get("model_urdf_sha256")


def materialize(
    selection: list[dict[str, Any]],
    output: Path,
    timeout_seconds: int,
    resume: bool,
) -> tuple[list[dict[str, Any]], str]:
    parse_hierarchy, shared_tree_hash = load_shared_tree_parser()
    results_path = contained(output / "records.jsonl")
    existing = load_jsonl(results_path) if resume and results_path.is_file() else []
    if existing:
        if [row["record_id"] for row in existing] != [
            row["record_id"] for row in selection[: len(existing)]
        ]:
            raise ValueError("existing result rows are not a prefix of frozen selection")
        if not all(result_is_resumable(row) for row in existing):
            raise ValueError("existing result artifact failed resume hash validation")

    data_root = contained(output / "selected_data")
    data_root.mkdir(parents=True, exist_ok=True)
    results = list(existing)
    for index, selected in enumerate(selection[len(results) :], start=len(results) + 1):
        print(
            f"[{index}/{len(selection)}] {selected['cohort']} {selected['record_id']}",
            flush=True,
        )
        result = base.compile_record(
            selected,
            data_root=data_root,
            output=output,
            timeout_seconds=timeout_seconds,
        )
        result["materialization_mode"] = "fresh_compile"
        result["ontology_category"] = selected["ontology_category"]
        if result["final_urdf_present"]:
            try:
                result.update(parse_hierarchy(Path(str(result["model_urdf"]))))
                result["parseable_final_urdf"] = True
            except Exception as exc:  # noqa: BLE001
                result["error"] = f"hierarchy_parse:{type(exc).__name__}:{exc}"
        results.append(result)
        write_jsonl(results_path, results)
    return results, shared_tree_hash


def build_evaluation_manifest(results: list[dict[str, Any]], output: Path) -> Path:
    rows: list[dict[str, Any]] = []
    for result in results:
        available = bool(result["final_urdf_present"] and result["parseable_final_urdf"])
        row = {
            "method": "Articraft",
            "sample_id": f"articraft_expanded__{result['cohort']}__{result['record_id']}",
            "category": result["ontology_category"],
            "urdf_path": result.get("model_urdf"),
            "urdf_sha256": result.get("model_urdf_sha256"),
            "available": available,
            "selection_rank": result["rank"],
        }
        if not available:
            row["unavailable_reason"] = result.get("error") or result["compile_status"]
        rows.append(row)
    path = contained(output / "evaluation_manifest.jsonl")
    write_jsonl(path, rows)
    return path


def extended_structure(results: list[dict[str, Any]], output: Path) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    for result in results:
        if not (result["final_urdf_present"] and result["parseable_final_urdf"]):
            continue
        record = {
            "sample_id": f"articraft_expanded__{result['cohort']}__{result['record_id']}",
            "cohort": result["cohort"],
            "record_id": result["record_id"],
            "selection_rank": result["rank"],
            "model_urdf": result["model_urdf"],
            "model_urdf_sha256": result["model_urdf_sha256"],
            **extended.analyze_urdf(Path(str(result["model_urdf"]))),
        }
        records.append(record)
    records_path = contained(output / "extended_structure_records.jsonl")
    write_jsonl(records_path, records)
    per_category: dict[str, Any] = {}
    for category in CATEGORY_MAP:
        category_rows = [row for row in records if row["cohort"] == category]
        per_category[category] = {
            "metrics": extended.aggregate(category_rows, PER_CATEGORY),
            "topology": extended.topology_consistency(category_rows),
        }
    topology_fields = (
        "unique_signature_rate",
        "mode_rate",
        "pairwise_exact_rate",
        "normalized_entropy",
    )
    payload = {
        "protocol_id": PROTOCOL,
        "requested_count": len(results),
        "requested_per_category": PER_CATEGORY,
        "available_parseable_count": len(records),
        "conditioning": "descriptor means use valid trees; requested rates retain all failures",
        "shared_evaluator": str(contained(Path(extended.__file__), exists=True)),
        "shared_evaluator_sha256": sha256(contained(Path(extended.__file__), exists=True)),
        "overall": extended.aggregate(records, len(results)),
        "per_category": per_category,
        "topology_macro_equal_category": {
            field: mean(
                float(per_category[category]["topology"][field])
                for category in CATEGORY_MAP
                if per_category[category]["topology"][field] is not None
            )
            for field in topology_fields
        },
        "records_path": str(records_path),
        "records_sha256": sha256(records_path),
    }
    summary_path = contained(output / "extended_structure_summary.json")
    summary_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return payload


def run_alignment(manifest: Path, output: Path) -> None:
    score_output = contained(output / "partnet_scores")
    command = [
        "python3",
        str(contained(SCORER_RUNNER, exists=True)),
        "--manifest",
        str(manifest),
        "--output",
        str(score_output),
        "--protocol",
        str(contained(ONTOLOGY, exists=True)),
        "--expected-per-category",
        str(PER_CATEGORY),
    ]
    subprocess.run(command, cwd=ARTI_SKILL, check=True)


def alignment_requested_value(row: dict[str, Any], metric: str) -> float:
    if metric == "role_coverage":
        return float(row.get("semantic_role_coverage") or 0.0)
    if metric == "scorable":
        return float(bool(row.get("scorable")))
    if metric == "edge_f1":
        return float(row.get("parent_child_edge_f1") or 0.0)
    if metric == "coverage_weighted_induced_edge_f1":
        return float(row.get("parent_child_edge_f1") or 0.0) * float(
            row.get("semantic_role_coverage") or 0.0
        )
    if metric == "exact":
        return float(bool(row.get("hierarchy_exact_match")))
    if metric == "nesting":
        return float(row.get("semantic_nesting_accuracy") or 0.0)
    raise KeyError(metric)


def quantile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def alignment_bootstrap(records: list[dict[str, Any]]) -> dict[str, Any]:
    categories = sorted({str(row["category"]) for row in records})
    metric_names = (
        "role_coverage",
        "scorable",
        "edge_f1",
        "coverage_weighted_induced_edge_f1",
        "exact",
        "nesting",
    )
    estimates = {
        metric: mean(alignment_requested_value(row, metric) for row in records)
        for metric in metric_names
    }
    samples = {metric: [] for metric in metric_names}
    rng = random.Random(BOOTSTRAP_SEED)
    for _ in range(BOOTSTRAP_REPLICATES):
        resampled: list[dict[str, Any]] = []
        for category in categories:
            population = [row for row in records if str(row["category"]) == category]
            if len(population) != PER_CATEGORY:
                raise ValueError(f"{category}: expected {PER_CATEGORY} bootstrap rows")
            resampled.extend(rng.choice(population) for _ in range(PER_CATEGORY))
        for metric in metric_names:
            samples[metric].append(
                mean(alignment_requested_value(row, metric) for row in resampled)
            )
    return {
        "design": (
            "category-stratified asset bootstrap with replacement; five categories equally "
            "weighted and 30 draws per category"
        ),
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "interval": "95% percentile",
        "metrics": {
            metric: {
                "estimate": estimates[metric],
                "ci95_percentile": [
                    quantile(sorted(samples[metric]), 0.025),
                    quantile(sorted(samples[metric]), 0.975),
                ],
            }
            for metric in metric_names
        },
    }


def run_alignment_direct(manifest: Path, output: Path) -> None:
    """Evaluate N=150 using the frozen shared scorer module directly."""
    protocol_path = contained(ONTOLOGY, exists=True)
    protocol = load_protocol(protocol_path)
    rows = load_jsonl(manifest)
    if len(rows) != PER_CATEGORY * len(CATEGORY_MAP):
        raise ValueError(f"expected {PER_CATEGORY * len(CATEGORY_MAP)} alignment rows")
    expected_categories = set(protocol["categories"])
    counts = Counter(str(row["category"]) for row in rows)
    if counts != Counter({category: PER_CATEGORY for category in expected_categories}):
        raise ValueError(f"expanded alignment category counts mismatch: {counts}")
    if len({str(row["sample_id"]) for row in rows}) != len(rows):
        raise ValueError("duplicate expanded alignment sample_id")
    for category in expected_categories:
        ranks = sorted(
            int(row["selection_rank"])
            for row in rows
            if str(row["category"]) == category
        )
        if ranks != list(range(1, PER_CATEGORY + 1)):
            raise ValueError(f"{category}: expanded alignment ranks are not 1..30")

    records: list[dict[str, Any]] = []
    for source in rows:
        row = dict(source)
        row["evaluation_complete"] = False
        if source.get("available"):
            urdf = contained(Path(str(source["urdf_path"])), exists=True)
            if sha256(urdf) != source["urdf_sha256"]:
                raise ValueError(f"URDF hash mismatch for {source['sample_id']}")
            try:
                row.update(evaluate_urdf(urdf, str(row["category"]), protocol))
                row["evaluation_complete"] = True
                row["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        else:
            row["evaluation_error"] = source.get("unavailable_reason", "unavailable")
        records.append(row)

    score_output = contained(output / "partnet_scores")
    score_output.mkdir(parents=True, exist_ok=True)
    records_path = contained(score_output / "records.jsonl")
    write_jsonl(records_path, records)
    scorer_core = contained(
        ARTI_SKILL / "exp/scripts/partnet_hierarchy_correctness.py", exists=True
    )
    per_category = {
        category: alignment_aggregate(
            [row for row in records if str(row["category"]) == category]
        )
        for category in sorted(expected_categories)
    }
    summary = {
        "protocol_id": "nano3d_partnet_hierarchy_correctness_expanded_n150_v1",
        "method": "Articraft",
        "claim_boundary": protocol["claim_boundary"],
        "requested_per_category": PER_CATEGORY,
        "requested_count": len(records),
        "selection_manifest_sha256": sha256(manifest),
        "partnet_protocol_sha256": sha256(protocol_path),
        "scorer_core": str(scorer_core),
        "scorer_sha256": sha256(scorer_core),
        "runner_mode": "direct import of the frozen shared scorer core",
        "overall": alignment_aggregate(records),
        "per_category": per_category,
        "bootstrap": alignment_bootstrap(records),
    }
    (score_output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def write_summary(
    selection_audit: dict[str, Any],
    selection_path: Path,
    results: list[dict[str, Any]],
    evaluation_manifest: Path,
    structure: dict[str, Any],
    output: Path,
    timeout_seconds: int,
    shared_tree_hash: str,
) -> dict[str, Any]:
    parseable = [row for row in results if row["parseable_final_urdf"]]
    failures = Counter(
        row["compile_status"]
        for row in results
        if not row["parseable_final_urdf"]
    )
    per_category = {}
    for category in CATEGORY_MAP:
        rows = [row for row in results if row["cohort"] == category]
        per_category[category] = {
            "requested_count": len(rows),
            "source_complete_count": sum(bool(row["source_complete"]) for row in rows),
            "compile_success_count": sum(row["compile_status"] == "success" for row in rows),
            "final_urdf_count": sum(bool(row["final_urdf_present"]) for row in rows),
            "parseable_count": sum(bool(row["parseable_final_urdf"]) for row in rows),
            "valid_tree_count": sum(bool(row.get("valid_tree")) for row in rows),
        }
    status = "COMPLETE" if len(parseable) == len(results) else "COMPLETE_WITH_FAILURES"
    alignment_summary_path = output / "partnet_scores/summary.json"
    alignment_summary = (
        load_json(alignment_summary_path) if alignment_summary_path.is_file() else None
    )
    summary = {
        "status": status,
        "protocol_id": PROTOCOL,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "claim_boundary": (
            "expanded official-release Articraft package audit; not a common-prompt rerun, "
            "and PartNet alignment remains an ontology proxy rather than kinematic gold"
        ),
        "evaluation_unit": "one frozen rating-4/5 Articraft-10K record freshly compiled to a final package",
        "selection": {
            **selection_audit,
            "manifest": str(selection_path),
            "manifest_sha256": sha256(selection_path),
        },
        "materialization": {
            "requested_count": len(results),
            "fresh_compile_attempt_count": len(results),
            "source_complete_count": sum(bool(row["source_complete"]) for row in results),
            "compile_success_count": sum(row["compile_status"] == "success" for row in results),
            "final_urdf_count": sum(bool(row["final_urdf_present"]) for row in results),
            "parseable_count": len(parseable),
            "failure_counts": dict(sorted(failures.items())),
            "timeout_seconds": timeout_seconds,
            "per_category": per_category,
        },
        "structure": structure,
        "alignment": {
            "status": "COMPLETE" if alignment_summary is not None else "PENDING",
            "output": str(output / "partnet_scores"),
            "summary": alignment_summary,
        },
        "hashes": {
            "records": sha256(contained(output / "records.jsonl", exists=True)),
            "evaluation_manifest": sha256(evaluation_manifest),
            "ontology": sha256(contained(ONTOLOGY, exists=True)),
            "shared_tree_parser": shared_tree_hash,
            "shared_extended_evaluator": structure["shared_evaluator_sha256"],
            "alignment_records": (
                sha256(contained(output / "partnet_scores/records.jsonl", exists=True))
                if alignment_summary is not None
                else None
            ),
            "alignment_summary": (
                sha256(contained(alignment_summary_path, exists=True))
                if alignment_summary is not None
                else None
            ),
            "runner": sha256(contained(Path(__file__), exists=True)),
        },
    }
    path = contained(output / "summary.json")
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def write_report(summary: dict[str, Any], output: Path) -> None:
    materialization = summary["materialization"]
    structure = summary["structure"]["overall"]
    alignment = summary["alignment"]["summary"]
    lines = [
        "# Articraft Table 3 expanded-N audit",
        "",
        f"Status: **{summary['status']}**",
        "",
        "The frozen cohort contains 30 rating-4/5 release records in each of five categories "
        "(N=150). It is an exact nested extension of the original frozen six per category; "
        "failures are retained without replacement.",
        "",
        "| Category | Eligible rating 4/5 | Requested | Source complete | Compile success | Parseable | Valid tree |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    capacity = {
        row["cohort"]: row["retained_rating_4_5_count"]
        for row in summary["selection"]["category_audit"]
    }
    for category in CATEGORY_MAP:
        row = materialization["per_category"][category]
        lines.append(
            f"| {category} | {capacity[category]} | {row['requested_count']} | "
            f"{row['source_complete_count']} | {row['compile_success_count']} | "
            f"{row['parseable_count']} | {row['valid_tree_count']} |"
        )
    lines.extend(
        [
            "",
            f"Overall requested-denominator valid-tree rate: "
            f"{structure['valid_tree_count']}/{structure['requested_count']}.",
            "Descriptor means are conditional on valid trees. PartNet scores are emitted "
            "separately and remain category-ontology alignment proxies.",
            "",
        ]
    )
    if alignment is not None:
        overall = alignment["overall"]
        weighted = alignment["bootstrap"]["metrics"][
            "coverage_weighted_induced_edge_f1"
        ]
        low, high = weighted["ci95_percentile"]
        lines.extend(
            [
                "## PartNet-ontology alignment proxy",
                "",
                f"- Available: {overall['available_count']}/150",
                f"- Scorable: {overall['scorable_count']}/150",
                f"- Role coverage (requested macro): "
                f"{100 * overall['semantic_role_coverage_requested_macro']:.2f}%",
                f"- Coverage-Weighted Induced Edge F1: "
                f"{100 * weighted['estimate']:.2f}% "
                f"[{100 * low:.2f}%, {100 * high:.2f}%]",
                f"- Induced Edge F1 requested / conditional: "
                f"{100 * overall['parent_child_edge_f1_requested_macro']:.2f}% / "
                f"{100 * overall['parent_child_edge_f1_conditional_macro']:.2f}%",
                "",
                "The alignment scores use the frozen shared scorer module directly and "
                "remain prediction-side lexical recovery plus category-level PartNet "
                "ontology alignment, not instance-level kinematic ground truth.",
                "",
            ]
        )
    (output / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--skip-alignment", action="store_true")
    parser.add_argument(
        "--alignment-direct",
        action="store_true",
        help="Import and call the frozen PartNet scorer core directly for N=150.",
    )
    args = parser.parse_args()
    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)

    selection, selection_audit = frozen_selection()
    selection_path = contained(output / "selection_manifest.jsonl")
    if selection_path.is_file():
        current = load_jsonl(selection_path)
        if current != selection:
            raise ValueError("existing expanded selection differs from deterministic selection")
    else:
        write_jsonl(selection_path, selection)
    selection_audit["selection_manifest_sha256"] = sha256(selection_path)
    (output / "selection_audit.json").write_text(
        json.dumps(selection_audit, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.preflight_only:
        print(json.dumps(selection_audit, indent=2, sort_keys=True))
        return 0

    results, shared_tree_hash = materialize(
        selection, output, args.timeout_seconds, args.resume
    )
    if len(results) != len(selection):
        raise ValueError("materialization did not produce one terminal row per request")
    evaluation_manifest = build_evaluation_manifest(results, output)
    structure = extended_structure(results, output)
    if not args.skip_alignment:
        if args.alignment_direct:
            run_alignment_direct(evaluation_manifest, output)
        else:
            run_alignment(evaluation_manifest, output)
    summary = write_summary(
        selection_audit,
        selection_path,
        results,
        evaluation_manifest,
        structure,
        output,
        args.timeout_seconds,
        shared_tree_hash,
    )
    write_report(summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
