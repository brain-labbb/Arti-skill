#!/usr/bin/env python3
"""Evaluate the strict LAM release pool and freeze the blocked balanced N=150 panel."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random
from statistics import mean
import subprocess
from typing import Any

from dotenv import dotenv_values
import pyarrow.parquet as pq
import yaml

from hierarchy_extended_metrics import analyze_urdf, aggregate as structure_aggregate
from hierarchy_extended_metrics import topology_consistency
from partnet_hierarchy_correctness import aggregate as partnet_aggregate
from partnet_hierarchy_correctness import evaluate_urdf, load_protocol
from run_nano3d_hierarchy_correctness_lam import MICROWAVE_ALLOWLIST
from run_nano3d_hierarchy_paper_lam import (
    CATEGORY_ALLOWLISTS,
    DATASET_FILE_SHA256,
    DATASET_REVISION,
    REPOSITORY_COMMIT,
    SELECTION_SALT,
    rank_key,
)


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_REPO = EXP_ROOT / "baselines/LAM-official"
DEFAULT_DATASET = EXP_ROOT / "baselines/LAM-official-dataset"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/lam"
DEFAULT_PARTNET_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_MAIN30 = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam/manifest.jsonl"
DEFAULT_ENV = EXP_ROOT.parent / "articraft_data/.env"
ADAPTED_RUN_ROOT = EXP_ROOT / "runtime/table4_constraints_v2_lam_real"
TARGET_PER_CATEGORY = 30
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812
EXPECTED_MAIN30_SHA256 = "07dd54be7921dac088b3a0aa0d94ebbe1fb1801dbc494290a0cb51f58bdb126a"

STRICT_CATEGORY_ALLOWLISTS = {
    "storage_furniture_cabinet": CATEGORY_ALLOWLISTS["storage_furniture_cabinet"],
    "table": CATEGORY_ALLOWLISTS["table"],
    "refrigerator": CATEGORY_ALLOWLISTS["refrigerator"],
    "dishwasher": CATEGORY_ALLOWLISTS["dishwasher"],
    "microwave": MICROWAVE_ALLOWLIST,
}
PARTNET_CATEGORY = {
    "storage_furniture_cabinet": "storage_furniture",
    "table": "table",
    "refrigerator": "refrigerator",
    "dishwasher": "dishwasher",
    "microwave": "microwave",
}
PROMPT_FILES = {
    "storage_furniture_cabinet": ("storage_furniture.txt", "cabinet.txt"),
    "table": ("table.txt",),
    "refrigerator": ("refrigerator.txt",),
    "dishwasher": ("dishwasher.txt",),
    "microwave": ("microwave.txt",),
}
PARTNET_MACRO_FIELDS = (
    "semantic_role_coverage_requested_macro",
    "scorable_asset_coverage_requested",
    "coverage_weighted_induced_edge_f1_requested_macro",
    "parent_child_edge_f1_requested_macro",
    "parent_child_edge_f1_conditional_macro",
    "hierarchy_exact_match_requested_rate",
    "semantic_nesting_accuracy_requested_macro",
)
STRUCTURE_MACRO_FIELDS = (
    "valid_tree_rate_requested",
    "node_count_mean",
    "edge_count_mean",
    "leaf_count_mean",
    "branching_node_count_mean",
    "movable_edge_count_mean",
    "fixed_edge_count_mean",
    "semantic_depth_mean",
    "movable_depth_mean",
    "visual_link_ratio_mean",
    "collision_link_ratio_mean",
    "largest_component_node_rate_mean_evaluated",
    "valid_joint_endpoint_rate_mean_evaluated",
)


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def category_macro(
    per_category: dict[str, dict[str, Any]], fields: tuple[str, ...]
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "category_count": len(per_category),
        "aggregation": "unweighted mean over the five strict shared categories",
    }
    for field in fields:
        values = [
            float(row[field])
            for row in per_category.values()
            if row.get(field) is not None
        ]
        result[field] = mean(values) if values else None
        result[f"{field}_contributing_category_count"] = len(values)
    return result


def topology_macro(per_category: dict[str, dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "category_count": len(per_category),
        "aggregation": "unweighted mean over the five strict shared categories",
    }
    for field in (
        "unique_signature_rate",
        "mode_rate",
        "pairwise_exact_rate",
        "normalized_entropy",
    ):
        values = [
            float(row[field])
            for row in per_category.values()
            if row.get(field) is not None
        ]
        result[field] = mean(values) if values else None
        result[f"{field}_contributing_category_count"] = len(values)
    return result


def quantile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def category_stratified_bootstrap(
    records: list[dict[str, Any]],
    value_functions: dict[str, Any],
    *,
    category_field: str,
) -> dict[str, Any]:
    populations = {
        category: [row for row in records if row[category_field] == category]
        for category in sorted({str(row[category_field]) for row in records})
    }
    if len(populations) != 5 or any(not rows for rows in populations.values()):
        raise ValueError("bootstrap requires five non-empty strict categories")

    estimates = {
        metric: mean(
            mean(float(value(row)) for row in population)
            for population in populations.values()
        )
        for metric, value in value_functions.items()
    }
    samples = {metric: [] for metric in value_functions}
    rng = random.Random(BOOTSTRAP_SEED)
    for _ in range(BOOTSTRAP_REPLICATES):
        sampled_by_category = {
            category: [rng.choice(population) for _ in range(len(population))]
            for category, population in populations.items()
        }
        for metric, value in value_functions.items():
            samples[metric].append(
                mean(
                    mean(float(value(row)) for row in population)
                    for population in sampled_by_category.values()
                )
            )
    return {
        "design": (
            "category-stratified asset bootstrap with replacement; each category "
            "retains its observed sample count and the five category estimates are "
            "averaged with equal weight"
        ),
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "interval": "95% percentile",
        "category_sample_counts": {
            category: len(population) for category, population in populations.items()
        },
        "metrics": {
            metric: {
                "estimate": estimates[metric],
                "ci95_percentile": [
                    quantile(sorted(values), 0.025),
                    quantile(sorted(values), 0.975),
                ],
            }
            for metric, values in samples.items()
        },
    }


def load_prompts(repo: Path) -> tuple[dict[str, list[str]], dict[str, dict[str, str]]]:
    prompts: dict[str, list[str]] = {}
    hashes: dict[str, dict[str, str]] = {}
    for category, filenames in PROMPT_FILES.items():
        category_prompts: list[str] = []
        hashes[category] = {}
        for filename in filenames:
            path = contained(repo / "data/val_data" / filename)
            hashes[category][filename] = sha256_file(path)
            category_prompts.extend(
                line.strip()
                for line in path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            )
        if not category_prompts:
            raise ValueError(f"no official prompts for {category}")
        prompts[category] = category_prompts
    return prompts, hashes


def strict_ranked_rows(rows: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    raw_counts = Counter(str(row["category"]) for row in rows)
    for category, aliases in STRICT_CATEGORY_ALLOWLISTS.items():
        missing = sorted(alias for alias in aliases if raw_counts[alias] == 0)
        if missing:
            raise ValueError(f"strict allowlist entries absent for {category}: {missing}")
        candidates = [
            {**row, "dataset_row_index": index}
            for index, row in enumerate(rows)
            if str(row["category"]) in set(aliases)
        ]
        grouped[category] = [
            row
            for _, row in sorted(
                ((rank_key(category, row), row) for row in candidates),
                key=lambda item: (item[0], str(item[1]["object_release_id"])),
            )
        ]
    return grouped


def release_manifest_row(
    category: str,
    rank: int,
    row: dict[str, Any],
    urdf_path: Path,
) -> dict[str, Any]:
    urdf_text = row.get("urdf")
    available = isinstance(urdf_text, str) and bool(urdf_text.strip())
    return {
        "method": "LAM",
        "sample_id": f"row_{int(row['dataset_row_index']):04d}:{row['object_release_id']}",
        "shared_class": category,
        "category": PARTNET_CATEGORY[category],
        "selection_rank": rank,
        "dataset_row_index": int(row["dataset_row_index"]),
        "object_release_id": str(row["object_release_id"]),
        "raw_category": str(row["category"]),
        "selection_hash": rank_key(category, row),
        "source_kind": "official_release",
        "dataset_revision": DATASET_REVISION,
        "tier": row.get("tier"),
        "release_status": row.get("status"),
        "caption": row.get("caption"),
        "model": row.get("model"),
        "pipeline": row.get("pipeline"),
        "available": available,
        "urdf_path": str(urdf_path) if available else None,
        "urdf_sha256": (
            sha256_bytes(str(urdf_text).encode("utf-8")) if available else None
        ),
        "unavailable_reason": None if available else "official_release_missing_urdf",
    }


def build_release_panel(
    ranked: dict[str, list[dict[str, Any]]],
    output: Path,
    protocol: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    urdf_root = output / "urdfs"
    urdf_root.mkdir(parents=True, exist_ok=True)
    manifest: list[dict[str, Any]] = []
    structure_records: list[dict[str, Any]] = []
    partnet_records: list[dict[str, Any]] = []
    for category in STRICT_CATEGORY_ALLOWLISTS:
        for rank, row in enumerate(ranked[category], 1):
            urdf_path = urdf_root / f"row_{int(row['dataset_row_index']):04d}.urdf"
            urdf_text = row.get("urdf")
            if isinstance(urdf_text, str) and urdf_text.strip():
                urdf_path.write_text(urdf_text, encoding="utf-8")
            item = release_manifest_row(category, rank, row, urdf_path)
            manifest.append(item)
            if not item["available"]:
                continue
            if sha256_file(urdf_path) != item["urdf_sha256"]:
                raise ValueError(f"URDF materialization hash mismatch: {item['sample_id']}")
            structure_records.append({**item, **analyze_urdf(urdf_path)})
            partnet_row = {**item, "evaluation_complete": False}
            try:
                partnet_row.update(
                    evaluate_urdf(urdf_path, str(item["category"]), protocol)
                )
                partnet_row["evaluation_complete"] = True
                partnet_row["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                partnet_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            partnet_records.append(partnet_row)

    per_category_structure = {
        category: structure_aggregate(
            [row for row in structure_records if row["shared_class"] == category],
            requested_count=len(ranked[category]),
        )
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    per_category_topology = {
        category: topology_consistency(
            [row for row in structure_records if row["shared_class"] == category]
        )
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    structure_summary = {
        "status": "VALIDATED_OFFICIAL_RELEASE_STRICT_UNBALANCED",
        "claim_boundary": (
            "Package-level URDF structure and release topology diversity; this is an "
            "unbalanced supplementary panel, not the balanced N=150 comparison."
        ),
        "candidate_counts": {
            category: len(rows) for category, rows in ranked.items()
        },
        "overall_asset_weighted": structure_aggregate(
            structure_records, requested_count=len(manifest)
        ),
        "per_category": per_category_structure,
        "category_macro": category_macro(
            per_category_structure, STRUCTURE_MACRO_FIELDS
        ),
        "topology_consistency_per_category": per_category_topology,
        "topology_consistency_category_macro": topology_macro(per_category_topology),
        "bootstrap_intervals_category_macro": category_stratified_bootstrap(
            structure_records,
            {
                "valid_tree_rate_requested": lambda row: bool(row.get("valid_tree")),
                "largest_component_node_rate": lambda row: row.get(
                    "largest_component_node_rate"
                )
                or 0.0,
                "valid_joint_endpoint_rate": lambda row: row.get(
                    "valid_joint_endpoint_rate"
                )
                or 0.0,
            },
            category_field="shared_class",
        ),
    }
    per_category_partnet = {
        PARTNET_CATEGORY[category]: partnet_aggregate(
            [row for row in partnet_records if row["shared_class"] == category]
        )
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    partnet_summary = {
        "status": "VALIDATED_PARTNET_ONTOLOGY_PROXY_STRICT_UNBALANCED",
        "claim_boundary": protocol["claim_boundary"],
        "overall_asset_weighted": partnet_aggregate(partnet_records),
        "per_category": per_category_partnet,
        "category_macro": category_macro(per_category_partnet, PARTNET_MACRO_FIELDS),
        "bootstrap_intervals_category_macro": category_stratified_bootstrap(
            partnet_records,
            {
                "semantic_role_coverage_requested": lambda row: row.get(
                    "semantic_role_coverage"
                )
                or 0.0,
                "scorable_asset_coverage_requested": lambda row: bool(
                    row.get("scorable")
                ),
                "coverage_weighted_induced_edge_f1_requested": lambda row: (
                    float(row.get("parent_child_edge_f1") or 0.0)
                    * float(row.get("semantic_role_coverage") or 0.0)
                ),
                "induced_edge_f1_requested": lambda row: row.get(
                    "parent_child_edge_f1"
                )
                or 0.0,
                "induced_exact_requested": lambda row: bool(
                    row.get("hierarchy_exact_match")
                ),
                "semantic_parent_alignment_requested": lambda row: row.get(
                    "semantic_nesting_accuracy"
                )
                or 0.0,
            },
            category_field="category",
        ),
    }
    write_jsonl(output / "selection_manifest.jsonl", manifest)
    write_jsonl(output / "structure_records.jsonl", structure_records)
    write_json(output / "structure_summary.json", structure_summary)
    write_jsonl(output / "partnet_records.jsonl", partnet_records)
    write_json(output / "partnet_summary.json", partnet_summary)
    return manifest, structure_summary, partnet_summary, structure_records


def fresh_slot(
    category: str,
    rank: int,
    release_count: int,
    prompts: dict[str, list[str]],
) -> dict[str, Any]:
    offset = rank - release_count - 1
    category_prompts = prompts[category]
    prompt_index = offset % len(category_prompts)
    repeat_index = offset // len(category_prompts)
    prompt = category_prompts[prompt_index]
    identity_payload = "\n".join(
        (
            "nano3d-hierarchy-lam-expanded-n150-v1",
            category,
            str(rank),
            str(prompt_index),
            str(repeat_index),
            prompt,
        )
    )
    return {
        "method": "LAM",
        "sample_id": f"fresh:{category}:{rank:02d}",
        "shared_class": category,
        "category": PARTNET_CATEGORY[category],
        "selection_rank": rank,
        "selection_hash": sha256_bytes(identity_payload.encode("utf-8")),
        "source_kind": "paper_backend_fresh_generation",
        "prompt": prompt,
        "prompt_index": prompt_index,
        "repeat_index": repeat_index,
        "available": False,
        "urdf_path": None,
        "urdf_sha256": None,
        "execution_status": "BLOCKED_PAPER_BACKEND_UNAVAILABLE",
        "unavailable_reason": (
            "Official LAM paper-backend provider configuration is not runnable in the "
            "workspace; no adapted Qwen output is substituted."
        ),
        "provider_calls_made": 0,
    }


def evaluate_target_panel(
    ranked: dict[str, list[dict[str, Any]]],
    release_manifest: list[dict[str, Any]],
    output: Path,
    protocol: dict[str, Any],
    prompts: dict[str, list[str]],
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    release_by_identity = {
        (str(row["shared_class"]), int(row["selection_rank"])): row
        for row in release_manifest
    }
    manifest: list[dict[str, Any]] = []
    for category in STRICT_CATEGORY_ALLOWLISTS:
        release_count = min(TARGET_PER_CATEGORY, len(ranked[category]))
        for rank in range(1, TARGET_PER_CATEGORY + 1):
            if rank <= release_count:
                source = release_by_identity[(category, rank)]
                manifest.append(
                    {
                        **source,
                        "target_panel": "balanced_intention_to_run_n150",
                        "target_panel_source_status": "OBSERVED_OFFICIAL_RELEASE",
                    }
                )
            else:
                manifest.append(
                    {
                        **fresh_slot(category, rank, release_count, prompts),
                        "target_panel": "balanced_intention_to_run_n150",
                        "target_panel_source_status": "BLOCKED_FRESH_GENERATION",
                    }
                )

    available = [row for row in manifest if row["available"]]
    structure_records: list[dict[str, Any]] = []
    partnet_records: list[dict[str, Any]] = []
    for item in manifest:
        if not item["available"]:
            partnet_records.append({**item, "evaluation_complete": False})
            continue
        urdf_path = contained(Path(str(item["urdf_path"])))
        if sha256_file(urdf_path) != item["urdf_sha256"]:
            raise ValueError(f"target URDF hash mismatch: {item['sample_id']}")
        structure_records.append({**item, **analyze_urdf(urdf_path)})
        partnet_row = {**item, "evaluation_complete": False}
        try:
            partnet_row.update(evaluate_urdf(urdf_path, str(item["category"]), protocol))
            partnet_row["evaluation_complete"] = True
            partnet_row["evaluation_error"] = None
        except Exception as exc:  # noqa: BLE001
            partnet_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        partnet_records.append(partnet_row)

    per_category_structure = {
        category: structure_aggregate(
            [row for row in structure_records if row["shared_class"] == category],
            requested_count=TARGET_PER_CATEGORY,
        )
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    per_category_partnet = {
        PARTNET_CATEGORY[category]: partnet_aggregate(
            [row for row in partnet_records if row["shared_class"] == category]
        )
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    source_counts = dict(sorted(Counter(row["source_kind"] for row in manifest).items()))
    structure_summary = {
        "status": "BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT",
        "requested_count": len(manifest),
        "available_observed_release_count": len(available),
        "blocked_fresh_generation_count": len(manifest) - len(available),
        "source_counts": source_counts,
        "claim_boundary": (
            "Partial diagnostic only. The missing fresh-generation rows are not scored as "
            "method failures, and these values must not enter the balanced method comparison."
        ),
        "partial_structure_requested_denominator_diagnostic": structure_aggregate(
            structure_records, requested_count=len(manifest)
        ),
        "per_category_partial_diagnostic": per_category_structure,
        "category_macro_partial_diagnostic": category_macro(
            per_category_structure, STRUCTURE_MACRO_FIELDS
        ),
    }
    partnet_summary = {
        "status": "BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT",
        "requested_count": len(manifest),
        "available_observed_release_count": len(available),
        "blocked_fresh_generation_count": len(manifest) - len(available),
        "claim_boundary": (
            "Partial PartNet-ontology proxy diagnostic only. It is neither instance-level "
            "kinematic gold nor a completed balanced N=150 score."
        ),
        "partial_requested_denominator_diagnostic": partnet_aggregate(partnet_records),
        "per_category_partial_diagnostic": per_category_partnet,
        "category_macro_partial_diagnostic": category_macro(
            per_category_partnet, PARTNET_MACRO_FIELDS
        ),
    }
    write_jsonl(output / "intention_manifest.jsonl", manifest)
    write_jsonl(output / "structure_records_available.jsonl", structure_records)
    write_json(output / "structure_partial_summary.json", structure_summary)
    write_jsonl(output / "partnet_records.jsonl", partnet_records)
    write_json(output / "partnet_partial_summary.json", partnet_summary)
    return manifest, structure_summary, partnet_summary


def backend_preflight(repo: Path, env_path: Path) -> dict[str, Any]:
    config_path = contained(repo / "config.example.yaml")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    values = dotenv_values(env_path) if env_path.is_file() else {}
    base_url = str(values.get("OPENAI_BASE_URL") or "").strip().lower()
    exact_openai_endpoint = not base_url or base_url.startswith("https://api.openai.com")
    google_key = bool(
        values.get("GOOGLE_API_KEY")
        or values.get("GEMINI_API_KEY")
        or values.get("GEMINI_API_KEYS")
    )
    google_adc = values.get("GOOGLE_APPLICATION_CREDENTIALS")
    google_adc_ready = bool(google_adc and Path(str(google_adc)).is_file())

    attempts = []
    for path in sorted(ADAPTED_RUN_ROOT.rglob("attempt.json")):
        try:
            attempts.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
    api_metrics = []
    for path in sorted(ADAPTED_RUN_ROOT.rglob("api_metrics.json")):
        try:
            api_metrics.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
    adapted_status = Counter(str(row.get("status")) for row in attempts)
    return {
        "status": "BLOCKED_PAPER_BACKEND_UNAVAILABLE",
        "provider_calls_made_for_expanded_n150": 0,
        "official_config_example_sha256": sha256_file(config_path),
        "official_agent_models": config["api"]["agents"],
        "official_runtime_config_exists": (repo / "config.yaml").is_file(),
        "credential_values_recorded": False,
        "exact_openai_endpoint_configured": exact_openai_endpoint,
        "openai_credential_declared": bool(values.get("OPENAI_API_KEY")),
        "google_or_gemini_credential_declared": google_key,
        "google_application_default_credentials_ready": google_adc_ready,
        "blockers": [
            "official LAM config.yaml is absent",
            "the configured OpenAI-compatible route is not validated as the official paper backend",
            "Google/Gemini credentials required by the official critic/fixer stages are absent",
        ],
        "adapted_backend_excluded": {
            "label": "official LAM pipeline plus DashScope qwen3.8-max adapted backend",
            "paper_backend_exact": False,
            "historical_attempt_count": len(attempts),
            "historical_status_counts": dict(sorted(adapted_status.items())),
            "historical_attempts_with_final_urdf": sum(
                bool(row.get("final_urdf")) for row in attempts
            ),
            "historical_elapsed_seconds_sum": sum(
                float(row.get("elapsed_seconds") or 0.0) for row in attempts
            ),
            "historical_recorded_cost_usd_sum": sum(
                float(row.get("recorded_cost_usd_sum") or 0.0) for row in api_metrics
            ),
            "reason": "substituting Qwen would change the evaluated LAM backend",
        },
    }


def render_report(
    full_structure: dict[str, Any],
    full_partnet: dict[str, Any],
    target_structure: dict[str, Any],
    preflight: dict[str, Any],
) -> str:
    counts = full_structure["candidate_counts"]
    structure = full_structure["overall_asset_weighted"]
    proxy = full_partnet["category_macro"]
    return "\n".join(
        [
            "# LAM Expanded-N Hierarchy Audit",
            "",
            "## Official release strict-category supplementary",
            "",
            f"- Status: **{full_structure['status']}**",
            f"- Total: `{sum(counts.values())}` official rows; category counts `{json.dumps(counts, sort_keys=True)}`.",
            f"- Valid trees: `{structure['valid_tree_count']}/{structure['requested_count']}`.",
            "- Sampling: none within the strict allowlists; every release row is retained, including FAIL/broken rows.",
            f"- Category-macro coverage-weighted induced Edge F1: `{proxy['coverage_weighted_induced_edge_f1_requested_macro']:.6f}`.",
            "- This panel is unbalanced and supplementary; the asset-weighted result is cabinet dominated.",
            "",
            "## Balanced N=150 intention-to-run",
            "",
            f"- Status: **{target_structure['status']}**",
            f"- Requested: `{target_structure['requested_count']}` (five categories x 30).",
            f"- Existing strict official release rows: `{target_structure['available_observed_release_count']}`.",
            f"- Blocked paper-backend fresh rows: `{target_structure['blocked_fresh_generation_count']}`.",
            "- The original six identities/category are the first six frozen identities in every category.",
            "- Partial diagnostics are not a completed N=150 paper result and must not be compared against other methods.",
            "",
            "## Backend boundary",
            "",
            f"- Paper backend: **{preflight['status']}**.",
            "- Expanded-N provider calls: `0`.",
            "- A runnable Qwen-adapted path exists locally but is explicitly excluded because it is not the paper backend.",
            "",
            "## Metric boundary",
            "",
            "PartNet measurements are prediction-side lexical recovery and category-ontology alignment proxies, not instance-level kinematic hierarchy correctness.",
        ]
    ) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--partnet-protocol", type=Path, default=DEFAULT_PARTNET_PROTOCOL)
    parser.add_argument("--main30", type=Path, default=DEFAULT_MAIN30)
    parser.add_argument("--env-file", type=Path, default=DEFAULT_ENV)
    args = parser.parse_args()

    repo = contained(args.repo)
    dataset = contained(args.dataset)
    output = contained(args.output, exists=False)
    protocol_path = contained(args.partnet_protocol)
    main30_path = contained(args.main30)
    env_path = contained(args.env_file) if args.env_file.exists() else args.env_file.resolve()
    output.mkdir(parents=True, exist_ok=True)

    head = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if head != REPOSITORY_COMMIT:
        raise ValueError(f"official LAM checkout drift: {head}")
    for filename, expected in DATASET_FILE_SHA256.items():
        actual = sha256_file(dataset / filename)
        if actual != expected:
            raise ValueError(f"LAM dataset hash drift for {filename}: {actual}")
    if sha256_file(main30_path) != EXPECTED_MAIN30_SHA256:
        raise ValueError("frozen LAM Main30 identity manifest changed")

    full_rows = pq.read_table(dataset / "articulated_code.parquet").to_pylist()
    if len(full_rows) != 3217:
        raise ValueError(f"expected 3217 official release rows, found {len(full_rows)}")
    ranked = strict_ranked_rows(full_rows)
    prompts, prompt_hashes = load_prompts(repo)
    protocol = load_protocol(protocol_path)

    protocol_snapshot = {
        "protocol_id": "nano3d_hierarchy_lam_expanded_n150_v1",
        "frozen_target": {
            "categories": list(STRICT_CATEGORY_ALLOWLISTS),
            "per_category": TARGET_PER_CATEGORY,
            "total": TARGET_PER_CATEGORY * len(STRICT_CATEGORY_ALLOWLISTS),
            "original_frozen_prefix_per_category": 6,
            "failure_replacement": False,
        },
        "strict_raw_category_allowlists": STRICT_CATEGORY_ALLOWLISTS,
        "release_selection": {
            "rule": (
                "within each strict allowlist, ascending SHA-256 rank using the existing "
                "LAM selection namespace; no tier, status, parse, tree, or score filter"
            ),
            "selection_salt": SELECTION_SALT,
            "rank_payload": (
                "salt\\nshared_class\\nobject_release_id\\nraw_category\\ndataset_row_index"
            ),
            "official_release_full_policy": "retain every strict candidate",
            "balanced_target_policy": (
                "use strict release ranks 1..min(30, capacity), then freeze official "
                "prompt/repeat fresh-generation slots up to rank 30"
            ),
        },
        "fresh_generation": {
            "prompt_files": PROMPT_FILES,
            "prompt_source_hashes": prompt_hashes,
            "backend": "official LAM config.example.yaml model assignments",
            "adapted_backend_substitution_allowed": False,
            "provider_calls_before_freeze": 0,
        },
        "provenance": {
            "repository": "https://github.com/gaoypeng/LAM.git",
            "repository_commit": REPOSITORY_COMMIT,
            "dataset": "https://huggingface.co/datasets/YipengGao/Articulated-Object-Code",
            "dataset_revision": DATASET_REVISION,
            "dataset_file_sha256": DATASET_FILE_SHA256,
            "partnet_protocol": str(protocol_path),
            "partnet_protocol_sha256": sha256_file(protocol_path),
            "original_main30_manifest": str(main30_path),
            "original_main30_manifest_sha256": sha256_file(main30_path),
        },
        "metric_boundary": {
            "structure": "package-level URDF structural validity and diagnostics",
            "partnet": protocol["claim_boundary"],
            "balanced_incomplete_policy": (
                "partial diagnostics are never reported as a completed N=150 paper score"
            ),
        },
    }
    write_json(output / "protocol.json", protocol_snapshot)

    release_root = output / "official_release_full_strict"
    target_root = output / "balanced_n150_intention"
    release_manifest, full_structure, full_partnet, _ = build_release_panel(
        ranked, release_root, protocol
    )
    target_manifest, target_structure, target_partnet = evaluate_target_panel(
        ranked, release_manifest, target_root, protocol, prompts
    )
    preflight = backend_preflight(repo, env_path)
    write_json(output / "paper_backend_preflight.json", preflight)

    old_main30 = load_jsonl(main30_path)
    old_by_category = {
        category: [
            row
            for row in sorted(
                old_main30,
                key=lambda item: (
                    str(item["shared_class"]),
                    int(item["selection_rank_within_class"]),
                ),
            )
            if row["shared_class"] == category
        ]
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    target_by_category = {
        category: [
            row for row in target_manifest if row["shared_class"] == category
        ]
        for category in STRICT_CATEGORY_ALLOWLISTS
    }
    expected_candidate_counts = {
        "storage_furniture_cabinet": 237,
        "table": 21,
        "refrigerator": 6,
        "dishwasher": 14,
        "microwave": 22,
    }
    checks = {
        "dataset_row_count_3217": len(full_rows) == 3217,
        "strict_candidate_counts_exact": {
            category: len(rows) for category, rows in ranked.items()
        }
        == expected_candidate_counts,
        "official_release_total_300": len(release_manifest) == 300,
        "official_release_all_rows_have_urdf": all(
            row["available"] for row in release_manifest
        ),
        "balanced_target_total_150": len(target_manifest) == 150,
        "balanced_target_30_per_category": all(
            len(rows) == 30 for rows in target_by_category.values()
        ),
        "balanced_target_release_count_93": sum(
            row["source_kind"] == "official_release" for row in target_manifest
        )
        == 93,
        "balanced_target_blocked_count_57": sum(
            not row["available"] for row in target_manifest
        )
        == 57,
        "main30_embedded_as_rank_prefix": all(
            [int(row["dataset_row_index"]) for row in old_by_category[category]]
            == [
                int(row["dataset_row_index"])
                for row in target_by_category[category][:6]
            ]
            for category in STRICT_CATEGORY_ALLOWLISTS
        ),
        "target_ranks_contiguous_1_to_30": all(
            [int(row["selection_rank"]) for row in rows]
            == list(range(1, 31))
            for rows in target_by_category.values()
        ),
        "no_duplicate_release_sample_ids": len(
            {row["sample_id"] for row in release_manifest}
        )
        == len(release_manifest),
        "no_duplicate_target_sample_ids": len(
            {row["sample_id"] for row in target_manifest}
        )
        == len(target_manifest),
        "all_release_raw_categories_strict": all(
            row["raw_category"] in STRICT_CATEGORY_ALLOWLISTS[row["shared_class"]]
            for row in release_manifest
        ),
        "expanded_n150_provider_calls_zero": preflight[
            "provider_calls_made_for_expanded_n150"
        ]
        == 0,
        "adapted_backend_not_claimed_exact": preflight["adapted_backend_excluded"][
            "paper_backend_exact"
        ]
        is False,
        "target_structure_marked_not_paper_result": target_structure["status"]
        == "BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT",
        "target_partnet_marked_not_paper_result": target_partnet["status"]
        == "BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT",
    }
    validation = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "all_checks_pass": all(checks.values()),
        "input_hashes": {
            "official_dataset_articulated_code_parquet": sha256_file(
                dataset / "articulated_code.parquet"
            ),
            "partnet_protocol": sha256_file(protocol_path),
            "main30_manifest": sha256_file(main30_path),
            "runner": sha256_file(Path(__file__)),
            "hierarchy_extended_metrics": sha256_file(
                Path(__file__).with_name("hierarchy_extended_metrics.py")
            ),
            "partnet_hierarchy_correctness": sha256_file(
                Path(__file__).with_name("partnet_hierarchy_correctness.py")
            ),
        },
        "prompt_source_hashes": prompt_hashes,
        "output_hashes": {
            str(path.relative_to(output)): sha256_file(path)
            for path in (
                release_root / "selection_manifest.jsonl",
                release_root / "structure_records.jsonl",
                release_root / "structure_summary.json",
                release_root / "partnet_records.jsonl",
                release_root / "partnet_summary.json",
                target_root / "intention_manifest.jsonl",
                target_root / "structure_partial_summary.json",
                target_root / "partnet_partial_summary.json",
                output / "paper_backend_preflight.json",
                output / "protocol.json",
            )
        },
    }
    write_json(output / "validation.json", validation)
    (output / "report.md").write_text(
        render_report(full_structure, full_partnet, target_structure, preflight),
        encoding="utf-8",
    )
    summary = {
        "protocol": "nano3d_hierarchy_lam_expanded_n150_v1",
        "status": "VALIDATED_RELEASE_SUPPLEMENT_BALANCED_N150_BLOCKED",
        "official_release_full_strict": {
            "status": full_structure["status"],
            "requested_count": len(release_manifest),
            "candidate_counts": full_structure["candidate_counts"],
            "structure_summary": str(release_root / "structure_summary.json"),
            "partnet_summary": str(release_root / "partnet_summary.json"),
        },
        "balanced_n150_intention": {
            "status": target_structure["status"],
            "requested_count": len(target_manifest),
            "available_observed_release_count": target_structure[
                "available_observed_release_count"
            ],
            "blocked_fresh_generation_count": target_structure[
                "blocked_fresh_generation_count"
            ],
            "manifest": str(target_root / "intention_manifest.jsonl"),
        },
        "paper_backend_preflight": preflight["status"],
        "validation": validation["status"],
        "claim_boundary": (
            "The N=300 release result is an unbalanced supplementary audit. The balanced "
            "N=150 LAM panel is incomplete and has no paper-comparison score."
        ),
    }
    write_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
    return 0 if validation["all_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
