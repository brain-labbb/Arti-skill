#!/usr/bin/env python3
"""Build and evaluate the frozen Articraft 5-category x 7 matched cohort."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import run_articraft_naming_baseline as base


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/articraft_naming_matched_v1"
README_PATH = REPO_ROOT / "articraft_data/README.md"
RUNNER_CLI_PATH = REPO_ROOT / "articraft_data/agent/runner_cli.py"
SEMANTIC_FIELDS = (
    "semantic_precision",
    "semantic_recall",
    "naming_richness",
    "functional_core_coverage",
    "instance_discriminability",
    "over_segmentation_rate",
)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def require_file(path: Path) -> Path:
    resolved = base.canonical_inside(path, WORKSPACE_ROOT)
    if resolved.is_symlink() or not resolved.is_file():
        raise RuntimeError(f"Required regular file is missing: {resolved}")
    return resolved


def read_json(path: Path) -> dict[str, Any]:
    return base.read_json(require_file(path))


def write_json(path: Path, payload: object) -> None:
    resolved = base.canonical_inside(path, REPO_ROOT, must_exist=False)
    base.dump_json(resolved, payload)


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def selection_digest(canonical_category: str, candidate: base.Candidate) -> str:
    payload = (
        "nano3d-table2-naming-matched-v1|Articraft|"
        f"{canonical_category}|{candidate.record_id}|{candidate.model_sha256}"
    )
    return sha256_bytes(payload.encode("utf-8"))


def verify_protocol(protocol_path: Path, protocol: dict[str, Any]) -> dict[str, str]:
    if protocol.get("protocol_id") != "nano3d_table2_naming_matched_v1":
        raise RuntimeError("Unexpected matched protocol ID")
    evidence: dict[str, str] = {
        "shared_protocol": base.sha256_file(protocol_path),
    }
    for key in ("base_naming_protocol",):
        item = protocol[key]
        source = require_file(REPO_ROOT / item["path"])
        evidence[item["path"]] = base.sha256_file(source)
        if evidence[item["path"]] != item["sha256"]:
            raise RuntimeError(f"Protocol source hash mismatch: {item['path']}")
    release = protocol["source_evidence"]["articraft_cohort_audit"]
    release_path = require_file(REPO_ROOT / release["path"])
    evidence[release["path"]] = base.sha256_file(release_path)
    if evidence[release["path"]] != release["sha256"]:
        raise RuntimeError("Articraft release cohort hash mismatch")
    return dict(sorted(evidence.items()))


def audit_target_candidates(
    protocol: dict[str, Any], mirror_records_root: Path
) -> tuple[
    dict[str, list[base.Candidate]],
    dict[str, Any],
    dict[str, dict[str, Any]],
]:
    categories = protocol["design"]["canonical_categories"]
    source_to_canonical = {
        protocol["taxonomy"][category]["articraft"]: category for category in categories
    }
    checkout_root = base.canonical_inside(base.RECORDS_ROOT, WORKSPACE_ROOT)
    mirror_root = base.canonical_inside(mirror_records_root, WORKSPACE_ROOT)
    pools: dict[str, list[base.Candidate]] = defaultdict(list)
    provenance_by_id: dict[str, dict[str, Any]] = {}
    counts: Counter[str] = Counter()
    resolution_counts: Counter[str] = Counter()

    for entry in sorted(checkout_root.iterdir(), key=lambda item: item.name):
        if entry.is_symlink() or not entry.is_dir():
            continue
        counts["record_directories"] += 1
        record_id = entry.name
        record_relative = Path(record_id) / "record.json"
        record_path, record_evidence = base.resolve_lfs_overlay_file(
            checkout_root, mirror_root, record_relative
        )
        resolution_counts[str(record_evidence.get("status"))] += 1
        if record_path is None:
            counts["unresolved_record"] += 1
            continue
        try:
            record_bytes = record_path.read_bytes()
            record = json.loads(record_bytes)
            if not isinstance(record, dict):
                raise ValueError
        except (json.JSONDecodeError, OSError, ValueError):
            counts["invalid_record"] += 1
            continue
        counts["parseable_records"] += 1
        source_category = record.get("category_slug")
        canonical_category = source_to_canonical.get(source_category)
        if canonical_category is None:
            continue
        counts["target_category_records"] += 1
        if "dataset" not in record.get("collections", []):
            counts["target_not_dataset"] += 1
            continue
        rating = record.get("rating")
        if rating not in (4, 5):
            counts["target_not_primary_rating"] += 1
            continue
        if record.get("record_id") != record_id:
            counts["target_identity_mismatch"] += 1
            continue
        artifacts = record.get("artifacts")
        if not isinstance(artifacts, dict):
            counts["target_missing_artifacts"] += 1
            continue
        relative_paths = {
            "record_json": record_relative,
            "dataset_json": Path(record_id) / "collections/dataset.json",
            "model_py": base.safe_record_relative_path(record_id, artifacts.get("model_py")),
            "provenance_json": base.safe_record_relative_path(
                record_id, artifacts.get("provenance_json")
            ),
            "prompt_txt": base.safe_record_relative_path(record_id, artifacts.get("prompt_txt")),
        }
        if any(path is None for path in relative_paths.values()):
            counts["target_unsafe_required_path"] += 1
            continue
        resolved: dict[str, Path] = {"record_json": record_path}
        file_evidence: dict[str, Any] = {"record_json": record_evidence}
        failed = False
        for key in ("dataset_json", "model_py", "provenance_json", "prompt_txt"):
            path, evidence = base.resolve_lfs_overlay_file(
                checkout_root,
                mirror_root,
                relative_paths[key],  # type: ignore[arg-type]
            )
            resolution_counts[str(evidence.get("status"))] += 1
            file_evidence[key] = evidence
            if path is None:
                failed = True
            else:
                resolved[key] = path
        if failed:
            counts["target_unresolved_required_file"] += 1
            continue
        try:
            dataset = read_json(resolved["dataset_json"])
        except (json.JSONDecodeError, OSError, ValueError):
            counts["target_invalid_dataset_sidecar"] += 1
            continue
        if (
            dataset.get("record_id") != record_id
            or dataset.get("category_slug") != source_category
        ):
            counts["target_dataset_sidecar_mismatch"] += 1
            continue
        model_hash = base.sha256_file(resolved["model_py"])
        declared_hashes = record.get("hashes")
        declared_model_hash = (
            declared_hashes.get("model_py_sha256")
            if isinstance(declared_hashes, dict)
            else None
        )
        if declared_model_hash != model_hash:
            counts["target_model_hash_mismatch"] += 1
            continue
        active_revision_id = record.get("active_revision_id")
        if not isinstance(active_revision_id, str) or not active_revision_id:
            counts["target_missing_revision"] += 1
            continue
        candidate = base.Candidate(
            record_id=record_id,
            category_slug=str(source_category),
            rating=int(rating),
            model_path=resolved["model_py"],
            provenance_path=resolved["provenance_json"],
            prompt_path=resolved["prompt_txt"],
            inputs_path=None,
            record_path=record_path,
            dataset_path=resolved["dataset_json"],
            model_sha256=model_hash,
            provenance_sha256=base.sha256_file(resolved["provenance_json"]),
            prompt_sha256=base.sha256_file(resolved["prompt_txt"]),
            record_sha256=sha256_bytes(record_bytes),
            provider=(str(record["provider"]) if record.get("provider") else None),
            model_id=(str(record["model_id"]) if record.get("model_id") else None),
            active_revision_id=active_revision_id,
            selection_digest="",
        )
        pools[canonical_category].append(candidate)
        provenance_by_id[record_id] = {
            "all_required_files_verified": all(
                item.get("verified") is True for item in file_evidence.values()
            ),
            "files": file_evidence,
        }
        counts["eligible_target_records"] += 1

    return pools, {
        "mode": "targeted_current_checkout_with_verified_lfs_overlay",
        "counts": dict(sorted(counts.items())),
        "resolution_counts": dict(sorted(resolution_counts.items())),
        "eligible_pool_counts": {
            category: len(pools[category]) for category in categories
        },
    }, provenance_by_id


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    direct = [row["direct_naming"] for row in rows]
    parts = [int(item["renderable_visual_part_count"]) for item in direct]
    named = [int(item["named_renderable_visual_part_count"]) for item in direct]
    placeholders = [len(item["placeholder_link_names"]) for item in direct]
    total_parts = sum(parts)
    total_named = sum(named)
    return {
        "assets": len(rows),
        "total_renderable_parts": total_parts,
        "total_named_renderable_parts": total_named,
        "parts_per_asset_mean": statistics.fmean(parts),
        "parts_per_asset_median": statistics.median(parts),
        "nameability_micro": total_named / total_parts,
        "nameability_asset_macro": statistics.fmean(
            named_count / part_count for named_count, part_count in zip(named, parts, strict=True)
        ),
        "placeholder_renderable_parts": sum(placeholders),
        "assets_with_placeholder_parts": sum(value > 0 for value in placeholders),
        "fully_nameable_assets": sum(value == 0 for value in placeholders),
        "mesh_only_links": sum(int(item["mesh_only_link_count"]) for item in direct),
        "primitive_only_links": sum(
            int(item["primitive_only_link_count"]) for item in direct
        ),
        "mixed_links": sum(int(item["mixed_link_count"]) for item in direct),
        "invalid_visual_geometries": sum(
            len(item["invalid_visual_geometries"]) for item in direct
        ),
    }


def category_bootstrap(
    rows: list[dict[str, Any]], protocol: dict[str, Any]
) -> dict[str, Any]:
    config = protocol["bootstrap"]
    categories = protocol["design"]["canonical_categories"]
    grouped = {
        category: [row for row in rows if row["canonical_category"] == category]
        for category in categories
    }
    rng = random.Random(int(config["seed"]))
    parts_means: list[float] = []
    micro_scores: list[float] = []
    category_macro_scores: list[float] = []
    for _ in range(int(config["resamples"])):
        sampled_categories = [rng.choice(categories) for _ in categories]
        sampled = [row for category in sampled_categories for row in grouped[category]]
        total_parts = sum(row["direct_naming"]["renderable_visual_part_count"] for row in sampled)
        total_named = sum(
            row["direct_naming"]["named_renderable_visual_part_count"] for row in sampled
        )
        parts_means.append(total_parts / len(sampled))
        micro_scores.append(total_named / total_parts)
        category_macro_scores.append(
            statistics.fmean(aggregate(grouped[category])["nameability_micro"] for category in sampled_categories)
        )
    alpha = (1.0 - float(config["confidence"])) / 2.0
    return {
        "method": "canonical-category cluster bootstrap with replacement",
        "cluster_count": len(categories),
        "assets_per_cluster": protocol["design"]["assets_per_category_per_method"],
        "resamples": config["resamples"],
        "confidence": config["confidence"],
        "seed": config["seed"],
        "parts_per_asset_mean": [
            percentile(parts_means, alpha),
            percentile(parts_means, 1.0 - alpha),
        ],
        "nameability_micro": [
            percentile(micro_scores, alpha),
            percentile(micro_scores, 1.0 - alpha),
        ],
        "nameability_category_macro": [
            percentile(category_macro_scores, alpha),
            percentile(category_macro_scores, 1.0 - alpha),
        ],
    }


def generation_preflight() -> dict[str, Any]:
    readme = require_file(README_PATH).read_text(encoding="utf-8")
    runner = require_file(RUNNER_CLI_PATH).read_text(encoding="utf-8")
    evidence = {
        "readme_requires_provider_keys_for_builtin_generation": (
            "Add API Keys" in readme and "articraft generate" in readme
        ),
        "runner_resolves_model_and_provider": "_resolve_model_and_provider" in runner,
        "runner_validates_provider_credentials": "validate_provider_credentials(provider)" in runner,
        "external_agent_route_documented": "external AI agents" in readme,
    }
    return {
        "status": "BLOCKED_EXTERNAL_MODEL_REQUIRED",
        "specified_common_prompt_generation_without_external_api_or_model": False,
        "reason": (
            "Built-in prompt generation selects an LLM provider and validates provider "
            "credentials. The documented no-key route delegates authoring to an external "
            "AI agent, which is outside this no-new-model baseline."
        ),
        "network_accessed": False,
        "credentials_or_environment_inspected": False,
        "generation_attempted": False,
        "evidence": evidence,
        "source_sha256": {
            "articraft_data/README.md": base.sha256_file(README_PATH),
            "articraft_data/agent/runner_cli.py": base.sha256_file(RUNNER_CLI_PATH),
        },
    }


def report_text(summary: dict[str, Any]) -> str:
    direct = summary["direct_metrics"]
    lines = [
        "# Articraft Naming matched cohort v1",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is a category-matched deterministic official-release resample, not a same-prompt or same-seed comparison.",
        "",
        "## Coverage",
        "",
        f"- Selected/evaluable assets: {summary['coverage']['evaluable_assets']}/{summary['coverage']['expected_assets']}.",
        f"- Assets per category: {summary['coverage']['assets_per_category']}.",
        "",
        "## Direct metrics",
        "",
        f"- Renderable URDF links: {direct['total_renderable_parts']}.",
        f"- Parts per asset: mean {direct['parts_per_asset_mean']:.6f}, median {direct['parts_per_asset_median']}.",
        f"- Nameability: micro {direct['nameability_micro']:.6f}, asset macro {direct['nameability_asset_macro']:.6f}.",
        f"- Placeholder links: {direct['placeholder_renderable_parts']} across {direct['assets_with_placeholder_parts']} assets.",
        "",
        "## Category breakdown",
        "",
        "| category | n | parts mean | nameability micro | placeholders |",
        "|---|---:|---:|---:|---:|",
    ]
    for category, row in summary["per_category"].items():
        lines.append(
            f"| {category} | {row['assets']} | {row['parts_per_asset_mean']:.6f} | "
            f"{row['nameability_micro']:.6f} | {row['placeholder_renderable_parts']} |"
        )
    lines.extend(
        [
            "",
            "## Evidence limits",
            "",
            "- Semantic metrics: N/A; there is no output-independent role gold with three complete blind judges.",
            "- Cross-seed: N/A; official release records are per-asset and expose no frozen reusable seed interface.",
            "- New common-prompt generation: blocked without an external API/model and was not attempted.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--lfs-mirror-records", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()

    protocol_path = require_file(args.protocol)
    output = base.canonical_inside(args.output, REPO_ROOT, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)
    mirror = base.canonical_inside(args.lfs_mirror_records, WORKSPACE_ROOT)
    protocol = read_json(protocol_path)
    observed_hashes = verify_protocol(protocol_path, protocol)
    base_protocol_path = require_file(REPO_ROOT / protocol["base_naming_protocol"]["path"])
    base_protocol = read_json(base_protocol_path)
    placeholder_pattern = str(base_protocol["nameability"]["placeholder_regex"])
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["URDF_COMPILE_TIMEOUT_SECONDS"] = str(float(args.timeout))

    pools, pool_audit, provenance_by_id = audit_target_candidates(protocol, mirror)
    categories = protocol["design"]["canonical_categories"]
    k = int(protocol["design"]["assets_per_category_per_method"])
    selected: list[tuple[str, int, str, base.Candidate]] = []
    selection_audit: dict[str, Any] = {}
    for category in categories:
        ranked = sorted(
            pools[category],
            key=lambda candidate: (
                selection_digest(category, candidate),
                candidate.record_id,
                candidate.model_sha256,
            ),
        )
        if len(ranked) < k:
            raise RuntimeError(f"Articraft pool below K for {category}: {len(ranked)} < {k}")
        chosen = ranked[:k]
        for rank, candidate in enumerate(chosen, start=1):
            selected.append(
                (category, rank, selection_digest(category, candidate), candidate)
            )
        selection_audit[category] = {
            "exact_source_category": protocol["taxonomy"][category]["articraft"],
            "eligible_pool_count": len(ranked),
            "selected_count": len(chosen),
            "selected_record_ids": [candidate.record_id for candidate in chosen],
            "first_excluded_digest": (
                selection_digest(category, ranked[k]) if len(ranked) > k else None
            ),
        }
    expected = int(protocol["design"]["assets_per_method"])
    if len(selected) != expected:
        raise RuntimeError("Matched cohort cardinality gate failed")

    shared_protocol_hash = base.sha256_file(protocol_path)
    script_hash = base.sha256_file(SCRIPT_PATH)
    results: list[dict[str, Any]] = []

    def run_selected(item: tuple[str, int, str, base.Candidate]) -> dict[str, Any]:
        category, rank, digest, candidate = item
        result = base.run_or_reevaluate_case(
            candidate,
            output,
            placeholder_pattern,
            shared_protocol_hash,
            script_hash,
        )
        result.update(
            {
                "canonical_category": category,
                "selection_rank": rank,
                "selection_digest": digest,
            }
        )
        write_json(output / "cases" / candidate.record_id / "result.json", result)
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, int(args.workers))) as pool:
        future_map = {pool.submit(run_selected, item): item for item in selected}
        for completed, future in enumerate(
            concurrent.futures.as_completed(future_map), start=1
        ):
            category, _, _, candidate = future_map[future]
            result = future.result()
            results.append(result)
            print(
                f"[{completed}/{expected}] {category} {candidate.record_id}: {result['status']}",
                flush=True,
            )
    results.sort(key=lambda row: (row["canonical_category"], row["selection_rank"]))
    evaluable = [
        row
        for row in results
        if row.get("status") == "PASS" and row["direct_naming"]["evaluable"] is True
    ]
    status_counts = Counter(str(row.get("status")) for row in results)
    direct = aggregate(evaluable) if evaluable else None
    per_category = {
        category: aggregate(
            [row for row in evaluable if row["canonical_category"] == category]
        )
        for category in categories
        if any(row["canonical_category"] == category for row in evaluable)
    }
    if direct is not None and len(per_category) == len(categories):
        direct["nameability_category_macro"] = statistics.fmean(
            row["nameability_micro"] for row in per_category.values()
        )
    bootstrap = (
        category_bootstrap(results, protocol)
        if len(evaluable) == expected
        else None
    )
    generation = generation_preflight()
    cohort = {
        "schema_version": 1,
        "method": "Articraft",
        "shared_protocol_id": protocol["protocol_id"],
        "shared_protocol_sha256": shared_protocol_hash,
        "comparison_label": protocol["comparison_label"],
        "prohibited_label": protocol["prohibited_label"],
        "selection_policy": protocol["selection"]["articraft"],
        "pool_audit": pool_audit,
        "selection_audit": selection_audit,
        "selected": [
            {
                "canonical_category": category,
                "selection_rank": rank,
                "selection_digest": digest,
                **base.candidate_manifest(candidate),
                "checkout_provenance": provenance_by_id[candidate.record_id],
            }
            for category, rank, digest, candidate in selected
        ],
    }
    cohort_path = output / "cohort_manifest.json"
    write_json(cohort_path, cohort)
    records_text = "".join(
        json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
        for row in results
    )
    (output / "records.jsonl").write_text(records_text, encoding="utf-8")
    summary = {
        "protocol_id": "nano3d_articraft_naming_matched_v1",
        "shared_protocol_id": protocol["protocol_id"],
        "shared_protocol_sha256": shared_protocol_hash,
        "status": "COMPLETE" if len(evaluable) == expected else "INCOMPLETE",
        "comparison_scope": (
            "five-category equal-N deterministic official-release resample; "
            "not same-prompt and not same-seed"
        ),
        "network_accessed": False,
        "generated_code_executed": True,
        "new_model_generation_executed": False,
        "paper_values_reused": False,
        "coverage": {
            "canonical_categories": len(categories),
            "category_names": categories,
            "assets_per_category": k,
            "selected_assets": len(results),
            "evaluable_assets": len(evaluable),
            "expected_assets": expected,
            "status_counts": dict(sorted(status_counts.items())),
        },
        "direct_metrics": direct,
        "per_category": per_category,
        "bootstrap_ci": bootstrap,
        "semantic_metrics": {field: None for field in SEMANTIC_FIELDS},
        "semantic_status": (
            "N/A: no output-independent role gold and no three complete independent blind judges"
        ),
        "cross_seed": {
            "eligible": False,
            "status": "N/A",
            "value": None,
            "reason": "Articraft official release records expose no frozen reusable seed interface",
        },
        "generation_preflight": generation,
        "provenance": {
            "source_hashes": observed_hashes,
            "runner_sha256": script_hash,
            "cohort_manifest_sha256": base.sha256_file(cohort_path),
            "records_sha256": sha256_bytes(records_text.encode("utf-8")),
        },
    }
    summary_path = output / "summary.json"
    write_json(summary_path, summary)
    report_path = output / "report.md"
    report_path.write_text(report_text(summary), encoding="utf-8")

    output_symlinks = 0
    for directory, dirnames, filenames in os.walk(output, followlinks=False):
        parent = Path(directory)
        output_symlinks += sum((parent / name).is_symlink() for name in dirnames)
        output_symlinks += sum((parent / name).is_symlink() for name in filenames)
    category_counts = Counter(row["canonical_category"] for row in results)
    selected_ids = [row["record_id"] for row in results]
    selected_by_id = {
        candidate.record_id: (category, rank, digest, candidate)
        for category, rank, digest, candidate in selected
    }
    source_hash_matches = sum(
        base.sha256_file(candidate.model_path) == candidate.model_sha256
        for _, _, _, candidate in selected
    )
    staged_model_hash_matches = 0
    urdf_hash_matches = 0
    mesh_reference_count = 0
    mesh_reference_hash_matches = 0
    selection_digest_matches = 0
    exact_source_category_matches = 0
    for row in results:
        record_id = str(row["record_id"])
        category, rank, digest, candidate = selected_by_id[record_id]
        package_dir = base.canonical_inside(
            output / "cases" / record_id / "package", REPO_ROOT
        )
        if base.sha256_file(package_dir / "model.py") == candidate.model_sha256:
            staged_model_hash_matches += 1
        urdf_path = base.canonical_inside(package_dir / "model.urdf", REPO_ROOT)
        if base.sha256_file(urdf_path) == row.get("urdf_sha256"):
            urdf_hash_matches += 1
        for mesh_ref in row["direct_naming"]["visual_mesh_references"]:
            mesh_reference_count += 1
            mesh_path = base.safe_mesh_path(package_dir, str(mesh_ref["filename"]))
            if (
                mesh_path is not None
                and mesh_path.is_file()
                and mesh_path.stat().st_size > 0
                and base.sha256_file(mesh_path) == mesh_ref["sha256"]
            ):
                mesh_reference_hash_matches += 1
        if (
            row["selection_rank"] == rank
            and row["selection_digest"] == digest
            and digest == selection_digest(category, candidate)
        ):
            selection_digest_matches += 1
        if candidate.category_slug == protocol["taxonomy"][category]["articraft"]:
            exact_source_category_matches += 1
    checks = {
        "selected_assets_35": len(results) == expected == 35,
        "unique_record_ids_35": len(set(selected_ids)) == expected,
        "five_categories": len(category_counts) == 5,
        "seven_assets_per_category": all(category_counts[category] == k for category in categories),
        "all_pass": all(row.get("status") == "PASS" for row in results),
        "all_evaluable": len(evaluable) == expected,
        "all_required_file_provenance_verified": all(
            provenance_by_id[candidate.record_id]["all_required_files_verified"] is True
            for _, _, _, candidate in selected
        ),
        "live_model_hashes_match": source_hash_matches == expected,
        "staged_model_hashes_match": staged_model_hash_matches == expected,
        "urdf_hashes_match": urdf_hash_matches == expected,
        "mesh_reference_hashes_match": mesh_reference_hash_matches == mesh_reference_count,
        "selection_digests_and_ranks_match": selection_digest_matches == expected,
        "exact_source_categories_match_protocol": (
            exact_source_category_matches == expected
        ),
        "shared_protocol_hash_current": (
            summary["shared_protocol_sha256"] == base.sha256_file(protocol_path)
        ),
        "base_protocol_hash_current": (
            base.sha256_file(base_protocol_path)
            == protocol["base_naming_protocol"]["sha256"]
        ),
        "composition_matches_parts": (
            direct is not None
            and direct["mesh_only_links"]
            + direct["primitive_only_links"]
            + direct["mixed_links"]
            == direct["total_renderable_parts"]
        ),
        "named_plus_placeholder_matches_parts": (
            direct is not None
            and direct["total_named_renderable_parts"]
            + direct["placeholder_renderable_parts"]
            == direct["total_renderable_parts"]
        ),
        "invalid_visual_geometries_zero": (
            direct is not None and direct["invalid_visual_geometries"] == 0
        ),
        "semantic_fields_null": all(value is None for value in summary["semantic_metrics"].values()),
        "cross_seed_na": summary["cross_seed"]["value"] is None,
        "category_matched_not_same_prompt": (
            protocol["comparison_label"] in summary["comparison_scope"]
            or "equal-N" in summary["comparison_scope"]
        ) and "not same-prompt" in summary["comparison_scope"],
        "output_symlink_count_zero": output_symlinks == 0,
    }
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"Matched self-check failed: {', '.join(failed)}")
    artifact_hashes = {
        name: base.sha256_file(output / name)
        for name in ("cohort_manifest.json", "records.jsonl", "summary.json", "report.md")
    }
    self_check = {
        "status": "PASS",
        "checks": checks,
        "counts": {
            "records": len(results),
            "categories": len(category_counts),
            "assets_per_category": dict(sorted(category_counts.items())),
            "renderable_parts": direct["total_renderable_parts"] if direct else None,
            "named_parts": direct["total_named_renderable_parts"] if direct else None,
            "placeholder_parts": direct["placeholder_renderable_parts"] if direct else None,
            "live_model_hash_matches": source_hash_matches,
            "staged_model_hash_matches": staged_model_hash_matches,
            "urdf_hash_matches": urdf_hash_matches,
            "mesh_reference_count": mesh_reference_count,
            "mesh_reference_hash_matches": mesh_reference_hash_matches,
            "selection_digest_matches": selection_digest_matches,
            "exact_source_category_matches": exact_source_category_matches,
            "output_symlink_count": output_symlinks,
        },
        "artifact_sha256": artifact_hashes,
        "reproduction_digest": sha256_bytes(
            json.dumps(artifact_hashes, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ),
    }
    write_json(output / "self_check.json", self_check)
    print(json.dumps({"status": summary["status"], "coverage": summary["coverage"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
