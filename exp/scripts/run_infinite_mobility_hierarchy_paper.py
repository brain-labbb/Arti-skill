#!/usr/bin/env python3
"""Build the paper-facing five-category Infinite Mobility hierarchy cohort.

The generated assets are reused from the frozen 20-factory run. This script
does not invoke Blender; it independently revalidates the selected URDFs and
retains terminal failures in each category's 36-seed denominator.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from math import comb
import os
from pathlib import Path
from statistics import mean
import subprocess
from typing import Any

import run_infinite_mobility_hierarchy as hierarchy


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = (
    EXP_ROOT / "reference/infinite_mobility_hierarchy_common_categories_v1.json"
)
DEFAULT_SOURCE_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
DEFAULT_INPUT = EXP_ROOT / "runtime/infinite_mobility_v1"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/infinite_mobility"
DEFAULT_LOCAL_SOURCE = EXP_ROOT.parent / ".cache/Infinite-Mobility"
DEFAULT_OFFICIAL_SOURCE = EXP_ROOT / "baselines/Infinite-Mobility-official"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_tree_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    files = sorted(
        path for path in root.rglob("*.py") if "__pycache__" not in path.parts
    )
    for path in files:
        resolved = hierarchy.ensure_contained(path, root)
        relative = resolved.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256(resolved)))
    return digest.hexdigest()


def git_output(repo: Path, *arguments: str) -> str:
    env = os.environ.copy()
    env.update(
        {
            "HOME": str(EXP_ROOT),
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_CONFIG_GLOBAL": os.devnull,
        }
    )
    completed = subprocess.run(
        ["git", "-C", str(repo), *arguments],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return completed.stdout.strip()


def provenance_audit(
    protocol: dict[str, Any],
    source_manifest: dict[str, Any],
    local_source: Path,
    official_source: Path,
) -> dict[str, Any]:
    official_commit = git_output(official_source, "rev-parse", "HEAD")
    official_tree = git_output(official_source, "rev-parse", "HEAD^{tree}")
    official_remote = git_output(official_source, "remote", "get-url", "origin")
    official_status = git_output(official_source, "status", "--porcelain")
    official_commit_date = git_output(
        official_source, "show", "-s", "--format=%cI", "HEAD"
    )
    official_subject = git_output(
        official_source, "show", "-s", "--format=%s", "HEAD"
    )
    if official_commit != protocol["official_commit"]:
        raise ValueError("official checkout is not at the frozen commit")
    if official_tree != protocol["official_tree"]:
        raise ValueError("official checkout tree differs from the frozen tree")
    if official_remote != protocol["official_repository"]:
        raise ValueError("official checkout remote differs from the frozen repository")
    if official_status:
        raise ValueError("official checkout is not clean")

    local_python_hash = source_tree_sha256(local_source)
    official_python_hash = source_tree_sha256(official_source)
    runtime_python_hash = source_manifest["baseline_source_tree_sha256"]
    if not local_python_hash == official_python_hash == runtime_python_hash:
        raise ValueError("runtime/local/official Python source trees do not match")

    source_files = {
        "KitchenCabinetFactory": "infinigen/assets/objects/shelves/kitchen_cabinet.py",
        "TableDiningFactory": "infinigen/assets/objects/tables/dining_table.py",
        "BeverageFridgeFactory": "infinigen/assets/objects/appliances/beverage_fridge.py",
        "DishwasherFactory": "infinigen/assets/objects/appliances/dishwasher.py",
        "OvenFactory": "infinigen/assets/objects/appliances/oven.py",
    }
    selected_source_files: dict[str, Any] = {}
    for factory, relative in source_files.items():
        local_file = hierarchy.ensure_contained(local_source / relative, local_source)
        official_file = hierarchy.ensure_contained(
            official_source / relative, official_source
        )
        local_hash = sha256(local_file)
        official_hash = sha256(official_file)
        if local_hash != official_hash:
            raise ValueError(f"factory source mismatch: {factory}")
        selected_source_files[factory] = {
            "path": relative,
            "sha256": local_hash,
            "matches_official": True,
        }

    local_license = hierarchy.ensure_contained(local_source / "LICENSE", local_source)
    official_license = hierarchy.ensure_contained(
        official_source / "LICENSE", official_source
    )
    local_license_hash = sha256(local_license)
    official_license_hash = sha256(official_license)
    if local_license_hash != official_license_hash:
        raise ValueError("local and official LICENSE files differ")
    license_text = official_license.read_text(encoding="utf-8")
    if not license_text.startswith("BSD 3-Clause License"):
        raise ValueError("official LICENSE is not BSD 3-Clause")

    agents_files = [
        path.relative_to(official_source).as_posix()
        for path in official_source.rglob("AGENTS.md")
        if hierarchy.ensure_contained(path, official_source)
    ]
    return {
        "status": "PASS",
        "official_repository": official_remote,
        "official_commit": official_commit,
        "official_tree": official_tree,
        "official_commit_date": official_commit_date,
        "official_commit_subject": official_subject,
        "official_checkout_clean": True,
        "official_license": protocol["official_license"],
        "license_sha256": official_license_hash,
        "official_agents_files": agents_files,
        "local_snapshot_has_git_metadata": (local_source / ".git").exists(),
        "runtime_python_source_tree_sha256": runtime_python_hash,
        "local_python_source_tree_sha256": local_python_hash,
        "official_python_source_tree_sha256": official_python_hash,
        "python_source_tree_matches_official_commit": True,
        "python_file_count": sum(
            1
            for path in official_source.rglob("*.py")
            if "__pycache__" not in path.parts
        ),
        "selected_factory_source_files": selected_source_files,
        "scope_note": (
            "The full Python source tree and selected factory files match the "
            "official commit. The cache lacks Git metadata, so commit provenance "
            "is established through the pinned clean checkout plus content hashes."
        ),
    }


def signature_stats(values: list[object]) -> dict[str, Any]:
    if not values:
        return {
            "sample_count": 0,
            "mode_count": 0,
            "mode_rate": None,
            "pair_count": 0,
            "pair_exact_count": 0,
            "pairwise_exact_rate": None,
        }
    counts = Counter(values)
    mode_count = counts.most_common(1)[0][1]
    pair_count = comb(len(values), 2)
    exact_count = sum(comb(count, 2) for count in counts.values())
    return {
        "sample_count": len(values),
        "mode_count": mode_count,
        "mode_rate": mode_count / len(values),
        "pair_count": pair_count,
        "pair_exact_count": exact_count,
        "pairwise_exact_rate": exact_count / pair_count if pair_count else None,
    }


def category_metrics(
    category: dict[str, Any], rows: list[dict[str, Any]]
) -> dict[str, Any]:
    requested = sorted(rows, key=lambda row: int(row["seed"]))
    generated = [row for row in requested if row["evaluation_status"] == "PASS"]
    valid = [row for row in generated if row["valid_tree"]]
    raw_signatures = [
        tuple(tuple(edge) for edge in row["edge_signature"]) for row in valid
    ]
    canonical_signatures = [row["canonical_shape_signature"] for row in valid]
    raw = signature_stats(raw_signatures)
    canonical = signature_stats(canonical_signatures)
    requested_count = len(requested)
    generated_count = len(generated)
    valid_tree_count = len(valid)
    has_hierarchy_count = sum(bool(row["has_hierarchy"]) for row in requested)
    return {
        **category,
        "requested_count": requested_count,
        "generated_asset_count": generated_count,
        "missing_asset_count": requested_count - generated_count,
        "generated_asset_rate": generated_count / requested_count,
        "valid_tree_count": valid_tree_count,
        "valid_tree_rate_requested": valid_tree_count / requested_count,
        "valid_tree_rate_generated": valid_tree_count / generated_count,
        "has_hierarchy_count": has_hierarchy_count,
        "has_hierarchy_rate_requested": has_hierarchy_count / requested_count,
        "has_hierarchy_rate_generated": has_hierarchy_count / generated_count,
        "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in valid),
        "semantic_depth_min": min(int(row["semantic_depth"]) for row in valid),
        "semantic_depth_max": max(int(row["semantic_depth"]) for row in valid),
        "named_groups_mean": mean(
            int(row["named_group_count"]) for row in valid
        ),
        "named_groups_total": sum(int(row["named_group_count"]) for row in valid),
        "pivots_mean": mean(int(row["pivot_count"]) for row in valid),
        "pivots_total": sum(int(row["pivot_count"]) for row in valid),
        "raw_edge_signature": raw,
        "canonical_shape_signature": canonical,
        "missing_seeds": [
            int(row["seed"])
            for row in requested
            if row["evaluation_status"] != "PASS"
        ],
    }


def micro_signature(category_rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    sample_count = sum(int(row[field]["sample_count"]) for row in category_rows)
    mode_count = sum(int(row[field]["mode_count"]) for row in category_rows)
    pair_count = sum(int(row[field]["pair_count"]) for row in category_rows)
    exact_count = sum(int(row[field]["pair_exact_count"]) for row in category_rows)
    return {
        "sample_count": sample_count,
        "within_category_mode_count": mode_count,
        "within_category_mode_rate": mode_count / sample_count,
        "within_category_pair_count": pair_count,
        "within_category_pair_exact_count": exact_count,
        "within_category_pairwise_exact_rate": exact_count / pair_count,
    }


def summarize(
    protocol: dict[str, Any],
    evaluated: list[dict[str, Any]],
    categories: list[dict[str, Any]],
    evaluation_unit: str,
) -> dict[str, Any]:
    valid = [row for row in evaluated if row["evaluation_status"] == "PASS"]
    valid_trees = [row for row in valid if row["valid_tree"]]
    requested_count = len(evaluated)
    generated_count = len(valid)
    valid_tree_count = len(valid_trees)
    has_hierarchy_count = sum(bool(row["has_hierarchy"]) for row in evaluated)
    macro = {
        "category_count": len(categories),
        "generated_asset_rate": mean(
            float(row["generated_asset_rate"]) for row in categories
        ),
        "valid_tree_rate_requested": mean(
            float(row["valid_tree_rate_requested"]) for row in categories
        ),
        "valid_tree_rate_generated": mean(
            float(row["valid_tree_rate_generated"]) for row in categories
        ),
        "has_hierarchy_rate_requested": mean(
            float(row["has_hierarchy_rate_requested"]) for row in categories
        ),
        "has_hierarchy_rate_generated": mean(
            float(row["has_hierarchy_rate_generated"]) for row in categories
        ),
        "semantic_depth_mean": mean(
            float(row["semantic_depth_mean"]) for row in categories
        ),
        "named_groups_mean": mean(
            float(row["named_groups_mean"]) for row in categories
        ),
        "pivots_mean": mean(float(row["pivots_mean"]) for row in categories),
        "raw_edge_signature_mode_rate": mean(
            float(row["raw_edge_signature"]["mode_rate"]) for row in categories
        ),
        "pairwise_raw_edge_exact_rate": mean(
            float(row["raw_edge_signature"]["pairwise_exact_rate"])
            for row in categories
        ),
        "canonical_shape_signature_mode_rate": mean(
            float(row["canonical_shape_signature"]["mode_rate"])
            for row in categories
        ),
        "pairwise_canonical_shape_exact_rate": mean(
            float(row["canonical_shape_signature"]["pairwise_exact_rate"])
            for row in categories
        ),
    }
    micro = {
        "requested_count": requested_count,
        "generated_asset_count": generated_count,
        "missing_asset_count": requested_count - generated_count,
        "generated_asset_rate": generated_count / requested_count,
        "valid_tree_count": valid_tree_count,
        "valid_tree_rate_requested": valid_tree_count / requested_count,
        "valid_tree_rate_generated": valid_tree_count / generated_count,
        "has_hierarchy_count": has_hierarchy_count,
        "has_hierarchy_rate_requested": has_hierarchy_count / requested_count,
        "has_hierarchy_rate_generated": has_hierarchy_count / generated_count,
        "semantic_depth_mean": mean(
            int(row["semantic_depth"]) for row in valid_trees
        ),
        "semantic_depth_min": min(int(row["semantic_depth"]) for row in valid_trees),
        "semantic_depth_max": max(int(row["semantic_depth"]) for row in valid_trees),
        "named_groups_mean": mean(
            int(row["named_group_count"]) for row in valid_trees
        ),
        "named_groups_total": sum(
            int(row["named_group_count"]) for row in valid_trees
        ),
        "pivots_mean": mean(int(row["pivot_count"]) for row in valid_trees),
        "pivots_total": sum(int(row["pivot_count"]) for row in valid_trees),
        "raw_edge_signature": micro_signature(categories, "raw_edge_signature"),
        "canonical_shape_signature": micro_signature(
            categories, "canonical_shape_signature"
        ),
    }
    return {
        "protocol_id": protocol["protocol_id"],
        "generated_at": utc_now(),
        "evaluation_unit": evaluation_unit,
        "denominator_policy": protocol["denominator_policy"],
        "hierarchy_mapping": protocol["hierarchy_mapping"],
        "per_category": categories,
        "macro": macro,
        "micro": micro,
        "unsupported_by_protocol": {
            "parent_child_edge_f1": "N/A: no independent hierarchy gold edge set",
            "hierarchy_exact_match": "N/A: no independent hierarchy gold tree",
            "semantic_nesting_accuracy": "N/A: no independent semantic nesting gold",
        },
        "cross_seed_claim": protocol["cross_seed_scope"],
    }


def render_report(
    summary: dict[str, Any], provenance: dict[str, Any]
) -> str:
    main = summary["paper_main"]
    main_micro = main["micro"]
    main_macro = main["macro"]
    rows = [
        "# Infinite Mobility common-category Table 3 Hierarchy",
        "",
        "## Provenance",
        "",
        f"- Official commit: `{provenance['official_commit']}`",
        f"- Official tree: `{provenance['official_tree']}`",
        f"- License: `{provenance['official_license']}`",
        (
            "- Runtime/local/official Python source-tree SHA-256: "
            f"`{provenance['runtime_python_source_tree_sha256']}` (exact match)"
        ),
        "- Official checkout clean: yes; official `AGENTS.md` files: none",
        "",
        "## Paper main (5 categories x 6 seeds = 30)",
        "",
        (
            "| Common category | Factory | Valid Tree | Has Hierarchy | "
            "Depth | Groups | Pivots | Raw mode | Raw pair | Canonical mode | "
            "Canonical pair |"
        ),
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in main["per_category"]:
        rows.append(
            "| {common_category} | `{factory}` | {valid_tree_count}/6 | "
            "{has_hierarchy_count}/6 | {semantic_depth_mean:.3f} | "
            "{named_groups_mean:.3f} | {pivots_mean:.3f} | {raw_mode:.3f} | "
            "{raw_pair:.3f} | {canonical_mode:.3f} | {canonical_pair:.3f} |".format(
                **item,
                raw_mode=item["raw_edge_signature"]["mode_rate"],
                raw_pair=item["raw_edge_signature"]["pairwise_exact_rate"],
                canonical_mode=item["canonical_shape_signature"]["mode_rate"],
                canonical_pair=item["canonical_shape_signature"][
                    "pairwise_exact_rate"
                ],
            )
        )
    rows.extend(
        [
            "",
            (
                f"- Main30 micro: Valid Tree {main_micro['valid_tree_count']}/30 "
                f"= {main_micro['valid_tree_rate_requested']:.3%}; Has Hierarchy "
                f"{main_micro['has_hierarchy_count']}/30 = "
                f"{main_micro['has_hierarchy_rate_requested']:.3%}; Depth "
                f"{main_micro['semantic_depth_mean']:.3f}; Groups "
                f"{main_micro['named_groups_mean']:.3f}; Pivots "
                f"{main_micro['pivots_mean']:.3f}"
            ),
            (
                f"- Main30 Cross-Seed macro: raw mode "
                f"{main_macro['raw_edge_signature_mode_rate']:.3f}, raw pair "
                f"{main_macro['pairwise_raw_edge_exact_rate']:.3f}, canonical mode "
                f"{main_macro['canonical_shape_signature_mode_rate']:.3f}, "
                f"canonical pair {main_macro['pairwise_canonical_shape_exact_rate']:.3f}"
            ),
            "",
            "## Full 36-seed supplementary results",
            "",
            (
                "| Common category | Factory | Generated | Valid Tree | Has Hierarchy | "
                "Depth | Groups | Pivots | Raw mode | Raw pair | Canonical mode | "
                "Canonical pair |"
            ),
            "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for item in summary["per_category"]:
        rows.append(
            "| {common_category} | `{factory}` | {generated_asset_count}/36 | "
            "{valid_tree_count}/36 | {has_hierarchy_count}/36 | "
            "{semantic_depth_mean:.3f} | {named_groups_mean:.3f} | "
            "{pivots_mean:.3f} | {raw_mode:.3f} | {raw_pair:.3f} | "
            "{canonical_mode:.3f} | {canonical_pair:.3f} |".format(
                **item,
                raw_mode=item["raw_edge_signature"]["mode_rate"],
                raw_pair=item["raw_edge_signature"]["pairwise_exact_rate"],
                canonical_mode=item["canonical_shape_signature"]["mode_rate"],
                canonical_pair=item["canonical_shape_signature"][
                    "pairwise_exact_rate"
                ],
            )
        )
    macro = summary["macro"]
    micro = summary["micro"]
    rows.extend(
        [
            "",
            "## Aggregate",
            "",
            (
                f"- Micro: Generated {micro['generated_asset_count']}/"
                f"{micro['requested_count']} = {micro['generated_asset_rate']:.3%}; "
                f"Valid Tree {micro['valid_tree_count']}/{micro['requested_count']} "
                f"= {micro['valid_tree_rate_requested']:.3%}; Has Hierarchy "
                f"{micro['has_hierarchy_count']}/{micro['requested_count']} = "
                f"{micro['has_hierarchy_rate_requested']:.3%}"
            ),
            (
                f"- Micro structure: Depth {micro['semantic_depth_mean']:.3f} "
                f"[{micro['semantic_depth_min']}, {micro['semantic_depth_max']}]; "
                f"Groups {micro['named_groups_mean']:.3f} "
                f"({micro['named_groups_total']} total); Pivots "
                f"{micro['pivots_mean']:.3f} ({micro['pivots_total']} total)"
            ),
            (
                f"- Macro rates: Valid Tree {macro['valid_tree_rate_requested']:.3%}; "
                f"Has Hierarchy {macro['has_hierarchy_rate_requested']:.3%}; "
                f"Depth {macro['semantic_depth_mean']:.3f}; Groups "
                f"{macro['named_groups_mean']:.3f}; Pivots {macro['pivots_mean']:.3f}"
            ),
            (
                f"- Cross-Seed macro: raw mode "
                f"{macro['raw_edge_signature_mode_rate']:.3f}, raw pair "
                f"{macro['pairwise_raw_edge_exact_rate']:.3f}, canonical mode "
                f"{macro['canonical_shape_signature_mode_rate']:.3f}, canonical pair "
                f"{macro['pairwise_canonical_shape_exact_rate']:.3f}"
            ),
            (
                f"- Cross-Seed micro within-category: raw mode "
                f"{micro['raw_edge_signature']['within_category_mode_rate']:.3f}, "
                f"raw pair {micro['raw_edge_signature']['within_category_pairwise_exact_rate']:.3f}, "
                f"canonical mode {micro['canonical_shape_signature']['within_category_mode_rate']:.3f}, "
                f"canonical pair {micro['canonical_shape_signature']['within_category_pairwise_exact_rate']:.3f}"
            ),
            "- Parent-Child Edge F1 / Exact Match / Semantic Nesting: N/A (no independent gold)",
            "",
            "Cross-seed values measure topology stability only, not semantic correctness.",
        ]
    )
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--source-protocol", type=Path, default=DEFAULT_SOURCE_PROTOCOL)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--local-source", type=Path, default=DEFAULT_LOCAL_SOURCE)
    parser.add_argument("--official-source", type=Path, default=DEFAULT_OFFICIAL_SOURCE)
    args = parser.parse_args()

    workspace_root = EXP_ROOT.parent.resolve(strict=True)
    protocol_path = hierarchy.ensure_contained(args.protocol, workspace_root)
    source_protocol_path = hierarchy.ensure_contained(
        args.source_protocol, workspace_root
    )
    input_root = hierarchy.ensure_contained(args.input_root, workspace_root)
    output_root = hierarchy.ensure_contained(
        args.output_root, workspace_root, must_exist=False
    )
    local_source = hierarchy.ensure_contained(args.local_source, workspace_root)
    official_source = hierarchy.ensure_contained(args.official_source, workspace_root)

    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    source_protocol, source_manifest, records = hierarchy.load_and_validate_inputs(
        source_protocol_path, input_root
    )
    if source_protocol["protocol_id"] != protocol["source_seed_protocol"]:
        raise ValueError("common-category protocol references the wrong source protocol")
    if protocol["seeds"] != source_protocol["seeds"]:
        raise ValueError("common-category seeds differ from the source seed protocol")
    source_factories = set(source_protocol["factories"])
    selected_factories = [item["factory"] for item in protocol["categories"]]
    if len(set(selected_factories)) != len(selected_factories):
        raise ValueError("common-category factory selection contains duplicates")
    if not set(selected_factories).issubset(source_factories):
        raise ValueError("common-category factory is absent from source protocol")

    record_map = {
        (str(record["factory"]), int(record["seed"])): record for record in records
    }
    evaluated: list[dict[str, Any]] = []
    for category in protocol["categories"]:
        factory = category["factory"]
        for seed in protocol["seeds"]:
            record = record_map[(factory, int(seed))]
            row = hierarchy.evaluate_case(input_root, record)
            row.update(
                {
                    "category_id": category["category_id"],
                    "common_category": category["common_category"],
                }
            )
            evaluated.append(row)
    if len(evaluated) != 180:
        raise ValueError(f"expected 180 selected records, found {len(evaluated)}")

    category_rows = [
        category_metrics(
            category,
            [row for row in evaluated if row["category_id"] == category["category_id"]],
        )
        for category in protocol["categories"]
    ]
    provenance = provenance_audit(
        protocol, source_manifest, local_source, official_source
    )
    summary = summarize(
        protocol,
        evaluated,
        category_rows,
        "5 common categories x 36 frozen seeds = 180 supplementary cases",
    )
    paper_main_seeds = {int(seed) for seed in protocol["paper_main_seeds"]}
    if paper_main_seeds != set(protocol["seeds"][:6]):
        raise ValueError("paper main seeds must be the frozen seed-list prefix")
    main_evaluated = [
        row for row in evaluated if int(row["seed"]) in paper_main_seeds
    ]
    main_category_rows = [
        category_metrics(
            category,
            [
                row
                for row in main_evaluated
                if row["category_id"] == category["category_id"]
            ],
        )
        for category in protocol["categories"]
    ]
    paper_main = summarize(
        protocol,
        main_evaluated,
        main_category_rows,
        "5 common categories x 6 frozen seeds = 30 paper-main cases",
    )
    paper_main.pop("generated_at")
    summary["paper_main"] = paper_main
    summary["full_36_seed_supplement"] = {
        "requested_count": summary["micro"]["requested_count"],
        "generated_asset_count": summary["micro"]["generated_asset_count"],
        "valid_tree_count": summary["micro"]["valid_tree_count"],
        "has_hierarchy_count": summary["micro"]["has_hierarchy_count"],
    }
    cohort_manifest = {
        "protocol": protocol,
        "protocol_sha256": sha256(protocol_path),
        "source_protocol_sha256": sha256(source_protocol_path),
        "source_runtime_manifest_sha256": sha256(input_root / "manifest.json"),
        "source_runtime_records_sha256": sha256(input_root / "records.json"),
        "evaluator_sha256": sha256(Path(__file__).resolve()),
        "provenance": provenance,
        "selection_count": len(evaluated),
        "selection": [
            {
                "category_id": row["category_id"],
                "common_category": row["common_category"],
                "factory": row["factory"],
                "seed": row["seed"],
                "baseline_status": row["baseline_status"],
                "case_dir": row["case_dir"],
                "urdf_path": row.get("urdf_path"),
                "urdf_sha256": row.get("urdf_sha256"),
                "recorded_package_sha256": row.get("recorded_package_sha256"),
                "paper_main": int(row["seed"]) in paper_main_seeds,
            }
            for row in evaluated
        ],
    }

    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Any] = {
        "provenance.json": provenance,
        "cohort_manifest.json": cohort_manifest,
        "case_records.json": evaluated,
        "category_metrics.json": category_rows,
        "main_category_metrics.json": main_category_rows,
        "summary.json": summary,
    }
    for name, payload in outputs.items():
        (output_root / name).write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    (output_root / "report.md").write_text(
        render_report(summary, provenance), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"outputs={output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
