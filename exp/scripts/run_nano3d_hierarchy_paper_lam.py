#!/usr/bin/env python3
"""Evaluate official LAM release rows with Nano3D's shared URDF tree mapping."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from statistics import mean
from typing import Any
import xml.etree.ElementTree as ET

import pyarrow.parquet as pq

from run_nano3d_hierarchy import parse_hierarchy


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPO = EXP_ROOT / "baselines/LAM-official"
DEFAULT_DATASET = EXP_ROOT / "baselines/LAM-official-dataset"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/lam"
DATASET_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
REPOSITORY_COMMIT = "0b3a87beb8c35273a5acf8681221791aff746d8e"
SELECTION_SALT = "nano3d-hierarchy-lam-official-v1"
REPOSITORY_LICENSE_SHA256 = "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4"
DATASET_FILE_SHA256 = {
    "README.md": "85b28c2338d0d64bf336cd032faca192df493762de8477af125844410e307eb3",
    "manifest.csv": "70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412",
    "manifest.parquet": "589a75c3bf63cb20e22596a7786b06215eb44d44177d1620289cf397ec43c0c1",
    "articulated_code.parquet": "ab45cf9154c5d98deef7b9032f622286efecd21228d9d0e9165a2f0811da6764",
}

# These are an explicit, result-independent mapping from release category slugs.
# They were frozen before reading tier/status or running the hierarchy evaluator.
CATEGORY_ALLOWLISTS: dict[str, tuple[str, ...]] = {
    "storage_furniture_cabinet": (
        "cabinet_with_a_drawer",
        "cabinet_with_five_drawers",
        "cabinet_with_three_drawers",
        "cabinet_with_two_drawers",
        "cabinet_with_two_drawers_two_door",
        "drawer_with_five_cabinets",
        "drawer_with_three_cabinets",
        "illuminated_medicine_cabinet_with_integrated",
        "illuminated_medicine_cabinet_with_integrated_led_l",
        "lateral_filing_cabinet_with_hanging",
        "lateral_filing_cabinet_with_hanging_folder_rails_a",
        "premium_storage_furniture_with_solid",
        "premium_storage_furniture_with_solid_wood_construc",
        "rectangular_wooden_cabinet_featuring",
        "stainless_steel_medicine_cabinet",
        "stainless_steel_medicine_cabinet_with_magnetic_d",
        "step1_a_cabinet_with_a_drawer",
        "step4_a_cabinet_with_five_drawers",
        "storage_cabinet_with_multiple",
        "storage_cabinet_with_multiple_compartments_and_a",
        "storage_furniture",
        "tall_wooden_cabinet_featuring",
        "tall_wooden_cabinet_featuring_two_hinged_doors_w",
        "vertical_filing_cabinet_with",
        "vertical_filing_cabinet_with_full_extension_slid",
    ),
    "table": (
        "drafting_table_with_adjustable_tilt_surface",
        "gate_leg_table_has",
        "gate_leg_table_has_a_fixed_top_center_section_an",
        "solid_wood_dining_table",
        "solid_wood_dining_table_with_a_rectangular_top_s",
        "table_with_some_cabinets",
        "table_with_some_cabinets_or_doors",
        "wooden_dining_table",
    ),
    "refrigerator": (
        "energy_efficient_refrigerator_with_adjustable_shel",
        "energy_efficient_refrigerator_with_adjustable_shelving",
        "stainless_steel_refrigerator_with",
        "stainless_steel_refrigerator_with_water_dispense",
    ),
    "dishwasher": (
        "built_in_dishwasher_with_sliding_racks",
        "dishwasher",
        "energy_efficient_dishwasher_featuring_adjustable_upper",
        "front_loading_dishwasher_with_articulated",
        "premium_dishwasher_with_third_level_rack",
        "premium_dishwasher_with_third_level_rack_system_a",
        "quiet_operation_dishwasher_with_height_adjustable",
        "stainless_steel_dishwasher_with_fold_down",
    ),
    "oven": (
        "gas_oven_with_broiler",
        "gas_oven_with_broiler_and_multiple_rack_position",
        "oven",
        "professional_oven_with_steam",
        "professional_oven_with_steam_injection_and_preci",
    ),
}

EXCLUDED_FALSE_POSITIVES = {
    "storage_furniture_cabinet": (
        "cabinet_door",
        "cabinet_soft_close_hinge",
        "four_wheeled_golf_cart_with_bag_storage_compartm",
        "similar_to_a_house_door_but_smaller_a_cabinet_doo",
        "storage",
    ),
    "table": ("scroll_saw_with_reciprocating_blade_and_table",),
    "oven": (
        "advanced_microwave_oven_with_inverter",
        "convection_toaster_oven_with_multiple",
        "convection_toaster_oven_with_multiple_cooking_func",
    ),
}

PROMPT_SOURCES = {
    "storage_furniture_cabinet": ("storage_furniture.txt", "cabinet.txt"),
    "table": ("table.txt",),
    "refrigerator": ("refrigerator.txt",),
    "dishwasher": ("dishwasher.txt",),
    "oven": ("oven.txt",),
}


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def count(values: list[Any]) -> dict[str, int]:
    return dict(sorted(Counter("<null>" if value is None else str(value) for value in values).items()))


def category_index() -> dict[str, str]:
    result: dict[str, str] = {}
    for shared_class, categories in CATEGORY_ALLOWLISTS.items():
        for category in categories:
            if category in result:
                raise ValueError(f"category appears in multiple allowlists: {category}")
            result[category] = shared_class
    return result


def json_field_state(value: Any) -> tuple[bool, str | None, str | None]:
    if not isinstance(value, str) or not value.strip():
        return False, None, "missing"
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError) as exc:
        return True, None, f"{type(exc).__name__}: {exc}"
    return True, type(parsed).__name__, None


def urdf_field_state(value: Any) -> tuple[bool, str | None]:
    if not isinstance(value, str) or not value.strip():
        return False, "missing"
    try:
        ET.fromstring(value)
    except ET.ParseError as exc:
        return True, f"ParseError: {exc}"
    return True, None


def audit_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    urdf_present = 0
    urdf_parseable = 0
    links_present = 0
    links_parseable = 0
    links_types: Counter[str] = Counter()
    for row in rows:
        present, error = urdf_field_state(row.get("urdf"))
        urdf_present += int(present)
        urdf_parseable += int(present and error is None)
        present, parsed_type, error = json_field_state(row.get("links_hierarchy_json"))
        links_present += int(present)
        links_parseable += int(present and error is None)
        if parsed_type:
            links_types[parsed_type] += 1
    return {
        "row_count": len(rows),
        "tier_counts": count([row.get("tier") for row in rows]),
        "status_counts": count([row.get("status") for row in rows]),
        "urdf_present_count": urdf_present,
        "urdf_xml_parseable_count": urdf_parseable,
        "links_hierarchy_json_present_count": links_present,
        "links_hierarchy_json_parseable_count": links_parseable,
        "links_hierarchy_json_types": dict(sorted(links_types.items())),
    }


def rank_key(shared_class: str, row: dict[str, Any]) -> str:
    payload = "\n".join(
        (
            SELECTION_SALT,
            shared_class,
            str(row["object_release_id"]),
            str(row["category"]),
            str(row["dataset_row_index"]),
        )
    )
    return sha256_bytes(payload.encode("utf-8"))


def safe_id(value: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value)
    if not sanitized or sanitized in {".", ".."}:
        raise ValueError(f"unsafe object_release_id: {value!r}")
    return sanitized


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def aggregate_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    complete = [row for row in records if row["evaluator_complete"]]
    total_n = len(records)
    if not complete or not total_n:
        raise ValueError("cannot aggregate an empty or evaluator-incomplete cohort")
    return {
        "selected_count": total_n,
        "evaluator_complete_count": len(complete),
        "valid_tree_count": sum(bool(row.get("valid_tree")) for row in records),
        "valid_tree_rate": sum(bool(row.get("valid_tree")) for row in records) / total_n,
        "has_hierarchy_count": sum(bool(row.get("has_tree")) for row in records),
        "has_hierarchy_rate": sum(bool(row.get("has_tree")) for row in records) / total_n,
        "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in complete),
        "semantic_depth_min": min(int(row["semantic_depth"]) for row in complete),
        "semantic_depth_max": max(int(row["semantic_depth"]) for row in complete),
        "named_groups_mean": mean(int(row["named_group_count"]) for row in complete),
        "named_groups_total": sum(int(row["named_group_count"]) for row in complete),
        "pivots_mean": mean(int(row["pivot_count"]) for row in complete),
        "pivots_total": sum(int(row["pivot_count"]) for row in complete),
    }


def render_report(summary: dict[str, Any], audit: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# Official LAM Table 3 Hierarchy Evaluation",
        "",
        f"- Status: **{summary['status']}**",
        f"- Dataset revision: `{summary['dataset']['revision']}`",
        f"- Selection: `{summary['selection']['rule']}`",
        f"- Shared classes: `{summary['selection']['class_count']}`",
        f"- Per-class N: `{summary['selection']['per_class_n']}`",
        f"- Total N: `{summary['selected_count']}`",
        f"- Evaluator-complete N: `{summary['evaluator_complete_count']}`",
        "",
        "## Table 3 results",
        "",
        f"- Valid Tree: {metrics['valid_tree_count']}/{summary['selected_count']} = {metrics['valid_tree_rate']:.6f}",
        f"- Has Hierarchy: {metrics['has_hierarchy_count']}/{summary['selected_count']} = {metrics['has_hierarchy_rate']:.6f}",
        f"- Semantic Depth: {metrics['semantic_depth_mean']:.6f} mean [{metrics['semantic_depth_min']}, {metrics['semantic_depth_max']}] over evaluator-complete rows",
        f"- Named Groups: {metrics['named_groups_mean']:.6f} mean; {metrics['named_groups_total']} total",
        f"- Pivots: {metrics['pivots_mean']:.6f} mean; {metrics['pivots_total']} total",
        "- Parent-Child Edge F1: N/A (no independent hierarchy gold)",
        "- Hierarchy Exact Match: N/A (no independent hierarchy gold)",
        "- Semantic Nesting Accuracy: N/A (no independent hierarchy gold)",
        "- Cross-Seed Consistency: N/A (LAM release rows are per-asset outputs)",
        "",
        "Invalid or unparsable selected assets remain in the binary-rate denominator. Structural means are reported over rows on which the shared evaluator completed.",
        "",
        "## Per-class metrics",
        "",
    ]
    for shared_class, item in sorted(summary["per_class_metrics"].items()):
        lines.append(
            f"- {shared_class}: tree {item['valid_tree_count']}/{item['selected_count']}; hierarchy {item['has_hierarchy_count']}/{item['selected_count']}; depth {item['semantic_depth_mean']:.3f}; groups {item['named_groups_mean']:.3f}; pivots {item['pivots_mean']:.3f}"
        )
    lines.extend(
        [
            "",
        "## Candidate audit",
        "",
        ]
    )
    for shared_class in sorted(audit["classes"]):
        item = audit["classes"][shared_class]
        lines.append(
            f"- {shared_class}: {item['row_count']} rows; tiers={json.dumps(item['tier_counts'], sort_keys=True)}; statuses={json.dumps(item['status_counts'], sort_keys=True)}; URDF XML={item['urdf_xml_parseable_count']}/{item['row_count']}; links JSON={item['links_hierarchy_json_parseable_count']}/{item['row_count']}"
        )
    lines.extend(
        [
            "",
            "## Selection policy",
            "",
            "The category allowlists and exclusions are frozen in the evaluator. Selection consumes only shared class, raw category, release ID, and immutable dataset row index. Each class contributes the global minimum candidate count, ranked by SHA-256 of those identity fields plus a fixed salt. No tier/status or metric field participates.",
            "",
            "The mapping is explicit in `selection_protocol.json`; cabinet doors/hinges, golf-cart storage, scroll-saw tables, microwave ovens, and toaster ovens are excluded as different object classes or components.",
            "",
            "## Scope",
            "",
            "Metrics reuse `run_nano3d_hierarchy.parse_hierarchy`, the documented URDF-equivalent scene-tree mapping. They do not reproduce a native GLB evaluator and do not establish semantic nesting correctness.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workspace-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    workspace = args.workspace_root.resolve()
    repo = args.repo.resolve()
    dataset = args.dataset.resolve()
    output = args.output.resolve(strict=False)
    for label, path in (("repo", repo), ("dataset", dataset), ("output", output)):
        try:
            path.relative_to(workspace)
        except ValueError as exc:
            raise ValueError(f"{label} is outside workspace: {path}") from exc

    parquet_path = (dataset / "articulated_code.parquet").resolve()
    readme_path = (dataset / "README.md").resolve()
    api_path = (dataset / "dataset_api.json").resolve()
    license_path = (repo / "LICENSE").resolve()
    for path in (parquet_path, readme_path, api_path, license_path):
        path.relative_to(workspace)
        if not path.is_file():
            raise FileNotFoundError(path)

    repository_head = (repo / ".git/HEAD").read_text(encoding="utf-8").strip()
    if repository_head != REPOSITORY_COMMIT:
        raise ValueError(f"LAM checkout is not pinned to {REPOSITORY_COMMIT}: {repository_head}")
    if sha256_file(license_path) != REPOSITORY_LICENSE_SHA256:
        raise ValueError("LAM repository LICENSE hash mismatch")
    for filename, expected_hash in DATASET_FILE_SHA256.items():
        actual_hash = sha256_file(dataset / filename)
        if actual_hash != expected_hash:
            raise ValueError(f"official dataset file hash mismatch for {filename}: {actual_hash}")

    output.mkdir(parents=True, exist_ok=True)
    urdf_dir = output / "selected_urdfs"
    urdf_dir.mkdir(parents=True, exist_ok=True)

    table = pq.read_table(parquet_path)
    rows = table.to_pylist()
    if len(rows) != 3217:
        raise ValueError(f"expected official 3217-row release, found {len(rows)}")
    release_id_counts = Counter(str(row["object_release_id"]) for row in rows)
    duplicate_release_ids = {
        key: value for key, value in sorted(release_id_counts.items()) if value > 1
    }

    mapping = category_index()
    grouped: dict[str, list[dict[str, Any]]] = {key: [] for key in CATEGORY_ALLOWLISTS}
    raw_category_counts = Counter(str(row["category"]) for row in rows)
    for row_index, row in enumerate(rows):
        shared_class = mapping.get(str(row["category"]))
        if shared_class:
            row["dataset_row_index"] = row_index
            grouped[shared_class].append(row)

    missing_allowlist_entries = {
        shared_class: sorted(category for category in categories if raw_category_counts[category] == 0)
        for shared_class, categories in CATEGORY_ALLOWLISTS.items()
    }
    if any(missing_allowlist_entries.values()):
        raise ValueError(f"allowlist contains absent release categories: {missing_allowlist_entries}")

    candidate_audit = {
        "dataset_row_count": len(rows),
        "dataset_unique_object_release_id_count": len(release_id_counts),
        "duplicate_object_release_id_count": len(duplicate_release_ids),
        "rows_with_duplicate_object_release_ids": sum(duplicate_release_ids.values()),
        "maximum_object_release_id_multiplicity": max(duplicate_release_ids.values(), default=1),
        "duplicate_identity_policy": "dataset row index is part of the immutable sample identity; rows are not dropped or selected by tier/status",
        "classes": {},
    }
    for shared_class, class_rows in grouped.items():
        candidate_audit["classes"][shared_class] = {
            **audit_rows(class_rows),
            "raw_category_counts": count([row["category"] for row in class_rows]),
        }

    per_class_n = min(len(class_rows) for class_rows in grouped.values())
    if per_class_n <= 0:
        raise ValueError("at least one shared class has no candidates")

    selected: list[tuple[str, dict[str, Any], str, int]] = []
    for shared_class, class_rows in grouped.items():
        ranked = sorted(
            ((rank_key(shared_class, row), row) for row in class_rows),
            key=lambda item: (item[0], str(item[1]["object_release_id"])),
        )
        selected.extend(
            (shared_class, row, key, rank)
            for rank, (key, row) in enumerate(ranked[:per_class_n], 1)
        )
    selected.sort(key=lambda item: (item[0], item[3], item[2]))

    manifest_rows: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    for shared_class, row, selection_hash, selection_rank in selected:
        object_id = str(row["object_release_id"])
        urdf_text = row.get("urdf")
        urdf_path: Path | None = None
        urdf_hash: str | None = None
        evaluator_error: str | None = None
        metrics: dict[str, Any] = {}
        if isinstance(urdf_text, str) and urdf_text.strip():
            urdf_path = urdf_dir / f"row_{int(row['dataset_row_index']):04d}-{safe_id(object_id)}.urdf"
            urdf_path.write_text(urdf_text, encoding="utf-8")
            urdf_hash = sha256_bytes(urdf_text.encode("utf-8"))
            try:
                metrics = parse_hierarchy(urdf_path)
            except Exception as exc:  # noqa: BLE001
                evaluator_error = f"{type(exc).__name__}: {exc}"
        else:
            evaluator_error = "missing URDF field"

        links_present, links_type, links_error = json_field_state(row.get("links_hierarchy_json"))
        manifest_row = {
            "method": "LAM",
            "dataset_revision": DATASET_REVISION,
            "dataset_row_index": row["dataset_row_index"],
            "sample_id": f"row_{int(row['dataset_row_index']):04d}:{object_id}",
            "object_release_id": object_id,
            "shared_class": shared_class,
            "raw_category": row["category"],
            "selection_hash": selection_hash,
            "selection_rank_within_class": selection_rank,
            "tier": row.get("tier"),
            "status": row.get("status"),
            "caption": row.get("caption"),
            "official_rel_path": row.get("rel_path"),
            "model": row.get("model"),
            "pipeline": row.get("pipeline"),
            "final_urdf": str(urdf_path) if urdf_path else None,
            "final_urdf_sha256": urdf_hash,
        }
        manifest_rows.append(manifest_row)
        records.append(
            {
                **manifest_row,
                "manifest_n_links": row.get("n_links"),
                "manifest_n_joints": row.get("n_joints"),
                "manifest_n_movable": row.get("n_movable"),
                "links_hierarchy_json_present": links_present,
                "links_hierarchy_json_type": links_type,
                "links_hierarchy_json_error": links_error,
                "links_hierarchy_json_sha256": (
                    sha256_bytes(row["links_hierarchy_json"].encode("utf-8"))
                    if isinstance(row.get("links_hierarchy_json"), str)
                    else None
                ),
                "evaluator_complete": evaluator_error is None,
                "evaluator_error": evaluator_error,
                **metrics,
            }
        )

    complete = [row for row in records if row["evaluator_complete"]]
    total_n = len(records)
    metrics = {
        **{
            key: value
            for key, value in aggregate_metrics(records).items()
            if key not in {"selected_count", "evaluator_complete_count"}
        },
        "parent_child_edge_f1": None,
        "hierarchy_exact_match": None,
        "semantic_nesting_accuracy": None,
        "cross_seed_consistency": None,
    }

    prompt_source_hashes = {}
    for shared_class, filenames in PROMPT_SOURCES.items():
        prompt_source_hashes[shared_class] = {}
        for filename in filenames:
            path = (repo / "data/val_data" / filename).resolve()
            path.relative_to(workspace)
            prompt_source_hashes[shared_class][filename] = sha256_file(path)

    protocol = {
        "protocol": "nano3d_hierarchy_lam_official_release_v1",
        "frozen_before_metric_evaluation": True,
        "shared_classes": list(CATEGORY_ALLOWLISTS),
        "official_prompt_sources": PROMPT_SOURCES,
        "official_prompt_source_sha256": prompt_source_hashes,
        "category_allowlists": CATEGORY_ALLOWLISTS,
        "excluded_false_positives": EXCLUDED_FALSE_POSITIVES,
        "selection": {
            "per_class_n": per_class_n,
            "rule": "minimum candidate N across classes, then ascending SHA-256 rank",
            "rank_payload": "salt\\nshared_class\\nobject_release_id\\nraw_category\\ndataset_row_index",
            "salt": SELECTION_SALT,
            "tier_or_status_filter": False,
            "failure_denominator_policy": "selected failures remain in binary metric denominators",
        },
    }

    dataset_api = json.loads(api_path.read_text(encoding="utf-8"))
    if dataset_api.get("sha") != DATASET_REVISION:
        raise ValueError("downloaded dataset API metadata does not match pinned revision")
    if "license:mit" not in dataset_api.get("tags", []):
        raise ValueError("dataset API metadata no longer identifies the release as MIT")
    if "license: mit" not in readme_path.read_text(encoding="utf-8"):
        raise ValueError("dataset README lacks the pinned MIT license declaration")
    provenance = {
        "evaluation_code": {
            "runner": str(Path(__file__).resolve()),
            "runner_sha256": sha256_file(Path(__file__).resolve()),
            "shared_evaluator": str((Path(__file__).resolve().parent / "run_nano3d_hierarchy.py").resolve()),
            "shared_evaluator_sha256": sha256_file(
                (Path(__file__).resolve().parent / "run_nano3d_hierarchy.py").resolve()
            ),
            "shared_metric_function": "run_nano3d_hierarchy.parse_hierarchy",
        },
        "repository": {
            "url": "https://github.com/gaoypeng/LAM",
            "commit": REPOSITORY_COMMIT,
            "license": "Apache-2.0",
            "license_sha256": REPOSITORY_LICENSE_SHA256,
            "agents_md_sha256": sha256_file(repo / "AGENTS.md"),
        },
        "dataset": {
            "url": "https://huggingface.co/datasets/YipengGao/Articulated-Object-Code",
            "revision": DATASET_REVISION,
            "api_reported_revision": dataset_api.get("sha"),
            "license": "MIT",
            "license_evidence": "dataset README YAML front matter and Hub API tag",
            "standalone_license_file_published": False,
            "files": DATASET_FILE_SHA256,
        },
    }
    summary = {
        "protocol": protocol["protocol"],
        "status": "VALIDATED" if len(complete) == total_n else "VALIDATED_WITH_PARSE_FAILURES",
        "validated": True,
        "representation": "official inlined URDF; Nano3D URDF-equivalent hierarchy mapping",
        "dataset": provenance["dataset"],
        "selection": {
            "rule": protocol["selection"]["rule"],
            "class_count": len(grouped),
            "candidate_counts": {key: len(value) for key, value in grouped.items()},
            "per_class_n": per_class_n,
            "selected_tier_counts": count([row.get("tier") for row in records]),
            "selected_status_counts": count([row.get("status") for row in records]),
        },
        "selected_count": total_n,
        "evaluator_complete_count": len(complete),
        "evaluator_failure_count": total_n - len(complete),
        "metrics": metrics,
        "per_class_metrics": {
            shared_class: aggregate_metrics(
                [row for row in records if row["shared_class"] == shared_class]
            )
            for shared_class in CATEGORY_ALLOWLISTS
        },
        "selected_field_audit": audit_rows(
            [row for _, row, _, _ in selected]
        ),
        "invalid_tree_samples": [
            {
                "sample_id": row["sample_id"],
                "shared_class": row["shared_class"],
                "tier": row["tier"],
                "status": row["status"],
                "root_names": row.get("root_names"),
                "malformed_edges": row.get("malformed_edges"),
            }
            for row in records
            if row["evaluator_complete"] and not row.get("valid_tree")
        ],
        "metric_notes": {
            "binary_denominator": "all selected rows, including structural or parse failures",
            "structural_means_denominator": "evaluator-complete selected rows",
            "correctness_metrics": "N/A: no independent hierarchy gold",
            "cross_seed_consistency": "N/A: LAM release rows are per-asset outputs",
        },
    }

    (output / "manifest.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in manifest_rows), encoding="utf-8"
    )
    (output / "records.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in records), encoding="utf-8"
    )
    write_json(output / "selection_protocol.json", protocol)
    write_json(output / "candidate_audit.json", candidate_audit)
    write_json(output / "source_provenance.json", provenance)
    write_json(output / "summary.json", summary)
    (output / "report.md").write_text(render_report(summary, candidate_audit), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
