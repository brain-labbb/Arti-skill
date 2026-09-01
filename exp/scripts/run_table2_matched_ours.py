#!/usr/bin/env python3
"""Generate and evaluate the frozen Ours cohort for Table 2 Naming."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
import random
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
from typing import Any, Iterable
from xml.etree import ElementTree as ET


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PROTOCOL_PATH = EXP_ROOT / "reference" / "table2_naming_matched_protocol_v1.json"
DEFAULT_OUT = EXP_ROOT / "runtime" / "table2_naming_matched_ours_v1"
PLACEHOLDER_RE = re.compile(
    r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$",
    re.IGNORECASE,
)

sys.path.insert(0, str(TEMPLATE_ROOT))

from agent.template_registry import TEMPLATE_REGISTRY  # noqa: E402
from agent.compiler import compile_urdf_report  # noqa: E402
from agent.template_sweep import (  # noqa: E402
    _GENERIC_MODEL_TEMPLATE,
    _config_to_dict,
    _resolve_config_from_seed,
    compiled_artifact_is_valid,
)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(payload: Any) -> str:
    data = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def write_text_atomic(path: Path, text: str) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def write_json(path: Path, payload: Any) -> None:
    write_text_atomic(path, json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n")


def tail_text(value: str | bytes | None, limit: int) -> str:
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return (value or "")[-limit:]


def remove_worker_metadata(package: Path) -> None:
    for name in ("compile_task.json", "compile_result.json"):
        candidate = contained(package / name)
        if candidate.is_file() and not candidate.is_symlink():
            candidate.unlink()


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def direct_children(node: ET.Element, name: str) -> Iterable[ET.Element]:
    return (child for child in node if local_name(child.tag) == name)


def finite_positive(raw: str | None, count: int) -> bool:
    if raw is None:
        return False
    try:
        values = [float(item) for item in raw.split()]
    except ValueError:
        return False
    return len(values) == count and all(math.isfinite(value) and value > 0 for value in values)


def geometry_status(geometry: ET.Element, package: Path) -> tuple[bool, str, str | None]:
    shapes = list(geometry)
    if len(shapes) != 1:
        return False, "unknown", "geometry_child_count"
    shape = shapes[0]
    kind = local_name(shape.tag)
    if kind == "mesh":
        filename = (shape.get("filename") or "").strip()
        if not filename or Path(filename).is_absolute():
            return False, kind, "mesh_filename_invalid"
        candidate = (package / filename).resolve()
        if candidate != package and package not in candidate.parents:
            return False, kind, "mesh_path_escape"
        if not candidate.is_file() or candidate.stat().st_size <= 0:
            return False, kind, "mesh_missing_or_empty"
        return True, kind, None
    if kind == "box":
        valid = finite_positive(shape.get("size"), 3)
    elif kind == "cylinder":
        valid = finite_positive(shape.get("radius"), 1) and finite_positive(shape.get("length"), 1)
    elif kind == "sphere":
        valid = finite_positive(shape.get("radius"), 1)
    else:
        return False, kind, "unsupported_geometry"
    return valid, kind, None if valid else "nonpositive_or_nonfinite_primitive"


def evaluate_package(category: str, slug: str, seed: int, package: Path) -> dict[str, Any]:
    package = contained(package)
    urdf = package / "model.urdf"
    record: dict[str, Any] = {
        "category": category,
        "slug": slug,
        "seed": seed,
        "package": str(package.relative_to(WORKSPACE_ROOT)),
        "artifact_valid": compiled_artifact_is_valid(str(package)),
        "urdf_parse_success": False,
        "renderable_part_count": 0,
        "named_renderable_part_count": 0,
        "placeholder_renderable_part_count": 0,
        "placeholder_names": {},
        "valid_visual_geometry_count": 0,
        "invalid_visual_geometry_count": 0,
        "valid_geometry_types": {},
        "invalid_geometry_reasons": {},
        "mesh_reference_sha256": {},
        "renderable_names": [],
        "urdf_sha256": None,
        "issues": [],
    }
    if not urdf.is_file() or urdf.stat().st_size <= 0:
        record["issues"].append("model.urdf missing or empty")
        return record
    record["urdf_sha256"] = sha256_file(urdf)
    try:
        root = ET.parse(urdf).getroot()
    except (ET.ParseError, OSError) as exc:
        record["issues"].append(f"URDF parse error: {type(exc).__name__}: {exc}")
        return record
    record["urdf_parse_success"] = True
    valid_kinds: Counter[str] = Counter()
    invalid_reasons: Counter[str] = Counter()
    placeholders: Counter[str] = Counter()
    mesh_hashes: dict[str, str] = {}
    names: list[str] = []
    for link in direct_children(root, "link"):
        link_name = (link.get("name") or "").strip()
        link_valid_visuals = 0
        for visual in direct_children(link, "visual"):
            geometries = list(direct_children(visual, "geometry"))
            if len(geometries) != 1:
                invalid_reasons["visual_geometry_count"] += 1
                continue
            valid, kind, issue = geometry_status(geometries[0], package)
            if valid:
                link_valid_visuals += 1
                valid_kinds[kind] += 1
                if kind == "mesh":
                    shape = list(geometries[0])[0]
                    filename = (shape.get("filename") or "").strip()
                    mesh_hashes[filename] = sha256_file((package / filename).resolve())
            else:
                invalid_reasons[issue or "invalid_geometry"] += 1
        if link_valid_visuals == 0:
            continue
        names.append(link_name)
        if link_name and PLACEHOLDER_RE.fullmatch(link_name) is None:
            record["named_renderable_part_count"] += 1
        else:
            placeholders[link_name or "<empty>"] += 1
    record["renderable_names"] = names
    record["renderable_part_count"] = len(names)
    record["placeholder_renderable_part_count"] = sum(placeholders.values())
    record["placeholder_names"] = dict(sorted(placeholders.items()))
    record["valid_visual_geometry_count"] = sum(valid_kinds.values())
    record["invalid_visual_geometry_count"] = sum(invalid_reasons.values())
    record["valid_geometry_types"] = dict(sorted(valid_kinds.items()))
    record["invalid_geometry_reasons"] = dict(sorted(invalid_reasons.items()))
    record["mesh_reference_sha256"] = dict(sorted(mesh_hashes.items()))
    if not names:
        record["issues"].append("no renderable URDF links")
    return record


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def pooled_metrics(records: list[dict[str, Any]]) -> tuple[float, float]:
    parts = sum(row["renderable_part_count"] for row in records)
    named = sum(row["named_renderable_part_count"] for row in records)
    return parts / len(records), named / parts if parts else 0.0


def category_cluster_ci(
    records: list[dict[str, Any]], categories: list[str], resamples: int, confidence: float, seed: int
) -> dict[str, list[float]]:
    by_category = {category: [row for row in records if row["category"] == category] for category in categories}
    rng = random.Random(seed)
    part_values: list[float] = []
    name_values: list[float] = []
    for _ in range(resamples):
        sampled = [rng.choice(categories) for _ in categories]
        cohort = list(itertools.chain.from_iterable(by_category[category] for category in sampled))
        parts, nameability = pooled_metrics(cohort)
        part_values.append(parts)
        name_values.append(nameability)
    alpha = (1.0 - confidence) / 2.0
    return {
        "parts_per_asset_mean": [percentile(part_values, alpha), percentile(part_values, 1.0 - alpha)],
        "nameability_micro": [percentile(name_values, alpha), percentile(name_values, 1.0 - alpha)],
    }


def jaccard(left: set[str], right: set[str]) -> float:
    union = left | right
    return len(left & right) / len(union) if union else 1.0


def weighted_jaccard(left: Counter[str], right: Counter[str]) -> float:
    names = set(left) | set(right)
    denominator = sum(max(left[name], right[name]) for name in names)
    return sum(min(left[name], right[name]) for name in names) / denominator if denominator else 1.0


def cross_seed(records: list[dict[str, Any]], categories: list[str]) -> dict[str, Any]:
    pair_set: list[float] = []
    pair_multiset: list[float] = []
    category_set: list[float] = []
    category_multiset: list[float] = []
    exact_modes: list[float] = []
    pair_count = 0
    for category in categories:
        rows = sorted((row for row in records if row["category"] == category), key=lambda row: row["seed"])
        sets = [set(row["renderable_names"]) for row in rows]
        counters = [Counter(row["renderable_names"]) for row in rows]
        set_values: list[float] = []
        multiset_values: list[float] = []
        for left, right in itertools.combinations(range(len(rows)), 2):
            set_values.append(jaccard(sets[left], sets[right]))
            multiset_values.append(weighted_jaccard(counters[left], counters[right]))
        signatures = Counter(tuple(sorted(counter.items())) for counter in counters)
        pair_count += len(set_values)
        pair_set.extend(set_values)
        pair_multiset.extend(multiset_values)
        category_set.append(statistics.mean(set_values))
        category_multiset.append(statistics.mean(multiset_values))
        exact_modes.append(max(signatures.values()) / len(rows))
    return {
        "within_category_pair_count": pair_count,
        "raw_unique_name_set_jaccard_pair_micro": statistics.mean(pair_set),
        "raw_unique_name_set_jaccard_category_macro": statistics.mean(category_set),
        "raw_name_multiset_weighted_jaccard_pair_micro": statistics.mean(pair_multiset),
        "raw_name_multiset_weighted_jaccard_category_macro": statistics.mean(category_multiset),
        "exact_raw_name_multiset_mode_rate_category_macro": statistics.mean(exact_modes),
        "interpretation": "raw link-name consistency only; not semantic-role consistency",
    }


def protocol_rows(protocol: dict[str, Any]) -> list[dict[str, Any]]:
    seeds = protocol["selection"]["ours"]["seeds"]
    rows = []
    for category in protocol["design"]["canonical_categories"]:
        template = protocol["taxonomy"][category]["ours"]["template"]
        path = contained(PROJECT_ROOT / template.removeprefix("arti-skill/"))
        if sha256_file(path) != protocol["taxonomy"][category]["ours"]["sha256"]:
            raise RuntimeError(f"template hash mismatch: {template}")
        slug = path.stem
        if slug not in TEMPLATE_REGISTRY:
            raise RuntimeError(f"template absent from registry: {slug}")
        rows.append({
            "category": category,
            "slug": slug,
            "stem": TEMPLATE_REGISTRY[slug],
            "template": str(path.relative_to(WORKSPACE_ROOT)),
            "template_sha256": sha256_file(path),
            "seeds": seeds,
        })
    return rows


def publish_visual_artifact(
    *,
    row: dict[str, Any],
    seed: int,
    destination: Path,
    timeout: float,
) -> dict[str, Any]:
    """Compile one frozen template seed at the representation needed by Naming."""
    start = time.monotonic()
    destination = contained(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if compiled_artifact_is_valid(destination):
        remove_worker_metadata(destination)
        metadata = json.loads((destination / "artifact.json").read_text(encoding="utf-8"))
        return {
            "category": row["category"],
            "slug": row["slug"],
            "seed": seed,
            "verdict": "pass",
            "elapsed_s": 0.0,
            "artifact_dir": str(destination),
            "failure_type": None,
            "failure_details": None,
            "compile_target": "visual",
            "run_checks": False,
            "cached": True,
            "config": metadata.get("config"),
            "warnings": metadata.get("warnings", []),
        }

    work_dir = Path(
        tempfile.mkdtemp(prefix=f".compile_{row['slug']}_{seed}_", dir=destination.parent)
    )
    try:
        task_path = work_dir / "compile_task.json"
        result_path = work_dir / "compile_result.json"
        task = {
            "slug": row["slug"],
            "stem": row["stem"],
            "seed": seed,
            "template_sha256": row["template_sha256"],
            "work_dir": str(work_dir),
        }
        task_path.write_text(
            json.dumps(task, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        try:
            worker = subprocess.run(
                [sys.executable, str(contained(Path(__file__))), "--compile-worker", str(task_path), str(result_path)],
                cwd=PROJECT_ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            return {
                "category": row["category"],
                "slug": row["slug"],
                "seed": seed,
                "verdict": "fail",
                "elapsed_s": time.monotonic() - start,
                "artifact_dir": None,
                "failure_type": "compile_timeout",
                "failure_details": f"visual compile exceeded {timeout:.1f}s",
                "compile_target": "visual",
                "run_checks": False,
                "cached": False,
                "worker_stdout_tail": tail_text(exc.stdout, 2000),
                "worker_stderr_tail": tail_text(exc.stderr, 4000),
            }
        result = (
            json.loads(result_path.read_text(encoding="utf-8"))
            if result_path.is_file()
            else {
                "verdict": "fail",
                "failure_type": "worker_result_missing",
                "failure_details": f"worker exit={worker.returncode}",
            }
        )
        if worker.returncode != 0 or result.get("verdict") != "pass":
            return {
                "category": row["category"],
                "slug": row["slug"],
                "seed": seed,
                "verdict": "fail",
                "elapsed_s": time.monotonic() - start,
                "artifact_dir": None,
                "failure_type": result.get("failure_type") or f"worker_exit_{worker.returncode}",
                "failure_details": result.get("failure_details"),
                "compile_target": "visual",
                "run_checks": False,
                "cached": False,
                "worker_stdout_tail": tail_text(worker.stdout, 2000),
                "worker_stderr_tail": tail_text(worker.stderr, 4000),
            }
        metadata = json.loads((work_dir / "artifact.json").read_text(encoding="utf-8"))
        remove_worker_metadata(work_dir)
        if destination.exists():
            if destination.is_dir():
                shutil.rmtree(destination)
            else:
                destination.unlink()
        work_dir.replace(destination)
        if not compiled_artifact_is_valid(destination):
            raise RuntimeError("published artifact failed hash validation")
        return {
            "category": row["category"],
            "slug": row["slug"],
            "seed": seed,
            "verdict": "pass",
            "elapsed_s": time.monotonic() - start,
            "artifact_dir": str(destination),
            "failure_type": None,
            "failure_details": None,
            "compile_target": "visual",
            "run_checks": False,
            "cached": False,
            "config": metadata["config"],
            "warnings": metadata["warnings"],
            "worker_stdout_tail": tail_text(worker.stdout, 2000),
            "worker_stderr_tail": tail_text(worker.stderr, 4000),
        }
    except Exception as exc:  # noqa: BLE001 - failure is preserved as experiment evidence
        return {
            "category": row["category"],
            "slug": row["slug"],
            "seed": seed,
            "verdict": "fail",
            "elapsed_s": time.monotonic() - start,
            "artifact_dir": None,
            "failure_type": type(exc).__name__,
            "failure_details": str(exc),
            "compile_target": "visual",
            "run_checks": False,
            "cached": False,
        }
    finally:
        if work_dir.exists():
            shutil.rmtree(work_dir, ignore_errors=True)


def generate(rows: list[dict[str, Any]], out: Path, timeout: float) -> list[dict[str, Any]]:
    generation_records: list[dict[str, Any]] = []
    for index, row in enumerate(rows, 1):
        print(f"[{index}/{len(rows)}] {row['category']} -> {row['slug']}", flush=True)
        for seed in row["seeds"]:
            outcome = publish_visual_artifact(
                row=row,
                seed=seed,
                destination=out / "artifacts" / row["category"] / f"seed_{seed}",
                timeout=timeout,
            )
            generation_records.append(outcome)
            print(
                f"  seed={seed} {outcome['verdict'].upper()} "
                f"elapsed={outcome['elapsed_s']:.3f}s cached={outcome['cached']}",
                flush=True,
            )
    write_json(out / "generation_records.json", generation_records)
    return generation_records


def evaluate(rows: list[dict[str, Any]], generation_records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    expected = {(row["category"], seed): row for row in rows for seed in row["seeds"]}
    found = {(row["category"], row["seed"]): row for row in generation_records}
    if set(expected) != set(found):
        raise RuntimeError(f"generation record set mismatch: missing={sorted(set(expected)-set(found))}")
    records: list[dict[str, Any]] = []
    for key in sorted(expected):
        source = found[key]
        category, seed = key
        row = expected[key]
        artifact_dir = source.get("artifact_dir")
        if source["verdict"] != "pass" or not artifact_dir:
            records.append({
                "category": category,
                "slug": row["slug"],
                "seed": seed,
                "generation_verdict": source["verdict"],
                "artifact_valid": False,
                "urdf_parse_success": False,
                "renderable_part_count": 0,
                "named_renderable_part_count": 0,
                "placeholder_renderable_part_count": 0,
                "renderable_names": [],
                "issues": [source.get("failure_type") or "strict generation failed"],
            })
            continue
        evaluated = evaluate_package(category, row["slug"], seed, Path(artifact_dir))
        evaluated["generation_verdict"] = source["verdict"]
        records.append(evaluated)
    return records


def build_artifact_hash_manifest(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "category": row["category"],
            "seed": row["seed"],
            "urdf_sha256": row.get("urdf_sha256"),
            "mesh_reference_sha256": row.get("mesh_reference_sha256", {}),
        }
        for row in records
    ]


def build_summary(
    protocol: dict[str, Any],
    records: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    artifact_hash_manifest: list[dict[str, Any]],
) -> dict[str, Any]:
    categories = protocol["design"]["canonical_categories"]
    evaluable = [row for row in records if row.get("artifact_valid") and row.get("urdf_parse_success") and row["renderable_part_count"] > 0]
    expected = protocol["design"]["assets_per_method"]
    parts = sum(row["renderable_part_count"] for row in evaluable)
    named = sum(row["named_renderable_part_count"] for row in evaluable)
    placeholders = sum(row["placeholder_renderable_part_count"] for row in evaluable)
    bootstrap = protocol["bootstrap"]
    ci = category_cluster_ci(
        evaluable,
        categories,
        bootstrap["resamples"],
        bootstrap["confidence"],
        bootstrap["seed"],
    ) if len(evaluable) == expected else None
    per_category = {}
    for category in categories:
        group = [row for row in evaluable if row["category"] == category]
        group_parts = sum(row["renderable_part_count"] for row in group)
        group_named = sum(row["named_renderable_part_count"] for row in group)
        per_category[category] = {
            "assets": len(group),
            "parts_per_asset_mean": group_parts / len(group) if group else None,
            "nameability_micro": group_named / group_parts if group_parts else None,
        }
    stable = {
        "protocol_id": protocol["protocol_id"],
        "method": "Ours",
        "comparison_label": protocol["comparison_label"],
        "status": "COMPLETE" if len(evaluable) == expected else "INCOMPLETE",
        "coverage": {
            "requested_assets": expected,
            "visual_compile_pass": sum(row["generation_verdict"] == "pass" for row in records),
            "naming_evaluable_assets": len(evaluable),
            "categories": len(categories),
            "assets_per_category": protocol["design"]["assets_per_category_per_method"],
        },
        "direct_metrics": {
            "total_renderable_parts": parts,
            "total_named_renderable_parts": named,
            "placeholder_renderable_parts": placeholders,
            "valid_visual_geometry_count": sum(
                row.get("valid_visual_geometry_count", 0) for row in evaluable
            ),
            "invalid_visual_geometry_count": sum(
                row.get("invalid_visual_geometry_count", 0) for row in evaluable
            ),
            "hashed_mesh_reference_count": sum(
                len(row.get("mesh_reference_sha256", {})) for row in evaluable
            ),
            "parts_per_asset_mean": parts / len(evaluable) if evaluable else None,
            "parts_per_asset_median": statistics.median(row["renderable_part_count"] for row in evaluable) if evaluable else None,
            "nameability_micro": named / parts if parts else None,
            "category_cluster_bootstrap_95ci": ci,
            "per_category": per_category,
            "representation": "URDF renderable-link",
        },
        "cross_seed": cross_seed(evaluable, categories) if len(evaluable) == expected else None,
        "semantic_metrics": {
            "semantic_precision": None,
            "semantic_recall": None,
            "naming_richness": None,
            "functional_core_coverage": None,
            "instance_discriminability": None,
            "over_segmentation_rate": None,
            "reason": "No matched-cohort output-independent role gold plus three blind judge verdict sets.",
        },
        "templates": rows,
        "records_sha256": sha256_json(records),
        "artifact_hash_manifest_sha256": sha256_json(artifact_hash_manifest),
    }
    return {
        **stable,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "protocol_sha256": sha256_file(PROTOCOL_PATH),
        "stable_summary_sha256": sha256_json(stable),
    }


def render_report(summary: dict[str, Any]) -> str:
    direct = summary["direct_metrics"]
    coverage = summary["coverage"]
    ci = direct["category_cluster_bootstrap_95ci"] or {}
    return f"""# Table 2 Naming: Ours category-matched cohort

- Status: **{summary['status']}**
- Protocol: `{summary['protocol_id']}` (`{summary['protocol_sha256']}`)
- Design: five categories x seven seeds = {coverage['requested_assets']} requested assets.
- Visual compile and Naming artifact gate: {coverage['visual_compile_pass']}/{coverage['requested_assets']}.
- Naming-evaluable: {coverage['naming_evaluable_assets']}/{coverage['requested_assets']}.
- Parts: {direct['total_renderable_parts']} renderable URDF links; {direct['parts_per_asset_mean']} mean/asset.
- Nameability: {direct['total_named_renderable_parts']}/{direct['total_renderable_parts']} = {direct['nameability_micro']} micro.
- Category-cluster 95% CI: parts={ci.get('parts_per_asset_mean')}; nameability={ci.get('nameability_micro')}.

This is a category-matched deterministic comparison, not a same-prompt comparison.
Semantic judge metrics remain N/A until output-independent gold and three blind verdict sets exist.
"""


def build_self_check(
    protocol: dict[str, Any],
    records: list[dict[str, Any]],
    summary: dict[str, Any],
    artifact_hash_manifest: list[dict[str, Any]],
    out: Path,
) -> dict[str, Any]:
    expected = protocol["design"]["assets_per_method"]
    categories = protocol["design"]["canonical_categories"]
    checks = {
        "status_complete": summary["status"] == "COMPLETE",
        "record_count_exact": len(records) == expected,
        "category_count_exact": len({row["category"] for row in records}) == len(categories),
        "seven_assets_each_category": all(sum(row["category"] == category for row in records) == 7 for category in categories),
        "seeds_zero_through_six_each_category": all(sorted(row["seed"] for row in records if row["category"] == category) == list(range(7)) for category in categories),
        "all_visual_compiles_pass": all(row["generation_verdict"] == "pass" for row in records),
        "all_artifacts_valid": all(row.get("artifact_valid") for row in records),
        "all_urdfs_parse": all(row.get("urdf_parse_success") for row in records),
        "all_have_renderable_parts": all(row["renderable_part_count"] > 0 for row in records),
        "all_visual_geometries_valid": all(
            row.get("invalid_visual_geometry_count") == 0 for row in records
        ),
        "all_mesh_references_hashed": all(
            all(re.fullmatch(r"[0-9a-f]{64}", digest) for digest in row.get("mesh_reference_sha256", {}).values())
            for row in records
        ),
        "artifact_file_counts_exact": (
            len(list((out / "artifacts").glob("*/seed_*/model.urdf"))) == expected
            and len(list((out / "artifacts").glob("*/seed_*/artifact.json"))) == expected
        ),
        "artifact_tree_has_no_symlinks": not any(
            path.is_symlink() for path in (out / "artifacts").rglob("*")
        ),
        "artifact_hash_manifest_current": (
            summary["artifact_hash_manifest_sha256"] == sha256_json(artifact_hash_manifest)
        ),
        "parts_conserved": sum(row["renderable_part_count"] for row in records) == summary["direct_metrics"]["total_renderable_parts"],
        "names_conserved": sum(row["named_renderable_part_count"] for row in records) == summary["direct_metrics"]["total_named_renderable_parts"],
        "named_plus_placeholder_equals_parts": all(row["named_renderable_part_count"] + row["placeholder_renderable_part_count"] == row["renderable_part_count"] for row in records),
        "semantic_fields_null": all(value is None for key, value in summary["semantic_metrics"].items() if key != "reason"),
        "comparison_not_same_prompt": protocol["prohibited_label"] != protocol["comparison_label"],
        "protocol_hash_current": summary["protocol_sha256"] == sha256_file(PROTOCOL_PATH),
    }
    return {
        "protocol_id": "nano3d_table2_naming_matched_ours_self_check_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "records_sha256": summary["records_sha256"],
        "stable_summary_sha256": summary["stable_summary_sha256"],
        "artifact_hash_manifest_sha256": summary["artifact_hash_manifest_sha256"],
    }


def compile_worker(task_path: Path, result_path: Path) -> int:
    """Isolated visual compiler entrypoint; the parent enforces the wall-time."""
    task_path = contained(task_path)
    result_path = contained(result_path)
    task = json.loads(task_path.read_text(encoding="utf-8"))
    work_dir = contained(Path(task["work_dir"]))
    try:
        config = _config_to_dict(_resolve_config_from_seed(task["slug"], int(task["seed"])))
        script = work_dir / "model.py"
        script.write_text(
            _GENERIC_MODEL_TEMPLATE.format(
                slug=task["slug"], stem=task["stem"], seed=int(task["seed"])
            ),
            encoding="utf-8",
        )
        report = compile_urdf_report(
            script,
            sdk_package="sdk",
            run_checks=False,
            target="visual",
            rewrite_visual_glb=False,
            motion_qc=False,
        )
        urdf = work_dir / "model.urdf"
        urdf.write_text(report.urdf_xml, encoding="utf-8")
        metadata = {
            "schema_version": 1,
            "slug": task["slug"],
            "seed": int(task["seed"]),
            "config": config,
            "urdf_sha256": sha256_file(urdf),
            "template_sha256": task["template_sha256"],
            "compile_target": "visual",
            "run_checks": False,
            "motion_qc": False,
            "warnings": [str(item) for item in report.warnings],
        }
        (work_dir / "artifact.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        write_json(result_path, {"verdict": "pass"})
        return 0
    except Exception as exc:  # noqa: BLE001 - serialized as experiment evidence
        write_json(
            result_path,
            {
                "verdict": "fail",
                "failure_type": type(exc).__name__,
                "failure_details": str(exc),
            },
        )
        return 1
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--evaluate-existing", action="store_true")
    parser.add_argument("--compile-worker", nargs=2, metavar=("TASK", "RESULT"), help=argparse.SUPPRESS)
    args = parser.parse_args()

    os.environ.update({
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
        "VECLIB_MAXIMUM_THREADS": "1",
    })
    if args.compile_worker:
        return compile_worker(Path(args.compile_worker[0]), Path(args.compile_worker[1]))
    out = contained(args.out)
    if EXP_ROOT.resolve() not in out.parents:
        raise SystemExit("--out must be inside exp")
    out.mkdir(parents=True, exist_ok=True)
    protocol = json.loads(contained(PROTOCOL_PATH).read_text(encoding="utf-8"))
    if sha256_file(EXP_ROOT / "reference" / "baseline_naming_protocol_v1.json") != protocol["base_naming_protocol"]["sha256"]:
        raise RuntimeError("base Naming protocol hash mismatch")
    rows = protocol_rows(protocol)
    manifest = {
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": sha256_file(PROTOCOL_PATH),
        "method": "Ours",
        "comparison_label": protocol["comparison_label"],
        "seeds": protocol["selection"]["ours"]["seeds"],
        "templates": rows,
        "artifact_gate": (
            "compiler target=visual, run_checks=False, followed by frozen Naming v1.1 "
            "renderable-geometry and artifact-hash validation"
        ),
        "scope_note": "Full physics QC is outside Table 2 Naming and is not an inclusion gate.",
    }
    write_json(out / "manifest.json", manifest)

    generation_path = out / "generation_records.json"
    if args.evaluate_existing:
        generation_records = json.loads(contained(generation_path).read_text(encoding="utf-8"))
    else:
        generation_records = generate(rows, out, args.timeout)
    records = evaluate(rows, generation_records)
    records_text = "".join(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n" for row in records)
    write_text_atomic(out / "records.jsonl", records_text)
    artifact_hash_manifest = build_artifact_hash_manifest(records)
    write_json(out / "artifact_hash_manifest.json", artifact_hash_manifest)
    summary = build_summary(protocol, records, rows, artifact_hash_manifest)
    write_json(out / "summary.json", summary)
    write_text_atomic(out / "report.md", render_report(summary))
    self_check = build_self_check(protocol, records, summary, artifact_hash_manifest, out)
    write_json(out / "self_check.json", self_check)
    print(json.dumps({
        "status": summary["status"],
        "self_check": self_check["status"],
        "coverage": summary["coverage"],
        "direct_metrics": summary["direct_metrics"],
    }, indent=2, ensure_ascii=False))
    return 0 if self_check["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
