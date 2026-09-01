#!/usr/bin/env python3
"""Evaluate direct Naming metrics for frozen Infinite Mobility URDF packages.

The evaluation intentionally stops at metrics that do not require semantic
part gold or human judges. A part is one URDF link with at least one valid
renderable visual geometry. Multiple visuals on the same link are merged for
the node count under the shared baseline Naming protocol v1.1.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import re
import statistics
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from itertools import combinations
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = REPO_ROOT / "exp/runtime/infinite_mobility_v1"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/infinite_mobility_naming_v1"
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/infinite_mobility_protocol_v1.json"
DEFAULT_COMMON_PROTOCOL = REPO_ROOT / "exp/reference/baseline_naming_protocol_v1.json"
DEFAULT_RECOVERY = REPO_ROOT / "exp/runtime/infinite_mobility_timeout_recovery_v1"
DEFAULT_MATCHED_PROTOCOL = REPO_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"

# Keep this exactly aligned with run_nano3d_naming.py.
PLACEHOLDER_RE = re.compile(
    r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$", re.I
)
OPAQUE_INDEX_RE = re.compile(r"^l[_-]?\d+$", re.I)
BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 260_811_002


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cohort_sha256(rows: Iterable[tuple[str, Path]]) -> str:
    digest = hashlib.sha256()
    for identifier, path in sorted(rows):
        encoded = identifier.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def percentile(values: list[float], quantile: float) -> float:
    position = (len(values) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def bootstrap_mean_ci(
    values: list[float], *, resamples: int, seed: int
) -> list[float] | None:
    if not values:
        return None
    rng = random.Random(seed)
    boot = sorted(
        statistics.mean(rng.choice(values) for _ in values)
        for _ in range(resamples)
    )
    return [percentile(boot, 0.025), percentile(boot, 0.975)]


def cluster_bootstrap_parts_ci(
    factory_parts: dict[str, list[int]], *, resamples: int, seed: int
) -> list[float]:
    """Resample factories, then observed PASS seeds within each factory."""
    rng = random.Random(seed)
    factories = sorted(factory_parts)
    boot = []
    for _ in range(resamples):
        sampled_values: list[int] = []
        for factory in (rng.choice(factories) for _ in factories):
            values = factory_parts[factory]
            sampled_values.extend(rng.choice(values) for _ in values)
        boot.append(statistics.mean(sampled_values))
    boot.sort()
    return [percentile(boot, 0.025), percentile(boot, 0.975)]


def set_jaccard(a: set[str], b: set[str]) -> float:
    union = a | b
    return len(a & b) / len(union) if union else 1.0


def multiset_jaccard(a: Counter[str], b: Counter[str]) -> float:
    keys = set(a) | set(b)
    denominator = sum(max(a[key], b[key]) for key in keys)
    return sum(min(a[key], b[key]) for key in keys) / denominator if denominator else 1.0


def signature(names: list[str]) -> tuple[tuple[str, int], ...]:
    return tuple(sorted(Counter(names).items()))


def require_within(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise RuntimeError(f"{label} escapes allowed root: {resolved}") from exc
    return resolved


def finite_positive_values(raw: str | None, expected: int) -> list[float] | None:
    if raw is None:
        return None
    try:
        values = [float(item) for item in raw.split()]
    except ValueError:
        return None
    if len(values) != expected or not all(math.isfinite(item) and item > 0 for item in values):
        return None
    return values


def renderable_geometry(
    geometry: ET.Element | None, urdf: Path, package_dir: Path
) -> dict[str, Any] | None:
    if geometry is None or len(geometry) != 1:
        return None
    shape = geometry[0]
    if shape.tag == "mesh":
        filename = shape.get("filename")
        if not filename:
            return None
        mesh_path = require_within(urdf.parent / filename, package_dir, "mesh")
        if not mesh_path.is_file() or mesh_path.stat().st_size <= 0:
            return None
        return {"type": "mesh", "filename": filename}
    if shape.tag == "box":
        values = finite_positive_values(shape.get("size"), 3)
        return {"type": "box", "size": values} if values is not None else None
    if shape.tag == "cylinder":
        radius = finite_positive_values(shape.get("radius"), 1)
        length = finite_positive_values(shape.get("length"), 1)
        if radius is not None and length is not None:
            return {"type": "cylinder", "radius": radius[0], "length": length[0]}
        return None
    if shape.tag == "sphere":
        radius = finite_positive_values(shape.get("radius"), 1)
        return {"type": "sphere", "radius": radius[0]} if radius is not None else None
    return None


def evaluate_asset(
    baseline_root: Path,
    row: dict[str, Any],
    *,
    original_status: str,
    artifact_source: str,
) -> tuple[dict[str, Any], tuple[str, Path]]:
    factory = str(row["factory"])
    seed = int(row["seed"])
    case_dir = baseline_root / "cases" / factory / f"seed_{seed:03d}"
    package_dir = require_within(case_dir / "package", baseline_root, "package")
    relative_urdf = Path(str(row["validation"]["urdf_path"]))
    urdf = require_within(package_dir / relative_urdf, package_dir, "URDF")
    if not urdf.is_file():
        raise RuntimeError(f"missing PASS URDF: {urdf}")

    root = ET.parse(urdf).getroot()
    part_links: list[dict[str, Any]] = []
    renderable_visual_geometry_count = 0
    mesh_visual_geometry_count = 0
    primitive_visual_geometry_count = 0
    invalid_or_unsupported_visual_geometry_count = 0
    mesh_reference_count = 0
    for link in root.findall("link"):
        name = link.get("name", "")
        if not name:
            raise RuntimeError(f"unnamed link in {urdf}")
        geometries: list[dict[str, Any]] = []
        for visual in link.findall("visual"):
            item = renderable_geometry(visual.find("geometry"), urdf, package_dir)
            if item is None:
                invalid_or_unsupported_visual_geometry_count += 1
                continue
            geometries.append(item)
        if geometries:
            renderable_visual_geometry_count += len(geometries)
            mesh_count = sum(item["type"] == "mesh" for item in geometries)
            primitive_count = len(geometries) - mesh_count
            mesh_visual_geometry_count += mesh_count
            primitive_visual_geometry_count += primitive_count
            mesh_reference_count += mesh_count
            part_links.append(
                {
                    "name": name,
                    "renderable_visual_geometry_count": len(geometries),
                    "mesh_reference_count": mesh_count,
                    "primitive_visual_geometry_count": primitive_count,
                    "renderable_geometries": geometries,
                    "placeholder": bool(PLACEHOLDER_RE.fullmatch(name)),
                    "opaque_generated_index_name": bool(OPAQUE_INDEX_RE.fullmatch(name)),
                }
            )

    names = [item["name"] for item in part_links]
    if len(names) != len(set(names)):
        raise RuntimeError(f"duplicate Naming-evaluable URDF part names in {urdf}")
    named = [name for name in names if not PLACEHOLDER_RE.fullmatch(name)]
    opaque = [name for name in names if OPAQUE_INDEX_RE.fullmatch(name)]
    return (
        {
            "factory": factory,
            "seed": seed,
            "source_status": row["status"],
            "original_300s_status": original_status,
            "artifact_source": artifact_source,
            "source_package_sha256": row.get("package_sha256"),
            "urdf_path": urdf.relative_to(REPO_ROOT).as_posix(),
            "urdf_sha256": sha256_file(urdf),
            "urdf_part_node_count": len(part_links),
            "renderable_visual_geometry_count": renderable_visual_geometry_count,
            "mesh_visual_geometry_count": mesh_visual_geometry_count,
            "primitive_visual_geometry_count": primitive_visual_geometry_count,
            "invalid_or_unsupported_visual_geometry_count": invalid_or_unsupported_visual_geometry_count,
            "mesh_reference_count": mesh_reference_count,
            "multi_visual_part_node_count": sum(
                item["renderable_visual_geometry_count"] > 1 for item in part_links
            ),
            "named_urdf_part_node_count": len(named),
            "placeholder_urdf_part_node_count": len(names) - len(named),
            "nameability": len(named) / len(names) if names else None,
            "opaque_generated_index_name_count": len(opaque),
            "opaque_generated_index_name_rate": len(opaque) / len(names) if names else None,
            "raw_part_names": names,
            "raw_name_multiset": dict(sorted(Counter(names).items())),
            "urdf_part_nodes": part_links,
            "semantic_precision": None,
            "semantic_recall": None,
            "naming_richness": None,
            "functional_core_coverage": None,
            "instance_discriminability": None,
            "over_segmentation_rate": None,
        },
        (f"{artifact_source}/{factory}/seed_{seed:03d}", urdf),
    )


def evaluate_factory(
    factory: str,
    assets: list[dict[str, Any]],
    terminal_rows: list[dict[str, Any]],
) -> tuple[dict[str, Any], list[float], list[float]]:
    assets = sorted(assets, key=lambda item: item["seed"])
    set_scores: list[float] = []
    multiset_scores: list[float] = []
    for left, right in combinations(assets, 2):
        left_names = left["raw_part_names"]
        right_names = right["raw_part_names"]
        set_scores.append(set_jaccard(set(left_names), set(right_names)))
        multiset_scores.append(
            multiset_jaccard(Counter(left_names), Counter(right_names))
        )
    signatures = Counter(signature(item["raw_part_names"]) for item in assets)
    signature_rows = sorted(
        (
            {
                "signature_sha256": hashlib.sha256(
                    json.dumps(sig, separators=(",", ":")).encode("utf-8")
                ).hexdigest(),
                "seed_count": count,
                "part_count": sum(value for _, value in sig),
            }
            for sig, count in signatures.items()
        ),
        key=lambda item: (-item["seed_count"], item["signature_sha256"]),
    )
    expected_seeds = sorted(int(item["seed"]) for item in terminal_rows)
    evaluated_seeds = [int(item["seed"]) for item in assets]
    evaluated_seed_set = set(evaluated_seeds)
    omitted = [
        {"seed": int(item["seed"]), "status": str(item["status"])}
        for item in sorted(terminal_rows, key=lambda item: int(item["seed"]))
        if int(item["seed"]) not in evaluated_seed_set
    ]
    recovered = [
        int(item["seed"])
        for item in assets
        if item.get("artifact_source") == "extended_timeout_recovery"
    ]
    parts = [int(item["urdf_part_node_count"]) for item in assets]
    named_total = sum(int(item["named_urdf_part_node_count"]) for item in assets)
    part_total = sum(parts)
    return (
        {
            "factory": factory,
            "expected_seed_count": len(expected_seeds),
            "evaluated_pass_seed_count": len(evaluated_seeds),
            "evaluated_seeds": evaluated_seeds,
            "omitted_nonpass_seeds": omitted,
            "recovered_extended_timeout_seeds": recovered,
            "pair_count": len(set_scores),
            "parts_per_asset_mean": statistics.mean(parts),
            "parts_per_asset_min": min(parts),
            "parts_per_asset_max": max(parts),
            "urdf_part_node_total": part_total,
            "renderable_visual_geometry_total": sum(
                int(item["renderable_visual_geometry_count"]) for item in assets
            ),
            "nameability_micro": named_total / part_total,
            "opaque_generated_index_name_rate_micro": (
                sum(int(item["opaque_generated_index_name_count"]) for item in assets)
                / part_total
            ),
            "raw_unique_name_set_jaccard_pair_mean": statistics.mean(set_scores),
            "raw_name_multiset_weighted_jaccard_pair_mean": statistics.mean(
                multiset_scores
            ),
            "exact_raw_name_multiset_mode_rate": max(signatures.values()) / len(assets),
            "distinct_raw_name_multiset_count": len(signatures),
            "raw_name_multiset_modes": signature_rows,
        },
        set_scores,
        multiset_scores,
    )


def mean(values: list[float]) -> float:
    return statistics.mean(values)


def build_report(summary: dict[str, Any]) -> str:
    ci = summary["parts_per_asset_asset_bootstrap_95ci"]
    cluster_ci = summary["parts_per_asset_factory_cluster_bootstrap_95ci"]
    raw_ci = summary["raw_unique_name_set_jaccard_factory_macro_95ci"]
    multi_ci = summary["raw_name_multiset_weighted_jaccard_factory_macro_95ci"]
    mode_ci = summary["exact_raw_name_multiset_mode_rate_factory_macro_95ci"]
    return "\n".join(
        [
            "# Infinite Mobility Table 2 Naming direct evaluation",
            "",
            "This is a public-factory supplementary cohort, not a common-category matched result.",
            "",
            "## Frozen policy",
            "",
            "- One URDF part is a link with at least one valid renderable visual geometry (mesh, box, cylinder, or sphere under protocol v1.1).",
            "- Multiple renderable visuals on one link are merged into one part node; geometry-type and mesh-reference counts remain audit fields.",
            "- The shared Nano3D placeholder regex is used unchanged. Therefore `l_<index>` passes lexical Nameability even though it is separately flagged as an opaque generated name.",
            "- Original 300-second PASS packages enter directly; the seven original TIMEOUTs use separately recorded 900-second strict-PASS recovery packages only for Naming artifact coverage.",
            "- Cross-seed metrics use raw names only. They are not semantic role consistency.",
            "",
            "## Direct results",
            "",
            "| Metric | Result | Scope |",
            "|---|---:|---|",
            f"| Parts | {summary['parts_per_asset_mean']:.3f} [{ci[0]:.3f}, {ci[1]:.3f}] | asset bootstrap; {summary['urdf_part_node_total']}/{summary['evaluated_pass_asset_count']} renderable-geometry URDF links |",
            f"| Parts cluster sensitivity | [{cluster_ci[0]:.3f}, {cluster_ci[1]:.3f}] | factory then observed-seed bootstrap |",
            f"| Named / Nameability | {summary['nameability_micro']:.3f} | {summary['named_urdf_part_node_total']}/{summary['urdf_part_node_total']} |",
            f"| Opaque `l_<index>` names | {summary['opaque_generated_index_name_rate_micro']:.3f} | supplementary audit; lexical Nameability is not semantic readability |",
            f"| Raw unique-name set Jaccard | {summary['raw_unique_name_set_jaccard_pair_micro']:.3f} pair-micro / {summary['raw_unique_name_set_jaccard_factory_macro']:.3f} factory-macro / {summary['raw_unique_name_set_jaccard_factory_median']:.3f} median [{raw_ci[0]:.3f}, {raw_ci[1]:.3f}] | within factory |",
            f"| Raw name-multiset weighted Jaccard | {summary['raw_name_multiset_weighted_jaccard_pair_micro']:.3f} pair-micro / {summary['raw_name_multiset_weighted_jaccard_factory_macro']:.3f} factory-macro / {summary['raw_name_multiset_weighted_jaccard_factory_median']:.3f} median [{multi_ci[0]:.3f}, {multi_ci[1]:.3f}] | within factory |",
            f"| Exact raw-name-multiset mode rate | {summary['exact_raw_name_multiset_mode_rate_factory_macro']:.3f} factory-macro / {summary['exact_raw_name_multiset_mode_rate_factory_median']:.3f} median [{mode_ci[0]:.3f}, {mode_ci[1]:.3f}] | modal signature frequency / PASS seeds, then factory mean |",
            "| Semantic Precision / Recall | N/A | no independent semantic gold or judges |",
            "| Naming Richness | N/A | no independent semantic role inventory |",
            "| Functional / Instance / Over-Segmentation | N/A | no independent functional, instance, or decomposition gold |",
            "",
            "## Coverage and audit",
            "",
            f"- Original 300-second reliability: {summary['source_300s_pass_asset_count']}/{summary['source_terminal_case_count']} PASS; Naming-evaluable after recovery overlay: {summary['evaluated_pass_asset_count']}/{summary['source_terminal_case_count']}; recovered: {summary['recovery_overlay_asset_count']}.",
            f"- Factories: {summary['factory_count']}; within-factory raw-name pairs: {summary['within_factory_pair_count']}.",
            f"- URDF part nodes: {summary['urdf_part_node_total']}; renderable visuals: {summary['renderable_visual_geometry_total']} (mesh {summary['mesh_visual_geometry_total']}, primitives {summary['primitive_visual_geometry_total']}); multi-visual links: {summary['multi_visual_part_node_total']}.",
            f"- Invalid or unsupported visual geometries excluded: {summary['invalid_or_unsupported_visual_geometry_total']}.",
            f"- Bootstrap: {summary['bootstrap_resamples']} deterministic resamples, seed {summary['bootstrap_seed']}.",
            "",
        ]
    )


def build_recovery_manifest(
    recovery_root: Path, source_records: list[dict[str, Any]]
) -> tuple[dict[str, Any], dict[tuple[str, int], dict[str, Any]]]:
    expected = {
        (str(row["factory"]), int(row["seed"])): row
        for row in source_records
        if row["status"] != "PASS"
    }
    recovered: dict[tuple[str, int], dict[str, Any]] = {}
    cases = []
    attempts = []
    for pair, original in sorted(expected.items()):
        factory, seed = pair
        case_dir = recovery_root / "cases" / factory / f"seed_{seed:03d}"
        record_path = case_dir / "record.json"
        if not record_path.is_file():
            raise RuntimeError(f"missing recovery record: {record_path}")
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if record.get("status") != "PASS" or not record.get("validation", {}).get("strict_pass"):
            raise RuntimeError(f"recovery case is not strict PASS: {factory} seed {seed}")
        recovered[pair] = record
        cases.append(
            {
                "factory": factory,
                "seed": seed,
                "original_status": original["status"],
                "original_elapsed_seconds": original["elapsed_seconds"],
                "recovery_status": record["status"],
                "recovery_elapsed_seconds": record["elapsed_seconds"],
                "recovery_record": record_path.relative_to(REPO_ROOT).as_posix(),
                "recovery_record_sha256": sha256_file(record_path),
                "package_sha256": record["package_sha256"],
            }
        )
        attempts_dir = case_dir / "attempts"
        if not attempts_dir.is_dir():
            continue
        for attempt in sorted(attempts_dir.iterdir()):
            if not attempt.is_dir():
                continue
            prior_record = attempt / "record.json"
            stderr_path = attempt / "stderr.log"
            if prior_record.is_file():
                prior = json.loads(prior_record.read_text(encoding="utf-8"))
                stderr = stderr_path.read_text(encoding="utf-8", errors="replace") if stderr_path.is_file() else ""
                if "FileNotFoundError" in stderr:
                    classification = "HARNESS_PATH_ERROR"
                elif "Resource temporarily unavailable" in stderr:
                    classification = "HOST_THREAD_RESOURCE_ERROR"
                else:
                    classification = "RECOVERY_ATTEMPT_NONPASS"
                attempts.append(
                    {
                        "factory": factory,
                        "seed": seed,
                        "attempt": attempt.name,
                        "classification": classification,
                        "recorded_status": prior.get("status"),
                        "process_exit_code": prior.get("validation", {}).get("process_exit_code"),
                        "record_sha256": sha256_file(prior_record),
                        "stderr_sha256": sha256_file(stderr_path) if stderr_path.is_file() else None,
                    }
                )
            elif attempt.name.startswith("orphan_staging_"):
                attempts.append(
                    {
                        "factory": factory,
                        "seed": seed,
                        "attempt": attempt.name,
                        "classification": "INTERRUPTED_ORPHAN_STAGING",
                        "recorded_status": None,
                        "file_count": sum(item.is_file() for item in attempt.rglob("*")),
                    }
                )
    manifest = {
        "protocol_id": "nano3d_infinite_mobility_timeout_recovery_v1",
        "policy": {
            "original_300s_status_is_immutable": True,
            "recovery_timeout_seconds": 900,
            "worker_concurrency": 1,
            "gpu_policy": "CUDA hidden; Blender CPU only",
            "naming_overlay_rule": "strict PASS recovery package replaces only the missing Naming artifact; it does not rewrite original 300s reliability",
            "window_seed_29_retry": "16-CPU affinity after a host thread-resource abort; geometry, seed, and adapter unchanged",
        },
        "expected_recovery_case_count": len(expected),
        "strict_pass_recovery_case_count": len(recovered),
        "all_recovery_cases_strict_pass": len(recovered) == len(expected),
        "cases": cases,
        "non_method_attempts": attempts,
    }
    write_json(recovery_root / "recovery_manifest.json", manifest)
    write_json(recovery_root / "recovery_records.json", cases)
    write_json(
        recovery_root / "recovery_status.json",
        {
            "protocol_id": manifest["protocol_id"],
            "status": "COMPLETE",
            "strict_pass_recovery_case_count": len(recovered),
            "expected_recovery_case_count": len(expected),
            "original_300s_reliability_rewritten": False,
            "naming_artifact_coverage_after_overlay": len(source_records),
            "non_method_attempt_count": len(attempts),
            "non_method_attempt_classifications": dict(
                sorted(Counter(item["classification"] for item in attempts).items())
            ),
        },
    )
    (recovery_root / "recovery_report.md").write_text(
        "\n".join(
            [
                "# Infinite Mobility timeout recovery",
                "",
                f"- Strict recovery PASS: {len(recovered)}/{len(expected)}",
                "- Original 300-second reliability records remain unchanged.",
                "- Recovery packages are used only to complete Naming artifact coverage.",
                f"- Non-method attempts retained: {len(attempts)}.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return manifest, recovered


def category_cluster_bootstrap_ci(
    grouped: dict[str, list[dict[str, Any]]],
    *,
    numerator: str,
    denominator: str | None,
    resamples: int,
    seed: int,
) -> list[float]:
    rng = random.Random(seed)
    categories = sorted(grouped)
    boot = []
    for _ in range(resamples):
        selected = [rng.choice(categories) for _ in categories]
        rows = [row for category in selected for row in grouped[category]]
        if denominator is None:
            boot.append(statistics.mean(float(row[numerator]) for row in rows))
        else:
            den = sum(int(row[denominator]) for row in rows)
            boot.append(sum(int(row[numerator]) for row in rows) / den)
    boot.sort()
    return [percentile(boot, 0.025), percentile(boot, 0.975)]


def build_matched_outputs(
    output: Path,
    asset_records: list[dict[str, Any]],
    matched_protocol: dict[str, Any],
    matched_protocol_path: Path,
    common_protocol_path: Path,
) -> tuple[dict[str, Any], list[Path]]:
    matched_output = output / "matched35"
    matched_output.mkdir(parents=True, exist_ok=True)
    by_pair = {(row["factory"], int(row["seed"])): row for row in asset_records}
    seeds = [int(item) for item in matched_protocol["selection"]["infinite_mobility"]["seeds"]]
    grouped: dict[str, list[dict[str, Any]]] = {}
    selected_records = []
    category_records = []
    all_set_scores: list[float] = []
    all_multiset_scores: list[float] = []
    for category in matched_protocol["design"]["canonical_categories"]:
        factory = str(matched_protocol["taxonomy"][category]["infinite_mobility"])
        rows = []
        for seed_value in seeds:
            source = by_pair.get((factory, seed_value))
            if source is None:
                raise RuntimeError(f"matched asset missing: {factory} seed {seed_value}")
            row = {**source, "canonical_category": category}
            rows.append(row)
            selected_records.append(row)
        grouped[category] = rows
        synthetic = [{**row, "factory": category} for row in rows]
        terminal = [{"seed": seed_value, "status": "PASS"} for seed_value in seeds]
        category_result, set_scores, multiset_scores = evaluate_factory(
            category, synthetic, terminal
        )
        category_result["source_factory"] = factory
        category_records.append(category_result)
        all_set_scores.extend(set_scores)
        all_multiset_scores.extend(multiset_scores)

    part_total = sum(int(row["urdf_part_node_count"]) for row in selected_records)
    named_total = sum(int(row["named_urdf_part_node_count"]) for row in selected_records)
    opaque_total = sum(int(row["opaque_generated_index_name_count"]) for row in selected_records)
    set_means = [float(row["raw_unique_name_set_jaccard_pair_mean"]) for row in category_records]
    multiset_means = [float(row["raw_name_multiset_weighted_jaccard_pair_mean"]) for row in category_records]
    mode_rates = [float(row["exact_raw_name_multiset_mode_rate"]) for row in category_records]
    samples = int(matched_protocol["bootstrap"]["resamples"])
    seed_value = int(matched_protocol["bootstrap"]["seed"])
    null_metrics = {
        "semantic_precision": None,
        "semantic_recall": None,
        "naming_richness": None,
        "functional_core_coverage": None,
        "instance_discriminability": None,
        "over_segmentation_rate": None,
    }
    summary = {
        "protocol_id": matched_protocol["protocol_id"],
        "method": "Infinite Mobility",
        "comparison_label": matched_protocol["comparison_label"],
        "prohibited_label": matched_protocol["prohibited_label"],
        "canonical_categories": matched_protocol["design"]["canonical_categories"],
        "category_count": len(grouped),
        "assets_per_category": len(seeds),
        "asset_count": len(selected_records),
        "selected_seeds": seeds,
        "urdf_part_node_total": part_total,
        "parts_per_asset_mean": part_total / len(selected_records),
        "parts_per_asset_category_cluster_bootstrap_95ci": category_cluster_bootstrap_ci(
            grouped, numerator="urdf_part_node_count", denominator=None,
            resamples=samples, seed=seed_value,
        ),
        "named_urdf_part_node_total": named_total,
        "nameability_micro": named_total / part_total,
        "nameability_category_cluster_bootstrap_95ci": category_cluster_bootstrap_ci(
            grouped, numerator="named_urdf_part_node_count", denominator="urdf_part_node_count",
            resamples=samples, seed=seed_value + 1,
        ),
        "placeholder_urdf_part_node_total": part_total - named_total,
        "opaque_generated_index_name_total": opaque_total,
        "opaque_generated_index_name_rate_micro": opaque_total / part_total,
        "within_category_pair_count": len(all_set_scores),
        "raw_unique_name_set_jaccard_pair_micro": statistics.mean(all_set_scores),
        "raw_unique_name_set_jaccard_category_macro": statistics.mean(set_means),
        "raw_unique_name_set_jaccard_category_median": statistics.median(set_means),
        "raw_unique_name_set_jaccard_category_macro_95ci": bootstrap_mean_ci(
            set_means, resamples=samples, seed=seed_value + 2
        ),
        "raw_name_multiset_weighted_jaccard_pair_micro": statistics.mean(all_multiset_scores),
        "raw_name_multiset_weighted_jaccard_category_macro": statistics.mean(multiset_means),
        "raw_name_multiset_weighted_jaccard_category_median": statistics.median(multiset_means),
        "raw_name_multiset_weighted_jaccard_category_macro_95ci": bootstrap_mean_ci(
            multiset_means, resamples=samples, seed=seed_value + 3
        ),
        "exact_raw_name_multiset_mode_rate_category_macro": statistics.mean(mode_rates),
        "exact_raw_name_multiset_mode_rate_category_median": statistics.median(mode_rates),
        "exact_raw_name_multiset_mode_rate_category_macro_95ci": bootstrap_mean_ci(
            mode_rates, resamples=samples, seed=seed_value + 4
        ),
        "semantic_metrics_status": "N/A: no independent semantic gold and three blind judges",
        "bootstrap_resamples": samples,
        "bootstrap_seed": seed_value,
        **null_metrics,
    }
    asset_path = matched_output / "asset_records.jsonl"
    asset_path.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in selected_records),
        encoding="utf-8",
    )
    category_path = matched_output / "category_records.json"
    summary_path = matched_output / "summary.json"
    protocol_path = matched_output / "protocol.json"
    report_path = matched_output / "report.md"
    write_json(category_path, category_records)
    write_json(summary_path, summary)
    write_json(
        protocol_path,
        {
            "protocol_id": matched_protocol["protocol_id"],
            "protocol_path": matched_protocol_path.relative_to(REPO_ROOT).as_posix(),
            "protocol_sha256": sha256_file(matched_protocol_path),
            "base_naming_protocol_sha256": sha256_file(common_protocol_path),
            "selection": {
                category: {
                    "factory": matched_protocol["taxonomy"][category]["infinite_mobility"],
                    "seeds": seeds,
                }
                for category in matched_protocol["design"]["canonical_categories"]
            },
            "label_rule": "category-matched deterministic cohort; never same-prompt or cross-method same-seed",
        },
    )
    parts_ci = summary["parts_per_asset_category_cluster_bootstrap_95ci"]
    raw_ci = summary["raw_unique_name_set_jaccard_category_macro_95ci"]
    report_path.write_text(
        "\n".join(
            [
                "# Infinite Mobility category-matched Naming cohort",
                "",
                "This is a category-matched deterministic cohort, not a same-prompt or cross-method same-seed comparison.",
                "",
                f"- Coverage: {summary['asset_count']} assets = {summary['category_count']} categories x {summary['assets_per_category']} official factory seeds.",
                f"- Parts: {summary['parts_per_asset_mean']:.3f} per asset, category-cluster 95% CI [{parts_ci[0]:.3f}, {parts_ci[1]:.3f}].",
                f"- Nameability: {summary['nameability_micro']:.3f} ({named_total}/{part_total}); opaque `l_<index>` rate {summary['opaque_generated_index_name_rate_micro']:.3f}.",
                f"- Raw unique-name set Jaccard: {summary['raw_unique_name_set_jaccard_pair_micro']:.3f} pair-micro / {summary['raw_unique_name_set_jaccard_category_macro']:.3f} category-macro [{raw_ci[0]:.3f}, {raw_ci[1]:.3f}].",
                "- Semantic Precision/Recall, Richness, Functional, Instance and Over-Segmentation: N/A without independent gold and three blind judges.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    artifacts = [asset_path, category_path, summary_path, protocol_path, report_path]
    selected_urdfs = [
        (f"{row['canonical_category']}/{row['seed']}", REPO_ROOT / row["urdf_path"])
        for row in selected_records
    ]
    self_check = {
        "all_checks_pass": True,
        "checks": {
            "exact_five_by_seven": len(grouped) == 5 and all(len(rows) == 7 for rows in grouped.values()),
            "seeds_exact_zero_through_six": seeds == list(range(7)),
            "all_selected_artifacts_directly_evaluable": all(row["urdf_part_node_count"] > 0 for row in selected_records),
            "all_selected_from_frozen_factory_mapping": all(
                row["factory"] == matched_protocol["taxonomy"][row["canonical_category"]]["infinite_mobility"]
                for row in selected_records
            ),
            "all_semantic_metrics_null": all(summary[key] is None for key in null_metrics),
        },
        "input_hashes": {
            "matched_protocol_sha256": sha256_file(matched_protocol_path),
            "base_naming_protocol_sha256": sha256_file(common_protocol_path),
            "selected_urdf_cohort_sha256": cohort_sha256(selected_urdfs),
        },
        "artifact_hashes": {path.name: sha256_file(path) for path in artifacts},
    }
    self_check["all_checks_pass"] = all(self_check["checks"].values())
    payload = json.dumps(self_check, sort_keys=True, separators=(",", ":")).encode("utf-8")
    self_check["reproduction_digest_sha256"] = hashlib.sha256(payload).hexdigest()
    self_check_path = matched_output / "self_check.json"
    write_json(self_check_path, self_check)
    if not self_check["all_checks_pass"]:
        raise RuntimeError("matched cohort self-check failed")
    return summary, artifacts + [self_check_path]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--common-protocol", type=Path, default=DEFAULT_COMMON_PROTOCOL)
    parser.add_argument("--recovery-input", type=Path, default=DEFAULT_RECOVERY)
    parser.add_argument("--matched-protocol", type=Path, default=DEFAULT_MATCHED_PROTOCOL)
    parser.add_argument("--bootstrap-resamples", type=int, default=BOOTSTRAP_RESAMPLES)
    parser.add_argument("--bootstrap-seed", type=int, default=BOOTSTRAP_SEED)
    args = parser.parse_args()

    baseline_root = require_within(args.input, REPO_ROOT, "input")
    output = require_within(args.output, REPO_ROOT, "output")
    protocol_path = require_within(args.protocol, REPO_ROOT, "protocol")
    common_protocol_path = require_within(
        args.common_protocol, REPO_ROOT, "common protocol"
    )
    recovery_root = require_within(args.recovery_input, REPO_ROOT, "recovery input")
    matched_protocol_path = require_within(
        args.matched_protocol, REPO_ROOT, "matched protocol"
    )
    output.mkdir(parents=True, exist_ok=True)

    source_records_path = baseline_root / "records.json"
    source_summary_path = baseline_root / "summary.json"
    source_records = json.loads(source_records_path.read_text(encoding="utf-8"))
    source_summary = json.loads(source_summary_path.read_text(encoding="utf-8"))
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    common_protocol = json.loads(common_protocol_path.read_text(encoding="utf-8"))
    matched_protocol = json.loads(matched_protocol_path.read_text(encoding="utf-8"))
    if common_protocol["protocol_id"] != "nano3d_table2_baseline_naming_v1.1":
        raise RuntimeError("expected shared baseline Naming protocol v1.1")
    expected_geometry_keys = {
        "mesh", "box", "cylinder", "sphere", "unsupported_or_invalid"
    }
    if set(common_protocol["units"]["urdf_renderable_geometry"]) != expected_geometry_keys:
        raise RuntimeError("shared renderable-geometry contract has unexpected fields")
    if common_protocol["nameability"]["placeholder_regex"] != PLACEHOLDER_RE.pattern:
        raise RuntimeError("evaluator placeholder regex differs from common protocol")
    if int(common_protocol["bootstrap"]["resamples"]) != args.bootstrap_resamples:
        raise RuntimeError("bootstrap resamples differ from common protocol")
    if int(common_protocol["bootstrap"]["seed"]) != args.bootstrap_seed:
        raise RuntimeError("bootstrap seed differs from common protocol")
    if matched_protocol["protocol_id"] != "nano3d_table2_naming_matched_v1":
        raise RuntimeError("unexpected matched Naming protocol")
    if matched_protocol["base_naming_protocol"]["sha256"] != sha256_file(common_protocol_path):
        raise RuntimeError("matched protocol base Naming hash mismatch")
    matched_source = matched_protocol["source_evidence"]["infinite_mobility_naming_records"]
    matched_source_path = require_within(REPO_ROOT / matched_source["path"], REPO_ROOT, "matched source evidence")
    matched_source_archive = output / "source_evidence_asset_records_v1.jsonl"
    if matched_source_archive.is_file():
        if sha256_file(matched_source_archive) != matched_source["sha256"]:
            raise RuntimeError("archived matched source evidence hash mismatch")
    else:
        if sha256_file(matched_source_path) != matched_source["sha256"]:
            raise RuntimeError("matched source evidence hash mismatch before overlay")
        matched_source_archive.write_bytes(matched_source_path.read_bytes())
    factories = [str(item) for item in protocol["factories"]]
    expected_seeds = [int(item) for item in protocol["seeds"]]

    expected_pairs = {(factory, seed) for factory in factories for seed in expected_seeds}
    observed_pairs = {(str(row["factory"]), int(row["seed"])) for row in source_records}
    if len(source_records) != len(observed_pairs) or observed_pairs != expected_pairs:
        raise RuntimeError("source terminal records do not exactly match frozen factory/seed grid")
    source_status_counts = Counter(str(row["status"]) for row in source_records)
    if source_status_counts != Counter(source_summary["status_counts"]):
        raise RuntimeError("records.json status counts differ from source summary.json")

    recovery_manifest, recovery_records = build_recovery_manifest(
        recovery_root, source_records
    )

    asset_records: list[dict[str, Any]] = []
    urdf_inputs: list[tuple[str, Path]] = []
    for row in sorted(source_records, key=lambda item: (item["factory"], item["seed"])):
        pair = (str(row["factory"]), int(row["seed"]))
        if row["status"] == "PASS":
            artifact_root = baseline_root
            artifact_row = row
            artifact_source = "original_300s_pass"
        else:
            artifact_root = recovery_root
            artifact_row = recovery_records[pair]
            artifact_source = "extended_timeout_recovery"
        asset, urdf_input = evaluate_asset(
            artifact_root,
            artifact_row,
            original_status=str(row["status"]),
            artifact_source=artifact_source,
        )
        asset_records.append(asset)
        urdf_inputs.append(urdf_input)

    grouped_assets: dict[str, list[dict[str, Any]]] = defaultdict(list)
    grouped_terminal: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in asset_records:
        grouped_assets[row["factory"]].append(row)
    for row in source_records:
        grouped_terminal[str(row["factory"])].append(row)

    factory_records: list[dict[str, Any]] = []
    all_set_scores: list[float] = []
    all_multiset_scores: list[float] = []
    for factory in factories:
        result, set_scores, multiset_scores = evaluate_factory(
            factory, grouped_assets[factory], grouped_terminal[factory]
        )
        factory_records.append(result)
        all_set_scores.extend(set_scores)
        all_multiset_scores.extend(multiset_scores)

    parts = [int(row["urdf_part_node_count"]) for row in asset_records]
    factory_parts = {
        factory: [int(row["urdf_part_node_count"]) for row in grouped_assets[factory]]
        for factory in factories
    }
    set_factory_means = [
        float(row["raw_unique_name_set_jaccard_pair_mean"]) for row in factory_records
    ]
    multiset_factory_means = [
        float(row["raw_name_multiset_weighted_jaccard_pair_mean"])
        for row in factory_records
    ]
    exact_mode_rates = [
        float(row["exact_raw_name_multiset_mode_rate"]) for row in factory_records
    ]
    part_total = sum(parts)
    named_total = sum(int(row["named_urdf_part_node_count"]) for row in asset_records)
    opaque_total = sum(int(row["opaque_generated_index_name_count"]) for row in asset_records)
    summary = {
        "protocol_id": "nano3d_infinite_mobility_naming_direct_v1.1",
        "source_protocol_id": protocol["protocol_id"],
        "cohort_scope": "public-factory supplementary; not common-category matched",
        "source_terminal_case_count": len(source_records),
        "source_status_counts": dict(sorted(source_status_counts.items())),
        "source_300s_pass_asset_count": source_status_counts["PASS"],
        "recovery_overlay_asset_count": sum(
            row["artifact_source"] == "extended_timeout_recovery"
            for row in asset_records
        ),
        "evaluated_pass_asset_count": len(asset_records),
        "omitted_nonpass_asset_count": len(source_records) - len(asset_records),
        "factory_count": len(factory_records),
        "urdf_part_node_policy": "one URDF link with >=1 valid renderable visual geometry; multiple visuals on one link merge to one node",
        "placeholder_regex": PLACEHOLDER_RE.pattern,
        "urdf_part_node_total": part_total,
        "renderable_visual_geometry_total": sum(
            int(row["renderable_visual_geometry_count"]) for row in asset_records
        ),
        "mesh_visual_geometry_total": sum(
            int(row["mesh_visual_geometry_count"]) for row in asset_records
        ),
        "primitive_visual_geometry_total": sum(
            int(row["primitive_visual_geometry_count"]) for row in asset_records
        ),
        "invalid_or_unsupported_visual_geometry_total": sum(
            int(row["invalid_or_unsupported_visual_geometry_count"])
            for row in asset_records
        ),
        "mesh_reference_total": sum(int(row["mesh_reference_count"]) for row in asset_records),
        "multi_visual_part_node_total": sum(
            int(row["multi_visual_part_node_count"]) for row in asset_records
        ),
        "parts_per_asset_mean": mean([float(value) for value in parts]),
        "parts_per_asset_asset_bootstrap_95ci": bootstrap_mean_ci(
            [float(value) for value in parts],
            resamples=args.bootstrap_resamples,
            seed=args.bootstrap_seed,
        ),
        "parts_per_asset_factory_cluster_bootstrap_95ci": cluster_bootstrap_parts_ci(
            factory_parts,
            resamples=args.bootstrap_resamples,
            seed=args.bootstrap_seed + 1,
        ),
        "named_urdf_part_node_total": named_total,
        "nameability_micro": named_total / part_total,
        "opaque_generated_index_name_total": opaque_total,
        "opaque_generated_index_name_rate_micro": opaque_total / part_total,
        "within_factory_pair_count": len(all_set_scores),
        "raw_unique_name_set_jaccard_pair_micro": mean(all_set_scores),
        "raw_unique_name_set_jaccard_factory_macro": mean(set_factory_means),
        "raw_unique_name_set_jaccard_factory_median": statistics.median(set_factory_means),
        "raw_unique_name_set_jaccard_factory_macro_95ci": bootstrap_mean_ci(
            set_factory_means,
            resamples=args.bootstrap_resamples,
            seed=args.bootstrap_seed + 2,
        ),
        "raw_name_multiset_weighted_jaccard_pair_micro": mean(all_multiset_scores),
        "raw_name_multiset_weighted_jaccard_factory_macro": mean(multiset_factory_means),
        "raw_name_multiset_weighted_jaccard_factory_median": statistics.median(
            multiset_factory_means
        ),
        "raw_name_multiset_weighted_jaccard_factory_macro_95ci": bootstrap_mean_ci(
            multiset_factory_means,
            resamples=args.bootstrap_resamples,
            seed=args.bootstrap_seed + 3,
        ),
        "exact_raw_name_multiset_mode_rate_factory_macro": mean(exact_mode_rates),
        "exact_raw_name_multiset_mode_rate_factory_median": statistics.median(
            exact_mode_rates
        ),
        "exact_raw_name_multiset_mode_rate_factory_macro_95ci": bootstrap_mean_ci(
            exact_mode_rates,
            resamples=args.bootstrap_resamples,
            seed=args.bootstrap_seed + 4,
        ),
        "semantic_precision": None,
        "semantic_recall": None,
        "naming_richness": None,
        "functional_core_coverage": None,
        "instance_discriminability": None,
        "over_segmentation_rate": None,
        "semantic_metrics_status": "N/A: no independent semantic gold or judges",
        "bootstrap_resamples": args.bootstrap_resamples,
        "bootstrap_seed": args.bootstrap_seed,
    }

    policy = {
        "protocol_id": summary["protocol_id"],
        "common_protocol_id": common_protocol["protocol_id"],
        "common_protocol": common_protocol_path.relative_to(REPO_ROOT).as_posix(),
        "input_protocol": protocol_path.relative_to(REPO_ROOT).as_posix(),
        "input_runtime": baseline_root.relative_to(REPO_ROOT).as_posix(),
        "recovery_runtime": recovery_root.relative_to(REPO_ROOT).as_posix(),
        "part_count_unit": "renderable-visual-geometry URDF link/node",
        "renderable_geometry_definition": common_protocol["units"]["urdf_renderable_geometry"],
        "multiple_visual_policy": "merge all qualifying renderable visuals on one link into one part/node; retain geometry-type and mesh-reference audit counts",
        "name_source": "URDF link name",
        "placeholder_regex": PLACEHOLDER_RE.pattern,
        "artifact_filter": "original 300s strict PASS plus separately recorded 900s strict-PASS recovery overlay; original reliability remains immutable",
        "cross_seed_metrics": {
            "raw_unique_name_set_jaccard": "intersection/union of exact Naming-evaluable URDF part-name sets for each within-factory PASS-seed pair",
            "raw_name_multiset_weighted_jaccard": "sum per-name minimum count / sum per-name maximum count for each within-factory PASS-seed pair",
            "exact_raw_name_multiset_mode_rate": "largest identical exact-name-multiset group / evaluated PASS seeds per factory",
            "aggregation": "pair-micro pools within-factory seed pairs; factory-macro averages factory pair means; 95% CI bootstraps 20 factory values",
        },
        "unsupported_without_independent_gold_or_judges": [
            "semantic_precision",
            "semantic_recall",
            "naming_richness",
            "functional_core_coverage",
            "instance_discriminability",
            "over_segmentation_rate",
        ],
    }

    asset_path = output / "asset_records.jsonl"
    asset_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n"
            for row in asset_records
        ),
        encoding="utf-8",
    )
    factory_path = output / "factory_records.json"
    summary_path = output / "summary.json"
    policy_path = output / "protocol.json"
    report_path = output / "report.md"
    write_json(factory_path, factory_records)
    write_json(summary_path, summary)
    write_json(policy_path, policy)
    report_path.write_text(build_report(summary), encoding="utf-8")

    matched_summary, matched_artifacts = build_matched_outputs(
        output,
        asset_records,
        matched_protocol,
        matched_protocol_path,
        common_protocol_path,
    )

    artifacts = [asset_path, factory_path, summary_path, policy_path, report_path]
    self_check = {
        "all_checks_pass": True,
        "checks": {
            "terminal_grid_exact": observed_pairs == expected_pairs,
            "terminal_record_count_matches_protocol": len(source_records) == len(expected_pairs),
            "source_status_counts_match_summary": source_status_counts == Counter(source_summary["status_counts"]),
            "recovery_overlay_complete": recovery_manifest["all_recovery_cases_strict_pass"],
            "evaluated_count_matches_full_grid": len(asset_records) == len(expected_pairs),
            "original_reliability_unchanged": source_status_counts == Counter({"PASS": 713, "TIMEOUT": 7}),
            "matched_cohort_exact_five_by_seven": matched_summary["asset_count"] == 35 and matched_summary["category_count"] == 5,
            "all_factories_have_at_least_two_pass_seeds": all(len(grouped_assets[item]) >= 2 for item in factories),
            "all_part_nodes_nonempty": all(row["urdf_part_node_count"] > 0 for row in asset_records),
            "node_count_not_greater_than_visual_count": all(
                row["urdf_part_node_count"] <= row["renderable_visual_geometry_count"]
                for row in asset_records
            ),
            "infinite_mobility_visuals_are_all_valid_meshes": all(
                row["primitive_visual_geometry_count"] == 0
                and row["invalid_or_unsupported_visual_geometry_count"] == 0
                and row["mesh_visual_geometry_count"]
                == row["renderable_visual_geometry_count"]
                for row in asset_records
            ),
            "metric_ranges_valid": all(
                0.0 <= value <= 1.0
                for value in [
                    summary["nameability_micro"],
                    summary["opaque_generated_index_name_rate_micro"],
                    summary["raw_unique_name_set_jaccard_pair_micro"],
                    summary["raw_unique_name_set_jaccard_factory_macro"],
                    summary["raw_name_multiset_weighted_jaccard_pair_micro"],
                    summary["raw_name_multiset_weighted_jaccard_factory_macro"],
                    summary["exact_raw_name_multiset_mode_rate_factory_macro"],
                ]
            ),
            "semantic_metrics_remain_null": all(
                summary[key] is None
                for key in [
                    "semantic_precision",
                    "semantic_recall",
                    "naming_richness",
                    "functional_core_coverage",
                    "instance_discriminability",
                    "over_segmentation_rate",
                ]
            ),
        },
        "input_hashes": {
            "source_records_sha256": sha256_file(source_records_path),
            "source_summary_sha256": sha256_file(source_summary_path),
            "source_protocol_sha256": sha256_file(protocol_path),
            "common_protocol_sha256": sha256_file(common_protocol_path),
            "matched_protocol_sha256": sha256_file(matched_protocol_path),
            "matched_source_evidence_sha256": sha256_file(matched_source_archive),
            "recovery_manifest_sha256": sha256_file(recovery_root / "recovery_manifest.json"),
            "source_urdf_cohort_sha256": cohort_sha256(urdf_inputs),
            "evaluator_script_sha256": sha256_file(Path(__file__).resolve()),
        },
        "artifact_hashes": {
            path.name: sha256_file(path) for path in artifacts
        } | {
            f"matched35/{path.name}": sha256_file(path) for path in matched_artifacts
        } | {
            f"recovery/{path.name}": sha256_file(path)
            for path in [
                recovery_root / "recovery_manifest.json",
                recovery_root / "recovery_records.json",
                recovery_root / "recovery_status.json",
                recovery_root / "recovery_report.md",
            ]
        },
    }
    self_check["all_checks_pass"] = all(self_check["checks"].values())
    digest_payload = json.dumps(
        {
            "input_hashes": self_check["input_hashes"],
            "artifact_hashes": self_check["artifact_hashes"],
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    self_check["reproduction_digest_sha256"] = hashlib.sha256(digest_payload).hexdigest()
    write_json(output / "self_check.json", self_check)
    if not self_check["all_checks_pass"]:
        raise RuntimeError("one or more self-checks failed")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
