#!/usr/bin/env python3
"""Render the complete PV-A Table 1/2/2-supplementary/3/4 results."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import render_table4_full_release_results as table4_renderer
import check_pva_table1234_full_release as pva_checker
import audit_pva_table1_topologies as topology_auditor
import table123_full_release_common as common


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _count(value: Any) -> str:
    return f"{int(value):,}"


def _percent(rate: Any, digits: int = 2) -> str:
    if rate is None:
        return "N/E"
    return f"{100.0 * float(rate):.{digits}f}%"


def _fraction(value: Mapping[str, Any], *, digits: int = 2) -> str:
    passed = value.get("passed", value.get("numerator"))
    denominator = value.get("denominator")
    if passed is None or denominator is None:
        return str(value.get("status", "N/E"))
    denominator_int = int(denominator)
    rate = float(passed) / denominator_int if denominator_int else None
    return f"{_count(passed)} / {_count(denominator)} ({_percent(rate, digits)})"


def _distribution(value: Mapping[str, Any]) -> str:
    denominator = int(value.get("denominator", 0) or 0)
    if not denominator or any(
        value.get(field) is None for field in ("mean", "median", "p90_nearest_rank")
    ):
        return f"N/E / N/E / N/E (n={_count(denominator)})"
    return (
        f"{float(value['mean']):.2f} / {value['median']} / {value['p90_nearest_rank']} "
        f"(n={_count(denominator)})"
    )


def _topology_views(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Separate the legacy pooled signature ratio from within-category views."""

    pooled = summary.get("unique_topologies")
    breakdown = summary.get("category_breakdown")
    if not isinstance(pooled, Mapping) or not isinstance(breakdown, Mapping):
        raise ValueError("Table 1 summary is missing topology aggregates")

    conditioned_unique = 0
    conditioned_denominator = 0
    category_rates: list[float] = []
    for category, metrics in breakdown.items():
        if not isinstance(metrics, Mapping):
            raise ValueError(f"invalid category metrics: {category!r}")
        topology = metrics.get("unique_topologies")
        if not isinstance(topology, Mapping):
            raise ValueError(f"missing category topology metrics: {category!r}")
        unique = int(topology.get("unique", 0) or 0)
        denominator = int(topology.get("denominator", 0) or 0)
        if denominator < 0 or unique < 0 or unique > denominator:
            raise ValueError(f"invalid category topology counts: {category!r}")
        conditioned_unique += unique
        conditioned_denominator += denominator
        if denominator:
            calculated = unique / denominator
            reported = topology.get("rate")
            if reported is None or not math.isclose(
                float(reported), calculated, rel_tol=0.0, abs_tol=1e-15
            ):
                raise ValueError(f"category topology rate mismatch: {category!r}")
            category_rates.append(calculated)

    pooled_denominator = int(pooled.get("denominator", 0) or 0)
    if conditioned_denominator != pooled_denominator:
        raise ValueError("category topology denominator does not match pooled denominator")
    macro_rate = (
        sum(category_rates) / len(category_rates) if category_rates else None
    )
    reported_macro = (summary.get("category_macro") or {}).get(
        "unique_topologies_rate"
    )
    if macro_rate is not None and (
        reported_macro is None
        or not math.isclose(
            float(reported_macro), macro_rate, rel_tol=0.0, abs_tol=1e-15
        )
    ):
        raise ValueError("category-macro topology rate mismatch")
    return {
        "pooled_unique": int(pooled.get("unique", 0) or 0),
        "pooled_denominator": pooled_denominator,
        "pooled_rate": pooled.get("rate"),
        "conditioned_unique": conditioned_unique,
        "conditioned_denominator": conditioned_denominator,
        "conditioned_rate": (
            conditioned_unique / conditioned_denominator
            if conditioned_denominator
            else None
        ),
        "category_macro_rate": macro_rate,
        "evaluable_categories": len(category_rates),
    }


def _table1(
    summary: Mapping[str, Any],
    topology_audit: Mapping[str, Any] | None = None,
) -> list[str]:
    cohort = summary["cohort"]
    topology = _topology_views(summary)
    lines = [
        "## Table 1. Dataset Scale and Structural Diversity",
        "",
        "> The legacy pooled raw-tree ratio is a cohort-size-dependent support descriptor, not a higher-is-better diversity score. Cross-method topology claims require a shared category set and equal per-category budget.",
        "",
        "| Dataset | N_release | N_eval | Raw categories (release / eval) | Links / asset (mean / median / P90) | Movable joints / asset (mean / median / P90) | Multi-joint assets | Pooled raw-tree support (descriptive) | Exact duplicate rate |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        "| Ours / PV-A | "
        f"{_count(cohort['N_release'])} | {_count(cohort['N_eval'])} | "
        f"{_count(cohort['release_raw_categories'])} / {_count(cohort['eval_raw_categories'])} | "
        f"{_distribution(summary['links_per_asset'])} | "
        f"{_distribution(summary['movable_joints_per_asset'])} | "
        f"{_percent(summary['multi_joint_assets']['rate'])} (n={_count(summary['multi_joint_assets']['denominator'])}) | "
        f"{_count(topology['pooled_unique'])} / {_count(topology['pooled_denominator'])} "
        f"({_percent(topology['pooled_rate'])}; pooled diagnostic) | "
        f"{_percent(summary['exact_duplicate_rate']['rate'])} (n={_count(summary['exact_duplicate_rate']['denominator'])}) |",
        "",
        "### Table 1 topology interpretation",
        "",
        "| Cohort | Pooled raw-tree support | Category-conditioned support | Category-macro support |",
        "|---|---:|---:|---:|",
        "| Ours / PV-A full release | "
        f"{_count(topology['pooled_unique'])} / {_count(topology['pooled_denominator'])} "
        f"({_percent(topology['pooled_rate'])}) | "
        f"{_count(topology['conditioned_unique'])} / {_count(topology['conditioned_denominator'])} "
        f"({_percent(topology['conditioned_rate'])}) | "
        f"{_percent(topology['category_macro_rate'])} "
        f"({_count(topology['evaluable_categories'])} categories) |",
        "",
        "`rooted-joint-tree-v1` ignores names, geometry, and numerical parameters, includes fixed joints, and does not encode mimic dependencies. These values therefore describe raw rooted-URDF-tree signature support; they are not mechanism-level or geometry diversity scores.",
    ]
    if topology_audit is not None:
        rarefaction = topology_audit["category_stratified_rarefaction"]
        lines[-1] += (
            " Exact category-stratified rarefaction at "
            f"`k={int(rarefaction['k'])}` is "
            f"{_percent(rarefaction['rate'])}; reproduce it with "
            "`exp/scripts/audit_pva_table1_topologies.py`."
        )
    return lines


def _table2(summary: Mapping[str, Any]) -> list[str]:
    metrics = summary["metrics"]
    columns: Sequence[tuple[str, str]] = (
        ("parse_rate", "Parse Rate"),
        ("resource_resolution", "Resource Resolution"),
        ("finite_fields", "Finite Fields"),
        ("valid_tree", "Valid Tree"),
        ("valid_joint_spec", "Valid Joint Spec."),
        ("collision_coverage", "Collision Coverage"),
        ("inertial_coverage", "Inertial Coverage"),
        ("inertia_validity", "Inertia Validity"),
        ("strict_urdf_pass", "Strict URDF Pass"),
    )
    return [
        "## Table 2. URDF Validity and Structural Integrity",
        "",
        "| Dataset | " + " | ".join(label for _key, label in columns) + " |",
        "|---|" + "---:|" * len(columns),
        "| Ours / PV-A | " + " | ".join(_fraction(metrics[key]) for key, _label in columns) + " |",
    ]


def _table2_supplementary(summary: Mapping[str, Any]) -> list[str]:
    metrics = summary["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]["asset"]
    placeholder = metrics["placeholder_mass_incidence"]
    placeholder_text = str(placeholder.get("status", "N/E"))
    return [
        "## Table 2 Supplementary. Collision, Joint, and Inertial Diagnostics",
        "",
        "| Dataset | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |",
        "|---|---:|---:|---:|---:|",
        "| Ours / PV-A | "
        f"{_fraction(visual)} | {_fraction(metrics['joint_limit_portability'])} | "
        f"{_fraction(metrics['joint_dynamics_coverage'])} | {placeholder_text} |",
    ]


def _table3(summary: Mapping[str, Any]) -> list[str]:
    metrics = summary.get("metrics", summary)
    columns: Sequence[tuple[str, str]] = (
        ("valid_range", "Valid Range"),
        ("joint_sweep_success", "Joint Sweep Success"),
        ("non_degenerate_motion", "Non-degenerate Motion"),
        ("subtree_consistency", "Subtree Consistency"),
        ("fk_roundtrip_error", "FK Round-trip Error"),
        ("joint_level_pass", "Joint-level Pass"),
        ("strict_kinematic_pass", "Strict Kinematic Pass"),
    )
    cells = []
    for key, _label in columns:
        value = metrics.get(key, summary.get(key, {}))
        if key == "fk_roundtrip_error" and isinstance(value, Mapping):
            measured = value.get("measured_joint_count", 0)
            denominator = value.get("denominator", 0)
            translation = value.get("max_normalized_translation")
            rotation = value.get("max_rotation_rad")
            status = value.get("status", "N/E")
            if translation is None or rotation is None:
                cells.append(f"N/E ({_count(measured)} / {_count(denominator)} measured; {status})")
            else:
                cells.append(
                    f"{float(translation):.6f} normalized translation / "
                    f"{float(rotation):.6e} rad rotation "
                    f"({_count(measured)} / {_count(denominator)} measured; {status})"
                )
        else:
            cells.append(_fraction(value) if isinstance(value, Mapping) else str(value))
    return [
        "## Table 3. Kinematic Executability",
        "",
        "| Dataset | " + " | ".join(label for _key, label in columns) + " |",
        "|---|" + "---:|" * len(columns),
        "| Ours / PV-A | " + " | ".join(cells) + " |",
    ]


def _table4(summary: Mapping[str, Any]) -> list[str]:
    metrics = summary["metrics"]
    values = [
        table4_renderer._format_metric(metrics.get(key), key)
        for key, _label in table4_renderer.METRICS
    ]
    return [
        "## Table 4. Collision and Mechanical Clearance",
        "",
        "| Dataset | " + " | ".join(label for _key, label in table4_renderer.METRICS) + " |",
        "|---|" + "---:|" * len(table4_renderer.METRICS),
        "| Ours / PV-A | " + " | ".join(values) + " |",
    ]


def _render_verified(root: Path) -> str:
    receipt = _json(root / "full_release_receipt.json")
    tables = receipt["tables"]
    summaries = {
        name: _json(root / str(tables[name]["summary"]))
        for name in ("table1", "table2", "table2_supplementary", "table3", "table4")
    }
    topology_denominators = [
        int(category["unique_topologies"]["denominator"])
        for category in summaries["table1"]["category_breakdown"].values()
    ]
    topology_audit = None
    if topology_denominators and min(topology_denominators) > 0:
        topology_audit = topology_auditor.audit_files(
            root / str(tables["table1"]["records"]),
            root / str(tables["table1"]["summary"]),
            k=min(5, min(topology_denominators)),
        )
    classification = str(receipt.get("classification", "UNCLASSIFIED"))
    title_scope = "Full-Release" if classification == "FORMAL_FULL_RELEASE" else classification
    lines = [
        f"# Ours / PV-A {title_scope} Results",
        "",
        f"Classification: **{classification}**. Frozen evaluation: **{int(receipt['N_eval']):,} assets**, **{int(receipt['J_eval']):,} movable joints**, **{int(receipt['eval_category_count']):,} generator classes**.",
        "",
        "All manifest assets remain in the denominator. Parser errors, native crashes, and timeouts are retained as failures. Table 4 uses K=21 single-joint states and R=64 Sobol states with seed 20260813.",
        "",
    ]
    for section in (
        _table1(summaries["table1"], topology_audit),
        _table2(summaries["table2"]),
        _table2_supplementary(summaries["table2_supplementary"]),
        _table3(summaries["table3"]),
        _table4(summaries["table4"]),
    ):
        lines.extend(section)
        lines.extend(["", "---", ""])
    evidence = [
        "## Evidence",
        "",
        f"- Full receipt: `{root / 'full_release_receipt.json'}`",
        f"- Read-only Table 1 topology audit: `{topology_auditor.SCRIPT}`",
    ]
    if (root / "automation_check.json").is_file():
        evidence.append(f"- Automation check: `{root / 'automation_check.json'}`")
    evidence.extend(
        [
            f"- Source roster: `{receipt['roster_manifest']}`",
            f"- Result database: `{root / 'results.sqlite3'}`",
            "",
        ]
    )
    lines.extend(evidence)
    return "\n".join(lines)


def render(root: Path) -> str:
    root = Path(root).resolve(strict=True)
    pva_checker.check_results(root)
    return _render_verified(root)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check-output", type=Path)
    args = parser.parse_args(argv)
    try:
        root = args.root.resolve(strict=True)
        output = args.output or root / "pva_table1234_full_release_results.md"
        check_output = args.check_output or root / "automation_check.json"
        report = pva_checker.check_results(root)
        common._atomic_write_bytes(
            check_output,
            (json.dumps(report, sort_keys=True, separators=(",", ":")) + "\n").encode(
                "utf-8"
            ),
        )
        markdown = _render_verified(root)
        common._atomic_write_bytes(output, markdown.encode("utf-8"))
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"output": str(output), "bytes": len(markdown.encode('utf-8'))}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["render"]
