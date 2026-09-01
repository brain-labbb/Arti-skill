from __future__ import annotations

import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_table123_full_release_checks as checks


def _minimal_primary_markdown() -> str:
    table1 = [
        "## Table 1. Dataset Scale and Structural Diversity",
        "| Dataset / Outputs | N_release | N_eval | Observed Labels (release / eval) | Links/Asset | Movable Joints/Asset | Multi-joint Assets (%) | Unique Topologies (%) | Exact Duplicate Rate (%) |",
        "|---|---:|---:|---|---:|---:|---:|---:|---:|",
        "| Ours-500K | 500 | 500 | x | x | x | x | x | x |",
        "| Ours per-class N=5 (supplementary) | 2,655 | 2,655 | x | x | x | x | x | x |",
    ]
    table2 = [
        "## Table 2. URDF Validity and Structural Integrity",
        "| Dataset / Outputs | Parse Rate | Resource Resolution | Finite Fields | Valid Tree | Valid Joint Spec. | Collision Coverage | Inertial Coverage | Inertia Validity | Strict URDF Pass |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        "| Ours-500K | x | x | x | x | x | x | x | x | x |",
        "| Ours per-class N=5 (supplementary) | x | x | x | x | x | x | x | x | x |",
    ]
    table3 = [
        "## Table 3. Kinematic Executability",
        "| Dataset / Outputs | Valid Range | Joint Sweep Success | Non-degenerate Motion | Subtree Consistency | FK Round-trip Error | Joint-level Pass | Strict Kinematic Pass |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        "| Ours-500K | x | x | x | x | x | x | x |",
        "| Ours per-class N=5 (supplementary) | x | x | x | x | x | x | x |",
    ]
    for item in checks.DATASETS:
        n = f"{item['n_eval']:,}"
        j = f"{item['j_eval']:,}"
        table1.append(
            f"| {item['display']} | {n} | {n} | x | 1 / 1 / 1 (n={n}) | "
            f"1 / 1 / 1 (n={n}) | 100.00% (n={n}) | 100.00% (n={n}) | 100.00% (n={n}) |"
        )
        pairs = " | ".join(f"{n} / {n} (100.00%)" for _ in checks.TABLE2_METRICS)
        table2.append(f"| {item['display']} | {pairs} |")
        t3_pairs = " | ".join(f"{j} / {j} (100.00%)" for _ in checks.TABLE3_PAIR_METRICS[:4])
        fk = f"0.000000 / 0.000000 rad ({j} / {j} measured; COMPLETE)"
        table3.append(f"| {item['display']} | {t3_pairs} | {fk} | {j} / {j} (100.00%) | {n} / {n} (100.00%) |")
    return "\n".join(
        table1
        + ["### Table 1 metric definitions"]
        + table2
        + ["### Table 2 metric definitions"]
        + table3
        + ["### Table 3 evaluation states", "Brain-500"]
    )


def _all_pass_results() -> dict[str, dict]:
    result = {}
    for item in checks.DATASETS:
        n = item["n_eval"]
        j = item["j_eval"]
        result[item["slug"]] = {
            "dataset": item["display"],
            "n_eval": n,
            "j_eval": j,
            "tables": {
                "table1": {
                    "metrics": {
                        "links_per_asset": {"mean": 1, "median": 1, "p90_nearest_rank": 1, "denominator": n},
                        "movable_joints_per_asset": {"mean": 1, "median": 1, "p90_nearest_rank": 1, "denominator": n},
                        "multi_joint_assets": {"rate": 1, "denominator": n},
                        "unique_topologies": {"rate": 1, "denominator": n},
                        "exact_duplicate_rate": {"rate": 1, "denominator": n},
                    }
                },
                "table2": {
                    "metrics": {
                        name: {"passed": n, "denominator": n}
                        for name in checks.TABLE2_METRICS
                    }
                },
                "table3": {
                    "metrics": {
                        name: {"passed": j, "denominator": j}
                        for name in checks.TABLE3_PAIR_METRICS
                    }
                    | {
                        "fk_roundtrip_error": {
                            "measured_joint_count": j,
                            "denominator": j,
                        },
                        "strict_kinematic_pass": {"passed": n, "denominator": n},
                    }
                },
            },
        }
    return result


def test_expected_release_contract_is_eight_datasets() -> None:
    assert len(checks.DATASETS) == 8
    assert sum(item["n_eval"] for item in checks.DATASETS) == 35_030
    assert sum(item["j_eval"] for item in checks.DATASETS) == 133_418


def test_validate_primary_markdown_checks_all_comparison_rows(tmp_path: Path) -> None:
    markdown = tmp_path / "evaluation.md"
    markdown.write_text(_minimal_primary_markdown(), encoding="utf-8")
    result = checks.validate_primary_markdown(
        markdown, _all_pass_results(), enforce_ours_baseline=False
    )
    assert result["table1_rows"] == 8
    assert result["table2_metrics"] == 8 * len(checks.TABLE2_METRICS)
    assert result["table3_metrics"] == 8 * (
        len(checks.TABLE3_PAIR_METRICS) + 2
    )


def test_validate_primary_markdown_rejects_n800_comparison_row(tmp_path: Path) -> None:
    markdown = tmp_path / "evaluation.md"
    content = _minimal_primary_markdown().replace(
        "| Articraft-10K | 9,996 | 9,996 |",
        "| Articraft-10K | 800 | 800 |",
    )
    markdown.write_text(content, encoding="utf-8")
    with pytest.raises(checks.AutomationError, match="N_release/N_eval"):
        checks.validate_primary_markdown(
            markdown, _all_pass_results(), enforce_ours_baseline=False
        )
