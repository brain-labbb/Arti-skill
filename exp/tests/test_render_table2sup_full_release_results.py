from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "render_table2sup_full_release_results.py"
spec = importlib.util.spec_from_file_location("render_table2sup", SCRIPT)
assert spec and spec.loader
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def _receipt() -> dict:
    rows = []
    for slug, display in renderer.BASELINE_DATASETS:
        receipt_display = "LAM" if slug == "lam" else display
        rows.append({
            "slug": slug,
            "dataset": receipt_display,
            "display": receipt_display,
            "N_eval": 12,
            "J_eval": 34,
            "status": "complete",
            "evidence": {"summary": f"{slug}/summary.json"},
            "metrics": {
                "visual_bearing_collision_coverage": {"passed": 10, "denominator": 12},
                "joint_limit_portability": {"passed": 20, "denominator": 34},
                "joint_dynamics_coverage": {"passed": 3, "denominator": 34},
                "placeholder_mass_incidence": {"value": "N/E"},
            },
        })
    return {"methods": rows}


def test_render_includes_source_ours_rows_and_all_eight_baselines(tmp_path: Path) -> None:
    source = tmp_path / "source.md"
    source.write_text(
        "### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics\n"
        "| Dataset / Outputs | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |\n"
        "|---|---:|---:|---:|---:|\n"
        "| Ours-500K | ours-collision | ours-joint | ours-dynamics | ours-placeholder |\n"
        "| Ours per-class N=5 (supplementary) | ours2-collision | ours2-joint | ours2-dynamics | ours2-placeholder |\n"
        "\n#### Table 2 supplementary metric definitions\n",
        encoding="utf-8",
    )
    receipt = tmp_path / "receipt.json"
    import json
    receipt.write_text(json.dumps(_receipt()), encoding="utf-8")
    rendered = renderer.render(tmp_path, source, receipt_path=receipt)
    assert rendered.count("| Ours") == 2
    assert rendered.count("| Art") >= 2
    for _, display in renderer.BASELINE_DATASETS:
        assert f"| {display} |" in rendered
    assert "10 / 12 (83.33%)" in rendered
    assert "20 / 34 (58.82%)" in rendered
    assert "N/E" in rendered
    assert "summary.json" in rendered


def test_metric_adapter_accepts_nested_table2_supplementary_and_rejects_missing_dataset(tmp_path: Path) -> None:
    entry = {
        "display_name": "Infinite Mobility",
        "n_eval": 2,
        "j_eval": 4,
        "checkpoint_state": "complete",
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {"passed": 1, "denominator": 2},
            "joint_limit_portability": {"passed": 2, "denominator": 4},
            "joint_dynamics_coverage": {"passed": 0, "denominator": 4},
            "placeholder_mass_incidence": {"numerator": 0, "denominator": 1},
        },
    }
    row = renderer.normalize_entry(entry)
    assert row["display"] == "Infinite Mobility"
    assert row["metrics"]["joint_limit_portability"]["passed"] == 2
    try:
        renderer.normalize_entry({"n_eval": 1})
    except ValueError as exc:
        assert "dataset" in str(exc)
    else:
        raise AssertionError("missing dataset identity must fail closed")
