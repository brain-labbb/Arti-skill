from __future__ import annotations

import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "render_table4_full_release_results.py"
spec = importlib.util.spec_from_file_location("render_table4_full_release", SCRIPT)
assert spec and spec.loader
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def _metric_rows(*, blocked: bool = False) -> dict[str, object]:
    if blocked:
        ne = {"status": "N/E", "reason": "BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT"}
        return {key: ne for key, _label in renderer.METRICS}
    return {
        "rest_all_pair_cf": {"passed": 1, "denominator": 2},
        "rest_non_adjacent_cf": {"passed": 2, "denominator": 2},
        "single_joint_sweep_cf": {"passed": 3, "denominator": 4},
        # Older receipts called this atom multi_joint_sweep_cf.  The renderer
        # must accept it while publishing the frozen Sobol label.
        "multi_joint_sweep_cf": {"passed": 4, "denominator": 4},
        "collision_state_rate": {
            "collision_states": 1,
            "denominator": 10,
            "executed_states": 10,
        },
        "aor": {"status": "N/E", "reason": "exact volume backend not registered"},
        "max_penetration": {
            "maximum_observed_normalized": 0.125,
            "fully_measured_assets": 1,
            "denominator": 2,
            "status": "PARTIAL",
        },
        "collision_free_range": {"passed_states": 7, "denominator": 8},
        "strict_collision_pass": {"passed": 1, "denominator": 2},
    }


def _receipt() -> dict[str, object]:
    rows = []
    for index, (slug, display) in enumerate(renderer.DATASETS):
        rows.append(
            {
                "slug": slug,
                "display": display,
                "N_eval": index + 2,
                "J_eval": index + 4,
                "status": "complete" if slug != "infinite" else "blocked",
                "evidence": {"summary": f"{slug}/summary.json"},
                "metrics": _metric_rows(blocked=slug == "infinite"),
            }
        )
    return {"schema_version": "table4_full_release_receipt_v1", "methods": rows}


def _source(tmp_path: Path) -> Path:
    path = tmp_path / "protocol.md"
    cells = " | ".join(f"ours-{i}" for i in range(1, 10))
    brain = " | ".join(f"brain-{i}" for i in range(1, 10))
    path.write_text(
        "## Table 4. Collision and Mechanical Clearance\n\n"
        "| Dataset / Outputs | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Collision-state Rate | AOR | Max Penetration | Collision-free Range | Strict Collision Pass |\n"
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n"
        f"| Ours-500K | {cells} |\n"
        f"| Brain-500 | {brain} |\n\n"
        "### Table 4 evaluation states\n\nFrozen protocol.\n",
        encoding="utf-8",
    )
    return path


def test_render_publishes_all_eight_rows_and_preserves_source_rows(tmp_path: Path) -> None:
    source = _source(tmp_path)
    receipt = tmp_path / "full_release_receipt.json"
    receipt.write_text(json.dumps(_receipt()), encoding="utf-8")

    rendered = renderer.render(tmp_path, source, receipt_path=receipt)

    assert "# Table 4 Full-Release Results" in rendered
    assert "| Ours-500K | 500 | 2,467 | ours-1 |" in rendered
    # A source Brain row is retained verbatim as well.
    assert "| Brain-500 | source | source | brain-1 |" in rendered
    for _slug, display in renderer.DATASETS:
        assert f"| {display} |" in rendered
    assert "1 / 2 (50.000%)" in rendered
    assert "1 / 10 (10.000%)" in rendered
    assert "0.125000 (1 / 2 measured; PARTIAL)" in rendered
    assert "7 / 8 (87.500%)" in rendered
    assert "N/E" in rendered
    assert "infinite/summary.json" in rendered


def test_renderer_accepts_nested_summary_path_and_sobol_alias(tmp_path: Path) -> None:
    summary = tmp_path / "nested-summary.json"
    summary.write_text(
        json.dumps({"n_eval": 3, "j_eval": 5, "status": "complete", "metrics": _metric_rows()}),
        encoding="utf-8",
    )
    row = renderer.normalize_entry(
        {
            "dataset": "LAM",
            "summary": "nested-summary.json",
            "evidence": {"summary": "nested-summary.json"},
        },
        root=tmp_path,
    )
    assert row["slug"] == "lam"
    assert row["n_eval"] == 3
    assert renderer._format_metric(renderer._metric(row["metrics"], "multi_joint_sobol_cf"), "multi_joint_sobol_cf") == "4 / 4 (100.000%)"


def test_renderer_fails_closed_on_missing_dataset_or_metrics(tmp_path: Path) -> None:
    source = _source(tmp_path)
    receipt = tmp_path / "receipt.json"
    receipt.write_text(
        json.dumps({"methods": [{"slug": "articraft", "N_eval": 1, "J_eval": 1, "metrics": {}}]}),
        encoding="utf-8",
    )
    try:
        renderer.render(tmp_path, source, receipt_path=receipt)
    except ValueError as exc:
        assert "metrics" in str(exc) or "missing datasets" in str(exc)
    else:
        raise AssertionError("incomplete receipt must be rejected")

