from __future__ import annotations

import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "update_table4_full_release_markdown.py"
spec = importlib.util.spec_from_file_location("update_table4_markdown", SCRIPT)
assert spec and spec.loader
updater = importlib.util.module_from_spec(spec)
spec.loader.exec_module(updater)


def _receipt() -> dict[str, object]:
    metrics = {
        key: {"passed": 1, "denominator": 2}
        for key, _label in updater.METRICS
        if key not in {"aor", "max_penetration", "collision_state_rate", "collision_free_range"}
    }
    metrics.update(
        {
            "collision_state_rate": {"collision_states": 1, "denominator": 10},
            "aor": {"status": "N/E"},
            "max_penetration": {"maximum_observed_normalized": 0.25, "measured_assets": 1, "denominator": 2, "status": "PARTIAL"},
            "collision_free_range": {"passed_states": 1, "denominator": 2},
        }
    )
    return {
        "methods": [
            {"slug": slug, "display": display, "N_eval": 2, "J_eval": 3, "status": "COMPLETE", "metrics": metrics}
            for slug, display in updater.DATASETS
        ]
    }


def test_update_preserves_ours_and_replaces_only_comparison_rows(tmp_path: Path) -> None:
    source = tmp_path / "protocol.md"
    source.write_text(
        "prefix\n"
        "## Table 4. Collision and Mechanical Clearance\n\n"
        "| Dataset / Outputs | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Collision-state Rate | AOR | Max Penetration | Collision-free Range | Strict Collision Pass |\n"
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n"
        "| Ours-500K | source-a | source-b | source-c | source-d | source-e | source-f | source-g | source-h | source-i |\n"
        "| Articraft-10K | old | old | old | old | old | old | old | old | old |\n\n"
        "### Table 4 evaluation states\n\nrest\n",
        encoding="utf-8",
    )
    receipt = tmp_path / "receipt.json"
    receipt.write_text(json.dumps(_receipt()), encoding="utf-8")
    output = tmp_path / "updated.md"
    updater.update(source, receipt, output, root=tmp_path)
    text = output.read_text(encoding="utf-8")
    assert "| Ours-500K | source-a | source-b | source-c | source-d | source-e | source-f | source-g | source-h | source-i |" in text
    assert "| Articraft-10K | old |" not in text
    for _slug, display in updater.DATASETS:
        assert f"| {display} |" in text
    assert "historical evidence" in text
    assert "\n\n### Table 4 evaluation states" in text
