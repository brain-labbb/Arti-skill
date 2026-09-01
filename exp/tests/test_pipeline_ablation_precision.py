from __future__ import annotations

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "exp/scripts/simulate_pipeline_ablation_precision.py"
CONFIG = REPO_ROOT / "exp/reference/pipeline_ablation_precision_scenarios_v1.json"


def load_module():
    spec = importlib.util.spec_from_file_location("pipeline_precision", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compact_config() -> dict:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    config["simulations_per_cell"] = 100
    config["task_counts"] = [12, 24]
    config["task_icc"] = [0.25]
    return config


def test_simulation_is_deterministic_and_complete() -> None:
    module = load_module()
    first = module.simulate(compact_config())
    second = module.simulate(compact_config())
    assert first == second
    assert first["status"] == "PASS"
    assert first["cell_count"] == 6
    assert {cell["scenario"] for cell in first["cells"]} == {
        "null_calibration",
        "modest",
        "moderate",
    }
    for cell in first["cells"]:
        assert set(cell["endpoints"]) == {
            "source",
            "design",
            "interaction",
            "package",
        }
        for endpoint in cell["endpoints"].values():
            assert 0.0 <= endpoint["two_sided_rejection_probability"] <= 1.0
            assert endpoint["median_ci95_half_width"] >= 0.0


def test_invalid_task_icc_fails_closed() -> None:
    module = load_module()
    config = compact_config()
    config["task_icc"] = [1.0]
    try:
        module.simulate(config)
    except ValueError as exc:
        assert "ICC" in str(exc)
    else:
        raise AssertionError("invalid ICC was accepted")
