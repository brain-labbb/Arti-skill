from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/run_table5_n200.py"
SPEC = importlib.util.spec_from_file_location("run_table5_n200", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_csv_selection_is_fail_closed() -> None:
    assert MODULE._csv("pybullet,mujoco", allowed=set(MODULE.SIMULATORS)) == [
        "pybullet",
        "mujoco",
    ]
    with pytest.raises(MODULE.OrchestrationError):
        MODULE._csv("pybullet,pybullet", allowed=set(MODULE.SIMULATORS))
    with pytest.raises(MODULE.OrchestrationError):
        MODULE._csv("unknown", allowed=set(MODULE.SIMULATORS))


def test_existing_manifest_must_match_six_group_denominator(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps({"sample_size": 200, "dataset_count": 6, "total_rows": 1200}),
        encoding="utf-8",
    )
    MODULE._validate_existing_manifest(manifest, 200)
    with pytest.raises(MODULE.OrchestrationError):
        MODULE._validate_existing_manifest(manifest, 199)


def test_gpu_gate_rejects_busy_device(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        MODULE,
        "_gpu_inventory",
        lambda: [
            {
                "index": "0",
                "uuid": "GPU-test",
                "name": "test",
                "memory_total_mib": 100_000,
                "memory_free_mib": 80_000,
                "utilization_percent": 95,
            }
        ],
    )
    with pytest.raises(MODULE.OrchestrationError, match="busy devices"):
        MODULE._check_gpus(
            ["0"],
            maximum_utilization=20,
            minimum_free_mib=32_768,
            allow_busy=False,
        )
    selected = MODULE._check_gpus(
        ["GPU-test"],
        maximum_utilization=20,
        minimum_free_mib=32_768,
        allow_busy=True,
    )
    assert selected[0]["index"] == "0"


def test_all_stage_dry_run_does_not_write(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert (
        MODULE.main(
            [
                "--stage",
                "all",
                "--run-root",
                str(tmp_path / "run"),
                "--sample-size",
                "1",
                "--simulators",
                "pybullet,mujoco",
                "--dry-run",
            ]
        )
        == 0
    )
    output = capsys.readouterr().out
    assert "table5_articraft_github_parent.py" in output
    assert " --source " in output
    assert " --source-manifest " not in output
    assert not (tmp_path / "run").exists()
