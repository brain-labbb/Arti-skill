from __future__ import annotations

import json
from pathlib import Path
import subprocess

from exp.scripts import run_urdf_table4a_ours_500k as runner


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4a_ours_500k.py"
GENESIS_PYTHON = Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python")


def test_hash_cross_check_closes_over_executed_not_intended_states() -> None:
    aggregates = {
        "state_counts": {
            "intended": 63,
            "executed": 42,
            "hash_cross_check": {
                "verified": 42,
                "mismatch": 0,
                "no_reference": 0,
            },
        }
    }

    assert runner.hash_cross_check_covers_executed_states(aggregates)


def test_smoke_run_preserves_frozen_ours_contract(tmp_path: Path) -> None:
    output_dir = tmp_path / "table4a-ours-smoke"
    completed = subprocess.run(
        [
            str(GENESIS_PYTHON),
            str(RUNNER),
            "--mode",
            "smoke",
            "--n",
            "1",
            "--workers",
            "1",
            "--output-dir",
            str(output_dir),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
        timeout=900,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["dataset"] == "Ours-500K"
    assert summary["engine_protocol_id"] == "genesis_contact_penetration_v1"
    assert summary["cohort"] == {
        "source_manifest": str(
            REPO
            / "exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/frozen_manifest.json"
        ),
        "source_manifest_file_sha256": "1b29d868112dcda326a08f8e3439d6b96c65833b99cc33af3bfcdb58fb4c2e24",
        "ordered_ids_sha256": "dcd19530ff3a3546fa149db58f331a042d5c3326f1b4fa1e5580914952e79289",
        "n_eval": 1,
        "j_eval": 2,
    }
    assert summary["status_counts"] == {"completed": 1, "error": 0, "total": 1}
    metrics = summary["metrics"]
    assert metrics["joint_level_full_range_cf"]["denominator"] == 2
    assert metrics["collision_safe_dof_retention"]["denominator"] == 2467
    assert metrics["state_counts"]["intended"] == 42
    assert metrics["state_counts"]["executed"] == 42
    assert metrics["state_counts"]["hash_cross_check"] == {
        "verified": 42,
        "mismatch": 0,
        "no_reference": 0,
    }
    assert metrics["normalized_clearance_p5"]["status"] == "N/E"
