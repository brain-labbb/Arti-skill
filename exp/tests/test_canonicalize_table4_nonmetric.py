from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CANONICALIZER = REPO / "exp/scripts/canonicalize_mobility_table4_nonmetric_artifact.py"
PYTHON = Path("/mnt/zsn/miniconda3/bin/python")


def test_nonmetric_mode_emits_no_metric_claims() -> None:
    temporary_parent = REPO / "exp/tests/.tmp"
    temporary_parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="canonical-nonmetric-", dir=temporary_parent) as raw:
        root = Path(raw)
        source = root / "cube.urdf"
        source.write_text(
            '<robot name="fixture"><link name="cube"><visual><geometry>'
            '<box size="2 3 4"/></geometry></visual></link></robot>\n',
            encoding="utf-8",
        )
        output = root / "canonical"
        result = subprocess.run(
            [
                str(PYTHON), str(CANONICALIZER), "--input", str(source),
                "--artifact-type", "urdf",
                "--output-dir", str(output),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        assert result.returncode == 0, result.stdout
        artifact = json.loads((output / "artifact.json").read_text(encoding="utf-8"))
        assert artifact["metric_eligible"] is False
        assert artifact["numeric_constraint_status"] == "N/A"
        assert artifact["coordinate_units"] == "dataset_units_metric_binding_unestablished"
        assert artifact["geometry_scale_applied"] == 1.0
        assert artifact["extents_dataset_units"] == [2.0, 3.0, 4.0]
        assert "bounds_dataset_units" in artifact
        assert "unit_scale_to_m" not in artifact
        assert not any(key.endswith("_m") for key in artifact)
