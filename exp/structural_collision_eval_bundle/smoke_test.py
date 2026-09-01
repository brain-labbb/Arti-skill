#!/usr/bin/env python3
"""Run a self-contained end-to-end smoke test with one primitive URDF."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile


URDF = """<?xml version="1.0"?>
<robot name="integrity_smoke">
  <link name="base">
    <visual><geometry><box size="1 1 0.2"/></geometry></visual>
    <collision><geometry><box size="1 1 0.2"/></geometry></collision>
  </link>
  <link name="door">
    <visual><origin xyz="0.5 0 0.4"/><geometry><box size="0.1 0.8 0.8"/></geometry></visual>
    <collision><origin xyz="0.5 0 0.4"/><geometry><box size="0.1 0.8 0.8"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57079632679" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def run(command: list[str], cwd: Path) -> None:
    subprocess.run(command, cwd=cwd, check=True)


def main() -> int:
    root = Path(__file__).resolve().parent
    python = sys.executable
    with tempfile.TemporaryDirectory(prefix="integrity_smoke_") as temporary:
        workspace = Path(temporary)
        asset = workspace / "asset"
        asset.mkdir()
        urdf = asset / "model.urdf"
        urdf.write_text(URDF, encoding="utf-8")
        manifest = workspace / "manifest.jsonl"
        manifest.write_text(
            json.dumps(
                {
                    "dataset_slug": "smoke",
                    "dataset_name": "Smoke",
                    "asset_id": "primitive_hinge",
                    "urdf_path": str(urdf),
                    "package_root": str(asset),
                },
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        protocol = json.loads((root / "protocol.json").read_text(encoding="utf-8"))
        protocol["structural"]["pose_samples"] = 3
        protocol["structural"]["surface_samples"] = 32
        protocol["collision"]["pose_samples"] = 3
        smoke_protocol = workspace / "protocol.json"
        smoke_protocol.write_text(json.dumps(protocol), encoding="utf-8")
        results = workspace / "results"
        aggregate = workspace / "aggregate"
        run(
            [
                python,
                str(root / "full_eval.py"),
                "--manifest",
                str(manifest),
                "--protocol",
                str(smoke_protocol),
                "--out",
                str(results),
                "--workers",
                "1",
                "--task",
                "both",
            ],
            root,
        )
        run(
            [
                python,
                str(root / "aggregate.py"),
                "--manifest",
                str(manifest),
                "--results",
                str(results),
                "--out",
                str(aggregate),
            ],
            root,
        )
        summary = json.loads((aggregate / "summary.json").read_text(encoding="utf-8"))
        metrics = summary["datasets"]["smoke"]
        required = {
            "rooted_assets_percentage",
            "joint_support_macro_percentage",
            "joint_gap_p95_percent_diag",
            "axis_rooted_assets_percentage",
            "axis_support_macro_percentage",
            "k9_axis_pose_support_macro_percentage",
            "collision_free_joint_motion_range_macro_percentage",
            "premature_collision_free_joints_macro_percentage",
            "penetration_growth_asset_balanced_p95_percent_diag",
        }
        missing = sorted(required - set(metrics))
        if missing:
            raise AssertionError(f"aggregate is missing metrics: {missing}")
        if metrics["recorded_assets"] != 1:
            raise AssertionError(metrics)
        print("smoke test passed: one asset, nine metric fields, atomic record and aggregate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
