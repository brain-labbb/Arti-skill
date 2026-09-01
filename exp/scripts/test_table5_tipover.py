#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from xml.etree import ElementTree


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_tipover_aggregate as aggregate
import table5_tipover_runtime as runtime


class TipOverRuntimeTest(unittest.TestCase):
    def test_tilt_ignores_yaw_and_measures_roll(self) -> None:
        yaw = math.radians(90.0) / 2.0
        self.assertAlmostEqual(runtime.tilt_deg([math.cos(yaw), 0.0, 0.0, math.sin(yaw)]), 0.0)
        roll = math.radians(30.0) / 2.0
        self.assertAlmostEqual(runtime.tilt_deg([math.cos(roll), math.sin(roll), 0.0, 0.0]), 30.0)

    def test_perturbation_is_height_normalized(self) -> None:
        small = runtime.perturbation_delta_v(0.25)
        large = runtime.perturbation_delta_v(1.0)
        self.assertAlmostEqual(large / small, 2.0)
        small_angular = runtime.perturbation_delta_omega(0.25)
        large_angular = runtime.perturbation_delta_omega(1.0)
        self.assertAlmostEqual(small_angular / large_angular, 2.0)

    def test_lock_transform_preserves_fixed_and_rewrites_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root / "mesh.obj"
            mesh.write_text("o x\n", encoding="ascii")
            source = root / "source.urdf"
            source.write_text(
                '<robot name="x"><link name="a"><visual><geometry>'
                '<mesh filename="mesh.obj"/></geometry></visual></link>'
                '<link name="b"/><joint name="door" type="revolute">'
                '<parent link="a"/><child link="b"/><axis xyz="0 0 1"/>'
                '<limit lower="0" upper="1" effort="1" velocity="1"/>'
                '</joint></robot>',
                encoding="ascii",
            )
            output = root / "locked" / "model.urdf"
            receipt = runtime.make_locked_urdf(source, root, output)
            tree = ElementTree.parse(output)
            joint = tree.getroot().find("joint")
            rewritten = tree.getroot().find(".//mesh")
            self.assertEqual(joint.get("type"), "fixed")
            self.assertIsNone(joint.find("axis"))
            self.assertEqual(Path(rewritten.get("filename")), mesh)
            self.assertEqual(receipt["locked_joint_names"], ["door"])

    def test_robust_policy_falls_back_to_visual_and_repairs_degenerate_mesh(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root / "triangle.obj"
            mesh.write_text(
                "v 0 0 0\nv 1 0 0\nv 0 1 1\nf 1 2 3\n",
                encoding="ascii",
            )
            source = root / "source.urdf"
            source.write_text(
                '<robot name="x"><link name="a"><visual><geometry>'
                '<mesh filename="triangle.obj"/></geometry></visual></link></robot>',
                encoding="ascii",
            )
            output = root / "locked" / "model.urdf"
            receipt = runtime.make_locked_urdf(
                source,
                root,
                output,
                collision_policy="robust_visual_collision",
            )
            tree = ElementTree.parse(output)
            self.assertEqual(len(tree.getroot().findall(".//collision")), 1)
            self.assertIsNotNone(tree.getroot().find(".//collision/geometry/box"))
            self.assertEqual(receipt["visual_fallback_geometry_count"], 1)
            self.assertEqual(receipt["repaired_degenerate_meshes"], 1)

    def test_aggregate_counts_only_completed_protocol_receipts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = {
                "manifest_sha256": "x",
                "datasets": [
                    {
                        "dataset_name": "Ours (PV-A)",
                        "dataset_slug": "pva",
                        "rows": [
                            {"dataset_id": "pva_0000", "physics": {"status": "ready"}},
                            {"dataset_id": "pva_0001", "physics": {"status": "ready"}},
                        ],
                    }
                ],
            }
            prepared = root / "manifest.json"
            prepared.write_text(json.dumps(manifest), encoding="utf-8")
            assets = root / "run/pva/genesis/assets"
            assets.mkdir(parents=True)
            receipt = {
                "schema_version": runtime.RECEIPT_SCHEMA,
                "protocol_id": runtime.PROTOCOL_ID,
                "tip_over_stability_passed": True,
                "perturbed_tip_over_stability_passed": False,
                "valid_trial_count": 5,
                "planned_trial_count": 5,
            }
            record = {
                "terminal_status": "completed",
                "evaluation": {"tipover_stability": {"rollout": receipt}},
            }
            (assets / "pva_0000.json").write_text(json.dumps(record), encoding="utf-8")
            result = aggregate.aggregate_dataset(prepared, root / "run", "pva")
            self.assertEqual(result["metrics"]["tip_over_stability"]["count"], 1)
            self.assertEqual(result["metrics"]["perturbed_tip_over_stability"]["count"], 0)
            self.assertEqual(result["metrics"]["valid_trial_coverage"]["percentage"], 50.0)


if __name__ == "__main__":
    unittest.main()
