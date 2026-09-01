from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]


def load_script(name: str):
    path = EXP_ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


canonical = load_script("canonicalize_table4_artifact")
scorer = load_script("score_table4_constraints_v2")


SYNTHETIC_URDF = """<robot name="stool">
  <link name="stool_body">
    <visual name="seat"><origin xyz="0 0 0.575"/><geometry><box size="0.36 0.36 0.05"/></geometry></visual>
    <visual name="leg_0"><origin xyz="0.15 0.15 0.275"/><geometry><box size="0.03 0.03 0.55"/></geometry></visual>
    <visual name="leg_1"><origin xyz="0.15 -0.15 0.275"/><geometry><box size="0.03 0.03 0.55"/></geometry></visual>
    <visual name="leg_2"><origin xyz="-0.15 0.15 0.275"/><geometry><box size="0.03 0.03 0.55"/></geometry></visual>
    <visual name="leg_3"><origin xyz="-0.15 -0.15 0.275"/><geometry><box size="0.03 0.03 0.55"/></geometry></visual>
    <visual name="footrest_ring"><origin xyz="0 0 0.25"/><geometry><cylinder radius="0.15" length="0.02"/></geometry></visual>
  </link>
</robot>
"""


class Table4ConstraintsV2Test(unittest.TestCase):
    def test_alias_boundaries(self):
        self.assertTrue(scorer.alias_match("front_leg_03", "leg"))
        self.assertFalse(scorer.alias_match("legacy_panel", "leg"))

    def test_synthetic_urdf_scores_frozen_task(self):
        runtime = EXP_ROOT / "runtime"
        runtime.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=runtime, prefix="table4_v2_test_") as tmp:
            root = Path(tmp)
            urdf = root / "model.urdf"
            urdf.write_text(SYNTHETIC_URDF, encoding="utf-8")
            scene, nodes = canonical.canonicalize_urdf(urdf, 1.0)
            bounds = scene.bounds
            extents = bounds[1] - bounds[0]
            artifact = {
                "z_extent_m": float(extents[2]),
                "horizontal_max_m": float(max(extents[0], extents[1])),
                "horizontal_min_m": float(min(extents[0], extents[1])),
            }
            spec = json.loads(
                (EXP_ROOT / "reference/table4_constraints_v2/specs/T4C001.json").read_text(encoding="utf-8")
            )
            records = []
            for constraint in spec["constraints"]:
                if constraint["type"] == "exact_count":
                    records.append(scorer.measure_count(constraint, nodes))
                else:
                    records.append(scorer.measure_numeric(constraint, artifact))
            self.assertEqual(len(records), 4)
            self.assertTrue(all(row["measurable"] for row in records))
            self.assertTrue(all(row["passed"] for row in records))
            self.assertAlmostEqual(artifact["z_extent_m"], 0.60, places=6)
            self.assertAlmostEqual(artifact["horizontal_max_m"], 0.36, places=6)

    def test_missing_names_reduce_coverage(self):
        constraint = {
            "aliases": ["drawer", "drawer_front"],
            "target": 6,
        }
        result = scorer.measure_count(constraint, [{"level": "glb_node", "name": "Cube", "mesh_bearing": True}])
        self.assertFalse(result["measurable"])
        self.assertFalse(result["passed"])

    def test_zero_coverage_display_is_explicit(self):
        self.assertEqual(scorer.display_ratio(None), "N/A")
        self.assertEqual(scorer.display_ratio(0.0), "0.000")


if __name__ == "__main__":
    unittest.main()
