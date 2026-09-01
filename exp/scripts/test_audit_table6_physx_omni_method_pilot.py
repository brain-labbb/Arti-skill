#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from audit_table6_physx_omni_method_pilot import (
    audit_urdf,
    output_cardinality,
    read_stage_timing,
    stage_output_inventory,
)


MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <link name="door"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


class AuditRulesTest(unittest.TestCase):
    def test_zero_collision_is_na_not_a_vacuous_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "basic.urdf"
            path.write_text(MINIMAL_URDF, encoding="utf-8")
            row = audit_urdf(path)
        self.assertEqual(row["collision_element_count"], 0)
        self.assertEqual(row["collision_evaluation_status"], "N/A_ZERO_COLLISION_GEOMETRY")
        self.assertEqual(row["movable_joint_count"], 1)
        self.assertTrue(row["valid_tree"])

    def test_missing_historical_times_remain_unknown(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            timing = read_stage_timing(Path(temporary), "stage1")
        self.assertEqual(timing["started_utc"], "unknown")
        self.assertEqual(timing["finished_utc"], "unknown")
        self.assertEqual(timing["wall_seconds"], "unknown")

    def test_stage_hashes_cover_only_stage_specific_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "allind.npy").write_bytes(b"stage1")
            (root / "objs/0").mkdir(parents=True)
            (root / "objs/0/0.glb").write_bytes(b"stage2")
            (root / "basic.urdf").write_text(MINIMAL_URDF, encoding="utf-8")
            self.assertEqual([row["path"] for row in stage_output_inventory(root, "stage1")], ["allind.npy"])
            self.assertEqual([row["path"] for row in stage_output_inventory(root, "stage2")], ["objs/0/0.glb"])
            self.assertEqual([row["path"] for row in stage_output_inventory(root, "stage3")], ["basic.urdf"])

    def test_output_cardinality_requires_each_declared_part_format(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index in range(2):
                (root / f"ind_{index}.npy").parent.mkdir(parents=True, exist_ok=True)
                (root / f"ind_{index}.npy").write_bytes(b"npy")
                (root / f"objs/{index}").mkdir(parents=True)
                (root / f"objs/{index}/{index}.glb").write_bytes(b"glb")
                (root / f"objs/{index}/{index}.obj").write_bytes(b"obj")
                (root / f"objs/{index}/material_0.png").write_bytes(b"png")
            row = output_cardinality(root)
            self.assertEqual(row["declared_part_count"], 2)
            self.assertTrue(row["stage2_cardinality_match"])
            (root / "objs/1/material_0.png").unlink()
            self.assertFalse(output_cardinality(root)["stage2_cardinality_match"])


if __name__ == "__main__":
    unittest.main()
