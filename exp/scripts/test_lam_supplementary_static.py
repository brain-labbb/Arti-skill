#!/usr/bin/env python3
"""Small unit tests for lam_supplementary_static; no formal cohort is touched."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lam_supplementary_static import audit_lam_package, sha256_file  # noqa: E402


GOOD_URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="door">
    <visual><geometry><box size="0.1 1 1"/></geometry></visual>
    <collision><geometry><box size="0.1 1 1"/></geometry></collision>
  </link>
  <joint name="door_joint" type="revolute">
    <parent link="base"/><child link="door"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="10" velocity="2"/>
    <dynamics damping="0.1" friction="0"/>
  </joint>
</robot>
"""


class LamSupplementaryStaticTest(unittest.TestCase):
    def make_package(self, urdf: str = GOOD_URDF) -> tuple[tempfile.TemporaryDirectory[str], Path]:
        temporary = tempfile.TemporaryDirectory()
        package = Path(temporary.name) / "asset"
        package.mkdir()
        (package / "generated.urdf").write_text(urdf, encoding="utf-8")
        return temporary, package

    def test_table2_atoms_placeholder_and_generation_config_privacy(self) -> None:
        temporary, package = self.make_package()
        self.addCleanup(temporary.cleanup)
        secret = "DO_NOT_LEAK_GENERATION_PROMPT_7f89"
        config = package / "generation_config.yaml"
        config.write_text(secret, encoding="utf-8")
        registry = [{
            "registry_id": "unit-mass-template",
            "mass": 1.0,
            "inertia": {
                "ixx": 0.1, "ixy": 0.0, "ixz": 0.0,
                "iyy": 0.1, "iyz": 0.0, "izz": 0.1,
            },
        }]
        record = audit_lam_package(
            package,
            expected_movable_joints=1,
            placeholder_registry=registry,
        )
        table2 = record["table2_supplementary"]
        self.assertEqual(record["status"], "completed")
        self.assertTrue(table2["visual_bearing_collision_coverage"]["asset_pass"])
        self.assertEqual(
            table2["visual_bearing_collision_coverage"]["covered_visual_bearing_links"], 2
        )
        self.assertEqual(table2["joint_limit_portability"]["joints_passed"], 1)
        self.assertEqual(table2["joint_dynamics_coverage"]["joints_covered"], 1)
        placeholder = table2["placeholder_mass_incidence"]
        self.assertEqual(placeholder["status"], "COMPLETE")
        self.assertEqual(placeholder["placeholder_links"], 1)
        config_metadata = record["s1_evidence"]["rebuild"]["generation_config_metadata_only"]
        self.assertEqual(config_metadata[0]["sha256"], sha256_file(config))
        serialized = json.dumps(record, sort_keys=True)
        self.assertNotIn(secret, serialized)

    def test_empty_placeholder_registry_is_unclassified_and_not_evaluable(self) -> None:
        temporary, package = self.make_package()
        self.addCleanup(temporary.cleanup)
        result = audit_lam_package(package, expected_movable_joints=1)
        metric = result["table2_supplementary"]["placeholder_mass_incidence"]
        self.assertEqual(metric["status"], "N/E")
        self.assertEqual(metric["complete_inertial_links"], 1)
        self.assertEqual(metric["classified_complete_inertial_links"], 0)
        self.assertEqual(metric["unclassified_complete_inertial_links"], 1)
        self.assertIsNone(metric["incidence_numerator"])

    def test_unsafe_collision_resource_fails_closed(self) -> None:
        urdf = GOOD_URDF.replace(
            '<collision><geometry><box size="0.1 1 1"/></geometry></collision>',
            '<collision><geometry><mesh filename="../escape.obj"/></geometry></collision>',
        )
        temporary, package = self.make_package(urdf)
        self.addCleanup(temporary.cleanup)
        result = audit_lam_package(package, expected_movable_joints=1)
        metric = result["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertFalse(metric["asset_pass"])
        self.assertEqual(metric["covered_visual_bearing_links"], 1)
        # v1.2 resolver: "../" references are normalized, then rejected at
        # resolution time as escaping or missing; either way fail-closed.
        self.assertTrue(any("mesh_resource" in issue for issue in metric["issues"]))
        self.assertFalse(result["resource_closure"]["complete"])

    def test_malformed_xml_retains_expected_joint_denominator(self) -> None:
        temporary, package = self.make_package("<robot><broken></robot>")
        self.addCleanup(temporary.cleanup)
        result = audit_lam_package(package, expected_movable_joints=3)
        self.assertEqual(result["status"], "error")
        self.assertFalse(result["parse"]["success"])
        joint = result["table2_supplementary"]["joint_limit_portability"]
        self.assertEqual(joint["joints_intended"], 3)
        self.assertEqual(joint["joints_passed"], 0)

    def test_receipt_rebuild_and_allowance_scans_use_strict_bindings(self) -> None:
        temporary, package = self.make_package()
        self.addCleanup(temporary.cleanup)
        config = package / "generation_config.yaml"
        config.write_text("private: true\n", encoding="utf-8")
        first = audit_lam_package(package, expected_movable_joints=1)
        receipt = {
            "asset_sha256": first["urdf_sha256"],
            "resource_closure_sha256": first["resource_closure"]["sha256"],
            "protocol_id": "fixture-protocol",
            "runner_sha256": "a" * 64,
            "pair_policy": "exclude-direct-parent-child",
            "thresholds": {"penetration_m": 1e-6},
            "conclusion": "pass",
        }
        (package / "mechanical_receipt.json").write_text(json.dumps(receipt), encoding="utf-8")
        recipe = {
            "runner": {"sha256": "b" * 64},
            "inputs": [
                {"path": "generated.urdf", "sha256": sha256_file(package / "generated.urdf")},
                {"path": "generation_config.yaml", "sha256": sha256_file(config)},
            ],
            "output": {"canonical_fingerprint_sha256": "c" * 64},
        }
        (package / "rebuild_recipe.json").write_text(json.dumps(recipe), encoding="utf-8")
        allowance = {"excluded_non_adjacent_pairs": []}
        (package / "allowances.json").write_text(json.dumps(allowance), encoding="utf-8")
        result = audit_lam_package(package, expected_movable_joints=1)
        s1 = result["s1_evidence"]
        self.assertEqual(s1["receipt"]["receipt_bound_asset"], 1)
        self.assertEqual(s1["receipt_replay"]["eligible_receipt_count"], 1)
        self.assertEqual(s1["rebuild"]["eligible_asset"], 1)
        self.assertEqual(s1["allowance"]["status"], "COMPLETE")
        self.assertEqual(s1["allowance"]["registered_excluded_pair_count"], 0)
        self.assertEqual(s1["allowance"]["eligible_nonadjacent_pair_count"], 0)


if __name__ == "__main__":
    unittest.main()
