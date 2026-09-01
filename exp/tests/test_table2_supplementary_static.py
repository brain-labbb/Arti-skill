from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[2]
MODULE = REPO / "exp/scripts/table2_supplementary_static.py"


def load_module():
    spec = importlib.util.spec_from_file_location("table2_supplementary_static", MODULE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CUBE_OBJ = """\
v -0.5 -0.5 -0.5
v 0.5 -0.5 -0.5
v 0.5 0.5 -0.5
v -0.5 0.5 -0.5
v -0.5 -0.5 0.5
v 0.5 -0.5 0.5
v 0.5 0.5 0.5
v -0.5 0.5 0.5
f 1 2 3 4
f 5 6 7 8
f 1 2 6 5
f 2 3 7 6
f 3 4 8 7
f 4 1 5 8
"""

GOOD_URDF = """\
<?xml version="1.0"?>
<robot name="good">
  <link name="base">
    <visual><geometry><mesh filename="cube.obj"/></geometry></visual>
    <collision><geometry><mesh filename="cube.obj"/></geometry></collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="lid">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="lid"/>
    <limit lower="0" upper="1" effort="10" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
</robot>
"""

NO_COLLISION_URDF = """\
<?xml version="1.0"?>
<robot name="nocoll">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
  </link>
  <link name="lid">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="lid"/>
    <limit lower="0" upper="1" effort="10" velocity="2"/>
  </joint>
</robot>
"""

MISSING_MESH_URDF = """\
<?xml version="1.0"?>
<robot name="missing">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><mesh filename="missing.obj"/></geometry></collision>
  </link>
</robot>
"""

BAD_JOINT_URDF = """\
<?xml version="1.0"?>
<robot name="badjoint">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="lid">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="slider" type="prismatic">
    <parent link="base"/>
    <child link="lid"/>
    <limit lower="2" upper="1" velocity="2"/>
  </joint>
  <joint name="spin" type="continuous">
    <parent link="base"/>
    <child link="lid"/>
    <limit effort="1" velocity="1"/>
  </joint>
</robot>
"""


def build_package(root: Path, name: str, urdf_text: str, *, with_obj: bool = True) -> Path:
    package = root / name
    package.mkdir(parents=True)
    (package / f"{name}.urdf").write_text(urdf_text, encoding="utf-8")
    if with_obj:
        (package / "cube.obj").write_text(CUBE_OBJ, encoding="utf-8")
    return package


class StaticEvaluatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def _audit(self, package: Path, **kwargs):
        name = package.name
        return self.module.audit_package(
            package, urdf_relative_path=f"{name}.urdf", asset_id=name, **kwargs
        )

    def test_good_package_passes_all_static_metrics(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_good_") as temporary:
            package = build_package(Path(temporary), "good", GOOD_URDF)
            record = self._audit(package, expected_movable_joints=1)
        self.assertEqual(record["status"], "completed")
        table2 = record["table2_supplementary"]
        visual = table2["visual_bearing_collision_coverage"]
        self.assertTrue(visual["asset_pass"])
        self.assertEqual(visual["visual_bearing_links_declared"], 2)
        self.assertEqual(visual["covered_visual_bearing_links"], 2)
        portability = table2["joint_limit_portability"]
        self.assertEqual(portability["joints_passed"], 1)
        self.assertEqual(portability["joints_intended"], 1)
        self.assertTrue(portability["extraction_complete"])
        dynamics = table2["joint_dynamics_coverage"]
        self.assertEqual(dynamics["joints_covered"], 1)
        placeholder = table2["placeholder_mass_incidence"]
        self.assertEqual(placeholder["status"], "N/E")
        self.assertEqual(placeholder["complete_inertial_links"], 1)
        self.assertEqual(placeholder["dynamic_links"], 2)

    def test_visual_link_without_collision_fails_closed(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_nocoll_") as temporary:
            package = build_package(Path(temporary), "nocoll", NO_COLLISION_URDF, with_obj=False)
            record = self._audit(package, expected_movable_joints=1)
        visual = record["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertFalse(visual["asset_pass"])
        self.assertEqual(visual["asset_passed"], 0)
        self.assertEqual(visual["visual_bearing_links_declared"], 2)
        self.assertEqual(visual["covered_visual_bearing_links"], 0)
        # dynamics missing -> not covered
        dynamics = record["table2_supplementary"]["joint_dynamics_coverage"]
        self.assertEqual(dynamics["joints_covered"], 0)
        self.assertEqual(dynamics["joints_intended"], 1)

    def test_missing_collision_mesh_fails_closed(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_missing_") as temporary:
            package = build_package(Path(temporary), "missing", MISSING_MESH_URDF, with_obj=False)
            record = self._audit(package, expected_movable_joints=0)
        visual = record["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertFalse(visual["asset_pass"])
        self.assertFalse(record["resource_closure"]["complete"])

    def test_invalid_limits_and_continuous_mapping(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_badjoint_") as temporary:
            package = build_package(Path(temporary), "badjoint", BAD_JOINT_URDF, with_obj=False)
            record = self._audit(package, expected_movable_joints=2)
        portability = record["table2_supplementary"]["joint_limit_portability"]
        self.assertEqual(portability["joints_extracted"], 2)
        self.assertEqual(portability["joints_passed"], 1)  # only continuous passes
        by_name = {r["joint_name"]: r for r in portability["joint_records"]}
        self.assertFalse(by_name["slider"]["limit_portability_pass"])
        self.assertIn("lower_upper_invalid", by_name["slider"]["limit_portability_issues"])
        self.assertIn("effort_missing_or_invalid", by_name["slider"]["limit_portability_issues"])
        self.assertTrue(by_name["spin"]["limit_portability_pass"])

    def test_expected_joint_mismatch_keeps_intended_denominator(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_mismatch_") as temporary:
            package = build_package(Path(temporary), "good", GOOD_URDF)
            record = self._audit(package, expected_movable_joints=3)
        portability = record["table2_supplementary"]["joint_limit_portability"]
        self.assertEqual(portability["joints_intended"], 3)
        self.assertEqual(portability["joints_extracted"], 1)
        self.assertFalse(portability["extraction_complete"])
        self.assertEqual(portability["status"], "PARTIAL")

    def test_unparseable_xml_keeps_intended_denominator(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_badxml_") as temporary:
            root = Path(temporary)
            package = root / "badxml"
            package.mkdir()
            (package / "badxml.urdf").write_text("<robot", encoding="utf-8")
            record = self._audit(package, expected_movable_joints=4)
        self.assertEqual(record["status"], "error")
        table2 = record["table2_supplementary"]
        self.assertEqual(table2["joint_limit_portability"]["joints_intended"], 4)
        self.assertEqual(table2["joint_limit_portability"]["joints_passed"], 0)
        self.assertEqual(table2["visual_bearing_collision_coverage"]["asset_passed"], 0)

    def test_missing_package_preflight_failure(self):
        import tempfile

        with tempfile.TemporaryDirectory(prefix="t2s_missingpkg_") as temporary:
            record = self.module.audit_package(
                Path(temporary) / "does_not_exist",
                urdf_relative_path="x.urdf",
                asset_id="does_not_exist",
                expected_movable_joints=2,
            )
        self.assertEqual(record["status"], "error")
        self.assertIn("package_preflight_failed", record["issues"][0])
        self.assertEqual(record["table2_supplementary"]["joint_dynamics_coverage"]["joints_intended"], 2)

    def test_package_relative_path_resolution_rule(self):
        lam_static = self.module.lam_static
        resolve = lam_static.safe_package_relative_path
        self.assertEqual(resolve("./objs/x.obj", field="t").as_posix(), "objs/x.obj")
        self.assertEqual(resolve("objs/./x.obj", field="t").as_posix(), "objs/x.obj")
        self.assertEqual(resolve("objs/x.obj", field="t").as_posix(), "objs/x.obj")
        # v1.2: dot-segment normalization; leading .. preserved for the
        # containment check at resolution time.
        self.assertEqual(resolve("./../partseg/1/x.obj", field="t").as_posix(), "../partseg/1/x.obj")
        self.assertEqual(resolve("a/../b.obj", field="t").as_posix(), "b.obj")
        self.assertEqual(resolve("../x.obj", field="t").as_posix(), "../x.obj")
        for bad in ("/abs.obj", "a//b.obj", "a/", "a\\b.obj", "", "./", "C:/x.obj", "."):
            with self.assertRaises(ValueError):
                resolve(bad, field="t")

    def test_subdirectory_urdf_with_dotdot_reference_resolves_inside_package(self):
        import tempfile

        urdf = """<?xml version="1.0"?>
<robot name="subdir">
  <link name="base">
    <visual><geometry><mesh filename="./../meshes/cube.obj"/></geometry></visual>
    <collision><geometry><mesh filename="./../meshes/cube.obj"/></geometry></collision>
  </link>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="t2s_subdir_") as temporary:
            root = Path(temporary)
            package = root / "pkg"
            (package / "urdf").mkdir(parents=True)
            (package / "meshes").mkdir()
            (package / "meshes" / "cube.obj").write_text(CUBE_OBJ, encoding="utf-8")
            (package / "urdf" / "pkg.urdf").write_text(urdf, encoding="utf-8")
            record = self.module.audit_package(
                package, urdf_relative_path="urdf/pkg.urdf", asset_id="pkg",
                expected_movable_joints=0,
            )
        visual = record["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertTrue(visual["asset_pass"])
        self.assertEqual(visual["covered_visual_bearing_links"], 1)
        self.assertTrue(record["resource_closure"]["complete"])

    def test_dotdot_reference_escaping_package_fails_closed(self):
        import tempfile

        urdf = """<?xml version="1.0"?>
<robot name="escape">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><mesh filename="./../../outside.obj"/></geometry></collision>
  </link>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="t2s_escape_") as temporary:
            root = Path(temporary)
            (root / "outside.obj").write_text(CUBE_OBJ, encoding="utf-8")
            package = root / "pkg"
            (package / "urdf").mkdir(parents=True)
            (package / "urdf" / "pkg.urdf").write_text(urdf, encoding="utf-8")
            record = self.module.audit_package(
                package, urdf_relative_path="urdf/pkg.urdf", asset_id="pkg",
                expected_movable_joints=0,
            )
        visual = record["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertFalse(visual["asset_pass"])
        self.assertEqual(visual["covered_visual_bearing_links"], 0)

    def test_dot_slash_collision_mesh_resolves(self):
        import tempfile

        urdf = """<?xml version="1.0"?>
<robot name="dotslash">
  <link name="base">
    <visual><geometry><mesh filename="./cube.obj"/></geometry></visual>
    <collision><geometry><mesh filename="./cube.obj"/></geometry></collision>
  </link>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="t2s_dotslash_") as temporary:
            package = build_package(Path(temporary), "dotslash", urdf)
            record = self._audit(package, expected_movable_joints=0)
        visual = record["table2_supplementary"]["visual_bearing_collision_coverage"]
        self.assertTrue(visual["asset_pass"])
        self.assertEqual(visual["covered_visual_bearing_links"], 1)
        self.assertTrue(record["resource_closure"]["complete"])

    def test_placeholder_registry_matching(self):
        import tempfile

        registry = [{
            "registry_id": "unit_mass_zero_inertia",
            "mass": 1.0,
            "inertia": {field: 0.0 for field in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
            "absolute_tolerance": 1e-9,
            "relative_tolerance": 1e-9,
        }]
        urdf = """\
<?xml version="1.0"?>
<robot name="placeholder">
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <link name="real">
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="t2s_placeholder_") as temporary:
            package = build_package(Path(temporary), "placeholder", urdf, with_obj=False)
            record = self._audit(package, expected_movable_joints=0, placeholder_registry=registry)
        placeholder = record["table2_supplementary"]["placeholder_mass_incidence"]
        self.assertEqual(placeholder["status"], "COMPLETE")
        self.assertEqual(placeholder["placeholder_links"], 1)
        self.assertEqual(placeholder["incidence_numerator"], 1)
        self.assertEqual(placeholder["incidence_denominator"], 2)


if __name__ == "__main__":
    unittest.main()
