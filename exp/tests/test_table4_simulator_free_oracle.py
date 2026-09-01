from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


EXP = Path(__file__).parents[1]
SCRIPT = EXP / "scripts" / "table4_simulator_free_oracle.py"
SPEC = importlib.util.spec_from_file_location(
    "table4_simulator_free_oracle_test", SCRIPT
)
assert SPEC and SPEC.loader
oracle = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = oracle
SPEC.loader.exec_module(oracle)


def _write_urdf(tmp_path: Path, body: str, *, name: str = "model.urdf") -> Path:
    urdf = tmp_path / name
    urdf.write_text(
        f'<robot name="fixture">\n{body}\n</robot>\n',
        encoding="ascii",
    )
    return urdf


def _box_link(name: str, *, collision_name: str | None = None) -> str:
    name_attribute = f' name="{collision_name}"' if collision_name is not None else ""
    return f"""
  <link name="{name}">
    <collision{name_attribute}>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>"""


def _fixed_joint(name: str, parent: str, child: str, xyz: str = "0 0 0") -> str:
    return f"""
  <joint name="{name}" type="fixed">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz}"/>
  </joint>"""


def test_fk_accepts_expanded_revolute_and_mimic_follower_values(
    tmp_path: Path,
) -> None:
    urdf = _write_urdf(
        tmp_path,
        """
  <link name="base"/>
  <link name="arm"/>
  <link name="slider"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.2" upper="3.2" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="prismatic">
    <parent link="arm"/>
    <child link="slider"/>
    <origin xyz="1 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
    <mimic joint="hinge" multiplier="2" offset="0.1"/>
  </joint>""",
    )
    model = oracle.load_model(urdf)

    # Mimic expansion is owned by the sampling compiler.  The oracle consumes
    # one explicit value for every non-fixed URDF joint, including followers.
    by_name = model.forward_kinematics({"hinge": math.pi / 2.0, "follower": 0.5})
    by_order = model.forward_kinematics([math.pi / 2.0, 0.5])

    assert [joint.name for joint in model.movable_joints] == ["hinge", "follower"]
    assert np.allclose(by_name["arm"][:3, 3], [1.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(by_name["slider"][:3, 3], [1.0, 1.5, 0.0], atol=1e-12)
    assert np.allclose(
        by_name["slider"][:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-12,
    )
    assert np.allclose(by_name["slider"], by_order["slider"], atol=1e-12)


@pytest.mark.parametrize(
    ("separation_m", "expected_intersections"),
    ((1.25, 0), (0.75, 1)),
)
def test_fcl_box_pair_distinguishes_separation_and_overlap(
    tmp_path: Path, separation_m: float, expected_intersections: int
) -> None:
    urdf = _write_urdf(
        tmp_path,
        "\n".join(
            (
                '  <link name="root"/>',
                _box_link("left", collision_name="left_box"),
                _box_link("right", collision_name="right_box"),
                _fixed_joint("left_mount", "root", "left"),
                _fixed_joint("right_mount", "root", "right", f"{separation_m} 0 0"),
            )
        ),
    )

    result = oracle.load_model(urdf).observe([])

    assert result["all_pair_intersection_count"] == expected_intersections
    assert result["non_adjacent_intersection_count"] == expected_intersections
    if expected_intersections:
        assert result["raw_non_adjacent_illegal_penetration_count"] > 0
        assert result["raw_non_adjacent_max_penetration_proxy_m"] == pytest.approx(
            0.25, abs=1e-8
        )
    else:
        assert result["all_pair_contact_count"] == 0
        assert result["raw_non_adjacent_max_penetration_proxy_m"] == 0.0


def test_direct_parent_pairs_are_excluded_but_fixed_grandchild_is_retained(
    tmp_path: Path,
) -> None:
    urdf = _write_urdf(
        tmp_path,
        "\n".join(
            (
                _box_link("base", collision_name="base_box"),
                _box_link("child", collision_name="child_box"),
                _box_link("grandchild", collision_name="grandchild_box"),
                _fixed_joint("base_to_child", "base", "child"),
                _fixed_joint("child_to_grandchild", "child", "grandchild"),
            )
        ),
    )

    result = oracle.load_model(urdf).observe({}, phase="rest")

    assert result["all_pair_intersection_count"] == 3
    assert result["non_adjacent_intersection_count"] == 1
    assert result["direct_parent_pairs_excluded_from_non_adjacent_only"] is True
    non_adjacent_pairs = {
        frozenset((contact["link_a_name"], contact["link_b_name"]))
        for contact in result["raw_contacts"]
        if contact["non_adjacent"]
    }
    assert non_adjacent_pairs == {frozenset(("base", "grandchild"))}


def test_contact_evidence_names_elements_and_preserves_local_coordinates(
    tmp_path: Path,
) -> None:
    urdf = _write_urdf(
        tmp_path,
        "\n".join(
            (
                '  <link name="root"/>',
                _box_link("left", collision_name="left_box"),
                _box_link("right", collision_name="right_box"),
                _fixed_joint("left_mount", "root", "left", "0.1 0 0"),
                _fixed_joint("right_mount", "root", "right", "0.6 0 0"),
            )
        ),
    )

    result = oracle.load_model(urdf).observe(
        [], phase="multi_joint_sobol", dataset="fixture", asset_id="box_pair"
    )

    assert result["phase"] == "multi_joint_sobol"
    assert result["recorded_contact_count"] > 0
    contact = result["raw_contacts"][0]
    assert contact["link_a_name"] == "left"
    assert contact["link_b_name"] == "right"
    assert contact["collision_element_a_name"] == "left_box"
    assert contact["collision_element_b_name"] == "right_box"
    assert contact["collision_element_a_id"] == "left:0"
    assert contact["collision_element_b_id"] == "right:0"
    assert contact["geometry_kind_a"] == "box"
    assert contact["geometry_kind_b"] == "box"
    assert contact["depth_proxy_available"] is True
    assert contact["penetration_depth_proxy_m"] == pytest.approx(0.5, abs=1e-8)
    assert contact["penetration_depth_m"] == contact["penetration_depth_proxy_m"]
    assert len(contact["world_position_m"]) == 3
    assert len(contact["local_position_a_m"]) == 3
    assert len(contact["local_position_b_m"]) == 3
    assert np.all(np.isfinite(contact["world_position_m"]))
    assert np.all(np.isfinite(contact["local_position_a_m"]))
    assert np.all(np.isfinite(contact["local_position_b_m"]))
    assert contact["intended_contact"] is False
    assert contact["intended_contact_match"] == {
        "intended_contact": False,
        "reason": "intended_contact_registry_not_configured",
    }


@pytest.mark.parametrize(
    ("filename", "error_type", "message"),
    (
        ("missing.obj", FileNotFoundError, "collision mesh is missing"),
        (
            "../outside.obj",
            oracle.SimulatorFreeOracleError,
            "collision mesh escapes package",
        ),
        (
            "/tmp/outside.obj",
            oracle.SimulatorFreeOracleError,
            "absolute/file URI collision mesh reference is forbidden",
        ),
    ),
)
def test_invalid_mesh_paths_fail_closed(
    tmp_path: Path, filename: str, error_type: type[Exception], message: str
) -> None:
    urdf = _write_urdf(
        tmp_path,
        f"""
  <link name="base">
    <collision name="bad_mesh">
      <geometry><mesh filename="{filename}"/></geometry>
    </collision>
  </link>""",
    )

    with pytest.raises(error_type, match=message):
        oracle.load_model(urdf)


def test_runtime_identity_does_not_import_a_simulator() -> None:
    probe = f"""
import importlib.util
import json
import sys

path = {str(SCRIPT)!r}
spec = importlib.util.spec_from_file_location("simfree_runtime_probe", path)
assert spec and spec.loader
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
before = sorted(name for name in module.FORBIDDEN_SIMULATOR_MODULES if name in sys.modules)
identity = module.runtime_identity()
after = sorted(name for name in module.FORBIDDEN_SIMULATOR_MODULES if name in sys.modules)
print(json.dumps({{"before": before, "after": after, "identity": identity}}))
"""
    completed = subprocess.run(
        [sys.executable, "-I", "-c", probe],
        check=True,
        capture_output=True,
        text=True,
        timeout=90,
    )
    payload = json.loads(completed.stdout)

    assert payload["before"] == []
    assert payload["after"] == []
    assert payload["identity"]["simulator_modules_loaded"] == []
    assert payload["identity"]["simulator_free_runtime_verified"] is True
    assert payload["identity"]["backend"] == oracle.ORACLE_POLICY_ID
