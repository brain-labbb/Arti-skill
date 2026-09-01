from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path

import pytest


EXP = Path(__file__).parents[1]
SCRIPT = EXP / "scripts" / "table4_intended_contact_policy.py"
SPEC = importlib.util.spec_from_file_location("table4_intended_contact_policy", SCRIPT)
assert SPEC and SPEC.loader
policy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(policy)


URDF = """<robot name="fixture">
  <link name="left">
    <collision name="left_hinge"><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="right">
    <collision name="right_hinge"><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="right_joint" type="revolute">
    <parent link="left"/><child link="right"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def digest(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()


def registry(urdf_sha256: str) -> dict[str, object]:
    return {
        "schema_version": policy.REGISTRY_SCHEMA,
        "policy_id": "reviewed_local_contacts_20260829",
        "entries": [
            {
                "registration_id": "fixture_coaxial_hinge",
                "dataset": "pva",
                "asset_id": "PV-A/fixture/seed_0000",
                "urdf_sha256": urdf_sha256,
                "link_pair": ["left", "right"],
                "allowed_phases": [
                    "rest",
                    "single_joint_sweep",
                    "multi_joint_sobol",
                ],
                "local_regions_m": {
                    "left": {
                        "component": "coaxial_hinge_root_left",
                        "collision_elements": ["left_hinge"],
                        "minimum": [-0.01, -0.01, -0.01],
                        "maximum": [0.01, 0.01, 0.01],
                    },
                    "right": {
                        "component": "coaxial_hinge_root_right",
                        "collision_elements": ["right_hinge"],
                        "minimum": [-0.01, -0.01, -0.01],
                        "maximum": [0.01, 0.01, 0.01],
                    },
                },
                "max_penetration_m": 0.0011,
                "reason": "reviewed coaxial hinge interface",
                "review": {
                    "status": "approved",
                    "reviewer": "fixture-reviewer",
                    "approved_at": "2026-08-29T00:00:00Z",
                    "evidence_sha256": "a" * 64,
                },
            }
        ],
    }


@pytest.fixture()
def bound(tmp_path: Path) -> tuple[Path, dict[str, object]]:
    urdf = tmp_path / "model.urdf"
    urdf.write_text(URDF, encoding="utf-8")
    normalized = policy.validate_registry(registry(policy.sha256_file(urdf)))
    policy.validate_entry_urdf_binding(normalized["entries"][0], urdf)
    return urdf, normalized


def match(normalized: dict[str, object], **changes: object) -> dict[str, object]:
    values: dict[str, object] = {
        "dataset": "pva",
        "asset_id": "PV-A/fixture/seed_0000",
        "urdf_sha256": digest(URDF),
        "phase": "rest",
        "link_a_name": "left",
        "link_b_name": "right",
        "penetration_depth_m": 0.001,
        "local_position_a_m": [0.0, 0.0, 0.0],
        "local_position_b_m": [0.0, 0.0, 0.0],
        "collision_element_a_name": "left_hinge",
        "collision_element_b_name": "right_hinge",
    }
    values.update(changes)
    return policy.match_contact(normalized, **values)


def test_exact_local_contact_and_reversed_pair_match(bound) -> None:
    _, normalized = bound
    result = match(normalized)
    assert result["intended_contact"] is True
    assert result["registration_id"] == "fixture_coaxial_hinge"

    reversed_result = match(
        normalized,
        link_a_name="right",
        link_b_name="left",
        local_position_a_m=[0.0, 0.0, 0.0],
        local_position_b_m=[0.0, 0.0, 0.0],
        collision_element_a_name="right_hinge",
        collision_element_b_name="left_hinge",
    )
    assert reversed_result["intended_contact"] is True


@pytest.mark.parametrize(
    ("changes", "reason"),
    (
        ({"asset_id": "PV-A/fixture/seed_0001"}, "asset_or_urdf_not_registered"),
        ({"urdf_sha256": "b" * 64}, "asset_or_urdf_not_registered"),
        ({"phase": "unknown"}, "contact_outside_registered_scope"),
        ({"link_b_name": "other"}, "contact_outside_registered_scope"),
        ({"penetration_depth_m": 0.0011001}, "contact_outside_registered_scope"),
        ({"local_position_a_m": [0.02, 0.0, 0.0]}, "contact_outside_registered_scope"),
        ({"local_position_a_m": None}, "missing_or_invalid_local_position"),
        (
            {"collision_element_a_name": "other"},
            "contact_outside_registered_scope",
        ),
        (
            {"collision_element_a_name": None},
            "missing_or_invalid_collision_element",
        ),
    ),
)
def test_every_scope_dimension_fails_closed(bound, changes, reason) -> None:
    _, normalized = bound
    result = match(normalized, **changes)
    assert result == {"intended_contact": False, "reason": reason}


def test_registry_rejects_category_or_wildcard_scope() -> None:
    broad = registry("a" * 64)
    broad["entries"][0]["category"] = "fixture"
    with pytest.raises(policy.IntendedContactPolicyError, match="extra=.*category"):
        policy.validate_registry(broad)

    wildcard = registry("a" * 64)
    wildcard["entries"][0]["asset_id"] = "PV-A/fixture/*"
    with pytest.raises(policy.IntendedContactPolicyError, match="wildcard"):
        policy.validate_registry(wildcard)


def test_registry_requires_two_local_regions_and_approval() -> None:
    missing_region = registry("a" * 64)
    del missing_region["entries"][0]["local_regions_m"]["right"]
    with pytest.raises(policy.IntendedContactPolicyError, match="both and only"):
        policy.validate_registry(missing_region)

    unapproved = registry("a" * 64)
    unapproved["entries"][0]["review"]["status"] = "pending"
    with pytest.raises(policy.IntendedContactPolicyError, match="must be approved"):
        policy.validate_registry(unapproved)


def test_urdf_hash_and_collision_element_bindings_fail_closed(bound, tmp_path) -> None:
    urdf, normalized = bound
    entry = normalized["entries"][0]
    drift = tmp_path / "drift.urdf"
    drift.write_text(URDF.replace("left_hinge", "changed_hinge"), encoding="utf-8")
    with pytest.raises(policy.IntendedContactPolicyError, match="SHA-256"):
        policy.validate_entry_urdf_binding(entry, drift)

    forged = dict(entry)
    forged["urdf_sha256"] = policy.sha256_file(urdf)
    forged["local_regions_m"] = dict(entry["local_regions_m"])
    forged["local_regions_m"]["left"] = dict(entry["local_regions_m"]["left"])
    forged["local_regions_m"]["left"]["collision_elements"] = ("missing",)
    with pytest.raises(policy.IntendedContactPolicyError, match="absent"):
        policy.validate_entry_urdf_binding(forged, urdf)


def test_duplicate_matching_entries_are_not_allowed_to_broaden_scope(bound) -> None:
    _, normalized = bound
    duplicate = dict(normalized["entries"][0])
    duplicate["registration_id"] = "second_review"
    normalized["entries"].append(duplicate)
    assert match(normalized) == {
        "intended_contact": False,
        "reason": "ambiguous_registry_match",
    }


def test_asset_binding_rejects_stale_hash_and_keeps_other_assets_out(bound) -> None:
    urdf, normalized = bound
    bound_registry = policy.bind_registry_for_asset(
        normalized,
        dataset="pva",
        asset_id="PV-A/fixture/seed_0000",
        urdf=urdf,
    )
    assert len(bound_registry["entries"]) == 1
    empty = policy.bind_registry_for_asset(
        normalized,
        dataset="pva",
        asset_id="PV-A/fixture/seed_0001",
        urdf=urdf,
    )
    assert empty["entries"] == []

    normalized["entries"][0]["urdf_sha256"] = "b" * 64
    with pytest.raises(policy.IntendedContactPolicyError, match="SHA-256"):
        policy.bind_registry_for_asset(
            normalized,
            dataset="pva",
            asset_id="PV-A/fixture/seed_0000",
            urdf=urdf,
        )


class FakeBullet:
    def getLinkState(self, body, link, **kwargs):
        assert (body, link) == (7, 3)
        return (None, None, None, None, (10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def getBasePositionAndOrientation(self, body, **kwargs):
        assert body == 7
        return (0.0, 20.0, 0.0), (0.0, 0.0, 0.0, 1.0)

    def getDynamicsInfo(self, body, link, **kwargs):
        assert (body, link) == (7, -1)
        return (None, None, None, (0.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def invertTransform(self, position, orientation):
        return tuple(-value for value in position), orientation

    def multiplyTransforms(self, position_a, orientation_a, position_b, orientation_b):
        return tuple(a + b for a, b in zip(position_a, position_b)), orientation_a


def test_pybullet_contact_evidence_uses_each_link_local_frame() -> None:
    contact = [None] * 14
    contact[3] = 3
    contact[4] = -1
    contact[5] = (11.0, 2.0, 3.0)
    contact[6] = (4.0, 22.0, 6.0)
    contact[8] = -0.002
    evidence = policy.pybullet_contact_evidence(
        FakeBullet(),
        body=7,
        client=9,
        contact=contact,
        link_names={3: "moving", -1: "base"},
    )
    assert evidence == {
        "link_a_name": "moving",
        "link_b_name": "base",
        "penetration_depth_m": 0.002,
        "local_position_a_m": (1.0, 2.0, 3.0),
        "local_position_b_m": (4.0, 7.0, 6.0),
    }


def test_real_pybullet_base_contact_uses_urdf_link_not_com_frame(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = tmp_path / "base_inertial.urdf"
    urdf.write_text(
        """<robot name="base_inertial">
  <link name="base">
    <inertial>
      <origin xyz="0.31 -0.22 0.17" rpy="0.2 -0.3 0.4"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <collision><geometry><sphere radius="0.01"/></geometry></collision>
  </link>
</robot>""",
        encoding="ascii",
    )
    client = bullet.connect(bullet.DIRECT)
    try:
        body = bullet.loadURDF(
            str(urdf),
            useFixedBase=True,
            flags=bullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=client,
        )
        expected = (0.4, -0.2, 0.1)
        observed = policy._world_to_link_local(
            bullet, body, client, -1, expected
        )
        assert observed == pytest.approx(expected, abs=1e-7)
    finally:
        bullet.disconnect(client)


def test_collision_element_identity_is_link_directional_and_required(bound) -> None:
    _, normalized = bound

    assert match(
        normalized,
        collision_element_a_name="right_hinge",
        collision_element_b_name="left_hinge",
    ) == {
        "intended_contact": False,
        "reason": "contact_outside_registered_scope",
    }
    assert policy.match_contact(
        normalized,
        dataset="pva",
        asset_id="PV-A/fixture/seed_0000",
        urdf_sha256=digest(URDF),
        phase="rest",
        link_a_name="left",
        link_b_name="right",
        penetration_depth_m=0.001,
        local_position_a_m=[0.0, 0.0, 0.0],
        local_position_b_m=[0.0, 0.0, 0.0],
    ) == {
        "intended_contact": False,
        "reason": "missing_or_invalid_collision_element",
    }
