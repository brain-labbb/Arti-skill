from __future__ import annotations

import copy
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "exp/scripts"))

import table5_pva_physics as physics  # noqa: E402


FIELDS = list(physics.PHYSICS_FIELDS)


def _write_fixture(tmp_path: Path) -> tuple[Path, Path, bytes]:
    urdf = tmp_path / "model.urdf"
    urdf.write_text(
        """<robot name="fixture">
  <link name="base">
    <visual name="c_box"><geometry><box size="1 2 3"/></geometry></visual>
    <visual name="c_ball"><geometry><sphere radius="1"/></geometry></visual>
    <collision name="c_box"><geometry><box size="1 2 3"/></geometry></collision>
    <collision name="c_ball"><origin xyz="1 0 0"/><geometry><sphere radius="1"/></geometry></collision>
  </link>
  <link name="kept">
    <visual name="c_kept"><geometry><box size="1 1 1"/></geometry></visual>
    <collision name="c_kept"><geometry><box size="1 1 1"/></geometry></collision>
    <inertial><mass value="7"/><inertia ixx="1" ixy="0" ixz="0" iyy="1.2" iyz="0" izz="1.4"/></inertial>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    source_hash = physics.sha256_file(urdf)

    def binding(key: str, density: float, dynamic: float) -> dict:
        return {
            "surface_key": key,
            "appearance_only": False,
            "values": {
                "density_kg_m3": density,
                "youngs_modulus_pa": 1.0e6,
                "poissons_ratio": 0.3,
                "static_friction_coefficient": dynamic + 0.1,
                "dynamic_friction_coefficient": dynamic,
                "restitution_coefficient": 0.2,
            },
        }

    sidecar = {
        "schema_version": physics.SIDECAR_SCHEMA,
        "model_urdf_sha256": source_hash,
        "fields": FIELDS,
        "bindings": [
            binding("base::c_box", 2.0, 0.2),
            binding("base::c_ball", 3.0, 0.8),
            binding("kept::c_kept", 4.0, 0.4),
        ],
    }
    sidecar_path = tmp_path / "physics.json"
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    return urdf, sidecar_path, urdf.read_bytes()


def test_overlay_derives_missing_inertia_and_preserves_valid_source(tmp_path: Path) -> None:
    urdf, sidecar, original = _write_fixture(tmp_path)
    destination = tmp_path / "out/model.physics.urdf"
    plan_path = tmp_path / "out/physics_plan.json"

    plan = physics.build_injected_asset(
        source_urdf=urdf,
        physics_path=sidecar,
        destination_urdf=destination,
        plan_path=plan_path,
    )

    assert urdf.read_bytes() == original
    assert plan["derived_inertial_link_count"] == 1
    assert plan["preserved_inertial_link_count"] == 1
    assert plan["binding_count"] == 3
    assert plan["injected_urdf_sha256"] == physics.sha256_file(destination)
    assert plan["plan_sha256"] == physics.sha256_file(plan_path)

    import xml.etree.ElementTree as ET

    root = ET.parse(destination).getroot()
    base = root.find("link[@name='base']")
    kept = root.find("link[@name='kept']")
    assert base is not None and kept is not None
    assert float(base.find("inertial/mass").get("value")) == pytest.approx(
        2.0 * 6.0 + 3.0 * (4.0 * 3.141592653589793 / 3.0)
    )
    # Use the plan as the numeric authority; it must contain a positive,
    # additive collision-solid result and the source mass must remain exactly 7.
    derived = next(item for item in plan["links"] if item["link_name"] == "base")
    assert derived["inertial"]["action"].startswith("derived_")
    assert derived["inertial"]["mass_kg"] > 12.0
    kept_plan = next(item for item in plan["links"] if item["link_name"] == "kept")
    assert kept_plan["inertial"]["action"] == "preserved_valid_source_urdf"
    assert kept_plan["inertial"]["mass_kg"] == 7.0
    names = [item.get("name") for item in root.findall(".//collision")]
    assert len(names) == len(set(names)) == 3

    expected_mu = (22.0 * 0.2 + 4.0 * 3.141592653589793 * 0.8) / (
        22.0 + 4.0 * 3.141592653589793
    )
    assert derived["dynamic_friction_coefficient"] == pytest.approx(expected_mu)


def test_sidecar_rejects_source_hash_and_field_drift(tmp_path: Path) -> None:
    urdf, sidecar, _ = _write_fixture(tmp_path)
    payload = json.loads(sidecar.read_text())
    with pytest.raises(physics.PhysicsInjectionError, match="bound"):
        physics.load_sidecar(sidecar, source_urdf_sha256="0" * 64)
    payload["bindings"][0]["values"] = copy.deepcopy(payload["bindings"][0]["values"])
    payload["bindings"][0]["values"].pop("youngs_modulus_pa")
    sidecar.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(physics.PhysicsInjectionError, match="exact field set"):
        physics.load_sidecar(sidecar, source_urdf_sha256=physics.sha256_file(urdf))


def test_plan_rejects_injected_urdf_hash_drift(tmp_path: Path) -> None:
    urdf, sidecar, _ = _write_fixture(tmp_path)
    destination = tmp_path / "out/model.physics.urdf"
    plan_path = tmp_path / "out/physics_plan.json"
    plan = physics.build_injected_asset(
        source_urdf=urdf,
        physics_path=sidecar,
        destination_urdf=destination,
        plan_path=plan_path,
    )
    destination.write_bytes(destination.read_bytes() + b" ")
    with pytest.raises(physics.PhysicsInjectionError, match="injected_urdf_sha256"):
        physics.load_plan(
            plan_path,
            source_urdf_sha256=plan["source_urdf_sha256"],
            physics_sha256=plan["physics_sha256"],
            injected_urdf_sha256=physics.sha256_file(destination),
        )
