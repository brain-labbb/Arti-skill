from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[2]
MODULE = REPO / "exp/scripts/lam_supplementary_geometry.py"


def load_module():
    spec = importlib.util.spec_from_file_location("lam_supplementary_geometry", MODULE)
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
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
"""


def write_mesh_fixture(tmp_path: Path) -> Path:
    package = tmp_path / "package"
    package.mkdir()
    (package / "cube.obj").write_text(CUBE_OBJ, encoding="utf-8")
    urdf = package / "model.urdf"
    urdf.write_text(
        """<robot name="fixture">
<link name="base">
  <visual><origin xyz="1 0 0"/><geometry><mesh filename="cube.obj" scale="2 1 1"/></geometry></visual>
  <collision><origin xyz="1 0 0"/><geometry><mesh filename="cube.obj" scale="2 1 1"/></geometry></collision>
</link>
<link name="child">
  <visual><geometry><mesh filename="cube.obj"/></geometry></visual>
  <collision><geometry><mesh filename="cube.obj"/></geometry></collision>
</link>
<joint name="mount" type="fixed">
  <parent link="base"/><child link="child"/><origin xyz="0 3 0"/>
</joint>
</robot>\n""",
        encoding="utf-8",
    )
    return urdf


def test_q0_fk_mesh_scale_dvisual_inventory_and_atomic_redundancy(tmp_path: Path) -> None:
    module = load_module()
    urdf = write_mesh_fixture(tmp_path)

    record = module.evaluate_table4b_geometry(
        urdf, "viable:objects/fixture", "table4b_test_v1", samples_per_direction=128
    )

    assert record["tree_valid"] is True
    assert record["declared_visual_element_count"] == 2
    assert record["loadable_visual_element_count"] == 2
    assert record["declared_collision_element_count"] == 2
    assert record["loadable_collision_element_count"] == 2
    assert record["d_visual"] == pytest.approx(math.sqrt(23.25))
    assert record["d_visual_bounds_min"] == pytest.approx([-0.5, -0.5, -0.5])
    assert record["d_visual_bounds_max"] == pytest.approx([2.0, 3.5, 0.5])
    assert record["analytic_collision_share"] == 0.0
    assert record["collision_shapes_per_visual_bearing_link"] == 1.0
    assert record["collision_mesh_valid_triangle_count"] == 24
    assert record["collision_watertight_mesh_count"] == 2
    assert record["collision_finite_positive_volume_mesh_count"] == 2
    assert record["intra_link_redundancy"] == 0.0
    assert record["intra_link_redundancy_status"] == "COMPLETE"
    assert record["redundancy_measured_link_count"] == 2
    assert all(
        row["protocol"] == "single_shape_union_identity_v1"
        for row in record["redundancy_link_records"]
    )
    assert record["visual_to_collision"]["status"] == "COMPLETE"
    assert record["collision_to_visual"]["status"] == "COMPLETE"
    assert record["visual_to_collision"]["normalized_p95"] == pytest.approx(0.0, abs=1e-12)
    assert record["collision_to_visual"]["normalized_p95"] == pytest.approx(0.0, abs=1e-12)
    json.dumps(record, sort_keys=True)


def test_seed_is_exact_sha256_contract_and_results_are_deterministic(tmp_path: Path) -> None:
    module = load_module()
    urdf = write_mesh_fixture(tmp_path)
    protocol_id = "protocol"
    asset_key = "asset"
    expected = hashlib.sha256(
        b"protocol|asset|visual_to_collision"
    ).hexdigest()

    first = module.evaluate_table4b_geometry(
        urdf, asset_key, protocol_id, samples_per_direction=64
    )
    second = module.evaluate_table4b_geometry(
        urdf, asset_key, protocol_id, samples_per_direction=64
    )

    assert first["visual_to_collision"]["seed_sha256"] == expected
    assert first["visual_to_collision"] == second["visual_to_collision"]
    assert module.seed_sha256(protocol_id, asset_key, "visual_to_collision") == expected


def test_analytic_inventory_does_not_inflate_collision_mesh_triangles(tmp_path: Path) -> None:
    module = load_module()
    urdf = tmp_path / "box.urdf"
    urdf.write_text(
        """<robot name="box">
<link name="base">
  <visual><geometry><box size="2 4 6"/></geometry></visual>
  <collision><geometry><box size="2 4 6"/></geometry></collision>
</link>
</robot>\n""",
        encoding="utf-8",
    )

    record = module.evaluate_table4b_geometry(
        urdf, "box", "protocol", samples_per_direction=64
    )

    assert record["analytic_collision_element_count"] == 1
    assert record["analytic_collision_share"] == 1.0
    assert record["collision_mesh_element_count"] == 0
    assert record["collision_mesh_valid_triangle_count"] == 0
    assert record["d_visual"] == pytest.approx(math.sqrt(56.0))
    assert record["intra_link_redundancy"] == 0.0
    assert record["visual_to_collision"]["normalized_p95"] == pytest.approx(0.0, abs=1e-12)


def test_unsafe_mesh_traversal_is_retained_not_raised(tmp_path: Path) -> None:
    module = load_module()
    (tmp_path / "outside.obj").write_text(CUBE_OBJ, encoding="utf-8")
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(
        """<robot name="unsafe"><link name="base">
<visual><geometry><mesh filename="../outside.obj"/></geometry></visual>
<collision><geometry><mesh filename="../outside.obj"/></geometry></collision>
</link></robot>\n""",
        encoding="utf-8",
    )

    record = module.evaluate_table4b_geometry(
        urdf, "unsafe", "protocol", samples_per_direction=16
    )

    assert record["declared_visual_element_count"] == 1
    assert record["loadable_visual_element_count"] == 0
    assert record["declared_collision_element_count"] == 1
    assert record["loadable_collision_element_count"] == 0
    assert record["status"] == "N/E"
    assert any("escapes the package root" in issue for issue in record["issues"])


def test_backend_unavailable_never_hides_static_fields_or_uses_kdtree(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = load_module()
    urdf = write_mesh_fixture(tmp_path)
    unavailable = {
        "backend": module.EXACT_BACKEND,
        "available": False,
        "status": "N/E",
        "no_kdtree_fallback": True,
        "reason": "fixture missing rtree",
    }
    monkeypatch.setattr(module, "qualify_exact_surface_backend", lambda: dict(unavailable))

    record = module.evaluate_table4b_geometry(
        urdf, "asset", "protocol", samples_per_direction=16
    )

    assert record["collision_mesh_valid_triangle_count"] == 24
    assert record["intra_link_redundancy"] == 0.0
    assert record["visual_to_collision"]["status"] == "N/E"
    assert record["collision_to_visual"]["status"] == "N/E"
    assert record["visual_to_collision"]["backend"]["no_kdtree_fallback"] is True
    assert record["status"] == "PARTIAL"


def test_exact_backend_known_distance_qualification() -> None:
    module = load_module()
    qualification = module.qualify_exact_surface_backend()
    if not qualification["available"]:
        pytest.skip(qualification.get("reason", "exact backend unavailable"))
    distances = module.exact_closest_surface_distances(
        np.asarray(((0.25, 0.25, 2.0), (2.0, 0.0, 0.0))),
        np.asarray(((0, 0, 0), (1, 0, 0), (0, 1, 0)), dtype=float),
        np.asarray(((0, 1, 2),), dtype=int),
    )
    assert distances["status"] == "COMPLETE"
    assert distances["distances"] == pytest.approx([2.0, 1.0])
    assert distances["backend"]["no_kdtree_fallback"] is True


def test_collision_load_time_is_warm_five_repeat_fresh_bvh_protocol(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = load_module()
    urdf = write_mesh_fixture(tmp_path)
    original = module._fresh_collision_bvh_construction
    constructions = []

    def tracked(*args, **kwargs):
        result = original(*args, **kwargs)
        constructions.append(result)
        return result

    monkeypatch.setattr(module, "_fresh_collision_bvh_construction", tracked)
    record = module.measure_collision_load_time_in_asset_child(urdf)

    assert record["status"] == "COMPLETE"
    assert record["reason"] is None
    assert record["warmup_repeats"] == 1
    assert record["repeats"] == 5
    assert record["completed_repeats"] == 5
    assert len(record["raw_times_seconds"]) == 5
    assert all(value > 0.0 and math.isfinite(value) for value in record["raw_times_seconds"])
    assert record["value"] == sorted(record["raw_times_seconds"])[2]
    assert record["thread_count"] == 1
    assert all(row["num_threads"] == 1 for row in record["threadpool_controllers"])
    assert record["no_visual_fallback"] is True
    assert record["no_simulator_fallback"] is True
    assert len(constructions) == 6
    assert all(len(construction) == 2 for construction in constructions)
    meshes = [mesh for construction in constructions for mesh, _ in construction]
    trees = [tree for construction in constructions for _, tree in construction]
    assert len({id(mesh) for mesh in meshes}) == 12
    assert len({id(tree) for tree in trees}) == 12
    assert all(len(tree) == 12 for tree in trees)
    json.dumps(record, sort_keys=True)


def test_collision_load_time_ignores_visual_geometry_and_fails_without_exact_bvh(
    tmp_path: Path,
) -> None:
    module = load_module()
    package = tmp_path / "package"
    package.mkdir()
    (package / "cube.obj").write_text(CUBE_OBJ, encoding="utf-8")
    mesh_urdf = package / "mesh.urdf"
    mesh_urdf.write_text(
        """<robot name="collision_only"><link name="base">
<visual><geometry><mesh filename="missing-visual.obj"/></geometry></visual>
<collision><geometry><mesh filename="cube.obj"/></geometry></collision>
</link></robot>\n""",
        encoding="utf-8",
    )

    collision_only = module.load_urdf_geometry(mesh_urdf, roles=("collision",))
    assert collision_only.issues == []
    assert collision_only.declared_counts == {"visual": 0, "collision": 1}
    assert module.measure_collision_load_time_in_asset_child(mesh_urdf)["status"] == "COMPLETE"

    sphere_urdf = package / "sphere.urdf"
    sphere_urdf.write_text(
        """<robot name="sphere"><link name="base">
<collision><geometry><sphere radius="1"/></geometry></collision>
</link></robot>\n""",
        encoding="utf-8",
    )
    unavailable = module.measure_collision_load_time_in_asset_child(sphere_urdf)
    assert unavailable["status"] == "FAILED"
    assert unavailable["value"] is None
    assert unavailable["completed_repeats"] == 0
    assert unavailable["raw_times_seconds"] == []
    assert "no exact triangle-BVH representation" in unavailable["reason"]


def test_triangle_and_redundancy_measurement_helpers_are_fail_closed(tmp_path: Path) -> None:
    module = load_module()
    record = module.evaluate_table4b_geometry(
        write_mesh_fixture(tmp_path),
        "asset",
        "protocol",
        samples_per_direction=16,
    )

    triangles = module.collision_triangle_validation_measurement(record)
    assert triangles == {
        "status": "COMPLETE",
        "value": 24,
        "intended_mesh_count": 2,
        "measured_mesh_count": 2,
        "reason": None,
    }
    redundancy = module.collision_redundancy_measurement(record)
    assert redundancy["status"] == "COMPLETE"
    assert redundancy["value"] == pytest.approx(0.0)
    assert redundancy["shape_volume_m3"] == pytest.approx(3.0)
    assert redundancy["redundant_volume_m3"] == pytest.approx(0.0)

    incomplete_triangles = dict(record)
    incomplete_triangles["collision_mesh_validation_measured"] = 1
    assert module.collision_triangle_validation_measurement(incomplete_triangles)["status"] == "N/E"

    incomplete_redundancy = dict(record)
    incomplete_redundancy["redundancy_measured_link_count"] = 1
    incomplete_redundancy["intra_link_redundancy_status"] = "PARTIAL"
    incomplete_redundancy["redundancy_link_records"] = [
        dict(record["redundancy_link_records"][0]),
        {
            **record["redundancy_link_records"][1],
            "status": "N/E",
            "sum_eligible_shape_volume": None,
            "union_volume": None,
            "redundancy": None,
        },
    ]
    partial = module.collision_redundancy_measurement(incomplete_redundancy)
    assert partial["status"] == "PARTIAL"
    assert partial["value"] == pytest.approx(0.0)
    assert partial["shape_volume_m3"] == pytest.approx(2.0)
    assert partial["redundant_volume_m3"] == pytest.approx(0.0)

    no_collision = dict(record)
    no_collision.update(
        declared_collision_element_count=0,
        loadable_collision_element_count=0,
        collision_mesh_validation_intended=0,
        collision_mesh_validation_measured=0,
        collision_mesh_valid_triangle_count=0,
        declared_collision_bearing_link_count=0,
        redundancy_intended_collision_bearing_link_count=0,
        redundancy_measured_link_count=0,
        intra_link_redundancy_status="N/E",
    )
    triangle_none = module.collision_triangle_validation_measurement(no_collision)
    redundancy_none = module.collision_redundancy_measurement(no_collision)
    assert triangle_none["status"] == "N/E"
    assert triangle_none["reason"] == "asset declares no collision geometry"
    assert redundancy_none["status"] == "N/E"
    assert redundancy_none["reason"] == "asset declares no collision geometry"
