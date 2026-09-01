from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_physx_mobility.py"
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_urdf_table4b_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


OBJ_TEXT = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\n"


def _urdf_with(visual: bool, collision: str | None) -> str:
    visual_xml = ""
    if visual:
        visual_xml = (
            '<visual><geometry><mesh filename="objs/shape.obj" scale="1 1 1"/></geometry>'
            '<origin xyz="0 0 0" rpy="0 0 0"/></visual>'
        )
    collision_xml = ""
    if collision == "box":
        collision_xml = '<collision><geometry><box size="1 1 1"/></geometry></collision>'
    elif collision == "mesh":
        collision_xml = '<collision><geometry><mesh filename="objs/shape.obj" scale="1 1 1"/></geometry></collision>'
    return f"""<robot name="fixture">
  <link name="l_world">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    {visual_xml}
    {collision_xml}
  </link>
</robot>
"""


def _write_package(tmp_path: Path, name: str, urdf_text: str, *, with_obj: bool = True) -> Path:
    pkg = tmp_path / "packages" / name
    pkg.mkdir(parents=True)
    if with_obj:
        (pkg / "objs").mkdir(parents=True)
        (pkg / "objs" / "shape.obj").write_text(OBJ_TEXT, encoding="utf-8")
    urdf = pkg / "model.urdf"
    urdf.write_text(urdf_text, encoding="utf-8")
    return pkg


def _base_job(runner, pkg: Path, name: str) -> dict:
    urdf = pkg / "model.urdf"
    return {
        "selection_index": 0,
        "dataset_id": "999999",
        "asset_id": "999999",
        "rank": 1,
        "category": "Fixture",
        "package": str(pkg),
        "package_binding": runner.package_binding(pkg),
        "urdf_path": str(urdf),
        "expected_urdf_sha256": runner.sha256_bytes(urdf.read_bytes()),
        "original_urdf_sha256": runner.sha256_bytes(urdf.read_bytes()),
        "input_identity_sha256": "fixture-row",
    }


def _run_child(runner, tmp_path: Path, job: dict, name: str) -> dict:
    job_root = tmp_path / f"child_{name}"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    job_path.write_text(json.dumps(job, sort_keys=True), encoding="utf-8")
    completed = subprocess.run(
        [sys.executable, str(RUNNER), "--child", "--job", str(job_path), "--result", str(result_path)],
        capture_output=True,
        text=True,
        timeout=600,
        cwd=str(REPO),
    )
    assert completed.returncode == 0, completed.stderr[-2000:]
    return json.loads(result_path.read_text(encoding="utf-8"))


def test_child_zero_collision_asset_all_ne(tmp_path: Path) -> None:
    runner = _load_runner()
    pkg = _write_package(tmp_path, "zc", _urdf_with(visual=True, collision=None))
    job = _base_job(runner, pkg, "zc")

    record = _run_child(runner, tmp_path, job, "zc")

    assert record["status"] == "completed", record["issues"]
    assert record["tree_valid"] is True
    assert record["declared_collision_element_count"] == 0
    assert record["loadable_collision_element_count"] == 0
    assert record["loadable_visual_element_count"] == 1
    assert record["d_visual_status"] == "COMPLETE"
    assert record["d_visual"] > 0
    for field in (
        "visual_to_collision_p95_normalized",
        "collision_to_visual_p95_normalized",
        "shapes_per_visual_bearing_link",
        "collision_mesh_triangles_per_asset",
        "collision_load_time_seconds",
    ):
        assert record[field]["status"] == "N/E", field
    assert record["intra_link_redundancy"]["status"] == "N/E"


def test_child_collision_asset_measures(tmp_path: Path) -> None:
    runner = _load_runner()
    pkg = _write_package(tmp_path, "wc", _urdf_with(visual=True, collision="mesh"))
    job = _base_job(runner, pkg, "wc")

    record = _run_child(runner, tmp_path, job, "wc")

    assert record["status"] == "completed", record["issues"]
    assert record["declared_collision_element_count"] == 1
    assert record["loadable_collision_element_count"] == 1
    assert record["shapes_per_visual_bearing_link"]["status"] == "COMPLETE"
    assert record["shapes_per_visual_bearing_link"]["value"] == 1.0
    assert record["visual_to_collision_p95_normalized"]["status"] == "COMPLETE"
    assert record["visual_to_collision_p95_normalized"]["value"] >= 0.0
    assert record["collision_to_visual_p95_normalized"]["status"] == "COMPLETE"
    assert record["collision_mesh_triangles_per_asset"]["status"] == "COMPLETE"
    assert record["collision_load_time_seconds"]["status"] == "COMPLETE"
    assert record["collision_load_time_seconds"]["value"] >= 0.0


def test_stage_rejects_byte_drift_and_joint_rewrite(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    (root / "urdf").mkdir(parents=True)
    (root / "partseg/10/objs").mkdir(parents=True)
    urdf_text = '<robot name="r"><link name="l"><visual><geometry><mesh filename="./../partseg/10/objs/x.obj"/></geometry></visual></link></robot>'
    urdf = root / "urdf/10.urdf"
    urdf.write_text(urdf_text, encoding="utf-8")
    obj = root / "partseg/10/objs/x.obj"
    obj.write_text(OBJ_TEXT, encoding="utf-8")
    row = {
        "dataset_id": 10,
        "urdf_relative_path": "urdf/10.urdf",
        "urdf_sha256": runner.sha256_bytes(urdf.read_bytes()),
        "resources": [{"relative_path": "partseg/10/objs/x.obj", "sha256": runner.sha256_bytes(obj.read_bytes())}],
    }
    staged = runner.stage_package(root, row, tmp_path / "staging_ok")
    assert (staged["package"] / "10.urdf").is_file()
    assert staged["reference_map"] == [{"original": "./../partseg/10/objs/x.obj", "relativized": "partseg/10/objs/x.obj"}]

    obj.write_text("v 9 9 9\n", encoding="utf-8")
    with pytest.raises(ValueError, match="staged byte mismatch"):
        runner.stage_package(root, row, tmp_path / "staging_bad")

    # structural invariance violation: rewrite changes a joint declaration
    import xml.etree.ElementTree as ET
    good_pkg = tmp_path / "invar"
    good_pkg.mkdir()
    src = tmp_path / "src.urdf"
    src.write_text(
        '<robot name="r"><link name="a"/><link name="b"/>'
        '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
        '<limit lower="0" upper="1"/></joint></robot>',
        encoding="utf-8",
    )
    bad = tmp_path / "bad.urdf"
    tree = ET.parse(src)
    tree.getroot().find("joint").set("type", "prismatic")
    tree.write(bad, encoding="utf-8")
    with pytest.raises(ValueError, match="rewrite changed joint declaration"):
        runner._verify_kinematic_invariance(src, bad)


def test_aggregate_zero_collision_cohort(tmp_path: Path) -> None:
    runner = _load_runner()

    def ne(field_reason: str) -> dict:
        return {"status": "N/E", "value": None, "reason": field_reason}

    records = []
    for i in range(3):
        records.append(
            {
                "dataset_id": str(i),
                "status": "completed",
                "declared_collision_element_count": 0,
                "loadable_collision_element_count": 0,
                "visual_bearing_link_count": 2,
                "analytic_collision_element_count": 0,
                "d_visual_status": "COMPLETE",
                "d_visual": 1.5,
                "visual_to_collision_p95_normalized": ne("surface union is empty"),
                "collision_to_visual_p95_normalized": ne("surface union is empty"),
                "shapes_per_visual_bearing_link": ne("incomplete"),
                "collision_mesh_triangles_per_asset": ne("no collision geometry"),
                "collision_load_time_seconds": ne("no collision geometry"),
                "intra_link_redundancy": {
                    "status": "N/E", "value": None, "shape_volume_m3": None,
                    "redundant_volume_m3": None, "intended_link_count": 0,
                    "measured_link_count": 0, "reason": "no collision geometry",
                },
            }
        )
    agg = runner.aggregate(records)
    assert agg["status_counts"] == {"completed": 3, "error": 0, "total": 3}
    assert agg["analytic_collision_share"]["rate"] is None
    assert agg["analytic_collision_share"]["loadable_collision_elements"] == 0
    for key in (
        "visual_to_collision_p95_normalized",
        "collision_to_visual_p95_normalized",
        "shapes_per_visual_bearing_link",
        "collision_mesh_triangles_per_asset",
        "collision_load_time_seconds",
    ):
        assert agg[key]["status"] == "N/E", key
        assert agg[key]["measured"] == 0 and agg[key]["intended"] == 3
    assert agg["intra_link_redundancy"]["status"] == "N/E"
    assert agg["d_visual_coverage"] == {"complete": 3, "intended": 3}
