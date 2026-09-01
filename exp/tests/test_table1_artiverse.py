from __future__ import annotations

import importlib.util
import json
from argparse import Namespace
import subprocess
import sys
from pathlib import Path
import textwrap

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table1_artiverse.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_artiverse", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_urdf(path: Path, body: str, *, robot_name: str = "fixture") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    xml = f"""\
    <?xml version="1.0"?>
    <robot name="{robot_name}">
    {textwrap.dedent(body)}
    </robot>
    """
    path.write_text(
        textwrap.dedent(xml).lstrip(),
        encoding="utf-8",
    )
    return path


def test_cli_exposes_frozen_sampling_inputs() -> None:
    completed = subprocess.run(
        [sys.executable, str(RUNNER), "--help"],
        cwd=REPO,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "--artiverse-root" in completed.stdout
    assert "--output" in completed.stdout
    assert "--sample-size" in completed.stdout
    assert "--seed" in completed.stdout


def test_frozen_selection_is_repeatable_and_manifest_order_independent() -> None:
    runner = _load_runner()
    identities = [
        {"asset_id": f"data/category/source/model_{index:02d}"}
        for index in range(12)
    ]

    first = runner.freeze_selection(
        identities,
        sample_size=5,
        seed="20260813",
        release_manifest_sha256="a" * 64,
    )
    second = runner.freeze_selection(
        list(reversed(identities)),
        sample_size=5,
        seed="20260813",
        release_manifest_sha256="a" * 64,
    )

    assert [row["asset_id"] for row in first] == [
        "data/category/source/model_03",
        "data/category/source/model_06",
        "data/category/source/model_02",
        "data/category/source/model_04",
        "data/category/source/model_05",
    ]
    assert first == second
    assert [row["selection_rank"] for row in first] == [1, 2, 3, 4, 5]
    assert len({row["selection_hash"] for row in first}) == 5


def test_frozen_selection_rejects_more_assets_than_the_release() -> None:
    runner = _load_runner()

    with pytest.raises(ValueError, match="sample size 3 exceeds release size 2"):
        runner.freeze_selection(
            [{"asset_id": "data/a/s/1"}, {"asset_id": "data/a/s/2"}],
            sample_size=3,
            seed="20260813",
            release_manifest_sha256="b" * 64,
        )


def test_topology_hash_ignores_names_mesh_paths_and_numeric_parameters(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first.urdf",
        """
        <link name="base"><visual><geometry><mesh filename="a.obj"/></geometry></visual></link>
        <link name="door"/>
        <joint name="hinge" type="revolute">
          <parent link="base"/><child link="door"/>
          <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
        """,
    )
    second = _write_urdf(
        tmp_path / "second.urdf",
        """
        <link name="renamed_root"><visual><geometry><mesh filename="elsewhere.glb"/></geometry></visual></link>
        <link name="renamed_child"/>
        <joint name="renamed_joint" type="revolute">
          <parent link="renamed_root"/><child link="renamed_child"/>
          <origin xyz="9 8 7"/><axis xyz="1 0 0"/>
          <limit lower="-3.5" upper="8.25" effort="99" velocity="4"/>
        </joint>
        """,
        robot_name="renamed_robot",
    )

    first_result = runner.analyze_urdf(first)
    second_result = runner.analyze_urdf(second)

    assert first_result["valid_tree"] is True
    assert second_result["valid_tree"] is True
    assert first_result["link_count"] == second_result["link_count"] == 2
    assert first_result["non_fixed_joint_count"] == 1
    assert first_result["topology_hash"] == second_result["topology_hash"]


def test_topology_hash_changes_with_joint_type_or_branching(tmp_path: Path) -> None:
    runner = _load_runner()
    chain = _write_urdf(
        tmp_path / "chain.urdf",
        """
        <link name="root"/><link name="middle"/><link name="leaf"/>
        <joint name="a" type="revolute"><parent link="root"/><child link="middle"/></joint>
        <joint name="b" type="fixed"><parent link="middle"/><child link="leaf"/></joint>
        """,
    )
    changed_type = _write_urdf(
        tmp_path / "changed_type.urdf",
        """
        <link name="root"/><link name="middle"/><link name="leaf"/>
        <joint name="a" type="prismatic"><parent link="root"/><child link="middle"/></joint>
        <joint name="b" type="fixed"><parent link="middle"/><child link="leaf"/></joint>
        """,
    )
    branch = _write_urdf(
        tmp_path / "branch.urdf",
        """
        <link name="root"/><link name="middle"/><link name="leaf"/>
        <joint name="a" type="revolute"><parent link="root"/><child link="middle"/></joint>
        <joint name="b" type="fixed"><parent link="root"/><child link="leaf"/></joint>
        """,
    )

    chain_result = runner.analyze_urdf(chain)
    changed_type_result = runner.analyze_urdf(changed_type)
    branch_result = runner.analyze_urdf(branch)

    assert chain_result["valid_tree"] is True
    assert changed_type_result["valid_tree"] is True
    assert branch_result["valid_tree"] is True
    assert chain_result["topology_hash"] != changed_type_result["topology_hash"]
    assert chain_result["topology_hash"] != branch_result["topology_hash"]


def test_joint_counts_follow_declared_non_fixed_policy(tmp_path: Path) -> None:
    runner = _load_runner()
    urdf = _write_urdf(
        tmp_path / "declared_types.urdf",
        """
        <link name="root"/><link name="a"/><link name="b"/><link name="c"/>
        <joint name="fixed" type="fixed"><parent link="root"/><child link="a"/></joint>
        <joint name="extension" type="universal"><parent link="a"/><child link="b"/></joint>
        <joint name="standard" type="revolute"><parent link="b"/><child link="c"/></joint>
        """,
    )

    result = runner.analyze_urdf(urdf)

    assert result["joint_type_counts"] == {
        "fixed": 1,
        "revolute": 1,
        "universal": 1,
    }
    assert result["non_fixed_joint_count"] == 2


def _write_package(
    root: Path,
    *,
    robot_name: str,
    root_name: str,
    child_name: str,
    joint_name: str,
    origin: str = "0 0 0",
    texture: bytes = b"texture-v1",
) -> Path:
    root.mkdir()
    (root / "mesh.obj").write_text(
        "mtllib material.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )
    (root / "material.mtl").write_text("map_Kd texture.png\n", encoding="utf-8")
    (root / "texture.png").write_bytes(texture)
    return _write_urdf(
        root / "model.urdf",
        f"""
        <link name="{root_name}">
          <visual><origin xyz="{origin}"/><geometry><mesh filename="mesh.obj"/></geometry></visual>
        </link>
        <link name="{child_name}"/>
        <joint name="{joint_name}" type="revolute">
          <parent link="{root_name}"/><child link="{child_name}"/>
          <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
        """,
        robot_name=robot_name,
    )


def test_package_fingerprint_normalizes_names_and_tracks_resource_closure(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    first = _write_package(
        tmp_path / "first",
        robot_name="one",
        root_name="base",
        child_name="door",
        joint_name="hinge",
    )
    second = _write_package(
        tmp_path / "second",
        robot_name="two",
        root_name="renamed_base",
        child_name="renamed_door",
        joint_name="renamed_hinge",
    )

    first_result = runner.fingerprint_package(first)
    second_result = runner.fingerprint_package(second)

    assert first_result["complete"] is True
    assert first_result["resource_count"] == 3
    assert first_result["fingerprint"] == second_result["fingerprint"]

    (tmp_path / "second/texture.png").write_bytes(b"texture-v2")
    changed_texture = runner.fingerprint_package(second)
    assert changed_texture["fingerprint"] != first_result["fingerprint"]

    (tmp_path / "second/texture.png").write_bytes(b"texture-v1")
    changed_origin = _write_package(
        tmp_path / "third",
        robot_name="three",
        root_name="base3",
        child_name="door3",
        joint_name="hinge3",
        origin="1 0 0",
    )
    assert runner.fingerprint_package(changed_origin)["fingerprint"] != first_result["fingerprint"]


def test_package_fingerprint_is_not_evaluable_when_resource_is_missing(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    urdf = _write_package(
        tmp_path / "missing",
        robot_name="missing",
        root_name="base",
        child_name="door",
        joint_name="hinge",
    )
    (tmp_path / "missing/texture.png").unlink()

    result = runner.fingerprint_package(urdf)

    assert result["complete"] is False
    assert result["fingerprint"] is None
    assert result["missing_resources"] == ["texture.png"]


def test_package_fingerprint_preserves_material_texture_slots(tmp_path: Path) -> None:
    runner = _load_runner()
    first = _write_package(
        tmp_path / "first",
        robot_name="first",
        root_name="root",
        child_name="child",
        joint_name="joint",
    )
    second = _write_package(
        tmp_path / "second",
        robot_name="second",
        root_name="renamed_root",
        child_name="renamed_child",
        joint_name="renamed_joint",
    )
    for package in (first.parent, second.parent):
        (package / "material.mtl").write_text(
            "map_Kd diffuse.png\nmap_Bump normal.png\n",
            encoding="utf-8",
        )
    (first.parent / "diffuse.png").write_bytes(b"diffuse")
    (first.parent / "normal.png").write_bytes(b"normal")
    (second.parent / "diffuse.png").write_bytes(b"normal")
    (second.parent / "normal.png").write_bytes(b"diffuse")

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        != runner.fingerprint_package(second)["fingerprint"]
    )


def test_package_fingerprint_ignores_resource_file_renames(tmp_path: Path) -> None:
    runner = _load_runner()
    first = _write_package(
        tmp_path / "first",
        robot_name="first",
        root_name="root",
        child_name="child",
        joint_name="joint",
    )
    second_root = tmp_path / "second"
    second_root.mkdir()
    (second_root / "renamed.obj").write_text(
        "mtllib renamed.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )
    (second_root / "renamed.mtl").write_text(
        "map_Kd renamed.png\n", encoding="utf-8"
    )
    (second_root / "renamed.png").write_bytes(b"texture-v1")
    second = _write_urdf(
        second_root / "model.urdf",
        """
        <link name="renamed_root">
          <visual><origin xyz="0 0 0"/><geometry><mesh filename="renamed.obj"/></geometry></visual>
        </link>
        <link name="renamed_child"/>
        <joint name="renamed_joint" type="revolute">
          <parent link="renamed_root"/><child link="renamed_child"/>
          <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
        """,
        robot_name="second",
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        == runner.fingerprint_package(second)["fingerprint"]
    )


def test_package_fingerprint_tracks_direct_urdf_textures(tmp_path: Path) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        """
        <material name="paint"><texture filename="texture.png"/></material>
        <link name="root"><visual><geometry><box size="1 1 1"/></geometry><material name="paint"/></visual></link>
        """,
    )
    second = _write_urdf(
        tmp_path / "second/model.urdf",
        """
        <material name="renamed_paint"><texture filename="texture.png"/></material>
        <link name="renamed_root"><visual><geometry><box size="1 1 1"/></geometry><material name="renamed_paint"/></visual></link>
        """,
        robot_name="renamed",
    )
    first.parent.joinpath("texture.png").write_bytes(b"texture-v1")
    second.parent.joinpath("texture.png").write_bytes(b"texture-v1")

    first_result = runner.fingerprint_package(first)
    second_result = runner.fingerprint_package(second)
    assert first_result["complete"] is True
    assert first_result["resource_count"] == 1
    assert first_result["fingerprint"] == second_result["fingerprint"]

    second.parent.joinpath("texture.png").write_bytes(b"texture-v2")
    assert runner.fingerprint_package(second)["fingerprint"] != first_result["fingerprint"]

    second.parent.joinpath("texture.png").unlink()
    missing = runner.fingerprint_package(second)
    assert missing["complete"] is False
    assert missing["fingerprint"] is None
    assert missing["missing_resources"] == ["texture.png"]


def test_package_fingerprint_ignores_link_and_joint_xml_order(tmp_path: Path) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        """
        <link name="root"><inertial><mass value="1"/></inertial></link>
        <link name="middle"><inertial><mass value="2"/></inertial></link>
        <link name="leaf"><inertial><mass value="3"/></inertial></link>
        <joint name="moving" type="revolute">
          <parent link="root"/><child link="middle"/><axis xyz="0 0 1"/>
        </joint>
        <joint name="fixed" type="fixed">
          <parent link="middle"/><child link="leaf"/>
        </joint>
        """,
    )
    reordered = _write_urdf(
        tmp_path / "second/model.urdf",
        """
        <joint name="renamed_fixed" type="fixed">
          <child link="renamed_leaf"/><parent link="renamed_middle"/>
        </joint>
        <link name="renamed_leaf"><inertial><mass value="3.0"/></inertial></link>
        <joint name="renamed_moving" type="revolute">
          <axis xyz="0.0 0 1.00"/><child link="renamed_middle"/><parent link="renamed_root"/>
        </joint>
        <link name="renamed_root"><inertial><mass value="1.0"/></inertial></link>
        <link name="renamed_middle"><inertial><mass value="2.00"/></inertial></link>
        """,
        robot_name="renamed",
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        == runner.fingerprint_package(reordered)["fingerprint"]
    )


def test_package_fingerprint_normalizes_known_urdf_name_references(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        """
        <material name="paint"><color rgba="1 0 0 1"/></material>
        <link name="root"><visual><geometry><box size="1 1 1"/></geometry><material name="paint"/></visual></link>
        <link name="middle"/><link name="leaf"/>
        <joint name="driver" type="revolute"><parent link="root"/><child link="middle"/></joint>
        <joint name="follower" type="revolute"><parent link="middle"/><child link="leaf"/><mimic joint="driver" multiplier="1"/></joint>
        <transmission name="gearbox"><joint name="follower"><hardwareInterface>EffortJointInterface</hardwareInterface></joint></transmission>
        <gazebo reference="leaf"><mu1>1</mu1></gazebo>
        """,
    )
    renamed = _write_urdf(
        tmp_path / "renamed/model.urdf",
        """
        <material name="renamed_paint"><color rgba="1.0 0 0 1"/></material>
        <link name="renamed_root"><visual><geometry><box size="1 1 1"/></geometry><material name="renamed_paint"/></visual></link>
        <link name="renamed_middle"/><link name="renamed_leaf"/>
        <joint name="renamed_driver" type="revolute"><parent link="renamed_root"/><child link="renamed_middle"/></joint>
        <joint name="renamed_follower" type="revolute"><parent link="renamed_middle"/><child link="renamed_leaf"/><mimic joint="renamed_driver" multiplier="1.0"/></joint>
        <transmission name="renamed_gearbox"><joint name="renamed_follower"><hardwareInterface>EffortJointInterface</hardwareInterface></joint></transmission>
        <gazebo reference="renamed_leaf"><mu1>1.0</mu1></gazebo>
        """,
        robot_name="renamed",
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        == runner.fingerprint_package(renamed)["fingerprint"]
    )


def test_package_fingerprint_preserves_unknown_extension_child_order(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        """
        <link name="root"/>
        <plugin name="ordered"><stage value="1"/><stage value="2"/></plugin>
        """,
    )
    reordered = _write_urdf(
        tmp_path / "second/model.urdf",
        """
        <link name="root"/>
        <plugin name="ordered"><stage value="2"/><stage value="1"/></plugin>
        """,
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        != runner.fingerprint_package(reordered)["fingerprint"]
    )


def test_package_fingerprint_preserves_unknown_string_attributes(tmp_path: Path) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        '<link name="root"/><plugin name="001"/>',
    )
    second = _write_urdf(
        tmp_path / "second/model.urdf",
        '<link name="root"/><plugin name="1"/>',
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        != runner.fingerprint_package(second)["fingerprint"]
    )


def test_package_fingerprint_is_invariant_to_symmetric_sibling_renames(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    first = _write_urdf(
        tmp_path / "first/model.urdf",
        """
        <link name="root"/><link name="left"/><link name="right"/>
        <joint name="left_joint" type="fixed"><parent link="root"/><child link="left"/></joint>
        <joint name="right_joint" type="fixed"><parent link="root"/><child link="right"/></joint>
        <gazebo reference="left"><mu1>1</mu1></gazebo>
        """,
    )
    renamed = _write_urdf(
        tmp_path / "second/model.urdf",
        """
        <link name="renamed_root"/><link name="renamed_right"/><link name="renamed_left"/>
        <joint name="renamed_right_joint" type="fixed"><parent link="renamed_root"/><child link="renamed_right"/></joint>
        <joint name="renamed_left_joint" type="fixed"><parent link="renamed_root"/><child link="renamed_left"/></joint>
        <gazebo reference="renamed_left"><mu1>1.0</mu1></gazebo>
        """,
        robot_name="renamed",
    )

    assert (
        runner.fingerprint_package(first)["fingerprint"]
        == runner.fingerprint_package(renamed)["fingerprint"]
    )


def test_aggregation_preserves_intent_denominator_and_evaluability() -> None:
    runner = _load_runner()
    records = [
        {
            "asset_id": "data/a/s/1",
            "raw_category": "a",
            "parse_success": True,
            "link_count": 2,
            "non_fixed_joint_count": 2,
            "valid_tree": True,
            "topology_hash": "topology-a",
            "fingerprint_complete": True,
            "package_fingerprint": "duplicate",
        },
        {
            "asset_id": "data/b/s/2",
            "raw_category": "b",
            "parse_success": True,
            "link_count": 4,
            "non_fixed_joint_count": 0,
            "valid_tree": True,
            "topology_hash": "topology-b",
            "fingerprint_complete": True,
            "package_fingerprint": "duplicate",
        },
        {
            "asset_id": "data/a/s/3",
            "raw_category": "a",
            "parse_success": False,
            "link_count": None,
            "non_fixed_joint_count": None,
            "valid_tree": False,
            "topology_hash": None,
            "fingerprint_complete": False,
            "package_fingerprint": None,
        },
    ]

    summary = runner.aggregate_records(
        records,
        release_asset_count=5,
        release_category_count=3,
    )

    assert summary["cohort"] == {
        "N_release": 5,
        "N_eval": 3,
        "N_parse": 2,
        "release_raw_categories": 3,
        "eval_raw_categories": 2,
        "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
    }
    assert summary["links_per_asset"] == {
        "denominator": 2,
        "mean": 3.0,
        "median": 3.0,
        "p90_nearest_rank": 4,
    }
    assert summary["movable_joints_per_asset"] == {
        "denominator": 2,
        "mean": 1.0,
        "median": 1.0,
        "p90_nearest_rank": 2,
    }
    assert summary["multi_joint_assets"] == {
        "numerator": 1,
        "denominator": 3,
        "rate": 1 / 3,
        "valid_only_denominator": 2,
        "valid_only_rate": 1 / 2,
    }
    assert summary["unique_topologies"] == {
        "unique": 2,
        "denominator": 2,
        "rate": 1.0,
        "coverage_denominator": 3,
        "coverage_rate": 2 / 3,
    }
    assert summary["exact_duplicate_rate"] == {
        "duplicate_excess": 1,
        "unique": 1,
        "denominator": 2,
        "rate": 1 / 2,
        "assets_in_duplicate_clusters": 2,
        "assets_in_duplicate_clusters_rate": 1.0,
        "duplicate_cluster_count": 1,
        "max_cluster_size": 2,
        "coverage_denominator": 3,
        "coverage_rate": 2 / 3,
    }
    assert summary["category_macro"] == {
        "category_count": 2,
        "multi_joint_assets_rate": 0.25,
        "unique_topologies_evaluable_categories": 2,
        "unique_topologies_rate": 1.0,
        "exact_duplicate_evaluable_categories": 2,
        "exact_duplicate_rate": 0.0,
    }
    assert summary["category_breakdown"]["a"]["N_eval"] == 2
    assert summary["category_breakdown"]["a"]["N_parse"] == 1
    assert summary["category_breakdown"]["a"]["multi_joint_assets"]["rate"] == 0.5
    assert summary["category_breakdown"]["b"]["multi_joint_assets"]["rate"] == 0.0


def test_report_renders_not_evaluable_distributions() -> None:
    runner = _load_runner()
    summary = runner.aggregate_records(
        [
            {
                "asset_id": "data/a/s/1",
                "raw_category": "a",
                "parse_success": False,
                "link_count": None,
                "non_fixed_joint_count": None,
                "joint_type_counts": None,
                "valid_tree": False,
                "topology_hash": None,
                "fingerprint_complete": False,
                "package_fingerprint": None,
            }
        ],
        release_asset_count=1,
        release_category_count=1,
    )

    report = runner._report(
        summary,
        {
            "release_manifest_sha256": "a" * 64,
            "seed": "fixture",
        },
    )

    assert "N/E / N/E / N/E (n=0)" in report
    assert "0 / 1 (0.00%)" in report
    assert "all declared XML joints except literal `fixed`" in report
    assert "valid rooted trees only" in report


def test_evaluate_asset_retains_record_when_primary_urdf_read_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    artiverse = tmp_path / "artiverse"
    identity = {
        "asset_id": "data/a/source/model",
        "manifest_root": "data/a/source/model",
        "raw_category": "a",
        "source": "source",
        "model_id": "model",
        "chunk_archive": "fixture.tar.gz",
        "selection_hash": "a" * 64,
        "selection_rank": 1,
    }
    urdf = _write_urdf(
        artiverse / identity["manifest_root"] / "urdf_w_collider/model.urdf",
        '<link name="root"/>',
    )
    real_sha256_file = runner.sha256_file

    def fail_for_urdf(path: Path) -> str:
        if path == urdf:
            raise OSError("fixture read failure")
        return real_sha256_file(path)

    monkeypatch.setattr(runner, "sha256_file", fail_for_urdf)

    record = runner.evaluate_asset(artiverse, identity)

    assert record["asset_id"] == identity["asset_id"]
    assert record["status"] == "URDF_READ_FAILED"
    assert record["parse_success"] is False
    assert record["error"] == "OSError: fixture read failure"


def test_output_lock_is_exclusive(tmp_path: Path) -> None:
    runner = _load_runner()
    output = tmp_path / "result"

    with runner._output_lock(output):
        with pytest.raises(RuntimeError, match="output is locked"):
            with runner._output_lock(output):
                pass


def test_publish_exchanges_existing_output_without_backup_residue(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    output = tmp_path / "result"
    first_staging = tmp_path / ".result.staging.first"
    first_staging.mkdir()
    (first_staging / "version.txt").write_text("old", encoding="utf-8")
    runner._publish_staged_output(first_staging, output)
    first_target = output.resolve()

    second_staging = tmp_path / ".result.staging.second"
    second_staging.mkdir()
    (second_staging / "version.txt").write_text("new", encoding="utf-8")

    runner._publish_staged_output(second_staging, output)

    assert output.is_symlink()
    assert (output / "version.txt").read_text(encoding="utf-8") == "new"
    assert not first_target.exists()
    assert not first_staging.exists()
    assert not second_staging.exists()
    assert len(list((tmp_path / ".result.runs").iterdir())) == 1


def test_unexpected_asset_error_becomes_one_fail_closed_record(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    identity = {
        "asset_id": "data/a/source/model",
        "manifest_root": "data/a/source/model",
        "raw_category": "a",
    }
    monkeypatch.setattr(
        runner,
        "evaluate_asset",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("fixture")),
    )

    record = runner._evaluate_asset_fail_closed(Path("/unused"), identity)

    assert record["asset_id"] == identity["asset_id"]
    assert record["status"] == "ASSET_EVALUATION_FAILED"
    assert record["error"] == "RuntimeError: fixture"


def test_failed_staged_run_preserves_previous_complete_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    artiverse = tmp_path / "artiverse"
    output = tmp_path / "output"
    output.mkdir()
    (output / "artifact_manifest.json").write_text(
        '{"previous": true}\n', encoding="utf-8"
    )
    chunks = artiverse / "dataset_chunks"
    chunks.mkdir(parents=True)
    root = "data/a/source/model"
    manifest = {
        "format": "artiverse-data-tar-gz-chunks-v1",
        "created_utc": "2026-01-01T00:00:00Z",
        "data_dir": "data",
        "chunk_count": 1,
        "model_count": 1,
        "file_count": 1,
        "input_bytes": 1,
        "chunks": [
            {
                "archive": "fixture.tar.gz",
                "sha256": "1" * 64,
                "archive_bytes": 1,
                "model_count": 1,
                "file_count": 1,
                "input_bytes": 1,
                "roots": [root],
            }
        ],
    }
    (chunks / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    _write_urdf(
        artiverse / root / "urdf_w_collider/model.urdf",
        '<link name="root"/>',
    )
    protocol = tmp_path / "protocol.md"
    protocol.write_text("fixture", encoding="utf-8")
    monkeypatch.setattr(
        runner,
        "aggregate_records",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("fixture stop")),
    )
    args = Namespace(
        artiverse_root=artiverse,
        output=output,
        protocol=protocol,
        sample_size=1,
        seed="fixture",
        workers=1,
    )

    with pytest.raises(RuntimeError, match="fixture stop"):
        runner.run(args)

    assert (output / "artifact_manifest.json").read_text(encoding="utf-8") == (
        '{"previous": true}\n'
    )
    assert not (output / "manifest.json").exists()
    assert list(tmp_path.glob(".output.staging.*")) == []


def test_cli_keeps_missing_and_malformed_assets_in_the_frozen_cohort(
    tmp_path: Path,
) -> None:
    artiverse = tmp_path / "artiverse"
    output = tmp_path / "output"
    roots = ["data/a/source/good", "data/b/source/missing", "data/a/source/bad"]
    chunks = artiverse / "dataset_chunks"
    chunks.mkdir(parents=True)
    manifest = {
        "format": "artiverse-data-tar-gz-chunks-v1",
        "created_utc": "2026-01-01T00:00:00Z",
        "data_dir": "data",
        "chunk_count": 1,
        "model_count": 3,
        "file_count": 2,
        "input_bytes": 10,
        "chunks": [
            {
                "archive": "fixture.tar.gz",
                "sha256": "1" * 64,
                "archive_bytes": 10,
                "model_count": 3,
                "file_count": 2,
                "input_bytes": 10,
                "roots": roots,
            }
        ],
    }
    (chunks / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    _write_urdf(
        artiverse / roots[0] / "urdf_w_collider/model.urdf",
        """
        <link name="root"/><link name="child"/>
        <joint name="joint" type="revolute"><parent link="root"/><child link="child"/></joint>
        """,
    )
    (artiverse / roots[1] / "urdf_w_collider").mkdir(parents=True)
    bad_package = artiverse / roots[2] / "urdf_w_collider"
    bad_package.mkdir(parents=True)
    (bad_package / "model.urdf").write_text("<robot>", encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--artiverse-root",
            str(artiverse),
            "--output",
            str(output),
            "--sample-size",
            "3",
            "--seed",
            "fixture",
        ],
        cwd=REPO,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    frozen = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert len(frozen["assets"]) == len(records) == 3
    assert [row["asset_id"] for row in records] == [
        row["asset_id"] for row in frozen["assets"]
    ]
    assert summary["cohort"]["N_release"] == 3
    assert summary["cohort"]["N_eval"] == 3
    assert summary["cohort"]["N_parse"] == 1
    assert {row["status"] for row in records} == {
        "EVALUATED",
        "MISSING_PRIMARY_URDF",
        "URDF_PARSE_FAILED",
    }
    assert (output / "report.md").is_file()
    assert (output / "artifact_manifest.json").is_file()
