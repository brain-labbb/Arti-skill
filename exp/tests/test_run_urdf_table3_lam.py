from __future__ import annotations

import csv
import importlib.util
import math
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table3_lam.py"


def load_runner():
    assert RUNNER.is_file(), "Table 3 LAM runner has not been implemented"
    spec = importlib.util.spec_from_file_location("urdf_table3_lam", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_urdf(tmp_path: Path, body: str, name: str = "model.urdf") -> Path:
    path = tmp_path / name
    path.write_text(f'<robot name="fixture">{body}</robot>\n', encoding="utf-8")
    return path


def branching_revolute_urdf() -> str:
    return """
<link name="base"/>
<link name="arm"/>
<link name="tip"/>
<link name="fixed_branch"/>
<joint name="hinge" type="revolute">
  <parent link="base"/><child link="arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1" upper="1" effort="1" velocity="1"/>
</joint>
<joint name="tip_mount" type="fixed">
  <parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/>
</joint>
<joint name="fixed_mount" type="fixed">
  <parent link="base"/><child link="fixed_branch"/><origin xyz="0 2 0"/>
</joint>
"""


def test_formal_contract_freezes_n_seed_and_21_states() -> None:
    runner = load_runner()
    args = runner.parse_args([])
    runner.validate_contract(args)

    assert args.mode == "formal"
    assert args.n == 800
    assert args.seed == 20260813
    assert args.samples == 21
    assert args.workers == 4

    with pytest.raises(ValueError, match="formal"):
        runner.validate_contract(runner.parse_args(["--n", "799"]))
    with pytest.raises(ValueError, match="formal"):
        runner.validate_contract(runner.parse_args(["--samples", "20"]))


def test_release_loader_retains_all_tiers_and_duplicate_public_ids(tmp_path: Path) -> None:
    runner = load_runner()
    source = tmp_path / "release"
    rows = [
        ("same", "door", "viable", "objects/door/a"),
        ("same", "door", "broken", "imperfect/door/a"),
        ("unique", "lamp", "loads_only", "objects/lamp/b"),
    ]
    for _public_id, _category, _tier, relative in rows:
        package = source / relative
        package.mkdir(parents=True)
        write_urdf(package, '<link name="base"/>', name="generated.urdf")
    manifest = tmp_path / "manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["object_release_id", "category", "tier", "rel_path", "n_movable"],
        )
        writer.writeheader()
        for public_id, category, tier, relative in rows:
            writer.writerow({
                "object_release_id": public_id,
                "category": category,
                "tier": tier,
                "rel_path": relative,
                "n_movable": "0",
            })

    loaded = runner.load_release_rows(manifest, source)

    assert len(loaded) == 3
    assert {row["tier"] for row in loaded} == {"viable", "loads_only", "broken"}
    assert len({row["asset_key"] for row in loaded}) == 3
    assert sum(row["object_release_id"] == "same" for row in loaded) == 2


def test_selection_is_stable_and_independent_of_input_order() -> None:
    runner = load_runner()
    pool = [{"asset_key": f"asset_{index}"} for index in range(10)]

    expected = ["asset_2", "asset_4", "asset_7", "asset_0"]
    assert [row["asset_key"] for row in runner.select_assets(pool, 4, 20260813)] == expected
    assert [
        row["asset_key"] for row in runner.select_assets(reversed(pool), 4, 20260813)
    ] == expected
    with pytest.raises(ValueError):
        runner.select_assets(pool, 11, 20260813)


def test_revolute_sweep_uses_21_states_and_preserves_other_branch(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(tmp_path, branching_revolute_urdf())

    record = runner.evaluate_urdf(urdf, asset_key="viable:objects/fixture", samples=21)

    assert record["declared_joint_count"] == 1
    assert record["status"] == "completed"
    assert record["strict_kinematic_pass"] is True
    joint = record["joints"][0]
    assert joint["joint_name"] == "hinge"
    assert joint["sample_count_expected"] == 21
    assert joint["sample_count_executed"] == 21
    assert joint["sample_values"][0] == pytest.approx(-1.0)
    assert joint["sample_values"][-1] == pytest.approx(1.0)
    assert joint["valid_range_pass"] is True
    assert joint["joint_sweep_success"] is True
    assert joint["non_degenerate_motion_pass"] is True
    assert joint["subtree_consistency_pass"] is True
    assert joint["fk_roundtrip_pass"] is True
    assert joint["joint_level_pass"] is True
    assert joint["max_non_descendant_translation"] <= 1e-12
    assert joint["max_non_descendant_rotation_rad"] <= 1e-12
    assert joint["fk_roundtrip_max_translation"] <= 1e-12
    assert joint["fk_roundtrip_max_rotation_rad"] <= 1e-12


def test_continuous_joint_uses_frozen_minus_pi_to_pi_interval(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="base"/><link name="wheel"/>
<joint name="spin" type="continuous">
  <parent link="base"/><child link="wheel"/><axis xyz="0 1 0"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="viable:objects/wheel", samples=21)
    joint = record["joints"][0]

    assert joint["range_source"] == "frozen_continuous_interval"
    assert joint["sample_values"][0] == pytest.approx(-math.pi)
    assert joint["sample_values"][-1] == pytest.approx(math.pi)
    assert joint["joint_level_pass"] is True


def test_empty_bounded_range_is_retained_as_joint_failure(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="base"/><link name="door"/>
<joint name="hinge" type="revolute">
  <parent link="base"/><child link="door"/><axis xyz="0 0 1"/>
  <limit lower="1" upper="1" effort="1" velocity="1"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="broken:imperfect/door", samples=21)
    joint = record["joints"][0]

    assert record["declared_joint_count"] == 1
    assert record["strict_kinematic_pass"] is False
    assert joint["valid_range_pass"] is False
    assert joint["sample_count_executed"] == 0
    assert joint["joint_level_pass"] is False


def test_failure_reason_names_the_metric_that_actually_failed(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="base"/><link name="slider"/>
<joint name="slide" type="prismatic">
  <parent link="base"/><child link="slider"/><axis xyz="1 0 0"/>
  <limit lower="0" upper="0.02" effort="1" velocity="1"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="viable:objects/slider", samples=21)
    joint = record["joints"][0]

    assert joint["valid_range_pass"] is True
    assert joint["joint_sweep_success"] is True
    assert joint["non_degenerate_motion_pass"] is False
    assert joint["issue"] == "failed checks: non_degenerate_motion"


def test_prismatic_motion_uses_q0_geometry_bbox_scale(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="base">
  <visual><geometry><box size="2 1 1"/></geometry></visual>
</link>
<link name="slider">
  <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
</link>
<joint name="slide" type="prismatic">
  <parent link="base"/><child link="slider"/><axis xyz="1 0 0"/>
  <limit lower="0" upper="0.02" effort="1" velocity="1"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="viable:objects/slider", samples=21)
    joint = record["joints"][0]

    assert record["kinematic_scale"] == pytest.approx(math.sqrt(6.0))
    assert joint["max_descendant_normalized_translation"] == pytest.approx(
        0.02 / math.sqrt(6.0)
    )
    assert joint["non_degenerate_motion_pass"] is True
    assert joint["joint_level_pass"] is True


def test_invalid_tree_retains_declared_joint_denominator(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="a"/><link name="b"/><link name="orphan"/>
<joint name="move" type="prismatic">
  <parent link="a"/><child link="b"/><axis xyz="1 0 0"/>
  <limit lower="0" upper="1" effort="1" velocity="1"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="broken:imperfect/tree", samples=21)

    assert record["declared_joint_count"] == 1
    assert record["tree_valid"] is False
    assert record["strict_kinematic_pass"] is False
    assert len(record["joints"]) == 1
    assert record["joints"][0]["joint_level_pass"] is False


def test_initial_fk_failure_preserves_each_joint_valid_range_result(tmp_path: Path) -> None:
    runner = load_runner()
    urdf = write_urdf(
        tmp_path,
        """
<link name="base"/><link name="floating_link"/><link name="door"/>
<joint name="unsupported" type="floating">
  <parent link="base"/><child link="floating_link"/>
</joint>
<joint name="hinge" type="revolute">
  <parent link="floating_link"/><child link="door"/><axis xyz="0 0 1"/>
  <limit lower="-1" upper="1" effort="1" velocity="1"/>
</joint>
""",
    )

    record = runner.evaluate_urdf(urdf, asset_key="broken:imperfect/floating", samples=21)
    by_name = {joint["joint_name"]: joint for joint in record["joints"]}

    assert record["status"] == "error"
    assert record["declared_joint_count"] == 2
    assert by_name["unsupported"]["valid_range_pass"] is False
    assert by_name["hinge"]["valid_range_pass"] is True
    assert by_name["hinge"]["joint_sweep_success"] is False
    assert by_name["hinge"]["joint_level_pass"] is False


def test_aggregation_is_fail_closed_for_asset_and_joint_denominators(tmp_path: Path) -> None:
    runner = load_runner()
    good = runner.evaluate_urdf(
        write_urdf(tmp_path, branching_revolute_urdf()),
        asset_key="viable:objects/good",
        samples=21,
    )
    failed = runner.failed_record(
        asset_key="broken:imperfect/bad",
        declared_joint_count=2,
        reason="worker timeout",
        status="timeout",
    )
    good["category"] = "category_a"
    failed["category"] = "category_b"

    summary = runner.aggregate_records([good, failed], expected_n=2)

    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 3
    assert summary["metrics"]["valid_range"]["passed"] == 1
    assert summary["metrics"]["valid_range"]["denominator"] == 3
    assert summary["metrics"]["joint_level_pass"]["passed"] == 1
    assert summary["metrics"]["joint_level_pass"]["denominator"] == 3
    assert summary["metrics"]["strict_kinematic_pass"]["passed"] == 1
    assert summary["metrics"]["strict_kinematic_pass"]["denominator"] == 2
    assert summary["status_counts"] == {"completed": 1, "timeout": 1}
    assert summary["category_macro"]["category_count"] == 2
    assert summary["category_macro"]["metrics"]["valid_range"]["rate"] == pytest.approx(0.5)
    assert summary["category_macro"]["metrics"]["joint_level_pass"]["rate"] == pytest.approx(0.5)
    assert summary["category_macro"]["metrics"]["strict_kinematic_pass"]["rate"] == pytest.approx(0.5)
