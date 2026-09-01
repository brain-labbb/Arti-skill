from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_artiverse_table2_pva.py"


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "run_artiverse_table2_pva_test", RUNNER
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_urdf(path: Path, *, include_universal: bool = True) -> Path:
    universal = (
        '<joint name="slider_to_wrist" type="universal">'
        '<parent link="slider"/><child link="wrist"/>'
        "</joint>"
        if include_universal
        else ""
    )
    path.write_text(
        f"""<robot name="fixture">
  <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="arm"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="slider"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="wrist"/>
  <link name="fixed_link"/>
  <joint name="base_to_fixed" type="fixed">
    <parent link="base"/><child link="fixed_link"/>
  </joint>
  <joint name="base_to_arm" type="revolute">
    <parent link="base"/><child link="arm"/>
  </joint>
  <joint name="arm_to_slider" type="prismatic">
    <parent link="arm"/><child link="slider"/>
  </joint>
  {universal}
</robot>
""",
        encoding="utf-8",
    )
    return path


def test_structural_proxy_counts_renderable_parts_and_dof_buckets(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    urdf = _write_urdf(tmp_path / "model.urdf")
    row = {
        "ordinal": 0,
        "asset_id": "fixture/seed_0",
        "category": "Fixture",
        "primary_urdf_path": str(urdf),
        "joint_count": 3,
    }

    record = runner.evaluate_xml(
        row,
        roster_base=tmp_path,
        annotations=None,
        mode="structural-proxy",
        verify_hash=False,
    )

    assert record["status"] == "EVALUATED"
    assert record["link_count"] == 5
    assert record["renderable_link_count"] == 3
    assert record["fixed_joint_count"] == 1
    assert record["representation_movable_joint_count"] == 3
    assert record["functional_part_count"] == 3
    assert record["articulated_part_count"] == 3
    assert record["logical_joint_count"] == 3
    assert record["joint_1dof_count"] == 2
    assert record["joint_2dof_count"] == 1
    assert record["joint_3dof_count"] == 0
    assert record["joint_type_counts"] == {
        "prismatic": 1,
        "revolute": 1,
        "universal": 1,
    }
    assert record["roster_joint_count_match"] is True


def test_semantic_joint_components_are_grouped_into_logical_dof_types() -> None:
    runner = load_runner()

    joint_types = runner.semantic_joint_types(
        {
            "joints": [
                {"pid": "slider", "type": "revolute"},
                {"pid": "slider", "type": "prismatic"},
                {"pid": "hinge", "type": "revolute"},
                {"pid": "hinge", "type": "continuous"},
                {"type": "planar"},
            ]
        }
    )

    assert joint_types == ["cylindrical", "universal", "planar"]
    assert runner.dof_bucket("cylindrical") == (2, "2")
    assert runner.dof_bucket("planar") == (3, "3")

    # Artiverse exports can use a free/base placeholder alongside the actual
    # motion record. The placeholder must not turn a prismatic joint into a
    # spurious 3-DoF joint; a free-only group remains diagnostic 3-DoF.
    assert runner.semantic_joint_types(
        {
            "joints": [
                {"pid": "base", "type": "free"},
                {"pid": "base", "type": "prismatic"},
                {"pid": "free_only", "type": "free"},
                {"pid": "fixed_only", "type": "fixed"},
            ]
        }
    ) == ["prismatic", "free"]


def test_semantic_run_writes_table2_artifacts_and_keeps_full_run_denominator(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    first = _write_urdf(tmp_path / "first.urdf", include_universal=False)
    second = _write_urdf(tmp_path / "second.urdf", include_universal=False)
    roster = tmp_path / "roster.jsonl"
    roster_rows = [
        {
            "ordinal": 0,
            "asset_id": "fixture/seed_0",
            "category": "Fixture-A",
            "primary_urdf_path": str(first),
            "joint_count": 2,
        },
        {
            "ordinal": 1,
            "asset_id": "fixture/seed_1",
            "category": "Fixture-B",
            "primary_urdf_path": str(second),
            "joint_count": 2,
        },
    ]
    roster.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in roster_rows),
        encoding="utf-8",
    )
    annotations = tmp_path / "annotations.json"
    annotations.write_text(
        json.dumps(
            {
                "schema_version": runner.ANNOTATION_SCHEMA_VERSION,
                "assets": [
                    {
                        "asset_id": "fixture/seed_0",
                        "functional_parts": ["base", "arm"],
                        "articulated_parts": ["arm"],
                        "joints": [{"pid": "j0", "type": "revolute"}],
                    },
                    {
                        "asset_id": "fixture/seed_1",
                        "functional_parts": ["base", "arm", "slider"],
                        "articulated_parts": ["arm", "slider"],
                        "joints": [
                            {"pid": "j1", "type": "revolute"},
                            {"pid": "j1", "type": "prismatic"},
                            {"pid": "j2", "type": "planar"},
                        ],
                    },
                ],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    output = tmp_path / "run"
    args = runner.parse_args(
        [
            "--roster",
            str(roster),
            "--annotations",
            str(annotations),
            "--mode",
            "semantic",
            "--output",
            str(output),
            "--workers",
            "2",
        ]
    )

    summary = runner.run(args)

    assert summary["classification"] == "SEMANTIC_ANNOTATION"
    assert summary["n_eval"] == summary["evaluated_assets"] == 2
    assert summary["failed_assets"] == 0
    assert summary["metric_coverage"] == pytest.approx(1.0)
    assert summary["table2"] == {
        "dataset": "Ours / PV-A",
        "mode": "SEMANTIC",
        "n_obj": 2,
        "category_total": 2,
        "category_avg_objects": 1.0,
        "functional_parts_total": 5,
        "functional_parts_avg": 2.5,
        "articulated_parts_total": 3,
        "articulated_parts_avg": 1.5,
        "joints_1dof": 1,
        "joints_2dof": 1,
    }
    assert summary["diagnostics"]["joints_3dof"] == 1
    assert summary["diagnostics"]["logical_joints_total"] == 3

    for name in (
        "asset_records.jsonl",
        "run_manifest.json",
        "summary.json",
        "table2.md",
        "table2.csv",
    ):
        assert (output / name).is_file(), name
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert [record["ordinal"] for record in records] == [0, 1]
    assert "STRUCTURAL_PROXY" not in (output / "table2.md").read_text(encoding="utf-8")
    manifest = json.loads((output / "run_manifest.json").read_text(encoding="utf-8"))
    for name, binding in manifest["artifacts"].items():
        artifact = output / name
        assert binding["bytes"] == artifact.stat().st_size
        assert binding["sha256"] == runner.sha256_file(artifact)


def test_summary_omits_part_averages_when_any_asset_fails() -> None:
    runner = load_runner()
    aggregate = runner._empty_aggregate()
    runner.update_aggregate(
        aggregate,
        {
            "status": "EVALUATED",
            "category": "Fixture",
            "functional_part_count": 2,
            "articulated_part_count": 1,
            "logical_joint_count": 1,
            "joint_1dof_count": 1,
            "joint_2dof_count": 0,
            "joint_3dof_count": 0,
            "joint_other_count": 0,
            "representation_movable_joint_count": 1,
            "joint_type_counts": {"revolute": 1},
            "roster_joint_count_match": True,
        },
    )
    runner.update_aggregate(
        aggregate,
        {
            "status": "FAILED",
            "category": "Fixture",
        },
    )
    roster = runner.RosterInput(
        manifest_path=Path("/tmp/fixture-manifest.jsonl"),
        rows_path=Path("/tmp/fixture-manifest.jsonl"),
        manifest={},
        manifest_sha256="manifest",
        rows_sha256="rows",
        declared_n=None,
        declared_categories=None,
        declared_joints=None,
    )

    summary = runner.make_summary(
        aggregate,
        mode="structural-proxy",
        roster=roster,
        annotation_store=None,
        limit=2,
        started_at=0.0,
        elapsed_s=0.1,
    )

    assert summary["metric_coverage"] == pytest.approx(0.5)
    assert summary["table2"]["functional_parts_total"] == 2
    assert summary["table2"]["functional_parts_avg"] is None
    assert summary["table2"]["articulated_parts_avg"] is None
    assert summary["diagnostics"]["logical_joints_avg"] is None


def test_protocol_rejects_wrong_table_identity(tmp_path: Path) -> None:
    runner = load_runner()
    protocol = tmp_path / "protocol.json"
    protocol.write_text(
        json.dumps({"protocol_id": "wrong", "protocol_version": 1}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="protocol id mismatch"):
        runner.load_protocol(protocol)
