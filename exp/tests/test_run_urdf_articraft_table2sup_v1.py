from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_articraft_table2sup_v1.py"


def load_runner():
    assert RUNNER.is_file(), "Articraft-10K Table 2 supplementary runner has not been implemented"
    spec = importlib.util.spec_from_file_location("urdf_table2sup_articraft10k", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PASS_URDF = """<?xml version="1.0"?>
<robot name="fixture_pass">
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

NOCOLL_URDF = """<?xml version="1.0"?>
<robot name="fixture_nocoll">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>
  <link name="wheel">
    <visual><geometry><cylinder radius="0.1" length="0.05"/></geometry></visual>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base"/><child link="wheel"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="5"/>
  </joint>
</robot>
"""


def write_fixture(tmp_path: Path) -> dict[str, Path]:
    module = load_runner()
    dataset_root = tmp_path / "Articraft-10K"
    release_root = dataset_root / "released_urdf"
    category_root = tmp_path / "official" / "records"
    assets = [
        ("rec_fixture_nocoll", "fixture_category_b", NOCOLL_URDF, 1),
        ("rec_fixture_pass", "fixture_category_a", PASS_URDF, 1),
    ]
    cohort_records = []
    table3_rows = []
    for index, (asset_id, category, urdf_text, declared) in enumerate(assets):
        package = release_root / asset_id
        package.mkdir(parents=True)
        urdf_path = package / "model.urdf"
        urdf_path.write_text(urdf_text, encoding="utf-8")
        (category_root / asset_id).mkdir(parents=True)
        (category_root / asset_id / "record.json").write_text(
            json.dumps({"record_id": asset_id, "category_slug": category}),
            encoding="utf-8",
        )
        binding = module._package_binding(package)
        cohort_records.append(
            {
                "asset_id": asset_id,
                "model_urdf_sha256": module.sha256_file(urdf_path),
                "package": str(package),
                "package_binding": binding,
                "selection_index": index,
            }
        )
        table3_rows.append(
            {
                "asset_key": asset_id,
                "asset_id": asset_id,
                "declared_joint_count": declared,
                "urdf_sha256": module.sha256_file(urdf_path),
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
                "selection_index": index,
            }
        )
    cohort_manifest = {
        "classification": "FORMAL",
        "dataset": "Articraft-10K",
        "source": {
            "repo_id": "fixture/repo",
            "revision": "fixture_revision",
            "release_asset_ids": [asset_id for asset_id, _, _, _ in assets],
            "release_asset_ids_sha256": module.canonical_sha256(
                [asset_id for asset_id, _, _, _ in assets]
            ),
        },
        "selection": {
            "algorithm": "random.Random(seed).sample(sorted(unique_asset_ids), n)",
            "seed": 20260813,
            "selected_asset_ids_sha256": module.canonical_sha256(
                [asset_id for asset_id, _, _, _ in assets]
            ),
        },
        "records": cohort_records,
    }
    cohort_manifest["manifest_content_sha256"] = module.canonical_sha256(
        {key: value for key, value in cohort_manifest.items() if key != "manifest_content_sha256"}
    )
    cohort_path = tmp_path / "cohort_manifest.json"
    cohort_path.write_text(json.dumps(cohort_manifest, indent=2), encoding="utf-8")
    table3_path = tmp_path / "table3_records.jsonl"
    table3_path.write_text(
        "\n".join(json.dumps(row) for row in table3_rows) + "\n", encoding="utf-8"
    )
    return {
        "module": module,
        "dataset_root": dataset_root,
        "cohort_path": cohort_path,
        "category_root": category_root,
        "table3_path": table3_path,
        "release_root": release_root,
    }


def test_formal_contract_freezes_canonical_paths() -> None:
    module = load_runner()
    args = module.parse_args(["--mode", "formal", "--limit", "5"])
    with pytest.raises(ValueError):
        module.validate_contract(args)
    args = module.parse_args(["--mode", "formal", "--workers", "8"])
    with pytest.raises(ValueError):
        module.validate_contract(args)
    args = module.parse_args(["--mode", "formal"])
    module.validate_contract(args)


def test_loader_preserves_exact_order_and_binds_categories(tmp_path: Path) -> None:
    fixture = write_fixture(tmp_path)
    module = fixture["module"]
    loaded = module.load_cohort(
        fixture["dataset_root"],
        fixture["cohort_path"],
        fixture["category_root"],
        fixture["table3_path"],
        formal=False,
    )
    assert [row["asset_key"] for row in loaded["assets"]] == [
        "rec_fixture_nocoll",
        "rec_fixture_pass",
    ]
    assert [row["category"] for row in loaded["assets"]] == [
        "fixture_category_b",
        "fixture_category_a",
    ]
    assert loaded["eval_category_count"] == 2
    assert all(row["expected_movable_joint_count"] == 1 for row in loaded["assets"])


def test_loader_rejects_package_drift(tmp_path: Path) -> None:
    fixture = write_fixture(tmp_path)
    module = fixture["module"]
    drift = fixture["release_root"] / "rec_fixture_pass" / "extra.txt"
    drift.write_text("drift", encoding="utf-8")
    with pytest.raises(ValueError, match="package binding mismatch"):
        module.load_cohort(
            fixture["dataset_root"],
            fixture["cohort_path"],
            fixture["category_root"],
            fixture["table3_path"],
            formal=False,
        )


def test_aggregate_keeps_fail_closed_denominators() -> None:
    module = load_runner()
    completed_pass = {
        "asset_key": "a",
        "category": "cat",
        "status": "completed",
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "asset_pass": True,
                "visual_bearing_links_declared": 2,
                "covered_visual_bearing_links": 2,
                "link_extraction_complete": True,
            },
            "joint_limit_portability": {
                "joints_intended": 2,
                "joints_extracted": 2,
                "joints_passed": 2,
                "extraction_complete": True,
                "joint_records": [
                    {"joint_type": "revolute", "limit_portability_pass": True},
                    {"joint_type": "revolute", "limit_portability_pass": True},
                ],
            },
            "joint_dynamics_coverage": {
                "joints_intended": 2,
                "joints_covered": 1,
                "joint_records": [
                    {"joint_type": "revolute", "covered": True},
                    {"joint_type": "revolute", "covered": False},
                ],
            },
            "placeholder_mass_incidence": {
                "dynamic_links": 3,
                "complete_inertial_links": 2,
                "complete_inertial_coverage_denominator": 3,
            },
        },
    }
    error_record = {
        "asset_key": "b",
        "category": "cat",
        "status": "error",
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
            },
            "joint_limit_portability": {
                "joints_intended": 3,
                "joints_extracted": 0,
                "joints_passed": 0,
                "extraction_complete": False,
                "joint_records": [],
            },
            "joint_dynamics_coverage": {
                "joints_intended": 3,
                "joints_covered": 0,
                "joint_records": [],
            },
            "placeholder_mass_incidence": {
                "dynamic_links": 0,
                "complete_inertial_links": 0,
                "complete_inertial_coverage_denominator": 0,
            },
        },
    }
    summary = module.aggregate_records([completed_pass, error_record], 2)
    metrics = summary["metrics"]
    assert summary["n_eval"] == 2
    # Failed asset stays in every denominator (fail closed).
    assert metrics["visual_bearing_collision_coverage"]["passed"] == 1
    assert metrics["visual_bearing_collision_coverage"]["denominator"] == 2
    assert metrics["visual_bearing_collision_coverage"]["link_micro_covered"] == 2
    assert metrics["visual_bearing_collision_coverage"]["link_micro_denominator"] == 2
    assert metrics["visual_bearing_collision_coverage"]["status"] == "PARTIAL"
    assert summary["j_eval"] == 5
    assert metrics["joint_limit_portability"] == {
        "passed": 2,
        "denominator": 5,
        "rate": 0.4,
        "joints_extracted": 2,
        "extraction_complete_assets": 1,
    }
    assert metrics["joint_dynamics_coverage"]["passed"] == 1
    assert metrics["joint_dynamics_coverage"]["denominator"] == 5
    # Registry frozen empty -> N/E, coverage still reported.
    placeholder = metrics["placeholder_mass_incidence"]
    assert placeholder["status"] == "N/E"
    assert placeholder["rate"] is None
    assert placeholder["complete_inertial_coverage"]["complete_inertial_links"] == 2
    assert placeholder["complete_inertial_coverage"]["dynamic_links"] == 3


def test_smoke_run_end_to_end(tmp_path: Path, monkeypatch) -> None:
    fixture = write_fixture(tmp_path)
    module = fixture["module"]
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    output = tmp_path / "output"
    exit_code = module.main(
        [
            "--mode",
            "smoke",
            "--dataset-root",
            str(fixture["dataset_root"]),
            "--cohort-manifest",
            str(fixture["cohort_path"]),
            "--category-records-root",
            str(fixture["category_root"]),
            "--table3-records",
            str(fixture["table3_path"]),
            "--output",
            str(output),
        ]
    )
    assert exit_code == 0
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    metrics = summary["metrics"]
    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 2
    # rec_fixture_pass has collision on every visual-bearing link; nocoll does not.
    assert metrics["visual_bearing_collision_coverage"]["passed"] == 1
    assert metrics["visual_bearing_collision_coverage"]["denominator"] == 2
    assert metrics["visual_bearing_collision_coverage"]["link_micro_covered"] == 2
    assert metrics["visual_bearing_collision_coverage"]["link_micro_denominator"] == 4
    # revolute bounded joint passes portability; continuous with effort/velocity passes too.
    assert metrics["joint_limit_portability"]["passed"] == 2
    assert metrics["joint_limit_portability"]["denominator"] == 2
    # only the revolute joint declares damping+friction.
    assert metrics["joint_dynamics_coverage"]["passed"] == 1
    assert metrics["placeholder_mass_incidence"]["status"] == "N/E"
    assert metrics["placeholder_mass_incidence"]["complete_inertial_coverage"]["complete_inertial_links"] == 2
    records = module.load_jsonl(output / "asset_records.jsonl")
    assert sorted(row["asset_key"] for row in records) == ["rec_fixture_nocoll", "rec_fixture_pass"]
    assert all(row["status"] == "completed" for row in records)
    assert (output / "manifest.json").is_file()
    assert (output / "summary.md").is_file()
    assert (output / "protocol_snapshot.md").is_file()
    assert (output / "checkpoint.json").is_file()
