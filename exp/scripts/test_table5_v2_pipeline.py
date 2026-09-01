#!/usr/bin/env python3
"""Simulator-free contract tests for the Table 5 v2 pipeline."""

from __future__ import annotations

import math
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_n200_manifest as base  # noqa: E402
import table5_v2_aggregate as aggregate  # noqa: E402
import table5_v2_aggregate_r2 as aggregate_r2  # noqa: E402
import table5_v2_prepare as prepare  # noqa: E402
import table5_v2_prepare_r2 as prepare_r2  # noqa: E402
import table5_v2_runtime as runtime  # noqa: E402
import table5_v2_runtime_compat as runtime_compat  # noqa: E402
import table5_v2_runtime_r2 as runtime_r2  # noqa: E402
import table5_v2_sample_n200 as sample  # noqa: E402
import table5_pva_physics as pva_physics  # noqa: E402
import run_table5_v2_native as native_runner  # noqa: E402


URDF = """\
<robot name="fixture">
  <link name="base">
    <collision><geometry><box size="1 2 3"/></geometry></collision>
  </link>
  <link name="door">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/>
    </inertial>
    <collision><geometry><box size="0.2 1 2"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="10" velocity="2"/>
  </joint>
</robot>
"""


class FakeAdapter:
    mapped_joint_names = ["hinge"]
    observed_link_names = ["base", "door"]
    observed_joint_names = ["hinge"]

    def __init__(self) -> None:
        self.positions = {"hinge": 0.5}

    def reset(self, positions: dict[str, float]) -> None:
        self.positions.update(positions)

    def step(self, efforts: dict[str, float]) -> None:
        del efforts

    def state(self) -> dict[str, dict[str, float]]:
        return {"hinge": {"q": self.positions["hinge"], "qdot": 0.0}}

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        q = self.positions["hinge"]
        return {
            "base": {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]},
            "door": {
                "translation": [0.5, 0.0, 0.0],
                "rotation": [math.cos(q / 2.0), 0.0, 0.0, math.sin(q / 2.0)],
            },
        }


def fixture_row(tree: dict, scalar: list[dict]) -> dict:
    return {
        "joint_tree": tree,
        "scalar_joints": scalar,
        "bounding_box_diagonal": math.sqrt(14.0),
    }


def synthetic_protocol() -> dict:
    return {
        "runtime": {
            "reset_repetitions": 2,
            "passive_settling": {"steps": 2},
            "actuation": {"trajectory": {"ramp_steps": 240, "hold_steps": 120}},
            "limit_enforcement": {
                "steps_each": 2,
                "targets_normalized": [-0.1, 1.1],
            },
        },
        "cross_simulator": {
            "joint_rmse": {"sample_steps": list(range(0, 361, 12))},
            "fk_probe": {
                "gravity": "off_by_direct_reset_without_step",
                "contact_response": "not_stepped",
            },
        },
        "v2_metrics": {
            "metric_semantics_id": runtime_r2.METRIC_SEMANTICS_ID,
            "fk_probe_alphas": [0.0, 0.25, 0.5, 0.75, 1.0],
            "bootstrap": {"resamples": 20, "seed": "fixture"},
            "passive_stable_rollout": {
                "steps": 2,
                "control": "zero_applied_joint_force",
            },
        },
    }


def synthetic_record(trace: list[float]) -> dict:
    fk_samples = [
        {
            "status": "evaluated",
            "position_error_over_bbox": 0.0,
            "rotation_error_rad": 0.0,
        }
        for _ in range(5)
    ]
    return {
        "terminal_status": "completed",
        "evaluation": {
            "v2": {
                "schema_version": runtime.V2_EVIDENCE_SCHEMA,
                "import": {"passed": True, "first_step": {"passed": True}},
                "dof_mapping": {"mapped_canonical_scalar_joint_names": ["hinge"]},
                "physics": {"status": "ready"},
                "fk_probe": {
                    "candidate_sample_count": 5,
                    "evaluated_sample_count": 5,
                    "samples": fk_samples,
                },
                "stable_rollout": {"passed": True},
            },
            "diagnostics": {
                "actuation": [
                    {
                        "joint_name": "hinge",
                        "trajectory": {
                            "sample_steps": list(range(0, 361, 12)),
                            "normalized_positions": trace,
                        },
                    }
                ],
                "limit_enforcement": [
                    {
                        "joint_name": "hinge",
                        "targets": [
                            {"minimum_q": 0.0, "maximum_q": 1.0},
                            {"minimum_q": 0.0, "maximum_q": 1.0},
                        ],
                    }
                ],
            },
        },
    }


def _receipt(value: dict) -> dict:
    result = dict(value)
    result["receipt_sha256"] = aggregate.canonical_sha256(
        result, exclude_fields=("receipt_sha256",)
    )
    return result


def synthetic_r2_record(trace: list[float]) -> dict:
    record = synthetic_record(trace)
    record["evaluation"]["v2"] = {
        "schema_version": runtime_r2.V2_EVIDENCE_SCHEMA,
        "metric_semantics_id": runtime_r2.METRIC_SEMANTICS_ID,
        "import": _receipt(
            {
                "schema_version": runtime_r2.IMPORT_RECEIPT_SCHEMA,
                "passed": True,
                "criterion": "native_simulator_asset_load",
            }
        ),
        "dof_mapping": {
            "mapped_canonical_scalar_joint_names": ["hinge"],
        },
        "physics": {"status": "ready"},
        "fk_probe": record["evaluation"]["v2"]["fk_probe"],
        "stable_rollout": _receipt(
            {
                "schema_version": runtime_r2.STABLE_RECEIPT_SCHEMA,
                "passed": True,
                "criterion": "fixed_step_zero_force_finite_passive_rollout",
                "required_steps": 2,
                "steps_completed": 2,
                "finite_state_steps": 2,
                "finite_pose_steps": 2,
                "checks": {
                    "reset_completed": True,
                    "all_steps_completed": True,
                    "all_mapped_states_finite": True,
                    "all_observed_link_poses_finite": True,
                    "mapping_unchanged": True,
                },
                "error": None,
            }
        ),
    }
    return record


class PrepareTests(unittest.TestCase):
    def test_semantic_scope_exclusions_are_pva_only(self) -> None:
        self.assertEqual(
            sample.dataset_eligibility_reason(
                sample.PVA_SLUG, "Public Restroom", "fixture", 1
            ),
            "public_toilet",
        )
        self.assertIsNone(
            sample.dataset_eligibility_reason(
                "lam_released_outputs", "Public Restroom", "fixture", 1
            )
        )
        self.assertEqual(
            sample.dataset_eligibility_reason(
                "lam_released_outputs", "Cabinet", "fixture", 21
            ),
            "movable_joint_count_gt_20",
        )

    def test_bbox_and_inertial_audit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            urdf = root / "model.urdf"
            urdf.write_text(URDF, encoding="utf-8")
            parsed = base._parse_urdf(root, urdf)
            self.assertEqual(parsed["issues"], [])
            row = fixture_row(parsed["joint_tree"], parsed["scalar_joints"])
            bbox = prepare.derive_collision_bbox(urdf, root, row["joint_tree"])
            self.assertEqual(bbox["status"], "available")
            self.assertGreater(bbox["diagonal_m"], 0.0)
            audit = prepare.audit_urdf_inertials(urdf, row["joint_tree"])
            self.assertEqual(audit["status"], "ready")
            self.assertEqual(audit["required_link_count"], 1)

    def test_missing_nonroot_inertial_is_blocked(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            urdf = root / "model.urdf"
            urdf.write_text(
                URDF.replace("    <inertial>", "    <!-- <inertial>", 1).replace(
                    "    </inertial>", "    </inertial> -->", 1
                ),
                encoding="utf-8",
            )
            parsed = base._parse_urdf(root, urdf)
            audit = prepare.audit_urdf_inertials(urdf, parsed["joint_tree"])
            self.assertEqual(audit["status"], "blocked")

    def test_repeated_link_inertial_is_blocked(self) -> None:
        repeated = URDF.replace(
            "    </inertial>",
            '    </inertial>\n    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>',
            1,
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            urdf = root / "model.urdf"
            urdf.write_text(repeated, encoding="utf-8")
            parsed = base._parse_urdf(root, urdf)
            audit = prepare.audit_urdf_inertials(urdf, parsed["joint_tree"])
            self.assertEqual(audit["status"], "blocked")
            invalid = audit["invalid_required_link_names"]
            self.assertEqual(invalid, ["door"])

    def test_bbox_uses_visual_fallback_only_when_collision_is_absent(self) -> None:
        visual_only = URDF.replace("<collision>", "<visual>").replace(
            "</collision>", "</visual>"
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            urdf = root / "model.urdf"
            urdf.write_text(visual_only, encoding="utf-8")
            parsed = base._parse_urdf(root, urdf)
            bbox = prepare.derive_collision_bbox(urdf, root, parsed["joint_tree"])
            self.assertEqual(bbox["status"], "available")
            self.assertEqual(bbox["geometry_role"], "visual")

    def test_pva_compiler_omits_appearance_only_collision(self) -> None:
        source_text = """\
<robot name="physics_fixture">
  <link name="base">
    <visual name="solid"><geometry><box size="1 1 1"/></geometry></visual>
    <collision name="solid"><geometry><box size="1 1 1"/></geometry></collision>
    <visual name="label"><geometry><box size="0.1 0.1 0.01"/></geometry></visual>
    <collision name="label"><geometry><box size="0.1 0.1 0.01"/></geometry></collision>
  </link>
  <link name="connector">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <joint name="fixed" type="fixed"><parent link="base"/><child link="connector"/></joint>
</robot>
"""
        values = {
            "density_kg_m3": 1000.0,
            "dynamic_friction_coefficient": 0.4,
            "poissons_ratio": 0.3,
            "restitution_coefficient": 0.1,
            "static_friction_coefficient": 0.5,
            "youngs_modulus_pa": 1.0e9,
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "model.urdf"
            sidecar = root / "physics.json"
            injected = root / "model.physics.urdf"
            plan_path = root / "physics_plan.json"
            source.write_text(source_text, encoding="utf-8")
            sidecar.write_text(
                json.dumps(
                    {
                        "schema_version": pva_physics.SIDECAR_SCHEMA,
                        "model_urdf_sha256": pva_physics.sha256_file(source),
                        "fields": list(pva_physics.PHYSICS_FIELDS),
                        "bindings": [
                            {
                                "surface_key": "base::solid",
                                "appearance_only": False,
                                "values": values,
                            },
                            {
                                "surface_key": "base::label",
                                "appearance_only": True,
                                "values": {key: None for key in values},
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            plan = pva_physics.build_injected_asset(
                source_urdf=source,
                physics_path=sidecar,
                destination_urdf=injected,
                plan_path=plan_path,
            )
            self.assertEqual(plan["appearance_only_collision_count"], 1)
            self.assertEqual(plan["collisionless_valid_inertial_link_count"], 1)
            self.assertNotIn(
                'collision name="label"', injected.read_text(encoding="utf-8")
            )


class RuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = {
            "links": ["base", "door"],
            "root_links": ["base"],
            "joints": [
                {
                    "name": "hinge",
                    "type": "revolute",
                    "parent": "base",
                    "child": "door",
                    "origin_xyz": [0.5, 0.0, 0.0],
                    "origin_rpy": [0.0, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0],
                    "fk_supported": True,
                    "lower": 0.0,
                    "upper": 1.0,
                    "effort": 10.0,
                    "velocity": 2.0,
                }
            ],
        }
        self.row = fixture_row(self.tree, self.tree["joints"])

    def test_mapping_first_step_and_fk(self) -> None:
        adapter = FakeAdapter()
        mapping = runtime._mapping_receipt(adapter, self.row)
        self.assertEqual(mapping["mapped_count"], 1)
        eligible = runtime._eligible_joints(self.row, ["hinge"])
        self.assertTrue(runtime._first_step(adapter, eligible)["passed"])
        probe = runtime.fk_probe(adapter, self.row, synthetic_protocol())
        self.assertEqual(probe["candidate_sample_count"], 5)
        self.assertEqual(probe["evaluated_sample_count"], 5)
        self.assertLess(
            max(item["position_error_m"] for item in probe["samples"]), 1e-12
        )

    def test_only_official_mujoco_mjcf_binding_is_accepted(self) -> None:
        source = {
            "format": "mjcf",
            "representation": "official_released_mjcf",
            "path": "/tmp/fixture.xml",
            "sha256": "0" * 64,
        }
        row = {"simulator_sources": {"mujoco": source, "genesis": source}}
        resolved = runtime._simulator_source(row, "mujoco")
        self.assertEqual(resolved["format"], "mjcf")
        with self.assertRaises(runtime.RuntimeErrorV2):
            runtime._simulator_source(row, "genesis")

        row["simulator_sources"]["mujoco"] = {
            **source,
            "representation": "converted_mjcf",
        }
        with self.assertRaises(runtime.RuntimeErrorV2):
            runtime._simulator_source(row, "mujoco")

    def test_stability_is_recomputed_from_complete_finite_evidence(self) -> None:
        adapter = FakeAdapter()
        protocol = synthetic_protocol()
        mapping = runtime._mapping_receipt(adapter, self.row)
        positions = [0.5 for _ in range(361)]
        evaluation = {
            "diagnostics": {
                "reset": [
                    {"finite": True, "error": None},
                    {"finite": True, "error": None},
                ],
                "settling": {"finite": True, "steps_completed": 2, "error": None},
                "actuation": [
                    {
                        "joint_name": "hinge",
                        "finite": True,
                        "steps_completed": 360,
                        "error": None,
                        "full_measured_positions_q": positions,
                        "constraint_drift": {"finite": True, "steps_compared": 360},
                        "missing_descendant_link_names": [],
                    }
                ],
                "limit_enforcement": [
                    {
                        "joint_name": "hinge",
                        "targets": [
                            {
                                "finite": True,
                                "steps_completed": 2,
                                "error": None,
                                "minimum_q": 0.0,
                                "maximum_q": 1.0,
                            },
                            {
                                "finite": True,
                                "steps_completed": 2,
                                "error": None,
                                "minimum_q": 0.0,
                                "maximum_q": 1.0,
                            },
                        ],
                    }
                ],
            }
        }
        stable = runtime._full_finite_rollout(
            evaluation, adapter, self.row, protocol, mapping
        )
        self.assertTrue(stable["passed"])
        evaluation["diagnostics"]["actuation"][0]["full_measured_positions_q"][3] = (
            float("nan")
        )
        unstable = runtime._full_finite_rollout(
            evaluation, adapter, self.row, protocol, mapping
        )
        self.assertFalse(unstable["passed"])

    def test_r2_passive_stability_runs_even_without_mapped_dofs(self) -> None:
        class NoDofAdapter(FakeAdapter):
            mapped_joint_names: list[str] = []
            observed_joint_names: list[str] = []

            def __init__(self) -> None:
                self.steps = 0

            def reset(self, positions: dict[str, float]) -> None:
                self.asserted_positions = positions

            def step(self, efforts: dict[str, float]) -> None:
                self.steps += 1
                self.asserted_efforts = efforts

            def state(self) -> dict[str, dict[str, float]]:
                return {}

            def link_poses(self) -> dict[str, dict[str, list[float]]]:
                return {
                    "base": {
                        "translation": [0.0, 0.0, 0.0],
                        "rotation": [1.0, 0.0, 0.0, 0.0],
                    },
                    "door": {
                        "translation": [0.5, 0.0, 0.0],
                        "rotation": [1.0, 0.0, 0.0, 0.0],
                    },
                }

        adapter = NoDofAdapter()
        mapping = runtime._mapping_receipt(adapter, self.row)
        stable = runtime_r2.passive_stable_rollout(
            adapter, self.row, synthetic_protocol(), mapping
        )
        self.assertEqual(adapter.steps, 2)
        self.assertEqual(stable["mapped_dof_count"], 0)
        self.assertTrue(stable["passed"])

    def test_r2_passive_stability_fails_nonfinite_pose(self) -> None:
        class NonfiniteAdapter(FakeAdapter):
            def link_poses(self) -> dict[str, dict[str, list[float]]]:
                poses = super().link_poses()
                poses["door"]["translation"][0] = float("nan")
                return poses

        adapter = NonfiniteAdapter()
        mapping = runtime._mapping_receipt(adapter, self.row)
        stable = runtime_r2.passive_stable_rollout(
            adapter, self.row, synthetic_protocol(), mapping
        )
        self.assertFalse(stable["passed"])
        self.assertEqual(stable["steps_completed"], 0)

    def test_r2_native_load_receipt_survives_later_adapter_failure(self) -> None:
        module = types.ModuleType("pybullet")

        def load_urdf(*_args: object, **_kwargs: object) -> int:
            return 7

        module.loadURDF = load_urdf
        source = {
            "format": "urdf",
            "representation": "official_released_urdf",
            "sha256": "a" * 64,
        }
        receipts: list[dict] = []
        with mock.patch.dict(sys.modules, {"pybullet": module}):
            with self.assertRaisesRegex(RuntimeError, "post-load diagnostic"):
                with runtime_r2._observe_native_load(
                    "pybullet", source, receipts.append
                ):
                    self.assertEqual(module.loadURDF("fixture.urdf"), 7)
                    raise RuntimeError("post-load diagnostic")
        self.assertIs(module.loadURDF, load_urdf)
        self.assertEqual(len(receipts), 1)
        self.assertEqual(receipts[0]["native_load_operation"], "loadURDF")
        self.assertTrue(
            aggregate_r2._valid_receipt(receipts[0], runtime_r2.IMPORT_RECEIPT_SCHEMA)
        )

    def test_r2_terminal_timeout_preserves_checkpointed_import(self) -> None:
        source = {
            "format": "urdf",
            "representation": "official_released_urdf",
            "sha256": "a" * 64,
        }
        response = {
            "metrics": runtime._false_legacy_metrics(),
            "v2": runtime_r2._v2_block(),
        }
        response["v2"]["import"] = runtime_r2._native_import_receipt("pybullet", source)
        outcome = runtime_r2._core._runtime.WorkerOutcome(
            timed_out=True,
            response=response,
        )
        record = runtime_r2._terminal_record({"timeout_s": 10.0}, outcome)
        self.assertEqual(record["terminal_status"], "timeout")
        self.assertTrue(aggregate_r2._import_pass(record))
        self.assertFalse(aggregate_r2._stable_pass(record))

    def test_genesis_compat_reconstructs_only_coincident_fixed_root(self) -> None:
        row = {
            "joint_tree": {
                "root_links": ["world"],
                "joints": [
                    {
                        "name": "world_joint",
                        "type": "fixed",
                        "parent": "world",
                        "child": "link_0",
                        "origin_xyz": [0.0, 0.0, 0.0],
                        "origin_rpy": [0.0, 0.0, 0.0],
                    }
                ],
            }
        }
        receipt = runtime_compat.fixed_root_mapping(row, ["link_0"])
        self.assertIsNotNone(receipt)
        self.assertEqual(receipt["canonical_root_link_name"], "world")
        self.assertEqual(receipt["surrogate_observed_link_name"], "link_0")

        shifted = json.loads(json.dumps(row))
        shifted["joint_tree"]["joints"][0]["origin_xyz"] = [0.1, 0.0, 0.0]
        self.assertIsNone(runtime_compat.fixed_root_mapping(shifted, ["link_0"]))

    def test_native_runner_routes_all_engines_through_r2_entrypoint(self) -> None:
        common = {
            "prepared": Path("prepared.json"),
            "output": Path("out"),
            "workers": 1,
            "executable": "/usr/bin/python3",
            "gpus": ["4"],
            "datasets": ["infinigen_sim"],
        }
        genesis = native_runner._runtime_command(simulator="genesis", **common)
        pybullet = native_runner._runtime_command(simulator="pybullet", **common)
        self.assertEqual(Path(genesis[1]), native_runner.RUNTIME_SCRIPT)
        self.assertEqual(Path(pybullet[1]), native_runner.RUNTIME_SCRIPT)

    def test_r2_protocol_binds_actual_entrypoint_and_compatibility(self) -> None:
        cohort = {
            "schema_version": "fixture",
            "cohort_sha256": "1" * 64,
            "manifest_sha256": "2" * 64,
            "protocol_sha256": "3" * 64,
            "protocol": {},
        }
        protocol = prepare_r2._protocol(cohort)
        self.assertEqual(protocol["protocol_id"], runtime_r2.PROTOCOL_ID)
        self.assertEqual(
            protocol["v2_metrics"]["metric_semantics_id"],
            runtime_r2.METRIC_SEMANTICS_ID,
        )
        runtime_r2._validate_protocol(protocol)

    def test_r2_preflight_does_not_gate_native_load_on_static_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "asset.urdf"
            source.write_text(URDF, encoding="utf-8")
            digest = base.sha256_file(source)
            row = {
                "preflight": {
                    "simulator_eligible": False,
                    "issues": ["invalid_joint_tree"],
                },
                "simulator_sources": {
                    simulator: {
                        "format": "urdf",
                        "representation": "released",
                        "path": str(source),
                        "sha256": digest,
                        "package_root": str(source.parent),
                    }
                    for simulator in runtime.SIMULATORS
                },
            }
            self.assertIsNone(runtime_r2._r2_preflight_failure(row, source))

    def test_compat_install_patches_dynamic_genesis_initialization(self) -> None:
        runtime_compat.install()
        self.assertIs(
            runtime_compat._runtime._runtime.DynamicGenesisAdapter.__init__,
            runtime_compat._dynamic_genesis_init,
        )


class AggregateTests(unittest.TestCase):
    def test_complete_metrics_and_missing_record_fail_closed(self) -> None:
        tree = {
            "links": ["base", "door"],
            "root_links": ["base"],
            "joints": [
                {
                    "name": "hinge",
                    "type": "revolute",
                    "parent": "base",
                    "child": "door",
                    "lower": 0.0,
                    "upper": 1.0,
                    "effort": 10.0,
                    "velocity": 2.0,
                }
            ],
        }
        row = fixture_row(tree, tree["joints"])
        rows = [row for _ in range(aggregate.EXPECTED_N)]
        dataset = {"dataset_slug": "fixture", "dataset_name": "Fixture", "rows": rows}
        steps = list(range(0, 361, 12))
        trace = [
            aggregate._legacy_minimum_jerk(step / 240.0) if step <= 240 else 1.0
            for step in steps
        ]
        record = synthetic_record(trace)
        records = {
            simulator: [record for _ in rows] for simulator in aggregate.SIMULATORS
        }
        summary = aggregate._dataset_summary(dataset, records, synthetic_protocol())
        self.assertEqual(summary["table5a"]["import_success"]["percentage"], 100.0)
        self.assertAlmostEqual(summary["table5a"]["tracking_nrmse_p95"]["p95"], 0.0)
        self.assertEqual(
            summary["table5b"]["genesis"]["fk_position_error_p95"][
                "coverage_percentage"
            ],
            100.0,
        )

        records["pybullet"] = list(records["pybullet"])
        records["pybullet"][0] = None
        missing = aggregate._dataset_summary(dataset, records, synthetic_protocol())
        self.assertEqual(
            missing["table5b"]["pybullet"]["import_success"]["percentage"], 99.5
        )
        self.assertEqual(
            missing["table5b"]["pybullet"]["dof_coverage"]["percentage"],
            99.5,
        )
        self.assertAlmostEqual(
            missing["table5b"]["pybullet"]["fk_position_error_p95"][
                "coverage_percentage"
            ],
            99.5,
        )
        self.assertNotIn("trajectory_nrmse_p95", missing["table5b"])

    def test_r2_import_is_independent_of_mapping_physics_and_first_step(self) -> None:
        imported = {
            "terminal_status": "worker_error",
            "evaluation": {
                "v2": {
                    "schema_version": runtime_r2.V2_EVIDENCE_SCHEMA,
                    "metric_semantics_id": runtime_r2.METRIC_SEMANTICS_ID,
                    "import": _receipt(
                        {
                            "schema_version": runtime_r2.IMPORT_RECEIPT_SCHEMA,
                            "passed": True,
                            "criterion": "native_simulator_asset_load",
                        }
                    ),
                    "dof_mapping": None,
                    "physics": None,
                    "stable_rollout": {"passed": False},
                }
            },
        }
        self.assertTrue(aggregate_r2._import_pass(imported))
        self.assertFalse(aggregate_r2._stable_pass(imported))

    def test_r2_aggregation_reports_declared_trajectory_coverage(self) -> None:
        aggregate_r2.install()
        tree = {
            "links": ["base", "door"],
            "root_links": ["base"],
            "joints": [
                {
                    "name": "hinge",
                    "type": "revolute",
                    "parent": "base",
                    "child": "door",
                    "lower": 0.0,
                    "upper": 1.0,
                    "effort": 10.0,
                    "velocity": 2.0,
                }
            ],
        }
        row = fixture_row(tree, tree["joints"])
        rows = [row for _ in range(aggregate.EXPECTED_N)]
        dataset = {"dataset_slug": "fixture", "dataset_name": "Fixture", "rows": rows}
        steps = list(range(0, 361, 12))
        trace = [
            aggregate._legacy_minimum_jerk(step / 240.0) if step <= 240 else 1.0
            for step in steps
        ]
        record = synthetic_r2_record(trace)
        records = {
            simulator: [record for _ in rows] for simulator in aggregate.SIMULATORS
        }
        summary = aggregate_r2._dataset_summary(dataset, records, synthetic_protocol())
        self.assertEqual(summary["table5a"]["import_success"]["percentage"], 100.0)
        self.assertEqual(summary["table5a"]["stable_rollout"]["percentage"], 100.0)
        self.assertEqual(summary["table5a"]["trajectory_coverage"]["percentage"], 100.0)
        report = aggregate_r2.render_markdown(
            {"classification": "COMPLETE", "datasets": [summary]}
        )
        self.assertIn("Import Success (%)", report)
        self.assertIn("Trajectory Coverage (%)", report)


if __name__ == "__main__":
    unittest.main()
