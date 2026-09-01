from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "verify_urdf_lam_supplementary_v1.py"


def load_verifier():
    spec = importlib.util.spec_from_file_location("lam_supplementary_verifier_test", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load supplementary verifier")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def unexecuted_observation(verifier, runtime, reason="fixture unavailable"):
    empty_hash = verifier.canonical_sha256([])
    return {
        "engine_protocol_id": verifier.GENESIS_ENGINE_PROTOCOL_ID,
        "executed": False,
        "illegal_collision": None,
        "clearance_normalized": None,
        "max_eligible_penetration_m": None,
        "runtime_binding": copy.deepcopy(runtime),
        "mapping": {
            "status": "N/E",
            "eligible_pair_count": 0,
            "mapped_pair_count": 0,
            "unmapped_pair_count": 0,
            "overflow_count": 0,
            "pruned_pair_count": 0,
            "eligible": 0,
            "mapped": 0,
            "unmapped": 0,
            "overflow": False,
            "pruned": False,
            "reason": reason,
        },
        "contact_readback": {
            "status": "N/E",
            "success": False,
            "finite": False,
            "overflow_count": 0,
            "pruning_count": 0,
            "overflow": False,
            "pruning": False,
            "reason": reason,
        },
        "readback": {
            "status": "N/E",
            "success": False,
            "finite": False,
            "reason": reason,
        },
        "q_intended_values": [],
        "q_readback_values": [],
        "q_intended_values_sha256": empty_hash,
        "q_readback_values_sha256": empty_hash,
        "q_values_sha256": empty_hash,
        "target_dof_index": None,
        "joint_dof_order": [],
        "joint_dof_order_sha256": empty_hash,
        "source_link_mapping_sha256": empty_hash,
        "contact_readout_source": "detect_collision_then_get_contacts",
        "q_readback_max_abs_error": None,
        "observation_status": "N/E",
        "overflow_or_pruning_detected": False,
        "clearance_status": "N/E",
        "clearance_reason": f"Genesis observation unavailable: {reason}",
        "raw_contact_count": 0,
        "eligible_contact_count": 0,
        "excluded_direct_parent_child_contact_count": 0,
        "terminal": True,
        "status": "not_executed",
    }


def strict_row(verifier, runtime, asset_key, phase, index, dimension):
    row = unexecuted_observation(verifier, runtime)
    row.update(
        {
            "strict_state_key": f"{asset_key}::strict::{phase}::{index}",
            "protocol_id": "protocol",
            "asset_key": asset_key,
            "selection_rank": 1,
            "input_identity_sha256": "identity",
            "phase": phase,
            "sample_index": index,
            "sobol_seed": verifier.SOBOL_SEED,
            "sobol_scramble": True,
            "sobol_dimension": dimension,
        }
    )
    return row


class GenesisObservationTests(unittest.TestCase):
    def setUp(self):
        self.verifier = load_verifier()
        self.runtime = {
            "engine": "genesis",
            "scipy_version": "fixture",
            "q_readback_tolerance": 1e-9,
        }

    def executed_state(self):
        v = self.verifier
        intended = [0.0, 0.25]
        order = [
            {"dof_index": 0, "joint_name": "hinge"},
            {"dof_index": 1, "joint_name": "slide"},
        ]
        return {
            "engine_protocol_id": v.GENESIS_ENGINE_PROTOCOL_ID,
            "runtime_binding": copy.deepcopy(self.runtime),
            "contact_readout_source": "detect_collision_then_get_contacts",
            "q_intended_values": intended,
            "q_readback_values": list(intended),
            "q_intended_values_sha256": v.canonical_sha256(intended),
            "q_readback_values_sha256": v.canonical_sha256(intended),
            "q_values_sha256": v.canonical_sha256(intended),
            "target_dof_index": 1,
            "joint_dof_order": order,
            "joint_dof_order_sha256": v.canonical_sha256(order),
            "joint_name": "slide",
            "joint_value": 0.25,
            "source_link_mapping_sha256": "mapping",
            "mapping": {
                "status": "COMPLETE",
                "eligible_pair_count": 1,
                "mapped_pair_count": 1,
                "unmapped_pair_count": 0,
                "overflow_count": 0,
                "pruned_pair_count": 0,
                "eligible": 1,
                "mapped": 1,
                "unmapped": 0,
                "overflow": False,
                "pruned": False,
            },
            "contact_readback": {
                "status": "COMPLETE",
                "success": True,
                "finite": True,
                "overflow_count": 0,
                "pruning_count": 0,
                "overflow": False,
                "pruning": False,
                "raw_contact_count": 0,
                "eligible_contact_count": 0,
                "excluded_direct_parent_child_contact_count": 0,
            },
            "readback": {
                "status": "COMPLETE",
                "success": True,
                "finite": True,
                "max_abs_error": 0.0,
            },
            "q_readback_max_abs_error": 0.0,
            "observation_status": "COMPLETE",
            "overflow_or_pruning_detected": False,
            "raw_contact_count": 0,
            "eligible_contact_count": 0,
            "excluded_direct_parent_child_contact_count": 0,
            "max_eligible_penetration_m": 0.0,
            "illegal_collision": False,
            "clearance_normalized": None,
            "clearance_status": "N/E",
            "clearance_reason": "Genesis has no complete signed clearance",
        }

    def test_executed_state_closes_runtime_full_q_and_dof_target(self):
        row = self.executed_state()
        intended, readback = self.verifier._validate_genesis_state(
            row, ("asset", "slide", 0), self.runtime
        )
        self.assertEqual(intended, readback)

        drift = copy.deepcopy(row)
        drift["runtime_binding"]["device"] = "other"
        with self.assertRaises(self.verifier.VerificationError):
            self.verifier._validate_genesis_state(drift, ("asset", "slide", 0), self.runtime)

        wrong_target = copy.deepcopy(row)
        wrong_target["joint_dof_order"][1]["joint_name"] = "other"
        wrong_target["joint_dof_order_sha256"] = self.verifier.canonical_sha256(
            wrong_target["joint_dof_order"]
        )
        with self.assertRaises(self.verifier.VerificationError):
            self.verifier._validate_genesis_state(wrong_target, ("asset", "slide", 0), self.runtime)

    def test_alias_overflow_and_unexecuted_reason_fail_closed(self):
        row = self.executed_state()
        row["contact_readback"]["overflow"] = True
        with self.assertRaises(self.verifier.VerificationError):
            self.verifier._validate_genesis_state(row, ("asset", "slide", 0), self.runtime)

        unavailable = unexecuted_observation(self.verifier, self.runtime)
        self.verifier._validate_unexecuted_genesis_state(
            unavailable, ("asset", "slide", 0), self.runtime
        )
        unavailable["mapping"]["reason"] = ""
        with self.assertRaises(self.verifier.VerificationError):
            self.verifier._validate_unexecuted_genesis_state(
                unavailable, ("asset", "slide", 0), self.runtime
            )


class StrictDenominatorTests(unittest.TestCase):
    def setUp(self):
        self.verifier = load_verifier()
        self.runtime = {
            "engine": "genesis",
            "scipy_version": "fixture",
            "q_readback_tolerance": 1e-9,
        }

    def source(self, with_joint):
        v = self.verifier
        joints = {}
        order = {}
        if with_joint:
            joints[("asset", "slide")] = {
                "joint_name": "slide",
                "range_lower": 0.0,
                "range_upper": 1.0,
                "sample_values": [index / 20 for index in range(21)],
            }
            order[("asset", "slide")] = 0
        return v.SourceCohort({}, {}, joints, order, "", "", "", "")

    def run_fixture(self, with_joint):
        v = self.verifier
        source = self.source(with_joint)
        item = {"asset_key": "asset", "input_identity_sha256": "identity"}
        items = {1: item}
        assets = {
            1: {
                "asset_key": "asset",
                "strict_collision_pass_no_method_allowance": False,
                "strict_collision_pass_registered_allowance": False,
            }
        }
        states_by_joint = {}
        if with_joint:
            states = []
            for index in range(21):
                state = unexecuted_observation(v, self.runtime)
                state.update(
                    {
                        "asset_key": "asset",
                        "joint_name": "slide",
                        "sample_index": index,
                    }
                )
                states.append(state)
            states_by_joint[("asset", "slide")] = states
        rows = [strict_row(v, self.runtime, "asset", "rest", 0, int(with_joint))]
        if with_joint:
            rows.extend(
                strict_row(v, self.runtime, "asset", "multi_joint_sobol", index, 1)
                for index in range(64)
            )
        config = v.VerifierConfig(sample_size=1, joint_count=int(with_joint))
        _, derived = v._validate_strict_state_records(
            rows,
            source,
            items,
            assets,
            states_by_joint,
            "protocol",
            self.runtime,
            config,
        )
        return rows, derived

    def test_movable_asset_requires_rest_plus_64_sobol(self):
        rows, derived = self.run_fixture(True)
        self.assertEqual(len(rows), 65)
        self.assertFalse(derived["asset"])

    def test_zero_dof_asset_has_rest_only_and_fails_strict(self):
        rows, derived = self.run_fixture(False)
        self.assertEqual(len(rows), 1)
        self.assertFalse(derived["asset"])


class AggregateSemanticsTests(unittest.TestCase):
    def test_table2_and_table4b_do_not_turn_missing_geometry_into_zero_cost(self):
        v = load_verifier()
        source = v.SourceCohort(
            {},
            {},
            {("asset", "slide"): {"joint_name": "slide"}},
            {("asset", "slide"): 0},
            "",
            "",
            "",
            "",
        )
        ne = {"status": "N/E", "value": None, "reason": "not measured"}
        asset = {
            "asset_key": "asset",
            "evaluation_success": False,
            "visual_bearing_link_count": 2,
            "collision_covered_visual_bearing_link_count": 2,
            "visual_bearing_collision_coverage_asset_pass": True,
            "mass_evaluable_link_count": 0,
            "placeholder_mass_link_count": 0,
            "analytic_collision_element_count": 0,
            "loadable_collision_element_count": 0,
            "collision_shape_count": 0,
            "collision_mesh_triangle_count": 0,
            "intra_link_redundant_volume_m3": 0.0,
            "intra_link_shape_volume_m3": 0.0,
            "visual_to_collision_p95_normalized": dict(ne),
            "collision_to_visual_p95_normalized": dict(ne),
            "collision_load_time_seconds": dict(ne),
            "shapes_per_visual_bearing_link": dict(ne),
            "collision_mesh_triangles_per_asset": dict(ne),
            "intra_link_redundancy": {
                "status": "N/E",
                "value": None,
                "redundant_volume_m3": None,
                "shape_volume_m3": None,
                "measured_links": 0,
                "intended_links": 0,
                "reason": "not measured",
            },
            "release_receipt_bound": False,
            "release_receipt_replay_pass": False,
            "deterministic_rebuild_eligible": False,
            "deterministic_rebuild_match": False,
            "eligible_non_adjacent_pair_count": 0,
            "registered_method_allowance_pair_count": 0,
        }
        joint = {
            "asset_key": "asset",
            "joint_name": "slide",
            "intended_state_count": 21,
            "executed_state_count": 0,
            "joint_limit_portable": False,
            "dynamics_present": False,
            "table3_joint_pass": False,
            "full_range_cf_pass": False,
            "bounded": True,
            "limit_reachable": False,
        }
        result = v.aggregate_records(
            {1: asset},
            {("asset", "slide"): joint},
            {("asset", "slide"): []},
            {"asset": False},
            source,
            v.VerifierConfig(sample_size=1, joint_count=1),
        )
        visual = result["table2_supplementary"]["visual_bearing_link_collision_coverage"]
        self.assertEqual(visual["rate"], 1.0)
        self.assertEqual(visual["link_micro_rate"], 1.0)
        placeholder = result["table2_supplementary"]["placeholder_mass_incidence"]
        self.assertEqual(placeholder["status"], "N/E")
        self.assertIsNone(placeholder["rate"])
        table4b = result["table4b"]
        self.assertEqual(table4b["shapes_per_visual_bearing_link"]["status"], "N/E")
        self.assertEqual(table4b["collision_mesh_triangles_per_asset"]["status"], "N/E")
        self.assertEqual(table4b["intra_link_redundancy"]["status"], "N/E")

        measured_asset = copy.deepcopy(asset)
        measured_asset["asset_key"] = "measured"
        measured_asset["intra_link_redundant_volume_m3"] = 1.0
        measured_asset["intra_link_shape_volume_m3"] = 2.0
        measured_asset["intra_link_redundancy"] = {
            "status": "COMPLETE",
            "value": 0.5,
            "redundant_volume_m3": 1.0,
            "shape_volume_m3": 2.0,
            "measured_links": 1,
            "intended_links": 1,
        }
        partial = v.aggregate_records(
            {1: asset, 2: measured_asset},
            {("asset", "slide"): joint},
            {("asset", "slide"): []},
            {"asset": False, "measured": False},
            source,
            v.VerifierConfig(sample_size=2, joint_count=1),
        )["table4b"]["intra_link_redundancy"]
        self.assertEqual(partial["status"], "PARTIAL")
        self.assertEqual(partial["complete_assets"], 1)
        self.assertEqual(partial["measured_assets"], 1)
        self.assertEqual(partial["intended_assets"], 2)
        self.assertEqual(partial["link_coverage"], 1.0)
        v._validate_measurement_semantics({"intra_link_redundancy": partial})


class SummaryBindingTests(unittest.TestCase):
    def test_summary_hash_is_bound_to_recomputed_aggregates(self):
        v = load_verifier()
        recomputed = {"schema_version": 1, "metric": {"status": "N/E", "value": None, "reason": "fixture"}}
        source = v.SourceCohort({}, {}, {}, {}, "keys", "records", "manifest", "content")
        summary = {
            "protocol_id": "protocol",
            "status": "COMPLETE",
            "cohort": {"selected": 1, "joints": 0},
            "input_binding": {
                "table3_asset_records_sha256": "records",
                "table3_manifest_sha256": "manifest",
                "table3_manifest_content_sha256": "content",
                "ordered_selected_asset_keys_sha256": "keys",
            },
            "verification_aggregates": copy.deepcopy(recomputed),
            "verification_aggregates_sha256": v.canonical_sha256(recomputed),
        }
        v._validate_summary(
            summary,
            recomputed,
            {"qualification_smoke": False},
            "protocol",
            source,
            v.VerifierConfig(sample_size=1, joint_count=0),
        )
        summary["verification_aggregates_sha256"] = "0" * 64
        with self.assertRaises(v.VerificationError):
            v._validate_summary(
                summary,
                recomputed,
                {"qualification_smoke": False},
                "protocol",
                source,
                v.VerifierConfig(sample_size=1, joint_count=0),
            )


if __name__ == "__main__":
    unittest.main()
