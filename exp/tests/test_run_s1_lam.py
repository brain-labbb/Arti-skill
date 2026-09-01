from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_s1_lam.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("s1_lam_runner_under_test", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def table4_fixture() -> tuple[dict, dict, list[dict]]:
    identity = {
        "protocol_id": "urdf_sim_ready_table4_lam_n800_v1",
        "order": 0,
        "dataset_id": "lam_0000",
        "asset_key": "viable:objects/fixture/fixture_001",
        "category": "fixture",
        "input_identity_sha256": "7" * 64,
        "selection_rank": 1,
        "selection_hash": "1" * 64,
        "tier": "viable",
        "rel_path": "objects/fixture/fixture_001",
        "object_release_id": "fixture_001",
        "package_relpath": "released_outputs/objects/fixture/fixture_001",
        "model_urdf_sha256": "2" * 64,
        "package_content_manifest_sha256": "3" * 64,
        "source_record_sha256": "4" * 64,
        "source_manifest_record_sha256": "5" * 64,
    }
    item = {
        **identity,
        "movable_dof_count": 1,
        "range_evaluable_dof_count": 1,
        "rest_state_expected": 1,
        "single_state_expected": 21,
        "sobol_state_expected": 64,
    }
    item["joint_specs"] = [{
        "xml_index": 0,
        "name": "hinge",
        "type": "revolute",
        "lower": 0.0,
        "upper": 1.0,
        "range_evaluable": True,
    }]
    item["joint_specs_sha256"] = canonical_sha256(item["joint_specs"])

    def state(
        phase: str,
        sample_index: int,
        values: list[float],
        joint_name: str | None = None,
    ) -> dict:
        return {
            **identity,
            "phase": phase,
            "sample_index": sample_index,
            "joint_name": joint_name,
            "joint_values_sha256": canonical_sha256(values),
            "reset_readback_max_abs_error": 0.0,
            "all_pair_contact_count": 0,
            "all_pair_illegal_penetration_count": 0,
            "all_pair_max_penetration_m": 0.0,
            "non_adjacent_contact_count": 0,
            "non_adjacent_illegal_penetration_count": 0,
            "non_adjacent_max_penetration_m": 0.0,
            "metric_max_penetration_m": 0.0,
        }

    from scipy.stats import qmc

    states = [state("rest", 0, [0.0])]
    states.extend(
        state("single_joint_sweep", index, [index / 20.0], "hinge")
        for index in range(21)
    )
    states.extend(
        state("multi_joint_sobol", index, [float(vector[0])])
        for index, vector in enumerate(
            qmc.Sobol(d=1, scramble=True, seed=20260813).random_base2(m=6)
        )
    )
    asset = {
        **identity,
        "load_success": True,
        "rest_state_expected": 1,
        "rest_state_executed": 1,
        "rest_non_adjacent_free": 1,
        "rest_all_pair_cf": True,
        "rest_non_adjacent_cf": True,
        "single_state_expected": 21,
        "single_state_executed": 21,
        "single_non_adjacent_free": 21,
        "joint_single_sweep_cf_passed": 1,
        "single_joint_sweep_cf": True,
        "sobol_state_expected": 64,
        "sobol_state_executed": 64,
        "sobol_non_adjacent_free": 64,
        "multi_joint_sobol_cf": True,
        "measurement_complete": True,
        "strict_collision_pass": True,
        "state_records_sha256": canonical_sha256(states),
    }
    return item, asset, states


def test_reaggregates_strict_pass_from_bound_raw_states() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()

    result = runner.reaggregate_table4_asset(item, asset, states)

    assert result["strict_collision_pass"] is True
    assert result["measurement_complete"] is True
    assert result["state_record_count"] == 86
    assert result["state_records_sha256"] == canonical_sha256(states)


def test_reaggregation_rejects_identity_preserving_summary_tamper() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    asset["strict_collision_pass"] = False

    with pytest.raises(ValueError, match="strict_collision_pass mismatch"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_rejects_state_identity_drift_even_with_updated_hash() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    states[0]["asset_key"] = "viable:objects/other/other_001"
    asset["state_records_sha256"] = canonical_sha256(states)

    with pytest.raises(ValueError, match="asset_key mismatch"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_rejects_duplicate_sampling_identity() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    states[2]["sample_index"] = states[1]["sample_index"]
    asset["state_records_sha256"] = canonical_sha256(states)

    with pytest.raises(ValueError, match="duplicate state identity"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_rejects_joint_value_hash_outside_frozen_sequence() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    states[3]["joint_values_sha256"] = "f" * 64
    asset["state_records_sha256"] = canonical_sha256(states)

    with pytest.raises(ValueError, match="frozen sampling sequence"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_rejects_metric_penetration_policy_drift() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    states[1]["metric_max_penetration_m"] = 0.5
    asset["state_records_sha256"] = canonical_sha256(states)

    with pytest.raises(ValueError, match="metric penetration policy"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_rejects_category_identity_drift() -> None:
    runner = load_runner()
    item, asset, states = table4_fixture()
    states[1]["category"] = "other"
    asset["state_records_sha256"] = canonical_sha256(states)

    with pytest.raises(ValueError, match="category mismatch"):
        runner.reaggregate_table4_asset(item, asset, states)


def test_reaggregation_keeps_empty_sweep_failed_when_asset_did_not_load() -> None:
    runner = load_runner()
    item, asset, _ = table4_fixture()
    item.update({
        "movable_dof_count": 0,
        "range_evaluable_dof_count": 0,
        "single_state_expected": 0,
        "sobol_state_expected": 0,
        "joint_specs": [],
        "joint_specs_sha256": canonical_sha256([]),
    })
    asset.update({
        "load_success": False,
        "rest_state_executed": 0,
        "rest_non_adjacent_free": 0,
        "rest_all_pair_cf": False,
        "rest_non_adjacent_cf": False,
        "single_state_expected": 0,
        "single_state_executed": 0,
        "single_non_adjacent_free": 0,
        "joint_single_sweep_cf_passed": 0,
        "single_joint_sweep_cf": False,
        "sobol_state_expected": 0,
        "sobol_state_executed": 0,
        "sobol_non_adjacent_free": 0,
        "multi_joint_sobol_cf": False,
        "measurement_complete": False,
        "strict_collision_pass": False,
        "state_records_sha256": canonical_sha256([]),
    })

    result = runner.reaggregate_table4_asset(item, asset, [])

    assert result["measurement_complete"] is False
    assert result["strict_collision_pass"] is False


def test_load_frozen_cohort_preserves_requested_table3_order() -> None:
    runner = load_runner()

    cohort = runner.load_frozen_cohort(limit=3)

    assert cohort["full_size"] == 800
    assert cohort["ordered_asset_keys_sha256"] == (
        "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
    )
    assert [row["selection_rank"] for row in cohort["records"]] == [1, 2, 3]
    assert [row["asset_key"] for row in cohort["records"]] == [
        "viable:objects/adjustable_wrench/adjustable_wrench_027",
        "viable:objects/cabinet_with_two_drawers/cabinet_with_two_drawers_007",
        "viable:objects/lever_style_nutcracker_with_mechanical/lever_style_nutcracker_with_mechanical_000",
    ]
    assert all(row["package"].is_dir() for row in cohort["records"])
    assert all(row["urdf_path"].is_file() for row in cohort["records"])


def test_resolve_release_package_rejects_root_and_ancestor_symlinks(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    release_root = tmp_path / "released_outputs"
    package = release_root / "objects/fixture/fixture_001"
    package.mkdir(parents=True)
    (package / "generated.urdf").write_text(
        '<robot name="fixture"><link name="base"/></robot>\n',
        encoding="utf-8",
    )

    root_link = release_root / "objects/fixture/fixture_link"
    root_link.symlink_to(package, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        runner.resolve_release_package(
            release_root,
            "objects/fixture/fixture_link",
        )

    ancestor_link = release_root / "objects_link"
    ancestor_link.symlink_to(release_root / "objects", target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        runner.resolve_release_package(
            release_root,
            "objects_link/fixture/fixture_001",
        )


def test_bind_table3_rows_rejects_asset_record_identity_drift() -> None:
    runner = load_runner()
    manifest_rows = [{
        "asset_key": "viable:objects/fixture/fixture_001",
        "category": "fixture",
        "declared_joint_count_hint": 1,
        "object_release_id": "fixture_001",
        "rel_path": "objects/fixture/fixture_001",
        "selection_hash": "1" * 64,
        "selection_rank": 1,
        "tier": "viable",
        "urdf_sha256": "2" * 64,
    }]
    asset_rows = [{
        **manifest_rows[0],
        "declared_joint_count": 1,
        "manifest_content_sha256": "3" * 64,
    }]
    asset_rows[0]["selection_hash"] = "4" * 64

    with pytest.raises(ValueError, match="selection_hash mismatch"):
        runner.bind_table3_rows(manifest_rows, asset_rows)


def test_load_table4_evidence_reaggregates_requested_cohort_prefix() -> None:
    runner = load_runner()
    cohort = runner.load_frozen_cohort(limit=3)

    evidence = runner.load_table4_evidence(cohort["records"])

    assert [row["strict_collision_pass"] for row in evidence["records"]] == [
        False,
        True,
        False,
    ]
    assert [row["state_record_count"] for row in evidence["records"]] == [
        0,
        149,
        86,
    ]
    assert evidence["verification_status"] == "PASS"
    assert evidence["manifest_file_sha256"] == (
        "8adc7d8698eaeab5ee5a62d881ed50d4e65c5dc80c9d1d8ae0f4a4a204474594"
    )
    assert evidence["state_records_file_sha256"] == (
        "ac62b73d71530982a63c1e8cf345cfda126608aa6e42ce9710383daace2af257"
    )


def s1_record(*, strict_pass: bool, eligible_pairs: int) -> dict:
    return {
        "status": "completed",
        "binding": {"verified": True, "issues": []},
        "strict_pass_no_method_allowance": strict_pass,
        "registered_allowance_strict_pass": strict_pass,
        "deterministic_rebuild_match": None,
        "rebuild_replay_status": "N/E",
        "s1_evidence": {
            "receipt": {"receipt_bound_asset": 0},
            "receipt_replay": {"passed": False, "status": "NO_VALID_RECEIPT"},
            "rebuild": {"eligible_asset": 0, "status": "N/E"},
            "allowance": {
                "status": "COMPLETE",
                "registered_excluded_pair_count": 0,
                "eligible_nonadjacent_pair_count": eligible_pairs,
            },
        },
    }


def test_aggregate_s1_applies_fail_closed_denominators() -> None:
    runner = load_runner()
    records = [
        s1_record(strict_pass=True, eligible_pairs=3),
        s1_record(strict_pass=False, eligible_pairs=1),
    ]

    metrics = runner.aggregate_s1(records, intended_assets=2)

    assert metrics["receipt_bound_assets"] == {
        "passed": 0,
        "denominator": 2,
        "rate": 0.0,
        "percentage": 0.0,
    }
    assert metrics["receipt_replay_pass"]["passed"] == 0
    assert metrics["deterministic_rebuild_match"] == {
        "status": "N/E",
        "passed": None,
        "denominator": 0,
        "rate": None,
        "percentage": None,
        "eligible_assets": 0,
        "asset_denominator": 2,
    }
    assert metrics["allowance_density"] == {
        "status": "COMPLETE",
        "registered_pairs": 0,
        "eligible_pairs": 4,
        "rate": 0.0,
        "percentage": 0.0,
        "measured_assets": 2,
        "intended_assets": 2,
    }
    assert metrics["strict_pass_no_method_allowance"]["passed"] == 1
    assert metrics["registered_allowance_gain_pp"] == {
        "status": "COMPLETE",
        "value": 0.0,
        "registered_passed": 1,
        "no_allowance_passed": 1,
        "denominator": 2,
    }


def test_aggregate_s1_does_not_invent_gain_for_unreplayed_allowance() -> None:
    runner = load_runner()
    record = s1_record(strict_pass=False, eligible_pairs=3)
    record["s1_evidence"]["allowance"]["registered_excluded_pair_count"] = 1
    record["registered_allowance_strict_pass"] = None

    gain = runner.aggregate_s1([record], intended_assets=1)[
        "registered_allowance_gain_pp"
    ]

    assert gain["status"] == "NOT_EVALUABLE"
    assert gain["value"] is None


def test_zero_allowance_candidates_keep_registered_variant_identical() -> None:
    runner = load_runner()
    topology_not_evaluable = {
        "status": "NOT_EVALUABLE",
        "candidate_file_count": 0,
        "registered_excluded_pair_count": None,
    }

    assert runner.registered_allowance_outcome(True, topology_not_evaluable) is True
    assert runner.registered_allowance_outcome(False, topology_not_evaluable) is False


def test_aggregate_s1_reports_zero_pair_allowance_denominator_as_not_evaluable() -> None:
    runner = load_runner()

    allowance = runner.aggregate_s1(
        [s1_record(strict_pass=True, eligible_pairs=0)],
        intended_assets=1,
    )["allowance_density"]

    assert allowance["status"] == "N/E"
    assert allowance["registered_pairs"] == 0
    assert allowance["eligible_pairs"] == 0
    assert allowance["rate"] is None
    assert allowance["percentage"] is None

    metrics = runner.aggregate_s1(
        [s1_record(strict_pass=True, eligible_pairs=0)],
        intended_assets=1,
    )
    markdown = runner.render_summary({
        "dataset": "LAM released outputs",
        "protocol_id": "fixture",
        "classification": "SMOKE",
        "n_eval": 1,
        "status_counts": {"completed": 1},
        "metrics": metrics,
    })
    assert "| Allowance Density | 0 / 0 (N/E) |" in markdown


def test_aggregate_s1_treats_missing_binding_as_ineligible() -> None:
    runner = load_runner()
    record = s1_record(strict_pass=True, eligible_pairs=3)
    record.pop("binding")

    metrics = runner.aggregate_s1([record], intended_assets=1)

    assert metrics["strict_pass_no_method_allowance"]["passed"] == 0
    assert metrics["allowance_density"]["status"] == "PARTIAL"
    markdown = runner.render_summary({
        "dataset": "LAM released outputs",
        "protocol_id": "fixture",
        "classification": "SMOKE",
        "n_eval": 1,
        "status_counts": {"completed": 1},
        "metrics": metrics,
    })
    assert "| Allowance Density | 0 / 0 (PARTIAL; 0 / 1 assets) |" in markdown


def test_evaluate_asset_rescans_bound_package_and_static_s1_evidence(
    tmp_path: Path,
) -> None:
    runner = load_runner()
    package = tmp_path / "released_outputs/objects/fixture/fixture_001"
    package.mkdir(parents=True)
    urdf = package / "generated.urdf"
    urdf.write_text(
        """
<robot name="fixture">
  <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="middle"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="tip"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="middle"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="mount" type="fixed"><parent link="middle"/><child link="tip"/></joint>
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )
    binding = runner.package_binding(package)
    source = {
        "selection_index": 0,
        "asset_key": "viable:objects/fixture/fixture_001",
        "selection_rank": 1,
        "selection_hash": "1" * 64,
        "tier": "viable",
        "rel_path": "objects/fixture/fixture_001",
        "object_release_id": "fixture_001",
        "category": "fixture",
        "declared_joint_count_hint": 1,
        "urdf_sha256": runner.sha256_file(urdf),
        "package": package,
        "urdf_path": urdf,
    }
    table4_item = {
        "package_binding": binding,
        "package_content_manifest_sha256": binding["content_manifest_sha256"],
        "model_urdf_sha256": source["urdf_sha256"],
        "input_identity_sha256": "2" * 64,
    }
    table4_result = {
        "strict_collision_pass": True,
        "measurement_complete": True,
        "state_record_count": 86,
        "state_records_sha256": "3" * 64,
        "table4_asset_record_sha256": "4" * 64,
    }

    record = runner.evaluate_asset(source, table4_item, table4_result)

    assert record["status"] == "completed"
    assert record["binding"] == {"verified": True, "issues": []}
    assert record["s1_evidence"]["receipt"]["receipt_bound_asset"] == 0
    assert record["s1_evidence"]["receipt_replay"]["passed"] is False
    assert record["s1_evidence"]["rebuild"]["eligible_asset"] == 0
    assert record["s1_evidence"]["allowance"][
        "eligible_nonadjacent_pair_count"
    ] == 1
    assert record["s1_evidence"]["allowance"][
        "registered_excluded_pair_count"
    ] == 0
    assert record["strict_pass_no_method_allowance"] is True
    assert record["registered_allowance_strict_pass"] is True


def test_cli_contract_fixes_formal_denominator_and_worker_count() -> None:
    runner = load_runner()

    formal = runner.parse_args(["--mode", "formal"])
    runner.validate_args(formal)
    assert runner.requested_n(formal) == 800
    assert formal.workers == 4

    with pytest.raises(ValueError, match="formal mode requires n=800"):
        runner.validate_args(
            runner.parse_args(["--mode", "formal", "--n", "5"])
        )
    with pytest.raises(ValueError, match="formal mode requires workers=4"):
        runner.validate_args(
            runner.parse_args(["--mode", "formal", "--workers", "2"])
        )
    with pytest.raises(ValueError, match="requires independent verification"):
        runner.validate_args(
            runner.parse_args(["--mode", "formal", "--skip-verify"])
        )


def test_formal_api_rejects_bundle_without_independent_verifier(tmp_path: Path) -> None:
    runner = load_runner()

    with pytest.raises(ValueError, match="requires independent verification"):
        runner.run_evaluation(
            mode="formal",
            n=800,
            workers=4,
            output=tmp_path / "formal_without_verifier",
            run_verifier=False,
        )


def test_cli_contract_allows_only_a_frozen_smoke_prefix() -> None:
    runner = load_runner()
    smoke = runner.parse_args(
        ["--mode", "smoke", "--n", "5", "--workers", "2"]
    )

    runner.validate_args(smoke)

    assert runner.requested_n(smoke) == 5
    with pytest.raises(ValueError, match="smoke mode requires --n"):
        runner.validate_args(runner.parse_args(["--mode", "smoke"]))


def test_smoke_run_writes_replayable_artifact_bundle(tmp_path: Path) -> None:
    runner = load_runner()
    output = tmp_path / "s1_lam_smoke_n2"

    result = runner.run_evaluation(
        mode="smoke",
        n=2,
        workers=2,
        output=output,
        run_verifier=False,
    )

    assert result["status"] == "completed"
    assert result["output"] == output
    expected_files = {
        "asset_records.jsonl",
        "environment.json",
        "frozen_config.json",
        "manifest.json",
        "protocol_snapshot.md",
        "summary.json",
        "summary.md",
    }
    assert expected_files <= {path.name for path in output.iterdir()}
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["classification"] == "SMOKE"
    assert summary["n_eval"] == 2
    assert summary["metrics"]["receipt_bound_assets"]["passed"] == 0
    assert summary["metrics"]["strict_pass_no_method_allowance"]["passed"] == 1
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    assert [row["selection_rank"] for row in rows] == [1, 2]
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    declared = manifest.pop("manifest_content_sha256")
    assert declared == runner.canonical_sha256(manifest)


def test_run_evaluation_rejects_symlinked_output_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = load_runner()
    target = tmp_path / "target"
    target.mkdir()
    output = tmp_path / "output_link"
    output.symlink_to(target, target_is_directory=True)

    def must_not_load_cohort(*, limit):
        raise AssertionError(f"cohort should not be loaded, limit={limit}")

    monkeypatch.setattr(runner, "load_frozen_cohort", must_not_load_cohort)
    with pytest.raises(ValueError, match="output directory is a symlink"):
        runner.run_evaluation(
            mode="smoke",
            n=1,
            workers=1,
            output=output,
            run_verifier=False,
        )
