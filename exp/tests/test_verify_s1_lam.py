from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
VERIFIER = REPO / "exp/scripts/verify_s1_lam.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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


def resign_artifacts(verifier, output: Path, names: list[str]) -> None:
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for name in names:
        path = output / name
        manifest["artifacts"][name] = {
            "bytes": path.stat().st_size,
            "sha256": verifier.sha256_file(path),
        }
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = verifier.canonical_sha256(manifest)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def test_verifier_recomputes_six_metrics_independently() -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_metrics_under_test")
    records = [
        s1_record(strict_pass=True, eligible_pairs=3),
        s1_record(strict_pass=False, eligible_pairs=1),
    ]

    metrics = verifier.recompute_metrics(records, intended_assets=2)

    assert metrics["receipt_bound_assets"]["passed"] == 0
    assert metrics["receipt_replay_pass"]["passed"] == 0
    assert metrics["deterministic_rebuild_match"]["status"] == "N/E"
    assert metrics["allowance_density"]["eligible_pairs"] == 4
    assert metrics["strict_pass_no_method_allowance"]["passed"] == 1
    assert metrics["registered_allowance_gain_pp"]["value"] == 0.0


def test_verifier_reports_zero_pair_allowance_denominator_as_not_evaluable() -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_zero_pair_metrics_test")

    allowance = verifier.recompute_metrics(
        [s1_record(strict_pass=True, eligible_pairs=0)],
        intended_assets=1,
    )["allowance_density"]

    assert allowance["status"] == "N/E"
    assert allowance["rate"] is None
    assert allowance["percentage"] is None


def test_verifier_metrics_treat_missing_binding_as_ineligible() -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_missing_binding_test")
    record = s1_record(strict_pass=True, eligible_pairs=3)
    record.pop("binding")

    metrics = verifier.recompute_metrics([record], intended_assets=1)

    assert metrics["strict_pass_no_method_allowance"]["passed"] == 0
    assert metrics["allowance_density"]["status"] == "PARTIAL"


def test_verifier_zero_candidates_keep_registered_variant_identical() -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_registered_variant_test")
    topology_not_evaluable = {
        "status": "NOT_EVALUABLE",
        "candidate_file_count": 0,
        "registered_excluded_pair_count": None,
    }

    assert verifier.registered_allowance_outcome(True, topology_not_evaluable) is True
    assert verifier.registered_allowance_outcome(False, topology_not_evaluable) is False


def test_verifier_independently_scans_topology_and_evidence_names(
    tmp_path: Path,
) -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_audit_under_test")
    package = tmp_path / "released_outputs/objects/fixture/fixture_001"
    package.mkdir(parents=True)
    (package / "generated.urdf").write_text(
        """
<robot name="fixture">
  <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="middle"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="tip"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="hinge" type="revolute"><parent link="base"/><child link="middle"/></joint>
  <joint name="mount" type="fixed"><parent link="middle"/><child link="tip"/></joint>
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (package / "workflow.json").write_text("{}\n", encoding="utf-8")
    (package / "export.js").write_text("export {};\n", encoding="utf-8")
    (package / "generation_config.yaml").write_text(
        "private: do-not-parse\n", encoding="utf-8"
    )
    (package / "mechanical_receipt.json").write_text("{}\n", encoding="utf-8")
    (package / "rebuild_recipe.json").write_text("{}\n", encoding="utf-8")
    (package / "collision_allowance.json").write_text("{}\n", encoding="utf-8")

    audit = verifier.audit_package(package)

    assert audit["eligible_nonadjacent_pair_count"] == 1
    assert audit["receipt_candidate_count"] == 1
    assert audit["receipt_candidates"] == ["mechanical_receipt.json"]
    assert audit["rebuild_recipe_candidate_count"] == 1
    assert audit["rebuild_recipe_candidates"] == ["rebuild_recipe.json"]
    assert audit["allowance_candidate_count"] == 1
    assert audit["allowance_candidates"] == ["collision_allowance.json"]
    assert audit["generation_config_count"] == 1


def test_verifier_rank6_preserves_topology_not_evaluable_allowance() -> None:
    verifier = load_module(VERIFIER, "s1_lam_verifier_rank6_topology_test")
    package = verifier.RELEASE_ROOT / "imperfect/laptop/laptop_001"

    audit = verifier.audit_package(package)

    assert audit["topology_issues"] == ["joint_parent_child_invalid"]
    assert audit["s1_evidence"]["allowance"] == {
        "status": "NOT_EVALUABLE",
        "candidate_file_count": 0,
        "valid_file_count": 0,
        "registered_excluded_pair_count": None,
        "eligible_nonadjacent_pair_count": None,
        "records": [],
        "issues": ["joint_parent_child_invalid"],
    }


def test_verifier_rejects_aggregate_preserving_strict_swap(tmp_path: Path) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_verifier_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_full_under_test")
    output = tmp_path / "s1_lam_smoke_n2"
    runner.run_evaluation(
        mode="smoke",
        n=2,
        workers=1,
        output=output,
        run_verifier=False,
    )
    baseline = verifier.verify_run(output, formal=False, write_result=False)
    assert baseline["status"] == "PASS"

    records_path = output / "asset_records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines()]
    assert [row["strict_pass_no_method_allowance"] for row in rows] == [False, True]
    rows[0]["strict_pass_no_method_allowance"] = True
    rows[0]["registered_allowance_strict_pass"] = True
    rows[1]["strict_pass_no_method_allowance"] = False
    rows[1]["registered_allowance_strict_pass"] = False
    records_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"]["asset_records.jsonl"] = {
        "bytes": records_path.stat().st_size,
        "sha256": verifier.sha256_file(records_path),
    }
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = verifier.canonical_sha256(manifest)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    tampered = verifier.verify_run(output, formal=False, write_result=False)

    assert tampered["status"] == "FAIL"
    assert tampered["checks"]["atomic_table4_projection"] is False


def test_verifier_rejects_re_manifested_static_provenance_tamper(
    tmp_path: Path,
) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_static_tamper_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_static_tamper_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    baseline = verifier.verify_run(output, formal=False, write_result=False)
    assert baseline["status"] == "PASS"

    records_path = output / "asset_records.jsonl"
    rows = [
        json.loads(line)
        for line in records_path.read_text(encoding="utf-8").splitlines()
    ]
    rows[0]["resource_closure"]["sha256"] = "0" * 64
    records_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"]["asset_records.jsonl"] = {
        "bytes": records_path.stat().st_size,
        "sha256": verifier.sha256_file(records_path),
    }
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = verifier.canonical_sha256(manifest)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    tampered = verifier.verify_run(output, formal=False, write_result=False)

    assert tampered["status"] == "FAIL"
    assert tampered["checks"]["atomic_static_evidence"] is False


def test_verifier_rejects_re_manifested_frozen_config_tamper(tmp_path: Path) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_config_tamper_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_config_tamper_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    config_path = output / "frozen_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["workers"] = 99
    config["privacy"]["released_code_execution"] = "enabled"
    config_path.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    resign_artifacts(verifier, output, ["frozen_config.json"])

    tampered = verifier.verify_run(output, formal=False, write_result=False)

    assert tampered["status"] == "FAIL"
    assert tampered["checks"]["frozen_config"] is False


def test_verifier_rejects_re_manifested_protocol_snapshot_tamper(
    tmp_path: Path,
) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_snapshot_tamper_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_snapshot_tamper_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    snapshot_path = output / "protocol_snapshot.md"
    snapshot_path.write_text(
        snapshot_path.read_text(encoding="utf-8") + "\nTAMPERED\n",
        encoding="utf-8",
    )
    config_path = output / "frozen_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["protocol_snapshot_sha256"] = verifier.sha256_file(snapshot_path)
    config_path.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    resign_artifacts(
        verifier,
        output,
        ["protocol_snapshot.md", "frozen_config.json"],
    )

    tampered = verifier.verify_run(output, formal=False, write_result=False)

    assert tampered["status"] == "FAIL"
    assert tampered["checks"]["protocol_snapshot"] is False


def test_verifier_rejects_re_manifested_environment_tamper(tmp_path: Path) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_environment_tamper_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_environment_tamper_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    environment_path = output / "environment.json"
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    environment["python"] = "forged-python"
    environment["platform"] = "forged-platform"
    environment["thread_environment"]["OMP_NUM_THREADS"] = "999"
    environment_path.write_text(
        json.dumps(environment, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    resign_artifacts(verifier, output, ["environment.json"])

    tampered = verifier.verify_run(output, formal=False, write_result=False)

    assert tampered["status"] == "FAIL"
    assert tampered["checks"]["environment_binding"] is False


def test_verifier_overwrites_stale_pass_for_malformed_nested_atom(
    tmp_path: Path,
) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_malformed_atom_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_malformed_atom_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    baseline = verifier.verify_run(output, formal=False, write_result=True)
    assert baseline["status"] == "PASS"
    assert json.loads((output / "verification.json").read_text(encoding="utf-8"))[
        "status"
    ] == "PASS"

    records_path = output / "asset_records.jsonl"
    rows = [
        json.loads(line)
        for line in records_path.read_text(encoding="utf-8").splitlines()
    ]
    rows[0]["s1_evidence"]["receipt"] = []
    records_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )
    resign_artifacts(verifier, output, ["asset_records.jsonl"])

    malformed = verifier.verify_run(output, formal=False, write_result=True)

    assert malformed["status"] == "FAIL"
    receipt = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    assert receipt["status"] == "FAIL"


def test_verifier_writes_fail_receipt_for_out_of_range_n_eval(tmp_path: Path) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_invalid_n_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_invalid_n_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["n_eval"] = 801
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    resign_artifacts(verifier, output, ["summary.json"])

    result = verifier.verify_run(output, formal=False, write_result=True)

    assert result["status"] == "FAIL"
    assert result["checks"]["n_eval_range"] is False
    receipt = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    assert receipt["status"] == "FAIL"


def test_verifier_rejects_formal_bundle_checked_without_formal_mode(
    tmp_path: Path,
) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_mode_downgrade_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_mode_downgrade_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["classification"] = "FORMAL"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    config_path = output / "frozen_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["classification"] = "FORMAL"
    config_path.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["classification"] = "FORMAL"
    for name in ("summary.json", "frozen_config.json"):
        path = output / name
        manifest["artifacts"][name] = {
            "bytes": path.stat().st_size,
            "sha256": verifier.sha256_file(path),
        }
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = verifier.canonical_sha256(manifest)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    downgraded = verifier.verify_run(output, formal=False, write_result=False)

    assert downgraded["status"] == "FAIL"
    assert downgraded["checks"]["verification_mode"] is False


def test_verifier_rejects_symlinked_or_unbound_output_artifacts(
    tmp_path: Path,
) -> None:
    runner = load_module(
        REPO / "exp/scripts/run_s1_lam.py",
        "s1_lam_runner_for_output_symlink_test",
    )
    verifier = load_module(VERIFIER, "s1_lam_verifier_output_symlink_test")
    output = tmp_path / "s1_lam_smoke_n1"
    runner.run_evaluation(
        mode="smoke",
        n=1,
        workers=1,
        output=output,
        run_verifier=False,
    )
    summary_markdown = output / "summary.md"
    backing = tmp_path / "summary_backing.md"
    backing.write_bytes(summary_markdown.read_bytes())
    summary_markdown.unlink()
    summary_markdown.symlink_to(backing)
    (output / "unbound.txt").write_text("unbound\n", encoding="utf-8")

    result = verifier.verify_run(output, formal=False, write_result=False)

    assert result["status"] == "FAIL"
    assert result["checks"]["required_artifacts"] is False
    assert result["checks"]["artifact_set"] is False
