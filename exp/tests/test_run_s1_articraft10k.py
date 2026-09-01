from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_s1_articraft10k.py"
VERIFIER = REPO / "exp/scripts/verify_s1_articraft10k.py"


def load_runner():
    assert RUNNER.is_file(), "Articraft-10K S1 runner has not been implemented"
    spec = importlib.util.spec_from_file_location("s1_articraft10k", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_verifier():
    assert VERIFIER.is_file(), "Articraft-10K S1 verifier has not been implemented"
    spec = importlib.util.spec_from_file_location("verify_s1_articraft10k", VERIFIER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def rebind_asset_records_artifact(output: Path, rows: list[dict[str, object]]) -> None:
    asset_records = output / "asset_records.jsonl"
    asset_records.write_text(
        "".join(
            json.dumps(
                row,
                sort_keys=True,
                ensure_ascii=True,
                allow_nan=False,
            )
            + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"]["asset_records.jsonl"] = {
        "bytes": asset_records.stat().st_size,
        "sha256": sha256_file(asset_records),
    }
    manifest.pop("manifest_content_sha256")
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def package_binding(package: Path) -> dict[str, object]:
    files: list[dict[str, object]] = []
    for current_raw, directories, names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directories.sort()
        names.sort()
        for name in names:
            path = current / name
            files.append(
                {
                    "path": path.relative_to(package).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def write_cohort_fixture(tmp_path: Path) -> dict[str, Path]:
    dataset_root = tmp_path / "Articraft-10K"
    release_root = dataset_root / "released_urdf"
    category_records_root = tmp_path / "official" / "records"
    records: list[dict[str, object]] = []
    urdf = """<robot name="fixture">
<link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="middle"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="tip"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<joint name="first" type="fixed"><parent link="base"/><child link="middle"/></joint>
<joint name="second" type="revolute"><parent link="middle"/><child link="tip"/>
<limit lower="0" upper="1" effort="1" velocity="1"/></joint>
</robot>
"""
    for index, asset_id in enumerate(("rec_second", "rec_first")):
        package = release_root / asset_id
        package.mkdir(parents=True)
        (package / "model.urdf").write_text(urdf, encoding="utf-8")
        (package / "compile_report.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "record_id": asset_id,
                    "status": "success",
                    "overlap_allowances": [],
                    "metrics": {
                        "fingerprint_inputs": {
                            "model_py_sha256": str(index + 1) * 64,
                            "sdk_fingerprint": None,
                        }
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        record_root = category_records_root / asset_id
        record_root.mkdir(parents=True)
        (record_root / "record.json").write_text(
            json.dumps(
                {
                    "record_id": asset_id,
                    "active_revision_id": "rev_000001",
                    "artifacts": {
                        "model_py": "revisions/rev_000001/model.py",
                        "prompt_txt": "revisions/rev_000001/prompt.txt",
                    },
                    "hashes": {"model_py_sha256": str(index + 1) * 64},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        binding = package_binding(package)
        records.append(
            {
                "asset_id": asset_id,
                "selection_index": index,
                "package": str(package),
                "model_urdf_sha256": sha256_file(package / "model.urdf"),
                "package_binding": binding,
            }
        )
    manifest = {
        "schema_version": "1.0.0",
        "dataset": "Articraft-10K",
        "mode": "formal",
        "classification": "FORMAL",
        "records": records,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return {
        "dataset_root": dataset_root,
        "release_root": release_root,
        "category_records_root": category_records_root,
        "manifest": manifest_path,
    }


def test_load_cohort_preserves_manifest_order_and_rejects_package_drift(
    tmp_path: Path,
) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()

    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )

    assert [row["asset_id"] for row in cohort["records"]] == [
        "rec_second",
        "rec_first",
    ]
    assert [row["selection_index"] for row in cohort["records"]] == [0, 1]

    drift = fixture["release_root"] / "rec_first" / "undeclared.txt"
    drift.write_text("drift\n", encoding="utf-8")
    with pytest.raises(ValueError, match="package binding mismatch"):
        runner.load_cohort(
            fixture["manifest"], fixture["dataset_root"], formal=False
        )


def test_cli_contract_fixes_formal_denominator_and_allows_smoke_prefix(
    tmp_path: Path,
) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    formal = runner.parse_args(["--mode", "formal"])
    runner.validate_args(formal)
    assert runner.requested_n(formal) == 800
    with pytest.raises(ValueError, match="formal mode requires n=800"):
        runner.validate_args(
            runner.parse_args(["--mode", "formal", "--n", "5"])
        )
    with pytest.raises(ValueError, match="formal mode requires workers=4"):
        runner.validate_args(
            runner.parse_args(["--mode", "formal", "--workers", "1"])
        )
    smoke = runner.parse_args(
        ["--mode", "smoke", "--n", "1", "--workers", "1"]
    )
    runner.validate_args(smoke)
    cohort = runner.load_cohort(
        fixture["manifest"],
        fixture["dataset_root"],
        formal=False,
        limit=runner.requested_n(smoke),
    )
    assert [row["asset_id"] for row in cohort["records"]] == ["rec_second"]


def test_release_evidence_excludes_compile_report_receipt_and_hash_only_rebuild(
    tmp_path: Path,
) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )

    evidence = runner.audit_release_evidence(
        cohort["records"][0], fixture["category_records_root"]
    )

    assert evidence["receipt"]["candidate_count"] == 0
    assert evidence["receipt"]["receipt_bound_asset"] is False
    assert evidence["compile_report"]["mechanical_receipt"] is False
    assert evidence["compile_report"]["overlap_allowances"] == []
    assert evidence["rebuild"]["status"] == "N/E"
    assert evidence["rebuild"]["eligible_asset"] is False
    assert evidence["rebuild"]["official_model_py"]["declared"] is True
    assert evidence["rebuild"]["official_model_py"]["exists"] is False
    assert evidence["allowance"]["status"] == "COMPLETE"
    assert evidence["allowance"]["eligible_nonadjacent_pair_count"] == 1
    assert evidence["allowance"]["registered_excluded_pair_count"] == 0
    assert evidence["allowance"]["registry_sources"] == ["compile_report.json"]


def test_compile_report_allowance_must_be_an_eligible_pair(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    package = Path(cohort["records"][0]["package"])
    report_path = package / "compile_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["overlap_allowances"] = [["base", "middle"]]
    report_path.write_text(json.dumps(report) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="compile report allowance pair is not eligible"):
        runner.audit_release_evidence(
            cohort["records"][0], fixture["category_records_root"]
        )


def test_allowance_pairs_are_deduplicated_across_registry_sources(
    tmp_path: Path,
) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    package = Path(cohort["records"][0]["package"])
    report_path = package / "compile_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["overlap_allowances"] = [["base", "tip"]]
    report_path.write_text(json.dumps(report) + "\n", encoding="utf-8")
    (package / "collision_allowances.json").write_text(
        json.dumps({"excluded_non_adjacent_pairs": [["base", "tip"]]}) + "\n",
        encoding="utf-8",
    )

    evidence = runner.audit_release_evidence(
        cohort["records"][0], fixture["category_records_root"]
    )

    assert evidence["allowance"]["registered_excluded_pair_count"] == 1
    assert evidence["allowance"]["registry_sources"] == [
        "compile_report.json",
        "collision_allowances.json",
    ]


def write_table4_fixture(
    tmp_path: Path,
    cohort_records: list[dict[str, object]],
) -> dict[str, Path]:
    protocol_id = "fixture-table4"
    items: list[dict[str, object]] = []
    asset_records: list[dict[str, object]] = []
    state_records: list[dict[str, object]] = []
    for index, cohort_record in enumerate(cohort_records):
        asset_id = str(cohort_record["asset_id"])
        items.append(
            {
                "protocol_id": protocol_id,
                "asset_id": asset_id,
                "selection_index": index,
                "order": index,
                "model_urdf_sha256": cohort_record["model_urdf_sha256"],
                "package_content_manifest_sha256": cohort_record["package_binding"][
                    "content_manifest_sha256"
                ],
                "movable_dof_count": 1,
                "range_evaluable_dof_count": 1,
                "rest_state_expected": 1,
                "single_state_expected": 2,
                "sobol_state_expected": 2,
            }
        )
        rows: list[dict[str, object]] = []
        for phase, count in (
            ("rest", 1),
            ("single_joint_sweep", 2),
            ("multi_joint_sobol", 2),
        ):
            for sample_index in range(count):
                illegal = int(index == 1 and phase == "multi_joint_sobol" and sample_index == 1)
                rows.append(
                    {
                        "protocol_id": protocol_id,
                        "asset_id": asset_id,
                        "selection_index": index,
                        "order": index,
                        "phase": phase,
                        "sample_index": sample_index,
                        "non_adjacent_illegal_penetration_count": illegal,
                    }
                )
        state_records.extend(rows)
        strict = index == 0
        asset_records.append(
            {
                "protocol_id": protocol_id,
                "asset_id": asset_id,
                "selection_index": index,
                "order": index,
                "model_urdf_sha256": cohort_record["model_urdf_sha256"],
                "package_content_manifest_sha256": cohort_record["package_binding"][
                    "content_manifest_sha256"
                ],
                "movable_dof_count": 1,
                "range_evaluable_dof_count": 1,
                "rest_state_expected": 1,
                "rest_state_executed": 1,
                "rest_non_adjacent_free": 1,
                "rest_non_adjacent_cf": True,
                "single_state_expected": 2,
                "single_state_executed": 2,
                "single_non_adjacent_free": 2,
                "single_joint_sweep_cf": True,
                "sobol_state_expected": 2,
                "sobol_state_executed": 2,
                "sobol_non_adjacent_free": 2 if strict else 1,
                "multi_joint_sobol_cf": strict,
                "measurement_complete": True,
                "strict_collision_pass": strict,
                "state_records_sha256": canonical_sha256(rows),
            }
        )
    manifest = {
        "protocol_id": protocol_id,
        "sample_size": len(items),
        "items": items,
    }
    manifest_path = tmp_path / "table4_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    asset_path = tmp_path / "table4_assets.json"
    asset_path.write_text(json.dumps(asset_records, indent=2) + "\n", encoding="utf-8")
    state_path = tmp_path / "table4_states.jsonl"
    state_path.write_text(
        "\n".join(json.dumps(row) for row in state_records) + "\n",
        encoding="utf-8",
    )
    return {
        "manifest": manifest_path,
        "assets": asset_path,
        "states": state_path,
    }


def test_table4_strict_pass_is_reaggregated_from_raw_states(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])

    evidence = runner.load_table4_evidence(
        table4["manifest"],
        table4["assets"],
        table4["states"],
        cohort,
        formal=False,
    )

    assert evidence["strict_passed"] == 1
    assert evidence["denominator"] == 2
    assert [row["strict_collision_pass"] for row in evidence["records"]] == [
        True,
        False,
    ]
    assert evidence["state_record_count"] == 10


def test_table4_state_tampering_is_rejected(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])
    rows = [json.loads(line) for line in table4["states"].read_text().splitlines()]
    rows[0]["non_adjacent_illegal_penetration_count"] = 1
    table4["states"].write_text(
        "\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8"
    )

    with pytest.raises(ValueError, match="state records SHA256 mismatch"):
        runner.load_table4_evidence(
            table4["manifest"],
            table4["assets"],
            table4["states"],
            cohort,
            formal=False,
        )


def test_s1_asset_records_and_aggregate_keep_fail_closed_denominators(
    tmp_path: Path,
) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4_paths = write_table4_fixture(tmp_path, cohort["records"])
    table4 = runner.load_table4_evidence(
        table4_paths["manifest"],
        table4_paths["assets"],
        table4_paths["states"],
        cohort,
        formal=False,
    )
    records = []
    for source, table4_record in zip(
        cohort["records"], table4["records"], strict=True
    ):
        release_evidence = runner.audit_release_evidence(
            source, fixture["category_records_root"]
        )
        records.append(
            runner.build_s1_asset_record(source, release_evidence, table4_record)
        )

    summary = runner.aggregate_s1(records)

    assert summary["receipt_bound_assets"] == {
        "passed": 0,
        "denominator": 2,
        "rate": 0.0,
    }
    assert summary["receipt_replay_pass"] == {
        "passed": 0,
        "denominator": 2,
        "rate": 0.0,
    }
    assert summary["deterministic_rebuild_match"]["status"] == "N/E"
    assert summary["deterministic_rebuild_match"]["denominator"] == 0
    assert summary["deterministic_rebuild_match"]["asset_denominator"] == 2
    assert summary["allowance_density"] == {
        "registered_pairs": 0,
        "eligible_pairs": 2,
        "rate": 0.0,
    }
    assert summary["strict_pass_no_method_allowance"] == {
        "passed": 1,
        "denominator": 2,
        "rate": 0.5,
    }
    assert summary["registered_allowance_gain_pp"] == {
        "value": 0.0,
        "registered_passed": 1,
        "no_allowance_passed": 1,
        "denominator": 2,
    }


def test_nonempty_allowance_requires_pair_specific_replay(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    source = cohort["records"][0]
    evidence = runner.audit_release_evidence(
        source, fixture["category_records_root"]
    )
    evidence["allowance"]["registered_excluded_pair_count"] = 1
    table4_record = {
        "asset_id": source["asset_id"],
        "selection_index": source["selection_index"],
        "strict_collision_pass": True,
        "measurement_complete": True,
        "state_record_count": 5,
        "state_records_sha256": "a" * 64,
        "table4_asset_record_sha256": "b" * 64,
    }

    with pytest.raises(
        ValueError, match="non-empty registered allowance requires pair-specific replay"
    ):
        runner.build_s1_asset_record(source, evidence, table4_record)


def test_run_evaluation_writes_replayable_hash_bound_artifacts(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# Frozen S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "run"

    summary = runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        category_records_root=fixture["category_records_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=2,
    )

    assert summary["metrics"]["strict_pass_no_method_allowance"]["passed"] == 1
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    assert [row["asset_id"] for row in rows] == ["rec_second", "rec_first"]
    assert (output / "protocol_snapshot.md").read_text() == protocol.read_text()
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["artifacts"]["asset_records.jsonl"]["sha256"] == sha256_file(
        output / "asset_records.jsonl"
    )
    content_hash = manifest.pop("manifest_content_sha256")
    assert content_hash == canonical_sha256(manifest)
    config = json.loads((output / "frozen_config.json").read_text())
    assert config["code_identity"]["verifier_sha256"] == sha256_file(VERIFIER)


def test_smoke_run_uses_same_prefix_across_cohort_and_table4(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# Frozen S1 fixture protocol\n", encoding="utf-8")

    summary = runner.run_evaluation(
        output=tmp_path / "smoke",
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        category_records_root=fixture["category_records_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=1,
        limit=1,
    )

    assert summary["n_eval"] == 1
    assert summary["metrics"]["strict_pass_no_method_allowance"]["passed"] == 1


def test_independent_verifier_accepts_smoke_prefix(tmp_path: Path) -> None:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    verifier = load_verifier()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# Frozen S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "smoke"
    runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        category_records_root=fixture["category_records_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=1,
        limit=1,
    )

    verification = verifier.verify_run(output, formal=False)

    assert verification["all_pass"] is True


def write_s1_run_fixture(tmp_path: Path) -> Path:
    fixture = write_cohort_fixture(tmp_path)
    runner = load_runner()
    cohort = runner.load_cohort(
        fixture["manifest"], fixture["dataset_root"], formal=False
    )
    table4 = write_table4_fixture(tmp_path, cohort["records"])
    protocol = tmp_path / "protocol.md"
    protocol.write_text("# Frozen S1 fixture protocol\n", encoding="utf-8")
    output = tmp_path / "run"
    runner.run_evaluation(
        output=output,
        cohort_manifest=fixture["manifest"],
        dataset_root=fixture["dataset_root"],
        category_records_root=fixture["category_records_root"],
        table4_manifest=table4["manifest"],
        table4_asset_records=table4["assets"],
        table4_state_records=table4["states"],
        protocol_document=protocol,
        formal=False,
        workers=2,
    )
    return output


def test_independent_verifier_accepts_complete_run(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()

    verification = verifier.verify_run(output, formal=False)

    assert verification["all_pass"] is True
    assert verification["check_count"] >= 8
    assert all(check["pass"] for check in verification["checks"])


def test_independent_verifier_rejects_artifact_tampering(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    with (output / "asset_records.jsonl").open("a", encoding="utf-8") as handle:
        handle.write("\n")

    with pytest.raises(verifier.VerificationError, match="artifact SHA256 mismatch"):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_table4_source_tampering(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    with (tmp_path / "table4_states.jsonl").open("a", encoding="utf-8") as handle:
        handle.write("\n")

    with pytest.raises(
        verifier.VerificationError, match="Table 4 state records SHA256 mismatch"
    ):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_aggregate_preserving_table4_flag_swap(
    tmp_path: Path,
) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    for field in (
        "strict_collision_pass_no_method_allowance",
        "strict_collision_pass_registered_allowance",
    ):
        rows[0][field], rows[1][field] = rows[1][field], rows[0][field]
    rebind_asset_records_artifact(output, rows)

    with pytest.raises(
        verifier.VerificationError, match="Table 4 strict collision mismatch"
    ):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_package_drift(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    first = json.loads(
        (output / "asset_records.jsonl").read_text().splitlines()[0]
    )
    (Path(first["package"]) / "post_run_drift.txt").write_text(
        "drift\n", encoding="utf-8"
    )

    with pytest.raises(verifier.VerificationError, match="package binding mismatch"):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_package_rebound_away_from_source_manifest(
    tmp_path: Path,
) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    package = Path(rows[0]["package"])
    (package / "post_run_drift.txt").write_text("drift\n", encoding="utf-8")
    rows[0]["package_content_manifest_sha256"] = package_binding(package)[
        "content_manifest_sha256"
    ]
    rebind_asset_records_artifact(output, rows)

    with pytest.raises(
        verifier.VerificationError, match="source cohort package binding mismatch"
    ):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_recomputes_release_evidence(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    rows[0]["release_receipt_bound"] = True
    rows[0]["receipt_replay_status"] = "VALID_RECEIPT_NOT_REPLAYED"
    rebind_asset_records_artifact(output, rows)

    with pytest.raises(
        verifier.VerificationError, match="release receipt evidence mismatch"
    ):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_nested_release_evidence_tampering(
    tmp_path: Path,
) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    rows[0]["release_evidence"]["rebuild"]["status"] = "ELIGIBLE_NOT_RUN"
    rebind_asset_records_artifact(output, rows)

    with pytest.raises(
        verifier.VerificationError, match="nested release evidence mismatch"
    ):
        verifier.verify_run(output, formal=False)


def test_independent_verifier_rejects_official_record_drift(tmp_path: Path) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    first = json.loads(
        (output / "asset_records.jsonl").read_text().splitlines()[0]
    )
    official = first["release_evidence"]["rebuild"]["official_model_py"]
    Path(official["record_json_path"]).write_text("{}\n", encoding="utf-8")

    with pytest.raises(
        verifier.VerificationError,
        match=r"official record (?:SHA256|identity) mismatch",
    ):
        verifier.verify_run(output, formal=False)


def test_verifier_rejects_rebound_official_source_manifest(
    tmp_path: Path,
) -> None:
    output = write_s1_run_fixture(tmp_path)
    verifier = load_verifier()
    rows = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text().splitlines()
    ]
    official = rows[0]["release_evidence"]["rebuild"]["official_model_py"]
    record_path = Path(official["record_json_path"])
    record = json.loads(record_path.read_text(encoding="utf-8"))
    record["post_run_annotation"] = "drift"
    record_path.write_text(json.dumps(record) + "\n", encoding="utf-8")
    official["record_json_sha256"] = sha256_file(record_path)
    rebind_asset_records_artifact(output, rows)

    with pytest.raises(
        verifier.VerificationError, match="official source manifest mismatch"
    ):
        verifier.verify_run(output, formal=False)
