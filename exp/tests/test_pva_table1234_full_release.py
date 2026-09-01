from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sqlite3
import sys
from pathlib import Path
import zlib

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def test_pva_automation_entrypoints_are_installed() -> None:
    expected = {
        "build_pva_full_release_roster.py",
        "run_pva_table1234_full_release.py",
        "check_pva_table1234_full_release.py",
        "audit_pva_table1_topologies.py",
        "render_pva_table1234_full_release_results.py",
        "render_pva_table123_interim.py",
    }
    assert expected <= {path.name for path in SCRIPTS.glob("*.py")}


def test_pva_checker_contract_suite_includes_evidence_and_resume_tests() -> None:
    checker = _load("check_pva_table1234_full_release")
    assert set(checker.CONTRACT_TEST_FILES) == {
        "test_pva_table1234_full_release.py",
        "test_pva_roster_shard_evidence.py",
        "test_pva_runner_scaling_resume.py",
    }


def _load(name: str):
    path = SCRIPTS / f"{name}.py"
    assert path.is_file(), f"missing automation script: {path}"
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_pva_interim_renderer_freezes_only_the_contiguous_result_prefix(
    tmp_path: Path,
) -> None:
    database = tmp_path / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        CREATE TABLE results(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            table1_json TEXT NOT NULL,
            table2_json TEXT NOT NULL,
            table2sup_json TEXT NOT NULL,
            table3_json TEXT NOT NULL,
            table4_json TEXT NOT NULL,
            table4_states_zlib BLOB NOT NULL,
            table4_state_count INTEGER NOT NULL,
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL,
            completed_at_utc TEXT NOT NULL
        );
        """
    )
    rows = [
        {
            "ordinal": 0,
            "asset_id": "PV-A/alpha/seed_0000",
            "raw_category": "alpha",
            "category": "alpha",
            "joint_count": 2,
            "primary_urdf_sha256": "a" * 64,
        },
        {
            "ordinal": 1,
            "asset_id": "PV-A/beta/seed_0000",
            "raw_category": "beta",
            "category": "beta",
            "joint_count": 3,
            "primary_urdf_sha256": "b" * 64,
        },
    ]
    connection.executemany(
        "INSERT INTO assets VALUES(?, ?, ?, ?, ?, ?)",
        [
            (
                row["ordinal"],
                row["asset_id"],
                row["category"],
                row["joint_count"],
                hashlib.sha256(json.dumps(row, sort_keys=True).encode()).hexdigest(),
                json.dumps(row, sort_keys=True),
            )
            for row in rows
        ],
    )
    empty = json.dumps({"asset_id": rows[0]["asset_id"]}, sort_keys=True)
    connection.execute(
        "INSERT INTO results VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (0, rows[0]["asset_id"], empty, empty, empty, empty, empty, b"", 7, "completed", 1.0, "now"),
    )
    connection.commit()
    connection.close()

    interim = _load("render_pva_table123_interim")
    snapshot = interim.capture_prefix(database)

    assert snapshot["N_snapshot"] == 1
    assert snapshot["ordinal_min"] == 0
    assert snapshot["ordinal_max"] == 0
    assert snapshot["J_snapshot"] == 2
    assert snapshot["category_count_snapshot"] == 1
    assert snapshot["table4_state_records"] == 7
    assert snapshot["worker_status_counts"] == {"completed": 1}
    assert [row["asset_id"] for row in snapshot["rows"]] == [rows[0]["asset_id"]]


def _urdf(*, collision: bool) -> str:
    geometry = (
        '<collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>'
        if collision
        else ""
    )
    return f"""<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">{geometry}</link>
  <link name="door">{geometry}</link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _dataset(root: Path) -> Path:
    root.mkdir()
    (root / "archives").mkdir()
    (root / "extracted").mkdir()
    entries = []
    for slug, collision in (("Alpha", True), ("Beta", False)):
        asset_id = "seed_0000"
        package = root / "extracted" / slug / asset_id
        package.mkdir(parents=True)
        (package / "model.urdf").write_text(_urdf(collision=collision), encoding="utf-8")
        (package / "appearance.json").write_text("{}\n", encoding="utf-8")
        (package / "physics.json").write_text("{}\n", encoding="utf-8")
        (root / "archives" / f"{slug}.tar.zst").write_bytes(f"archive:{slug}".encode())
        entries.append(
            {
                "slug": slug,
                "stem": slug.lower(),
                "seed": "0",
                "asset_id": asset_id,
                "overrides_json": "{}",
            }
        )
    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(entries[0]))
        writer.writeheader()
        writer.writerows(entries)
    return root


@pytest.fixture()
def frozen_roster(tmp_path: Path) -> tuple[Path, Path]:
    builder = _load("build_pva_full_release_roster")
    dataset = _dataset(tmp_path / "PV-A")
    output = tmp_path / "roster"
    manifest_path = builder.build_roster(
        dataset,
        output,
        workers=2,
        expected_n=2,
        expected_categories=2,
    )
    return manifest_path, dataset


def test_pva_roster_freezes_every_manifest_identity_and_archive_binding(
    frozen_roster: tuple[Path, Path],
) -> None:
    builder = _load("build_pva_full_release_roster")
    manifest_path, dataset = frozen_roster

    manifest = builder.load_roster_manifest(manifest_path, verify_rows=True)
    rows = list(builder.iter_roster_rows(manifest_path))

    assert manifest["schema_version"] == builder.ROSTER_SCHEMA_VERSION
    assert manifest["N_eval"] == 2
    assert manifest["J_eval"] == 2
    assert manifest["release_category_count"] == 2
    assert [row["asset_id"] for row in rows] == [
        "PV-A/Alpha/seed_0000",
        "PV-A/Beta/seed_0000",
    ]
    assert [row["ordinal"] for row in rows] == [0, 1]
    assert all(row["joint_count"] == 1 for row in rows)
    assert all(len(row["package_files"]) == 3 for row in rows)
    assert rows[0]["archive_name"] == "Alpha.tar.zst"
    assert len(rows[0]["archive_sha256"]) == 64
    assert Path(rows[0]["primary_urdf_path"]).is_relative_to(dataset)
    progress = json.loads(
        (manifest_path.parent / "progress.json").read_text(encoding="utf-8")
    )
    assert progress["status"] == "COMPLETE"
    assert progress["completed_categories"] == 2
    assert progress["completed_assets"] == 2


def test_pva_roster_rejects_manifest_denominator_drift(tmp_path: Path) -> None:
    builder = _load("build_pva_full_release_roster")
    dataset = _dataset(tmp_path / "PV-A")

    with pytest.raises(ValueError, match="release count mismatch"):
        builder.build_roster(
            dataset,
            tmp_path / "roster",
            workers=1,
            expected_n=3,
            expected_categories=2,
        )


def test_pva_roster_does_not_rehash_every_package_after_shards_are_sealed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    builder = _load("build_pva_full_release_roster")
    dataset = _dataset(tmp_path / "PV-A")

    def redundant_verification(_row: object) -> None:
        raise AssertionError("sealed packages were redundantly rehashed")

    monkeypatch.setattr(builder, "_verify_frozen_package", redundant_verification)
    manifest_path = builder.build_roster(
        dataset,
        tmp_path / "roster",
        workers=1,
        expected_n=2,
        expected_categories=2,
    )
    assert builder.load_roster_manifest(manifest_path, verify_rows=False)["N_eval"] == 2


def test_pva_roster_keeps_qc_seed_distinct_from_archive_asset_id(tmp_path: Path) -> None:
    builder = _load("build_pva_full_release_roster")
    dataset = _dataset(tmp_path / "PV-A")
    with (dataset / "manifest.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    rows[0]["seed"] = "3"
    with (dataset / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    manifest_path = builder.build_roster(
        dataset,
        tmp_path / "roster",
        workers=1,
        expected_n=2,
        expected_categories=2,
    )
    first = next(builder.iter_roster_rows(manifest_path))
    assert first["source_asset_id"] == "seed_0000"
    assert first["seed"] == 3


def test_pva_roster_resume_rebinds_changed_manifest_archive_and_package(
    tmp_path: Path,
) -> None:
    builder = _load("build_pva_full_release_roster")
    dataset = _dataset(tmp_path / "PV-A")
    output = tmp_path / "roster"
    manifest_path = builder.build_roster(
        dataset, output, workers=2, expected_n=2, expected_categories=2
    )
    before = list(builder.iter_roster_rows(manifest_path))[0]

    appearance = dataset / "extracted" / "Alpha" / "seed_0000" / "appearance.json"
    appearance.write_text('{"changed":true}\n', encoding="utf-8")
    (dataset / "archives" / "Alpha.tar.zst").write_bytes(b"changed archive")
    with (dataset / "manifest.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    rows[0]["stem"] = "alpha_changed"
    with (dataset / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    manifest_path = builder.build_roster(
        dataset,
        output,
        workers=2,
        expected_n=2,
        expected_categories=2,
        resume=True,
    )
    after = list(builder.iter_roster_rows(manifest_path))[0]
    assert after["stem"] == "alpha_changed"
    assert after["archive_sha256"] != before["archive_sha256"]
    before_appearance = next(
        item for item in before["package_files"] if item["path"] == "appearance.json"
    )
    after_appearance = next(
        item for item in after["package_files"] if item["path"] == "appearance.json"
    )
    assert after_appearance["sha256"] != before_appearance["sha256"]


def test_table1_and_table2_roster_membership_is_hash_indexed(monkeypatch: pytest.MonkeyPatch) -> None:
    class CountingString(str):
        comparisons = 0

        def __eq__(self, other: object) -> bool:
            type(self).comparisons += 1
            return super().__eq__(other)

        __hash__ = str.__hash__

    size = 80
    for module_name in ("run_table1_full_release", "run_table2_full_release"):
        module = _load(module_name)
        monkeypatch.setattr(module, "str", CountingString, raising=False)
        rows = [
            {
                "asset_id": CountingString(f"asset-{index}"),
                "ordinal": index,
                "joint_count": 0,
                "category": "fixture",
                "raw_category": "fixture",
            }
            for index in range(size)
        ]
        records = [module.failure_record(row, "fixture") for row in rows]
        roster = {
            "schema_version": "table123_full_release_manifest_v1",
            "dataset": "fixture",
            "N_eval": size,
            "J_eval": 0,
            "rows": rows,
        }
        CountingString.comparisons = 0
        module.aggregate_full_release(records, roster)
        assert CountingString.comparisons < size * 6


def test_pva_automation_runs_all_tables_resumes_checks_and_renders(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    renderer = _load("render_pva_table1234_full_release_results")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "evaluation"

    runner.run_full_release(
        manifest_path,
        output,
        workers=2,
        timeout_seconds=90,
        run_standard_parser=True,
    )

    with sqlite3.connect(output / "results.sqlite3") as connection:
        assert connection.execute("SELECT COUNT(*) FROM assets").fetchone()[0] == 2
        assert connection.execute("SELECT COUNT(*) FROM results").fetchone()[0] == 2
        before = connection.execute(
            "SELECT ordinal, table1_json, table2_json, table3_json, table4_json "
            "FROM results ORDER BY ordinal"
        ).fetchall()

    for table in ("table1", "table2", "table2_supplementary", "table3", "table4"):
        summary = json.loads((output / table / "summary.json").read_text(encoding="utf-8"))
        assert summary.get("n_eval", summary.get("N_eval", summary.get("cohort", {}).get("N_eval"))) == 2
    table4 = json.loads((output / "table4" / "summary.json").read_text(encoding="utf-8"))
    assert table4["state_records_executed"] == 86
    execution = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert execution["classification"] == "TEST_FIXTURE"
    assert {
        "table1_core",
        "table2_core",
        "table2_supplementary_core",
        "table3_core",
        "table4_core",
        "table123_common",
    } <= set(execution["source_hashes"])
    protocol_snapshot = output / execution["protocol"]["snapshot"]
    protocol_source = Path(execution["protocol"]["source_document"])
    assert protocol_snapshot.read_bytes() == protocol_source.read_bytes()
    assert execution["protocol"]["snapshot_sha256"] == hashlib.sha256(
        protocol_snapshot.read_bytes()
    ).hexdigest()

    runner.run_full_release(
        manifest_path,
        output,
        workers=2,
        timeout_seconds=90,
        run_standard_parser=True,
        resume=True,
    )
    with sqlite3.connect(output / "results.sqlite3") as connection:
        after = connection.execute(
            "SELECT ordinal, table1_json, table2_json, table3_json, table4_json "
            "FROM results ORDER BY ordinal"
        ).fetchall()
    assert after == before

    report = checker.check_results(output, expected_n=2, expected_categories=2)
    assert report["all_pass"] is True
    assert report["tables_checked"] == 5
    receipt_path = output / "full_release_receipt.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    for binding in receipt["tables"].values():
        assert binding["artifact_manifest"].endswith("artifact_manifest.json")
        assert len(binding["artifact_manifest_sha256"]) == 64

    markdown = renderer.render(output)
    assert "# Ours / PV-A TEST_FIXTURE Results" in markdown
    assert "## Table 1" in markdown
    assert "## Table 2 Supplementary" in markdown
    assert "## Table 4" in markdown
    assert "FK Round-trip Error" in markdown
    assert "Ours / PV-A" in markdown
    assert "Pooled raw-tree support (descriptive)" in markdown
    assert "pooled diagnostic" in markdown
    assert "Category-conditioned support" in markdown
    assert "rarefaction at `k=1` is 100.00%" in markdown
    assert "not a higher-is-better diversity score" in markdown

    protocol_bytes = protocol_snapshot.read_bytes()
    protocol_snapshot.write_text("tampered protocol\n", encoding="utf-8")
    with pytest.raises(checker.AutomationError, match="protocol snapshot hash mismatch"):
        checker.check_results(output, expected_n=2, expected_categories=2)
    protocol_snapshot.write_bytes(protocol_bytes)

    execution["roster_manifest_sha256"] = "0" * 64
    execution["manifest_content_sha256"] = runner._self_hash(
        execution, "manifest_content_sha256"
    )
    runner._atomic_json(output / "manifest.json", execution)
    receipt["execution_manifest_sha256"] = runner.common.sha256_file(
        output / "manifest.json"
    )
    receipt["receipt_content_sha256"] = runner._self_hash(
        receipt, "receipt_content_sha256"
    )
    runner._atomic_json(receipt_path, receipt)
    with pytest.raises(
        checker.AutomationError, match="execution/receipt roster binding mismatch"
    ):
        checker.check_results(output, expected_n=2, expected_categories=2)


def test_pva_table1_renderer_separates_pooled_and_category_topology_views() -> None:
    renderer = _load("render_pva_table1234_full_release_results")
    summary = {
        "cohort": {
            "N_release": 4,
            "N_eval": 4,
            "release_raw_categories": 2,
            "eval_raw_categories": 2,
        },
        "links_per_asset": {
            "denominator": 4,
            "mean": 2,
            "median": 2,
            "p90_nearest_rank": 2,
        },
        "movable_joints_per_asset": {
            "denominator": 4,
            "mean": 1,
            "median": 1,
            "p90_nearest_rank": 1,
        },
        "multi_joint_assets": {"denominator": 4, "rate": 0.0},
        "unique_topologies": {"unique": 2, "denominator": 4, "rate": 0.5},
        "exact_duplicate_rate": {"denominator": 4, "rate": 0.0},
        "category_macro": {"unique_topologies_rate": 0.75},
        "category_breakdown": {
            "alpha": {
                "unique_topologies": {
                    "unique": 1,
                    "denominator": 2,
                    "rate": 0.5,
                }
            },
            "beta": {
                "unique_topologies": {
                    "unique": 2,
                    "denominator": 2,
                    "rate": 1.0,
                }
            },
        },
    }

    audit = {
        "category_stratified_rarefaction": {
            "k": 2,
            "rate": 0.875,
        }
    }
    markdown = "\n".join(renderer._table1(summary, audit))

    assert "2 / 4 (50.00%; pooled diagnostic)" in markdown
    assert "3 / 4 (75.00%)" in markdown
    assert "75.00% (2 categories)" in markdown
    assert "rarefaction at `k=2` is 87.50%" in markdown


def test_pva_runner_resumes_a_partial_asset_database_import(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    builder = _load("build_pva_full_release_roster")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "partial-import"
    output.mkdir()
    connection = runner._connect(output / "results.sqlite3")
    runner._create_schema(connection)
    manifest = builder.load_roster_manifest(manifest_path, verify_rows=False)
    binding = runner._asset_import_binding(
        manifest_path, manifest, selected_n=2, limit=None
    )
    runner._set_meta(connection, {**binding, "asset_import_state": "LOADING"})
    first = next(builder.iter_roster_rows(manifest_path))
    row_text = runner._canonical_text(first)
    connection.execute(
        "INSERT INTO assets(ordinal, asset_id, category, joint_count, row_sha256, row_json) "
        "VALUES(?, ?, ?, ?, ?, ?)",
        (
            0,
            first["asset_id"],
            first["raw_category"],
            first["joint_count"],
            hashlib.sha256(row_text.encode("utf-8")).hexdigest(),
            row_text,
        ),
    )
    connection.commit()
    connection.close()

    runner.run_full_release(
        manifest_path,
        output,
        workers=2,
        timeout_seconds=90,
        run_standard_parser=False,
        resume=True,
    )
    with sqlite3.connect(output / "results.sqlite3") as connection:
        assert connection.execute("SELECT COUNT(*) FROM assets").fetchone()[0] == 2
        assert connection.execute("SELECT COUNT(*) FROM results").fetchone()[0] == 2


def test_pva_runner_fails_closed_when_a_frozen_package_disappears(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    renderer = _load("render_pva_table1234_full_release_results")
    manifest_path, dataset = frozen_roster
    package = dataset / "extracted" / "Alpha" / "seed_0000"
    for path in sorted(package.rglob("*"), reverse=True):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
    package.rmdir()

    output = tmp_path / "missing-package"
    runner.run_full_release(
        manifest_path,
        output,
        workers=1,
        timeout_seconds=90,
        run_standard_parser=False,
        limit=1,
    )
    table1 = json.loads(
        (output / "table1" / "asset_records.jsonl").read_text(encoding="utf-8")
    )
    table4 = json.loads(
        (output / "table4" / "records.jsonl").read_text(encoding="utf-8")
    )
    assert table1["parse_success"] is False
    assert "package_binding_preflight" in table1["error"]
    assert table4["rest_state_executed"] == 0
    assert table4["single_state_expected"] == 21
    assert table4["sobol_state_expected"] == 64
    assert checker.check_results(output, expected_n=1, expected_categories=1)[
        "all_pass"
    ]
    markdown = renderer.render(output)
    assert "N/E / N/E / N/E (n=0)" in markdown


def test_pva_checker_binds_every_table_record_to_the_frozen_asset(
    frozen_roster: tuple[Path, Path],
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    builder = _load("build_pva_full_release_roster")
    manifest_path, _dataset_root = frozen_roster
    row = next(builder.iter_roster_rows(manifest_path))
    records = runner._failure_records(row, "fixture")

    for table, record in records.items():
        checker._check_record_source_binding(table, record, row, 0)

    records["table1"]["primary_urdf_sha256"] = "0" * 64
    with pytest.raises(checker.AutomationError, match="source binding mismatch"):
        checker._check_record_source_binding("table1", records["table1"], row, 0)
    records["table1"]["primary_urdf_sha256"] = row["primary_urdf_sha256"]
    records["table1"]["roster_joint_count"] = int(row["joint_count"]) + 1
    with pytest.raises(checker.AutomationError, match="source binding mismatch"):
        checker._check_record_source_binding("table1", records["table1"], row, 0)


def test_pva_checker_cross_checks_checkpoint_internal_hashes() -> None:
    checker = _load("check_pva_table1234_full_release")
    binding = {
        "records_sha256": "a" * 64,
        "summary_sha256": "b" * 64,
    }
    checkpoint = {
        "schema_version": "pva_table_checkpoint_v1",
        "state": "complete",
        "records": 2,
        "N_eval": 2,
        "J_eval": 2,
        "records_sha256": "a" * 64,
        "summary_sha256": "b" * 64,
    }
    checker._check_checkpoint_binding(
        "table1",
        checkpoint,
        binding,
        n_eval=2,
        j_eval=2,
        records_hash="a" * 64,
        summary_hash="b" * 64,
    )
    checkpoint["records_sha256"] = "c" * 64
    with pytest.raises(checker.AutomationError, match="checkpoint records hash mismatch"):
        checker._check_checkpoint_binding(
            "table1",
            checkpoint,
            binding,
            n_eval=2,
            j_eval=2,
            records_hash="a" * 64,
            summary_hash="b" * 64,
        )


def test_pva_checker_recomputes_table4_asset_metrics_from_states(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "state-reaggregation"
    runner.run_full_release(
        manifest_path,
        output,
        workers=1,
        timeout_seconds=90,
        run_standard_parser=False,
        limit=1,
    )
    records = checker._records(output / "table4" / "records.jsonl")
    assert records[0]["state_records_count"] > 0
    checker._verify_table4_states_stream(
        output / "table4" / "state_records.jsonl",
        records,
        database_path=output / "results.sqlite3",
    )
    original_rest_free = int(records[0]["rest_non_adjacent_free"])
    records[0]["rest_non_adjacent_free"] = 1 - original_rest_free
    with pytest.raises(checker.AutomationError, match="state-derived metric mismatch"):
        checker._verify_table4_states_stream(
            output / "table4" / "state_records.jsonl", records
        )
    records[0]["rest_non_adjacent_free"] = original_rest_free

    states = checker._records(output / "table4" / "state_records.jsonl")
    sobol = next(state for state in states if state["phase"] == "multi_joint_sobol")
    sobol["sample_index"] = 10_000
    records[0]["state_records_sha256"] = runner.table4.canonical_sha256(states)
    resigned_states = tmp_path / "resigned-state-records.jsonl"
    runner._atomic_jsonl(resigned_states, states)
    with pytest.raises(checker.AutomationError, match="sample coverage mismatch"):
        checker._verify_table4_states_stream(resigned_states, records)

    with sqlite3.connect(output / "results.sqlite3") as connection:
        connection.execute(
            "UPDATE results SET table4_states_zlib=? WHERE ordinal=0",
            (zlib.compress(b""),),
        )
        connection.commit()
    with pytest.raises(checker.AutomationError, match="database state export mismatch"):
        checker._verify_table4_states_stream(
            output / "table4" / "state_records.jsonl",
            checker._records(output / "table4" / "records.jsonl"),
            database_path=output / "results.sqlite3",
        )


def test_table4_rejects_ambiguous_joint_names_before_state_generation() -> None:
    table4 = _load("run_table4_full_release")
    table4._validate_joint_state_identity([{"name": "hinge"}, {"name": "slide"}])
    with pytest.raises(ValueError, match="non-empty and unique"):
        table4._validate_joint_state_identity([{"name": ""}])
    with pytest.raises(ValueError, match="non-empty and unique"):
        table4._validate_joint_state_identity([{"name": "hinge"}, {"name": "hinge"}])


def test_table4_empty_native_observation_is_ne_not_partial() -> None:
    table4 = _load("run_table4_full_release")
    job = {
        "protocol_id": "fixture",
        "dataset": "pva",
        "dataset_id": "asset",
        "category": "fixture",
        "package": "/fixture",
        "urdf_path": "/fixture/model.urdf",
        "primary_urdf_relative_path": "model.urdf",
        "expected_primary_urdf_sha256": "0" * 64,
        "expected_movable_joints": 1,
        "order": 0,
        "input_identity_sha256": "1" * 64,
    }
    record = table4._empty_record(job, "fixture")
    record.update(
        {
            "status": "error",
            "collision_metric_status": "partial",
            "native_collision_elements": 1,
        }
    )
    summary = table4.aggregate_records([record], 1, 1)
    assert summary["metrics"]["max_penetration"]["status"] == "N/E"


def test_pva_checker_detects_a_tampered_summary(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    renderer = _load("render_pva_table1234_full_release_results")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "evaluation"
    runner.run_full_release(
        manifest_path,
        output,
        workers=1,
        timeout_seconds=90,
        run_standard_parser=False,
        limit=1,
    )
    summary_path = output / "table1" / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["N_eval"] = 99
    summary_path.write_text(json.dumps(summary), encoding="utf-8")

    with pytest.raises(checker.AutomationError, match="hash mismatch"):
        checker.check_results(output, expected_n=1, expected_categories=1)
    with pytest.raises(checker.AutomationError, match="hash mismatch"):
        renderer.render(output)


def test_pva_checker_reaggregates_resigned_record_atoms(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "resigned-atoms"
    runner.run_full_release(
        manifest_path,
        output,
        workers=2,
        timeout_seconds=90,
        run_standard_parser=False,
    )

    records_path = output / "table2" / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines()]
    records[0]["metrics"]["collision_coverage"]["pass"] = False
    runner._atomic_jsonl(records_path, records)
    with sqlite3.connect(output / "results.sqlite3") as connection:
        connection.execute(
            "UPDATE results SET table2_json=? WHERE ordinal=0",
            (runner._canonical_text(records[0]),),
        )
        connection.commit()

    checkpoint_path = output / "table2" / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    checkpoint["records_sha256"] = runner.common.sha256_file(records_path)
    checkpoint["checkpoint_content_sha256"] = runner._self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    runner._atomic_json(checkpoint_path, checkpoint)

    receipt_path = output / "full_release_receipt.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["tables"]["table2"]["records_sha256"] = runner.common.sha256_file(
        records_path
    )
    receipt["tables"]["table2"]["checkpoint_sha256"] = runner.common.sha256_file(
        checkpoint_path
    )
    database = output / "results.sqlite3"
    receipt["result_database_bytes"] = database.stat().st_size
    receipt["result_database_sha256"] = runner.common.sha256_file(database)
    receipt["receipt_content_sha256"] = runner._self_hash(
        receipt, "receipt_content_sha256"
    )
    runner._atomic_json(receipt_path, receipt)

    with pytest.raises(checker.AutomationError, match="table2.*reaggregation mismatch"):
        checker.check_results(output, expected_n=2, expected_categories=2)


def test_pva_database_roster_check_rejects_a_same_size_identity_substitution(
    frozen_roster: tuple[Path, Path], tmp_path: Path
) -> None:
    runner = _load("run_pva_table1234_full_release")
    checker = _load("check_pva_table1234_full_release")
    manifest_path, _dataset_root = frozen_roster
    output = tmp_path / "db-substitution"
    runner.run_full_release(
        manifest_path,
        output,
        workers=1,
        timeout_seconds=90,
        run_standard_parser=False,
        limit=1,
    )
    database = output / "results.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE assets SET asset_id='PV-A/replaced/seed_0000'")
        connection.commit()
    with pytest.raises(checker.AutomationError, match="database roster mismatch"):
        checker._check_database_roster(
            database,
            manifest_path,
            n_eval=1,
            j_eval=1,
            category_count=1,
        )
