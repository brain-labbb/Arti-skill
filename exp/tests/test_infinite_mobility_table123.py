from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

import pytest


REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "exp/scripts"


def _load(name: str, filename: str):
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _legacy_digest_sorted(package: Path) -> str:
    digest = hashlib.sha256()
    entries = []
    for path in sorted(item for item in package.rglob("*") if item.is_file()):
        relative = path.relative_to(package).as_posix()
        if Path(relative).name in {"stdout.log", "stderr.log", "record.json"}:
            continue
        entries.append((relative, _sha(path)))
    for relative, file_hash in entries:
        encoded = relative.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(bytes.fromhex(file_hash))
    return digest.hexdigest()


def _write_package(root: Path, seed: int) -> tuple[Path, str]:
    package = root / str(seed)
    (package / "objs").mkdir(parents=True)
    (package / "objs/part.obj").write_text("v 0 0 0\n", encoding="utf-8")
    urdf = package / "scene.urdf"
    urdf.write_text(
        """<robot name="fixture">
<link name="base"><visual><geometry><mesh filename="objs/part.obj"/></geometry></visual></link>
<link name="door"/>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/></joint>
</robot>\n""",
        encoding="utf-8",
    )
    return package, _sha(urdf)


def _fixture_sources(tmp_path: Path) -> dict[str, Path]:
    primary = tmp_path / "primary"
    recovery = tmp_path / "recovery"
    package0, urdf0 = _write_package(primary / "cases/FactoryA/seed_000/package", 0)
    package1, urdf1 = _write_package(recovery / "cases/FactoryA/seed_001/package", 1)

    primary_record = {
        "factory": "FactoryA", "seed": 0, "status": "PASS",
        "package_sha256": None,
        "validation": {"urdf_path": "0/scene.urdf"},
    }
    (primary / "records.json").parent.mkdir(parents=True, exist_ok=True)
    timeout_record = {
        "factory": "FactoryA", "seed": 1, "status": "TIMEOUT",
        "package_sha256": None,
        "validation": {},
    }
    (primary / "records.json").write_text(
        json.dumps([primary_record, timeout_record]), encoding="utf-8"
    )
    (primary / "manifest.json").write_text(
        json.dumps({"factories": ["FactoryA"], "protocol": {"seeds": [0, 1]}}),
        encoding="utf-8",
    )
    recovery_record = {
        "factory": "FactoryA", "seed": 1, "status": "PASS",
        "package_sha256": None,
        "validation": {"urdf_path": "1/scene.urdf"},
    }
    recovery_record_path = recovery / "cases/FactoryA/seed_001/record.json"
    recovery_record_path.parent.mkdir(parents=True, exist_ok=True)
    recovery_record_path.write_text(json.dumps(recovery_record), encoding="utf-8")
    recovery_case = {
        "factory": "FactoryA", "seed": 1, "original_status": "TIMEOUT",
        "recovery_status": "PASS",
        "recovery_record": recovery_record_path.relative_to(tmp_path).as_posix(),
        "recovery_record_sha256": _sha(recovery_record_path),
    }
    (recovery / "recovery_manifest.json").write_text(
        json.dumps({"expected_recovery_case_count": 1, "cases": [recovery_case]}),
        encoding="utf-8",
    )
    (recovery / "recovery_records.json").write_text(json.dumps([recovery_case]), encoding="utf-8")
    return {
        "primary": primary,
        "recovery": recovery,
        "package0": package0,
        "package1": package1,
        "urdf0": Path(urdf0),
        "urdf1": Path(urdf1),
    }


def test_freeze_overlay_retains_timeout_provenance_and_nested_urdf(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    paths = _fixture_sources(tmp_path)

    rows = common.build_cohort_rows(
        repo_root=tmp_path,
        primary_root=paths["primary"],
        recovery_root=paths["recovery"],
    )

    assert [row["asset_id"] for row in rows] == ["FactoryA/seed_000", "FactoryA/seed_001"]
    assert rows[0]["recovery_used"] is False
    assert rows[1]["original_status"] == "TIMEOUT"
    assert rows[1]["recovery_used"] is True
    assert rows[1]["recovery_provenance"]["recovery_record_sha256"]
    assert rows[1]["urdf_relpath"] == "1/scene.urdf"
    assert rows[1]["primary_urdf_sha256"] == _sha(paths["package1"] / "scene.urdf")
    assert [row["selection_index"] for row in rows] == [1, 2]
    assert rows[0]["source"] == "primary" and rows[1]["source"] == "recovery"
    assert rows[0]["declared_joint_count_hint"] == 1


def test_freeze_refuses_timeout_without_recovery(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    paths = _fixture_sources(tmp_path)
    (paths["recovery"] / "recovery_manifest.json").write_text(
        json.dumps({"expected_recovery_case_count": 0, "cases": []}), encoding="utf-8"
    )
    (paths["recovery"] / "recovery_records.json").write_text("[]", encoding="utf-8")

    with pytest.raises(ValueError, match="missing recovery"):
        common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])


def test_freeze_refuses_duplicate_asset_and_escaping_urdf(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    paths = _fixture_sources(tmp_path)
    records = json.loads((paths["primary"] / "records.json").read_text(encoding="utf-8"))
    records.append({**records[0], "validation": {"urdf_path": "../../escape.urdf"}})
    (paths["primary"] / "records.json").write_text(json.dumps(records), encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate primary record"):
        common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])

    (paths["primary"] / "records.json").write_text(
        json.dumps([
            {**records[0], "validation": {"urdf_path": "../../escape.urdf"}},
            records[1],
        ]),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="escapes package"):
        common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])


def test_table1_retains_drifted_asset_as_failed_record_in_denominator(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    runner = _load("run_table1_infinite_mobility", "run_table1_infinite_mobility.py")
    paths = _fixture_sources(tmp_path)
    rows = common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])
    manifest = common.cohort_manifest(rows, factory_order=["FactoryA"], seeds=[0, 1])
    manifest["manifest_content_sha256"] = common.manifest_self_hash(manifest)
    cohort = tmp_path / "cohort"
    cohort.mkdir()
    (cohort / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    records = runner.evaluate_cohort(cohort / "manifest.json", workers=1)
    assert records[0]["parse_success"] is True
    assert records[0]["fingerprint_complete"] is True
    assert records[0]["status"] == "COMPLETED"

    (paths["package0"] / "scene.urdf").write_text("<robot/>", encoding="utf-8")
    records = runner.evaluate_cohort(cohort / "manifest.json", workers=1)

    assert [record["status"] for record in records] == ["FAILED", "COMPLETED"]
    summary = runner.aggregate_records(records, release_asset_count=2, release_category_count=1)
    assert summary["cohort"]["N_eval"] == 2
    assert summary["multi_joint_assets"]["denominator"] == 2
    assert summary["unique_topologies"]["coverage_denominator"] == 2
    assert summary["exact_duplicate_rate"]["coverage_denominator"] == 2


def test_formal_contract_rejects_limit_and_wrong_cohort_size(tmp_path: Path) -> None:
    runner = _load("run_table1_infinite_mobility", "run_table1_infinite_mobility.py")

    with pytest.raises(ValueError, match="formal Table 1 does not permit --limit"):
        runner.validate_contract(n_release=720, n_eval=1, limit=1, formal=True)
    with pytest.raises(ValueError, match="must contain exactly 720"):
        runner.validate_contract(n_release=2, n_eval=2, limit=None, formal=True)


def test_freeze_rejects_recovery_record_redirection_and_package_symlink(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    paths = _fixture_sources(tmp_path)
    alternate = paths["recovery"] / "other.json"
    alternate.write_text(
        json.dumps({"factory": "FactoryA", "seed": 1, "status": "PASS", "validation": {"urdf_path": "1/scene.urdf"}}),
        encoding="utf-8",
    )
    case = json.loads((paths["recovery"] / "recovery_manifest.json").read_text(encoding="utf-8"))["cases"][0]
    case["recovery_record"] = alternate.relative_to(tmp_path).as_posix()
    case["recovery_record_sha256"] = _sha(alternate)
    (paths["recovery"] / "recovery_manifest.json").write_text(
        json.dumps({"expected_recovery_case_count": 1, "cases": [case]}), encoding="utf-8"
    )
    (paths["recovery"] / "recovery_records.json").write_text(json.dumps([case]), encoding="utf-8")

    with pytest.raises(ValueError, match="expected recovery record path"):
        common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])

    paths = _fixture_sources(tmp_path / "symlink")
    package_root = paths["primary"] / "cases/FactoryA/seed_000"
    package = package_root / "package"
    target = package_root / "target"
    package.rename(target)
    package.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        common.build_cohort_rows(tmp_path / "symlink", paths["primary"], paths["recovery"])


def test_publish_and_run_freeze_real_protocol_and_refuse_overwrite(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    runner = _load("run_table1_infinite_mobility", "run_table1_infinite_mobility.py")
    paths = _fixture_sources(tmp_path)
    cohort_output = tmp_path / "cohort-output"
    frozen = common.publish_cohort(
        repo_root=tmp_path,
        primary_root=paths["primary"],
        recovery_root=paths["recovery"],
        output=cohort_output,
        formal=False,
    )
    resolved_cohort = cohort_output.resolve()
    assert frozen["manifest_content_sha256"] == common.manifest_self_hash(frozen)
    assert frozen["source"]["bindings"] == frozen["source_bindings"]
    assert frozen["evaluation"]["protocol_sha256"]
    assert frozen["evaluation"]["preparer_sha256"]
    assert (resolved_cohort / "source_selection.json").is_file()
    assert cohort_output.is_dir() and not cohort_output.is_symlink()
    assert common.verify_cohort_manifest(cohort_output)["N_eval"] == 2
    common.verify_artifacts(resolved_cohort)
    altered = dict(frozen)
    altered["evaluation"] = dict(frozen["evaluation"])
    altered["evaluation"]["protocol_sha256"] = "0" * 64
    altered["manifest_content_sha256"] = common.manifest_self_hash(altered)
    altered_path = tmp_path / "altered-manifest.json"
    altered_path.write_text(json.dumps(altered), encoding="utf-8")
    with pytest.raises(ValueError, match="evaluation binding drift"):
        common.verify_cohort_manifest(altered_path)

    output = tmp_path / "table1-output"
    summary = runner.run(
        cohort_manifest=resolved_cohort / "manifest.json",
        output=output,
        workers=1,
        limit=None,
        formal=False,
    )
    published = output.resolve()
    assert summary["cohort"]["N_eval"] == 2
    assert (published / "protocol_snapshot.md").read_bytes() == runner.PROTOCOL_PATH.read_bytes()
    manifest = json.loads((published / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["manifest_content_sha256"] == common.manifest_self_hash(manifest)
    report = (published / "report.md").read_text(encoding="utf-8")
    assert "Multi-joint Assets" in report
    assert "supplementary full generated cohort" in report
    assert "5,402" not in report and "salted SHA-256" not in report
    common.verify_artifacts(published)
    with pytest.raises(RuntimeError, match="refusing to overwrite"):
        runner.run(cohort_manifest=resolved_cohort / "manifest.json", output=output, workers=1, limit=None, formal=False)


def test_formal_matrix_and_single_scan_per_evaluated_asset(tmp_path: Path, monkeypatch: Any) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    runner = _load("run_table1_infinite_mobility", "run_table1_infinite_mobility.py")
    recovery_ids = set(common.APPROVED_RECOVERY_IDENTITIES)
    formal_rows = []
    for factory in common.APPROVED_FACTORIES:
        for seed in range(36):
            asset_id = f"{factory}/seed_{seed:03d}"
            recovered = asset_id in recovery_ids
            formal_rows.append({
                "asset_id": asset_id, "factory": factory, "raw_category": factory, "seed": seed,
                "original_status": "TIMEOUT" if recovered else "PASS", "recovery_used": recovered,
                "recovery_provenance": {"original_record_path": "/fixture/records.json", "original_record_sha256": "a" * 64, "recovery_record_path": f"/fixture/{asset_id}/record.json", "recovery_record_sha256": "b" * 64} if recovered else None,
                "selection_index": len(formal_rows) + 1, "source": "recovery" if recovered else "primary",
                "declared_joint_count_hint": 1,
            })
    formal = {
        "N_release": 720, "N_eval": 720, "factory_order": list(common.APPROVED_FACTORIES),
        "seeds": list(range(36)), "assets": formal_rows,
    }
    runner.validate_formal_manifest(formal)
    formal["assets"][0]["raw_category"] = "wrong"
    with pytest.raises(ValueError, match="identity fields"):
        runner.validate_formal_manifest(formal)

    paths = _fixture_sources(tmp_path)
    rows = common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])
    manifest = common.cohort_manifest(rows, factory_order=["FactoryA"], seeds=[0, 1])
    manifest["manifest_content_sha256"] = common.manifest_self_hash(manifest)
    cohort = tmp_path / "cohort"
    cohort.mkdir()
    (cohort / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    calls = 0
    original = runner.scan_package

    def counted(package: Path):
        nonlocal calls
        calls += 1
        return original(package)

    monkeypatch.setattr(runner, "scan_package", counted)
    records = runner.evaluate_cohort(cohort / "manifest.json", workers=1)
    assert len(records) == 2
    assert calls == 2


def test_atomic_directory_publish_refuses_existing_target(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    source = tmp_path / "source"
    source.mkdir()
    (source / "complete").write_text("ok", encoding="utf-8")
    output = tmp_path / "output"

    common.rename_noreplace(source, output)

    assert output.is_dir() and (output / "complete").read_text(encoding="utf-8") == "ok"
    replacement = tmp_path / "replacement"
    replacement.mkdir()
    with pytest.raises(RuntimeError, match="refusing to overwrite"):
        common.rename_noreplace(replacement, output)


def test_legacy_package_hash_uses_global_relative_path_order(tmp_path: Path) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    paths = _fixture_sources(tmp_path)
    package_root = paths["package0"].parent
    (package_root / "root.bin").write_bytes(b"root-content")
    expected = _legacy_digest_sorted(package_root)
    records = json.loads((paths["primary"] / "records.json").read_text(encoding="utf-8"))
    records[0]["package_sha256"] = expected
    (paths["primary"] / "records.json").write_text(json.dumps(records), encoding="utf-8")

    rows = common.build_cohort_rows(tmp_path, paths["primary"], paths["recovery"])

    assert rows[0]["baseline_package_sha256"] == expected


def test_runtime_mount_publish_run_and_late_target_are_no_replace_safe(tmp_path: Path, monkeypatch: Any) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    runner = _load("run_table1_infinite_mobility", "run_table1_infinite_mobility.py")
    paths = _fixture_sources(tmp_path)
    runtime_root = REPO / "runtime/infinite_mobility_table123_task_reports"
    runtime_root.mkdir(parents=True, exist_ok=True)
    sandbox = Path(tempfile.mkdtemp(prefix=".cpfs-table123-", dir=runtime_root))
    try:
        cohort_output = sandbox / "cohort"
        common.publish_cohort(
            repo_root=tmp_path,
            primary_root=paths["primary"],
            recovery_root=paths["recovery"],
            output=cohort_output,
            formal=False,
        )
        result = runner.run(
            cohort_manifest=cohort_output / "manifest.json",
            output=sandbox / "table1",
            workers=1,
            limit=None,
            formal=False,
        )
        assert result["cohort"]["N_eval"] == 2
        assert (sandbox / "table1" / "manifest.json").is_file()

        staging = sandbox / "late-staging"
        staging.mkdir()
        payload = staging / "payload.txt"
        payload.write_text("staged", encoding="utf-8")
        (staging / "artifact_manifest.json").write_text(
            json.dumps({"schema_version": 1, "files": {"payload.txt": {"bytes": payload.stat().st_size, "sha256": _sha(payload)}}}),
            encoding="utf-8",
        )
        target = sandbox / "late-target"
        original_reserve = common.reserve_output

        def create_target_then_reserve(destination: Path) -> tuple[int, int]:
            destination.mkdir()
            return original_reserve(destination)

        monkeypatch.setattr(common, "reserve_output", create_target_then_reserve)
        with pytest.raises(RuntimeError, match="refusing to overwrite"):
            common.publish_staged(staging, target)
        assert target.is_dir()
        assert staging.is_dir()
    finally:
        shutil.rmtree(sandbox, ignore_errors=True)


def test_publish_detects_replaced_reservation_without_deleting_new_target(
    tmp_path: Path, monkeypatch: Any
) -> None:
    common = _load("infinite_mobility_table123_common", "infinite_mobility_table123_common.py")
    source = tmp_path / "source"
    source.mkdir()
    (source / "first").write_text("one", encoding="utf-8")
    (source / "second").write_text("two", encoding="utf-8")
    target = tmp_path / "target"
    original_guard = common._guard_reservation
    calls = 0

    def replace_after_first_move(destination: Path, reservation: tuple[int, int]) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            shutil.rmtree(destination)
            destination.mkdir()
            (destination / "new-owner").write_text("keep", encoding="utf-8")
        original_guard(destination, reservation)

    monkeypatch.setattr(common, "_guard_reservation", replace_after_first_move)

    with pytest.raises(RuntimeError, match="output reservation changed"):
        common.rename_noreplace(source, target)
    assert (target / "new-owner").read_text(encoding="utf-8") == "keep"
