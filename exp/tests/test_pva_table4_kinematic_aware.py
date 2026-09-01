from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys
import zlib

import pytest


EXP = Path(__file__).resolve().parents[1]
SCRIPTS = EXP / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import run_pva_table4_kinematic_aware_full_release as v3  # noqa: E402
import run_pva_table4_mimic_aware_full_release as v2  # noqa: E402
import run_urdf_table4_partnet_mobility as core  # noqa: E402


PACKAGE = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/extracted/"
    "rack_and_pinion_slider/seed_0000"
)
URDF_SHA256 = "ed077ecb5378b49cdc09433296115f75b909945733958fc8d1d445f670f55ffa"
DUAL_PACKAGE = PACKAGE.parent / "seed_0004"
DUAL_URDF_SHA256 = (
    "50033fc328fa18ee91cd4c39d458f6b1cc1c4727f6c33dd9e4b89663d671b451"
)


def _row() -> dict[str, object]:
    if not (PACKAGE / "model.urdf").is_file():
        pytest.skip("frozen PV-A rack fixture is unavailable")
    return {
        "ordinal": 225047,
        "asset_id": "PV-A/rack_and_pinion_slider/seed_0000",
        "category": "rack_and_pinion_slider",
        "raw_category": "rack_and_pinion_slider",
        "joint_count": 2,
        "source_path": str(PACKAGE),
        "primary_urdf_path": str(PACKAGE / "model.urdf"),
        "primary_urdf_relative_path": (
            "extracted/rack_and_pinion_slider/seed_0000/model.urdf"
        ),
        "primary_urdf_sha256": URDF_SHA256,
        "package_binding_sha256": (
            "19eb9d7ff021c42a621bd3985a447b86b95e6889d0541cbe17a539888bc3b8b7"
        ),
        "overrides_json": json.dumps(
            {
                "drive_skeleton": "moving_rack_straddle",
                "envelope_carriage": "bare_open",
                "pinion_teeth": 18,
                "rack_form": "flat_bar_rack",
                "rack_tooth_count": 12,
            },
            sort_keys=True,
        ),
    }


def _dual_row() -> dict[str, object]:
    if not (DUAL_PACKAGE / "model.urdf").is_file():
        pytest.skip("frozen dual-pinion PV-A rack fixture is unavailable")
    return {
        "ordinal": 225051,
        "asset_id": "PV-A/rack_and_pinion_slider/seed_0004",
        "category": "rack_and_pinion_slider",
        "raw_category": "rack_and_pinion_slider",
        "joint_count": 3,
        "source_path": str(DUAL_PACKAGE),
        "primary_urdf_path": str(DUAL_PACKAGE / "model.urdf"),
        "primary_urdf_relative_path": (
            "extracted/rack_and_pinion_slider/seed_0004/model.urdf"
        ),
        "primary_urdf_sha256": DUAL_URDF_SHA256,
        "package_binding_sha256": (
            "eed021ecf7c32998cca51f3d0dce10327e7a01644a2dcda286e6846e29676f57"
        ),
        "overrides_json": json.dumps(
            {
                "drive_skeleton": "moving_rack_dual",
                "envelope_carriage": "bare_open",
                "pinion_teeth": 22,
                "rack_form": "flat_bar_rack",
                "rack_tooth_count": 16,
            },
            sort_keys=True,
        ),
    }


def _single_variant_row(
    *,
    seed: int,
    ordinal: int,
    drive_skeleton: str,
    pinion_teeth: int,
    urdf_sha256: str,
    package_binding_sha256: str,
    envelope_carriage: str,
    rack_form: str,
    rack_tooth_count: int,
) -> dict[str, object]:
    package = PACKAGE.parent / f"seed_{seed:04d}"
    if not (package / "model.urdf").is_file():
        pytest.skip(f"frozen PV-A rack fixture seed {seed:04d} is unavailable")
    return {
        "ordinal": ordinal,
        "asset_id": f"PV-A/rack_and_pinion_slider/seed_{seed:04d}",
        "category": "rack_and_pinion_slider",
        "raw_category": "rack_and_pinion_slider",
        "joint_count": 2,
        "source_path": str(package),
        "primary_urdf_path": str(package / "model.urdf"),
        "primary_urdf_relative_path": (
            f"extracted/rack_and_pinion_slider/seed_{seed:04d}/model.urdf"
        ),
        "primary_urdf_sha256": urdf_sha256,
        "package_binding_sha256": package_binding_sha256,
        "overrides_json": json.dumps(
            {
                "drive_skeleton": drive_skeleton,
                "envelope_carriage": envelope_carriage,
                "pinion_teeth": pinion_teeth,
                "rack_form": rack_form,
                "rack_tooth_count": rack_tooth_count,
            },
            sort_keys=True,
        ),
    }


def test_historical_v2_plan_and_job_identity_are_byte_compatible() -> None:
    row = _row()
    urdf = Path(str(row["primary_urdf_path"]))

    metadata = core.sampling_plan_metadata(
        urdf, declared_dof=2, expected_sha256=URDF_SHA256
    )
    plan = core.compile_joint_sampling_plan(core.parse_urdf_joints(urdf))
    job = v2._build_job(row)

    assert metadata == {
        "independent_dof_count": 2,
        "range_evaluable_independent_dof_count": 2,
        "mimic_joint_count": 0,
        "fixed_root_joint_count": 0,
        "joint_sampling_plan_sha256": (
            "09eef60001754a8a65527dcdd1c0515e241ee0818b3f84b32aaed5ee7d59a1b1"
        ),
        "sampling_plan_error": None,
    }
    assert job["input_identity_sha256"] == (
        "75951cc80ee9cc0ea033a7c704d0e7bdafa24115e8f951b4752701a949ee8e9a"
    )
    assert not any(
        key in job
        for key in (
            "external_joint_constraints",
            "external_joint_constraint_count",
            "native_mimic_joint_count",
            "kinematic_constraint_binding",
        )
    )
    assert not any(
        key in plan
        for key in (
            "external_joint_constraints",
            "external_joint_constraint_count",
            "native_mimic_joint_count",
        )
    )


def test_v3_binds_rack_ratio_and_samples_only_the_rack_driver() -> None:
    row = _row()
    job = v3._build_job(row)

    assert job["protocol_id"] == v3.PROTOCOL_ID
    assert job["independent_dof_count"] == 1
    assert job["mimic_joint_count"] == 1
    assert job["native_mimic_joint_count"] == 0
    assert job["external_joint_constraint_count"] == 1
    assert job["single_state_expected"] == 21
    assert job["sobol_state_expected"] == 64
    assert job["collision_oracle"] == v3.table4.COLLISION_ORACLE_ZERO_MARGIN
    assert job["zero_margin_oracle_sha256"] == hashlib.sha256(
        v3.table4.ZERO_MARGIN_ORACLE_SCRIPT.read_bytes()
    ).hexdigest()
    assert set(job["execution_source_hashes"]) == {
        "adapter",
        "base_adapter",
        "table4_runner",
        "table4_core",
        "pva_roster_verifier",
        "pva_table1234_runner",
        "table123_common",
        "kinematic_constraint_registry",
        "zero_margin_oracle",
    }
    assert job["execution_source_hashes_sha256"] == v3.table4.canonical_sha256(
        job["execution_source_hashes"]
    )
    binding = job["kinematic_constraint_binding"]
    assert binding["urdf_sha256"] == URDF_SHA256
    assert binding["template_sha256"] == (
        "303bd0b9e915322eb50ccb6c02554ea4025be929412d87c9d98e48c1928de28e"
    )
    assert binding["pinion_teeth"] == 18
    assert binding["phase_offset_rad"] == 0.0
    assert binding["external_joint_constraints"] == job[
        "external_joint_constraints"
    ]

    plan = core.compile_joint_sampling_plan(
        core.parse_urdf_joints(Path(str(row["primary_urdf_path"]))),
        external_joint_constraints=job["external_joint_constraints"],
    )
    assert core.expand_joint_values(plan, [0.03]) == pytest.approx(
        [0.03, 0.5511568609208269]
    )


def test_v3_dual_pinion_uses_one_driver_and_two_derived_followers() -> None:
    job = v3._build_job(_dual_row())

    assert job["independent_dof_count"] == 1
    assert job["external_joint_constraint_count"] == 2
    assert [
        constraint["follower_joint"]
        for constraint in job["external_joint_constraints"]
    ] == ["pinion_spin_0", "pinion_spin_1"]
    plan = core.compile_joint_sampling_plan(
        core.parse_urdf_joints(DUAL_PACKAGE / "model.urdf"),
        external_joint_constraints=job["external_joint_constraints"],
    )
    follower_value = 0.02 / (0.019 * 22 / (2.0 * math.pi))
    assert core.expand_joint_values(plan, [0.02]) == pytest.approx(
        [0.02, follower_value, follower_value]
    )


def test_v3_accepts_template_minimum_pinion_limit_without_changing_ratio() -> None:
    package = PACKAGE.parent / "seed_0096"
    if not (package / "model.urdf").is_file():
        pytest.skip("frozen minimum-limit PV-A rack fixture is unavailable")
    row = {
        "ordinal": 225143,
        "asset_id": "PV-A/rack_and_pinion_slider/seed_0096",
        "category": "rack_and_pinion_slider",
        "raw_category": "rack_and_pinion_slider",
        "joint_count": 3,
        "source_path": str(package),
        "primary_urdf_path": str(package / "model.urdf"),
        "primary_urdf_relative_path": (
            "extracted/rack_and_pinion_slider/seed_0096/model.urdf"
        ),
        "primary_urdf_sha256": (
            "a1354b0ddf7b367ae4898e43f745035c12b211778302a1c09360286721052661"
        ),
        "package_binding_sha256": (
            "ef3010f41698e8670dd7b1ae9167947f4733df2181e8cd624520fe08412de9c6"
        ),
        "overrides_json": json.dumps(
            {
                "drive_skeleton": "moving_rack_dual",
                "envelope_carriage": "bare_open",
                "pinion_teeth": 25,
                "rack_form": "flat_bar_rack",
                "rack_tooth_count": 12,
            },
            sort_keys=True,
        ),
    }

    job = v3._build_job(row)

    assert job["independent_dof_count"] == 1
    assert job["external_joint_constraint_count"] == 2
    multiplier = 2.0 * math.pi / (0.019 * 25)
    assert {
        constraint["multiplier"]
        for constraint in job["external_joint_constraints"]
    } == {multiplier}


@pytest.mark.parametrize(
    "row",
    [
        pytest.param(
            lambda: _single_variant_row(
                seed=1,
                ordinal=225048,
                drive_skeleton="moving_rack_cantilever",
                pinion_teeth=19,
                urdf_sha256=(
                    "2204966387bda0956c8a4ed9505e87c974813ba1e8334141e4b0f43c9c0d3d3a"
                ),
                package_binding_sha256=(
                    "d955b2b5993240e60f99262bf1a339940de4d638d21ea51f48cbc3c2150a11e3"
                ),
                envelope_carriage="linear_stage_table",
                rack_form="round_shaft_rack",
                rack_tooth_count=13,
            ),
            id="moving_rack_cantilever",
        ),
        pytest.param(
            lambda: _single_variant_row(
                seed=3,
                ordinal=225050,
                drive_skeleton="traveling_pinion",
                pinion_teeth=21,
                urdf_sha256=(
                    "0102b8e242ccb03b8d95c4366c3a5455ba9fe183bac51a77b443209e0ed803c6"
                ),
                package_binding_sha256=(
                    "b27a4c42c71814cbb8560155e318ce90c30833e299e73568793c01c5e6f8f7aa"
                ),
                envelope_carriage="bare_open",
                rack_form="flat_bar_rack",
                rack_tooth_count=15,
            ),
            id="traveling_pinion",
        ),
    ],
)
def test_v3_registered_single_pinion_drive_skeletons(
    row: object,
) -> None:
    job = v3._build_job(row())

    assert job["independent_dof_count"] == 1
    assert job["external_joint_constraint_count"] == 1
    assert job["external_joint_constraints"][0]["follower_joint"] == (
        "pinion_spin"
    )


def test_v3_does_not_infer_constraint_from_joint_names_alone() -> None:
    row = _row()
    row["category"] = "unregistered_fixture"
    row["raw_category"] = "unregistered_fixture"

    assert v3._kinematic_constraint_binding(
        row,
        package=PACKAGE,
        urdf=PACKAGE / "model.urdf",
    ) is None


def test_v3_rejects_axis_drift_even_when_hash_bindings_are_updated(
    tmp_path: Path,
) -> None:
    row = _row()
    package = tmp_path / "package"
    shutil.copytree(PACKAGE, package)
    urdf = package / "model.urdf"
    urdf.write_text(
        urdf.read_text(encoding="utf-8").replace(
            '<axis xyz="1 0 0" />', '<axis xyz="0 1 0" />', 1
        ),
        encoding="utf-8",
    )
    urdf_sha256 = hashlib.sha256(urdf.read_bytes()).hexdigest()
    physics_path = package / "physics.json"
    physics = json.loads(physics_path.read_text(encoding="utf-8"))
    physics["model_urdf_sha256"] = urdf_sha256
    physics_path.write_text(json.dumps(physics), encoding="utf-8")
    row["source_path"] = str(package)
    row["primary_urdf_path"] = str(urdf)
    row["primary_urdf_sha256"] = urdf_sha256

    with pytest.raises(ValueError, match="driver axis mismatch"):
        v3._kinematic_constraint_binding(row, package=package, urdf=urdf)


def test_v3_registry_snapshot_detects_file_drift(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    registry = tmp_path / "registry.json"
    payload = v3.KINEMATIC_CONSTRAINT_REGISTRY.read_bytes()
    registry.write_bytes(payload)
    monkeypatch.setattr(v3, "KINEMATIC_CONSTRAINT_REGISTRY", registry)
    v3._kinematic_constraint_registry_snapshot.cache_clear()
    try:
        frozen_registry = v3._kinematic_constraint_registry()
        frozen_sha256 = v3._kinematic_constraint_registry_sha256()
        registry.write_bytes(payload + b"\n")

        assert v3._kinematic_constraint_registry() is frozen_registry
        assert v3._kinematic_constraint_registry_sha256() == frozen_sha256
        with pytest.raises(ValueError, match="registry changed during"):
            v3._verify_kinematic_constraint_registry_sha256(frozen_sha256)
    finally:
        v3._kinematic_constraint_registry_snapshot.cache_clear()


def test_v3_runtime_preserves_constraint_provenance() -> None:
    pytest.importorskip("pybullet")
    job = v3._build_job(_row())

    result = v3.table4.evaluate_job(job)
    validated = v3._validate_result(result, job)

    assert validated["status"] == "completed"
    assert validated["measurement_complete"] is True
    assert validated["independent_dof_count"] == 1
    assert validated["external_joint_constraint_count"] == 1
    assert validated["collision_oracle"] == (
        v3.table4.COLLISION_ORACLE_ZERO_MARGIN
    )
    assert validated["zero_margin_oracle_sha256"] == job[
        "zero_margin_oracle_sha256"
    ]
    assert len(validated["state_records"]) == 86
    assert {
        state["joint_name"]
        for state in validated["state_records"]
        if state["phase"] == "single_joint_sweep"
    } == {"rack_slide"}
    assert all(
        state["kinematic_constraint_binding_sha256"]
        == job["kinematic_constraint_binding_sha256"]
        for state in validated["state_records"]
    )
    assert all(
        state["collision_oracle"] == v3.table4.COLLISION_ORACLE_ZERO_MARGIN
        for state in validated["state_records"]
    )
    assert validated["execution_source_hashes"] == job[
        "execution_source_hashes"
    ]
    assert all(
        state["execution_source_hashes_sha256"]
        == job["execution_source_hashes_sha256"]
        for state in validated["state_records"]
    )
    assert all(
        "execution_source_hashes" not in state
        for state in validated["state_records"]
    )

    tampered = json.loads(json.dumps(validated))
    tampered["state_records"][0]["schema_version"] = "table4_state_v2"
    tampered["state_records_sha256"] = v3.table4.canonical_sha256(
        tampered["state_records"]
    )
    with pytest.raises(ValueError, match="state 0 binding mismatch for schema_version"):
        v3._validate_result(tampered, job)

    tampered_sources = json.loads(json.dumps(validated))
    tampered_sources["execution_source_hashes"]["table4_core"] = "0" * 64
    tampered_sources["execution_source_hashes_sha256"] = (
        v3.table4.canonical_sha256(
            tampered_sources["execution_source_hashes"]
        )
    )
    with pytest.raises(ValueError, match="execution_source_hashes"):
        v3._validate_result(tampered_sources, job)


@pytest.mark.parametrize(
    ("source_key", "source_path"),
    (
        ("adapter", v3.SCRIPT),
        ("pva_table1234_runner", Path(v3.pva_run.__file__)),
        ("table123_common", Path(v3.pva_run.common.__file__)),
    ),
)
def test_v3_child_source_check_rejects_any_pva_source_drift(
    monkeypatch: pytest.MonkeyPatch, source_key: str, source_path: Path
) -> None:
    job = v3._build_job(_row())
    original = v3.table4.sha256_file

    def drift_selected_source(path: Path) -> str:
        if Path(path) == source_path:
            return "0" * 64
        return original(path)

    monkeypatch.setattr(v3.table4, "sha256_file", drift_selected_source)
    with pytest.raises(
        ValueError, match=f"execution source changed: {source_key}"
    ):
        v3.table4._validate_v3_execution_source_binding(
            job, verify_current_files=True
        )


@pytest.mark.parametrize(
    "missing_key", ("pva_table1234_runner", "table123_common")
)
def test_v3_manifest_requires_every_runtime_source(missing_key: str) -> None:
    source_hashes = v3._source_hashes()
    del source_hashes[missing_key]
    manifest = {
        "schema_version": v3.RUN_SCHEMA_VERSION,
        "protocol_id": v3.PROTOCOL_ID,
        "protocol": {
            "collision_oracle": v3.table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": source_hashes["zero_margin_oracle"],
            "external_kinematic_constraints": {
                "registry_sha256": source_hashes[
                    "kinematic_constraint_registry"
                ],
            },
        },
        "source_hashes": source_hashes,
        "execution_source_hashes_sha256": v3.table4.canonical_sha256(
            source_hashes
        ),
    }
    manifest["manifest_content_sha256"] = v2._self_hash(
        manifest, "manifest_content_sha256"
    )

    with pytest.raises(ValueError, match="execution source set mismatch"):
        v3._manifest_bindings(manifest)


def test_v3_finalize_rejects_a_persisted_source_integrity_marker(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    connection = sqlite3.connect(":memory:")
    connection.execute(
        "CREATE TABLE results (ordinal INTEGER, asset_id TEXT, record_json TEXT)"
    )
    connection.execute(
        "INSERT INTO results VALUES (?, ?, ?)",
        (
            7,
            "PV-A/fixture/seed_0007",
            json.dumps(
                {
                    "execution_source_integrity": "failed",
                    "issues": [v3.table4.EXECUTION_SOURCE_INTEGRITY_FATAL],
                }
            ),
        ),
    )
    source_hashes = v3._source_hashes()
    manifest = {
        "schema_version": v3.RUN_SCHEMA_VERSION,
        "protocol_id": v3.PROTOCOL_ID,
        "protocol": {
            "collision_oracle": v3.table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": source_hashes["zero_margin_oracle"],
            "external_kinematic_constraints": {
                "registry_sha256": source_hashes[
                    "kinematic_constraint_registry"
                ],
            },
        },
        "source_hashes": source_hashes,
        "execution_source_hashes_sha256": v3.table4.canonical_sha256(
            source_hashes
        ),
    }
    manifest["manifest_content_sha256"] = v2._self_hash(
        manifest, "manifest_content_sha256"
    )
    base_called = False

    def forbidden_finalize(*args: object, **kwargs: object) -> dict[str, object]:
        nonlocal base_called
        del args, kwargs
        base_called = True
        return {}

    monkeypatch.setattr(v3, "_V2_FINALIZE", forbidden_finalize)
    with pytest.raises(
        RuntimeError, match=v3.table4.EXECUTION_SOURCE_INTEGRITY_FATAL
    ):
        v3._finalize(
            connection,
            tmp_path,
            manifest,
            n_eval=1,
            j_eval=0,
            category_count=1,
        )
    assert base_called is False
    connection.close()


@pytest.mark.parametrize(
    "drift_key", ("pva_table1234_runner", "table123_common")
)
def test_v3_finalize_rechecks_new_runtime_sources_before_publication(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, drift_key: str
) -> None:
    connection = sqlite3.connect(":memory:")
    connection.execute(
        "CREATE TABLE results (ordinal INTEGER, asset_id TEXT, record_json TEXT)"
    )
    connection.execute(
        "INSERT INTO results VALUES (0, 'PV-A/fixture/seed_0000', '{}')"
    )
    source_hashes = v3._source_hashes()
    manifest = {
        "schema_version": v3.RUN_SCHEMA_VERSION,
        "protocol_id": v3.PROTOCOL_ID,
        "protocol": {
            "collision_oracle": v3.table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": source_hashes["zero_margin_oracle"],
            "external_kinematic_constraints": {
                "registry_sha256": source_hashes[
                    "kinematic_constraint_registry"
                ],
            },
        },
        "source_hashes": source_hashes,
        "execution_source_hashes_sha256": v3.table4.canonical_sha256(
            source_hashes
        ),
    }
    manifest["manifest_content_sha256"] = v2._self_hash(
        manifest, "manifest_content_sha256"
    )
    observed = dict(source_hashes)
    observed[drift_key] = "0" * 64
    monkeypatch.setattr(v3, "_compute_source_hashes", lambda: observed)
    base_called = False

    def forbidden_finalize(*args: object, **kwargs: object) -> dict[str, object]:
        nonlocal base_called
        del args, kwargs
        base_called = True
        return {}

    monkeypatch.setattr(v3, "_V2_FINALIZE", forbidden_finalize)
    with pytest.raises(ValueError, match=drift_key):
        v3._finalize(
            connection,
            tmp_path,
            manifest,
            n_eval=1,
            j_eval=0,
            category_count=1,
        )
    assert base_called is False
    connection.close()


def test_v3_resume_prefix_rejects_stale_record_identity(tmp_path: Path) -> None:
    row = _row()
    row["ordinal"] = 0
    job = v3._build_job(row)
    record = v3.table4._empty_record(job, "fixture terminal record")
    states = record.pop("state_records")
    assert states == []

    source = sqlite3.connect(tmp_path / "source.sqlite3")
    result_path = tmp_path / "run" / "results.sqlite3"
    result_path.parent.mkdir()
    result = sqlite3.connect(result_path)
    try:
        source.execute(
            "CREATE TABLE assets (ordinal INTEGER PRIMARY KEY, asset_id TEXT, "
            "category TEXT, joint_count INTEGER, row_sha256 TEXT, row_json TEXT)"
        )
        row_payload = v2._canonical_text(row)
        source.execute(
            "INSERT INTO assets VALUES (?, ?, ?, ?, ?, ?)",
            (
                0,
                row["asset_id"],
                row["raw_category"],
                row["joint_count"],
                v2._canonical_sha256(row),
                row_payload,
            ),
        )
        source.commit()
        v2._create_schema(result)
        result.execute(
            "INSERT INTO results VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (
                0,
                row["asset_id"],
                v2._canonical_text(record),
                zlib.compress(b""),
                0,
                "completed",
                0.0,
                "2026-08-29T00:00:00Z",
            ),
        )
        result.commit()

        source_hashes = v3._source_hashes()
        oracle_sha256 = source_hashes["zero_margin_oracle"]
        registry_sha256 = source_hashes["kinematic_constraint_registry"]
        manifest = {
            "schema_version": v3.RUN_SCHEMA_VERSION,
            "protocol_id": v3.PROTOCOL_ID,
            "protocol": {
                "collision_oracle": v3.table4.COLLISION_ORACLE_ZERO_MARGIN,
                "zero_margin_oracle_sha256": oracle_sha256,
                "external_kinematic_constraints": {
                    "registry_sha256": registry_sha256,
                },
            },
            "source_hashes": source_hashes,
            "execution_source_hashes_sha256": v3.table4.canonical_sha256(
                source_hashes
            ),
            "package_root_binding": None,
        }
        manifest["manifest_content_sha256"] = v2._self_hash(
            manifest, "manifest_content_sha256"
        )
        v2._atomic_json(result_path.parent / "manifest.json", manifest)

        assert v3._validated_result_prefix(result, source) == 1
        stale = dict(record)
        stale["input_identity_sha256"] = "0" * 64
        result.execute(
            "UPDATE results SET record_json = ? WHERE ordinal = 0",
            (v2._canonical_text(stale),),
        )
        result.commit()
        with pytest.raises(ValueError, match="input_identity_sha256"):
            v3._validated_result_prefix(result, source)
    finally:
        result.close()
        source.close()


def test_v3_run_restores_v2_module_and_seals_combined_receipt(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    names = (
        "RUN_SCHEMA_VERSION",
        "RESULT_DB_SCHEMA_VERSION",
        "RECEIPT_SCHEMA_VERSION",
        "PROTOCOL_ID",
        "_job_with_plan",
        "_build_job",
        "_validate_result",
        "_validated_result_prefix",
        "_source_hashes",
        "_manifest",
        "_finalize",
        "_write_progress",
        "_artifact_manifest",
    )
    original = {name: getattr(v2, name) for name in names}
    output = tmp_path / "output"

    def fake_run(*args: object, **kwargs: object) -> Path:
        del args, kwargs
        assert v2.PROTOCOL_ID == v3.PROTOCOL_ID
        assert v2._build_job is v3._build_job
        output.mkdir()
        source_hashes = v3._source_hashes()
        oracle_sha256 = source_hashes["zero_margin_oracle"]
        registry_sha256 = source_hashes["kinematic_constraint_registry"]
        manifest = {
            "schema_version": v3.RUN_SCHEMA_VERSION,
            "protocol_id": v3.PROTOCOL_ID,
            "protocol": {
                "collision_oracle": v3.table4.COLLISION_ORACLE_ZERO_MARGIN,
                "zero_margin_oracle_sha256": oracle_sha256,
                "external_kinematic_constraints": {
                    "registry_sha256": registry_sha256,
                },
            },
            "source_hashes": source_hashes,
            "execution_source_hashes_sha256": v3.table4.canonical_sha256(
                source_hashes
            ),
        }
        manifest["manifest_content_sha256"] = v2._self_hash(
            manifest, "manifest_content_sha256"
        )
        v2._atomic_json(output / "manifest.json", manifest)
        receipt = {
            "schema_version": v3.RECEIPT_SCHEMA_VERSION,
            "protocol_id": v3.PROTOCOL_ID,
        }
        receipt["receipt_content_sha256"] = v2._self_hash(
            receipt, "receipt_content_sha256"
        )
        v2._atomic_json(output / "full_release_receipt.json", receipt)
        return output

    monkeypatch.setattr(v3, "_V2_RUN", fake_run)
    assert v3.run_pva_table4_v3(Path("source"), output) == output

    for name, value in original.items():
        assert getattr(v2, name) is value
    receipt = v2._load_json(output / "full_release_receipt.json")
    assert receipt["collision_oracle"] == v3.table4.COLLISION_ORACLE_ZERO_MARGIN
    assert receipt["zero_margin_oracle_sha256"] == v3._zero_margin_oracle_sha256()
    assert receipt["kinematic_constraint_registry_sha256"] == hashlib.sha256(
        v3.KINEMATIC_CONSTRAINT_REGISTRY.read_bytes()
    ).hexdigest()
    assert receipt["execution_source_hashes_sha256"] == v3.table4.canonical_sha256(
        v3._source_hashes()
    )
    assert receipt["receipt_content_sha256"] == v2._self_hash(
        receipt, "receipt_content_sha256"
    )
    assert v3._source_hashes()["base_adapter"] == hashlib.sha256(
        Path(v2.__file__).read_bytes()
    ).hexdigest()
    assert v3._source_hashes()["pva_table1234_runner"] == hashlib.sha256(
        Path(v3.pva_run.__file__).read_bytes()
    ).hexdigest()
    assert v3._source_hashes()["table123_common"] == hashlib.sha256(
        Path(v3.pva_run.common.__file__).read_bytes()
    ).hexdigest()


def test_v3_run_restores_v2_module_after_exception(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    original_protocol = v2.PROTOCOL_ID
    original_build_job = v2._build_job

    def fail_run(*args: object, **kwargs: object) -> Path:
        del args, kwargs
        assert v2.PROTOCOL_ID == v3.PROTOCOL_ID
        raise RuntimeError("fixture failure")

    monkeypatch.setattr(v3, "_V2_RUN", fail_run)
    with pytest.raises(RuntimeError, match="fixture failure"):
        v3.run_pva_table4_v3(Path("source"), tmp_path / "output")

    assert v2.PROTOCOL_ID == original_protocol
    assert v2._build_job is original_build_job
