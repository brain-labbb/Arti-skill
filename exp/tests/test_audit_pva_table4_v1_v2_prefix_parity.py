from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
import zlib

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/audit_pva_table4_v1_v2_prefix_parity.py"
SPEC = importlib.util.spec_from_file_location("table4_parity_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = AUDIT
SPEC.loader.exec_module(AUDIT)

NON_MIMIC_URDF = """<robot name="plain">
<link name="base"/><link name="door"/>
<joint name="root" type="revolute"><parent link="base"/><child link="door"/>
<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>\n"""
MIMIC_URDF = """<robot name="mimic">
<link name="base"/><link name="left"/><link name="right"/>
<joint name="root" type="revolute"><parent link="base"/><child link="left"/>
<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
<joint name="follower" type="revolute"><parent link="base"/><child link="right"/>
<limit lower="-1" upper="1" effort="1" velocity="1"/>
<mimic joint="root" multiplier="-1" offset="0"/></joint></robot>\n"""


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="ascii")


def _self_hashed(value: dict[str, object], field: str) -> dict[str, object]:
    result = dict(value)
    result[field] = AUDIT.canonical_sha256(result)
    return result


def _state(
    ordinal: int,
    phase: str,
    sample: int,
    *,
    version: int,
    input_identity: str,
    plan_sha: str | None,
) -> dict[str, object]:
    value: dict[str, object] = {
        "all_pair_contact_count": 1,
        "all_pair_illegal_penetration_count": 0,
        "all_pair_max_penetration_m": 0.0,
        "category": "Category",
        "dataset": "pva",
        "dataset_id": f"PV-A/Category/seed_{ordinal:04d}",
        "input_identity_sha256": input_identity,
        "joint_name": "root" if phase == "single_joint_sweep" else None,
        "joint_values_sha256": "3" * 64,
        "metric_max_penetration_m": 0.0,
        "non_adjacent_contact_count": 0,
        "non_adjacent_illegal_penetration_count": 0,
        "non_adjacent_max_penetration_m": 0.0,
        "order": ordinal,
        "phase": phase,
        "protocol_id": f"urdf_sim_ready_table4_pva_full_release_v{version}",
        "reset_readback_max_abs_error": 0.0,
        "sample_index": sample,
        "schema_version": f"table4_state_v{version}",
    }
    if version == 2:
        value.update(
            {
                "joint_sampling_plan_sha256": plan_sha,
                "sampling_protocol": AUDIT.V2_SAMPLING_PROTOCOL,
            }
        )
    return value


def _states(
    ordinal: int,
    single_count: int,
    *,
    version: int,
    input_identity: str,
    plan_sha: str | None,
) -> list[dict[str, object]]:
    values = [
        _state(
            ordinal, "rest", 0, version=version,
            input_identity=input_identity, plan_sha=plan_sha,
        )
    ]
    values += [
        _state(
            ordinal, "single_joint_sweep", index % AUDIT.table4.SINGLE_SAMPLES,
            version=version, input_identity=input_identity, plan_sha=plan_sha,
        )
        for index in range(single_count)
    ]
    values += [
        _state(
            ordinal, "multi_joint_sobol", index, version=version,
            input_identity=input_identity, plan_sha=plan_sha,
        )
        for index in range(AUDIT.table4.SOBOL_SAMPLES)
    ]
    return values


def _blob(states: list[dict[str, object]]) -> bytes:
    payload = "".join(AUDIT.canonical_text(row) + "\n" for row in states)
    return zlib.compress(payload.encode("ascii"))


def _apply_schedule(
    states: list[dict[str, object]], schedule: list[dict[str, object]]
) -> None:
    assert len(states) == len(schedule)
    for state, expected in zip(states, schedule, strict=True):
        state.update(expected)


def _record(
    row: dict[str, object],
    *,
    version: int,
    runtime: dict[str, object],
    states: list[dict[str, object]],
    input_identity: str,
) -> dict[str, object]:
    single = sum(state["phase"] == "single_joint_sweep" for state in states)
    sobol = sum(state["phase"] == "multi_joint_sobol" for state in states)
    return {
        "category": "Category",
        "collision_mesh_references": 1,
        "collision_metric_status": AUDIT.table4.MEASURED_COLLISION_GEOMETRY,
        "dataset": "pva",
        "dataset_id": row["asset_id"],
        "expected_movable_joints": row["joint_count"],
        "expected_primary_urdf_sha256": row["primary_urdf_sha256"],
        "geometry_kinds": ["mesh"],
        "input_identity_sha256": input_identity,
        "issues": [],
        "joint_single_sweep_cf_passed": int(row["joint_count"]),
        "link_count": int(row["joint_count"]) + 1,
        "load_success": True,
        "max_penetration_m": 0.0,
        "max_penetration_normalized": 0.0,
        "max_reset_readback_error": 0.0,
        "measurement_complete": True,
        "missing_collision_mesh_references": 0,
        "movable_dof_count": row["joint_count"],
        "multi_joint_sobol_cf": True,
        "native_collision_elements": 1,
        "object_bbox_diagonal_m": 1.0,
        "order": row["ordinal"],
        "package": row["source_path"],
        "package_binding_sha256": row["package_binding_sha256"],
        "package_binding_verified": True,
        "primary_urdf_relative_path": row["primary_urdf_relative_path"],
        "protocol_id": f"urdf_sim_ready_table4_pva_full_release_v{version}",
        "range_evaluable_dof_count": row["joint_count"],
        "rest_all_pair_cf": True,
        "rest_non_adjacent_cf": True,
        "rest_non_adjacent_free": 1,
        "rest_state_executed": 1,
        "rest_state_expected": 1,
        "runtime_identity": runtime,
        "schema_version": f"table4_full_release_run_v{version}",
        "single_joint_sweep_cf": True,
        "single_non_adjacent_free": single,
        "single_state_executed": single,
        "single_state_expected": single,
        "sobol_non_adjacent_free": sobol,
        "sobol_state_executed": sobol,
        "sobol_state_expected": sobol,
        "state_records_count": len(states),
        "state_records_sha256": AUDIT.canonical_sha256(states),
        "status": "completed",
        "strict_collision_pass": True,
        "unexecuted_state_count": 0,
        "urdf_path": row["primary_urdf_path"],
    }


def _retained_zero_state_error(record: dict[str, object]) -> None:
    expected_total = sum(
        int(record[field])
        for field in (
            "rest_state_expected",
            "single_state_expected",
            "sobol_state_expected",
        )
    )
    record.update(
        {
            "collision_metric_status": "N/E",
            "issues": ["error: getJointState failed."],
            "joint_single_sweep_cf_passed": 0,
            "max_penetration_m": None,
            "max_penetration_normalized": None,
            "max_reset_readback_error": None,
            "measurement_complete": False,
            "multi_joint_sobol_cf": False,
            "rest_all_pair_cf": False,
            "rest_non_adjacent_cf": False,
            "rest_non_adjacent_free": 0,
            "rest_state_executed": 0,
            "single_joint_sweep_cf": False,
            "single_non_adjacent_free": 0,
            "single_state_executed": 0,
            "sobol_non_adjacent_free": 0,
            "sobol_state_executed": 0,
            "state_records_count": 0,
            "state_records_sha256": AUDIT.canonical_sha256([]),
            "status": "error",
            "strict_collision_pass": False,
            "unexecuted_state_count": expected_total,
        }
    )


def _retained_partial_state_error(
    record: dict[str, object],
    states: list[dict[str, object]],
    *,
    keep_count: int = 2,
) -> None:
    del states[keep_count:]
    rest = states[0]
    single = [
        state for state in states if state["phase"] == "single_joint_sweep"
    ]
    expected_total = sum(
        int(record[field])
        for field in (
            "rest_state_expected",
            "single_state_expected",
            "sobol_state_expected",
        )
    )
    rest_all_free = int(rest["all_pair_illegal_penetration_count"]) == 0
    rest_nonadj_free = int(rest["non_adjacent_illegal_penetration_count"]) == 0
    single_free = sum(
        int(state["non_adjacent_illegal_penetration_count"]) == 0
        for state in single
    )
    joint_rows: dict[str, list[dict[str, object]]] = {}
    for state in single:
        joint_rows.setdefault(str(state["joint_name"]), []).append(state)
    joint_passed = sum(
        len(rows) == AUDIT.table4.SINGLE_SAMPLES
        and all(
            int(state["non_adjacent_illegal_penetration_count"]) == 0
            for state in rows
        )
        for rows in joint_rows.values()
    )
    sobol = [state for state in states if state["phase"] == "multi_joint_sobol"]
    sobol_free = sum(
        int(state["non_adjacent_illegal_penetration_count"]) == 0
        for state in sobol
    )
    record.update(
        {
            "collision_metric_status": "partial",
            "issues": [
                "RuntimeError: reset/readback error 0.008 exceeds 1e-09"
            ],
            "joint_single_sweep_cf_passed": joint_passed,
            "max_penetration_m": None,
            "max_penetration_normalized": None,
            "max_reset_readback_error": None,
            "measurement_complete": False,
            "multi_joint_sobol_cf": False,
            "rest_all_pair_cf": rest_all_free,
            "rest_non_adjacent_cf": rest_nonadj_free,
            "rest_non_adjacent_free": int(rest_nonadj_free),
            "rest_state_executed": 1,
            "single_joint_sweep_cf": False,
            "single_non_adjacent_free": single_free,
            "single_state_executed": len(single),
            "sobol_non_adjacent_free": sobol_free,
            "sobol_state_executed": len(sobol),
            "state_records_count": len(states),
            "state_records_sha256": AUDIT.canonical_sha256(states),
            "status": "error",
            "strict_collision_pass": False,
            "unexecuted_state_count": expected_total - len(states),
        }
    )


def _retained_v1_parent_error(record: dict[str, object]) -> None:
    for field in (
        "collision_mesh_references",
        "geometry_kinds",
        "link_count",
        "missing_collision_mesh_references",
        "state_records_count",
        "unexecuted_state_count",
    ):
        record.pop(field, None)
    record.update(
        {
            "collision_metric_status": "N/E",
            "issues": [
                "parent_executor_exception: BlockingIOError: [Errno 11] Resource temporarily unavailable"
            ],
            "joint_single_sweep_cf_passed": 0,
            "load_success": False,
            "max_penetration_m": None,
            "max_penetration_normalized": None,
            "max_reset_readback_error": None,
            "measurement_complete": False,
            "multi_joint_sobol_cf": None,
            "native_collision_elements": 0,
            "object_bbox_diagonal_m": None,
            "package_binding_verified": False,
            "range_evaluable_dof_count": 0,
            "rest_all_pair_cf": None,
            "rest_non_adjacent_cf": None,
            "rest_non_adjacent_free": 0,
            "rest_state_executed": 0,
            "runtime_identity": None,
            "single_joint_sweep_cf": None,
            "single_non_adjacent_free": 0,
            "single_state_executed": 0,
            "sobol_non_adjacent_free": 0,
            "sobol_state_executed": 0,
            "state_records": [],
            "state_records_sha256": AUDIT.canonical_sha256([]),
            "status": "error",
            "strict_collision_pass": None,
        }
    )


def _retained_v1_timeout(record: dict[str, object]) -> None:
    _retained_v1_parent_error(record)
    record.pop("state_records")
    record["issues"] = ["asset_timeout_after_600_seconds"]


def _meta(connection: sqlite3.Connection, values: dict[str, object]) -> None:
    connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
    connection.executemany(
        "INSERT INTO meta VALUES(?, ?)",
        [(key, AUDIT.canonical_text(value)) for key, value in values.items()],
    )


def _build_fixture(
    tmp_path: Path,
    *,
    mimic_retained_error: bool = False,
    mimic_partial_error: bool = False,
    mimic_partial_after_single_error: bool = False,
    v1_retained_non_mimic: bool = False,
    v1_retained_worker_status: str = "parent_error",
    v1_retained_mutation: dict[str, object] | None = None,
) -> dict[str, Path]:
    assert sum(
        map(
            int,
            (
                mimic_retained_error,
                mimic_partial_error,
                mimic_partial_after_single_error,
            ),
        )
    ) <= 1
    assert v1_retained_non_mimic or v1_retained_mutation is None
    assert v1_retained_worker_status in {"parent_error", "timeout"}
    v1_root, v2_root, mirror = tmp_path / "v1", tmp_path / "v2", tmp_path / "mirror"
    v1_root.mkdir(); v2_root.mkdir(); mirror.mkdir()
    logical_root = Path("/sealed/PV-A/extracted")
    rows: list[dict[str, object]] = []
    for ordinal, (urdf_text, joints) in enumerate(((NON_MIMIC_URDF, 1), (MIMIC_URDF, 2))):
        relative = Path("Category") / f"seed_{ordinal:04d}"
        package = mirror / relative
        package.mkdir(parents=True)
        urdf = package / "model.urdf"
        urdf.write_text(urdf_text, encoding="ascii")
        rows.append(
            {
                "ordinal": ordinal,
                "asset_id": f"PV-A/Category/seed_{ordinal:04d}",
                "raw_category": "Category",
                "category": "Category",
                "joint_count": joints,
                "source_path": str(logical_root / relative),
                "primary_urdf_path": str(logical_root / relative / "model.urdf"),
                "primary_urdf_relative_path": (Path("extracted") / relative / "model.urdf").as_posix(),
                "primary_urdf_sha256": AUDIT.sha256_file(urdf),
                "package_binding_sha256": ("8" if ordinal == 0 else "9") * 64,
            }
        )

    roster = _self_hashed(
        {
            "schema_version": "pva_table1234_full_release_roster_v1",
            "source_bindings": {"extracted_root": str(logical_root)},
        },
        "manifest_content_sha256",
    )
    roster_path = v1_root / "roster_manifest.json"
    _write_json(roster_path, roster)
    roster_sha = AUDIT.sha256_file(roster_path)
    v1_runtime = {"runner": "sealed-v1"}
    v1_manifest = _self_hashed(
        {
            "schema_version": AUDIT.V1_MANIFEST_SCHEMA,
            "N_eval": 2,
            "J_eval": 3,
            "asset_timeout_seconds": 600.0,
            "runtime_identity": v1_runtime,
        },
        "manifest_content_sha256",
    )
    v1_manifest_path = v1_root / "manifest.json"
    _write_json(v1_manifest_path, v1_manifest)

    v1_database = v1_root / "results.sqlite3"
    connection = sqlite3.connect(v1_database)
    _meta(
        connection,
        {
            "schema_version": AUDIT.V1_DATABASE_SCHEMA,
            "selected_n": 2,
            "selected_j": 3,
            "roster_manifest_sha256": roster_sha,
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        },
    )
    connection.executescript(
        """CREATE TABLE assets(ordinal INTEGER PRIMARY KEY, asset_id TEXT UNIQUE,
        category TEXT, row_sha256 TEXT, row_json TEXT);
        CREATE TABLE results(ordinal INTEGER PRIMARY KEY, asset_id TEXT UNIQUE,
        table4_json TEXT, table4_states_zlib BLOB, table4_state_count INTEGER,
        worker_status TEXT);"""
    )
    for row in rows:
        ordinal = int(row["ordinal"])
        identity = AUDIT._expected_v1_input_identity(row)
        states = _states(
            ordinal,
            AUDIT.table4.SINGLE_SAMPLES * int(row["joint_count"]),
            version=1,
            input_identity=identity,
            plan_sha=None,
        )
        if ordinal == 0:
            physical_urdf = mirror / "Category/seed_0000/model.urdf"
            compiled = AUDIT._collision_core().compile_joint_sampling_plan(
                AUDIT._collision_core().parse_urdf_joints(physical_urdf)
            )
            _apply_schedule(
                states,
                AUDIT._expected_v2_schedule({"_compiled_plan": compiled}),
            )
        if ordinal == 1:
            states[0].update({
                "all_pair_illegal_penetration_count": 1,
                "all_pair_max_penetration_m": 2e-6,
                "non_adjacent_contact_count": 1,
                "non_adjacent_illegal_penetration_count": 1,
                "non_adjacent_max_penetration_m": 2e-6,
                "metric_max_penetration_m": 2e-6,
            })
            states[1].update({
                "non_adjacent_contact_count": 1,
                "non_adjacent_illegal_penetration_count": 1,
                "non_adjacent_max_penetration_m": 2e-6,
                "metric_max_penetration_m": 2e-6,
            })
        record = _record(row, version=1, runtime=v1_runtime, states=states, input_identity=identity)
        worker_status = "completed"
        if ordinal == 0 and v1_retained_non_mimic:
            states = []
            if v1_retained_worker_status == "parent_error":
                _retained_v1_parent_error(record)
            else:
                _retained_v1_timeout(record)
            if v1_retained_mutation:
                record.update(v1_retained_mutation)
            worker_status = v1_retained_worker_status
        if ordinal == 1:
            record.update({
                "joint_single_sweep_cf_passed": 0,
                "max_penetration_m": 2e-6,
                "max_penetration_normalized": 2e-6,
                "rest_all_pair_cf": False,
                "rest_non_adjacent_cf": False,
                "rest_non_adjacent_free": 0,
                "single_joint_sweep_cf": False,
                "single_non_adjacent_free": len(states) - 2 - AUDIT.table4.SOBOL_SAMPLES,
                "strict_collision_pass": False,
            })
            if mimic_retained_error:
                states = []
                _retained_zero_state_error(record)
            elif mimic_partial_error:
                _retained_partial_state_error(record, states)
            elif mimic_partial_after_single_error:
                _retained_partial_state_error(
                    record,
                    states,
                    keep_count=1 + int(record["single_state_expected"]) + 1,
                )
        connection.execute(
            "INSERT INTO assets VALUES(?,?,?,?,?)",
            (ordinal, row["asset_id"], "Category", AUDIT.canonical_sha256(row), AUDIT.canonical_text(row)),
        )
        connection.execute(
            "INSERT INTO results VALUES(?,?,?,?,?,?)",
            (ordinal, row["asset_id"], AUDIT.canonical_text(record), _blob(states), len(states), worker_status),
        )
    connection.commit(); connection.close()

    v1_database_sha = AUDIT.sha256_file(v1_database)
    receipt = _self_hashed(
        {
            "schema_version": AUDIT.V1_RECEIPT_SCHEMA,
            "N_eval": 2,
            "J_eval": 3,
            "result_database": "results.sqlite3",
            "result_database_bytes": v1_database.stat().st_size,
            "result_database_sha256": v1_database_sha,
            "execution_manifest": "manifest.json",
            "execution_manifest_sha256": AUDIT.sha256_file(v1_manifest_path),
            "roster_manifest": str(roster_path),
            "roster_manifest_sha256": roster_sha,
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        },
        "receipt_content_sha256",
    )
    receipt_path = v1_root / "full_release_receipt.json"
    _write_json(receipt_path, receipt)

    pybullet = tmp_path / "pybullet.so"
    pybullet.write_bytes(b"sealed fixture")
    sources = {
        "adapter": AUDIT.EXP / "scripts/run_pva_table4_mimic_aware_full_release.py",
        "pva_roster_verifier": AUDIT.EXP / "scripts/build_pva_full_release_roster.py",
        "table4_core": Path(AUDIT.table4.CORE_SCRIPT),
        "table4_runner": AUDIT.EXP / "scripts/run_table4_full_release.py",
    }
    source_hashes = {key: AUDIT.sha256_file(path) for key, path in sources.items()}
    v2_runtime = {
        "collision_core_sha256": source_hashes["table4_core"],
        "runner_sha256": source_hashes["table4_runner"],
        "pybullet_module": str(pybullet),
        "pybullet_module_sha256": AUDIT.sha256_file(pybullet),
        "scipy_version": __import__("scipy").__version__,
    }
    binding = _self_hashed(
        {
            "schema_version": AUDIT.PACKAGE_ROOT_BINDING_SCHEMA,
            "mapping_policy": AUDIT.PACKAGE_ROOT_MAPPING_POLICY,
            "logical_root": str(logical_root),
            "physical_root": str(mirror),
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
            "package_verification": AUDIT.PACKAGE_VERIFICATION,
        },
        "binding_content_sha256",
    )
    v2_manifest = _self_hashed(
        {
            "schema_version": AUDIT.V2_MANIFEST_SCHEMA,
            "classification": "FORMAL_FULL_RELEASE",
            "N_eval": 2,
            "J_eval": 3,
            "category_count": 1,
            "limit": None,
            "protocol_id": AUDIT.V2_PROTOCOL_ID,
            "sampling_protocol": AUDIT.V2_SAMPLING_PROTOCOL,
            "protocol": {
                "single_joint_samples": AUDIT.table4.SINGLE_SAMPLES,
                "sobol_samples": AUDIT.table4.SOBOL_SAMPLES,
                "sobol_seed": AUDIT.table4.SOBOL_SEED,
                "penetration_threshold_m": AUDIT.table4.PENETRATION_THRESHOLD_M,
                "mimic_constraints": "affine_expansion_from_independent_roots",
                "contact_policy": "all_non_direct-parent_pairs",
            },
            "source": {
                "N_eval": 2,
                "J_eval": 3,
                "source_receipt": str(receipt_path),
                "source_receipt_sha256": AUDIT.sha256_file(receipt_path),
                "source_receipt_content_sha256": receipt["receipt_content_sha256"],
                "source_result_database": str(v1_database),
                "source_result_database_declared_sha256": v1_database_sha,
                "roster_manifest": str(roster_path),
                "roster_manifest_sha256": roster_sha,
                "roster_manifest_content_sha256": roster["manifest_content_sha256"],
            },
            "source_hashes": source_hashes,
            "runtime_identity": v2_runtime,
            "package_root_binding": binding,
        },
        "manifest_content_sha256",
    )
    v2_manifest_path = v2_root / "manifest.json"
    _write_json(v2_manifest_path, v2_manifest)

    plan_evidence = {
        "logical_root": logical_root,
        "physical_root": mirror,
        "binding_content_sha256": binding["binding_content_sha256"],
        "v2_runtime_identity": v2_runtime,
    }
    v2_database = v2_root / "results.sqlite3"
    connection = sqlite3.connect(v2_database)
    _meta(
        connection,
        {
            "schema_version": AUDIT.V2_DATABASE_SCHEMA,
            "manifest_content_sha256": v2_manifest["manifest_content_sha256"],
            "source_receipt_content_sha256": receipt["receipt_content_sha256"],
            "source_result_database_declared_sha256": v1_database_sha,
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
            "N_eval": 2, "J_eval": 3, "limit": None,
            "sampling_protocol": AUDIT.V2_SAMPLING_PROTOCOL,
            "protocol_id": AUDIT.V2_PROTOCOL_ID,
            "package_root_binding_content_sha256": binding["binding_content_sha256"],
        },
    )
    connection.execute(
        """CREATE TABLE results(ordinal INTEGER PRIMARY KEY, asset_id TEXT UNIQUE,
        record_json TEXT, states_zlib BLOB, state_count INTEGER, worker_status TEXT)"""
    )
    for row in rows:
        ordinal = int(row["ordinal"])
        paths, plan = AUDIT._mapped_urdf_and_plan(row, plan_evidence, ordinal)
        identity = AUDIT._expected_v2_input_identity(row, plan)
        states = _states(
            ordinal,
            AUDIT.table4.SINGLE_SAMPLES * int(plan["independent_dof_count"]),
            version=2,
            input_identity=identity,
            plan_sha=str(plan["joint_sampling_plan_sha256"]),
        )
        _apply_schedule(states, AUDIT._expected_v2_schedule(plan))
        record = _record(row, version=2, runtime=v2_runtime, states=states, input_identity=identity)
        record.update(
            {
                field: plan[field]
                for field in (
                    "independent_dof_count",
                    "range_evaluable_independent_dof_count",
                    "mimic_joint_count",
                    "joint_sampling_plan_sha256",
                    "sampling_plan_error",
                )
            }
        )
        record["joint_single_sweep_cf_passed"] = int(plan["independent_dof_count"])
        record.update(
            {
                "sampling_protocol": AUDIT.V2_SAMPLING_PROTOCOL,
                "roster_ordinal": ordinal,
                "evaluation_package_relative_path": paths["evaluation_package_relative_path"],
                "evaluation_urdf_relative_path": paths["evaluation_urdf_relative_path"],
                "package_root_binding_content_sha256": binding["binding_content_sha256"],
            }
        )
        record["execution_input_sha256"] = AUDIT.canonical_sha256(
            {
                "input_identity_sha256": identity,
                "package_root_binding_content_sha256": binding["binding_content_sha256"],
                "evaluation_package_relative_path": paths["evaluation_package_relative_path"],
                "evaluation_urdf_relative_path": paths["evaluation_urdf_relative_path"],
                "package_binding_sha256": row["package_binding_sha256"],
                "expected_primary_urdf_sha256": row["primary_urdf_sha256"],
            }
        )
        if ordinal == 1 and mimic_retained_error:
            states = []
            _retained_zero_state_error(record)
        elif ordinal == 1 and mimic_partial_error:
            _retained_partial_state_error(record, states)
        elif ordinal == 1 and mimic_partial_after_single_error:
            _retained_partial_state_error(
                record,
                states,
                keep_count=1 + int(record["single_state_expected"]) + 1,
            )
        connection.execute(
            "INSERT INTO results VALUES(?,?,?,?,?,?)",
            (ordinal, row["asset_id"], AUDIT.canonical_text(record), _blob(states), len(states), "completed"),
        )
    connection.commit(); connection.close()
    return {
        "v1": v1_database, "v2": v2_database, "receipt": receipt_path,
        "v2_manifest": v2_manifest_path, "mirror": mirror,
    }


def _update_v2_record(path: Path, ordinal: int, mutation: dict[str, object]) -> None:
    connection = sqlite3.connect(path)
    raw = connection.execute("SELECT record_json FROM results WHERE ordinal=?", (ordinal,)).fetchone()[0]
    record = json.loads(raw); record.update(mutation)
    connection.execute("UPDATE results SET record_json=? WHERE ordinal=?", (AUDIT.canonical_text(record), ordinal))
    connection.commit(); connection.close()


def _delete_v2_record_field(path: Path, ordinal: int, field: str) -> None:
    connection = sqlite3.connect(path)
    raw = connection.execute(
        "SELECT record_json FROM results WHERE ordinal=?", (ordinal,)
    ).fetchone()[0]
    record = json.loads(raw)
    del record[field]
    connection.execute(
        "UPDATE results SET record_json=? WHERE ordinal=?",
        (AUDIT.canonical_text(record), ordinal),
    )
    connection.commit(); connection.close()


def _seal_v2_fixture(fixture: dict[str, Path]) -> Path:
    root = fixture["v2"].parent
    manifest = json.loads(fixture["v2_manifest"].read_text(encoding="ascii"))
    files = {
        "manifest.json": fixture["v2_manifest"],
        "results.sqlite3": fixture["v2"],
    }
    for name in (
        "protocol_snapshot.md", "records.jsonl", "asset_records.jsonl",
        "state_records.jsonl", "summary.json", "summary.md", "checkpoint.json",
    ):
        path = root / name
        path.write_text(f"sealed {name}\n", encoding="ascii")
        files[name] = path
    _write_json(files["summary.json"], {"status": "COMPLETE"})
    artifact = _self_hashed(
        {
            "schema_version": AUDIT.V2_ARTIFACT_MANIFEST_SCHEMA,
            "artifacts": [
                {
                    "path": name,
                    "bytes": path.stat().st_size,
                    "sha256": AUDIT.sha256_file(path),
                }
                for name, path in files.items()
            ],
        },
        "artifact_manifest_content_sha256",
    )
    artifact_path = root / "artifact_manifest.json"
    _write_json(artifact_path, artifact)
    receipt = _self_hashed(
        {
            "schema_version": AUDIT.V2_RECEIPT_SCHEMA,
            "classification": manifest["classification"],
            "N_eval": manifest["N_eval"],
            "J_eval": manifest["J_eval"],
            "category_count": manifest["category_count"],
            "protocol_id": AUDIT.V2_PROTOCOL_ID,
            "sampling_protocol": AUDIT.V2_SAMPLING_PROTOCOL,
            "source": manifest["source"],
            "package_root_binding": manifest["package_root_binding"],
            "manifest": "manifest.json",
            "manifest_sha256": AUDIT.sha256_file(fixture["v2_manifest"]),
            "records": "records.jsonl",
            "records_sha256": AUDIT.sha256_file(files["records.jsonl"]),
            "state_records": "state_records.jsonl",
            "state_records_sha256": AUDIT.sha256_file(files["state_records.jsonl"]),
            "summary": "summary.json",
            "summary_sha256": AUDIT.sha256_file(files["summary.json"]),
            "artifact_manifest": "artifact_manifest.json",
            "artifact_manifest_sha256": AUDIT.sha256_file(artifact_path),
            "result_database": "results.sqlite3",
            "result_database_sha256": AUDIT.sha256_file(fixture["v2"]),
            "status": "COMPLETE",
            "metrics": {},
        },
        "receipt_content_sha256",
    )
    receipt_path = root / "full_release_receipt.json"
    _write_json(receipt_path, receipt)
    return receipt_path


def _rewrite_v2_states(
    path: Path,
    ordinal: int,
    states: list[dict[str, object]],
    mutation: dict[str, object],
) -> None:
    connection = sqlite3.connect(path)
    record = json.loads(
        connection.execute(
            "SELECT record_json FROM results WHERE ordinal=?", (ordinal,)
        ).fetchone()[0]
    )
    record.update(mutation)
    record["state_records_count"] = len(states)
    record["state_records_sha256"] = AUDIT.canonical_sha256(states)
    connection.execute(
        "UPDATE results SET record_json=?, states_zlib=?, state_count=? WHERE ordinal=?",
        (AUDIT.canonical_text(record), _blob(states), len(states), ordinal),
    )
    connection.commit(); connection.close()


def _read_v2_states(path: Path, ordinal: int) -> list[dict[str, object]]:
    connection = sqlite3.connect(path)
    blob = connection.execute(
        "SELECT states_zlib FROM results WHERE ordinal=?", (ordinal,)
    ).fetchone()[0]
    connection.close()
    return [json.loads(line) for line in zlib.decompress(blob).splitlines()]


def test_happy_path_recomputes_mimic_strata(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is True
    assert report["schema_version"] == "pva_table4_v1_v2_prefix_parity_audit_v2"
    assert report["stratification"]["non_mimic_asset_count"] == 1
    assert report["stratification"]["mimic_asset_count"] == 1
    mimic = report["phase_migrations"]["mimic_assets"]
    assert mimic["strict"]["improved_ordinals"] == [1]
    assert mimic["rest"]["improved_ordinals"] == [1]
    assert mimic["single"]["improved_ordinals"] == [1]
    assert mimic["sobol"]["regressed_ordinals"] == []
    declared = report.pop("audit_content_sha256")
    assert declared == AUDIT.canonical_sha256(report)


def test_v1_v2_zero_state_retained_mimic_errors_are_reaggregated(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_retained_error=True)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is True
    assert report["parity"]["mimic_integrity_mismatch_count"] == 0
    assert report["phase_migrations"]["mimic_assets"]["strict"][
        "transitions"
    ] == {
        "fail_to_fail": 1,
        "fail_to_pass": 0,
        "pass_to_fail": 0,
        "pass_to_pass": 0,
    }


def test_zero_state_retained_mimic_error_rejects_numeric_maximum(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_retained_error=True)
    _update_v2_record(fixture["v2"], 1, {"max_penetration_m": 0.0})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="v2 mimic ordinal 1 recomputed field mismatch: max_penetration_m",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_zero_state_retained_mimic_error_rejects_missing_maximum(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_retained_error=True)
    _delete_v2_record_field(fixture["v2"], 1, "max_penetration_m")
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="record is missing recomputed fields: max_penetration_m",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_zero_state_retained_mimic_rejects_boolean_state_count(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_retained_error=True)
    _update_v2_record(fixture["v2"], 1, {"state_records_count": False})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="state_records_count must be an integer",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_v2_partial_state_retained_mimic_errors_are_reaggregated(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_partial_error=True)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is True
    assert report["parity"]["mimic_integrity_mismatch_count"] == 0
    assert report["phase_migrations"]["mimic_assets"]["strict"][
        "transitions"
    ] == {
        "fail_to_fail": 1,
        "fail_to_pass": 0,
        "pass_to_fail": 0,
        "pass_to_pass": 0,
    }


@pytest.mark.parametrize(
    "field",
    (
        "max_penetration_m",
        "max_penetration_normalized",
        "max_reset_readback_error",
    ),
)
def test_partial_state_retained_mimic_error_rejects_numeric_maximum(
    tmp_path: Path, field: str,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_partial_error=True)
    _update_v2_record(fixture["v2"], 1, {field: 0.0})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="partial collision metric is not a sealed exception prefix",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_partial_collision_metric_requires_retained_states(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path, mimic_retained_error=True)
    _update_v2_record(
        fixture["v2"], 1, {"collision_metric_status": "partial"}
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="partial collision metric is not a sealed exception prefix",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_partial_after_complete_single_sweep_remains_fail_closed(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_partial_after_single_error=True)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is True
    assert report["phase_migrations"]["mimic_assets"]["single"][
        "transitions"
    ]["fail_to_fail"] == 1


def test_partial_after_complete_single_sweep_rejects_true_pass(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, mimic_partial_after_single_error=True)
    _update_v2_record(fixture["v2"], 1, {"single_joint_sweep_cf": True})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained error is not fail-closed: single_joint_sweep_cf",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_retained_infrastructure_reexecution_is_separately_reported(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is True
    assert report["stratification"]["non_mimic_comparable_asset_count"] == 0
    assert report["parity"][
        "v1_retained_infrastructure_reexecution_count"
    ] == 1
    recovery = report["parity"][
        "v1_retained_infrastructure_reexecutions"
    ][0]
    assert recovery["v1_worker_status"] == "parent_error"
    assert recovery["v2_strict_collision_pass"] is True
    recovery_phases = report["phase_migrations"][
        "v1_retained_infrastructure_reexecutions"
    ]
    for phase in ("rest", "single", "sobol", "strict"):
        assert recovery_phases[phase]["outcomes"] == {
            "unobserved_to_fail": 0,
            "unobserved_to_pass": 1,
        }
        assert recovery_phases[phase]["v1_unobserved_count"] == 1
    assert report["phase_migrations"]["non_mimic_comparable_assets"][
        "strict"
    ]["transitions"] == {
        "fail_to_fail": 0,
        "fail_to_pass": 0,
        "pass_to_fail": 0,
        "pass_to_pass": 0,
    }
    assert report["parity"]["non_mimic_record_mismatch_count"] == 0
    assert report["parity"]["non_mimic_state_mismatch_count"] == 0


def test_v1_retained_infrastructure_record_is_not_allowlisted(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_mutation={"strict_collision_pass": False},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained infrastructure record mismatch: strict_collision_pass",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_retained_boolean_integer_alias_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_mutation={"load_success": 0},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained infrastructure record mismatch: load_success",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_parent_error_requires_nonempty_detail(tmp_path: Path) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_mutation={"issues": ["parent_executor_exception: "]},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained infrastructure issue/status mismatch",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_parent_error_rejects_whitespace_only_detail(tmp_path: Path) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_mutation={"issues": ["parent_executor_exception:   "]},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained infrastructure issue/status mismatch",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_timeout_must_match_manifest_duration(tmp_path: Path) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_worker_status="timeout",
        v1_retained_mutation={"issues": ["asset_timeout_after_599_seconds"]},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="retained infrastructure issue/status mismatch",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_parent_error_requires_explicit_empty_state_records(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_mutation={"state_records": None},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="parent_error state_records envelope mismatch",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_timeout_requires_absent_state_records(tmp_path: Path) -> None:
    fixture = _build_fixture(
        tmp_path,
        v1_retained_non_mimic=True,
        v1_retained_worker_status="timeout",
        v1_retained_mutation={"state_records": []},
    )
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="timeout state_records envelope mismatch",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("category", "forged"),
        ("dataset", "forged"),
        ("expected_movable_joints", 99),
        ("movable_dof_count", 99),
        ("primary_urdf_relative_path", "forged/model.urdf"),
    ],
)
def test_v2_reexecution_source_identity_is_roster_bound(
    tmp_path: Path, field: str, value: object
) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    _update_v2_record(fixture["v2"], 0, {field: value})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match=rf"execution/runtime binding mismatch: {field}",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize("field", ["category", "dataset"])
def test_v2_reexecution_state_identity_is_roster_bound(
    tmp_path: Path, field: str
) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    states = _read_v2_states(fixture["v2"], 0)
    states[0][field] = "forged"
    _rewrite_v2_states(fixture["v2"], 0, states, {})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match=rf"state 0 binding mismatch: {field}",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v2_reexecution_must_be_fully_completed(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    _update_v2_record(fixture["v2"], 0, {"issues": ["retained error"]})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="infrastructure reexecution is not complete: issues",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("rest_state_executed", True, "state execution coverage mismatch: rest"),
        ("unexecuted_state_count", False, "unexecuted_state_count must be an integer"),
        ("max_penetration_m", False, "recomputed field mismatch: max_penetration_m"),
    ],
)
def test_v2_reexecution_rejects_boolean_numeric_aliases(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    _update_v2_record(fixture["v2"], 0, {field: value})
    with pytest.raises(AUDIT.ParityAuditError, match=message):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_retained_reexecution_v2_outcome_is_recomputed(
    tmp_path: Path,
) -> None:
    fixture = _build_fixture(tmp_path, v1_retained_non_mimic=True)
    _update_v2_record(fixture["v2"], 0, {"strict_collision_pass": False})
    with pytest.raises(
        AUDIT.ParityAuditError,
        match="v2 infrastructure reexecution mimic ordinal 0 recomputed field mismatch: strict_collision_pass",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_forged_mimic_count_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {"mimic_joint_count": 0})
    with pytest.raises(AUDIT.ParityAuditError, match="sampling plan mismatch: mimic_joint_count"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mimic_worker_sampling_follower_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    states = _read_v2_states(fixture["v2"], 1)
    follower = dict(states[21])
    follower.update(
        {
            "joint_name": "follower",
            "sample_index": 0,
            "joint_values_sha256": "f" * 64,
        }
    )
    states.insert(22, follower)
    _rewrite_v2_states(
        fixture["v2"],
        1,
        states,
        {"single_state_executed": 22, "unexecuted_state_count": -1},
    )
    with pytest.raises(AUDIT.ParityAuditError, match="beyond the recomputed schedule|schedule mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_missing_completed_state_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    states = _read_v2_states(fixture["v2"], 1)
    states.pop()
    _rewrite_v2_states(
        fixture["v2"],
        1,
        states,
        {"sobol_state_executed": 63, "unexecuted_state_count": 1},
    )
    with pytest.raises(AUDIT.ParityAuditError, match="completed measurement does not close"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_expanded_joint_value_hash_tamper_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    states = _read_v2_states(fixture["v2"], 1)
    states[1]["joint_values_sha256"] = "0" * 64
    _rewrite_v2_states(fixture["v2"], 1, states, {})
    with pytest.raises(AUDIT.ParityAuditError, match="schedule mismatch: joint_values_sha256"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mimic_collision_state_hash_and_outcome_tamper_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    states = _read_v2_states(fixture["v2"], 1)
    states[-1]["non_adjacent_contact_count"] = 1
    states[-1]["non_adjacent_illegal_penetration_count"] = 1
    _rewrite_v2_states(
        fixture["v2"],
        1,
        states,
        {
            "sobol_non_adjacent_free": AUDIT.table4.SOBOL_SAMPLES - 1,
            "multi_joint_sobol_cf": False,
            "strict_collision_pass": False,
        },
    )
    with pytest.raises(AUDIT.ParityAuditError, match="count/max threshold mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize("field,value", [("independent_dof_count", 99), ("joint_sampling_plan_sha256", "0" * 64)])
def test_forged_plan_is_rejected(tmp_path: Path, field: str, value: object) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {field: value})
    with pytest.raises(AUDIT.ParityAuditError, match="recomputed sampling plan mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mirror_urdf_hash_drift_is_rejected(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    (fixture["mirror"] / "Category/seed_0001/model.urdf").write_text(NON_MIMIC_URDF, encoding="ascii")
    with pytest.raises(AUDIT.ParityAuditError, match="physical mirror URDF hash drift"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mimic_issues_are_not_allowlisted(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {"issues": ["forged"]})
    with pytest.raises(AUDIT.ParityAuditError, match="recomputed field mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize(
    "field,value",
    [("status", "error"), ("measurement_complete", False)],
)
def test_mimic_completion_contract_is_a_hard_gate(
    tmp_path: Path, field: str, value: object
) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {field: value})
    with pytest.raises(AUDIT.ParityAuditError, match="measurement|completed"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_worker_status_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    connection = sqlite3.connect(fixture["v2"])
    connection.execute("UPDATE results SET worker_status='worker_error' WHERE ordinal=1")
    connection.commit(); connection.close()
    with pytest.raises(AUDIT.ParityAuditError, match="unsuccessful worker status"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mimic_rest_execution_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {"rest_state_executed": 0})
    with pytest.raises(AUDIT.ParityAuditError, match="state execution coverage mismatch: rest"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_mimic_package_integrity_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 1, {"package_binding_verified": False})
    with pytest.raises(AUDIT.ParityAuditError, match="execution/runtime binding mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v1_database_sha_is_receipt_anchored(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    receipt = json.loads(fixture["receipt"].read_text(encoding="ascii"))
    receipt["result_database_sha256"] = "0" * 64
    receipt.pop("receipt_content_sha256")
    _write_json(fixture["receipt"], _self_hashed(receipt, "receipt_content_sha256"))
    with pytest.raises(AUDIT.ParityAuditError, match="sealed receipt"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize("suffix", ["-wal", "-journal"])
def test_nonempty_v1_durable_sidecar_is_rejected(
    tmp_path: Path, suffix: str
) -> None:
    fixture = _build_fixture(tmp_path)
    sidecar = Path(str(fixture["v1"]) + suffix)
    sidecar.write_bytes(b"unsealed database content")
    with pytest.raises(AUDIT.ParityAuditError, match="sidecar is non-empty"):
        AUDIT._verify_v1_sidecars(fixture["v1"])


def test_zero_wal_and_volatile_shm_policy_is_explicit(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    Path(str(fixture["v1"]) + "-wal").write_bytes(b"")
    Path(str(fixture["v1"]) + "-shm").write_bytes(b"volatile index")
    observed = AUDIT._verify_v1_sidecars(fixture["v1"])
    assert observed["observed"]["wal"]["bytes"] == 0
    assert observed["observed"]["shm"]["content_role"].startswith("volatile")


def test_v2_manifest_self_hash_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    manifest = json.loads(fixture["v2_manifest"].read_text(encoding="ascii"))
    manifest["protocol"]["sobol_seed"] += 1
    _write_json(fixture["v2_manifest"], manifest)
    with pytest.raises(AUDIT.ParityAuditError, match="self-hash mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v2_meta_source_binding_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    connection = sqlite3.connect(fixture["v2"])
    connection.execute("UPDATE meta SET value=? WHERE key='source_receipt_content_sha256'", (AUDIT.canonical_text("0" * 64),))
    connection.commit(); connection.close()
    with pytest.raises(AUDIT.ParityAuditError, match="manifest/database meta mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v2_manifest_source_denominators_bind_receipt(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    manifest = json.loads(fixture["v2_manifest"].read_text(encoding="ascii"))
    manifest.pop("manifest_content_sha256")
    manifest["source"]["J_eval"] = 99
    manifest = _self_hashed(manifest, "manifest_content_sha256")
    _write_json(fixture["v2_manifest"], manifest)
    connection = sqlite3.connect(fixture["v2"])
    connection.execute(
        "UPDATE meta SET value=? WHERE key='manifest_content_sha256'",
        (AUDIT.canonical_text(manifest["manifest_content_sha256"]),),
    )
    connection.commit(); connection.close()
    with pytest.raises(AUDIT.ParityAuditError, match="N_eval or J_eval"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v2_runtime_binding_is_a_hard_gate(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 0, {"runtime_identity": {"forged": True}})
    with pytest.raises(AUDIT.ParityAuditError, match="execution/runtime binding mismatch"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_non_mimic_regression_fails_parity(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 0, {"strict_collision_pass": False})
    _seal_v2_fixture(fixture)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is False
    assert report["parity"]["non_mimic_record_mismatch_count"] == 1
    assert report["inputs"]["v2_full_release_receipt_verified"] is True
    assert report["publication_status"]["final_publication_eligible"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("rest_non_adjacent_free", True),
        ("max_penetration_m", False),
    ],
)
def test_non_mimic_record_rejects_boolean_numeric_aliases(
    tmp_path: Path, field: str, value: object
) -> None:
    fixture = _build_fixture(tmp_path)
    _update_v2_record(fixture["v2"], 0, {field: value})
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is False
    assert report["parity"]["non_mimic_record_mismatches"] == [
        {
            "asset_id": "PV-A/Category/seed_0000",
            "fields": [field],
            "ordinal": 0,
        }
    ]


def test_non_mimic_state_rejects_boolean_numeric_alias(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    states = _read_v2_states(fixture["v2"], 0)
    states[0]["non_adjacent_contact_count"] = False
    _rewrite_v2_states(fixture["v2"], 0, states, {})
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["all_pass"] is False
    assert report["parity"]["non_mimic_state_mismatches"] == [
        {
            "asset_id": "PV-A/Category/seed_0000",
            "fields": ["non_adjacent_contact_count"],
            "ordinal": 0,
            "state_index": 0,
        }
    ]


def test_connection_is_read_only(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    connection, _resolved = AUDIT.connect_read_only(
        fixture["v1"], "fixture", immutable=True
    )
    assert connection.execute("PRAGMA query_only").fetchone() == (1,)
    assert connection.in_transaction is True
    with pytest.raises(sqlite3.OperationalError):
        connection.execute("DELETE FROM meta")
    connection.rollback(); connection.close()


def test_active_prefix_without_v2_receipt_is_not_publishable(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["inputs"]["v2_full_release_receipt_verified"] is False
    assert report["publication_status"] == {
        "interim_non_durable_active_snapshot": True,
        "final_publication_eligible": False,
        "replacement_policy": "replace with a sealed full-prefix audit after the formal run closes",
    }


def test_explicit_missing_v2_receipt_fails(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    with pytest.raises(AUDIT.ParityAuditError, match="v2 full-release receipt"):
        AUDIT.audit_prefix(
            fixture["v1"], fixture["v2"],
            v2_receipt=tmp_path / "missing-receipt.json",
        )


def test_auto_discovered_sealed_v2_receipt_enables_publication(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _seal_v2_fixture(fixture)
    Path(str(fixture["v2"]) + "-wal").write_bytes(b"")
    Path(str(fixture["v2"]) + "-shm").write_bytes(b"volatile index")
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["inputs"]["v2_full_release_receipt_verified"] is True
    assert report["inputs"]["v2_receipt_auto_discovered"] is True
    assert report["publication_status"]["final_publication_eligible"] is True
    assert report["publication_status"]["interim_non_durable_active_snapshot"] is False


def test_sealed_immutable_audit_does_not_create_shm(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _seal_v2_fixture(fixture)
    sidecars = [
        Path(str(fixture[label]) + "-shm") for label in ("v1", "v2")
    ]
    assert all(not path.exists() for path in sidecars)
    AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert all(not path.exists() for path in sidecars)


def test_sealed_immutable_audit_does_not_rewrite_shm(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _seal_v2_fixture(fixture)
    wal = Path(str(fixture["v2"]) + "-wal")
    shm = Path(str(fixture["v2"]) + "-shm")
    wal.write_bytes(b"")
    shm.write_bytes(b"stale volatile index")
    before = shm.stat()
    AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    after = shm.stat()
    assert shm.read_bytes() == b"stale volatile index"
    assert (after.st_ino, after.st_size, after.st_mtime_ns) == (
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )


def test_v2_receipt_database_sha_tamper_fails(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    receipt_path = _seal_v2_fixture(fixture)
    receipt = json.loads(receipt_path.read_text(encoding="ascii"))
    receipt.pop("receipt_content_sha256")
    receipt["result_database_sha256"] = "0" * 64
    _write_json(receipt_path, _self_hashed(receipt, "receipt_content_sha256"))
    with pytest.raises(AUDIT.ParityAuditError, match="database SHA256"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_v2_receipt_artifact_manifest_tamper_fails(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    _seal_v2_fixture(fixture)
    artifact = fixture["v2"].parent / "artifact_manifest.json"
    artifact.write_text(artifact.read_text(encoding="ascii") + "\n", encoding="ascii")
    with pytest.raises(AUDIT.ParityAuditError, match="artifact-manifest SHA256"):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


@pytest.mark.parametrize("suffix", ["-wal", "-journal"])
def test_nonempty_v2_durable_sidecar_rejects_sealed_receipt(
    tmp_path: Path, suffix: str
) -> None:
    fixture = _build_fixture(tmp_path)
    _seal_v2_fixture(fixture)
    Path(str(fixture["v2"]) + suffix).write_bytes(b"uncheckpointed")
    kind = suffix.removeprefix("-")
    with pytest.raises(
        AUDIT.ParityAuditError,
        match=rf"sealed v2 SQLite {kind} sidecar is non-empty",
    ):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_sealed_smoke_classification_is_not_publishable(tmp_path: Path) -> None:
    fixture = _build_fixture(tmp_path)
    manifest = json.loads(fixture["v2_manifest"].read_text(encoding="ascii"))
    manifest.pop("manifest_content_sha256")
    manifest["classification"] = "SMOKE"
    manifest = _self_hashed(manifest, "manifest_content_sha256")
    _write_json(fixture["v2_manifest"], manifest)
    connection = sqlite3.connect(fixture["v2"])
    connection.execute(
        "UPDATE meta SET value=? WHERE key='manifest_content_sha256'",
        (AUDIT.canonical_text(manifest["manifest_content_sha256"]),),
    )
    connection.commit(); connection.close()
    _seal_v2_fixture(fixture)
    report = AUDIT.audit_prefix(fixture["v1"], fixture["v2"])
    assert report["inputs"]["v2_full_release_receipt_verified"] is True
    assert report["publication_status"]["final_publication_eligible"] is False


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("category_count", 99, "protocol/denominator/source binding"),
        ("status", "RUNNING", "invalid full-release status"),
    ],
)
def test_v2_receipt_release_metadata_tamper_fails(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    fixture = _build_fixture(tmp_path)
    receipt_path = _seal_v2_fixture(fixture)
    receipt = json.loads(receipt_path.read_text(encoding="ascii"))
    receipt.pop("receipt_content_sha256")
    receipt[field] = value
    _write_json(receipt_path, _self_hashed(receipt, "receipt_content_sha256"))
    with pytest.raises(AUDIT.ParityAuditError, match=message):
        AUDIT.audit_prefix(fixture["v1"], fixture["v2"])


def test_cli_writes_self_hashed_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    fixture = _build_fixture(tmp_path)
    output = tmp_path / "audit.json"
    assert AUDIT.main([str(fixture["v1"]), str(fixture["v2"]), "--limit", "1", "--json-out", str(output)]) == 0
    written = json.loads(output.read_text(encoding="ascii"))
    declared = written.pop("audit_content_sha256")
    assert declared == AUDIT.canonical_sha256(written)
    assert json.loads(capsys.readouterr().out)["all_pass"] is True


def test_cli_accepts_explicit_v2_receipt(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    fixture = _build_fixture(tmp_path)
    receipt = _seal_v2_fixture(fixture)
    output = tmp_path / "sealed-audit.json"
    assert AUDIT.main([
        str(fixture["v1"]), str(fixture["v2"]),
        "--v2-receipt", str(receipt), "--json-out", str(output),
    ]) == 0
    report = json.loads(output.read_text(encoding="ascii"))
    assert report["inputs"]["v2_full_release_receipt_verified"] is True
    assert report["inputs"]["v2_receipt_auto_discovered"] is False
    assert report["publication_status"]["final_publication_eligible"] is True
    capsys.readouterr()
