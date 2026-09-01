#!/usr/bin/env python3
"""Aggregate the final Table 5 numerical-validity protocol for eight cohorts."""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_1_runtime as _runtime  # noqa: E402
import table5_stable_v2_aggregate as _base  # noqa: E402
import table5_functional_motion as _functional  # noqa: E402


SCHEMA_VERSION = "table5_finite_rollout_v2_1_aggregate_v2"


def _rate(passed: int) -> dict[str, Any]:
    return {
        "passed": passed,
        "denominator": _base.EXPECTED_N,
        "percentage": 100.0 * passed / _base.EXPECTED_N,
    }


def _unit_rate(passed: int, denominator: int) -> dict[str, Any]:
    return {
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
    }


def _receipt_hash_matches(receipt: Mapping[str, Any]) -> bool:
    observed = receipt.get("receipt_sha256")
    return bool(
        isinstance(observed, str)
        and observed
        == _runtime._core._runtime.canonical_sha256(
            receipt, exclude_fields=("receipt_sha256",)
        )
    )


def _readiness_evidence(
    record: Mapping[str, Any],
    *,
    simulator: str,
    dataset_slug: str,
    row: Mapping[str, Any],
    expected_protocol: Mapping[str, Any],
) -> tuple[bool, set[str]]:
    if record.get("terminal_status") != "completed":
        return False, set()
    identity = record.get("identity")
    if not isinstance(identity, Mapping) or any(
        identity.get(field) != expected
        for field, expected in (
            ("simulator", simulator),
            ("dataset_slug", dataset_slug),
            ("dataset_id", row.get("dataset_id")),
            ("row_sha256", row.get("row_sha256")),
        )
    ):
        return False, set()
    evaluation = record.get("evaluation")
    evidence = evaluation.get("stable_v2") if isinstance(evaluation, Mapping) else None
    if (
        not isinstance(evidence, Mapping)
        or evidence.get("schema_version") != _runtime.EVIDENCE_SCHEMA
        or evidence.get("protocol") != expected_protocol
    ):
        return False, set()

    native_import = evidence.get("native_import")
    imported = bool(
        isinstance(native_import, Mapping)
        and native_import.get("schema_version")
        == _runtime._r2.IMPORT_RECEIPT_SCHEMA
        and native_import.get("simulator") == simulator
        and native_import.get("passed") is True
        and _receipt_hash_matches(native_import)
    )
    if not imported:
        return False, set()

    declared_names = [
        str(joint["name"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
    ]
    declared_set = set(declared_names)
    mapping = evidence.get("dof_mapping")
    if not isinstance(mapping, Mapping) or not _receipt_hash_matches(mapping):
        return True, set()
    mapped_names = mapping.get("mapped_canonical_scalar_joint_names")
    unmapped_names = mapping.get("unmapped_canonical_scalar_joint_names")
    if (
        mapping.get("declared_canonical_scalar_joint_names") != declared_names
        or not isinstance(mapped_names, list)
        or not all(isinstance(name, str) for name in mapped_names)
        or len(mapped_names) != len(set(mapped_names))
        or not set(mapped_names) <= declared_set
        or unmapped_names != sorted(declared_set - set(mapped_names))
        or mapping.get("declared_count") != len(declared_names)
        or mapping.get("mapped_count") != len(mapped_names)
    ):
        return True, set()
    return True, set(mapped_names)


def _intersect_readiness(
    rows: Sequence[Mapping[str, Any]],
    evidence_by_simulator: Mapping[str, Sequence[tuple[bool, set[str]]]],
) -> dict[str, dict[str, Any]]:
    if any(
        len(evidence_by_simulator.get(simulator, ())) != len(rows)
        for simulator in _base.SIMULATORS
    ):
        raise _base.AggregateError("cross-simulator readiness evidence is incomplete")
    all_three_imports = 0
    all_three_mapped = 0
    declared_dofs = 0
    for index, row in enumerate(rows):
        declared = {
            str(joint["name"])
            for joint in row.get("scalar_joints", [])
            if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
        }
        declared_dofs += len(declared)
        simulator_evidence = [
            evidence_by_simulator[simulator][index]
            for simulator in _base.SIMULATORS
        ]
        if all(imported for imported, _ in simulator_evidence):
            all_three_imports += 1
        mapped_intersection = set(declared)
        for imported, mapped in simulator_evidence:
            mapped_intersection &= mapped if imported else set()
        all_three_mapped += len(mapped_intersection)
    return {
        "import_success": _unit_rate(all_three_imports, len(rows)),
        "dof_mapping": _unit_rate(all_three_mapped, declared_dofs),
    }


def _all_three_readiness(
    dataset: Mapping[str, Any], run_root: Path
) -> dict[str, dict[str, Any]]:
    slug = str(dataset["dataset_slug"])
    rows = dataset["rows"]
    expected_protocol = _runtime._stable_protocol()
    evidence_by_simulator: dict[str, list[tuple[bool, set[str]]]] = {
        simulator: [] for simulator in _base.SIMULATORS
    }
    for row in rows:
        dataset_id = str(row["dataset_id"])
        for simulator in _base.SIMULATORS:
            path = _base._record_path(run_root, simulator, slug, dataset_id)
            if not path.is_file():
                evidence = (False, set())
            else:
                evidence = _readiness_evidence(
                    _base._read_json(path),
                    simulator=simulator,
                    dataset_slug=slug,
                    row=row,
                    expected_protocol=expected_protocol,
                )
            evidence_by_simulator[simulator].append(evidence)
    return _intersect_readiness(rows, evidence_by_simulator)


def _required_inertial_details(row: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    physics = row.get("physics")
    if not isinstance(physics, Mapping):
        return []
    links = physics.get("links")
    if isinstance(links, list):
        details = [
            link.get("details")
            for link in links
            if isinstance(link, Mapping)
            and link.get("required_for_fixed_base_dynamics") is True
        ]
        return details if all(isinstance(value, Mapping) for value in details) else []

    plan_path_value = physics.get("physics_plan_path")
    plan_sha256 = physics.get("physics_plan_sha256")
    joint_tree = row.get("joint_tree")
    if (
        not isinstance(plan_path_value, str)
        or not isinstance(plan_sha256, str)
        or not isinstance(joint_tree, Mapping)
    ):
        return []
    plan_path = Path(plan_path_value)
    try:
        observed_sha256 = _runtime._core._runtime.sha256_file(plan_path)
    except OSError as error:
        raise _base.AggregateError(
            f"cannot hash physics plan {plan_path}: {error}"
        ) from error
    if observed_sha256 != plan_sha256:
        raise _base.AggregateError(f"physics plan hash mismatch: {plan_path}")
    plan = _base._read_json(plan_path)
    plan_links = plan.get("links")
    if not isinstance(plan_links, list):
        raise _base.AggregateError(f"physics plan links are missing: {plan_path}")
    by_name: dict[str, Mapping[str, Any]] = {}
    for link in plan_links:
        name = link.get("link_name") if isinstance(link, Mapping) else None
        inertial = link.get("inertial") if isinstance(link, Mapping) else None
        if (
            not isinstance(name, str)
            or not isinstance(inertial, Mapping)
            or name in by_name
        ):
            raise _base.AggregateError(f"physics plan link is malformed: {plan_path}")
        by_name[name] = inertial
    all_links = joint_tree.get("links")
    root_links = joint_tree.get("root_links")
    if not isinstance(all_links, list) or not isinstance(root_links, list):
        return []
    required_names = [str(name) for name in all_links if name not in set(root_links)]
    if any(name not in by_name for name in required_names):
        raise _base.AggregateError(
            f"physics plan lacks required dynamic link: {plan_path}"
        )
    return [by_name[name] for name in required_names]


def _is_exact_unit_inertial(details: Mapping[str, Any]) -> bool:
    center = details.get("center_of_mass_xyz")
    return bool(
        details.get("mass_kg") == 1.0
        and details.get("inertia_eigenvalues_kg_m2") == [1.0, 1.0, 1.0]
        and (center is None or center == [0.0, 0.0, 0.0])
    )


def _is_exact_unit_inertial_placeholder(row: Mapping[str, Any]) -> bool:
    physics = row.get("physics")
    if not isinstance(physics, Mapping) or physics.get("status") != "ready":
        return False
    required = _required_inertial_details(row)
    return bool(required) and all(
        _is_exact_unit_inertial(details) for details in required
    )


def _inertial_rates(rows: Sequence[Mapping[str, Any]]) -> dict[str, dict[str, Any]]:
    complete = sum(
        isinstance(row.get("physics"), Mapping)
        and row["physics"].get("status") == "ready"
        for row in rows
    )
    exact_unit_placeholders = sum(
        _is_exact_unit_inertial_placeholder(row) for row in rows
    )
    return {
        "mathematically_valid": _rate(complete),
        "complete_non_placeholder": _rate(complete - exact_unit_placeholders),
        "exact_unit_placeholder": _rate(exact_unit_placeholders),
    }


def _complete_non_placeholder_inertial(row: Mapping[str, Any]) -> bool:
    physics = row.get("physics")
    return bool(
        isinstance(physics, Mapping)
        and physics.get("status") == "ready"
        and not _is_exact_unit_inertial_placeholder(row)
    )


def _complete_genesis_readiness(
    dataset: Mapping[str, Any], run_root: Path
) -> dict[str, Any]:
    slug = str(dataset["dataset_slug"])
    expected_protocol = _runtime._stable_protocol()
    passed = 0
    reason_counts: dict[str, int] = {}
    for row in dataset["rows"]:
        dataset_id = str(row["dataset_id"])
        path = _base._record_path(run_root, "genesis", slug, dataset_id)
        checks = {
            "native_import": False,
            "complete_dof_mapping": False,
            "complete_non_placeholder_inertials": _complete_non_placeholder_inertial(
                row
            ),
            "finite_rollout_v2_1": False,
        }
        if path.is_file():
            record = _base._read_json(path)
            imported, mapped = _readiness_evidence(
                record,
                simulator="genesis",
                dataset_slug=slug,
                row=row,
                expected_protocol=expected_protocol,
            )
            declared = {
                str(joint["name"])
                for joint in row.get("scalar_joints", [])
                if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
            }
            receipt = _validated_v21_receipt(record, "genesis", slug)
            checks["native_import"] = imported
            checks["complete_dof_mapping"] = bool(imported and mapped == declared)
            checks["finite_rollout_v2_1"] = bool(
                isinstance(receipt, Mapping) and receipt.get("passed") is True
            )
        failed = [name for name, value in checks.items() if not value]
        if not failed:
            passed += 1
        else:
            reason = "+".join(failed)
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    return {
        **_rate(passed),
        "criterion": (
            "exact per-asset intersection of Genesis native import, complete declared "
            "DoF mapping, complete non-placeholder inertials, and finite rollout v2.1"
        ),
        "failure_reason_counts": reason_counts,
    }


def _prepared_datasets(
    formal_prepared: Path, articraft_prepared: Path
) -> list[dict[str, Any]]:
    return _base._fixed_datasets(
        _base._read_json(formal_prepared), _base._read_json(articraft_prepared)
    )


def _strict_by_slug(path: Path | None) -> dict[str, Mapping[str, Any]]:
    if path is None:
        return {}
    summary = _base._read_json(path)
    datasets = summary.get("datasets")
    if not isinstance(datasets, list):
        raise _base.AggregateError("strict sensitivity summary lacks datasets")
    return {
        str(dataset["dataset_slug"]): dataset
        for dataset in datasets
        if isinstance(dataset, Mapping)
    }


def _functional_by_slug(path: Path | None) -> dict[str, Mapping[str, Any]]:
    if path is None:
        return {}
    summary = _base._read_json(path)
    if (
        summary.get("schema_version") != _functional.SUMMARY_SCHEMA
        or summary.get("classification") != "COMPLETE"
        or summary.get("protocol") != _functional._protocol()
        or summary.get("summary_sha256")
        != _runtime._core._runtime.canonical_sha256(
            summary, exclude_fields=("summary_sha256",)
        )
    ):
        raise _base.AggregateError("functional-motion summary is invalid or incomplete")
    datasets = summary.get("datasets")
    if not isinstance(datasets, list):
        raise _base.AggregateError("functional-motion summary lacks datasets")
    result = {
        str(dataset["dataset_slug"]): dataset
        for dataset in datasets
        if isinstance(dataset, Mapping)
    }
    if set(result) != set(_base.DATASET_ORDER):
        raise _base.AggregateError("functional-motion dataset set differs from Table 5")
    for slug, dataset in result.items():
        if dataset.get("n") != _base.EXPECTED_N or dataset.get("record_count") != _base.EXPECTED_N:
            raise _base.AggregateError(
                f"functional-motion dataset is incomplete: {slug}"
            )
    return result


def _validated_v21_receipt(
    record: Mapping[str, Any], simulator: str, dataset_slug: str
) -> Mapping[str, Any] | None:
    previous_runtime = _base._runtime
    _base._runtime = _runtime
    try:
        receipt, _ = _base._validated_rollout(record, simulator, dataset_slug)
    finally:
        _base._runtime = previous_runtime
    return receipt


def _trial_max(receipt: Mapping[str, Any], field: str) -> float | None:
    if receipt.get("passed") is not True:
        return None
    trials = receipt.get("trials")
    if not isinstance(trials, list) or len(trials) != _runtime.RESET_REPETITIONS:
        return None
    values = [
        trial.get(field) if isinstance(trial, Mapping) else None for trial in trials
    ]
    if not all(_base._finite(value) for value in values):
        return None
    return max(float(value) for value in values)


def _bounded_joint_types(row: Mapping[str, Any]) -> set[str]:
    return {
        str(joint["type"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping)
        and joint.get("type") in {"revolute", "prismatic"}
        and _base._finite(joint.get("lower"))
        and _base._finite(joint.get("upper"))
        and float(joint["lower"]) < float(joint["upper"])
    }


def _neutral_diagnostics(
    prepared: Sequence[Mapping[str, Any]], run_root: Path
) -> dict[str, dict[str, dict[str, Any]]]:
    output: dict[str, dict[str, dict[str, Any]]] = {}
    for dataset in prepared:
        slug = str(dataset["dataset_slug"])
        rows = dataset["rows"]
        types = [_bounded_joint_types(row) for row in rows]
        output[slug] = {}
        for simulator in _base.SIMULATORS:
            limit_values: list[list[float]] = []
            for row, joint_types in zip(rows, types, strict=True):
                dataset_id = str(row["dataset_id"])
                path = _base._record_path(run_root, simulator, slug, dataset_id)
                receipt = None
                if path.is_file():
                    receipt = _validated_v21_receipt(
                        _base._read_json(path), simulator, slug
                    )
                limit = (
                    _trial_max(receipt, "limit_violation_over_range_max")
                    if isinstance(receipt, Mapping) and joint_types
                    else None
                )
                limit_values.append([] if limit is None else [limit])
            output[slug][simulator] = {
                "limit_violation_p95": _base._continuous(
                    limit_values,
                    candidate_units=sum(bool(value) for value in types),
                    scale=100.0,
                    unit="percent_joint_range",
                ),
            }
    return output


def aggregate(
    formal_prepared: Path,
    articraft_prepared: Path,
    run_root: Path,
    old_summary_path: Path,
    strict_summary_path: Path | None = None,
    functional_summary_path: Path | None = None,
) -> dict[str, Any]:
    prepared = _prepared_datasets(formal_prepared, articraft_prepared)
    previous_runtime = _base._runtime
    _base._runtime = _runtime
    try:
        summary = _base.aggregate(
            formal_prepared, articraft_prepared, run_root, old_summary_path
        )
    finally:
        _base._runtime = previous_runtime
    prepared_by_slug = {str(dataset["dataset_slug"]): dataset for dataset in prepared}
    strict = _strict_by_slug(strict_summary_path)
    functional = _functional_by_slug(functional_summary_path)
    neutral = _neutral_diagnostics(prepared, run_root)
    for dataset in summary["datasets"]:
        slug = str(dataset["dataset_slug"])
        rows = prepared_by_slug[slug]["rows"]
        dataset["inertial_assets"] = _inertial_rates(rows)
        dataset["complete_genesis_readiness"] = _complete_genesis_readiness(
            prepared_by_slug[slug], run_root
        )
        if functional_summary_path is not None:
            dataset["dependency_aware_functional_motion"] = functional.get(slug)
        dataset["all_three_readiness"] = _all_three_readiness(
            prepared_by_slug[slug], run_root
        )
        dataset["neutral_long_horizon_diagnostics"] = neutral[slug]
        dataset["finite_rollout_v2_1"] = dataset.pop("stable_v2")
        dataset["all_three_finite_rollout_v2_1"] = dataset.pop("all_three_stable_v2")
        strict_dataset = strict.get(slug)
        dataset["strict_stable_v2_sensitivity"] = (
            {
                "by_simulator": strict_dataset.get("stable_v2"),
                "all_three": strict_dataset.get("all_three_stable_v2"),
            }
            if isinstance(strict_dataset, Mapping)
            else None
        )

    summary["schema_version"] = SCHEMA_VERSION
    summary["finite_rollout_v2_1_protocol"] = summary.pop("stable_v2_protocol")
    summary["strict_sensitivity_summary"] = (
        str(strict_summary_path.resolve()) if strict_summary_path is not None else None
    )
    if functional_summary_path is not None:
        summary["functional_motion_summary"] = str(functional_summary_path.resolve())
    summary["metric_roles"] = {
        "primary": [
            "import_success",
            "dof_coverage",
            "complete_non_placeholder_inertial_assets",
            "genesis_finite_rollout_v2_1",
            "trajectory_coverage",
            "complete_genesis_readiness",
        ],
        "cross_simulator": [
            "all_three_import_success",
            "all_three_dof_mapping",
            "all_three_finite_rollout_v2_1",
        ],
        "supplementary_sensitivity": [
            "mathematically_valid_inertial_assets",
            "exact_unit_inertial_placeholders",
            "constraint_drift_position_p95",
            "constraint_drift_rotation_p95",
            "neutral_limit_violation_p95",
            "strict_stable_v2",
            "tracking_nrmse_p95",
            "limit_violation_p95",
            "fk_position_error_p95",
            "fk_rotation_error_p95",
        ],
    }
    if functional_summary_path is not None:
        summary["metric_roles"]["supplementary_sensitivity"].append(
            "dependency_aware_functional_motion_diagnostic"
        )
    summary["summary_sha256"] = _runtime._core._runtime.canonical_sha256(
        summary, exclude_fields=("summary_sha256",)
    )
    return summary


def _best_high(summary: Mapping[str, Any], getter: Any) -> float:
    return max(float(getter(dataset)) for dataset in summary["datasets"])


def _high_cell(value: float, best: float) -> str:
    text = f"{value:.2f}"
    return f"**{text}**" if math.isclose(value, best, abs_tol=1.0e-12) else text


def _drift_cell(value: Mapping[str, Any]) -> str:
    evaluated = int(value.get("evaluated_units", 0))
    candidate = int(value.get("candidate_units", 0))
    p95 = value.get("p95")
    if p95 is None:
        return f"N/E ({evaluated}/{candidate})"
    numeric = float(p95)
    formatted = f"{numeric:.2e}" if 0.0 < abs(numeric) < 1.0e-4 else f"{numeric:.4f}"
    return f"{formatted} ({evaluated}/{candidate})"


def _distribution_cell(value: Mapping[str, Any]) -> str:
    evaluated = int(value.get("evaluated_units", 0))
    candidate = int(value.get("candidate_units", 0))
    median = value.get("median")
    p95 = value.get("p95")
    if median is None or p95 is None:
        return f"N/E ({evaluated}/{candidate})"
    return f"{float(median):.4f} / {float(p95):.4f} ({evaluated}/{candidate})"


def _report(summary: Mapping[str, Any]) -> str:
    import_best = _best_high(
        summary, lambda row: row["primary_existing"]["import_success"]["percentage"]
    )
    dof_best = _best_high(
        summary, lambda row: row["primary_existing"]["dof_mapping"]["percentage"]
    )
    inertial_best = _best_high(
        summary,
        lambda row: row["inertial_assets"]["complete_non_placeholder"]["percentage"],
    )
    genesis_finite_best = _best_high(
        summary,
        lambda row: row["finite_rollout_v2_1"]["genesis"]["percentage"],
    )
    trajectory_best = _best_high(
        summary,
        lambda row: row["primary_existing"]["actuated_trajectory_coverage"][
            "percentage"
        ],
    )
    complete_readiness_best = _best_high(
        summary, lambda row: row["complete_genesis_readiness"]["percentage"]
    )
    all_three_import_best = _best_high(
        summary, lambda row: row["all_three_readiness"]["import_success"]["percentage"]
    )
    all_three_dof_best = _best_high(
        summary, lambda row: row["all_three_readiness"]["dof_mapping"]["percentage"]
    )
    all_three_stable_best = _best_high(
        summary,
        lambda row: row["all_three_finite_rollout_v2_1"]["percentage"],
    )
    lines = [
        "# Table 5: articulated-asset simulation readiness",
        "",
        f"Run classification: **{summary['classification']}**.",
        "",
        "The 10 s simulation-validity test is not physical settling. Each asset is reset three times to its URDF neutral state (zero clamped to bounded ranges), then simulated for 10 s at 240 Hz with gravity, contacts, self-collision, a fixed base, manifest-bound physics, and zero applied joint force. Passing requires accurate finite resets, finite mapped states and observed poses for all 2,400 steps, and unchanged mapping in all repetitions. Table 5a reports the Genesis single-simulator evaluation. Table 5b reports exact Genesis/PyBullet/MuJoCo intersections for import, canonical DoF mapping, and 10 s numerical stability; these values are not averages of per-simulator rates. Joint speed and 0.5% limit compliance are retained only as strict sensitivity diagnostics because they depend on authored damping, armature, collision filtering, and functional multi-joint dependencies.",
        "",
        "Complete Non-placeholder Inertials requires every non-root dynamic link to have a positive finite mass, finite center of mass, and positive-definite inertia satisfying the rigid-body triangle inequality. An asset is excluded when every required dynamic link uses the exact unit placeholder `mass=1, inertia=I` (and zero COM when published); simulator-generated defaults do not count. PV-A overlay plans are hash-checked and audited with the same rule. The unfiltered mathematical-validity rate and placeholder incidence remain in the supplementary report.",
        "",
        "Complete Genesis Readiness is the exact per-asset intersection of native import, complete declared DoF mapping, complete non-placeholder inertials, and the three-repeat 10 s Genesis validity gate.",
        "",
        "## Table 5a: Genesis single-simulator evaluation",
        "",
        "| Dataset | N | Import (%) ↑ | DoF Mapping (%) ↑ | Complete Non-placeholder Inertials (%) ↑ | Genesis 10 s Simulation Validity (%) ↑ | Trajectory Coverage (%) ↑ | Complete Genesis Readiness (%) ↑ |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        existing = dataset["primary_existing"]
        import_value = float(existing["import_success"]["percentage"])
        dof_value = float(existing["dof_mapping"]["percentage"])
        inertial_value = float(
            dataset["inertial_assets"]["complete_non_placeholder"]["percentage"]
        )
        finite_value = float(
            dataset["finite_rollout_v2_1"]["genesis"]["percentage"]
        )
        trajectory_value = float(existing["actuated_trajectory_coverage"]["percentage"])
        complete_readiness_value = float(
            dataset["complete_genesis_readiness"]["percentage"]
        )
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_high_cell(import_value, import_best)} | "
            f"{_high_cell(dof_value, dof_best)} | "
            f"{_high_cell(inertial_value, inertial_best)} | "
            f"{_high_cell(finite_value, genesis_finite_best)} | "
            f"{_high_cell(trajectory_value, trajectory_best)} | "
            f"{_high_cell(complete_readiness_value, complete_readiness_best)} |"
        )
    lines.extend(
        [
            "",
            "## Table 5b: Cross-simulator evaluation",
            "",
            "`All-3` is the exact per-asset or per-joint intersection across Genesis, PyBullet, and MuJoCo. Stable denotes the same 10 s numerical-stability gate used in Table 5a, not physical settling.",
            "",
            "| Dataset | All-3 Import (%) ↑ | All-3 DoF Mapping (%) ↑ | All-3 Stable (%) ↑ |",
            "|---|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        readiness = dataset["all_three_readiness"]
        import_value = float(readiness["import_success"]["percentage"])
        dof_value = float(readiness["dof_mapping"]["percentage"])
        stable_value = float(dataset["all_three_finite_rollout_v2_1"]["percentage"])
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_high_cell(import_value, all_three_import_best)} | "
            f"{_high_cell(dof_value, all_three_dof_best)} | "
            f"{_high_cell(stable_value, all_three_stable_best)} |"
        )
    lines.extend(
        [
            "",
            "## Method basis",
            "",
            "- NVIDIA PhysX treats 200-300 deg/s as a tuning recommendation and separately emphasizes timestep, mass/inertia ratios, armature, damping, and non-adjacent self-collision: https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/articulation_stability.html",
            "- MuJoCo models joint limits, damping, friction loss, and armature as separate authored properties: https://mujoco.readthedocs.io/en/3.1.3/XMLreference.html",
            "- Articraft's simulation guide reports standing stability, penetration, separation, residual velocity, and released-joint peak speed as distinct diagnostics: https://github.com/articraftresearch/Articraft/blob/main/docs/simulation.md",
            "",
        ]
    )
    return "\n".join(lines)


def _supplementary_report(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 supplementary diagnostics",
        "",
        "These diagnostics do not participate in the primary ranking.",
        "",
        "## Inertial audit detail",
        "",
        "The mathematical-validity column accepts any complete positive-definite rigid-body inertia. The exact-unit-placeholder column counts assets for which every required dynamic link is exactly `mass=1, inertia=I` (and zero COM when published). Hash-bound PV-A overlay plans are parsed with the same rule.",
        "",
        "| Dataset | Mathematically Valid (%) | Exact Unit Placeholder (%) | Complete Non-placeholder (%) |",
        "|---|---:|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        inertial = dataset["inertial_assets"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{inertial['mathematically_valid']['percentage']:.2f} | "
            f"{inertial['exact_unit_placeholder']['percentage']:.2f} | "
            f"{inertial['complete_non_placeholder']['percentage']:.2f} |"
        )

    lines.extend(
        [
            "",
            "## Kinematic drift numerical diagnostics",
            "",
            "These FK residuals are retained for numerical auditing only. All values are near solver precision and do not participate in Table 5 ranking. Each cell includes evaluated/candidate joint coverage.",
            "",
            "| Dataset | Genesis Pos (% diag.) | Genesis Rot (deg) | PyBullet Pos (% diag.) | PyBullet Rot (deg) | MuJoCo Pos (% diag.) | MuJoCo Rot (deg) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        drift = dataset["constraint_drift"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_drift_cell(drift['genesis']['position_p95'])} | "
            f"{_drift_cell(drift['genesis']['rotation_p95'])} | "
            f"{_drift_cell(drift['pybullet']['position_p95'])} | "
            f"{_drift_cell(drift['pybullet']['rotation_p95'])} | "
            f"{_drift_cell(drift['mujoco']['position_p95'])} | "
            f"{_drift_cell(drift['mujoco']['rotation_p95'])} |"
        )

    lines.extend(
        [
            "",
            "## Neutral long-horizon physical diagnostics",
            "",
            "This is the continuous limit-violation distribution from the same three neutral 10 s trials. No arbitrary pass/fail threshold is applied. Each cell is `P50 / P95 (evaluated/candidate bounded-joint assets)`; missing or failed rollouts reduce coverage rather than improving the statistic.",
            "",
            "| Dataset | Genesis Limit Violation (%) | PyBullet Limit Violation (%) | MuJoCo Limit Violation (%) |",
            "|---|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        diagnostics = dataset["neutral_long_horizon_diagnostics"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_distribution_cell(diagnostics['genesis']['limit_violation_p95'])} | "
            f"{_distribution_cell(diagnostics['pybullet']['limit_violation_p95'])} | "
            f"{_distribution_cell(diagnostics['mujoco']['limit_violation_p95'])} |"
        )

    lines.extend(
        [
            "",
            "## Strict multi-pose sensitivity",
            "",
            "This protocol synchronously places every bounded joint at 25%, 50%, and 75% of its range and requires <=0.5% limit violation plus <=300 deg/s revolute and <=5 m/s prismatic peak speed for all three 10 s trials. It is retained as a deliberately strict stress test, not as the primary stability construct.",
            "",
        ]
    )
    if any(
        dataset.get("strict_stable_v2_sensitivity") is not None
        for dataset in summary["datasets"]
    ):
        lines.extend(
            [
                "| Dataset | Genesis (%) | PyBullet (%) | MuJoCo (%) | All three (%) |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        for dataset in summary["datasets"]:
            strict = dataset.get("strict_stable_v2_sensitivity")
            if not isinstance(strict, Mapping):
                lines.append(f"| {dataset['dataset_name']} | N/E | N/E | N/E | N/E |")
                continue
            by_simulator = strict["by_simulator"]
            all_three = strict["all_three"]
            lines.append(
                f"| {dataset['dataset_name']} | "
                f"{by_simulator['genesis']['percentage']:.2f} | "
                f"{by_simulator['pybullet']['percentage']:.2f} | "
                f"{by_simulator['mujoco']['percentage']:.2f} | "
                f"{all_three['percentage']:.2f} |"
            )
    else:
        lines.append("Strict sensitivity aggregation is pending.")

    if any(
        isinstance(dataset.get("dependency_aware_functional_motion"), Mapping)
        for dataset in summary["datasets"]
    ):
        lines.extend(
            [
                "",
                "## Dependency-aware motion diagnostic",
                "",
                "This diagnostic is excluded from Table 5 ranking. It is a strict geometry-only proxy without semantic task annotations, treats bounded joints sequentially, and does not support continuous or coupled/mimic motion. Asset success therefore rewards simpler articulations and must not be interpreted as function correctness.",
                "",
                "| Dataset | Asset Success (%) | Joint Completion (%) | Evaluable Asset Coverage (%) |",
                "|---|---:|---:|---:|",
            ]
        )
        for dataset in summary["datasets"]:
            functional = dataset.get("dependency_aware_functional_motion")
            if not isinstance(functional, Mapping):
                lines.append(f"| {dataset['dataset_name']} | N/E | N/E | N/E |")
                continue
            lines.append(
                f"| {dataset['dataset_name']} | "
                f"{functional['asset_success']['percentage']:.2f} | "
                f"{functional['joint_completion']['percentage']:.2f} | "
                f"{functional['evaluable_asset_coverage']['percentage']:.2f} |"
            )

    lines.extend(
        [
            "",
            "## Existing active-control diagnostics",
            "",
            "| Dataset | Tracking NRMSE P95 (%) | Limit Violation P95 (%) |",
            "|---|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        diagnostics = dataset["supplementary_existing"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_drift_cell(diagnostics['tracking_nrmse_p95'])} | "
            f"{_drift_cell(diagnostics['limit_violation_p95'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def _csv(summary: Mapping[str, Any]) -> str:
    output = io.StringIO()
    fields = [
        "dataset_slug",
        "dataset",
        "n",
        "import_percentage",
        "dof_mapping_percentage",
        "valid_inertial_assets_percentage",
        "complete_non_placeholder_inertial_assets_percentage",
        "exact_unit_placeholder_inertial_assets_percentage",
        "genesis_finite_rollout_percentage",
        "all_three_import_percentage",
        "all_three_dof_mapping_percentage",
        "all_three_finite_rollout_percentage",
        "trajectory_coverage_percentage",
        "complete_genesis_readiness_percentage",
    ]
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    for dataset in summary["datasets"]:
        existing = dataset["primary_existing"]
        finite = dataset["finite_rollout_v2_1"]
        writer.writerow(
            {
                "dataset_slug": dataset["dataset_slug"],
                "dataset": dataset["dataset_name"],
                "n": dataset["n"],
                "import_percentage": existing["import_success"]["percentage"],
                "dof_mapping_percentage": existing["dof_mapping"]["percentage"],
                "valid_inertial_assets_percentage": dataset["inertial_assets"][
                    "mathematically_valid"
                ]["percentage"],
                "complete_non_placeholder_inertial_assets_percentage": dataset[
                    "inertial_assets"
                ]["complete_non_placeholder"]["percentage"],
                "exact_unit_placeholder_inertial_assets_percentage": dataset[
                    "inertial_assets"
                ]["exact_unit_placeholder"]["percentage"],
                "genesis_finite_rollout_percentage": finite["genesis"]["percentage"],
                "all_three_import_percentage": dataset["all_three_readiness"][
                    "import_success"
                ]["percentage"],
                "all_three_dof_mapping_percentage": dataset[
                    "all_three_readiness"
                ]["dof_mapping"]["percentage"],
                "all_three_finite_rollout_percentage": dataset[
                    "all_three_finite_rollout_v2_1"
                ]["percentage"],
                "trajectory_coverage_percentage": existing[
                    "actuated_trajectory_coverage"
                ]["percentage"],
                "complete_genesis_readiness_percentage": dataset[
                    "complete_genesis_readiness"
                ]["percentage"],
            }
        )
    return output.getvalue()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formal-prepared", type=Path, required=True)
    parser.add_argument("--articraft-prepared", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--old-summary", type=Path, required=True)
    parser.add_argument("--strict-summary", type=Path)
    parser.add_argument("--functional-summary", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise _base.AggregateError(f"output already exists: {args.out}")
    summary = aggregate(
        args.formal_prepared,
        args.articraft_prepared,
        args.run_root,
        args.old_summary,
        args.strict_summary,
        args.functional_summary,
    )
    args.out.mkdir(parents=True)
    _base._atomic_json(args.out / "summary.json", summary)
    _base._atomic_text(args.out / "report.md", _report(summary))
    _base._atomic_text(
        args.out / "supplementary_diagnostics.md", _supplementary_report(summary)
    )
    _base._atomic_text(args.out / "table5.csv", _csv(summary))
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "out": str(args.out.resolve()),
                "summary_sha256": summary["summary_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except _base.AggregateError as error:
        print(f"table5_stable_v2_1_aggregate: {error}", file=sys.stderr)
        raise SystemExit(2)
