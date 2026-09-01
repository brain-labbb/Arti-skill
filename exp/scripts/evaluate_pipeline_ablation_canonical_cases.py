#!/usr/bin/env python3
"""Development evaluator for externally owned canonical template cases.

This is infrastructure, not a hidden benchmark.  Cases, field mappings, source
bindings, and any available gold are read from an evaluator-owned JSON spec.
The candidate template's sampler and authored corners are never used to choose
the cases.  Metrics without independent gold remain N/E and cannot silently
become a strict success.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping


REPO_ROOT = Path(__file__).resolve().parents[2]
TEMPLATE_ROOT = REPO_ROOT / "arti-template"
if str(TEMPLATE_ROOT) not in sys.path:
    sys.path.insert(0, str(TEMPLATE_ROOT))

PASS = "PASS"
FAIL = "FAIL"
NOT_EVALUABLE = "N/E"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def load_template(path: Path) -> Any:
    module_name = f"_pipeline_ablation_candidate_{sha256_file(path)[:16]}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot import candidate template: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    return module


def result(status: str, detail: str, **evidence: Any) -> dict[str, Any]:
    row = {"status": status, "detail": detail}
    if evidence:
        row["evidence"] = evidence
    return row


def combine(rows: Iterable[dict[str, Any]], *, empty_reason: str) -> dict[str, Any]:
    values = list(rows)
    if not values:
        return result(NOT_EVALUABLE, empty_reason)
    statuses = {str(row.get("status")) for row in values}
    if FAIL in statuses:
        status = FAIL
    elif NOT_EVALUABLE in statuses:
        status = NOT_EVALUABLE
    else:
        status = PASS
    counts = {name: sum(row.get("status") == name for row in values) for name in (PASS, FAIL, NOT_EVALUABLE)}
    return result(status, f"aggregated {len(values)} evaluator-owned case result(s)", counts=counts)


def stable_value(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def vector(value: Any, *, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} must be a length-three vector")
    parsed = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{field} must contain finite values")
    return parsed  # type: ignore[return-value]


def distance(a: Any, b: Any) -> float:
    av = vector(a, field="actual vector")
    bv = vector(b, field="expected vector")
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(av, bv)))


def axis_error_and_flip(a: Any, b: Any, *, unsigned: bool) -> tuple[float, bool]:
    av = vector(a, field="actual axis")
    bv = vector(b, field="expected axis")
    an = math.sqrt(sum(value * value for value in av))
    bn = math.sqrt(sum(value * value for value in bv))
    if an <= 0.0 or bn <= 0.0:
        raise ValueError("axes must be nonzero")
    dot = sum(x * y for x, y in zip(av, bv)) / (an * bn)
    flipped = unsigned and dot < 0.0
    if unsigned:
        dot = abs(dot)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot)))), flipped


def finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def name_matches(name: str, aliases: Iterable[Any]) -> bool:
    normalized = str(name).strip().lower().replace("-", "_")
    for alias_value in aliases:
        alias = str(alias_value).strip().lower().replace("-", "_")
        if normalized == alias:
            return True
    return False


def domain_check(module: Any, spec: Mapping[str, Any]) -> dict[str, Any]:
    target = spec.get("target_domain")
    if not isinstance(target, dict) or not isinstance(target.get("slots"), dict):
        return result(NOT_EVALUABLE, "no independent target-domain contract")
    adapter = spec.get("adapter") if isinstance(spec.get("adapter"), dict) else {}
    field_map = adapter.get("config_fields") if isinstance(adapter.get("config_fields"), dict) else {}
    domain = getattr(module, "TEMPLATE_DOMAIN", None)
    slots = getattr(domain, "slots", None)
    if slots is None:
        return result(FAIL, "candidate has no inspectable TEMPLATE_DOMAIN")
    actual_slots: dict[str, list[Any]] = {}
    duplicate_slots: list[str] = []
    for slot in slots:
        name = getattr(slot, "name", None)
        values = getattr(slot, "values", None)
        if isinstance(name, str) and values is not None:
            if name in actual_slots:
                duplicate_slots.append(name)
            actual_slots[name] = list(values)

    records: list[dict[str, Any]] = []
    mapped_slot_names = {
        str(field_map[name])
        for name in target["slots"]
        if isinstance(field_map.get(name), str)
    }
    unmapped_slots = sorted(set(actual_slots) - mapped_slot_names)
    passed = not duplicate_slots and not unmapped_slots
    for canonical_name, expected_values in target["slots"].items():
        template_name = field_map.get(canonical_name)
        actual_values = actual_slots.get(str(template_name)) if isinstance(template_name, str) else None
        expected_keys = {stable_value(value) for value in expected_values}
        actual_keys = {stable_value(value) for value in actual_values or []}
        missing = sorted(expected_keys - actual_keys)
        extra = sorted(actual_keys - expected_keys)
        exact = actual_values is not None and not missing and not extra
        passed = passed and exact
        records.append(
            {
                "canonical_slot": canonical_name,
                "template_field": template_name,
                "missing_values": missing,
                "extra_values": extra,
                "pass": exact,
            }
        )
    return result(
        PASS if passed else FAIL,
        "candidate domain compared with frozen external domain",
        records=records,
        duplicate_candidate_slots=sorted(set(duplicate_slots)),
        unmapped_candidate_slots=unmapped_slots,
    )


def source_revision_check(module: Any, spec: Mapping[str, Any]) -> dict[str, Any]:
    contract = spec.get("source_contract")
    if not isinstance(contract, dict) or not isinstance(contract.get("records"), list):
        return result(NOT_EVALUABLE, "no independent source-revision contract")
    adapter = spec.get("adapter") if isinstance(spec.get("adapter"), dict) else {}
    export_name = adapter.get("source_provenance_export")
    provenance = getattr(module, str(export_name), None) if export_name else None
    if not isinstance(provenance, Mapping):
        return result(FAIL, f"candidate lacks mapping export {export_name!r}")
    records: list[dict[str, Any]] = []
    passed = True
    for expected in contract["records"]:
        record_id = str(expected.get("record_id", ""))
        expected_revision = str(expected.get("revision", ""))
        actual = provenance.get(record_id)
        actual_revision = actual.get("revision") if isinstance(actual, Mapping) else actual
        matched = str(actual_revision) == expected_revision
        passed = passed and matched
        records.append(
            {
                "record_id": record_id,
                "expected_revision": expected_revision,
                "actual_revision": actual_revision,
                "pass": matched,
            }
        )
    return result(
        PASS if passed else FAIL,
        "declared source revision compared with frozen binding; content fidelity is not inferred",
        records=records,
    )


def role_candidates(model: Any, roles: list[dict[str, Any]]) -> tuple[dict[str, list[str]], dict[str, Any]]:
    parts = getattr(model, "parts", getattr(model, "links", None))
    if parts is None:
        return {}, result(FAIL, "built object has no inspectable parts/links")
    names = [str(getattr(part, "name", "")) for part in parts]
    candidates: dict[str, list[str]] = {}
    records: list[dict[str, Any]] = []
    passed = True
    for role in roles:
        role_id = str(role["role_id"])
        hits = sorted(name for name in names if name_matches(name, role.get("aliases", ())))
        exact = role.get("exact_count")
        minimum = int(role.get("min_count", 1))
        ok = len(hits) == int(exact) if exact is not None else len(hits) >= minimum
        passed = passed and ok
        candidates[role_id] = hits
        records.append(
            {
                "role_id": role_id,
                "minimum": minimum if exact is None else None,
                "exact": int(exact) if exact is not None else None,
                "matched_parts": hits,
                "pass": ok,
            }
        )
    return candidates, result(PASS if passed else FAIL, "parts compared with evaluator-owned role gold", records=records)


def joint_checks(
    model: Any,
    joint_gold: list[dict[str, Any]],
    candidates: Mapping[str, list[str]],
) -> dict[str, dict[str, Any]]:
    joints = getattr(model, "articulations", getattr(model, "joints", None))
    if joints is None:
        failed = result(FAIL, "built object has no inspectable articulations/joints")
        return {name: failed for name in ("joint_detection", "joint_axis", "joint_origin", "joint_limit")}
    used: set[int] = set()
    records: dict[str, list[dict[str, Any]]] = {
        "joint_detection": [],
        "joint_axis": [],
        "joint_origin": [],
        "joint_limit": [],
    }
    for expected in joint_gold:
        parent_names = set(candidates.get(str(expected.get("parent_role")), ()))
        child_names = set(candidates.get(str(expected.get("child_role")), ()))
        expected_type = str(expected.get("type", "")).lower()
        eligible_indices: list[int] = []
        for index, joint in enumerate(joints):
            actual_type = getattr(joint, "articulation_type", getattr(joint, "joint_type", ""))
            actual_type = str(getattr(actual_type, "value", actual_type)).lower()
            if (
                index not in used
                and str(getattr(joint, "parent", "")) in parent_names
                and str(getattr(joint, "child", "")) in child_names
                and actual_type == expected_type
            ):
                eligible_indices.append(index)
        joint_id = str(expected.get("joint_id", ""))
        if len(eligible_indices) != 1:
            reason = (
                "no matching GT movable joint"
                if not eligible_indices
                else f"ambiguous joint match: {len(eligible_indices)} eligible candidates"
            )
            failed = {
                "joint_id": joint_id,
                "eligible_candidate_indices": eligible_indices,
                "pass": False,
                "reason": reason,
            }
            records["joint_detection"].append(failed)
            records["joint_axis"].append(dict(failed))
            records["joint_origin"].append(dict(failed))
            records["joint_limit"].append(dict(failed))
            continue
        matched_index = eligible_indices[0]
        records["joint_detection"].append(
            {
                "joint_id": joint_id,
                "eligible_candidate_indices": eligible_indices,
                "pass": True,
            }
        )
        used.add(matched_index)
        joint = joints[matched_index]
        axis_tol = float(expected.get("axis_tolerance_degrees", 0.0))
        axis_flipped: bool | None = None
        try:
            axis_error, axis_flipped = axis_error_and_flip(
                getattr(joint, "axis", None),
                expected.get("axis"),
                unsigned=bool(expected.get("axis_unsigned", False)),
            )
            records["joint_axis"].append(
                {
                    "joint_id": joint_id,
                    "error_degrees": axis_error,
                    "tolerance_degrees": axis_tol,
                    "axis_flipped": axis_flipped,
                    "pass": axis_error <= axis_tol,
                }
            )
        except (TypeError, ValueError, OverflowError) as exc:
            records["joint_axis"].append(
                {
                    "joint_id": joint_id,
                    "pass": False,
                    "reason": f"malformed axis: {type(exc).__name__}: {exc}",
                }
            )
        origin = getattr(getattr(joint, "origin", None), "xyz", None)
        origin_tol = float(expected.get("origin_tolerance", 0.0))
        try:
            origin_error = distance(origin, expected.get("origin_xyz"))
            records["joint_origin"].append(
                {
                    "joint_id": joint_id,
                    "error": origin_error,
                    "tolerance": origin_tol,
                    "pass": origin_error <= origin_tol,
                }
            )
        except (TypeError, ValueError, OverflowError) as exc:
            records["joint_origin"].append(
                {
                    "joint_id": joint_id,
                    "pass": False,
                    "reason": f"malformed origin: {type(exc).__name__}: {exc}",
                }
            )
        limits = getattr(joint, "motion_limits", getattr(joint, "limit", None))
        actual_lower = getattr(limits, "lower", None) if limits is not None else None
        actual_upper = getattr(limits, "upper", None) if limits is not None else None
        expected_lower = expected.get("lower")
        expected_upper = expected.get("upper")
        limit_tol = float(expected.get("limit_tolerance", 0.0))
        compared_lower = actual_lower
        compared_upper = actual_upper
        limit_reason = None
        if not all(finite_number(value) for value in (actual_lower, actual_upper, expected_lower, expected_upper)):
            limit_ok = False
            limit_reason = "limits must be finite numeric bounds"
        elif axis_flipped is None:
            limit_ok = False
            limit_reason = "axis coordinate direction is not evaluable"
        else:
            if axis_flipped:
                compared_lower, compared_upper = -float(actual_upper), -float(actual_lower)
            limit_ok = (
                float(compared_lower) <= float(expected_lower) + limit_tol
                and float(compared_upper) >= float(expected_upper) - limit_tol
            )
        records["joint_limit"].append(
            {
                "joint_id": joint_id,
                "actual": [actual_lower, actual_upper],
                "axis_aligned_actual": [compared_lower, compared_upper],
                "axis_flipped": axis_flipped,
                "required_coverage": [expected_lower, expected_upper],
                "tolerance": limit_tol,
                "reason": limit_reason,
                "pass": limit_ok,
            }
        )
    output: dict[str, dict[str, Any]] = {}
    for metric, metric_records in records.items():
        passed = bool(metric_records) and all(row["pass"] for row in metric_records)
        output[metric] = result(PASS if passed else FAIL, f"{metric} compared with evaluator-owned joint gold", records=metric_records)
    return output


def evaluate_case(module: Any, spec: Mapping[str, Any], case: Mapping[str, Any]) -> dict[str, Any]:
    adapter = spec.get("adapter") if isinstance(spec.get("adapter"), dict) else {}
    field_map = adapter.get("config_fields") if isinstance(adapter.get("config_fields"), dict) else {}
    resolved_map = adapter.get("resolved_fields") if isinstance(adapter.get("resolved_fields"), dict) else field_map
    metrics: dict[str, dict[str, Any]] = {}
    try:
        config_class = getattr(module, str(adapter["config_class"]))
        if not dataclasses.is_dataclass(config_class) or not isinstance(config_class, type):
            raise TypeError("canonical adapter requires an exported dataclass Config type")
        canonical = case.get("canonical_values")
        if not isinstance(canonical, dict):
            raise ValueError("canonical_values must be a mapping")
        overrides = {str(field_map[name]): value for name, value in canonical.items()}
        defaults = adapter.get("config_defaults", {})
        if not isinstance(defaults, dict):
            raise ValueError("adapter.config_defaults must be a mapping")
        config_values = {**defaults, **overrides}
        init_fields = {field.name for field in dataclasses.fields(config_class) if field.init}
        missing_fields = sorted(init_fields - set(config_values))
        extra_fields = sorted(set(config_values) - init_fields)
        if missing_fields or extra_fields:
            raise ValueError(
                "evaluator-owned Config is not exact: "
                f"missing={missing_fields}, extra={extra_fields}"
            )
        config = config_class(**config_values)
        resolver_name = adapter.get("resolve_function")
        resolved = getattr(module, str(resolver_name))(config) if resolver_name else config
        preservation: list[dict[str, Any]] = []
        for canonical_name, expected_value in canonical.items():
            resolved_name = str(resolved_map[canonical_name])
            actual_value = getattr(resolved, resolved_name)
            preservation.append(
                {
                    "canonical_field": canonical_name,
                    "resolved_field": resolved_name,
                    "expected": expected_value,
                    "actual": actual_value,
                    "pass": stable_value(actual_value) == stable_value(expected_value),
                }
            )
        preserved = all(row["pass"] for row in preservation)
        metrics["canonical_value_preserved"] = result(
            PASS if preserved else FAIL,
            "resolved Config compared with evaluator-owned canonical values",
            records=preservation,
        )
        build_name = str(adapter["build_function"])
        build_function = getattr(module, build_name)
        model = build_function(config)
        metrics["canonical_case_execution"] = result(PASS, "canonical Config resolved and built")
    except BaseException as exc:  # fail closed at the case boundary
        metrics.setdefault(
            "canonical_value_preserved",
            result(FAIL, f"canonical Config adaptation failed: {type(exc).__name__}: {exc}"),
        )
        metrics["canonical_case_execution"] = result(
            FAIL, f"canonical Config execution failed: {type(exc).__name__}: {exc}"
        )
        for name in ("role_presence", "joint_detection", "joint_axis", "joint_origin", "joint_limit"):
            metrics[name] = result(FAIL, "canonical case did not produce an evaluable object")
        return {"case_id": case.get("case_id"), "metrics": metrics}

    gold = case.get("gold") if isinstance(case.get("gold"), dict) else {}
    roles = gold.get("roles")
    if isinstance(roles, list) and roles:
        candidates, metrics["role_presence"] = role_candidates(model, roles)
    else:
        candidates = {}
        metrics["role_presence"] = result(NOT_EVALUABLE, "no independent role gold")
    joints = gold.get("joints")
    if isinstance(joints, list) and joints and isinstance(roles, list) and roles:
        metrics.update(joint_checks(model, joints, candidates))
    else:
        for name in ("joint_detection", "joint_axis", "joint_origin", "joint_limit"):
            metrics[name] = result(NOT_EVALUABLE, "no independent joint/role gold")
    return {"case_id": case.get("case_id"), "metrics": metrics}


def status_for(metrics: Mapping[str, dict[str, Any]], required: Iterable[Any]) -> str:
    statuses = [str(metrics.get(str(name), {}).get("status", NOT_EVALUABLE)) for name in required]
    if FAIL in statuses:
        return FAIL
    if not statuses or NOT_EVALUABLE in statuses:
        return NOT_EVALUABLE
    return PASS


def evaluate(template_path: Path, spec_path: Path) -> dict[str, Any]:
    template_path = template_path.resolve()
    spec_path = spec_path.resolve()
    spec = load_json(spec_path)
    if spec.get("schema_version") != "pipeline_ablation_canonical_dev_v1":
        raise ValueError("unsupported development canonical-case spec")
    if spec.get("evaluation_scope") != "development_fixture_only":
        raise ValueError("this evaluator only accepts development_fixture_only specs")
    module = load_template(template_path)
    cases_spec = spec.get("cases")
    if not isinstance(cases_spec, list) or not cases_spec:
        raise ValueError("spec must own at least one canonical case")

    task_metrics: dict[str, dict[str, Any]] = {
        "target_domain_exact": domain_check(module, spec),
        "source_revision_binding": source_revision_check(module, spec),
    }
    cases = [evaluate_case(module, spec, case) for case in cases_spec]
    case_metric_names = (
        "canonical_value_preserved",
        "canonical_case_execution",
        "role_presence",
        "joint_detection",
        "joint_axis",
        "joint_origin",
        "joint_limit",
    )
    for metric_name in case_metric_names:
        task_metrics[metric_name] = combine(
            (case["metrics"][metric_name] for case in cases),
            empty_reason="no evaluator-owned canonical cases",
        )
    for unavailable in spec.get("not_evaluable_metrics", []):
        metric_id = str(unavailable["metric_id"])
        if metric_id in task_metrics:
            raise ValueError(f"metric {metric_id!r} is both evaluated and declared N/E")
        task_metrics[metric_id] = result(NOT_EVALUABLE, str(unavailable["reason"]))

    smoke_required = spec.get("development_smoke_required_metrics", [])
    strict_required = spec.get("strict_required_metrics", [])
    return {
        "schema_version": "pipeline_ablation_canonical_dev_report_v1",
        "evaluation_scope": "development_fixture_only",
        "claim_boundary": spec.get("claim_boundary"),
        "task_id": spec.get("task_id"),
        "template": str(template_path),
        "template_sha256": sha256_file(template_path),
        "spec": str(spec_path),
        "spec_sha256": sha256_file(spec_path),
        "case_source": "evaluator_owned_json; candidate samplers/seeds/corners are not case generators",
        "role_alias_policy": "normalized exact equality only; no token, prefix, or substring matching",
        "joint_matcher_policy": (
            "fail closed unless each evaluator-owned gold joint has exactly one unused candidate "
            "with matching parent role, child role, and type; no greedy ambiguity resolution"
        ),
        "axis_limit_rule": (
            "when axis_unsigned=true and the candidate axis is antiparallel, compare limits after "
            "mapping candidate [lower, upper] to expected coordinates as [-upper, -lower]"
        ),
        "development_smoke_status": status_for(task_metrics, smoke_required),
        "strict_success_status": status_for(task_metrics, strict_required),
        "strict_success_denominator_note": "N/E is never counted as strict success",
        "metrics": task_metrics,
        "cases": cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        report = evaluate(args.template, args.spec)
    except BaseException as exc:
        report = {
            "schema_version": "pipeline_ablation_canonical_dev_report_v1",
            "evaluation_scope": "development_fixture_only",
            "development_smoke_status": FAIL,
            "strict_success_status": NOT_EVALUABLE,
            "error": f"{type(exc).__name__}: {exc}",
        }
    payload = json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report.get("development_smoke_status") == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
