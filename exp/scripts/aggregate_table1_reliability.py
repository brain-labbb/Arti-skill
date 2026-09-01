#!/usr/bin/env python3
"""Aggregate Nano3D Table 1 reliability evidence without widening claims.

This script performs no generation, network, or API work. It reads the frozen
common-authoring contract and prepare evidence together with the historical
PV-A seed/corner evidence, partial current-HEAD diagnostic, and supplementary
LAM, Articraft, and Infinite Mobility audits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


for _thread_env in (
    "OPENBLAS_NUM_THREADS",
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
):
    os.environ[_thread_env] = "1"


SCHEMA_VERSION = 1
PROTOCOL_ID = "nano3d_table1_reliability_combined_v1"
EXPECTED_METHODS = ("pva", "lam", "articraft")
EXPECTED_AUTHORING_RUNS = 162
MINIMUM_COMMON_FROZEN_CHECKS = 112
NON_CAPABILITY_BLOCKER_CODES = {
    "PROTOCOL_METHOD_NOT_READY",
    "PROTOCOL_GLOBAL_NOT_READY",
    "GLOBAL_NOT_READY",
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "exp/runtime/table1_reliability/combined",
    )
    return parser.parse_args()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_if_regular(path: Path) -> str | None:
    try:
        return sha256_file(path) if path.is_file() and not path.is_symlink() else None
    except OSError:
        return None


def repo_path(repo_root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        return None
    try:
        path = (repo_root / value).resolve(strict=True)
    except OSError:
        return None
    if repo_root not in path.parents:
        return None
    try:
        if path.relative_to(repo_root).as_posix() != value:
            return None
    except ValueError:
        return None
    return path


def git_value(checkout: Path, arguments: list[str]) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(checkout), *arguments],
            env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def audit_implementations(
    repo_root: Path, protocol: dict[str, Any]
) -> tuple[dict[str, dict[str, Any]], dict[str, str]]:
    records: dict[str, dict[str, Any]] = {}
    file_hashes: dict[str, str] = {}
    methods = protocol.get("methods")
    methods = methods if isinstance(methods, dict) else {}
    for method in EXPECTED_METHODS:
        row = methods.get(method)
        row = row if isinstance(row, dict) else {}
        implementation = row.get("implementation")
        implementation = implementation if isinstance(implementation, dict) else {}
        checkout = repo_path(repo_root, implementation.get("checkout_path"))
        entrypoint = repo_path(repo_root, implementation.get("entrypoint"))
        adapter = repo_path(repo_root, row.get("adapter_entrypoint"))
        expected_adapter = repo_root / f"exp/scripts/run_table1_{method}_authoring.py"
        head = git_value(checkout, ["rev-parse", "HEAD"]) if checkout else None
        tree = git_value(checkout, ["rev-parse", "HEAD^{tree}"]) if checkout else None
        git_root = (
            git_value(checkout, ["rev-parse", "--show-toplevel"]) if checkout else None
        )
        tracked_status = (
            git_value(checkout, ["status", "--porcelain=v1", "--untracked-files=no"])
            if checkout
            else None
        )
        relative_entrypoint = None
        entrypoint_tracked = False
        entrypoint_matches_head = False
        if checkout and entrypoint and checkout in entrypoint.parents:
            relative_entrypoint = entrypoint.relative_to(checkout).as_posix()
            entrypoint_tracked = (
                git_value(
                    checkout,
                    ["ls-files", "--error-unmatch", "--", relative_entrypoint],
                )
                == relative_entrypoint
            )
            entrypoint_matches_head = (
                git_value(
                    checkout,
                    ["diff", "--name-only", "HEAD", "--", relative_entrypoint],
                )
                == ""
            )
        entrypoint_hash = (
            sha256_file(entrypoint)
            if entrypoint and entrypoint.is_file() and not entrypoint.is_symlink()
            else None
        )
        adapter_hash = (
            sha256_file(adapter)
            if adapter and adapter.is_file() and not adapter.is_symlink()
            else None
        )
        if entrypoint_hash is not None:
            file_hashes[f"{method}_implementation_entrypoint"] = entrypoint_hash
        if adapter_hash is not None:
            file_hashes[f"{method}_adapter_entrypoint"] = adapter_hash
        checks = {
            "checkout_unique_path_present": checkout is not None and checkout.is_dir(),
            "checkout_is_git_root": bool(
                checkout and git_root and Path(git_root).resolve() == checkout
            ),
            "commit_matches": head == implementation.get("commit"),
            "git_tree_matches": tree == implementation.get("git_tree"),
            "tracked_clean_matches": implementation.get("tracked_clean_at_freeze")
            is True
            and tracked_status == "",
            "provenance_nonempty": isinstance(implementation.get("provenance"), str)
            and bool(implementation["provenance"].strip()),
            "entrypoint_regular": bool(
                entrypoint and entrypoint.is_file() and not entrypoint.is_symlink()
            ),
            "entrypoint_tracked": entrypoint_tracked,
            "entrypoint_matches_head": entrypoint_matches_head,
            "adapter_path_exact": adapter == expected_adapter,
            "adapter_regular": bool(
                adapter and adapter.is_file() and not adapter.is_symlink()
            ),
            "adapter_sha256_matches": adapter_hash == row.get("adapter_sha256"),
        }
        records[method] = {
            "adapter_entrypoint": row.get("adapter_entrypoint"),
            "adapter_sha256": adapter_hash,
            "checks": checks,
            "checkout_path": implementation.get("checkout_path"),
            "commit": head,
            "entrypoint": implementation.get("entrypoint"),
            "entrypoint_sha256": entrypoint_hash,
            "git_tree": tree,
            "provenance": implementation.get("provenance"),
            "tracked_clean": (
                tracked_status == "" if tracked_status is not None else None
            ),
        }
    return records, file_hashes


def all_boolean_values(mapping: Any, expected: bool = True) -> bool:
    return (
        isinstance(mapping, dict)
        and bool(mapping)
        and all(value is expected for value in mapping.values())
    )


def named_boolean_self_check_ready(
    self_check: Any, *, minimum_checks: int
) -> bool:
    """Require declared counts to match a complete, all-true named check map."""

    if not isinstance(self_check, dict):
        return False
    checks = self_check.get("checks")
    if not isinstance(checks, dict) or len(checks) < minimum_checks:
        return False
    return (
        self_check.get("status") == "PASS"
        and self_check.get("passed")
        == self_check.get("total")
        == len(checks)
        and all_boolean_values(checks)
    )


def string_list(value: Any) -> list[str] | None:
    if not isinstance(value, list) or not all(
        isinstance(item, str) and bool(item.strip()) for item in value
    ):
        return None
    return value


def normalize_blocker(value: str) -> str:
    """Make formatting-only differences irrelevant without weakening the gate."""

    return " ".join(value.split())


def is_non_capability_gate(value: str, code: str | None = None) -> bool:
    """Filter only execution-invocation gates, never implementation capability gaps."""

    normalized = normalize_blocker(value)
    normalized_code = normalize_blocker(code).upper() if isinstance(code, str) else ""
    normalized_upper = normalized.upper()
    gate_code = normalized_code.replace("-", "_").replace(" ", "_")
    gate_text = normalized_upper.replace("-", "_").replace(" ", "_")
    if gate_code in NON_CAPABILITY_BLOCKER_CODES:
        return True
    if any(marker in gate_text for marker in NON_CAPABILITY_BLOCKER_CODES):
        return True
    if "PAID CONFIRMATION" in normalized_upper:
        return True
    if "CREDENTIAL" in normalized_upper and (
        "AT EXECUTION" in normalized_upper
        or "EXECUTION TIME" in normalized_upper
        or "AT RUN TIME" in normalized_upper
    ):
        return True
    return False


def normalized_blocker_list(value: list[str]) -> list[str]:
    return sorted(normalize_blocker(item) for item in value)


def protocol_capability_blockers(
    protocol: dict[str, Any], method: str
) -> list[str] | None:
    readiness = protocol.get("execution_readiness")
    if not isinstance(readiness, dict):
        return None
    method_blockers = readiness.get("method_blockers")
    if not isinstance(method_blockers, dict) or method not in method_blockers:
        return None
    blockers = string_list(method_blockers[method])
    if blockers is None:
        return None
    return [item for item in blockers if not is_non_capability_gate(item)]


def pva_capability_blockers(manifest: dict[str, Any]) -> list[str] | None:
    """Read the adapter's own capability audit, never its protocol copy."""

    blockers = string_list(manifest.get("capability_blockers"))
    if blockers is None:
        return None
    return [item for item in blockers if not is_non_capability_gate(item)]


def lam_capability_blockers(summary: dict[str, Any]) -> list[str] | None:
    """Exclude invocation state that does not change the implemented capability."""

    runtime = string_list(summary.get("runtime_blockers"))
    adapter = string_list(summary.get("adapter_blockers"))
    if runtime is None or adapter is None:
        return None
    return [
        item
        for item in [*runtime, *adapter]
        if not is_non_capability_gate(item)
    ]


def articraft_capability_blockers(preflight: dict[str, Any]) -> list[str] | None:
    """Convert structured adapter findings to the protocol's normalized text list."""

    readiness = preflight.get("readiness")
    if not isinstance(readiness, dict):
        return None
    rows = readiness.get("blockers")
    if not isinstance(rows, list):
        return None
    normalized: list[str] = []
    for row in rows:
        if not isinstance(row, dict):
            return None
        code = row.get("code")
        detail = row.get("detail")
        if not isinstance(code, str) or not code or not isinstance(detail, str) or not detail.strip():
            return None
        if not is_non_capability_gate(detail, code):
            normalized.append(detail)
    return normalized


def capability_blockers_match_protocol(
    protocol: dict[str, Any], method: str, actual_blockers: list[str] | None
) -> bool:
    """Require complete, normalized agreement between adapter evidence and protocol."""

    expected_blockers = protocol_capability_blockers(protocol, method)
    return (
        actual_blockers is not None
        and expected_blockers is not None
        and normalized_blocker_list(actual_blockers)
        == normalized_blocker_list(expected_blockers)
    )


def common_frozen_consistency_ready(
    common_summary: dict[str, Any], common_self_check: dict[str, Any]
) -> bool:
    """Validate every reported frozen check without freezing its historical count."""

    frozen = common_summary.get("frozen_consistency")
    checks = common_self_check.get("checks")
    if not isinstance(frozen, dict) or not isinstance(checks, list):
        return False
    if not checks or not all(isinstance(row, dict) for row in checks):
        return False
    total = frozen.get("total_checks")
    passed = frozen.get("passed_checks")
    frozen_rows = [
        row
        for row in checks
        if isinstance(row, dict) and row.get("scope") == "frozen_consistency"
    ]
    return (
        common_summary.get("status") == "READY"
        and frozen.get("status") == "READY"
        and frozen.get("ready") is True
        and isinstance(total, int)
        and not isinstance(total, bool)
        and total >= MINIMUM_COMMON_FROZEN_CHECKS
        and passed == total == len(frozen_rows)
        and all(row.get("passed") is True for row in frozen_rows)
        and all(isinstance(row.get("passed"), bool) for row in frozen_rows)
        and common_self_check.get("pass") is True
    )


def common_adapter_readiness_matches_protocol(
    protocol: dict[str, Any],
    common_summary: dict[str, Any],
    common_self_check: dict[str, Any],
) -> bool:
    """Require per-method readiness evidence to match the frozen protocol."""

    execution = protocol.get("execution_readiness")
    if not isinstance(execution, dict):
        return False
    expected = execution.get("method_adapters_ready")
    if (
        not isinstance(expected, dict)
        or set(expected) != set(EXPECTED_METHODS)
        or not all(isinstance(expected[method], bool) for method in EXPECTED_METHODS)
        or not isinstance(protocol.get("execution_ready"), bool)
        or protocol["execution_ready"] is not all(expected.values())
    ):
        return False

    reported = common_summary.get("execution_adapter_readiness")
    if not isinstance(reported, dict):
        return False
    methods = reported.get("methods")
    if not isinstance(methods, dict) or set(methods) != set(EXPECTED_METHODS):
        return False

    checks = common_self_check.get("checks")
    if not isinstance(checks, list):
        return False
    adapter_rows = [
        row
        for row in checks
        if isinstance(row, dict) and row.get("scope") == "adapter_readiness"
    ]
    rows_by_id = {
        row.get("check_id"): row
        for row in adapter_rows
        if isinstance(row.get("check_id"), str)
    }
    expected_check_ids = {
        f"adapter.{method}.ready" for method in EXPECTED_METHODS
    }
    if (
        len(adapter_rows) != len(EXPECTED_METHODS)
        or set(rows_by_id) != expected_check_ids
        or common_self_check.get("adapter_readiness_excluded_from_gate") is not True
        or reported.get("declared_execution_ready") is not protocol["execution_ready"]
        or reported.get("ready") is not all(expected.values())
    ):
        return False

    for method in EXPECTED_METHODS:
        method_record = methods.get(method)
        expected_ready = expected[method]
        if (
            not isinstance(method_record, dict)
            or method_record.get("declared_ready") is not expected_ready
            or method_record.get("ready") is not expected_ready
            or rows_by_id[f"adapter.{method}.ready"].get("passed")
            is not expected_ready
        ):
            return False
    return True


def flat_bindings_match(bindings: Any, expected: dict[str, str]) -> bool:
    return isinstance(bindings, dict) and all(
        bindings.get(name) == value for name, value in expected.items()
    )


def nested_binding_matches(
    section: Any, expected_path: str, expected_hash: str
) -> bool:
    return (
        isinstance(section, dict)
        and section.get("path") == expected_path
        and section.get("sha256") == expected_hash
        and section.get("binding_matches") is True
    )


def nr_metrics(reason: str) -> dict[str, dict[str, Any]]:
    return {
        name: missing("N/R", reason)
        for name in (
            "api_cost",
            "artifact_saved",
            "executable",
            "final_success",
            "first_shot",
            "repair_turns",
            "tokens",
            "wall_time",
        )
    }


def ratio(numerator: int, denominator: int, evidence_class: str) -> dict[str, Any]:
    return {
        "denominator": denominator,
        "display": f"{numerator}/{denominator}",
        "evidence_class": evidence_class,
        "numerator": numerator,
        "state": "MEASURED",
        "value": numerator / denominator,
    }


def missing(display: str, reason: str) -> dict[str, Any]:
    state = {
        "N/R": "NOT_REPORTED_UNDER_EXACT_PROTOCOL",
        "N/E": "NOT_EVALUABLE",
        "N/A": "NOT_APPLICABLE",
    }[display]
    return {
        "denominator": None,
        "display": display,
        "numerator": None,
        "reason": reason,
        "state": state,
        "value": None,
    }


def pct(metric: dict[str, Any]) -> str:
    return f"{100.0 * metric['value']:.1f}% ({metric['display']})"


def check(
    checks: list[dict[str, Any]], check_id: str, passed: bool, **details: Any
) -> None:
    checks.append({"details": details, "id": check_id, "pass": bool(passed)})


def all_lam_checks_pass(self_check: dict[str, Any]) -> bool:
    values = self_check.get("checks")
    return (
        self_check.get("status") == "PASS"
        and isinstance(values, dict)
        and bool(values)
        and all(value is True for value in values.values())
    )


def all_articraft_checks_pass(self_check: dict[str, Any]) -> bool:
    values = self_check.get("checks")
    return (
        self_check.get("all_passed") is True
        and isinstance(values, list)
        and bool(values)
        and all(item.get("pass") is True for item in values)
        and self_check.get("passed") == self_check.get("total") == len(values)
    )


def all_infinite_checks_pass(self_check: dict[str, Any]) -> bool:
    values = self_check.get("checks")
    return (
        self_check.get("overall_pass") is True
        and isinstance(values, list)
        and bool(values)
        and all(item.get("passed") is True for item in values)
        and self_check.get("passed") == self_check.get("total") == len(values)
    )


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "frozen_protocol": repo_root
        / "exp/reference/table1_reliability_protocol_v1.json",
        "frozen_manifest": repo_root
        / "exp/reference/table1_reliability_common_authoring_v1.json",
        "hidden_specs": repo_root
        / "exp/reference/table1_reliability_hidden_specs_v1.json",
        "common_evaluator": repo_root
        / "exp/scripts/evaluate_table1_authoring_common.py",
        "common_preflight_script": repo_root
        / "exp/scripts/preflight_table1_authoring_common.py",
        "package_schema": repo_root
        / "exp/reference/table1_authoring_package_schema_v1.json",
        "result_schema": repo_root
        / "exp/reference/table1_authoring_result_schema_v1.json",
        "common_manifest": repo_root
        / "exp/runtime/table1_reliability/common/manifest.json",
        "common_summary": repo_root
        / "exp/runtime/table1_reliability/common/summary.json",
        "common_self_check": repo_root
        / "exp/runtime/table1_reliability/common/self_check.json",
        "evaluator_self_check": repo_root
        / "exp/runtime/table1_reliability/evaluator_self_test/self_check.json",
        "evaluator_positive_report": repo_root
        / "exp/runtime/table1_reliability/evaluator_self_test/positive_z_axis/report.json",
        "evaluator_negative_report": repo_root
        / "exp/runtime/table1_reliability/evaluator_self_test/negative_x_axis/report.json",
        "pva_authoring_manifest": repo_root
        / "exp/runtime/table1_reliability/pva_authoring_v1/experiment_manifest.json",
        "lam_authoring_manifest": repo_root
        / "exp/runtime/table1_reliability/lam_authoring_v1/manifest.json",
        "lam_authoring_summary": repo_root
        / "exp/runtime/table1_reliability/lam_authoring_v1/summary.json",
        "lam_authoring_self_check": repo_root
        / "exp/runtime/table1_reliability/lam_authoring_v1/self_check.json",
        "lam_authoring_report": repo_root
        / "exp/runtime/table1_reliability/lam_authoring_v1/report.md",
        "articraft_authoring_manifest": repo_root
        / "exp/runtime/table1_reliability/articraft_authoring_v1/experiment_manifest.json",
        "articraft_authoring_preflight": repo_root
        / "exp/runtime/table1_reliability/articraft_authoring_v1/preflight.json",
        "articraft_authoring_summary": repo_root
        / "exp/runtime/table1_reliability/articraft_authoring_v1/summary.json",
        "articraft_authoring_self_check": repo_root
        / "exp/runtime/table1_reliability/articraft_authoring_v1/self_check.json",
        "pva_frozen_seed_manifest": repo_root
        / "exp/runtime/nano3d_seed_reliability/manifest.json",
        "pva_frozen_seed_summary": repo_root
        / "exp/runtime/nano3d_seed_reliability/summary.json",
        "pva_frozen_corner_manifest": repo_root
        / "exp/runtime/nano3d_corner/manifest.json",
        "pva_frozen_corner_summary": repo_root
        / "exp/runtime/nano3d_corner/summary.json",
        "pva_current_manifest": repo_root
        / "exp/runtime/table1_reliability/pva_current/manifest.json",
        "pva_current_summary": repo_root
        / "exp/runtime/table1_reliability/pva_current/summary.json",
        "lam_release_summary": repo_root
        / "exp/runtime/table1_reliability/lam/summary.json",
        "lam_release_self_check": repo_root
        / "exp/runtime/table1_reliability/lam/self_check.json",
        "articraft_release_summary": repo_root
        / "exp/runtime/table1_reliability/articraft/summary.json",
        "articraft_release_self_check": repo_root
        / "exp/runtime/table1_reliability/articraft/self_check.json",
        "infinite_summary": repo_root
        / "exp/runtime/table1_reliability/infinite_mobility/summary.json",
        "infinite_self_check": repo_root
        / "exp/runtime/table1_reliability/infinite_mobility/self_check.json",
    }
    missing_inputs = [name for name, path in paths.items() if not path.is_file()]
    if missing_inputs:
        raise FileNotFoundError(f"missing required Table 1 inputs: {missing_inputs}")

    inputs = {
        name: read_json(path) if path.suffix == ".json" else None
        for name, path in paths.items()
    }
    input_hashes = {name: sha256_file(path) for name, path in paths.items()}

    protocol = inputs["frozen_protocol"]
    frozen_manifest = inputs["frozen_manifest"]
    common_manifest = inputs["common_manifest"]
    common_summary = inputs["common_summary"]
    common_self_check = inputs["common_self_check"]
    evaluator_self_check = inputs["evaluator_self_check"]
    evaluator_positive = inputs["evaluator_positive_report"]
    evaluator_negative = inputs["evaluator_negative_report"]
    pva_authoring = inputs["pva_authoring_manifest"]
    lam_authoring_manifest = inputs["lam_authoring_manifest"]
    lam_authoring_summary = inputs["lam_authoring_summary"]
    lam_authoring_self_check = inputs["lam_authoring_self_check"]
    articraft_authoring_manifest = inputs["articraft_authoring_manifest"]
    articraft_authoring_preflight = inputs["articraft_authoring_preflight"]
    articraft_authoring_summary = inputs["articraft_authoring_summary"]
    articraft_authoring_self_check = inputs["articraft_authoring_self_check"]
    protocol_sha256 = input_hashes["frozen_protocol"]
    common_manifest_sha256 = input_hashes["frozen_manifest"]
    evaluator_sha256 = input_hashes["common_evaluator"]
    package_schema_sha256 = input_hashes["package_schema"]
    result_schema_sha256 = input_hashes["result_schema"]
    hidden_specs_sha256 = input_hashes["hidden_specs"]
    implementation_audit, implementation_hashes = audit_implementations(
        repo_root, protocol
    )
    input_hashes.update(implementation_hashes)

    seed_manifest = inputs["pva_frozen_seed_manifest"]
    seed_summary = inputs["pva_frozen_seed_summary"]
    corner_manifest = inputs["pva_frozen_corner_manifest"]
    corner_summary = inputs["pva_frozen_corner_summary"]
    current_manifest = inputs["pva_current_manifest"]
    current_summary = inputs["pva_current_summary"]
    lam_release_summary = inputs["lam_release_summary"]
    articraft_release_summary = inputs["articraft_release_summary"]
    infinite_summary = inputs["infinite_summary"]

    frozen_templates = {item["slug"]: item for item in seed_manifest["templates"]}
    corner_templates = {item["slug"]: item for item in corner_manifest["templates"]}
    current_templates = {item["slug"]: item for item in current_manifest["templates"]}
    source_drift: list[dict[str, Any]] = []
    for slug in sorted(frozen_templates):
        frozen = frozen_templates[slug]
        source_path = Path(frozen["template_path"])
        source_exists = source_path.is_file()
        current_hash = sha256_file(source_path) if source_exists else None
        current_manifest_hash = current_templates.get(slug, {}).get("template_sha256")
        source_drift.append(
            {
                "current_file_exists": source_exists,
                "current_manifest_sha256": current_manifest_hash,
                "current_manifest_matches_file": current_hash == current_manifest_hash,
                "current_sha256": current_hash,
                "frozen_sha256": frozen["template_sha256"],
                "frozen_snapshot_matches_current": current_hash
                == frozen["template_sha256"],
                "slug": slug,
                "template_path": str(source_path),
            }
        )
    drifted = [
        item for item in source_drift if not item["frozen_snapshot_matches_current"]
    ]
    input_hashes.update(
        {
            f"pva_current_template_{item['slug']}": item["current_sha256"]
            for item in source_drift
            if item["current_sha256"] is not None
        }
    )

    pva_frozen_class = "FROZEN_SNAPSHOT_2026_08_05"
    pva_frozen = {
        "claim_boundary": (
            "Historical frozen-template evidence only. Current source hashes differ, so these "
            "values are not current-HEAD measurements."
        ),
        "corner_cases": ratio(
            corner_summary["corner_cases_passed"],
            corner_summary["corner_case_count"],
            pva_frozen_class,
        ),
        "evidence_class": pva_frozen_class,
        "full_qc_all_36_templates": ratio(
            seed_summary["templates_36_of_36_qc"],
            seed_summary["template_count"],
            pva_frozen_class,
        ),
        "regression_retention": missing(
            "N/A",
            "the frozen seed/corner protocols contain no repair event or pre/post-repair cohort",
        ),
        "seed_compile": ratio(
            seed_summary["compile_pass"], seed_summary["seed_count"], pva_frozen_class
        ),
        "seed_full_qc": ratio(
            seed_summary["qc_pass"], seed_summary["seed_count"], pva_frozen_class
        ),
        "source_drift": {
            "checked": len(source_drift),
            "drifted": len(drifted),
            "matches_frozen": len(source_drift) - len(drifted),
            "records": source_drift,
        },
        "strict_elapsed_seconds": seed_summary["strict_seed_elapsed_s"],
        "template_count": seed_summary["template_count"],
    }

    intended_current_cases = len(current_manifest["templates"]) * len(
        current_manifest["seeds"]
    )
    pva_current = {
        "claim_boundary": (
            "Incomplete diagnostic only; observed values are not promoted into the Table 1 "
            "current-HEAD row. The attempted full rerun was stopped after host PID/thread "
            "exhaustion; the one strict subprocess crash is an infrastructure-confounded "
            "diagnostic, not a method failure rate."
        ),
        "cohort_status": "BLOCKED_INFRASTRUCTURE_PARTIAL_RERUN",
        "infrastructure_blocker": (
            "Host-wide PID/thread pressure caused EAGAIN/OpenBLAS launch failures; no complete "
            "current-HEAD 33-template cohort is available."
        ),
        "intended_cases": intended_current_cases,
        "intended_templates": len(current_manifest["templates"]),
        "observed_diagnostics": {
            "artifact_saved": ratio(
                current_summary["artifact_saved"],
                current_summary["seed_count"],
                "CURRENT_HEAD_PARTIAL",
            ),
            "compile": ratio(
                current_summary["compile_pass"],
                current_summary["seed_count"],
                "CURRENT_HEAD_PARTIAL",
            ),
            "failure_types": current_summary["failure_types"],
            "strict_full_qc": ratio(
                current_summary["qc_pass"],
                current_summary["seed_count"],
                "CURRENT_HEAD_PARTIAL",
            ),
        },
        "observed_cases": current_summary["seed_count"],
        "observed_templates": current_summary["template_count"],
        "table1_metrics": {
            "corner_cases": missing("N/R", "current-HEAD corner rerun is absent"),
            "full_qc_all_36_templates": missing(
                "N/R", "only one of 33 intended templates completed"
            ),
            "regression_retention": missing(
                "N/A", "no repair event exists in the partial seed run"
            ),
            "seed_compile": missing("N/R", "only 36 of 1188 intended cases completed"),
            "seed_full_qc": missing("N/R", "only 36 of 1188 intended cases completed"),
        },
    }

    pva_blockers = pva_capability_blockers(pva_authoring)
    lam_blockers = lam_capability_blockers(lam_authoring_summary)
    articraft_blockers = articraft_capability_blockers(articraft_authoring_preflight)
    pva_blocker_count = len(pva_blockers) if pva_blockers is not None else None
    lam_blocker_count = len(lam_blockers) if lam_blockers is not None else None
    articraft_blocker_count = (
        len(articraft_blockers) if articraft_blockers is not None else None
    )
    authoring_records = {
        "PV-A": {
            "method_id": "pva",
            "status": pva_authoring.get("status"),
            "prepared_runs": pva_authoring.get("intended_runs"),
            "attempted_runs": 0,
            "completed_runs": 0,
            "evaluable_runs": 0,
            "provider_calls": pva_authoring.get("provider_calls_made"),
            "capability_blockers": pva_blockers,
            "claim_boundary": pva_authoring.get("claim_boundary"),
            "reason": (
                "PV-A prepared the frozen cohort but made zero authoring attempts because "
                "author-process hidden-spec read isolation is not enforced and the protocol "
                "execution gate is not ready."
            ),
        },
        "LAM": {
            "method_id": "lam",
            "status": lam_authoring_summary.get("status"),
            "prepared_runs": lam_authoring_summary.get("prepared_jobs"),
            "attempted_runs": lam_authoring_summary.get("authoring_attempts"),
            "completed_runs": lam_authoring_summary.get("completed_results"),
            "evaluable_runs": lam_authoring_summary.get("metric_denominator"),
            "provider_calls": lam_authoring_summary.get("provider_calls_made"),
            "capability_blockers": lam_blockers,
            "claim_boundary": lam_authoring_manifest.get("claim_boundary"),
            "reason": "LAM prepared the frozen cohort but made zero common-authoring attempts.",
        },
        "Articraft": {
            "method_id": "articraft",
            "status": articraft_authoring_summary.get("status"),
            "prepared_runs": articraft_authoring_summary.get("prepared_jobs"),
            "attempted_runs": articraft_authoring_summary.get("attempted_jobs"),
            "completed_runs": articraft_authoring_summary.get("completed_jobs"),
            "evaluable_runs": articraft_authoring_summary.get("evaluable_jobs"),
            "provider_calls": articraft_authoring_summary.get("provider_calls_made"),
            "capability_blockers": articraft_blockers,
            "claim_boundary": articraft_authoring_summary.get("claim_boundary"),
            "reason": (
                "Articraft prepared the frozen cohort but made zero common-authoring attempts."
            ),
        },
    }
    common_authoring: dict[str, dict[str, Any]] = {}
    for method, record in authoring_records.items():
        common_authoring[method] = {
            **record,
            "metrics": nr_metrics(record["reason"]),
        }

    public = infinite_summary["public_main_cohort"]
    matched = infinite_summary["matched_supplementary_cohort"]
    infinite_public = {
        "claim_boundary": (
            "Deterministic 180-second sensitivity over recorded public-factory telemetry, not a "
            "fresh rerun. Structural package QC at 300 seconds is supplementary and is not Full-QC."
        ),
        "compile_all_36_factories_180s": ratio(
            public["compile_only_all_36_180s"]["numerator"],
            public["compile_only_all_36_180s"]["denominator"],
            "RECORDED_TELEMETRY_180S_SENSITIVITY",
        ),
        "corner_cases": missing("N/R", public["corner_pass"]["reason"]),
        "factory_count": public["factory_count"],
        "full_qc_all_36_factories": missing("N/E", public["all_36_full_qc"]["reason"]),
        "regression_retention": missing(
            "N/E", public["regression_retention"]["reason"]
        ),
        "seed_compile_180s": ratio(
            public["seed_compile_180s"]["numerator"],
            public["seed_compile_180s"]["denominator"],
            "RECORDED_TELEMETRY_180S_SENSITIVITY",
        ),
        "seed_full_qc": missing("N/E", public["seed_full_qc"]["reason"]),
        "structural_package_qc_300s_supplementary": ratio(
            public["structural_package_qc_300s"]["numerator"],
            public["structural_package_qc_300s"]["denominator"],
            "FROZEN_STRUCTURAL_PACKAGE_AUDIT",
        ),
        "structural_package_qc_all_36_300s_supplementary": ratio(
            public["structural_package_qc_all_36_300s"]["numerator"],
            public["structural_package_qc_all_36_300s"]["denominator"],
            "FROZEN_STRUCTURAL_PACKAGE_AUDIT",
        ),
    }
    infinite_matched = {
        "claim_boundary": "Five-category supplementary subset of the same recorded telemetry.",
        "compile_all_36_factories_180s": ratio(
            matched["compile_only_all_36_180s"]["numerator"],
            matched["compile_only_all_36_180s"]["denominator"],
            "RECORDED_TELEMETRY_180S_SENSITIVITY",
        ),
        "factory_count": matched["factory_count"],
        "seed_compile_180s": ratio(
            matched["seed_compile_180s"]["numerator"],
            matched["seed_compile_180s"]["denominator"],
            "RECORDED_TELEMETRY_180S_SENSITIVITY",
        ),
        "seed_full_qc": missing("N/E", matched["seed_full_qc"]["reason"]),
        "structural_package_qc_300s_supplementary": ratio(
            matched["structural_package_qc_300s"]["numerator"],
            matched["structural_package_qc_300s"]["denominator"],
            "FROZEN_STRUCTURAL_PACKAGE_AUDIT",
        ),
    }

    supplementary_release = {
        "Articraft": {
            "claim_boundary": articraft_release_summary[
                "supplementary_official_release_audit"
            ]["claim_boundary"],
            "included_in_common_authoring_metrics": False,
            "release_index_rows": articraft_release_summary[
                "supplementary_official_release_audit"
            ]["records_index"]["rows"],
            "release_rating_ge_4": {
                "denominator": articraft_release_summary[
                    "supplementary_official_release_audit"
                ]["rating"]["effective"]["rating_ge_4_denominator"],
                "numerator": articraft_release_summary[
                    "supplementary_official_release_audit"
                ]["rating"]["effective"]["rating_ge_4_count"],
            },
        },
        "LAM": {
            "all_core_fields_nonempty": lam_release_summary[
                "official_release_telemetry"
            ]["all_core_fields_nonempty"],
            "claim_boundary": lam_release_summary["official_release_telemetry"][
                "claim_boundary"
            ],
            "included_in_common_authoring_metrics": False,
            "release_records": lam_release_summary["official_release_telemetry"][
                "all_core_fields_nonempty"
            ]["denominator"],
            "status_counts": lam_release_summary["official_release_telemetry"][
                "status_counts"
            ],
            "tier_counts": lam_release_summary["official_release_telemetry"][
                "tier_counts"
            ],
        },
    }

    checks: list[dict[str, Any]] = []
    expected_seeds = list(range(36))
    check(
        checks, "all_required_inputs_exist", not missing_inputs, input_count=len(paths)
    )
    expected_bindings = {
        "protocol_sha256": protocol_sha256,
        "manifest_sha256": common_manifest_sha256,
        "hidden_specs_sha256": hidden_specs_sha256,
        "package_schema_sha256": package_schema_sha256,
        "result_schema_sha256": result_schema_sha256,
        "common_evaluator_sha256": evaluator_sha256,
    }
    check(
        checks,
        "frozen_protocol_shape_and_denominator",
        protocol.get("protocol_id") == "table1_reliability_protocol_v1"
        and protocol.get("frozen_design") is True
        and protocol.get("expected_task_count") == 54
        and protocol.get("repeat_ids") == ["r0", "r1", "r2"]
        and protocol.get("expected_runs_per_method") == EXPECTED_AUTHORING_RUNS,
        protocol_sha256=protocol_sha256,
    )
    check(
        checks,
        "frozen_protocol_file_bindings_match_current_bytes",
        protocol.get("manifest", {}).get("sha256") == common_manifest_sha256
        and protocol.get("hidden_specs", {}).get("sha256") == hidden_specs_sha256
        and protocol.get("common_evaluator", {}).get("sha256") == evaluator_sha256
        and protocol.get("package_schema", {}).get("sha256") == package_schema_sha256
        and protocol.get("result_schema", {}).get("sha256") == result_schema_sha256,
        bindings=expected_bindings,
    )
    check(
        checks,
        "frozen_public_manifest_is_54_by_3",
        frozen_manifest.get("frozen") is True
        and frozen_manifest.get("task_count") == 54
        and len(frozen_manifest.get("tasks", [])) == 54
        and frozen_manifest.get("repeat_ids") == ["r0", "r1", "r2"],
    )
    implementation_paths = [
        record.get("checkout_path") for record in implementation_audit.values()
    ]
    check(
        checks,
        "method_checkout_and_adapter_bindings_are_unique_and_current",
        len(implementation_audit) == len(EXPECTED_METHODS)
        and len(set(implementation_paths)) == len(EXPECTED_METHODS)
        and all(
            all_boolean_values(record.get("checks"))
            for record in implementation_audit.values()
        ),
        implementations=implementation_audit,
    )
    common_inputs = common_manifest.get("inputs")
    common_inputs = common_inputs if isinstance(common_inputs, dict) else {}
    check(
        checks,
        "common_preflight_bound_to_final_inputs",
        common_inputs.get("protocol", {}).get("sha256") == protocol_sha256
        and common_inputs.get("manifest", {}).get("sha256") == common_manifest_sha256
        and common_inputs.get("hidden_specs", {}).get("sha256") == hidden_specs_sha256
        and common_inputs.get("common_evaluator", {}).get("sha256") == evaluator_sha256
        and common_inputs.get("package_schema", {}).get("sha256")
        == package_schema_sha256
        and common_inputs.get("result_schema", {}).get("sha256")
        == result_schema_sha256,
    )
    check(
        checks,
        "common_preflight_script_hash_current",
        common_manifest.get("preflight_script", {}).get("path")
        == "exp/scripts/preflight_table1_authoring_common.py"
        and common_manifest.get("preflight_script", {}).get("sha256")
        == input_hashes["common_preflight_script"],
    )
    common_frozen = common_summary.get("frozen_consistency")
    common_frozen = common_frozen if isinstance(common_frozen, dict) else {}
    common_checks = common_self_check.get("checks")
    common_checks = common_checks if isinstance(common_checks, list) else []
    frozen_common_checks = [
        row for row in common_checks if row.get("scope") == "frozen_consistency"
    ]
    check(
        checks,
        "common_frozen_consistency_all_reported_checks_ready",
        common_frozen_consistency_ready(common_summary, common_self_check),
        frozen_consistency=common_frozen,
        reported_frozen_checks=len(frozen_common_checks),
        minimum_frozen_checks=MINIMUM_COMMON_FROZEN_CHECKS,
    )
    common_implementations = common_summary.get("contract", {}).get(
        "method_implementations"
    )
    common_implementations = (
        common_implementations if isinstance(common_implementations, dict) else {}
    )
    check(
        checks,
        "common_preflight_implementation_records_match_current",
        all(
            common_implementations.get(method, {}).get("actual_commit")
            == implementation_audit[method]["commit"]
            and common_implementations.get(method, {}).get("actual_git_tree")
            == implementation_audit[method]["git_tree"]
            and common_implementations.get(method, {}).get("actual_adapter_sha256")
            == implementation_audit[method]["adapter_sha256"]
            and common_implementations.get(method, {}).get("actual_tracked_clean")
            is True
            for method in EXPECTED_METHODS
        ),
    )
    check(
        checks,
        "common_adapter_readiness_matches_protocol",
        common_adapter_readiness_matches_protocol(
            protocol,
            common_summary,
            common_self_check,
        ),
    )
    check(
        checks,
        "common_preflight_made_zero_calls_and_attempts",
        common_self_check.get("network_or_api_calls") == 0
        and common_self_check.get("authoring_attempts") == 0
        and common_self_check.get("input_files_modified") is False
        and common_self_check.get("hidden_spec_content_persisted") is False,
    )
    evaluator_checks = evaluator_self_check.get("checks")
    check(
        checks,
        "common_evaluator_self_test_6_of_6_passes",
        evaluator_self_check.get("status") == "PASS"
        and isinstance(evaluator_checks, dict)
        and len(evaluator_checks) == 6
        and all_boolean_values(evaluator_checks)
        and evaluator_self_check.get("positive_report_sha256")
        == input_hashes["evaluator_positive_report"]
        and evaluator_self_check.get("negative_report_sha256")
        == input_hashes["evaluator_negative_report"],
    )
    evaluator_report_bindings = {
        key: value
        for key, value in expected_bindings.items()
        if key != "result_schema_sha256"
    }
    check(
        checks,
        "common_evaluator_reports_bind_final_protocol",
        flat_bindings_match(
            evaluator_positive.get("bindings"), evaluator_report_bindings
        )
        and flat_bindings_match(
            evaluator_negative.get("bindings"), evaluator_report_bindings
        ),
        protocol_sha256=protocol_sha256,
    )
    positive_verdicts = evaluator_positive.get("verdicts")
    negative_verdicts = evaluator_negative.get("verdicts")
    check(
        checks,
        "common_evaluator_positive_and_negative_semantics",
        all_boolean_values(positive_verdicts)
        and isinstance(negative_verdicts, dict)
        and negative_verdicts.get("executable") is True
        and negative_verdicts.get("artifact_saved") is True
        and negative_verdicts.get("joint_spec_pass") is False
        and negative_verdicts.get("common_qc_pass") is False
        and evaluator_negative.get("feedback", {}).get("failure_codes")
        == ["JOINT_SPEC_FAILED"],
    )
    check(
        checks,
        "pva_authoring_prepare_is_162_zero_call_blocked",
        pva_authoring.get("method_id") == "pva"
        and pva_authoring.get("mode") == "prepare_only"
        and pva_authoring.get("status") == "PREPARED_EXECUTION_BLOCKED"
        and pva_authoring.get("selected_task_count") == 54
        and pva_authoring.get("selected_repeat_count") == 3
        and pva_authoring.get("intended_runs") == EXPECTED_AUTHORING_RUNS
        and len(pva_authoring.get("job_order", [])) == EXPECTED_AUTHORING_RUNS
        and len(set(pva_authoring.get("job_order", []))) == EXPECTED_AUTHORING_RUNS
        and pva_authoring.get("provider_calls_made") == 0
        and flat_bindings_match(pva_authoring.get("bindings"), expected_bindings),
        blocker_count=pva_blocker_count,
    )
    check(
        checks,
        "pva_authoring_capability_blockers_match_protocol",
        capability_blockers_match_protocol(protocol, "pva", pva_blockers)
        and pva_authoring.get("execution_readiness", {}).get("status")
        == "BLOCKED_ADAPTERS"
        and pva_authoring.get("execution_readiness", {})
        .get("method_adapters_ready", {})
        .get("pva")
        is False,
    )
    lam_evidence = lam_authoring_self_check.get("evidence_sha256")
    lam_evidence = lam_evidence if isinstance(lam_evidence, dict) else {}
    check(
        checks,
        "lam_authoring_self_check_17_of_17_current",
        named_boolean_self_check_ready(
            lam_authoring_self_check, minimum_checks=17
        )
        and lam_evidence.get("manifest.json") == input_hashes["lam_authoring_manifest"]
        and lam_evidence.get("summary.json") == input_hashes["lam_authoring_summary"]
        and lam_evidence.get("report.md") == input_hashes["lam_authoring_report"],
    )
    lam_bindings = lam_authoring_manifest.get("bindings")
    lam_bindings = lam_bindings if isinstance(lam_bindings, dict) else {}
    check(
        checks,
        "lam_authoring_prepare_is_162_zero_attempt_zero_call",
        lam_authoring_summary.get("method_id") == "lam"
        and lam_authoring_summary.get("status") == "PREPARED_NOT_RUN"
        and lam_authoring_summary.get("expected_jobs")
        == lam_authoring_summary.get("prepared_jobs")
        == EXPECTED_AUTHORING_RUNS
        and lam_authoring_summary.get("authoring_attempts") == 0
        and lam_authoring_summary.get("completed_results") == 0
        and lam_authoring_summary.get("metric_denominator") == 0
        and lam_authoring_summary.get("provider_calls_made") == 0
        and lam_authoring_manifest.get("authoring_attempts") == 0
        and lam_authoring_manifest.get("provider_calls_made") == 0
        and lam_authoring_manifest.get("prepared_job_count") == EXPECTED_AUTHORING_RUNS
        and nested_binding_matches(
            lam_bindings.get("protocol"),
            "exp/reference/table1_reliability_protocol_v1.json",
            protocol_sha256,
        )
        and nested_binding_matches(
            lam_bindings.get("manifest"),
            "exp/reference/table1_reliability_common_authoring_v1.json",
            common_manifest_sha256,
        ),
    )
    check(
        checks,
        "lam_authoring_capability_blockers_match_protocol_and_metrics_nr",
        capability_blockers_match_protocol(protocol, "lam", lam_blockers)
        and all(
            isinstance(item, dict)
            and item.get("state") == "N/R"
            and item.get("value") is None
            for item in lam_authoring_summary.get("metrics", {}).values()
        ),
        capability_blocker_count=lam_blocker_count,
    )
    check(
        checks,
        "articraft_authoring_prepare_is_162_zero_attempt_zero_call",
        articraft_authoring_summary.get("method_id") == "articraft"
        and articraft_authoring_summary.get("status") == "PREPARED_EXECUTION_BLOCKED"
        and articraft_authoring_summary.get("intent_denominator")
        == articraft_authoring_summary.get("prepared_jobs")
        == EXPECTED_AUTHORING_RUNS
        and articraft_authoring_summary.get("attempted_jobs") == 0
        and articraft_authoring_summary.get("completed_jobs") == 0
        and articraft_authoring_summary.get("evaluable_jobs") == 0
        and articraft_authoring_summary.get("provider_calls_made") == 0
        and articraft_authoring_manifest.get("provider_calls_made") == 0
        and articraft_authoring_manifest.get("prepared_job_count")
        == EXPECTED_AUTHORING_RUNS
        and flat_bindings_match(
            articraft_authoring_manifest.get("bindings"),
            {
                **expected_bindings,
                "adapter_sha256": implementation_audit["articraft"]["adapter_sha256"],
            },
        ),
    )
    check(
        checks,
        "articraft_authoring_self_check_and_preflight_pass",
        articraft_authoring_self_check.get("pass") is True
        and articraft_authoring_self_check.get("prepare_only_contract_pass") is True
        and articraft_authoring_self_check.get("execution_ready") is False
        and articraft_authoring_self_check.get("provider_calls_made") == 0
        and articraft_authoring_self_check.get("api_or_network_accessed") is False
        and articraft_authoring_self_check.get("generated_assets") == 0
        and all_boolean_values(articraft_authoring_self_check.get("checks"))
        and isinstance(articraft_authoring_preflight.get("frozen_checks_total"), int)
        and articraft_authoring_preflight.get("frozen_checks_total") >= 20
        and articraft_authoring_preflight.get("frozen_checks_passed")
        == articraft_authoring_preflight.get("frozen_checks_total")
        == len(articraft_authoring_preflight.get("frozen_checks", {}))
        and all_boolean_values(articraft_authoring_preflight.get("frozen_checks"))
        and all_boolean_values(articraft_authoring_preflight.get("job_plan_checks")),
    )
    check(
        checks,
        "articraft_authoring_capability_blockers_match_protocol_and_metrics_nr",
        capability_blockers_match_protocol(protocol, "articraft", articraft_blockers)
        and all(
            isinstance(item, dict)
            and item.get("state") == "not_reported"
            and item.get("value") is None
            and item.get("denominator") == 0
            for item in articraft_authoring_summary.get("metrics", {}).values()
        ),
        capability_blocker_count=articraft_blocker_count,
    )
    check(
        checks,
        "method_prepare_snapshot_timestamps_are_internally_consistent",
        lam_authoring_manifest.get("generated_at_utc")
        == lam_authoring_summary.get("generated_at_utc")
        == lam_authoring_self_check.get("generated_at_utc")
        and articraft_authoring_manifest.get("generated_at_utc")
        == articraft_authoring_preflight.get("generated_at_utc")
        == articraft_authoring_summary.get("generated_at_utc")
        == articraft_authoring_self_check.get("generated_at_utc"),
    )
    common_run_roots = repo_root / "exp/runtime/table1_reliability/common_authoring"
    check(
        checks,
        "three_method_authoring_output_roots_absent",
        all(not (common_run_roots / method).exists() for method in EXPECTED_METHODS),
    )
    check(
        checks,
        "all_table1a_metrics_are_nr",
        all(
            metric.get("display") == "N/R"
            and metric.get("state") == "NOT_REPORTED_UNDER_EXACT_PROTOCOL"
            and metric.get("value") is None
            for record in common_authoring.values()
            for metric in record["metrics"].values()
        ),
        methods=len(common_authoring),
        cells=sum(len(record["metrics"]) for record in common_authoring.values()),
    )
    check(
        checks,
        "pva_frozen_protocol_pair",
        seed_manifest.get("protocol")
        == seed_summary.get("protocol")
        == "nano3d_seed_distribution_reliability_v1",
    )
    check(
        checks,
        "pva_frozen_manifest_is_33_by_36",
        len(frozen_templates) == 33 and seed_manifest.get("seeds") == expected_seeds,
        templates=len(frozen_templates),
        seeds=len(seed_manifest.get("seeds", [])),
    )
    check(
        checks,
        "pva_frozen_seed_denominator_is_1188",
        seed_summary.get("seed_count") == 1188 == 33 * 36,
        seed_count=seed_summary.get("seed_count"),
    )
    check(
        checks,
        "pva_frozen_seed_counts_conserved",
        seed_summary.get("compile_pass")
        == seed_summary.get("qc_pass")
        == seed_summary.get("artifact_saved")
        == 1188,
        compile_pass=seed_summary.get("compile_pass"),
        qc_pass=seed_summary.get("qc_pass"),
        artifact_saved=seed_summary.get("artifact_saved"),
    )
    check(
        checks,
        "pva_frozen_all_templates_are_36_of_36",
        seed_summary.get("templates_36_of_36_compile")
        == seed_summary.get("templates_36_of_36_qc")
        == 33
        and len(seed_summary.get("template_records", [])) == 33
        and all(
            item.get("seeds")
            == item.get("compile_pass")
            == item.get("qc_pass")
            == item.get("artifact_saved")
            == 36
            for item in seed_summary.get("template_records", [])
        ),
    )
    check(
        checks,
        "pva_corner_cohort_matches_frozen_seed_cohort",
        set(corner_templates) == set(frozen_templates)
        and all(
            corner_templates[slug].get("template_sha256")
            == frozen_templates[slug].get("template_sha256")
            for slug in frozen_templates
        ),
    )
    check(
        checks,
        "pva_corner_counts_conserved",
        corner_summary.get("corner_case_count")
        == corner_summary.get("corner_cases_passed")
        == 231
        and corner_summary.get("corner_cases_failed") == 0
        and corner_summary.get("strict_all_corner_templates_passed") == 33,
    )
    check(
        checks,
        "pva_current_manifest_is_33_by_36",
        len(current_templates) == 33
        and current_manifest.get("seeds") == expected_seeds,
        intended_cases=intended_current_cases,
    )
    check(
        checks,
        "pva_current_rerun_is_partial_and_not_promoted",
        pva_current["cohort_status"] == "BLOCKED_INFRASTRUCTURE_PARTIAL_RERUN"
        and "PID/thread pressure" in pva_current["infrastructure_blocker"]
        and current_summary.get("template_count") == 1
        and current_summary.get("seed_count") == 36
        and current_summary.get("seed_count") < intended_current_cases
        and pva_current["table1_metrics"]["seed_compile"]["display"] == "N/R",
        intended_cases=intended_current_cases,
        observed_cases=current_summary.get("seed_count"),
    )
    check(
        checks,
        "pva_current_partial_counts_conserved",
        current_summary.get("compile_pass")
        == current_summary.get("artifact_saved")
        == 36
        and current_summary.get("qc_pass") == 35
        and current_summary.get("failure_types", {}).get("subprocess_crash") == 1,
    )
    check(
        checks,
        "pva_all_frozen_source_hashes_checked",
        len(source_drift) == 33
        and all(item["current_file_exists"] for item in source_drift),
        checked=len(source_drift),
    )
    check(
        checks,
        "pva_all_frozen_sources_drifted_from_current_head",
        len(drifted) == 33,
        drifted=len(drifted),
        matches_frozen=33 - len(drifted),
    )
    check(
        checks,
        "pva_current_manifest_hashes_match_current_files",
        all(item["current_manifest_matches_file"] for item in source_drift),
    )
    check(
        checks,
        "lam_release_upstream_self_check_passes",
        all_lam_checks_pass(inputs["lam_release_self_check"]),
    )
    lam_release_metrics = lam_release_summary.get("common_authoring_rerun", {}).get(
        "metrics", {}
    )
    check(
        checks,
        "lam_release_audit_authoring_claim_not_promoted",
        lam_release_summary.get("status") == "BLOCKED"
        and lam_release_summary.get("common_authoring_rerun", {}).get("attempted_runs")
        == 0
        and bool(lam_release_metrics)
        and all(
            item.get("state") == "N/R" and item.get("value") is None
            for item in lam_release_metrics.values()
        ),
    )
    check(
        checks,
        "articraft_release_upstream_self_check_passes",
        all_articraft_checks_pass(inputs["articraft_release_self_check"]),
    )
    check(
        checks,
        "articraft_release_audit_authoring_claim_not_promoted",
        articraft_release_summary.get("status") == "ADAPTER_REQUIRED"
        and articraft_release_summary.get("local_rerun", {}).get(
            "actual_generation_runs"
        )
        == 0
        and articraft_release_summary.get("local_rerun", {}).get("metric_denominator")
        == 0
        and all(
            value is None
            for value in articraft_release_summary.get("local_rerun", {})
            .get("metrics", {})
            .values()
        ),
    )
    check(
        checks,
        "release_telemetry_not_mapped_to_authoring_metrics",
        supplementary_release["LAM"]["included_in_common_authoring_metrics"] is False
        and supplementary_release["Articraft"]["included_in_common_authoring_metrics"]
        is False
        and all(
            metric["display"] == "N/R"
            for method in ("LAM", "Articraft")
            for metric in common_authoring[method]["metrics"].values()
        ),
    )
    check(
        checks,
        "infinite_upstream_self_check_passes",
        all_infinite_checks_pass(inputs["infinite_self_check"]),
    )
    check(
        checks,
        "infinite_public_180s_denominators_exact",
        public["requested_cases"] == 720
        and public["seed_compile_180s"]["numerator"] == 702
        and public["seed_compile_180s"]["denominator"] == 720
        and public["compile_only_all_36_180s"]["numerator"] == 16
        and public["compile_only_all_36_180s"]["denominator"] == 20,
    )
    check(
        checks,
        "infinite_matched_180s_denominators_exact",
        matched["requested_cases"] == 180
        and matched["seed_compile_180s"]["numerator"] == 170
        and matched["seed_compile_180s"]["denominator"] == 180
        and matched["compile_only_all_36_180s"]["numerator"] == 4
        and matched["compile_only_all_36_180s"]["denominator"] == 5,
    )
    check(
        checks,
        "infinite_full_qc_and_corner_fail_closed",
        infinite_public["seed_full_qc"]["display"] == "N/E"
        and infinite_public["full_qc_all_36_factories"]["display"] == "N/E"
        and infinite_public["corner_cases"]["display"] == "N/R",
    )
    check(
        checks,
        "missing_state_vocabulary_preserved",
        pva_current["table1_metrics"]["seed_compile"]["display"] == "N/R"
        and infinite_public["seed_full_qc"]["display"] == "N/E"
        and pva_frozen["regression_retention"]["display"] == "N/A",
    )

    path_hashes_stable = all(
        sha256_if_regular(path) == input_hashes[name] for name, path in paths.items()
    )
    template_hashes_stable = all(
        sha256_if_regular(Path(item["template_path"])) == item["current_sha256"]
        for item in source_drift
    )
    final_implementation_audit, final_implementation_hashes = audit_implementations(
        repo_root, protocol
    )
    implementation_stable = (
        final_implementation_audit == implementation_audit
        and final_implementation_hashes == implementation_hashes
    )
    check(
        checks,
        "all_inputs_stable_during_aggregation",
        path_hashes_stable and template_hashes_stable and implementation_stable,
        path_files=len(paths),
        template_files=len(source_drift),
        implementation_files=len(implementation_hashes),
    )

    all_passed = all(item["pass"] for item in checks)
    generated_at = datetime.now(timezone.utc).isoformat()
    summary = {
        "claim_boundary": (
            "Aggregation only. No missing cell is filled from paper values, release telemetry, "
            "a nearby proxy, or a partial rerun."
        ),
        "common_authoring_table1a": common_authoring,
        "common_authoring_contract": {
            "adapter_readiness": (
                "READY"
                if common_summary.get("execution_adapter_readiness", {}).get("ready")
                is True
                else "NOT_READY"
            ),
            "all_table1a_cells": "N/R",
            "common_frozen_consistency": {
                "legacy_base_checks": 76,
                "final_checks_passed": common_frozen.get("passed_checks"),
                "final_checks_total": common_frozen.get("total_checks"),
                "status": common_frozen.get("status"),
            },
            "evaluator_self_test": {
                "passed": sum(evaluator_checks.values()),
                "status": evaluator_self_check.get("status"),
                "total": len(evaluator_checks),
            },
            "frozen_manifest_sha256": common_manifest_sha256,
            "implementations": implementation_audit,
            "protocol_sha256": protocol_sha256,
            "provider_calls": 0,
            "authoring_attempts": 0,
        },
        "generated_at_utc": generated_at,
        "infinite_mobility_matched_supplementary": infinite_matched,
        "infinite_mobility_public_factory_panel": infinite_public,
        "protocol_id": PROTOCOL_ID,
        "pva_current_head_partial": pva_current,
        "pva_frozen_snapshot": pva_frozen,
        "schema_version": SCHEMA_VERSION,
        "status": (
            "COMPLETE_WITH_BLOCKED_AND_NON_EVALUABLE_CELLS"
            if all_passed
            else "FAILED_SELF_CHECK"
        ),
        "supplementary_release_telemetry": supplementary_release,
    }

    report_lines = [
        "# Table 1 reliability evidence aggregation",
        "",
        f"Generated: `{generated_at}`",
        "",
        "This is an evidence aggregation only. It performed no generation, network, or API work.",
        "",
        "## Common Authoring Contract",
        "",
        f"- Frozen protocol: `{protocol_sha256}`.",
        f"- Common frozen consistency: {common_frozen.get('passed_checks')}/{common_frozen.get('total_checks')} reported checks; all must pass and the total must be at least {MINIMUM_COMMON_FROZEN_CHECKS}.",
        f"- Common evaluator self-test: {sum(evaluator_checks.values())}/{len(evaluator_checks)} PASS; both reports bind the final protocol SHA.",
        f"- PV-A: `{pva_authoring.get('status')}`, {pva_authoring.get('intended_runs')} prepared, 0 attempts, {pva_authoring.get('provider_calls_made')} provider calls, {pva_blocker_count} capability blockers matching protocol.",
        f"- LAM: `{lam_authoring_summary.get('status')}`, {lam_authoring_summary.get('prepared_jobs')} prepared, {lam_authoring_summary.get('authoring_attempts')} attempts, {lam_authoring_summary.get('provider_calls_made')} provider calls, {lam_blocker_count} capability blockers matching protocol.",
        f"- Articraft: `{articraft_authoring_summary.get('status')}`, {articraft_authoring_summary.get('prepared_jobs')} prepared, {articraft_authoring_summary.get('attempted_jobs')} attempts, {articraft_authoring_summary.get('provider_calls_made')} provider calls, {articraft_blocker_count} capability blockers matching protocol; implementation `articraft_data@{implementation_audit['articraft']['commit']}`.",
        "- All 24 common-authoring metric cells remain N/R. Prepare evidence is not promoted to a measured result.",
        "",
        "## Main result boundaries",
        "",
        f"- PV-A frozen snapshot (2026-08-05): seed compile {pct(pva_frozen['seed_compile'])}; Full-QC {pct(pva_frozen['seed_full_qc'])}; 36/36 Full-QC templates {pct(pva_frozen['full_qc_all_36_templates'])}; project-native corners {pct(pva_frozen['corner_cases'])}.",
        f"- PV-A source drift: {len(drifted)}/{len(source_drift)} frozen template hashes differ from current files. The frozen scores are historical, not current-HEAD scores.",
        f"- PV-A current HEAD: BLOCKED_INFRASTRUCTURE_PARTIAL_RERUN. {current_summary['seed_count']}/{intended_current_cases} intended cases observed across {current_summary['template_count']}/{len(current_templates)} templates. Host PID/thread pressure stopped the full cohort. Diagnostics are compile {current_summary['compile_pass']}/{current_summary['seed_count']} and strict Full-QC {current_summary['qc_pass']}/{current_summary['seed_count']}; aggregate current-HEAD cells remain N/R.",
        f"- Infinite Mobility public panel at the 180 s sensitivity: seed compile {pct(infinite_public['seed_compile_180s'])}, all-36 compile factories {pct(infinite_public['compile_all_36_factories_180s'])}; Full-QC N/E and corners N/R.",
        f"- Infinite Mobility matched five-category subset: seed compile {pct(infinite_matched['seed_compile_180s'])}, all-36 compile factories {pct(infinite_matched['compile_all_36_factories_180s'])}; Full-QC N/E.",
        "",
        "## Non-comparable supplementary evidence",
        "",
        f"- Infinite Mobility 300 s structural-package QC: {pct(infinite_public['structural_package_qc_300s_supplementary'])}; this is not Full-QC.",
        f"- LAM release audit: {supplementary_release['LAM']['release_records']} records; release fields/status are not mapped to Table 1 authoring success.",
        f"- Articraft release audit: {supplementary_release['Articraft']['release_index_rows']} index rows; release status/rating is not mapped to Table 1 authoring success.",
        "",
        "## Self-check",
        "",
        f"- {'PASS' if all_passed else 'FAIL'}: {sum(item['pass'] for item in checks)}/{len(checks)} checks passed.",
        "",
        "See `summary.json` for machine-readable cells and `self_check.json` for all invariants.",
    ]

    write_json(output_dir / "summary.json", summary)
    (output_dir / "report.md").write_text(
        "\n".join(report_lines) + "\n", encoding="utf-8"
    )
    self_check = {
        "all_passed": all_passed,
        "checks": checks,
        "generated_at_utc": generated_at,
        "input_sha256": input_hashes,
        "passed": sum(item["pass"] for item in checks),
        "protocol_id": f"{PROTOCOL_ID}_self_check",
        "schema_version": SCHEMA_VERSION,
        "total": len(checks),
    }
    write_json(output_dir / "self_check.json", self_check)

    print(
        json.dumps(
            {
                "output_dir": str(output_dir),
                "self_check": f"{self_check['passed']}/{self_check['total']}",
                "status": summary["status"],
            },
            sort_keys=True,
        )
    )
    return 0 if all_passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
