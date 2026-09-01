#!/usr/bin/env python3
"""Run Table 4 on the frozen Ours per-class N=5 supplementary cohort."""

from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
DEFAULT_COHORT_MANIFEST = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
BASE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_ours_500k.py"

DATASET_LABEL = "Ours per-class N=5 (supplementary)"
PROTOCOL_ID = "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1"
SAMPLE_SIZE = 2_655
EXPECTED_N_RELEASE = 302_440
EXPECTED_CATEGORY_COUNT = 531
EXPECTED_PER_CLASS = 5
EXPECTED_J_EVAL = 14_968
EXPECTED_STATE_COUNT = 486_903
EXPECTED_COHORT_FILE_SHA256 = (
    "e78f4b767023f8a5c1517d96bfab35a39482d6eee28238820a9b91ac3ea8d293"
)
EXPECTED_COHORT_CONTENT_SHA256 = (
    "eea55287dd70b710a7c03b11b16c6685208bbaa63cde925232293cb9012c8158"
)


def _load_base():
    spec = importlib.util.spec_from_file_location("urdf_table4_ours_pva_base", BASE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import shared Table 4 runner: {BASE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


base = _load_base()
_base_audit_asset = base.audit_asset
_base_atomic_json = base.atomic_json
_base_build_frozen_items = base.build_frozen_items
_base_evaluate_asset = base.evaluate_asset
_base_report_text = base.report_text
_base_run = base.run
_base_run_one_subprocess = base.run_one_subprocess
_base_verify_result_against_item = base.verify_result_against_item

FORMAL_WORKERS = 4
FORMAL_AUDIT_WORKERS = 4
FORMAL_CHILD_TIMEOUT_SECONDS = 900.0

_active_execution_controls: dict[str, int | float] | None = None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _artifact_self_hash(artifact: dict[str, Any]) -> str:
    payload = dict(artifact)
    payload.pop("artifact_manifest_content_sha256", None)
    return canonical_sha256(payload)


def _validate_package_binding(row: dict[str, Any]) -> dict[str, Any]:
    binding = row.get("package_binding")
    if not isinstance(binding, dict) or not isinstance(binding.get("files"), list):
        raise ValueError(f"missing package binding: {row.get('dataset_id')}")
    files = binding["files"]
    if binding.get("content_manifest_sha256") != canonical_sha256(files):
        raise ValueError(f"package binding self-hash mismatch: {row.get('dataset_id')}")
    if binding.get("file_count") != len(files):
        raise ValueError(f"package binding file count mismatch: {row.get('dataset_id')}")
    if binding.get("total_bytes") != sum(int(item["bytes"]) for item in files):
        raise ValueError(f"package binding byte count mismatch: {row.get('dataset_id')}")
    urdf_rows = [item for item in files if item.get("path") == "model.urdf"]
    if len(urdf_rows) != 1 or urdf_rows[0].get("sha256") != row.get("urdf_sha256"):
        raise ValueError(f"package binding URDF mismatch: {row.get('dataset_id')}")
    return binding


def load_cohort(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    file_sha = sha256_file(path)
    if file_sha != EXPECTED_COHORT_FILE_SHA256:
        raise ValueError("Ours per-class N=5 cohort manifest file hash mismatch")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise ValueError("Ours per-class N=5 cohort manifest self-hash mismatch")
    if manifest.get("manifest_content_sha256") != EXPECTED_COHORT_CONTENT_SHA256:
        raise ValueError("Ours per-class N=5 cohort manifest content hash mismatch")
    if (
        manifest.get("schema_version") != "pva-per-class-extracted-cohort/v2"
        or manifest.get("protocol_id")
        != "pva-per-class-n5-fence-ferris-max-movable-joints-v1"
        or manifest.get("dataset") != "PV-A-per-class-n5"
        or manifest.get("classification") != "FROZEN_MIXED_STRATIFIED_SAMPLE"
        or manifest.get("n_eval") != SAMPLE_SIZE
        or manifest.get("class_count") != EXPECTED_CATEGORY_COUNT
        or manifest.get("per_class") != EXPECTED_PER_CLASS
    ):
        raise ValueError("Ours per-class N=5 cohort protocol metadata mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != SAMPLE_SIZE:
        raise ValueError("Ours per-class N=5 cohort asset count mismatch")

    rows: list[dict[str, Any]] = []
    seen: set[str] = set()
    categories: Counter[str] = Counter()
    for index, raw in enumerate(assets):
        if not isinstance(raw, dict) or raw.get("selection_index") != index:
            raise ValueError(f"cohort selection order mismatch at index {index}")
        dataset_id = str(raw.get("dataset_id", ""))
        category = str(raw.get("category", ""))
        if not dataset_id or not category or dataset_id in seen:
            raise ValueError(f"invalid cohort identity at index {index}")
        seen.add(dataset_id)
        categories[category] += 1
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        if not package.is_dir() or not Path(str(raw.get("package", ""))).is_absolute():
            raise ValueError(f"package path is not an absolute directory: {dataset_id}")
        if raw.get("primary_urdf_relative_path") != "model.urdf":
            raise ValueError(f"unexpected primary URDF path: {dataset_id}")
        urdf = package / "model.urdf"
        if urdf.is_symlink() or not urdf.is_file():
            raise ValueError(f"canonical model.urdf missing or symlinked: {dataset_id}")
        binding = _validate_package_binding(raw)
        rows.append(
            {
                "asset_id": dataset_id,
                "dataset_id": dataset_id,
                "raw_category": category,
                "category": category,
                "seed_name": str(raw["asset_id"]),
                "asset_root": package.name,
                "selection_index": index,
                "primary_urdf_sha256": str(raw["urdf_sha256"]),
                "package": str(package),
                "package_binding_content_manifest_sha256": str(
                    binding["content_manifest_sha256"]
                ),
                "package_binding_file_count": int(binding["file_count"]),
                "package_binding_total_bytes": int(binding["total_bytes"]),
            }
        )
    if len(categories) != EXPECTED_CATEGORY_COUNT or set(categories.values()) != {
        EXPECTED_PER_CLASS
    }:
        raise ValueError("cohort category/per-class coverage mismatch")
    return {
        "file_sha256": file_sha,
        "content_sha256": manifest["manifest_content_sha256"],
        "n_release": EXPECTED_N_RELEASE,
        "release_category_count": EXPECTED_CATEGORY_COUNT,
        "per_class": EXPECTED_PER_CLASS,
        "source": {
            "cohort_type": "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES"
        },
        "rows": rows,
    }


def audit_asset(_dataset_root: Path, row: dict[str, Any]) -> dict[str, Any]:
    package = Path(row["package"]).resolve(strict=True)
    return _base_audit_asset(package.parent, row)


def current_runtime_identity() -> dict[str, Any]:
    return base.current_runtime_identity()


def _input_identity_payload(item: dict[str, Any]) -> dict[str, Any]:
    return {
        "base_fields": {key: item[key] for key in base.FROZEN_INPUT_FIELDS},
        "runtime_binding": item["runtime_binding"],
        "package": item["package"],
        "package_binding_content_manifest_sha256": item[
            "package_binding_content_manifest_sha256"
        ],
        "package_binding_file_count": item["package_binding_file_count"],
        "package_binding_total_bytes": item["package_binding_total_bytes"],
    }


def build_frozen_items(
    cohort_rows: list[dict[str, Any]],
    audits: dict[str, dict[str, Any]],
    runtime_identity: dict[str, Any],
) -> list[dict[str, Any]]:
    items = _base_build_frozen_items(cohort_rows, audits, runtime_identity)
    for item, row in zip(items, cohort_rows):
        item.update(
            {
                "package": row["package"],
                "package_binding_content_manifest_sha256": row[
                    "package_binding_content_manifest_sha256"
                ],
                "package_binding_file_count": row["package_binding_file_count"],
                "package_binding_total_bytes": row["package_binding_total_bytes"],
            }
        )
        item["input_identity_sha256"] = canonical_sha256(
            _input_identity_payload(item)
        )
    if len(items) == SAMPLE_SIZE:
        validate_formal_items(items)
    return items


def validate_formal_items(items: list[dict[str, Any]]) -> dict[str, int]:
    n_eval = len(items)
    category_counts = Counter(str(item["category"]) for item in items)
    category_count = len(category_counts)
    j_eval = sum(int(item["movable_dof_count"]) for item in items)
    expected_states = sum(
        int(item["rest_state_expected"])
        + int(item["single_state_expected"])
        + int(item["sobol_state_expected"])
        for item in items
    )
    if n_eval != SAMPLE_SIZE:
        raise ValueError(f"formal N_eval mismatch: {n_eval} != {SAMPLE_SIZE}")
    if category_count != EXPECTED_CATEGORY_COUNT or set(category_counts.values()) != {
        EXPECTED_PER_CLASS
    }:
        raise ValueError("formal category/per-class denominator mismatch")
    if j_eval != EXPECTED_J_EVAL:
        raise ValueError(f"formal J_eval mismatch: {j_eval} != {EXPECTED_J_EVAL}")
    if any(int(item["movable_dof_count"]) <= 0 for item in items):
        raise ValueError("formal cohort contains a zero-DoF asset")
    if expected_states != EXPECTED_STATE_COUNT:
        raise ValueError(
            f"formal state denominator mismatch: {expected_states} != {EXPECTED_STATE_COUNT}"
        )
    return {
        "n_eval": n_eval,
        "category_count": category_count,
        "j_eval": j_eval,
        "expected_states": expected_states,
    }


def _require_item_runtime(
    item: dict[str, Any], observed: dict[str, Any], *, context: str
) -> None:
    expected = item.get("runtime_binding")
    if not isinstance(expected, dict) or expected != observed:
        raise RuntimeError(f"{context} runtime binding mismatch")
    if expected.get("adapter_runner_sha256") != sha256_file(SCRIPT):
        raise RuntimeError(f"{context} adapter identity mismatch")
    if expected.get("collision_core_sha256") != sha256_file(base.CORE_SCRIPT):
        raise RuntimeError(f"{context} collision core identity mismatch")
    if expected.get("pybullet_module_sha256") != expected.get(
        "core_runtime", {}
    ).get("pybullet_module_sha256"):
        raise RuntimeError(f"{context} PyBullet identity mismatch")


def verify_result_against_item(
    item: dict[str, Any], result: dict[str, Any]
) -> None:
    _base_verify_result_against_item(item, result)
    frozen_fields = (
        "protocol_id",
        "order",
        "dataset_id",
        "category",
        "input_identity_sha256",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "object_bbox_diagonal_m",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
    )
    for key in frozen_fields:
        if result.get(key) != item.get(key):
            raise ValueError(f"result frozen field mismatch: {key}")
    state_frozen_fields = (
        "protocol_id",
        "order",
        "dataset_id",
        *base.IDENTITY_FIELDS,
        "category",
        "input_identity_sha256",
    )
    for state in result.get("state_records") or []:
        for key in state_frozen_fields:
            if state.get(key) != item.get(key):
                raise ValueError(f"state frozen field mismatch: {key}")
    expected = item.get("runtime_binding")
    if not isinstance(expected, dict):
        raise ValueError("result runtime binding is missing from frozen item")
    if result.get("runtime_identity") != expected:
        raise ValueError("result runtime binding mismatch")
    if result.get("runner_sha256") != expected.get("adapter_runner_sha256"):
        raise ValueError("result adapter identity mismatch")
    if result.get("collision_core_sha256") != expected.get(
        "collision_core_sha256"
    ):
        raise ValueError("result collision core identity mismatch")
    if expected.get("pybullet_module_sha256") != expected.get(
        "core_runtime", {}
    ).get("pybullet_module_sha256"):
        raise ValueError("result PyBullet identity mismatch")


def evaluate_asset(item: dict[str, Any], _dataset_root: Path) -> dict[str, Any]:
    _require_item_runtime(item, current_runtime_identity(), context="child")
    package = Path(item["package"]).resolve(strict=True)
    result = _base_evaluate_asset(item, package.parent)
    verify_result_against_item(item, result)
    return result


def run_one_subprocess(
    item: dict[str, Any], scratch: Path, timeout_seconds: float
) -> dict[str, Any]:
    _require_item_runtime(item, current_runtime_identity(), context="parent")
    result = _base_run_one_subprocess(item, scratch, timeout_seconds)
    verify_result_against_item(item, result)
    return result


def _load_resume_manifest(output: Path) -> dict[str, Any]:
    manifest_path = output.resolve(strict=True) / "frozen_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise RuntimeError("resume manifest self-hash mismatch")
    if (
        manifest.get("schema_version")
        != "table4_ours_pva_per_class_n5_frozen_manifest_v1"
        or manifest.get("dataset") != DATASET_LABEL
        or manifest.get("protocol_id") != PROTOCOL_ID
    ):
        raise RuntimeError("resume manifest protocol identity mismatch")
    return manifest


def _validate_resume_controls(args: Any, manifest: dict[str, Any]) -> None:
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise RuntimeError("resume manifest evaluation is missing")
    controls = {
        "workers": args.workers,
        "audit_workers": args.audit_workers,
        "child_timeout_seconds": args.child_timeout_seconds,
    }
    for key, observed in controls.items():
        if evaluation.get(key) != observed:
            option = key.replace("_", "-")
            raise ValueError(
                f"resume --{option} must equal frozen evaluation value "
                f"{evaluation.get(key)!r}"
            )
    items = manifest.get("items")
    if not isinstance(items, list) or not items:
        raise RuntimeError("resume manifest has no frozen items")
    classification = manifest.get("classification")
    expected_limit = None if classification == "FORMAL" else len(items)
    if classification not in {"FORMAL", "SMOKE"} or args.limit != expected_limit:
        raise ValueError(
            f"resume --limit must equal frozen selection value {expected_limit!r}"
        )


def validate_args(args: Any) -> None:
    if args.limit is not None and not 1 <= args.limit <= SAMPLE_SIZE:
        raise ValueError(f"--limit must be in [1, {SAMPLE_SIZE}]")
    if args.workers <= 0:
        raise ValueError("--workers must be positive")
    if args.audit_workers <= 0:
        raise ValueError("--audit-workers must be positive")
    if args.child_timeout_seconds <= 0:
        raise ValueError("--child-timeout-seconds must be positive")
    if args.resume and args.output is None:
        raise ValueError("--resume requires an explicit --output")
    if args.limit is None:
        formal_controls = {
            "workers": (args.workers, FORMAL_WORKERS),
            "audit-workers": (args.audit_workers, FORMAL_AUDIT_WORKERS),
            "child-timeout-seconds": (
                args.child_timeout_seconds,
                FORMAL_CHILD_TIMEOUT_SECONDS,
            ),
        }
        for name, (observed, expected) in formal_controls.items():
            if observed != expected:
                raise ValueError(f"formal mode requires --{name}={expected:g}")
    if args.resume:
        _validate_resume_controls(args, _load_resume_manifest(args.output))


def _validate_resume_provenance(args: Any) -> dict[str, Any]:
    manifest = _load_resume_manifest(args.output)
    _validate_resume_controls(args, manifest)
    evaluation = manifest["evaluation"]
    observed_runtime = current_runtime_identity()
    if evaluation.get("runtime_identity") != observed_runtime:
        raise RuntimeError("resume runtime binding differs from frozen evaluation")
    if evaluation.get("adapter_path") != str(SCRIPT) or evaluation.get(
        "adapter_sha256"
    ) != observed_runtime.get("adapter_runner_sha256"):
        raise RuntimeError("resume adapter identity differs from frozen evaluation")
    if evaluation.get("core_path") != str(base.CORE_SCRIPT) or evaluation.get(
        "core_sha256"
    ) != observed_runtime.get("collision_core_sha256"):
        raise RuntimeError("resume collision core identity differs from frozen evaluation")
    if evaluation.get("child_python") != observed_runtime.get("python_executable"):
        raise RuntimeError("resume child Python differs from frozen evaluation")

    item_by_id: dict[str, dict[str, Any]] = {}
    for item in manifest["items"]:
        asset_id = str(item.get("asset_id", ""))
        if not asset_id or asset_id in item_by_id:
            raise RuntimeError(f"invalid frozen resume item identity: {asset_id!r}")
        _require_item_runtime(item, observed_runtime, context=f"resume item {asset_id}")
        if item.get("input_identity_sha256") != canonical_sha256(
            _input_identity_payload(item)
        ):
            raise RuntimeError(f"resume item input identity mismatch: {asset_id}")
        item_by_id[asset_id] = item

    records_path = args.output / "asset_records.jsonl"
    if records_path.is_file():
        seen: set[str] = set()
        for line_number, line in enumerate(
            records_path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f"invalid resume record JSON at line {line_number}: {exc}"
                ) from exc
            asset_id = str(record.get("dataset_id", ""))
            if asset_id not in item_by_id or asset_id in seen:
                raise RuntimeError(f"invalid or duplicate resume record: {asset_id!r}")
            verify_result_against_item(item_by_id[asset_id], record)
            seen.add(asset_id)
    return manifest


def _atomic_json_with_controls(path: Path, value: Any) -> None:
    if (
        _active_execution_controls is not None
        and path.name == "frozen_manifest.json"
        and isinstance(value, dict)
        and value.get("protocol_id") == PROTOCOL_ID
    ):
        evaluation = value.get("evaluation")
        if isinstance(evaluation, dict):
            value["schema_version"] = (
                "table4_ours_pva_per_class_n5_frozen_manifest_v1"
            )
            evaluation.update(_active_execution_controls)
            value["manifest_content_sha256"] = _manifest_self_hash(value)
    _base_atomic_json(path, value)


def _finalize_metadata(output: Path, cohort_path: Path) -> None:
    manifest_path = output / "frozen_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    items = manifest["items"]
    j_eval = sum(int(item["movable_dof_count"]) for item in items)
    expected_states = sum(
        int(item["rest_state_expected"])
        + int(item["single_state_expected"])
        + int(item["sobol_state_expected"])
        for item in items
    )
    manifest["schema_version"] = "table4_ours_pva_per_class_n5_frozen_manifest_v1"
    manifest["dataset"] = DATASET_LABEL
    manifest["cohort_label"] = DATASET_LABEL
    manifest["protocol_id"] = PROTOCOL_ID
    manifest["source"] = {
        "cohort_manifest_path": str(cohort_path.resolve(strict=True)),
        "cohort_manifest_file_sha256": EXPECTED_COHORT_FILE_SHA256,
        "cohort_manifest_content_sha256": EXPECTED_COHORT_CONTENT_SHA256,
        "cohort_type": "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES",
        "n_release": EXPECTED_N_RELEASE,
        "n_eval": len(items),
        "release_category_count": EXPECTED_CATEGORY_COUNT,
        "eval_category_count": len({item["category"] for item in items}),
        "per_class": EXPECTED_PER_CLASS,
        "per_item_package_paths": True,
        "j_eval": j_eval,
        "expected_state_count": expected_states,
    }
    manifest["selection"].update(
        {
            "algorithm": (
                "exact frozen Ours per-class N=5 cohort .assets[] order; "
                "optional smoke prefix only"
            ),
            "ordered_input_identities_sha256": canonical_sha256(
                [item["input_identity_sha256"] for item in items]
            ),
            "outcome_based_reselection": False,
        }
    )
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    base.atomic_json(manifest_path, manifest)

    for name in ("summary.json", "checkpoint.json"):
        path = output / name
        value = json.loads(path.read_text(encoding="utf-8"))
        value["manifest_content_sha256"] = manifest["manifest_content_sha256"]
        base.atomic_json(path, value)


RECEIPT_FILES = (
    "frozen_manifest.json",
    "asset_records.jsonl",
    "state_records.jsonl",
    "summary.json",
    "report.md",
    "verification.json",
    "checkpoint.json",
    "protocol_document_at_freeze.md",
    "timing.json",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_receipts(
    output: Path,
    *,
    started_at_utc: str,
    wall_time_seconds: float,
    resume: bool,
    workers: int = FORMAL_WORKERS,
    audit_workers: int = FORMAL_AUDIT_WORKERS,
    child_timeout_seconds: float = FORMAL_CHILD_TIMEOUT_SECONDS,
) -> None:
    manifest = json.loads((output / "frozen_manifest.json").read_text(encoding="utf-8"))
    verification = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    checkpoint = json.loads((output / "checkpoint.json").read_text(encoding="utf-8"))
    items = manifest["items"]
    expected_states = sum(
        int(item["rest_state_expected"])
        + int(item["single_state_expected"])
        + int(item["sobol_state_expected"])
        for item in items
    )
    completed_at_utc = _utc_now()
    invocation = {
        "started_at_utc": started_at_utc,
        "completed_at_utc": completed_at_utc,
        "wall_time_seconds": round(float(wall_time_seconds), 6),
        "resume": bool(resume),
        "workers": int(workers),
        "audit_workers": int(audit_workers),
        "child_timeout_seconds": float(child_timeout_seconds),
    }
    timing = {
        "schema_version": "table4_wall_timing_v1",
        "dataset": DATASET_LABEL,
        "protocol_id": PROTOCOL_ID,
        **invocation,
        "measurement_endpoint": "after_base_verification_before_artifact_manifest",
        "n_eval": len(items),
        "j_eval": sum(int(item["movable_dof_count"]) for item in items),
        "expected_states": expected_states,
        "executed_states": int(verification["executed_states"]),
        "workers": int(workers),
        "audit_workers": int(audit_workers),
        "child_timeout_seconds": float(child_timeout_seconds),
        "resume_history": [],
        "resume_invocation_count": 0,
        "cumulative_wall_time_seconds": invocation["wall_time_seconds"],
    }
    timing_path = output / "timing.json"
    if resume and timing_path.is_file():
        initial = json.loads(timing_path.read_text(encoding="utf-8"))
        binding_fields = (
            "schema_version",
            "dataset",
            "protocol_id",
            "n_eval",
            "j_eval",
            "expected_states",
            "executed_states",
            "workers",
            "audit_workers",
            "child_timeout_seconds",
        )
        if any(initial.get(key) != timing.get(key) for key in binding_fields):
            raise RuntimeError("resume timing binding differs from completed run")
        history = list(initial.get("resume_history", []))
        history.append(invocation)
        timing = dict(initial)
        timing["resume_history"] = history
        timing["resume_invocation_count"] = len(history)
        timing["cumulative_wall_time_seconds"] = round(
            float(
                initial.get(
                    "cumulative_wall_time_seconds", initial["wall_time_seconds"]
                )
            )
            + float(invocation["wall_time_seconds"]),
            6,
        )
        timing["last_resume_completed_at_utc"] = completed_at_utc
    base.atomic_json(timing_path, timing)

    closure = {
        "manifest_self_hash_valid": manifest.get("manifest_content_sha256")
        == _manifest_self_hash(manifest),
        "base_verification_passed": verification.get("status") == "PASS",
        "checkpoint_complete": checkpoint.get("state") == "complete",
        "expected_states_match_verification": int(verification.get("expected_states", -1))
        == expected_states,
        "executed_states_within_denominator": 0
        <= int(verification.get("executed_states", -1))
        <= expected_states,
    }
    if not all(closure.values()):
        raise RuntimeError(f"cannot close invalid Table 4 output: {closure}")
    artifact = {
        "schema_version": "table4_artifact_manifest_v1",
        "dataset": DATASET_LABEL,
        "protocol_id": PROTOCOL_ID,
        "created_at_utc": _utc_now(),
        "run_manifest_content_sha256": manifest["manifest_content_sha256"],
        "n_eval": len(items),
        "j_eval": timing["j_eval"],
        "expected_states": expected_states,
        "executed_states": timing["executed_states"],
        "closure_checks": closure,
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": sha256_file(output / name),
            }
            for name in RECEIPT_FILES
        },
    }
    artifact["artifact_manifest_content_sha256"] = _artifact_self_hash(artifact)
    base.atomic_json(output / "artifact_manifest.json", artifact)


def verify_artifact_receipt(output: Path) -> dict[str, Any]:
    checks = {
        "artifact_self_hash_valid": False,
        "artifact_protocol_matches": False,
        "manifest_self_hash_valid": False,
        "manifest_binding_matches": False,
        "artifact_hashes_match": False,
        "base_verification_passed": False,
        "timing_denominators_match": False,
    }
    issues: list[str] = []
    try:
        artifact = json.loads(
            (output / "artifact_manifest.json").read_text(encoding="utf-8")
        )
        checks["artifact_self_hash_valid"] = artifact.get(
            "artifact_manifest_content_sha256"
        ) == _artifact_self_hash(artifact)
        checks["artifact_protocol_matches"] = (
            artifact.get("schema_version") == "table4_artifact_manifest_v1"
            and artifact.get("dataset") == DATASET_LABEL
            and artifact.get("protocol_id") == PROTOCOL_ID
            and set(artifact.get("files", {})) == set(RECEIPT_FILES)
        )
        manifest = json.loads(
            (output / "frozen_manifest.json").read_text(encoding="utf-8")
        )
        checks["manifest_self_hash_valid"] = manifest.get(
            "manifest_content_sha256"
        ) == _manifest_self_hash(manifest)
        checks["manifest_binding_matches"] = artifact.get(
            "run_manifest_content_sha256"
        ) == manifest.get("manifest_content_sha256")
        checks["artifact_hashes_match"] = all(
            (output / name).is_file()
            and (output / name).stat().st_size == int(binding["bytes"])
            and sha256_file(output / name) == binding["sha256"]
            for name, binding in artifact.get("files", {}).items()
        ) and set(artifact.get("files", {})) == set(RECEIPT_FILES)
        verification = json.loads(
            (output / "verification.json").read_text(encoding="utf-8")
        )
        checks["base_verification_passed"] = verification.get("status") == "PASS"
        timing = json.loads((output / "timing.json").read_text(encoding="utf-8"))
        checks["timing_denominators_match"] = (
            timing.get("n_eval") == artifact.get("n_eval")
            and timing.get("j_eval") == artifact.get("j_eval")
            and timing.get("expected_states") == artifact.get("expected_states")
            and timing.get("executed_states") == artifact.get("executed_states")
            and verification.get("expected_states") == artifact.get("expected_states")
            and verification.get("executed_states") == artifact.get("executed_states")
        )
    except Exception as exc:  # noqa: BLE001
        issues.append(f"{type(exc).__name__}: {exc}")
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "issues": issues,
        "verified_at_utc": _utc_now(),
    }


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    report = _base_report_text(summary, manifest)
    report = report.replace("Ours-500K", DATASET_LABEL)
    report = report.replace(
        "full acquired roster, Table 2 manifest order",
        "frozen per-class N=5 cohort order with max-joint fence/Ferris-wheel overrides",
    )
    return report


def run(args: Any) -> Path:
    global _active_execution_controls

    validate_args(args)
    if args.resume:
        _validate_resume_provenance(args)
    started_at_utc = _utc_now()
    started_perf = time.perf_counter()
    if args.output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n2655"
        args.output = (
            REPO
            / "exp/runtime"
            / f"urdf_table4_ours_pva_per_class_n5_max_joints_{suffix}_{timestamp}"
        )
    _active_execution_controls = {
        "workers": int(args.workers),
        "audit_workers": int(args.audit_workers),
        "child_timeout_seconds": float(args.child_timeout_seconds),
    }
    try:
        output = _base_run(args)
        _finalize_metadata(output, args.table2_manifest)
        _write_receipts(
            output,
            started_at_utc=started_at_utc,
            wall_time_seconds=time.perf_counter() - started_perf,
            resume=args.resume,
            workers=args.workers,
            audit_workers=args.audit_workers,
            child_timeout_seconds=args.child_timeout_seconds,
        )
        receipt_verification = verify_artifact_receipt(output)
        if receipt_verification["status"] != "PASS":
            raise RuntimeError(
                f"artifact receipt verification failed: {receipt_verification}"
            )
        return output
    finally:
        _active_execution_controls = None


def _configure_base() -> None:
    base.SCRIPT = SCRIPT
    base.DATASET_LABEL = DATASET_LABEL
    base.DATASET_ROOT = DEFAULT_COHORT_MANIFEST.parent
    base.DEFAULT_TABLE2_MANIFEST = DEFAULT_COHORT_MANIFEST
    base.PROTOCOL_ID = PROTOCOL_ID
    base.SAMPLE_SIZE = SAMPLE_SIZE
    base.EXPECTED_N_RELEASE = EXPECTED_N_RELEASE
    base.EXPECTED_TABLE2_MANIFEST_FILE_SHA256 = EXPECTED_COHORT_FILE_SHA256
    base.EXPECTED_TABLE2_MANIFEST_CONTENT_SHA256 = EXPECTED_COHORT_CONTENT_SHA256
    base.EXPECTED_ARCHIVE_SHA256 = EXPECTED_COHORT_FILE_SHA256
    base.load_cohort = load_cohort
    base.audit_asset = audit_asset
    base.atomic_json = _atomic_json_with_controls
    base.build_frozen_items = build_frozen_items
    base.evaluate_asset = evaluate_asset
    base.run_one_subprocess = run_one_subprocess
    base.verify_result_against_item = verify_result_against_item
    base.report_text = report_text
    base.run = run


_configure_base()


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments[:1] == ["--verify-output"]:
        if len(arguments) != 2:
            raise ValueError("--verify-output requires exactly one output directory")
        result = verify_artifact_receipt(Path(arguments[1]).resolve(strict=True))
        print(json.dumps(result, ensure_ascii=True, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    return base.main(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
