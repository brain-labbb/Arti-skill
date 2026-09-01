#!/usr/bin/env python3
"""Prepare, merge, and verify sparse Table 4 sampling-protocol corrections.

The historical Table 4 runner sampled every non-fixed URDF joint as an
independent degree of freedom.  A full mimic-aware rerun is unnecessary for
assets whose v1 and v2 state plans are identical.  This tool freezes the exact
set that can change, runs that set through the existing v2 runner, and then
merges those atomic results into the immutable full-cohort v1 evidence.

The resulting artifact is deliberately labelled ``sparse-equivalent-v2``:
unselected v1 records and state objects remain the authoritative observations,
while every asset with a mimic relation, a changed q=0/range plan, or an
invalid v2 sampling plan is replaced by a fresh v2 result.
"""

from __future__ import annotations

import argparse
from collections import Counter
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
from typing import Any, Iterable, Iterator, Mapping, Sequence
import uuid


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
EXP_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_table4_full_release as checker  # noqa: E402
import run_table4_full_release as table4  # noqa: E402
import table123_full_release_common as roster_common  # noqa: E402


SCHEMA = "table4_sparse_sampling_correction_run_v1"
SUMMARY_SCHEMA = "table4_sparse_sampling_correction_summary_v1"
RECEIPT_SCHEMA = "table4_sparse_sampling_correction_receipt_v1"
PREPARE_SCHEMA = "table4_sparse_sampling_correction_prepare_v1"
VERIFICATION_SCHEMA = "table4_sparse_sampling_correction_verification_v1"
ARTIFACT_SCHEMA = "table4_sparse_sampling_correction_artifacts_v1"
EFFECTIVE_PROTOCOL = "sparse_equivalent_mimic_aware_independent_sampling_v2"
V1 = table4.SAMPLING_PROTOCOL_V1
V2 = table4.SAMPLING_PROTOCOL_V2
SELECTION_REASONS = (
    "mimic",
    "range_semantic_change",
    "sampling_plan_error",
)
PHASE_ORDER = {
    "rest": 0,
    "single_joint_sweep": 1,
    "multi_joint_sobol": 2,
}


class CorrectionError(ValueError):
    """Raised when sparse correction evidence cannot close exactly."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_text(value).encode("ascii")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    return canonical_sha256(_without(value, field))


def _require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != _self_hash(value, field):
        raise CorrectionError(f"{label} self-hash mismatch")


def _regular_file(path: Path, label: str) -> Path:
    candidate = Path(path)
    try:
        info = candidate.lstat()
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise CorrectionError(f"{label} is unavailable: {candidate}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise CorrectionError(f"{label} must be a regular non-symlink file")
    return resolved


def _binding(
    path: Path,
    label: str | None = None,
    *,
    stored_path: str | None = None,
) -> dict[str, Any]:
    resolved = _regular_file(path, label or str(path))
    return {
        "path": stored_path if stored_path is not None else str(resolved),
        "bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def _verify_binding(
    value: Mapping[str, Any], label: str, *, root: Path | None = None
) -> Path:
    raw = Path(str(value.get("path", "")))
    if not raw.is_absolute():
        if root is None or ".." in raw.parts:
            raise CorrectionError(f"{label} has an unsafe relative path")
        raw = Path(root) / raw
    path = _regular_file(raw, label)
    if int(value.get("bytes", -1)) != path.stat().st_size:
        raise CorrectionError(f"{label} byte-size drift")
    if value.get("sha256") != sha256_file(path):
        raise CorrectionError(f"{label} SHA-256 drift")
    return path


def _atomic_text(path: Path, text: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_text(path, json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False) + "\n")


def _atomic_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    _atomic_text(path, "".join(canonical_text(dict(row)) + "\n" for row in rows))


def _load_json(path: Path, label: str) -> dict[str, Any]:
    resolved = _regular_file(path, label)
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CorrectionError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise CorrectionError(f"{label} must contain an object")
    return value


def _read_jsonl(path: Path, label: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with _regular_file(path, label).open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                raise CorrectionError(f"blank {label} row {number}")
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise CorrectionError(f"invalid {label} JSON row {number}") from exc
            if not isinstance(value, dict):
                raise CorrectionError(f"non-object {label} row {number}")
            rows.append(value)
    return rows


def _records_path(output: Path) -> Path:
    for name in ("asset_records.jsonl", "records.jsonl"):
        candidate = Path(output) / name
        if candidate.is_file():
            return candidate
    raise CorrectionError(f"no asset records in {output}")


def _load_roster(path: Path) -> dict[str, Any]:
    try:
        return roster_common.load_roster(Path(path), verify_sources=False)
    except Exception as exc:
        raise CorrectionError(f"invalid frozen roster {path}: {exc}") from exc


def _v1_plan_signature(core: Any, joints: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for joint in joints:
        try:
            lower, upper = core._joint_interval(joint)
            evaluable = True
        except (TypeError, ValueError):
            lower = upper = None
            evaluable = False
        if not (
            isinstance(lower, (int, float))
            and isinstance(upper, (int, float))
            and math.isfinite(float(lower))
            and math.isfinite(float(upper))
        ):
            lower = upper = None
            evaluable = False
        rows.append(
            {
                "name": str(joint.get("name", "")),
                "lower": lower,
                "upper": upper,
                "range_evaluable": evaluable,
            }
        )
    return {"joint_rows": rows, "sobol_dimension": len(joints)}


def _v2_plan_signature(plan: Mapping[str, Any]) -> dict[str, Any]:
    def finite_or_none(value: Any) -> float | None:
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return None
        return float(value)

    rows = [
        {
            "name": str(joint.get("name", "")),
            "lower": finite_or_none(joint.get("sampling_lower")),
            "upper": finite_or_none(joint.get("sampling_upper")),
            "range_evaluable": bool(joint.get("sampling_range_evaluable")),
        }
        for joint in plan.get("independent_joints", [])
    ]
    return {"joint_rows": rows, "sobol_dimension": len(rows)}


def analyze_selection(
    roster: Mapping[str, Any], dataset_slug: str
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Return v2 jobs, selected rows, and a deterministic parity contract."""

    jobs = table4.build_jobs(roster, dataset_slug, sampling_protocol=V2)
    rows = list(roster.get("rows", []))
    if len(jobs) != len(rows):
        raise CorrectionError("full-roster job count mismatch")
    core = table4._core()
    selected: list[dict[str, Any]] = []
    unselected: list[dict[str, Any]] = []
    for index, (row, job) in enumerate(zip(rows, jobs, strict=True)):
        if str(row.get("asset_id")) != str(job.get("dataset_id")):
            raise CorrectionError(f"job/roster identity mismatch at {index}")
        reasons: list[str] = []
        plan_error = job.get("sampling_plan_error")
        v1_signature: dict[str, Any] | None = None
        v2_signature: dict[str, Any] | None = None
        if plan_error:
            reasons.append("sampling_plan_error")
        else:
            try:
                joints = core.parse_urdf_joints(Path(str(job["urdf_path"])))
                plan = core.compile_joint_sampling_plan(joints)
            except (OSError, TypeError, ValueError) as exc:
                raise CorrectionError(
                    f"sampling plan became unreadable at {index}: {exc}"
                ) from exc
            if int(plan["mimic_joint_count"]) != int(job["mimic_joint_count"]):
                raise CorrectionError(f"mimic count drift at {index}")
            if int(plan["mimic_joint_count"]) > 0:
                reasons.append("mimic")
            else:
                v1_signature = _v1_plan_signature(core, joints)
                v2_signature = _v2_plan_signature(plan)
                if canonical_text(v1_signature) != canonical_text(v2_signature):
                    reasons.append("range_semantic_change")
        detail = {
            "asset_id": str(row["asset_id"]),
            "parent_ordinal": index,
            "declared_dof_count": int(job["expected_movable_joints"]),
            "independent_dof_count": int(job["independent_dof_count"]),
            "mimic_joint_count": int(job["mimic_joint_count"]),
            "joint_sampling_plan_sha256": job.get("joint_sampling_plan_sha256"),
            "sampling_plan_error": plan_error,
            "reasons": reasons,
            "v1_plan_signature_sha256": (
                canonical_sha256(v1_signature) if v1_signature is not None else None
            ),
            "v2_plan_signature_sha256": (
                canonical_sha256(v2_signature) if v2_signature is not None else None
            ),
        }
        if reasons:
            selected.append(detail)
        else:
            if v1_signature is None or v2_signature is None:
                raise CorrectionError(f"unselected asset lacks parity proof at {index}")
            if detail["v1_plan_signature_sha256"] != detail["v2_plan_signature_sha256"]:
                raise CorrectionError(f"unselected plan mismatch at {index}")
            unselected.append(detail)
    reason_counts = Counter(reason for item in selected for reason in item["reasons"])
    contract = {
        "selection_policy": (
            "rerun every asset with a mimic follower, a non-mimic v1/v2 "
            "range-state-plan difference, or a v2 sampling-plan error"
        ),
        "selected_asset_count": len(selected),
        "unselected_asset_count": len(unselected),
        "reason_counts": {reason: int(reason_counts.get(reason, 0)) for reason in SELECTION_REASONS},
        "selected_assets_sha256": canonical_sha256(selected),
        "unselected_parity_sha256": canonical_sha256(unselected),
        "selected": selected,
    }
    return jobs, selected, contract


def _directory_transaction(output: Path) -> tuple[Path, Path]:
    output = Path(output).resolve()
    if output.exists():
        raise CorrectionError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.parent / f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
    staging.mkdir()
    return output, staging


def prepare(full_roster: Path, output: Path, dataset_slug: str) -> dict[str, Any]:
    full_roster = _regular_file(full_roster, "full roster")
    roster = _load_roster(full_roster)
    jobs, selected, contract = analyze_selection(roster, dataset_slug)
    if not selected:
        raise CorrectionError("sparse correction selected no assets")
    selected_by_id = {item["asset_id"]: item for item in selected}
    subset_rows: list[dict[str, Any]] = []
    for row in roster["rows"]:
        asset_id = str(row["asset_id"])
        detail = selected_by_id.get(asset_id)
        if detail is None:
            continue
        copy = dict(row)
        copy["sparse_parent_ordinal"] = int(detail["parent_ordinal"])
        copy["sparse_selection_reasons"] = list(detail["reasons"])
        copy["ordinal"] = len(subset_rows)
        subset_rows.append(copy)
    if len(subset_rows) != len(selected):
        raise CorrectionError("subset roster selection does not close")

    destination, staging = _directory_transaction(output)
    try:
        roster_jsonl = staging / "full_release_roster.jsonl"
        _atomic_jsonl(roster_jsonl, subset_rows)
        full_binding = _binding(full_roster, "full roster")
        subset: dict[str, Any] = {
            "schema_version": table4.ROSTER_SCHEMA,
            "dataset": str(roster.get("dataset", dataset_slug)),
            "rows": subset_rows,
            "N_eval": len(subset_rows),
            "J_eval": sum(int(row.get("joint_count", 0)) for row in subset_rows),
            "source_bindings": list(roster.get("source_bindings", [])),
            "roster_sha256": canonical_sha256(subset_rows),
            "roster_jsonl_sha256": sha256_file(roster_jsonl),
            "sparse_correction": {
                "schema_version": PREPARE_SCHEMA,
                "dataset_slug": dataset_slug,
                "full_roster": full_binding,
                "full_roster_manifest_content_sha256": roster.get(
                    "manifest_content_sha256"
                ),
                "full_roster_content_sha256": roster.get("roster_sha256"),
                "full_N_eval": int(roster["N_eval"]),
                "full_J_eval": int(roster["J_eval"]),
                **contract,
            },
        }
        subset["manifest_content_sha256"] = _self_hash(
            subset, "manifest_content_sha256"
        )
        subset_path = staging / "full_release_manifest.json"
        _atomic_json(subset_path, subset)
        _load_roster(subset_path)
        subset_jobs = table4.build_jobs(subset, dataset_slug, sampling_protocol=V2)
        if [job["dataset_id"] for job in subset_jobs] != [row["asset_id"] for row in subset_rows]:
            raise CorrectionError("subset v2 jobs do not preserve selected order")
        prepare_receipt = {
            "schema_version": PREPARE_SCHEMA,
            "created_at": utc_now(),
            "dataset_slug": dataset_slug,
            "full_roster": full_binding,
            "subset_roster": _binding(
                subset_path,
                "subset roster",
                stored_path="full_release_manifest.json",
            ),
            "subset_roster_jsonl": _binding(
                roster_jsonl,
                "subset roster JSONL",
                stored_path="full_release_roster.jsonl",
            ),
            "full_N_eval": int(roster["N_eval"]),
            "full_J_eval": int(roster["J_eval"]),
            "subset_N_eval": len(subset_rows),
            "subset_J_eval": int(subset["J_eval"]),
            "selection": contract,
            "v2_jobs_sha256": canonical_sha256(subset_jobs),
            "implementation": {
                "correction_script_sha256": sha256_file(SCRIPT),
                "table4_runner_sha256": sha256_file(table4.SCRIPT),
                "collision_core_sha256": sha256_file(table4.CORE_SCRIPT),
            },
        }
        prepare_receipt["receipt_content_sha256"] = _self_hash(
            prepare_receipt, "receipt_content_sha256"
        )
        _atomic_json(staging / "prepare_receipt.json", prepare_receipt)
        os.replace(staging, destination)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {
        "status": "PREPARED",
        "output": str(destination),
        "subset_roster": str(destination / "full_release_manifest.json"),
        "selected_asset_count": len(selected),
        "reason_counts": contract["reason_counts"],
        "runner_command": [
            str(sys.executable),
            str(table4.SCRIPT),
            "--dataset",
            dataset_slug,
            "--roster",
            str(destination / "full_release_manifest.json"),
            "--output",
            "<SPARSE_V2_OUTPUT>",
            "--sampling-protocol",
            V2,
        ],
    }


def _validate_record_identity(
    record: Mapping[str, Any], row: Mapping[str, Any], order: int, label: str
) -> None:
    if str(record.get("dataset_id", "")) != str(row.get("asset_id", "")):
        raise CorrectionError(f"{label} dataset identity mismatch at {order}")
    if int(record.get("order", -1)) != order:
        raise CorrectionError(f"{label} order mismatch at {order}")
    if record.get("expected_primary_urdf_sha256") != row.get("primary_urdf_sha256"):
        raise CorrectionError(f"{label} primary URDF binding mismatch at {order}")
    declared = int(row.get("joint_count", len(row.get("non_fixed_joints", []))))
    observed = int(record.get("expected_movable_joints", -1))
    if observed != declared:
        raise CorrectionError(f"{label} declared DoF mismatch at {order}")


def _records_by_roster(
    records: Sequence[dict[str, Any]], rows: Sequence[Mapping[str, Any]], label: str
) -> list[dict[str, Any]]:
    if len(records) != len(rows):
        raise CorrectionError(f"{label} record count mismatch")
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, (record, row) in enumerate(zip(records, rows, strict=True)):
        _validate_record_identity(record, row, index, label)
        asset_id = str(record["dataset_id"])
        if asset_id in seen:
            raise CorrectionError(f"duplicate {label} asset: {asset_id}")
        seen.add(asset_id)
        result.append(record)
    return result


def _iter_state_groups(path: Path, label: str) -> Iterator[tuple[int, str, list[dict[str, Any]]]]:
    current_key: tuple[int, str] | None = None
    current_rows: list[dict[str, Any]] = []
    previous_order = -1
    with _regular_file(path, label).open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                raise CorrectionError(f"blank {label} row {number}")
            try:
                state = json.loads(line)
            except json.JSONDecodeError as exc:
                raise CorrectionError(f"invalid {label} JSON row {number}") from exc
            if not isinstance(state, dict):
                raise CorrectionError(f"non-object {label} row {number}")
            try:
                key = (int(state.get("order", -1)), str(state.get("dataset_id", "")))
            except (TypeError, ValueError) as exc:
                raise CorrectionError(f"invalid {label} identity row {number}") from exc
            if key[0] < 0 or not key[1]:
                raise CorrectionError(f"missing {label} identity row {number}")
            if current_key is None:
                current_key = key
            if key != current_key:
                if key[0] <= previous_order or key[0] <= current_key[0]:
                    raise CorrectionError(f"{label} groups are not strictly ordered")
                yield current_key[0], current_key[1], current_rows
                previous_order = current_key[0]
                current_key = key
                current_rows = []
            current_rows.append(state)
    if current_key is not None:
        if current_key[0] <= previous_order:
            raise CorrectionError(f"{label} final group is not ordered")
        yield current_key[0], current_key[1], current_rows


def _group_hash(rows: Sequence[Mapping[str, Any]]) -> str:
    return canonical_sha256(list(rows))


def _phase_counts(rows: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    counts = {"rest": 0, "single": 0, "sobol": 0}
    seen: set[tuple[str, str, int]] = set()
    for state in rows:
        phase = str(state.get("phase", ""))
        if phase not in PHASE_ORDER:
            raise CorrectionError(f"unknown state phase: {phase!r}")
        try:
            sample = int(state.get("sample_index", -1))
        except (TypeError, ValueError) as exc:
            raise CorrectionError("state sample index is invalid") from exc
        joint = str(state.get("joint_name") or "")
        key = (phase, joint, sample)
        if key in seen:
            raise CorrectionError(f"duplicate state coordinate: {key}")
        seen.add(key)
        counts[{"rest": "rest", "single_joint_sweep": "single", "multi_joint_sobol": "sobol"}[phase]] += 1
    return counts


def _validate_state_group(
    record: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    *,
    require_hash: bool,
    label: str,
) -> None:
    asset_id = str(record.get("dataset_id", ""))
    order = int(record.get("order", -1))
    for state in rows:
        if str(state.get("dataset_id", "")) != asset_id or int(state.get("order", -1)) != order:
            raise CorrectionError(f"{label} state/record identity mismatch: {asset_id}")
        expected_identity = record.get("input_identity_sha256")
        if expected_identity and state.get("input_identity_sha256") not in {None, expected_identity}:
            raise CorrectionError(f"{label} state input identity mismatch: {asset_id}")
    counts = _phase_counts(rows)
    for phase in ("rest", "single", "sobol"):
        if counts[phase] != int(record.get(f"{phase}_state_executed", 0) or 0):
            raise CorrectionError(f"{label} {phase} state closure mismatch: {asset_id}")
    declared_count = record.get("state_records_count")
    if declared_count is not None and int(declared_count) != len(rows):
        raise CorrectionError(f"{label} state count mismatch: {asset_id}")
    if require_hash and record.get("state_records_sha256") != _group_hash(rows):
        raise CorrectionError(f"{label} state hash mismatch: {asset_id}")
    protocol = str(record.get("sampling_protocol") or V1)
    if protocol == V2:
        for state in rows:
            if (
                state.get("schema_version") != "table4_state_v2"
                or state.get("sampling_protocol") != V2
                or state.get("joint_sampling_plan_sha256")
                != record.get("joint_sampling_plan_sha256")
                or state.get("input_identity_sha256")
                != record.get("input_identity_sha256")
            ):
                raise CorrectionError(f"{label} v2 state binding mismatch: {asset_id}")


def _consume_group(
    iterator: Iterator[tuple[int, str, list[dict[str, Any]]]],
    current: tuple[int, str, list[dict[str, Any]]] | None,
    order: int,
) -> tuple[list[dict[str, Any]], tuple[int, str, list[dict[str, Any]]] | None]:
    if current is not None and current[0] < order:
        raise CorrectionError(f"orphan state group at order {current[0]}")
    if current is not None and current[0] == order:
        rows = current[2]
        return rows, next(iterator, None)
    return [], current


def _rebind_sparse_record(
    record: Mapping[str, Any], row: Mapping[str, Any], parent_order: int
) -> dict[str, Any]:
    result = dict(record)
    result["order"] = parent_order
    result["category"] = str(row.get("category") or row.get("raw_category") or "")
    if "cohort_origin" in row:
        result["cohort_origin"] = row["cohort_origin"]
    if "roster_ordinal" in result:
        result["sparse_roster_ordinal"] = result["roster_ordinal"]
        result["roster_ordinal"] = parent_order
    return result


def _rebind_sparse_states(
    rows: Sequence[Mapping[str, Any]], roster_row: Mapping[str, Any], parent_order: int
) -> list[dict[str, Any]]:
    rebound: list[dict[str, Any]] = []
    for state in rows:
        copy = dict(state)
        copy["order"] = parent_order
        copy["category"] = str(
            roster_row.get("category") or roster_row.get("raw_category") or ""
        )
        if "cohort_origin" in roster_row:
            copy["cohort_origin"] = roster_row["cohort_origin"]
        rebound.append(copy)
    return rebound


def _validate_parent_bindings(parent_output: Path, records: Path, states: Path) -> dict[str, Any]:
    bindings = {
        "records": _binding(records, "parent records"),
        "state_records": _binding(states, "parent states"),
    }
    summary_path = Path(parent_output) / "summary.json"
    if summary_path.is_file():
        summary = _load_json(summary_path, "parent summary")
        if summary.get("records_sha256") and summary["records_sha256"] != bindings["records"]["sha256"]:
            raise CorrectionError("parent summary records binding mismatch")
        if summary.get("state_records_sha256") and summary["state_records_sha256"] != bindings["state_records"]["sha256"]:
            raise CorrectionError("parent summary states binding mismatch")
        bindings["summary"] = _binding(summary_path, "parent summary")
    artifact_path = Path(parent_output) / "artifact_manifest.json"
    if artifact_path.is_file():
        artifact = _load_json(artifact_path, "parent artifact manifest")
        field = "artifact_manifest_content_sha256"
        if field in artifact:
            _require_self_hash(artifact, field, "parent artifact manifest")
        entries = {str(item.get("path")): item for item in artifact.get("artifacts", []) if isinstance(item, Mapping)}
        for name, binding in ((records.name, bindings["records"]), (states.name, bindings["state_records"])):
            item = entries.get(name)
            if item is None or item.get("sha256") != binding["sha256"] or int(item.get("size", item.get("bytes", -1))) != binding["bytes"]:
                raise CorrectionError(f"parent artifact binding mismatch: {name}")
        bindings["artifact_manifest"] = _binding(artifact_path, "parent artifact manifest")
    return bindings


def _validate_sparse(
    sparse_output: Path,
    subset_path: Path,
    subset: Mapping[str, Any],
    dataset_slug: str,
) -> tuple[list[dict[str, Any]], dict[str, list[dict[str, Any]]], dict[str, Any]]:
    sparse_output = Path(sparse_output).resolve()
    manifest_path = sparse_output / "manifest.json"
    manifest = _load_json(manifest_path, "sparse run manifest")
    _require_self_hash(manifest, "manifest_content_sha256", "sparse run manifest")
    if manifest.get("schema_version") != table4.SCHEMA_VERSION_V2 or manifest.get("sampling_protocol") != V2:
        raise CorrectionError("sparse run is not a v2 Table 4 run")
    if Path(str(manifest.get("roster", ""))).resolve() != subset_path.resolve():
        raise CorrectionError("sparse run roster path mismatch")
    if manifest.get("roster_sha256") != sha256_file(subset_path):
        raise CorrectionError("sparse run roster SHA-256 mismatch")
    if int(manifest.get("N_eval", -1)) != int(subset["N_eval"]):
        raise CorrectionError("sparse run N_eval mismatch")
    if int(manifest.get("J_eval", -1)) != int(subset["J_eval"]):
        raise CorrectionError("sparse run J_eval mismatch")
    if manifest.get("runner_sha256") != sha256_file(table4.SCRIPT):
        raise CorrectionError("sparse run runner implementation drift")
    if manifest.get("collision_core_sha256") != sha256_file(table4.CORE_SCRIPT):
        raise CorrectionError("sparse run collision core drift")
    records_path = _records_path(sparse_output)
    records = _records_by_roster(
        _read_jsonl(records_path, "sparse records"), subset["rows"], "sparse"
    )
    for index, (record, row) in enumerate(zip(records, subset["rows"], strict=True)):
        checker._sampling_metadata(
            record,
            index,
            expected_protocol=V2,
            expected_declared_dof=int(row.get("joint_count", 0)),
        )
    state_path = sparse_output / "state_records.jsonl"
    iterator = _iter_state_groups(state_path, "sparse states")
    current = next(iterator, None)
    by_id: dict[str, list[dict[str, Any]]] = {}
    for index, record in enumerate(records):
        states, current = _consume_group(iterator, current, index)
        _validate_state_group(record, states, require_hash=True, label="sparse")
        by_id[str(record["dataset_id"])] = states
    if current is not None:
        raise CorrectionError("sparse state stream has trailing groups")
    bindings = {
        "manifest": _binding(manifest_path, "sparse manifest"),
        "records": _binding(records_path, "sparse records"),
        "state_records": _binding(state_path, "sparse states"),
    }
    for name in ("summary.json", "checkpoint.json", "artifact_manifest.json"):
        path = sparse_output / name
        if path.is_file():
            bindings[name.removesuffix(".json")] = _binding(path, f"sparse {name}")
    return records, by_id, bindings


def _write_states_and_rebind_records(
    output_path: Path,
    full_rows: Sequence[Mapping[str, Any]],
    parent_records: Sequence[dict[str, Any]],
    parent_states_path: Path,
    sparse_records: Mapping[str, dict[str, Any]],
    sparse_states: Mapping[str, list[dict[str, Any]]],
    target_ids: set[str],
) -> tuple[list[dict[str, Any]], int]:
    parent_iterator = _iter_state_groups(parent_states_path, "parent states")
    parent_current = next(parent_iterator, None)
    merged_records: list[dict[str, Any]] = []
    total_states = 0
    with output_path.open("w", encoding="utf-8") as output:
        for order, (roster_row, parent_record) in enumerate(
            zip(full_rows, parent_records, strict=True)
        ):
            parent_group, parent_current = _consume_group(
                parent_iterator, parent_current, order
            )
            _validate_state_group(
                parent_record,
                parent_group,
                require_hash=False,
                label="parent",
            )
            asset_id = str(roster_row["asset_id"])
            if asset_id in target_ids:
                sparse_record = sparse_records.get(asset_id)
                if sparse_record is None or asset_id not in sparse_states:
                    raise CorrectionError(f"missing sparse replacement: {asset_id}")
                record = _rebind_sparse_record(sparse_record, roster_row, order)
                states = _rebind_sparse_states(
                    sparse_states[asset_id], roster_row, order
                )
            else:
                record = dict(parent_record)
                states = parent_group
            record["state_records_count"] = len(states)
            record["state_records_sha256"] = _group_hash(states)
            _validate_state_group(record, states, require_hash=True, label="merged")
            for state in states:
                output.write(canonical_text(state) + "\n")
            total_states += len(states)
            merged_records.append(record)
        output.flush()
        os.fsync(output.fileno())
    if parent_current is not None:
        raise CorrectionError("parent state stream has trailing groups")
    if set(sparse_records) != target_ids or set(sparse_states) != target_ids:
        raise CorrectionError("sparse replacement identity set mismatch")
    return merged_records, total_states


def _summary_markdown(summary: Mapping[str, Any]) -> str:
    labels = (
        ("rest_all_pair_cf", "Rest All-pair CF"),
        ("rest_non_adjacent_cf", "Rest Non-adjacent CF"),
        ("single_joint_sweep_cf", "Single-joint Sweep CF"),
        ("multi_joint_sobol_cf", "Multi-joint Sobol CF"),
        ("collision_state_rate", "Collision-state Rate"),
        ("aor", "AOR"),
        ("max_penetration", "Max Penetration"),
        ("collision_free_range", "Collision-free Range"),
        ("strict_collision_pass", "Strict Collision Pass"),
    )
    lines = [
        f"# {summary['dataset']}: sparse-equivalent mimic-aware Table 4",
        "",
        f"Status: **{summary['status']}**",
        "",
        f"N_eval: {summary['n_eval']}  \\",
        f"J_eval: {summary['j_eval']}  \\",
        f"Independent J_eval: {summary['independent_j_eval']}",
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for key, label in labels:
        metric = summary["metrics"][key]
        if str(metric.get("status", "")).upper() == "N/E":
            value = "N/E"
        elif key == "max_penetration":
            value = str(metric.get("maximum_observed_normalized"))
        else:
            numerator = metric.get("numerator")
            denominator = metric.get("denominator")
            value = f"{numerator} / {denominator}"
        lines.append(f"| {label} | {value} |")
    lines.extend(
        [
            "",
            "This is a sparse-equivalent v2 result: all protocol-sensitive assets "
            "were rerun under v2 and only proven plan-identical v1 assets were reused.",
            "",
        ]
    )
    return "\n".join(lines)


def _artifact_manifest(output: Path, names: Sequence[str]) -> dict[str, Any]:
    entries = []
    for name in names:
        path = _regular_file(output / name, f"output artifact {name}")
        entries.append(
            {"path": name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
        )
    result = {"schema_version": ARTIFACT_SCHEMA, "artifacts": entries}
    result["artifact_manifest_content_sha256"] = _self_hash(
        result, "artifact_manifest_content_sha256"
    )
    return result


def merge(
    *,
    full_roster: Path,
    subset_roster: Path,
    parent_output: Path,
    sparse_output: Path,
    output: Path,
    dataset_slug: str,
) -> dict[str, Any]:
    full_roster = _regular_file(full_roster, "full roster")
    subset_roster = _regular_file(subset_roster, "subset roster")
    full = _load_roster(full_roster)
    subset = _load_roster(subset_roster)
    _jobs, selected, contract = analyze_selection(full, dataset_slug)
    correction = subset.get("sparse_correction")
    if not isinstance(correction, Mapping):
        raise CorrectionError("subset roster lacks sparse-correction binding")
    if correction.get("selected_assets_sha256") != contract["selected_assets_sha256"]:
        raise CorrectionError("subset selection drift from full roster")
    if correction.get("unselected_parity_sha256") != contract["unselected_parity_sha256"]:
        raise CorrectionError("unselected parity proof drift")
    selected_ids = [str(item["asset_id"]) for item in selected]
    subset_ids = [str(row["asset_id"]) for row in subset["rows"]]
    if subset_ids != selected_ids:
        raise CorrectionError("subset roster is not the exact corrective selection")

    parent_output = Path(parent_output).resolve()
    parent_records_path = _records_path(parent_output)
    parent_states_path = parent_output / "state_records.jsonl"
    parent_records = _records_by_roster(
        _read_jsonl(parent_records_path, "parent records"),
        full["rows"],
        "parent",
    )
    for index, record in enumerate(parent_records):
        checker._sampling_metadata(
            record,
            index,
            expected_protocol=V1,
            expected_declared_dof=int(full["rows"][index].get("joint_count", 0)),
        )
    parent_bindings = _validate_parent_bindings(
        parent_output, parent_records_path, parent_states_path
    )
    sparse_records_list, sparse_states, sparse_bindings = _validate_sparse(
        sparse_output, subset_roster, subset, dataset_slug
    )
    sparse_records = {
        str(record["dataset_id"]): record for record in sparse_records_list
    }

    destination, staging = _directory_transaction(output)
    try:
        manifest: dict[str, Any] = {
            "schema_version": SCHEMA,
            "classification": "SPARSE_CORRECTIVE_FULL_COHORT",
            "dataset": str(full.get("dataset", dataset_slug)),
            "dataset_slug": dataset_slug,
            "N_eval": int(full["N_eval"]),
            "J_eval": int(full["J_eval"]),
            "effective_sampling_protocol": EFFECTIVE_PROTOCOL,
            "physical_record_protocols": [V1, V2],
            "selection": contract,
            "inputs": {
                "full_roster": _binding(full_roster, "full roster"),
                "subset_roster": _binding(subset_roster, "subset roster"),
                "parent": parent_bindings,
                "sparse_v2": sparse_bindings,
            },
            "implementation": {
                "correction_script_sha256": sha256_file(SCRIPT),
                "table4_runner_sha256": sha256_file(table4.SCRIPT),
                "table4_checker_sha256": sha256_file(Path(checker.__file__).resolve()),
                "collision_core_sha256": sha256_file(table4.CORE_SCRIPT),
            },
            "execution": {
                "atomic_directory_publication": True,
                "unselected_v1_state_objects_preserved": True,
                "selected_records_replaced_from_sparse_v2": True,
                "asset_and_state_denominators_reaggregated": True,
            },
            "created_at": utc_now(),
        }
        manifest["manifest_content_sha256"] = _self_hash(
            manifest, "manifest_content_sha256"
        )
        _atomic_json(staging / "manifest.json", manifest)
        merged_records, state_count = _write_states_and_rebind_records(
            staging / "state_records.jsonl",
            full["rows"],
            parent_records,
            parent_states_path,
            sparse_records,
            sparse_states,
            set(selected_ids),
        )
        _atomic_jsonl(staging / "records.jsonl", merged_records)
        shutil.copyfile(staging / "records.jsonl", staging / "asset_records.jsonl")
        aggregate = checker.aggregate_records(
            merged_records, int(full["N_eval"]), int(full["J_eval"])
        )
        if aggregate.get("sampling_protocol") != "mixed":
            raise CorrectionError("merged physical protocol accounting is not mixed")
        protocol_counts = Counter(str(row.get("sampling_protocol") or V1) for row in merged_records)
        summary: dict[str, Any] = {
            **aggregate,
            "schema_version": SUMMARY_SCHEMA,
            "dataset": str(full.get("dataset", dataset_slug)),
            "dataset_slug": dataset_slug,
            "sampling_protocol": EFFECTIVE_PROTOCOL,
            "physical_record_protocol_counts": dict(sorted(protocol_counts.items())),
            "independent_j_eval": int(aggregate["independent_dof_count"]),
            "mimic_joint_count": int(aggregate["mimic_joint_count"]),
            "state_records_count": state_count,
            "state_records_sha256": sha256_file(staging / "state_records.jsonl"),
            "records_sha256": sha256_file(staging / "records.jsonl"),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "sparse_equivalence": {
                "selected_asset_count": len(selected_ids),
                "reused_plan_identical_asset_count": len(full["rows"]) - len(selected_ids),
                "selection_reasons": contract["reason_counts"],
                "unselected_parity_sha256": contract["unselected_parity_sha256"],
            },
            "completed_at": utc_now(),
        }
        summary["summary_content_sha256"] = _self_hash(
            summary, "summary_content_sha256"
        )
        _atomic_json(staging / "summary.json", summary)
        _atomic_text(staging / "summary.md", _summary_markdown(summary))
        checkpoint = {
            "schema_version": "table4_sparse_sampling_correction_checkpoint_v1",
            "state": "complete",
            "records": int(full["N_eval"]),
            "state_records": state_count,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "records_sha256": sha256_file(staging / "records.jsonl"),
            "state_records_sha256": sha256_file(staging / "state_records.jsonl"),
            "summary_sha256": sha256_file(staging / "summary.json"),
            "completed_at": utc_now(),
        }
        checkpoint["checkpoint_content_sha256"] = _self_hash(
            checkpoint, "checkpoint_content_sha256"
        )
        _atomic_json(staging / "checkpoint.json", checkpoint)
        core_names = (
            "manifest.json",
            "records.jsonl",
            "asset_records.jsonl",
            "state_records.jsonl",
            "summary.json",
            "summary.md",
            "checkpoint.json",
        )
        artifact = _artifact_manifest(staging, core_names)
        _atomic_json(staging / "artifact_manifest.json", artifact)
        receipt = {
            "schema_version": RECEIPT_SCHEMA,
            "classification": "SPARSE_EQUIVALENT_V2",
            "status": summary["status"],
            "dataset": summary["dataset"],
            "dataset_slug": dataset_slug,
            "N_eval": int(full["N_eval"]),
            "J_eval": int(full["J_eval"]),
            "independent_J_eval": summary["independent_j_eval"],
            "effective_sampling_protocol": EFFECTIVE_PROTOCOL,
            "manifest": _binding(
                staging / "manifest.json", "output manifest", stored_path="manifest.json"
            ),
            "records": _binding(
                staging / "records.jsonl", "output records", stored_path="records.jsonl"
            ),
            "state_records": _binding(
                staging / "state_records.jsonl",
                "output states",
                stored_path="state_records.jsonl",
            ),
            "summary": _binding(
                staging / "summary.json", "output summary", stored_path="summary.json"
            ),
            "checkpoint": _binding(
                staging / "checkpoint.json",
                "output checkpoint",
                stored_path="checkpoint.json",
            ),
            "artifact_manifest": _binding(
                staging / "artifact_manifest.json",
                "output artifact manifest",
                stored_path="artifact_manifest.json",
            ),
            "source_inputs": manifest["inputs"],
            "selection": contract,
            "created_at": utc_now(),
        }
        receipt["receipt_content_sha256"] = _self_hash(
            receipt, "receipt_content_sha256"
        )
        _atomic_json(staging / "correction_receipt.json", receipt)
        verification = verify(staging, write=False)
        _atomic_json(staging / "verification.json", verification)
        os.replace(staging, destination)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {
        "status": "COMPLETE",
        "output": str(destination),
        "selected_asset_count": len(selected_ids),
        "reason_counts": contract["reason_counts"],
        "summary": summary,
    }


def _verify_artifacts(output: Path, artifact: Mapping[str, Any]) -> None:
    _require_self_hash(
        artifact, "artifact_manifest_content_sha256", "artifact manifest"
    )
    if artifact.get("schema_version") != ARTIFACT_SCHEMA:
        raise CorrectionError("artifact manifest schema mismatch")
    seen: set[str] = set()
    for item in artifact.get("artifacts", []):
        if not isinstance(item, Mapping) or not isinstance(item.get("path"), str):
            raise CorrectionError("invalid artifact manifest entry")
        name = str(item["path"])
        if Path(name).is_absolute() or ".." in Path(name).parts or name in seen:
            raise CorrectionError(f"unsafe/duplicate artifact path: {name}")
        seen.add(name)
        path = _regular_file(output / name, f"artifact {name}")
        if int(item.get("bytes", -1)) != path.stat().st_size or item.get("sha256") != sha256_file(path):
            raise CorrectionError(f"artifact binding mismatch: {name}")


def verify(output: Path, *, write: bool = True) -> dict[str, Any]:
    output = Path(output).resolve()
    manifest = _load_json(output / "manifest.json", "correction manifest")
    receipt = _load_json(output / "correction_receipt.json", "correction receipt")
    summary = _load_json(output / "summary.json", "correction summary")
    checkpoint = _load_json(output / "checkpoint.json", "correction checkpoint")
    artifact = _load_json(output / "artifact_manifest.json", "artifact manifest")
    _require_self_hash(manifest, "manifest_content_sha256", "correction manifest")
    _require_self_hash(receipt, "receipt_content_sha256", "correction receipt")
    _require_self_hash(summary, "summary_content_sha256", "correction summary")
    _require_self_hash(checkpoint, "checkpoint_content_sha256", "correction checkpoint")
    if manifest.get("schema_version") != SCHEMA or receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise CorrectionError("correction schema mismatch")
    if manifest.get("effective_sampling_protocol") != EFFECTIVE_PROTOCOL or receipt.get("effective_sampling_protocol") != EFFECTIVE_PROTOCOL:
        raise CorrectionError("effective sampling protocol mismatch")
    _verify_artifacts(output, artifact)
    for field, filename in (
        ("manifest", "manifest.json"),
        ("records", "records.jsonl"),
        ("state_records", "state_records.jsonl"),
        ("summary", "summary.json"),
        ("checkpoint", "checkpoint.json"),
        ("artifact_manifest", "artifact_manifest.json"),
    ):
        binding = receipt.get(field)
        if not isinstance(binding, Mapping):
            raise CorrectionError(f"receipt lacks {field} binding")
        path = _verify_binding(binding, f"receipt {field}", root=output)
        if path != (output / filename).resolve():
            raise CorrectionError(f"receipt {field} path mismatch")

    inputs = manifest.get("inputs")
    if not isinstance(inputs, Mapping):
        raise CorrectionError("manifest input bindings are missing")
    for key in ("full_roster", "subset_roster"):
        if not isinstance(inputs.get(key), Mapping):
            raise CorrectionError(f"manifest lacks {key} binding")
        _verify_binding(inputs[key], key)
    for group_name in ("parent", "sparse_v2"):
        group = inputs.get(group_name)
        if not isinstance(group, Mapping):
            raise CorrectionError(f"manifest lacks {group_name} bindings")
        for name, binding in group.items():
            if not isinstance(binding, Mapping):
                raise CorrectionError(f"invalid {group_name}.{name} binding")
            _verify_binding(binding, f"{group_name}.{name}")

    full_path = Path(str(inputs["full_roster"]["path"]))
    subset_path = Path(str(inputs["subset_roster"]["path"]))
    full = _load_roster(full_path)
    subset = _load_roster(subset_path)
    dataset_slug = str(manifest.get("dataset_slug", ""))
    _jobs, selected, contract = analyze_selection(full, dataset_slug)
    if manifest.get("selection") != contract or receipt.get("selection") != contract:
        raise CorrectionError("frozen selection/parity contract drift")
    target_ids = {str(item["asset_id"]) for item in selected}
    subset_ids = [str(row["asset_id"]) for row in subset["rows"]]
    if subset_ids != [str(item["asset_id"]) for item in selected]:
        raise CorrectionError("subset target identity drift")

    output_records = _records_by_roster(
        _read_jsonl(output / "records.jsonl", "output records"),
        full["rows"],
        "output",
    )
    if (output / "asset_records.jsonl").read_bytes() != (output / "records.jsonl").read_bytes():
        raise CorrectionError("record aliases differ")
    parent_records_path = Path(str(inputs["parent"]["records"]["path"]))
    parent_records = _records_by_roster(
        _read_jsonl(parent_records_path, "parent records"),
        full["rows"],
        "parent",
    )
    sparse_records_path = Path(str(inputs["sparse_v2"]["records"]["path"]))
    sparse_records_list = _records_by_roster(
        _read_jsonl(sparse_records_path, "sparse records"),
        subset["rows"],
        "sparse",
    )
    sparse_records = {str(row["dataset_id"]): row for row in sparse_records_list}
    for index, (record, parent, roster_row) in enumerate(
        zip(output_records, parent_records, full["rows"], strict=True)
    ):
        asset_id = str(record["dataset_id"])
        if asset_id in target_ids:
            expected = _rebind_sparse_record(sparse_records[asset_id], roster_row, index)
        else:
            expected = dict(parent)
        for field in ("state_records_count", "state_records_sha256"):
            expected.pop(field, None)
        observed = dict(record)
        for field in ("state_records_count", "state_records_sha256"):
            observed.pop(field, None)
        if canonical_text(observed) != canonical_text(expected):
            raise CorrectionError(f"record semantic preservation mismatch: {asset_id}")

    state_iterator = _iter_state_groups(output / "state_records.jsonl", "output states")
    state_current = next(state_iterator, None)
    state_count = 0
    for index, record in enumerate(output_records):
        states, state_current = _consume_group(state_iterator, state_current, index)
        _validate_state_group(record, states, require_hash=True, label="output")
        state_count += len(states)
    if state_current is not None:
        raise CorrectionError("output state stream has trailing groups")
    aggregate = checker.aggregate_records(
        output_records, int(full["N_eval"]), int(full["J_eval"])
    )
    for field in ("status", "status_counts", "expected_states", "executed_states", "metrics"):
        if summary.get(field) != aggregate.get(field):
            raise CorrectionError(f"summary aggregate mismatch: {field}")
    if int(summary.get("state_records_count", -1)) != state_count:
        raise CorrectionError("summary state count mismatch")
    if summary.get("records_sha256") != sha256_file(output / "records.jsonl") or summary.get("state_records_sha256") != sha256_file(output / "state_records.jsonl"):
        raise CorrectionError("summary output hash mismatch")
    if int(checkpoint.get("records", -1)) != len(output_records) or int(checkpoint.get("state_records", -1)) != state_count:
        raise CorrectionError("checkpoint count mismatch")

    report: dict[str, Any] = {
        "schema_version": VERIFICATION_SCHEMA,
        "status": "PASS",
        "output": str(output),
        "N_eval": int(full["N_eval"]),
        "J_eval": int(full["J_eval"]),
        "selected_asset_count": len(target_ids),
        "reason_counts": contract["reason_counts"],
        "records": len(output_records),
        "state_records": state_count,
        "records_sha256": sha256_file(output / "records.jsonl"),
        "state_records_sha256": sha256_file(output / "state_records.jsonl"),
        "summary_sha256": sha256_file(output / "summary.json"),
        "receipt_sha256": sha256_file(output / "correction_receipt.json"),
        "all_pass": True,
    }
    report["verification_content_sha256"] = _self_hash(
        report, "verification_content_sha256"
    )
    if write:
        _atomic_json(output / "verification.json", report)
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--dataset-slug", choices=table4.DATASETS, required=True)
    prepare_parser.add_argument("--full-roster", type=Path, required=True)
    prepare_parser.add_argument("--output", type=Path, required=True)
    merge_parser = subparsers.add_parser("merge")
    merge_parser.add_argument("--dataset-slug", choices=table4.DATASETS, required=True)
    merge_parser.add_argument("--full-roster", type=Path, required=True)
    merge_parser.add_argument("--subset-roster", type=Path, required=True)
    merge_parser.add_argument("--parent-output", type=Path, required=True)
    merge_parser.add_argument("--sparse-output", type=Path, required=True)
    merge_parser.add_argument("--output", type=Path, required=True)
    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare(args.full_roster, args.output, args.dataset_slug)
        elif args.command == "merge":
            result = merge(
                full_roster=args.full_roster,
                subset_roster=args.subset_roster,
                parent_output=args.parent_output,
                sparse_output=args.sparse_output,
                output=args.output,
                dataset_slug=args.dataset_slug,
            )
        else:
            result = verify(args.output)
    except (CorrectionError, OSError, ValueError) as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True, ensure_ascii=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
