#!/usr/bin/env python3
"""Run the Table 3 FK protocol over an immutable full-release roster.

This adapter deliberately has no sampling or legacy cohort constants.  The
frozen roster is the sole source of asset and joint denominators; malformed,
unsupported, and worker-failed assets remain explicit records.
"""

from __future__ import annotations

from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
COMMON_PATH = SCRIPT_PATH.with_name("table123_full_release_common.py")


def _load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load module {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CORE = _load_module(CORE_PATH, "table123_table3_core")
COMMON = _load_module(COMMON_PATH, "table123_full_release_common")

SUPPORTED_SCALAR_JOINT_TYPES = {"revolute", "continuous", "prismatic"}


def _failed(row: dict[str, Any], reason: str, *, status: str = "error", samples: int = 21) -> dict[str, Any]:
    count = int(row.get("joint_count", 0))
    joints = []
    for index in range(count):
        joints.append({
            "name": f"__declared_joint_{index}",
            "type": "unsupported",
            "valid_range_pass": False,
            "joint_sweep_success": False,
            "non_degenerate_motion_pass": False,
            "subtree_consistency_pass": False,
            "joint_level_pass": False,
            "sample_count_expected": samples,
        })
    return {
        "asset_key": str(row["asset_id"]),
        "asset_id": str(row["asset_id"]),
        "status": status,
        "error": reason,
        "parse_success": False,
        "tree_valid": False,
        "declared_joint_count": count,
        "joints": joints,
        "strict_kinematic_pass": False,
        "sample_count_expected": samples,
    }


def _supported_joint_view(
    path: Path,
    row: dict[str, Any],
    *,
    scratch_root: Path | None = None,
) -> tuple[Path, Path | None]:
    """Return a temporary URDF with unsupported declarations fixed in place.

    PhysX-Mobility contains a small number of ``floating`` declarations.  The
    release roster excludes those from J_eval, while the analytic core rejects
    unknown joint types.  Converting only those declarations to fixed joints
    preserves the link tree for the supported scalar-joint evaluation without
    modifying the source-bound URDF.
    """

    unsupported_expected = int(row.get("unsupported_joint_count", 0) or 0)
    if unsupported_expected <= 0:
        return path, None
    tree = ET.parse(path)
    changed = 0
    for joint in tree.getroot().findall("joint"):
        joint_type = str(joint.get("type", "")).lower()
        if joint_type != "fixed" and joint_type not in SUPPORTED_SCALAR_JOINT_TYPES:
            joint.set("type", "fixed")
            changed += 1
    if changed != unsupported_expected:
        raise ValueError(
            "unsupported joint inventory changed after roster freeze: "
            f"{changed} != {unsupported_expected}"
        )
    root = Path(scratch_root) if scratch_root is not None else REPO_ROOT / "exp" / "runtime" / ".table3_supported_scratch"
    root.mkdir(parents=True, exist_ok=True)
    scratch = Path(tempfile.mkdtemp(prefix="table3_supported_", dir=str(root)))
    sanitized = scratch / path.name
    tree.write(sanitized, encoding="utf-8", xml_declaration=True)
    return sanitized, scratch


def evaluate_row(
    row: dict[str, Any],
    *,
    samples: int = 21,
    scratch_root: Path | None = None,
) -> dict[str, Any]:
    """Evaluate one source-bound row and retain all roster identity fields."""

    if samples != 21:
        raise ValueError("Table 3 requires exactly K=21 samples per joint")

    result: dict[str, Any]
    path = Path(str(row.get("primary_urdf_path", "")))
    expected_hash = row.get("primary_urdf_sha256")
    cleanup: Path | None = None
    try:
        if not path.is_file() or path.is_symlink():
            raise RuntimeError("primary URDF is missing or is a symlink")
        if expected_hash and COMMON.sha256_file(path) != expected_hash:
            raise RuntimeError("primary URDF changed after roster freeze")
        evaluation_path, cleanup = _supported_joint_view(
            path, row, scratch_root=scratch_root
        )
        result = CORE.evaluate_urdf(
            evaluation_path,
            str(row["asset_id"]),
            samples=samples,
            declared_joint_count_hint=int(row.get("joint_count", 0)),
        )
        expected_count = int(row.get("joint_count", 0))
        if int(result.get("declared_joint_count", 0)) != expected_count and row.get("unsupported_joint_count"):
            # The PhysX release inventory defines J_eval over supported scalar
            # joints.  Keep unsupported declarations in the asset record but
            # exclude them from the supported-joint metric denominator.
            supported_types = {"revolute", "continuous", "prismatic"}
            supported = [joint for joint in result.get("joints", []) if str(joint.get("type", "")).lower() in supported_types]
            result["unsupported_joint_count"] = int(result.get("declared_joint_count", 0)) - len(supported)
            result["joints"] = supported
            result["declared_joint_count"] = len(supported)
            result["strict_kinematic_pass"] = bool(supported) and all(bool(joint.get("joint_level_pass")) for joint in supported)
        if expected_hash and COMMON.sha256_file(path) != expected_hash:
            result = _failed(row, "primary URDF changed during evaluation", samples=samples)
    except Exception as exc:  # noqa: BLE001
        result = _failed(row, f"{type(exc).__name__}: {exc}", samples=samples)
    finally:
        if cleanup is not None:
            shutil.rmtree(cleanup, ignore_errors=True)
    if row.get("unsupported_joint_count"):
        result["unsupported_joint_count"] = int(row["unsupported_joint_count"])
    if row.get("declared_joint_count_all") is not None:
        result["declared_joint_count_all"] = int(row["declared_joint_count_all"])
    result.update({
        "asset_id": str(row["asset_id"]),
        "ordinal": row.get("ordinal"),
        "category": row.get("category", row.get("raw_category")),
        "source_relative_path": row.get("source_relative_path", row.get("portable_path")),
        "primary_urdf_sha256": expected_hash,
        "roster_joint_count": int(row.get("joint_count", 0)),
        "sample_count_expected": samples,
    })
    # The core names this field asset_key; the full-release contract exposes
    # asset_id as the stable identity while retaining both for compatibility.
    result["asset_key"] = str(row["asset_id"])
    expected_count = int(row.get("joint_count", 0))
    if int(result.get("declared_joint_count", 0)) != expected_count:
        # A source changed between roster freeze and evaluation.  Rebind the
        # failure to the frozen denominator instead of allowing aggregation to
        # silently change J_eval.
        result = _failed(row, "declared movable-joint count changed after roster freeze", samples=samples)
        result.update({
            "asset_id": str(row["asset_id"]),
            "asset_key": str(row["asset_id"]),
            "ordinal": row.get("ordinal"),
            "category": row.get("category", row.get("raw_category")),
            "source_relative_path": row.get("source_relative_path", row.get("portable_path")),
            "primary_urdf_sha256": expected_hash,
            "roster_joint_count": expected_count,
            "sample_count_expected": samples,
        })
        if row.get("unsupported_joint_count"):
            result["unsupported_joint_count"] = int(row["unsupported_joint_count"])
        if row.get("declared_joint_count_all") is not None:
            result["declared_joint_count_all"] = int(row["declared_joint_count_all"])
    return result


def aggregate_full_release(records: Iterable[dict[str, Any]], roster: dict[str, Any]) -> dict[str, Any]:
    ordered = list(records)
    rows = list(roster.get("rows", []))
    expected_ids = [str(row["asset_id"]) for row in rows]
    actual_ids = [str(record.get("asset_id", record.get("asset_key"))) for record in ordered]
    if actual_ids != expected_ids:
        raise ValueError("records do not match frozen roster order")
    if len(ordered) != int(roster.get("N_eval", -1)):
        raise ValueError("record count does not match N_eval")
    for row, record in zip(rows, ordered):
        if record.get("ordinal") != row.get("ordinal"):
            raise ValueError(f"record ordinal mismatch for {row['asset_id']}")
        if record.get("primary_urdf_sha256") != row.get("primary_urdf_sha256"):
            raise ValueError(f"record source hash mismatch for {row['asset_id']}")
        if int(record.get("roster_joint_count", -1)) != int(row.get("joint_count", -1)):
            raise ValueError(f"record roster joint binding mismatch for {row['asset_id']}")
        if int(record.get("declared_joint_count", -1)) != int(row.get("joint_count", -1)):
            raise ValueError(f"record joint count mismatch for {row['asset_id']}")
        if len(record.get("joints", [])) != int(row.get("joint_count", 0)):
            raise ValueError(f"record joint list mismatch for {row['asset_id']}")
        if int(record.get("sample_count_expected", 21)) != 21:
            raise ValueError(f"record sample count mismatch for {row['asset_id']}")
    expected_joints = int(roster.get("J_eval", -1))
    actual_joints = sum(int(record.get("declared_joint_count", 0)) for record in ordered)
    if actual_joints != expected_joints:
        raise ValueError(f"joint denominator mismatch: {actual_joints} != {expected_joints}")
    summary = CORE.aggregate_records(ordered, len(ordered))
    summary.update({
        "schema_version": "table123_full_release_table3_v1",
        "dataset": roster.get("dataset"),
        "n_eval": len(ordered),
        "j_eval": actual_joints,
        "roster_sha256": roster.get("roster_sha256"),
        "status_counts": dict(sorted(Counter(str(row.get("status")) for row in ordered).items())),
        "asset_failure_count": sum(str(row.get("status")) != "completed" for row in ordered),
    })
    return summary


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _manifest_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return COMMON.canonical_sha256(payload)


def _write_checkpoint(path: Path, payload: dict[str, Any]) -> None:
    COMMON.write_checkpoint(path, payload)


def _verify_checkpoint(path: Path, *, roster_sha256: str) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    declared = value.get("checkpoint_content_sha256")
    payload = dict(value)
    payload.pop("checkpoint_content_sha256", None)
    if declared != COMMON.canonical_sha256(payload):
        raise ValueError("checkpoint self-hash mismatch")
    if value.get("roster_sha256") != roster_sha256:
        raise ValueError("checkpoint roster binding mismatch")
    return value


def _write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    text = "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _artifact_manifest(output: Path, names: list[str]) -> dict[str, Any]:
    artifacts = []
    for name in sorted(names):
        path = output / name
        artifacts.append({"path": name, "size": path.stat().st_size, "sha256": COMMON.sha256_file(path)})
    manifest = {"schema_version": COMMON.ARTIFACT_SCHEMA_VERSION, "artifacts": artifacts}
    manifest["artifact_manifest_content_sha256"] = COMMON.canonical_sha256(manifest)
    return manifest


def _load_existing_records(path: Path, ids: list[str]) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        return {}
    records: dict[str, dict[str, Any]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        key = str(row.get("asset_id", row.get("asset_key")))
        if key not in ids or key in records:
            raise ValueError(f"invalid or duplicate resume record: {key}")
        records[key] = row
    return records


def _evaluate_row_child(row: dict[str, Any], samples: int, timeout_seconds: float, scratch: Path) -> dict[str, Any]:
    """Run one evaluation in a killable process group."""

    scratch.mkdir(parents=True, exist_ok=True)
    job_dir = Path(tempfile.mkdtemp(prefix="table3_job_", dir=scratch))
    job = job_dir / "job.json"
    result_path = job_dir / "result.json"
    _write_json(job, row)
    environment = dict(os.environ)
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        environment[name] = "1"
    process = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--internal-row",
            str(job),
            "--internal-result",
            str(result_path),
            "--internal-samples",
            str(samples),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=environment,
        start_new_session=True,
    )
    try:
        _stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.communicate()
        return _failed(row, f"asset timeout after {timeout_seconds:g} seconds", status="timeout", samples=samples)
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-2000:]
        return _failed(row, f"worker failed with exit {process.returncode}: {detail}", samples=samples)
    try:
        return json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return _failed(row, f"worker result invalid: {exc}", samples=samples)


def run_full_release(
    roster_path: Path,
    output: Path,
    *,
    workers: int,
    timeout_seconds: float,
    samples: int = 21,
) -> Path:
    if workers <= 0 or timeout_seconds <= 0 or samples != 21:
        raise ValueError("workers/timeout must be positive and Table 3 samples must equal 21")
    # Rebind every source package before starting workers.  Deferred shared
    # bindings (PhysX-Mobility) remain handled by the common contract.
    roster = COMMON.load_roster(Path(roster_path), verify_sources="auto")
    if not roster.get("rows"):
        raise ValueError("full-release roster is empty")
    rows = list(roster["rows"])
    ids = [str(row["asset_id"]) for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate roster identities")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    binding = {
        "schema_version": "table123_full_release_table3_manifest_v1",
        "dataset": roster["dataset"],
        "roster_path": str(Path(roster_path).resolve()),
        "roster_sha256": roster["roster_sha256"],
        "manifest_content_sha256": roster["manifest_content_sha256"],
        "samples": samples,
        "workers": workers,
        "timeout_seconds": timeout_seconds,
        "n_eval": roster["N_eval"],
        "j_eval": roster["J_eval"],
    }
    binding["manifest_content_sha256"] = _manifest_hash(binding)
    manifest_path = output / "manifest.json"
    if manifest_path.is_file():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing.get("manifest_content_sha256") != _manifest_hash(existing) or existing != binding:
            raise ValueError("existing output binding differs from frozen roster/config")
        roster_copy = output / "roster_manifest.json"
        if not roster_copy.is_file():
            _write_json(roster_copy, roster)
        elif COMMON.load_roster(roster_copy).get("manifest_content_sha256") != roster.get("manifest_content_sha256"):
            raise ValueError("existing roster manifest differs from frozen roster")
    else:
        _write_json(output / "roster_manifest.json", roster)
        _write_json(manifest_path, binding)
    records_path = output / "records.jsonl"
    existing = _load_existing_records(records_path, ids)
    for key, record in existing.items():
        row = rows[ids.index(key)]
        if record.get("ordinal") != row.get("ordinal"):
            raise ValueError(f"resume ordinal mismatch for {key}")
        if record.get("primary_urdf_sha256") != row.get("primary_urdf_sha256"):
            raise ValueError(f"resume source hash mismatch for {key}")
        if int(record.get("roster_joint_count", -1)) != int(row.get("joint_count", -1)):
            raise ValueError(f"resume joint binding mismatch for {key}")
        if int(record.get("declared_joint_count", -1)) != int(row.get("joint_count", -1)):
            raise ValueError(f"resume declared joint mismatch for {key}")
        if len(record.get("joints", [])) != int(row.get("joint_count", 0)):
            raise ValueError(f"resume joint list mismatch for {key}")
        if int(record.get("sample_count_expected", 21)) != 21:
            raise ValueError(f"resume sample count mismatch for {key}")
    pending = [(index, row) for index, row in enumerate(rows) if str(row["asset_id"]) not in existing]
    results = dict(existing)
    checkpoint_path = output / "checkpoint.json"
    if checkpoint_path.is_file():
        _verify_checkpoint(checkpoint_path, roster_sha256=roster["roster_sha256"])
    _write_checkpoint(checkpoint_path, {
        "state": "running" if pending else "complete",
        "completed": len(results),
        "n_eval": len(rows),
        "j_eval": roster["J_eval"],
        "roster_sha256": roster["roster_sha256"],
    })
    if pending:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            scratch = output / ".worker_scratch"
            futures = {
                executor.submit(_evaluate_row_child, row, samples, timeout_seconds, scratch): (index, row)
                for index, row in pending
            }
            for future in as_completed(futures):
                index, row = futures[future]
                try:
                    record = future.result()
                except Exception as exc:  # noqa: BLE001
                    record = _failed(row, f"worker failure: {type(exc).__name__}: {exc}", samples=samples)
                # Parent-side identity binding is authoritative even if a
                # child exits after writing a malformed payload.
                record.update({
                    "asset_id": row["asset_id"],
                    "asset_key": row["asset_id"],
                    "ordinal": row.get("ordinal"),
                    "category": row.get("category", row.get("raw_category")),
                    "primary_urdf_sha256": row.get("primary_urdf_sha256"),
                    "roster_joint_count": int(row.get("joint_count", 0)),
                    "sample_count_expected": samples,
                })
                if int(record.get("declared_joint_count", -1)) != int(row.get("joint_count", -1)):
                    record = _failed(row, "worker record declared-joint binding mismatch", samples=samples)
                    record.update({
                        "asset_id": row["asset_id"], "asset_key": row["asset_id"],
                        "ordinal": row.get("ordinal"), "category": row.get("category", row.get("raw_category")),
                        "primary_urdf_sha256": row.get("primary_urdf_sha256"),
                        "roster_joint_count": int(row.get("joint_count", 0)), "sample_count_expected": samples,
                    })
                results[str(row["asset_id"])] = record
                _write_checkpoint(checkpoint_path, {
                    "state": "running",
                    "completed": len(results),
                    "n_eval": len(rows),
                    "j_eval": roster["J_eval"],
                    "last_completed_ordinal": index,
                    "roster_sha256": roster["roster_sha256"],
                })
    ordered = [results[asset_id] for asset_id in ids]
    _write_jsonl(records_path, ordered)
    summary = aggregate_full_release(ordered, roster)
    _write_json(output / "summary.json", summary)
    _write_checkpoint(checkpoint_path, {
        "state": "complete",
        "completed": len(rows),
        "n_eval": len(rows),
        "j_eval": roster["J_eval"],
        "roster_sha256": roster["roster_sha256"],
    })
    artifact = _artifact_manifest(output, ["checkpoint.json", "manifest.json", "records.jsonl", "roster_manifest.json", "summary.json"])
    _write_json(output / "artifact_manifest.json", artifact)
    COMMON.verify_artifacts(output)
    return output


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roster", type=Path, nargs="?")
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--samples", type=int, default=21)
    parser.add_argument("--internal-row", type=Path)
    parser.add_argument("--internal-result", type=Path)
    parser.add_argument("--internal-samples", dest="_internal_samples", type=int)
    args = parser.parse_args(argv)
    if args.internal_row is not None or args.internal_result is not None:
        if args.internal_row is None or args.internal_result is None or args._internal_samples != 21:
            raise ValueError("internal row mode requires --internal-row, --internal-result, and samples=21")
        row = json.loads(args.internal_row.read_text(encoding="utf-8"))
        result = evaluate_row(
            row,
            samples=args._internal_samples,
            scratch_root=args.internal_result.parent.parent,
        )
        _write_json(args.internal_result, result)
        return 0
    if args.roster is None or args.output is None:
        parser.error("roster and output are required")
    print(run_full_release(args.roster, args.output, workers=args.workers, timeout_seconds=args.timeout_seconds, samples=args.samples))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
