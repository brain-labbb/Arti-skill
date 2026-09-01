#!/usr/bin/env python3
"""Run Table 2 supplementary diagnostics on the frozen SketchMobility cohort."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = Path(os.environ.get("SKETCHMOBILITY_REPO_ROOT", SCRIPT.parents[2])).resolve()
SOURCE_ROOT = Path(os.environ.get("SKETCHMOBILITY_SOURCE_ROOT", REPO)).resolve()
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from exp.scripts import lam_supplementary_static as static  # noqa: E402
from exp.scripts import run_urdf_table2sup_partnet_mobility as atoms  # noqa: E402
from exp.scripts import sketchmobility_supplementary_common as common  # noqa: E402


VERIFIER_PATH = SCRIPT.with_name("verify_table2sup_urdf_sketch_mobility.py")
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"
DATASET = "SketchMobility"
PROTOCOL_ID = "table2-supplementary-sketchmobility-table1-cohort-n800-v1"
SCHEMA_VERSION = "table2sup-sketchmobility/v1"
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
FORMAL_CATEGORY_COUNT = 67
FORMAL_WORKERS = 4
ASSET_TIMEOUT_SECONDS = 120.0
EXPECTED_STATIC_SHA256 = (
    "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"
)
EXPECTED_TABLE3_MANIFEST_SHA256 = (
    "0f90fbdec03cf4be69dc2b870b2aa7eaa3c00de93e49c005394e402907276f4a"
)
EXPECTED_TABLE3_RECORDS_SHA256 = (
    "13124125cbdef565efc95c7526e052576aead73fa6499d7b0b81bcc0490a24f7"
)
EXPECTED_TABLE2_MANIFEST_SHA256 = (
    "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5"
)
EXPECTED_TABLE2_RECORDS_SHA256 = (
    "03b6d5e0d335052f123664a7a85dcdbc33ffbad8143ffb4bb62560e9b44ea2d1"
)
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = (
    "6b51d10a094bea63d20829cf16a4a4034b5cbe31ebdc3852617fc7690ebed58a"
)
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = (
    "91a1b9b676436f5ff753c0fec6f1dfcc9f4e1c32b60cad1172c14f1ce5c12a40"
)
CHILD_THREAD_ENV = {
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
}


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    return common.sha256_file(path)


def canonical_sha256(value: Any) -> str:
    return common.canonical_sha256(value)


def atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(
            value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False
        )
        + "\n",
    )


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"blank JSONL row at {path}:{line_number}")
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"non-object JSONL row at {path}:{line_number}")
            rows.append(value)
    return rows


def current_evaluator_identity() -> dict[str, Any]:
    static_path = Path(static.__file__).resolve()
    identity = {
        "protocol_version": static.SCHEMA_VERSION,
        "static_module": str(static_path),
        "static_module_sha256": sha256_file(static_path),
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
    }
    for module_name in ("numpy", "trimesh"):
        try:
            module = __import__(module_name)
            identity[f"{module_name}_version"] = getattr(module, "__version__", None)
        except Exception:  # noqa: BLE001
            identity[f"{module_name}_version"] = None
    return identity


def _configure_atoms() -> None:
    atoms.SCRIPT = SCRIPT
    atoms.SCHEMA_VERSION = SCHEMA_VERSION
    atoms.PROTOCOL_ID = PROTOCOL_ID
    atoms.DATASET = DATASET
    atoms.N_EVAL = FORMAL_N_EVAL
    atoms.J_EVAL = FORMAL_J_EVAL
    atoms.EXPECTED_CATEGORY_COUNT = FORMAL_CATEGORY_COUNT
    atoms.URDF_RELATIVE_PATH = "mobility.urdf"
    atoms.WORKERS = FORMAL_WORKERS
    atoms.ASSET_TIMEOUT_SECONDS = ASSET_TIMEOUT_SECONDS
    atoms.CHILD_THREAD_ENV = CHILD_THREAD_ENV


_configure_atoms()


def load_formal_cohort(*, formal: bool) -> dict[str, Any]:
    cohort = common.load_frozen_cohort(formal=formal)
    table3_manifest = common.DEFAULT_TABLE3_RECEIPT / "manifest.json"
    table3_records_path = common.DEFAULT_TABLE3_RECEIPT / "asset_records.jsonl"
    if formal:
        observed = {
            "manifest": sha256_file(table3_manifest),
            "records": sha256_file(table3_records_path),
            "table2_manifest": sha256_file(
                common.DEFAULT_TABLE2_RECEIPT / "manifest.json"
            ),
            "table2_records": sha256_file(
                common.DEFAULT_TABLE2_RECEIPT / "asset_records.jsonl"
            ),
            "table4_asset_records": sha256_file(
                common.DEFAULT_TABLE4_RECEIPT / "asset_records.jsonl"
            ),
            "table4_state_records": sha256_file(
                common.DEFAULT_TABLE4_RECEIPT / "state_records.jsonl"
            ),
        }
        expected = {
            "manifest": EXPECTED_TABLE3_MANIFEST_SHA256,
            "records": EXPECTED_TABLE3_RECORDS_SHA256,
            "table2_manifest": EXPECTED_TABLE2_MANIFEST_SHA256,
            "table2_records": EXPECTED_TABLE2_RECORDS_SHA256,
            "table4_asset_records": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
            "table4_state_records": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
        }
        if observed != expected:
            raise ValueError(f"formal Table 3 receipt drift: {observed}")
    table3_records = read_jsonl(table3_records_path)
    if len(table3_records) != FORMAL_N_EVAL:
        raise ValueError("Table 3 record count mismatch")
    rows: list[dict[str, Any]] = []
    for index, (item, table3_record) in enumerate(
        zip(cohort["rows"], table3_records, strict=True)
    ):
        expected = {
            "asset_id": item["asset_id"],
            "selection_index": index,
            "urdf_sha256": item["urdf_sha256"],
            "package_content_manifest_sha256": item[
                "package_content_manifest_sha256"
            ],
            "declared_joint_count": item["movable_dof_count"],
        }
        observed = {key: table3_record.get(key) for key in expected}
        if observed != expected:
            raise ValueError(f"Table 3 cohort binding mismatch at index {index}")
        rows.append({**item, "table3_declared_joint_count": expected["declared_joint_count"]})
    return {**cohort, "rows": rows}


def build_jobs(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    root = common.DEFAULT_DATASET_ROOT.resolve(strict=True)
    jobs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        asset_id = str(row["asset_id"])
        package = root / asset_id
        jobs.append(
            {
                "selection_index": index,
                "selection_rank": index + 1,
                "asset_id": asset_id,
                "dataset_id": asset_id,
                "category": str(row["category"]),
                "source": str(row["source"]),
                "package": str(package),
                "urdf_relative_path": "mobility.urdf",
                "expected_movable_joints": int(row["movable_dof_count"]),
                "expected_urdf_sha256": str(row["urdf_sha256"]),
                "expected_package_content_manifest_sha256": str(
                    row["package_content_manifest_sha256"]
                ),
                "table3_declared_joint_count": int(
                    row["table3_declared_joint_count"]
                ),
                "frozen_joint_spec_count": len(row.get("joint_specs", [])),
            }
        )
    return jobs


def validate_live_inputs(jobs: list[dict[str, Any]], *, workers: int) -> None:
    def validate(job: dict[str, Any]) -> str | None:
        try:
            package = Path(job["package"])
            binding = common.package_binding(package)
            if (
                binding["content_manifest_sha256"]
                != job["expected_package_content_manifest_sha256"]
            ):
                return f"{job['asset_id']}: package binding drift"
            urdf = package / "mobility.urdf"
            if urdf.is_symlink() or sha256_file(urdf) != job["expected_urdf_sha256"]:
                return f"{job['asset_id']}: URDF binding drift"
        except Exception as exc:  # noqa: BLE001
            return f"{job['asset_id']}: input validation failed: {type(exc).__name__}: {exc}"
        return None

    with ThreadPoolExecutor(max_workers=workers) as executor:
        issues = [issue for issue in executor.map(validate, jobs) if issue is not None]
    if issues:
        raise ValueError("live input package binding drift: " + "; ".join(issues[:10]))


def _failed_record(job: dict[str, Any], issue: str) -> dict[str, Any]:
    record = atoms._failed_record(job=job, issue=issue)
    record.update(
        {
            "selection_rank": job["selection_rank"],
            "expected_package_content_manifest_sha256": job[
                "expected_package_content_manifest_sha256"
            ],
            "package_content_manifest_sha256": None,
        }
    )
    return record


def _parent_failure_record(
    job: dict[str, Any],
    *,
    manifest_hash: str,
    source_snapshots: dict[str, str],
    failure_kind: str,
    returncode: int | None,
) -> dict[str, Any]:
    if failure_kind not in {"timeout", "process_failed", "result_binding_mismatch"}:
        raise ValueError(f"unsupported parent failure kind: {failure_kind}")
    issue = (
        f"parent_runtime_failure:{failure_kind}:returncode="
        f"{returncode if returncode is not None else 'N/E'}"
    )
    record = _failed_record(job, issue)
    record["manifest_content_sha256"] = manifest_hash
    record["child"] = {
        "duration_seconds": None,
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "thread_environment": CHILD_THREAD_ENV,
        "executed_runner": None,
        "executed_runner_sha256": source_snapshots[
            "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
        ],
        "executed_source_snapshots": source_snapshots,
        "parent_failure_attestation": {
            "schema_version": "parent-child-failure-attestation/v1",
            "failure_kind": failure_kind,
            "returncode": returncode,
            "bound_job_sha256": canonical_sha256(
                {"job": job, "manifest_content_sha256": manifest_hash}
            ),
            "manifest_content_sha256": manifest_hash,
        },
    }
    return record


def run_child(job_path: Path, result_path: Path) -> int:
    started = time.monotonic()
    job = json.loads(job_path.read_text(encoding="utf-8"))
    try:
        binding = common.package_binding(Path(job["package"]))
        if (
            binding["content_manifest_sha256"]
            != job["expected_package_content_manifest_sha256"]
        ):
            raise ValueError("full package binding drift")
        record = atoms.audit_partnet_mobility_asset(job)
        record.update(
            {
                "selection_rank": job["selection_rank"],
                "expected_package_content_manifest_sha256": job[
                    "expected_package_content_manifest_sha256"
                ],
                "package_content_manifest_sha256": binding[
                    "content_manifest_sha256"
                ],
                "package_file_count": binding["file_count"],
            }
        )
    except Exception as exc:  # noqa: BLE001
        record = _failed_record(
            job, f"child_preflight_failed: {type(exc).__name__}: {exc}"
        )
    record["manifest_content_sha256"] = job["manifest_content_sha256"]
    record["child"] = {
        "duration_seconds": round(time.monotonic() - started, 6),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "thread_environment": CHILD_THREAD_ENV,
        "executed_runner": str(SCRIPT),
        "executed_runner_sha256": sha256_file(SCRIPT),
        "executed_source_snapshots": {
            "exp/scripts/run_table2sup_urdf_sketch_mobility.py": sha256_file(
                SCRIPT
            ),
            "exp/scripts/verify_table2sup_urdf_sketch_mobility.py": sha256_file(
                VERIFIER_PATH
            ),
            "exp/scripts/lam_supplementary_static.py": sha256_file(
                Path(static.__file__).resolve()
            ),
            "exp/scripts/sketchmobility_supplementary_common.py": sha256_file(
                Path(common.__file__).resolve()
            ),
            "exp/scripts/run_urdf_table2sup_partnet_mobility.py": sha256_file(
                Path(atoms.__file__).resolve()
            ),
        },
    }
    atomic_write_json(result_path, record)
    return 0


def _execute_job(
    job: dict[str, Any],
    work: Path,
    manifest_hash: str,
    frozen_runner: Path,
    source_snapshots: dict[str, str],
) -> dict[str, Any]:
    rank = int(job["selection_rank"])
    job_path = work / "child_jobs" / f"rank_{rank:04d}.json"
    result_path = work / "children" / f"rank_{rank:04d}.json"
    bound_job = {**job, "manifest_content_sha256": manifest_hash}
    atomic_write_json(job_path, bound_job)
    environment = dict(os.environ)
    environment.update(CHILD_THREAD_ENV)
    snapshot_root = frozen_runner.parents[2]
    environment["SKETCHMOBILITY_REPO_ROOT"] = str(REPO)
    environment["SKETCHMOBILITY_SOURCE_ROOT"] = str(snapshot_root)
    environment["PYTHONPATH"] = str(snapshot_root)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    process = subprocess.Popen(
        [
            sys.executable,
            str(frozen_runner),
            "--child",
            "--job",
            str(job_path),
            "--result",
            str(result_path),
        ],
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        return_code = process.wait(timeout=ASSET_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10.0)
        record = _parent_failure_record(
            job,
            manifest_hash=manifest_hash,
            source_snapshots=source_snapshots,
            failure_kind="timeout",
            returncode=None,
        )
        atomic_write_json(result_path, record)
        return record
    if return_code != 0 or not result_path.is_file():
        record = _parent_failure_record(
            job,
            manifest_hash=manifest_hash,
            source_snapshots=source_snapshots,
            failure_kind="process_failed",
            returncode=return_code,
        )
        atomic_write_json(result_path, record)
        return record
    record = json.loads(result_path.read_text(encoding="utf-8"))
    if (
        record.get("asset_id") != job["asset_id"]
        or record.get("manifest_content_sha256") != manifest_hash
    ):
        record = _parent_failure_record(
            job,
            manifest_hash=manifest_hash,
            source_snapshots=source_snapshots,
            failure_kind="result_binding_mismatch",
            returncode=return_code,
        )
        atomic_write_json(result_path, record)
    return record


def aggregate_records(records: list[dict[str, Any]]) -> dict[str, Any]:
    aggregated = atoms.aggregate(records)
    metrics = aggregated["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]
    return {
        "n_eval": len(records),
        "j_eval": sum(int(record["expected_movable_joints"]) for record in records),
        "status_counts": aggregated["status_counts"],
        "parse_passed_assets": aggregated["parse_passed_assets"],
        "metrics": {
            "visual_bearing_collision_coverage": {
                "passed": visual["asset"]["numerator"],
                "denominator": visual["asset"]["denominator"],
                "percent": visual["asset"]["percent"],
                "link_micro": visual["link_micro"],
                "link_extraction_complete_assets": visual[
                    "link_extraction_complete_assets"
                ],
                "zero_visual_bearing_assets_completed": visual[
                    "zero_visual_bearing_assets_completed"
                ],
            },
            "joint_limit_portability": metrics["joint_limit_portability"],
            "joint_dynamics_coverage": metrics["joint_dynamics_coverage"],
            "placeholder_mass_incidence": metrics[
                "placeholder_mass_incidence"
            ],
        },
        "category_macro": aggregated["category_macro"],
    }


def _render_summary(summary: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]
    portability = metrics["joint_limit_portability"]
    dynamics = metrics["joint_dynamics_coverage"]
    return (
        "# SketchMobility Table 2 supplementary\n\n"
        f"- N_eval: {summary['n_eval']}\n"
        f"- J_eval: {summary['j_eval']}\n"
        f"- Visual-bearing collision coverage: {visual['passed']} / {visual['denominator']} ({visual['percent']:.2f}%)\n"
        f"- Joint-limit portability: {portability['numerator']} / {portability['denominator']} ({portability['percent']:.2f}%)\n"
        f"- Joint dynamics coverage: {dynamics['numerator']} / {dynamics['denominator']} ({dynamics['percent']:.2f}%)\n"
        "- Placeholder-mass incidence: N/E (placeholder_registry_empty)\n"
    )


def _source_paths() -> dict[str, Path]:
    return {
        "exp/scripts/run_table2sup_urdf_sketch_mobility.py": SCRIPT,
        "exp/scripts/verify_table2sup_urdf_sketch_mobility.py": VERIFIER_PATH,
        "exp/scripts/lam_supplementary_static.py": Path(static.__file__).resolve(),
        "exp/scripts/sketchmobility_supplementary_common.py": Path(
            common.__file__
        ).resolve(),
        "exp/scripts/run_urdf_table2sup_partnet_mobility.py": Path(
            atoms.__file__
        ).resolve(),
    }


def current_source_hashes() -> dict[str, str]:
    return {relative: sha256_file(path) for relative, path in _source_paths().items()}


def _snapshot_sources(work: Path) -> dict[str, str]:
    sources = _source_paths()
    root = work / "source_snapshots"
    root.mkdir(parents=True, exist_ok=True)
    hashes: dict[str, str] = {}
    for relative, source in sources.items():
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        hashes[relative] = sha256_file(target)
    return hashes


def _artifact_manifest(work: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for path in sorted(work.rglob("*")):
        if not path.is_file() or path.name in {
            "artifact_manifest.json",
            "verification.json",
            "receipt_digest.json",
        }:
            continue
        if path.is_symlink():
            raise ValueError(f"receipt contains symlink: {path}")
        rows.append(
            {
                "path": path.relative_to(work).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return {"schema_version": "artifact-closure/v1", "files": rows}


def _load_verifier():
    spec = importlib.util.spec_from_file_location(
        "table2sup_sketch_frozen_verifier", VERIFIER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load verifier: {VERIFIER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _run_frozen_verifier(
    root: Path, *, write_receipt: bool, require_receipt_digest: bool
) -> dict[str, Any]:
    verifier = (
        root
        / "source_snapshots/exp/scripts/verify_table2sup_urdf_sketch_mobility.py"
    )
    environment = dict(os.environ)
    environment["SKETCHMOBILITY_REPO_ROOT"] = str(REPO)
    environment["SKETCHMOBILITY_SOURCE_ROOT"] = str(root / "source_snapshots")
    environment["PYTHONPATH"] = str(root / "source_snapshots")
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    command = [sys.executable, str(verifier), str(root)]
    if not write_receipt:
        command.append("--no-write")
    if not require_receipt_digest:
        command.append("--allow-missing-receipt-digest")
    process = subprocess.run(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        env=environment,
    )
    try:
        result = json.loads(process.stdout.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise RuntimeError("frozen verifier produced invalid JSON") from exc
    if process.returncode != 0 or result.get("status") != "PASS":
        detail = process.stderr.decode("utf-8", errors="replace")[-4000:]
        raise RuntimeError(f"frozen verifier failed: {result}; stderr={detail}")
    return result


def _receipt_digest(root: Path) -> dict[str, Any]:
    files = [
        {
            "path": path.relative_to(root).as_posix(),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for path in sorted(root.rglob("*"))
        if path.is_file()
        and not path.is_symlink()
        and path.name != "receipt_digest.json"
    ]
    return {
        "schema_version": "whole-receipt-digest/v1",
        "file_count": len(files),
        "files": files,
        "tree_sha256": canonical_sha256(files),
    }


def _validate_resume(
    args: argparse.Namespace,
    work: Path,
    manifest: dict[str, Any],
    jobs: list[dict[str, Any]],
    identity: dict[str, Any],
) -> None:
    payload = dict(manifest)
    declared_hash = payload.pop("manifest_content_sha256", None)
    if declared_hash != canonical_sha256(payload):
        raise RuntimeError("resume manifest self-hash mismatch")
    expected_classification = "FORMAL" if args.mode == "formal" else "SMOKE"
    if (
        manifest.get("protocol_id") != PROTOCOL_ID
        or manifest.get("mode") != args.mode
        or manifest.get("classification") != expected_classification
        or manifest.get("items") != jobs
        or manifest.get("evaluator") != identity
        or manifest.get("execution", {}).get("workers") != args.workers
        or manifest.get("execution", {}).get("asset_timeout_seconds")
        != ASSET_TIMEOUT_SECONDS
    ):
        raise RuntimeError("resume manifest configuration mismatch")
    if sha256_file(work / "protocol_snapshot.md") != manifest.get(
        "protocol_snapshot_sha256"
    ):
        raise RuntimeError("resume protocol snapshot mismatch")
    for relative, digest in manifest.get("source_snapshots", {}).items():
        path = work / "source_snapshots" / relative
        if not path.is_file() or path.is_symlink() or sha256_file(path) != digest:
            raise RuntimeError(f"resume source snapshot mismatch: {relative}")


def _load_resume_record(
    path: Path, job: dict[str, Any], manifest_hash: str, frozen_runner_hash: str
) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    record = json.loads(path.read_text(encoding="utf-8"))
    if (
        record.get("selection_index") != job["selection_index"]
        or record.get("selection_rank") != job["selection_rank"]
        or record.get("asset_id") != job["asset_id"]
        or record.get("expected_urdf_sha256") != job["expected_urdf_sha256"]
        or record.get("expected_movable_joints") != job["expected_movable_joints"]
        or record.get("manifest_content_sha256") != manifest_hash
    ):
        raise RuntimeError(f"resume child binding mismatch: {path.name}")
    if record.get("status") != "completed":
        return None
    if record.get("child", {}).get("executed_runner_sha256") != frozen_runner_hash:
        raise RuntimeError(f"resume child runner mismatch: {path.name}")
    return record


def smoke_receipt_binding(path: Path | None) -> dict[str, Any]:
    if path is None:
        raise ValueError("formal mode requires a passing N=5 smoke receipt")
    root = path.resolve(strict=True)
    manifest_path = root / "manifest.json"
    summary_path = root / "summary.json"
    verification_path = root / "verification.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    observed = {
        "classification": manifest.get("classification"),
        "mode": manifest.get("mode"),
        "n_eval": summary.get("n_eval"),
        "workers": manifest.get("execution", {}).get("workers"),
        "asset_timeout_seconds": manifest.get("execution", {}).get(
            "asset_timeout_seconds"
        ),
        "full_cohort_ordered_asset_ids_sha256": manifest.get("selection", {}).get(
            "full_cohort_ordered_asset_ids_sha256"
        ),
        "static_module_sha256": manifest.get("evaluator", {}).get(
            "static_module_sha256"
        ),
        "verification_status": verification.get("status"),
        "source_snapshots": manifest.get("source_snapshots"),
    }
    expected = {
        "classification": "SMOKE",
        "mode": "smoke",
        "n_eval": 5,
        "workers": FORMAL_WORKERS,
        "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
        "full_cohort_ordered_asset_ids_sha256": common.EXPECTED_ORDERED_ASSET_IDS_SHA256,
        "static_module_sha256": EXPECTED_STATIC_SHA256,
        "verification_status": "PASS",
        "source_snapshots": current_source_hashes(),
    }
    if observed != expected:
        raise ValueError(f"smoke receipt N=5 configuration mismatch: {observed}")
    frozen_verifier = (
        root
        / "source_snapshots/exp/scripts/verify_table2sup_urdf_sketch_mobility.py"
    )
    if (
        sha256_file(frozen_verifier)
        != manifest.get("source_snapshots", {}).get(
            "exp/scripts/verify_table2sup_urdf_sketch_mobility.py"
        )
    ):
        raise ValueError("smoke receipt frozen verifier binding mismatch")
    environment = dict(os.environ)
    environment["SKETCHMOBILITY_REPO_ROOT"] = str(REPO)
    environment["SKETCHMOBILITY_SOURCE_ROOT"] = str(root / "source_snapshots")
    environment["PYTHONPATH"] = str(root / "source_snapshots")
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    replay = subprocess.run(
        [sys.executable, str(frozen_verifier), str(root), "--no-write"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        env=environment,
    )
    try:
        replay_receipt = json.loads(replay.stdout.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError("smoke receipt verifier replay produced invalid JSON") from exc
    if replay.returncode != 0 or replay_receipt.get("status") != "PASS":
        detail = replay.stderr.decode("utf-8", errors="replace")[-2000:]
        raise ValueError(f"smoke receipt verifier replay failed: {detail}")
    return {
        "path": str(root),
        "manifest_sha256": sha256_file(manifest_path),
        "summary_sha256": sha256_file(summary_path),
        "asset_records_sha256": sha256_file(root / "asset_records.jsonl"),
        "artifact_manifest_sha256": sha256_file(root / "artifact_manifest.json"),
        "verification_sha256": sha256_file(verification_path),
        "receipt_digest_sha256": sha256_file(root / "receipt_digest.json"),
        "receipt_tree_sha256": json.loads(
            (root / "receipt_digest.json").read_text(encoding="utf-8")
        )["tree_sha256"],
        "frozen_verifier_sha256": sha256_file(frozen_verifier),
        "source_snapshots": manifest["source_snapshots"],
    }


def validate_contract(args: argparse.Namespace) -> None:
    if args.mode == "formal":
        if args.limit is not None or args.workers != FORMAL_WORKERS:
            raise ValueError("formal mode requires N=800 and workers=4")
        smoke_receipt_binding(args.smoke_receipt)
    elif args.limit is None or args.limit < 1 or args.limit > FORMAL_N_EVAL:
        raise ValueError("smoke mode requires --limit in [1, 800]")


def _default_output(mode: str, limit: int | None) -> Path:
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if mode == "formal":
        name = f"table2sup_urdf_sketch_mobility_table1cohort_n800_{stamp}"
    else:
        name = f"table2sup_urdf_sketch_mobility_smoke_n{limit}_{stamp}"
    return DEFAULT_OUTPUT_PARENT / name


def run(args: argparse.Namespace) -> dict[str, Any]:
    validate_contract(args)
    identity = current_evaluator_identity()
    if identity["static_module_sha256"] != EXPECTED_STATIC_SHA256:
        raise ValueError("static evaluator source drift")
    output = (args.output or _default_output(args.mode, args.limit)).resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    work = output.with_name(f".{output.name}.work")
    cohort = load_formal_cohort(formal=True)
    jobs = build_jobs(cohort["rows"])
    if args.mode == "smoke":
        jobs = jobs[: args.limit]
    validate_live_inputs(jobs, workers=args.workers)
    if args.resume:
        if not work.is_dir():
            raise RuntimeError(f"resume staging output does not exist: {work}")
        manifest = json.loads((work / "manifest.json").read_text(encoding="utf-8"))
        _validate_resume(args, work, manifest, jobs, identity)
    else:
        if work.exists():
            raise RuntimeError(f"staging output already exists: {work}")
        work.mkdir(parents=True)
        (work / "child_jobs").mkdir()
        (work / "children").mkdir()
        source_hashes = _snapshot_sources(work)
        shutil.copyfile(PROTOCOL_DOCUMENT, work / "protocol_snapshot.md")
        manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": "FORMAL" if args.mode == "formal" else "SMOKE",
        "mode": args.mode,
        "created_at_utc": utc_now(),
        "selection": {
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_joints"] for job in jobs),
            "ordered_asset_ids_sha256": canonical_sha256(
                [job["asset_id"] for job in jobs]
            ),
            "full_cohort_ordered_asset_ids_sha256": common.EXPECTED_ORDERED_ASSET_IDS_SHA256,
            "policy": "exact frozen Table 4 order; smoke uses prefix only",
        },
        "source": {
            "table4_manifest_path": cohort["manifest_path"],
            "table4_manifest_file_sha256": cohort["manifest_file_sha256"],
            "table4_manifest_content_sha256": cohort["manifest_content_sha256"],
            "table3_manifest_sha256": sha256_file(
                common.DEFAULT_TABLE3_RECEIPT / "manifest.json"
            ),
            "table3_records_sha256": sha256_file(
                common.DEFAULT_TABLE3_RECEIPT / "asset_records.jsonl"
            ),
            "table2_manifest_sha256": sha256_file(
                common.DEFAULT_TABLE2_RECEIPT / "manifest.json"
            ),
            "table2_records_sha256": sha256_file(
                common.DEFAULT_TABLE2_RECEIPT / "asset_records.jsonl"
            ),
            "table4_asset_records_sha256": sha256_file(
                common.DEFAULT_TABLE4_RECEIPT / "asset_records.jsonl"
            ),
            "table4_state_records_sha256": sha256_file(
                common.DEFAULT_TABLE4_RECEIPT / "state_records.jsonl"
            ),
            "dataset_root": str(common.DEFAULT_DATASET_ROOT.resolve(strict=True)),
        },
        "evaluator": identity,
        "execution": {
            "workers": args.workers,
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "fresh_child_per_asset": True,
            "thread_environment": CHILD_THREAD_ENV,
            "placeholder_registry": [],
        },
        "source_snapshots": source_hashes,
        "protocol_snapshot_sha256": sha256_file(work / "protocol_snapshot.md"),
        "items": jobs,
        }
        if args.mode == "formal":
            manifest["source"]["smoke_receipt"] = smoke_receipt_binding(
                args.smoke_receipt
            )
        manifest["manifest_content_sha256"] = canonical_sha256(manifest)
        atomic_write_json(work / "manifest.json", manifest)
        atomic_write_json(
            work / "frozen_config.json",
            {
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "metric_atom": identity,
                "execution": manifest["execution"],
                "denominator_policy": "all intended assets and movable joints fail closed",
            },
        )

    start = time.monotonic()
    frozen_runner = (
        work
        / "source_snapshots/exp/scripts/run_table2sup_urdf_sketch_mobility.py"
    )
    frozen_runner_hash = manifest["source_snapshots"][
        "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
    ]
    records: list[dict[str, Any] | None] = [
        _load_resume_record(
            work / "children" / f"rank_{job['selection_rank']:04d}.json",
            job,
            manifest["manifest_content_sha256"],
            frozen_runner_hash,
        )
        for job in jobs
    ]
    pending = [index for index, record in enumerate(records) if record is None]
    atomic_write_json(
        work / "checkpoint.json",
        {
            "state": "running",
            "completed": len(jobs) - len(pending),
            "remaining": len(pending),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        },
    )
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                _execute_job,
                job,
                work,
                manifest["manifest_content_sha256"],
                frozen_runner,
                manifest["source_snapshots"],
            ): index
            for index in pending
            for job in (jobs[index],)
        }
        for future in as_completed(futures):
            records[futures[future]] = future.result()
            completed = sum(record is not None for record in records)
            atomic_write_json(
                work / "checkpoint.json",
                {
                    "state": "running" if completed < len(jobs) else "aggregating",
                    "completed": completed,
                    "remaining": len(jobs) - completed,
                    "manifest_content_sha256": manifest["manifest_content_sha256"],
                },
            )
    terminal_records = [record for record in records if record is not None]
    if len(terminal_records) != len(jobs):
        raise RuntimeError("missing terminal child record")
    runtime_failures = [
        {
            "selection_rank": record.get("selection_rank"),
            "asset_id": record.get("asset_id"),
            "status": record.get("status"),
            "issues": record.get("issues", []),
        }
        for record in terminal_records
        if record.get("status") != "completed"
    ]
    if runtime_failures:
        atomic_write_json(
            work / "checkpoint.json",
            {
                "state": "runtime_failures_require_resume",
                "completed": len(jobs) - len(runtime_failures),
                "remaining": len(runtime_failures),
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "runtime_failures": runtime_failures,
            },
        )
        raise RuntimeError(
            f"{len(runtime_failures)} child runtime failure(s); rerun with --resume"
        )
    atomic_write_text(
        work / "asset_records.jsonl",
        "".join(
            json.dumps(
                record, sort_keys=True, separators=(",", ":"), ensure_ascii=True
            )
            + "\n"
            for record in terminal_records
        ),
    )
    aggregate = aggregate_records(terminal_records)
    summary = {
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": manifest["classification"],
        "mode": args.mode,
        "run_directory": str(output),
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "wall_seconds": round(time.monotonic() - start, 3),
        **aggregate,
    }
    atomic_write_json(work / "summary.json", summary)
    atomic_write_text(work / "summary.md", _render_summary(summary))
    atomic_write_json(
        work / "checkpoint.json",
        {
            "state": "published",
            "completed": len(jobs),
            "remaining": 0,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        },
    )
    atomic_write_json(work / "artifact_manifest.json", _artifact_manifest(work))

    verification = _run_frozen_verifier(
        work, write_receipt=True, require_receipt_digest=False
    )
    atomic_write_json(work / "receipt_digest.json", _receipt_digest(work))
    verification = _run_frozen_verifier(
        work, write_receipt=False, require_receipt_digest=True
    )
    os.replace(work, output)
    return {"output": output, "summary": summary, "verification": verification}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"))
    parser.add_argument("--limit", type=int)
    parser.add_argument("--workers", type=int, default=FORMAL_WORKERS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--smoke-receipt", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return run_child(args.job, args.result)
    if args.mode is None:
        raise SystemExit("--mode is required")
    result = run(args)
    print(
        json.dumps(
            {
                "output": str(result["output"]),
                "summary": result["summary"],
                "verification": result["verification"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
