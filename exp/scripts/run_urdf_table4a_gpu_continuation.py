#!/usr/bin/env python3
"""Continue the interrupted PV-A Table 4a run on an explicitly selected GPU.

This is an operational continuation driver, not a replacement formal runner.
It reuses the existing CPU prefix without evaluating it again, consumes the
rank-0102 GPU probe when present, and labels all later records as a mixed,
non-public CPU-prefix/GPU-suffix result.  The frozen formal runner is left
untouched because its protocol explicitly freezes the CPU backend.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_lam_supplementary_v1 as lam  # noqa: E402
from exp.scripts import run_urdf_table4a_articraft10k as shared  # noqa: E402
from exp.scripts import run_urdf_table4a_ours_pva_per_class_n5 as ours  # noqa: E402


UPSTREAM_TABLE4 = REPO / (
    "exp/runtime/urdf_table4_ours_pva_per_class_n5_max_joints_n2655_"
    "20260824T060336Z"
)
OLD_PARTIAL = REPO / (
    "exp/runtime/table4a_urdf_ours_pva_per_class_n5_max_joints_n2655_"
    "20260825T015318Z"
)
GPU_PROBE = REPO / "exp/runtime/table4a_gpu_probe_rank0102_20260825T"
DEFAULT_TIMEOUT_SECONDS = 1800
DEFAULT_POLL_SECONDS = 30
DEFAULT_MIN_FREE_MIB = 100_000
DEFAULT_GPU_CANDIDATES = (3, 4, 7)
PREFIX_LAST_INDEX = 100
PROBE_INDEX = 101


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        "".join(
            json.dumps(dict(row), sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )
    temporary.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def select_gpu_device(
    *,
    candidates: Sequence[int] = DEFAULT_GPU_CANDIDATES,
    min_free_mib: int = DEFAULT_MIN_FREE_MIB,
) -> dict[str, Any] | None:
    """Return an idle, sufficiently large GPU without touching other jobs."""

    query = subprocess.run(
        [
            "nvidia-smi",
            "--query-gpu=index,uuid,name,memory.total,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if query.returncode != 0:
        return None
    rows: dict[int, dict[str, Any]] = {}
    for raw in query.stdout.splitlines():
        fields = [field.strip() for field in raw.split(",")]
        if len(fields) != 6:
            continue
        try:
            index = int(fields[0])
            total = int(fields[3])
            used = int(fields[4])
            utilization = int(fields[5])
        except ValueError:
            continue
        rows[index] = {
            "index": index,
            "uuid": fields[1],
            "name": fields[2],
            "memory_total_mib": total,
            "memory_used_mib": used,
            "memory_free_mib": total - used,
            "utilization_gpu_percent": utilization,
        }
    apps = subprocess.run(
        [
            "nvidia-smi",
            "--query-compute-apps=gpu_uuid,pid,process_name,used_memory",
            "--format=csv,noheader,nounits",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    busy_uuids: set[str] = set()
    if apps.returncode == 0:
        for raw in apps.stdout.splitlines():
            fields = [field.strip() for field in raw.split(",")]
            if fields and fields[0]:
                busy_uuids.add(fields[0])
    for index in candidates:
        row = rows.get(int(index))
        if row is None:
            continue
        if row["uuid"] in busy_uuids:
            continue
        if int(row["memory_free_mib"]) < int(min_free_mib):
            continue
        if int(row["utilization_gpu_percent"]) > 10:
            continue
        return row
    return None


def configure_jobs() -> tuple[list[dict[str, Any]], dict[str, bool], dict[str, Any]]:
    """Reconstruct the frozen job list without invoking the formal runner."""

    frozen_protocol = UPSTREAM_TABLE4 / "protocol_document_at_freeze.md"
    ours.PROTOCOL_DOCUMENT = frozen_protocol
    cohort = ours.load_canonical_cohort()
    binding = ours.validate_table4_receipt(
        UPSTREAM_TABLE4,
        cohort,
        expected_n=ours.N_EVAL,
        expected_j=ours.J_EVAL,
        expected_category_count=ours.EXPECTED_CATEGORY_COUNT,
    )
    ours._BOUND_TABLE4 = binding
    ours.configure_shared_runner(binding)
    manifest = ours.load_source_manifest()
    table3_pass, table3_joint_total = ours.load_table3_joint_pass()
    if table3_joint_total != ours.J_EVAL:
        raise RuntimeError(f"Table 3 denominator drifted: {table3_joint_total}")
    table4_strict = ours.load_table4_strict_pass()
    state_hashes = ours.load_table4_state_hashes()
    jobs, category_info = ours.build_jobs(manifest, table3_pass, state_hashes, formal=True)
    if len(jobs) != ours.N_EVAL:
        raise RuntimeError(f"canonical job count drifted: {len(jobs)}")
    return jobs, table4_strict, category_info


def _record_matches_job(record: Mapping[str, Any], job: Mapping[str, Any], index: int) -> bool:
    return (
        int(record.get("selection_index", -1)) == index
        and str(record.get("dataset_id")) == str(job["dataset_id"])
        and str(record.get("asset_id")) == str(job["asset_id"])
        and str(record.get("package")) == str(job["package"])
        and str(record.get("expected_urdf_sha256")) == str(job["expected_urdf_sha256"])
        and str(record.get("input_identity_sha256")) == str(job["input_identity_sha256"])
        and int(record.get("expected_movable_dof", -1)) == int(job["expected_movable_dof"])
        and str(record.get("expected_package_file_binding_sha256"))
        == str(job["expected_package_file_binding_sha256"])
    )


def load_prefix(old_dir: Path, jobs: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for index in range(PREFIX_LAST_INDEX + 1):
        path = old_dir / "children" / f"rank_{index + 1:04d}.json"
        if not path.is_file():
            raise RuntimeError(f"missing frozen prefix result: {path}")
        record = read_json(path)
        if not _record_matches_job(record, jobs[index], index):
            raise RuntimeError(f"prefix identity mismatch at rank {index + 1}: {path}")
        # ``children/`` is scratch written before the Ours wrapper merges
        # zero-width joints back into the full frozen denominator.  Rebuild
        # that deterministic merge in memory; the Genesis child is not rerun.
        records.append(ours.merge_range_failures([dict(jobs[index])], [record])[0])
    if [int(row["selection_index"]) for row in records] != list(range(PREFIX_LAST_INDEX + 1)):
        raise RuntimeError("prefix selection order is not contiguous")
    return records


def runnable_job(job: Mapping[str, Any], cache_path: Path) -> dict[str, Any]:
    value = dict(job)
    value["joints"] = [
        dict(joint) for joint in job["joints"] if bool(joint.get("range_evaluable"))
    ]
    value["expected_state_count"] = ours.SINGLE_SAMPLES * len(value["joints"])
    value["genesis_cache_path"] = str(cache_path)
    value["template_cache_path"] = None
    return value


def gpu_execution_receipt(device: Mapping[str, Any], *, source: str) -> dict[str, Any]:
    return {
        "status": "COMPLETE",
        "source": source,
        "backend": "cuda",
        "logical_device": "cuda:0",
        "physical_index": int(device["index"]),
        "uuid": str(device["uuid"]),
        "name": str(device["name"]),
        "visible_devices": str(device["index"]),
        "selection_snapshot": dict(device),
        "publication_status": "NON_PUBLIC_MIXED_BACKEND",
    }


def attach_gpu_receipt(record: dict[str, Any], receipt: Mapping[str, Any]) -> dict[str, Any]:
    value = dict(record)
    value["gpu_execution"] = dict(receipt)
    for joint in value.get("joint_records", []):
        for state in joint.get("state_summaries", []) or []:
            if isinstance(state, dict):
                state["gpu_execution"] = {
                    "backend": receipt["backend"],
                    "logical_device": receipt["logical_device"],
                    "physical_index": receipt["physical_index"],
                    "uuid": receipt["uuid"],
                }
    return value


def patch_gpu_runtime() -> None:
    """Patch only the child process; the frozen CPU modules remain unchanged."""

    physical = int(os.environ["GPU_CONTINUATION_PHYSICAL_INDEX"])
    uuid = os.environ["GPU_CONTINUATION_UUID"]
    os.environ["GS_PARA_LEVEL"] = "1"
    lam.THREAD_ENV_VALUES = dict(lam.THREAD_ENV_VALUES)
    lam.THREAD_ENV_VALUES["GS_PARA_LEVEL"] = "1"
    original_binding = lam.genesis_runtime_binding
    import genesis as gs

    original_init = gs.init

    def gpu_binding(*, require_current_interpreter: bool = True, expected_cache_path: Path | None = None):
        runtime = dict(
            original_binding(
                require_current_interpreter=require_current_interpreter,
                expected_cache_path=expected_cache_path,
            )
        )
        runtime["backend"] = "cuda"
        runtime["device"] = "cuda:0"
        runtime["gpu"] = {
            "backend": "cuda",
            "logical_device": "cuda:0",
            "physical_index": physical,
            "uuid": uuid,
            "visible_devices": str(physical),
        }
        thread_environment = dict(runtime.get("thread_environment", {}))
        thread_environment["GS_PARA_LEVEL"] = "1"
        runtime["thread_environment"] = thread_environment
        return runtime

    def gpu_init(*args: Any, **kwargs: Any):
        if kwargs.get("backend") == gs.cpu:
            kwargs = dict(kwargs)
            kwargs["backend"] = gs.cuda
        return original_init(*args, **kwargs)

    lam.genesis_runtime_binding = gpu_binding
    gs.init = gpu_init


def child_main(job_path: Path, result_path: Path) -> int:
    ours.configure_shared_runner(None)
    patch_gpu_runtime()
    return ours.run_child(job_path, result_path)


def launch_child(
    full_job: Mapping[str, Any],
    *,
    outdir: Path,
    device: Mapping[str, Any],
    affinity: str,
    timeout_seconds: float,
) -> dict[str, Any]:
    index = int(full_job["selection_index"])
    job_dir = outdir / "child_jobs"
    raw_dir = outdir / "raw_children"
    child_dir = outdir / "children"
    log_dir = outdir / "logs"
    for directory in (job_dir, raw_dir, child_dir, log_dir):
        directory.mkdir(parents=True, exist_ok=True)
    cache = outdir / "genesis-cache" / "shared"
    cache.mkdir(parents=True, exist_ok=True)
    runnable = runnable_job(full_job, cache)
    job_path = job_dir / f"rank_{index + 1:04d}.json"
    raw_path = raw_dir / f"rank_{index + 1:04d}.json"
    child_path = child_dir / f"rank_{index + 1:04d}.json"
    atomic_json(job_path, runnable)
    env = os.environ.copy()
    env.update(
        {
            "CUDA_VISIBLE_DEVICES": str(device["index"]),
            "GPU_CONTINUATION_PHYSICAL_INDEX": str(device["index"]),
            "GPU_CONTINUATION_UUID": str(device["uuid"]),
            "GS_CACHE_FILE_PATH": str(cache.resolve()),
            "LAM_GENESIS_CPU_AFFINITY": affinity,
            "GS_PARA_LEVEL": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    launcher = REPO / "exp/scripts/exec_with_cpu_affinity.py"
    command = [
        str(lam.DEFAULT_GENESIS_PYTHON.resolve(strict=True)),
        "-S",
        str(launcher),
        "--",
        str(SCRIPT),
        "--child",
        "--job",
        str(job_path),
        "--result",
        str(raw_path),
    ]
    log_path = log_dir / f"rank_{index + 1:04d}.log"
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=str(REPO),
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        reason: str | None = None
        try:
            return_code = process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            reason = f"asset_timeout after {timeout_seconds}s"
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except OSError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except OSError:
                    pass
                process.wait(timeout=30)
            return_code = -signal.SIGTERM
    elapsed = round(time.monotonic() - started, 3)
    if raw_path.is_file():
        raw = read_json(raw_path)
    else:
        issue = reason or f"child_process_failed: rc={return_code}"
        raw = ours.failed_asset_record(runnable, issue)
        atomic_json(raw_path, raw)
    if reason is None and return_code != 0 and raw.get("status") == "completed":
        raw["status"] = "error"
        raw.setdefault("issues", []).append(f"child_process_failed: rc={return_code}")
    merged = ours.merge_range_failures([dict(full_job)], [raw])[0]
    receipt = gpu_execution_receipt(device, source="gpu_continuation")
    merged = attach_gpu_receipt(merged, receipt)
    merged["continuation"] = {
        "elapsed_seconds": elapsed,
        "raw_result_sha256": sha256_file(raw_path),
        "return_code": return_code,
        "job_path": str(job_path),
        "log_path": str(log_path),
    }
    atomic_json(child_path, merged)
    return merged


def load_continuation_records(outdir: Path) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for path in sorted((outdir / "children").glob("rank_*.json")):
        record = read_json(path)
        index = int(record.get("selection_index", -1))
        if index >= 0:
            result[index] = record
    return result


def write_progress(
    outdir: Path,
    *,
    prefix: Sequence[Mapping[str, Any]],
    continuation: Mapping[int, Mapping[str, Any]],
    jobs: Sequence[Mapping[str, Any]],
    table4_strict: Mapping[str, bool],
    started_at: str,
) -> None:
    records = list(prefix) + [continuation[index] for index in sorted(continuation)]
    aggregate = shared.aggregate(records, table4_strict)
    atomic_jsonl(
        outdir / "continuation_records.jsonl",
        [continuation[index] for index in sorted(continuation)],
    )
    atomic_json(
        outdir / "progress.json",
        {
            "status": "RUNNING" if len(records) < len(jobs) else "COMPLETE",
            "started_at_utc": started_at,
            "updated_at_utc": utc_now(),
            "prefix_records_reused": len(prefix),
            "gpu_records_completed": len(continuation),
            "records_total": len(records),
            "records_expected": len(jobs),
            "next_selection_index": next(
                (index for index in range(len(jobs)) if index not in {int(row["selection_index"]) for row in records}),
                None,
            ),
            "mixed_backend": True,
            "publication_status": "NON_PUBLIC_MIXED_BACKEND",
            "metrics": aggregate,
        },
    )


def seed_probe(
    outdir: Path,
    job: Mapping[str, Any],
    continuation: dict[int, dict[str, Any]],
) -> None:
    if int(job["selection_index"]) != PROBE_INDEX or not (GPU_PROBE / "result.json").is_file():
        return
    source = read_json(GPU_PROBE / "result.json")
    if not _record_matches_job(source, job, PROBE_INDEX):
        raise RuntimeError("rank-0102 GPU probe identity does not match canonical job")
    receipt = source.get("gpu_execution")
    if not isinstance(receipt, Mapping):
        receipt = {
            "status": "COMPLETE",
            "source": "gpu_probe_rank0102",
            "backend": "cuda",
            "logical_device": "cuda:0",
            "physical_index": 4,
            "uuid": "GPU-d94e3f3a-cc8d-0f03-23ac-b10846ed016e",
            "name": "NVIDIA L20X",
            "visible_devices": "4",
            "publication_status": "NON_PUBLIC_MIXED_BACKEND",
            "source_result": str((GPU_PROBE / "result.json").resolve()),
            "source_result_sha256": sha256_file(GPU_PROBE / "result.json"),
        }
    record = attach_gpu_receipt(source, receipt)
    record["continuation"] = {
        "source": str((GPU_PROBE / "result.json").resolve()),
        "source_result_sha256": sha256_file(GPU_PROBE / "result.json"),
        "reused_without_rerun": True,
    }
    atomic_json(outdir / "children" / "rank_0102.json", record)
    atomic_json(outdir / "raw_children" / "rank_0102.json", source)
    continuation[PROBE_INDEX] = record


def run(args: argparse.Namespace) -> int:
    jobs, table4_strict, category_info = configure_jobs()
    old_dir = args.old_dir.resolve(strict=True)
    prefix = load_prefix(old_dir, jobs)
    if args.output_dir is None:
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output_dir = REPO / "exp/runtime" / f"table4a_gpu_continuation_from_rank0101_{stamp}"
    outdir = args.output_dir.resolve()
    if outdir.exists() and not args.resume:
        raise RuntimeError(f"output exists; pass --resume to continue: {outdir}")
    outdir.mkdir(parents=True, exist_ok=True)
    for name in ("children", "raw_children", "child_jobs", "logs", "genesis-cache"):
        (outdir / name).mkdir(parents=True, exist_ok=True)
    started_at = utc_now()
    metadata_path = outdir / "continuation_metadata.json"
    if not metadata_path.exists():
        atomic_json(
            metadata_path,
            {
                "schema_version": "table4a-gpu-continuation/v1",
                "created_at_utc": started_at,
                "old_partial_directory": str(old_dir),
                "old_prefix_last_selection_index": PREFIX_LAST_INDEX,
                "gpu_probe_directory": str(GPU_PROBE),
                "upstream_table4_directory": str(UPSTREAM_TABLE4),
                "protocol_snapshot_sha256": sha256_file(UPSTREAM_TABLE4 / "protocol_document_at_freeze.md"),
                "n_eval": len(jobs),
                "j_eval": sum(int(job["expected_movable_dof"]) for job in jobs),
                "category_info": category_info,
                "backend": "cuda",
                "publication_status": "NON_PUBLIC_MIXED_BACKEND",
                "resume_policy": "continuation child results are append-only; old prefix is never rerun",
            },
        )
    else:
        existing = read_json(metadata_path)
        if existing.get("old_partial_directory") != str(old_dir):
            raise RuntimeError("continuation output is bound to a different old partial directory")
    continuation = load_continuation_records(outdir)
    seed_probe(outdir, jobs[PROBE_INDEX], continuation)
    write_progress(
        outdir,
        prefix=prefix,
        continuation=continuation,
        jobs=jobs,
        table4_strict=table4_strict,
        started_at=started_at,
    )
    affinity_cpus = sorted(int(cpu) for cpu in os.sched_getaffinity(0))[: lam.CPU_AFFINITY_WIDTH]
    if len(affinity_cpus) != lam.CPU_AFFINITY_WIDTH:
        raise RuntimeError("host does not expose the required four CPU affinity slots")
    affinity = ",".join(str(cpu) for cpu in affinity_cpus)
    launched_this_invocation = 0
    for index in range(PROBE_INDEX + 1, len(jobs)):
        if index in continuation:
            continue
        if args.max_ranks is not None and launched_this_invocation >= args.max_ranks:
            break
        while True:
            device = select_gpu_device(
                candidates=args.gpu_candidates,
                min_free_mib=args.min_free_mib,
            )
            if device is not None:
                break
            print(
                f"[{utc_now()}] waiting for an idle GPU (candidates={args.gpu_candidates}, "
                f"min_free_mib={args.min_free_mib})",
                flush=True,
            )
            time.sleep(args.poll_seconds)
        print(
            f"[{utc_now()}] rank={index + 1}/{len(jobs)} {jobs[index]['dataset_id']} "
            f"GPU{device['index']} free={device['memory_free_mib']}MiB",
            flush=True,
        )
        record = launch_child(
            jobs[index],
            outdir=outdir,
            device=device,
            affinity=affinity,
            timeout_seconds=args.timeout_seconds,
        )
        continuation[index] = record
        launched_this_invocation += 1
        print(
            f"[{utc_now()}] finished rank={index + 1} status={record.get('status')} "
            f"states={record.get('states_executed')}/{record.get('states_intended')} "
            f"load={record.get('load_time_seconds')}",
            flush=True,
        )
        write_progress(
            outdir,
            prefix=prefix,
            continuation=continuation,
            jobs=jobs,
            table4_strict=table4_strict,
            started_at=started_at,
        )
    atomic_jsonl(
        outdir / "continuation_records.jsonl",
        [continuation[index] for index in sorted(continuation)],
    )
    combined = prefix + [continuation[index] for index in sorted(continuation)]
    if len(combined) < len(jobs):
        return 0
    atomic_jsonl(outdir / "combined_asset_records.jsonl", combined)
    atomic_json(
        outdir / "summary.json",
        {
            "schema_version": "table4a-gpu-continuation-summary/v1",
            "status": "COMPLETE",
            "completed_at_utc": utc_now(),
            "n_eval": len(combined),
            "j_eval": sum(int(job["expected_movable_dof"]) for job in jobs),
            "prefix_cpu_records_reused": len(prefix),
            "gpu_records": len(continuation),
            "mixed_backend": True,
            "publication_status": "NON_PUBLIC_MIXED_BACKEND",
            "reason": "The frozen Table 4a protocol is CPU; GPU suffix is retained for exploratory completion only.",
            "metrics": shared.aggregate(combined, table4_strict),
        },
    )
    return 0


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-dir", type=Path, default=OLD_PARTIAL)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--poll-seconds", type=float, default=DEFAULT_POLL_SECONDS)
    parser.add_argument("--min-free-mib", type=int, default=DEFAULT_MIN_FREE_MIB)
    parser.add_argument("--gpu-candidates", type=int, nargs="+", default=list(DEFAULT_GPU_CANDIDATES))
    parser.add_argument("--max-ranks", type=int)
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return child_main(args.job, args.result)
    if (
        args.timeout_seconds <= 0
        or args.poll_seconds <= 0
        or args.min_free_mib <= 0
        or (args.max_ranks is not None and args.max_ranks <= 0)
    ):
        raise SystemExit("timeouts, polling interval, and GPU memory threshold must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
