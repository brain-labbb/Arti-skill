#!/usr/bin/env python3
"""Export all 10,787 Articraft-10K-github records as visual+collision URDFs.

Every record is compiled from its Articraft-10K-github source with the
official Articraft compiler in full mode (visual+collision) WITHOUT the
--validate gate.  The run is resume-safe and writes a per-record census used
by the diagnostic Table 5 resample to stratify assets by export quality.
Records that cannot be compiled receive a placeholder URDF and are counted as
export failures.
"""

from __future__ import annotations

import argparse
import json
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import random
import subprocess
import tempfile
import threading
import time
from typing import Any, Sequence
import xml.etree.ElementTree as ET

SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"

MERGED_ROSTER = (
    EXP_ROOT
    / "runtime/articraft_github_merged_10787_20260827/rosters/merged/full_release_roster.jsonl"
)
SOURCE_ROOT = EXP_ROOT / "Articraft-10K-github"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/articraft_github_full10787_export_noverify_20260830"

COMPILE_HARNESS = EXP_ROOT / "baselines/Articraft-paper-harness"
COMPILE_PYTHON = COMPILE_HARNESS / ".venv/bin/python"
COMPILE_SCRIPT = COMPILE_HARNESS / "cli/compile_record.py"
COMPAT_SCRIPT = SCRIPT_PATH.with_name("articraft_compile_compat.py")
COMPILE_TIMEOUT_SECONDS = 900.0


class ExportError(RuntimeError):
    """Raised when the full-population export cannot proceed."""


def atomic_text(path: Path, value: str) -> None:
    path = path.resolve(strict=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(value)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ExportError(f"JSON root is not an object: {path}")
    return value


def package_is_full_visual_collision(package: Path, asset_id: str) -> bool:
    urdf = package / "model.urdf"
    report = package / "compile_report.json"
    if not urdf.is_file() or urdf.is_symlink() or not report.is_file():
        return False
    try:
        payload = load_json(report)
        metrics = payload.get("metrics")
        if not isinstance(metrics, dict):
            return False
        if not (
            payload.get("status") == "success"
            and payload.get("record_id") == asset_id
            and metrics.get("compile_level") == "full"
        ):
            return False
        root = ET.parse(urdf).getroot()
        visual = sum(len(link.findall("visual")) for link in root.findall("link"))
        collision = sum(len(link.findall("collision")) for link in root.findall("link"))
        return visual > 0 and collision > 0
    except (OSError, ET.ParseError, ValueError):
        return False


def compile_one(
    *,
    asset_id: str,
    staging_data: Path,
    log_root: Path,
    timeout_seconds: float,
) -> dict[str, Any]:
    package = staging_data / "cache/record_materialization" / asset_id
    if package_is_full_visual_collision(package, asset_id):
        return {"record_id": asset_id, "status": "success", "origin": "resume"}
    source_dir = SOURCE_ROOT / "records" / asset_id
    if not source_dir.is_dir():
        return {
            "record_id": asset_id,
            "status": "failed",
            "origin": "executed",
            "reason": "github record directory missing",
        }
    source = source_dir.resolve(strict=True)
    records = staging_data / "records"
    records.mkdir(parents=True, exist_ok=True)
    link = records / asset_id
    if link.exists() or link.is_symlink():
        if not link.is_symlink() or link.resolve(strict=True) != source:
            return {
                "record_id": asset_id,
                "status": "failed",
                "origin": "executed",
                "reason": "staging source binding mismatch",
            }
    else:
        link.symlink_to(source, target_is_directory=True)
    base_command = [
        "--repo-root",
        str(COMPILE_HARNESS),
        "--data-dir",
        str(staging_data),
        "--target",
        "full",
    ]
    environment = os.environ.copy()
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    last_reason = "unknown"
    for attempt in range(3):
        if attempt:
            time.sleep(2.0 * attempt + random.uniform(0.0, 3.0))
        for origin, script in (
            ("executed", COMPILE_SCRIPT),
            ("compatibility_fallback", COMPAT_SCRIPT),
        ):
            command = [str(COMPILE_PYTHON), str(script), asset_id, *base_command]
            suffix = f".try{attempt}" if attempt else ""
            suffix += "" if origin == "executed" else ".fallback"
            try:
                completed = subprocess.run(
                    command,
                    cwd=COMPILE_HARNESS,
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=timeout_seconds,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                atomic_text(log_root / f"{asset_id}{suffix}.timeout.txt", "timeout")
                last_reason = f"timeout after {timeout_seconds:.0f}s"
                continue
            except (BlockingIOError, OSError) as error:
                last_reason = f"{type(error).__name__}: {error}"
                continue
            atomic_text(log_root / f"{asset_id}{suffix}.stdout.txt", completed.stdout)
            atomic_text(log_root / f"{asset_id}{suffix}.stderr.txt", completed.stderr)
            if package_is_full_visual_collision(package, asset_id):
                return {
                    "record_id": asset_id,
                    "status": "success",
                    "origin": origin if attempt == 0 else f"{origin}_retry{attempt}",
                    "returncode": completed.returncode,
                }
            last_reason = (
                completed.stderr.strip().splitlines()[-1]
                if completed.stderr.strip()
                else f"returncode={completed.returncode}"
            )
    return {
        "record_id": asset_id,
        "status": "failed",
        "origin": "executed",
        "reason": last_reason,
    }


def write_placeholder(package: Path, asset_id: str) -> None:
    package.mkdir(parents=True, exist_ok=True)
    atomic_text(
        package / "model.urdf",
        f"ARTICRAFT_TABLE5_COMPILE_FAILED record_id={asset_id}\n",
    )
    if not (package / "compile_report.json").is_file():
        atomic_text(
            package / "compile_report.json",
            json.dumps(
                {
                    "schema_version": 1,
                    "record_id": asset_id,
                    "status": "failure",
                    "checks_run": ["compile_urdf"],
                    "metrics": {"compile_level": "full"},
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=48)
    parser.add_argument("--limit", type=int, default=0)
    arguments = parser.parse_args(argv)
    output = arguments.out.resolve(strict=False)
    staging_data = output / "staging/data"
    staging_data.mkdir(parents=True, exist_ok=True)
    log_root = output / "compile_logs"
    log_root.mkdir(parents=True, exist_ok=True)
    asset_ids: list[str] = []
    with MERGED_ROSTER.open("r", encoding="utf-8") as handle:
        for line in handle:
            asset_ids.append(str(json.loads(line)["asset_id"]))
    if len(asset_ids) != 10_787:
        raise SystemExit(f"roster must contain 10,787 rows; got {len(asset_ids)}")
    if arguments.limit > 0:
        asset_ids = asset_ids[: arguments.limit]
    ledger_path = output / "export_ledger.jsonl"
    ledger_lock = threading.Lock()
    ledger_handle = ledger_path.open("a", encoding="utf-8")
    done = {"success": 0, "failed": 0}
    futures_map = {}
    with ThreadPoolExecutor(max_workers=arguments.workers) as executor:
        for asset_id in asset_ids:
            futures_map[
                executor.submit(
                    compile_one,
                    asset_id=asset_id,
                    staging_data=staging_data,
                    log_root=log_root,
                    timeout_seconds=COMPILE_TIMEOUT_SECONDS,
                )
            ] = asset_id
        for index, future in enumerate(as_completed(futures_map), start=1):
            asset_id = futures_map[future]
            try:
                record = future.result()
            except Exception as error:
                record = {
                    "record_id": asset_id,
                    "status": "failed",
                    "origin": "executed",
                    "reason": f"{type(error).__name__}: {error}",
                }
            if record["status"] == "success":
                done["success"] += 1
            else:
                done["failed"] += 1
                write_placeholder(
                    staging_data / "cache/record_materialization" / asset_id, asset_id
                )
            with ledger_lock:
                ledger_handle.write(
                    json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n"
                )
                ledger_handle.flush()
            if index % 200 == 0 or index == len(asset_ids):
                print(
                    json.dumps(
                        {
                            "progress": index,
                            "total": len(asset_ids),
                            "success": done["success"],
                            "failed": done["failed"],
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
    ledger_handle.close()
    census: dict[str, Any] = {}
    with ledger_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            census[str(record["record_id"])] = record
    success = sum(
        1
        for asset_id in asset_ids
        if package_is_full_visual_collision(
            staging_data / "cache/record_materialization" / asset_id, asset_id
        )
    )
    atomic_text(
        output / "export_census.json",
        json.dumps(
            {
                "schema_version": "articraft_full10787_export_noverify_census_v1",
                "compile_policy": "official_full_visual_collision_without_validate_gate",
                "source_root": str(SOURCE_ROOT),
                "roster_path": str(MERGED_ROSTER),
                "total": len(asset_ids),
                "success": success,
                "failed": len(asset_ids) - success,
                "records": [census.get(aid, {"record_id": aid, "status": "missing"}) for aid in sorted(asset_ids)],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
    )
    print(
        json.dumps(
            {"stage": "complete", "total": len(asset_ids), "success": success},
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
