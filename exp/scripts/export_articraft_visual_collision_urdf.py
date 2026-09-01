#!/usr/bin/env python3
"""Export every Articraft data record as a full visual+collision URDF package.

Outputs use Articraft's standard ignored cache layout:

    <data-root>/cache/record_materialization/<record-id>/

The exporter is resumable.  It accepts an existing package only when the
official compile report says full+full and the URDF and mesh dependencies pass
structural checks.  Valid packages from earlier official runs are hard-linked
when possible; all remaining records are compiled with the official harness.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from dataclasses import dataclass
from datetime import UTC, datetime
import errno
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from typing import Any, Sequence
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
EXP_ROOT = SCRIPT_PATH.parent.parent
DEFAULT_DATA_ROOT = EXP_ROOT / "Articraft-10K-github"
HARNESS_ROOT = EXP_ROOT / "baselines/Articraft-paper-harness"
HARNESS_PYTHON = HARNESS_ROOT / ".venv/bin/python"
COMPILE_SCRIPT = HARNESS_ROOT / "cli/compile_record.py"
COMPAT_SCRIPT = SCRIPT_PATH.with_name("articraft_compile_compat.py")
DEFAULT_REUSE_ROOTS = (
    EXP_ROOT / "runtime/articraft_github_merged_10787_20260827/packages",
    EXP_ROOT
    / "runtime/table5_v2_articraft_github_core200_visual_collision_official"
    / "staging/data/cache/record_materialization",
)
DEFAULT_LEGACY_ROOT = EXP_ROOT / "Articraft-10K/released_urdf"


@dataclass(frozen=True)
class PackageCheck:
    valid: bool
    reason: str
    visual_count: int = 0
    collision_count: int = 0
    validation_level: str | None = None


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def _mesh_path(package: Path, filename: str) -> Path:
    if filename.startswith("file://"):
        return Path(filename.removeprefix("file://"))
    candidate = Path(filename)
    return candidate if candidate.is_absolute() else package / candidate


def check_package(
    package: Path, record_id: str, *, require_validation: bool = False
) -> PackageCheck:
    report_path = package / "compile_report.json"
    urdf_path = package / "model.urdf"
    if not report_path.is_file() or not urdf_path.is_file():
        return PackageCheck(False, "missing compile_report.json or model.urdf")
    try:
        report = _load_json(report_path)
        metrics = report.get("metrics")
        if not isinstance(metrics, dict):
            return PackageCheck(False, "compile report has no metrics object")
        if report.get("status") != "success":
            return PackageCheck(False, "compile report status is not success")
        if report.get("record_id") != record_id:
            return PackageCheck(False, "compile report record_id mismatch")
        if metrics.get("compile_level") != "full":
            return PackageCheck(False, "compile level is not full")
        validation_level = str(metrics.get("validation_level") or "none")
        if require_validation and validation_level != "full":
            return PackageCheck(False, "validation level is not full")

        root = ET.parse(urdf_path).getroot()
        if root.tag != "robot":
            return PackageCheck(False, "URDF root is not <robot>")
        visual_count = 0
        collision_count = 0
        for link in root.findall("link"):
            visuals = link.findall("visual")
            collisions = link.findall("collision")
            visual_count += len(visuals)
            collision_count += len(collisions)
            if visuals and not collisions:
                return PackageCheck(
                    False,
                    f"visual-bearing link {link.get('name')!r} has no collision",
                    visual_count,
                    collision_count,
                )
        if visual_count == 0 or collision_count == 0:
            return PackageCheck(
                False,
                "URDF does not contain both visual and collision geometry",
                visual_count,
                collision_count,
            )
        for mesh in root.findall(".//mesh"):
            filename = mesh.get("filename")
            if not filename:
                return PackageCheck(False, "mesh element has no filename")
            if not _mesh_path(package, filename).is_file():
                return PackageCheck(False, f"missing mesh dependency: {filename}")
        return PackageCheck(
            True,
            "ok",
            visual_count,
            collision_count,
            validation_level,
        )
    except (ET.ParseError, OSError, UnicodeError, ValueError, json.JSONDecodeError) as error:
        return PackageCheck(False, f"{type(error).__name__}: {error}")


def _hardlink_or_copy(source: str, destination: str) -> str:
    try:
        os.link(source, destination)
    except OSError as error:
        if error.errno not in {errno.EXDEV, errno.EPERM, errno.EACCES}:
            raise
        shutil.copy2(source, destination)
    return destination


def reuse_package(source: Path, destination: Path, *, replace: bool = False) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.reuse.", dir=destination.parent)
    )
    try:
        for child in source.iterdir():
            if child.name == "materialization_failure.json":
                continue
            target = temporary / child.name
            if child.is_dir():
                shutil.copytree(child, target, copy_function=_hardlink_or_copy)
            elif child.is_file():
                _hardlink_or_copy(str(child), str(target))
        backup: Path | None = None
        if destination.exists():
            if not replace:
                raise FileExistsError(destination)
            backup = Path(
                tempfile.mkdtemp(
                    prefix=f".{destination.name}.replaced.", dir=destination.parent
                )
            )
            backup.rmdir()
            os.replace(destination, backup)
        try:
            os.replace(temporary, destination)
        except Exception:
            if backup is not None and backup.exists() and not destination.exists():
                os.replace(backup, destination)
            raise
        if backup is not None:
            shutil.rmtree(backup)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, ensure_ascii=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    os.close(fd)
    temporary = Path(name)
    try:
        tree = ET.ElementTree(root)
        ET.indent(tree, space="  ")
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def promote_legacy_visual_package(
    source: Path, destination: Path, record_id: str
) -> PackageCheck:
    reuse_package(source, destination, replace=True)
    urdf_path = destination / "model.urdf"
    root = ET.parse(urdf_path).getroot()
    promoted = 0
    for link in root.findall("link"):
        if link.findall("collision"):
            continue
        for visual in link.findall("visual"):
            collision = deepcopy(visual)
            collision.tag = "collision"
            for material in collision.findall("material"):
                collision.remove(material)
            link.append(collision)
            promoted += 1
    atomic_xml(urdf_path, root)
    visual_count = sum(len(link.findall("visual")) for link in root.findall("link"))
    collision_count = sum(len(link.findall("collision")) for link in root.findall("link"))
    atomic_json(
        destination / "compile_report.json",
        {
            "schema_version": 1,
            "record_id": record_id,
            "status": "success",
            "checks_run": [
                "urdf_xml_parse",
                "visual_collision_link_coverage",
                "mesh_dependency_resolution",
            ],
            "metrics": {
                "compile_level": "full",
                "validation_level": "legacy_visual_collision_promotion",
                "visual_count": visual_count,
                "collision_count": collision_count,
                "promoted_collision_count": promoted,
            },
            "export_policy": (
                "Frozen source no longer compiles with the current CadQuery runtime; "
                "each visual geometry and origin was copied to collision without material."
            ),
            "legacy_source_package": str(source),
        },
    )
    return check_package(destination, record_id)


def _run_compile(
    *,
    record_id: str,
    data_root: Path,
    script: Path,
    timeout_seconds: float,
    validate: bool,
) -> tuple[int | None, str, str, str | None]:
    command = [
        str(HARNESS_PYTHON),
        str(script),
        record_id,
        "--repo-root",
        str(HARNESS_ROOT),
        "--data-dir",
        str(data_root),
        "--target",
        "full",
    ]
    if validate:
        command.append("--validate")
    environment = os.environ.copy()
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    try:
        completed = subprocess.run(
            command,
            cwd=HARNESS_ROOT,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_seconds,
            check=False,
        )
        return completed.returncode, completed.stdout, completed.stderr, None
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else (error.stdout or "")
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else (error.stderr or "")
        return None, stdout, stderr, f"compile timed out after {timeout_seconds:g}s"


def _write_log(
    path: Path,
    *,
    label: str,
    returncode: int | None,
    stdout: str,
    stderr: str,
    error: str | None,
) -> None:
    sections = [f"phase={label}", f"returncode={returncode}"]
    if error:
        sections.append(f"error={error}")
    sections.extend(("\n[stdout]", stdout, "\n[stderr]", stderr))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(sections), encoding="utf-8")


def export_one(
    *,
    record_id: str,
    data_root: Path,
    reuse_roots: Sequence[Path],
    legacy_root: Path | None,
    state_root: Path,
    timeout_seconds: float,
) -> dict[str, Any]:
    started = time.perf_counter()
    destination = data_root / "cache/record_materialization" / record_id
    existing = check_package(destination, record_id)
    if existing.valid:
        return {
            "record_id": record_id,
            "status": "success",
            "origin": "resume",
            "visual_count": existing.visual_count,
            "collision_count": existing.collision_count,
            "validation_level": existing.validation_level,
            "elapsed_seconds": time.perf_counter() - started,
        }

    if not destination.exists():
        for root in reuse_roots:
            candidate = root / record_id
            check = check_package(candidate, record_id, require_validation=True)
            if not check.valid:
                continue
            reuse_package(candidate, destination)
            copied = check_package(destination, record_id, require_validation=True)
            if not copied.valid:
                raise RuntimeError(f"reused package failed destination check: {copied.reason}")
            return {
                "record_id": record_id,
                "status": "success",
                "origin": "reused",
                "reuse_source": str(candidate),
                "visual_count": copied.visual_count,
                "collision_count": copied.collision_count,
                "validation_level": copied.validation_level,
                "elapsed_seconds": time.perf_counter() - started,
            }

    attempts: list[dict[str, Any]] = []
    compile_phases = (
        ("official", COMPILE_SCRIPT, True),
        ("compatibility", COMPAT_SCRIPT, True),
        ("official_unvalidated", COMPILE_SCRIPT, False),
    )
    for label, script, validate in compile_phases:
        returncode, stdout, stderr, error = _run_compile(
            record_id=record_id,
            data_root=data_root,
            script=script,
            timeout_seconds=timeout_seconds,
            validate=validate,
        )
        _write_log(
            state_root / "logs" / f"{record_id}.{label}.log",
            label=label,
            returncode=returncode,
            stdout=stdout,
            stderr=stderr,
            error=error,
        )
        check = check_package(destination, record_id, require_validation=validate)
        attempts.append(
            {
                "phase": label,
                "returncode": returncode,
                "error": error,
                "package_check": check.reason,
            }
        )
        if check.valid:
            return {
                "record_id": record_id,
                "status": "success",
                "origin": label,
                "visual_count": check.visual_count,
                "collision_count": check.collision_count,
                "validation_level": check.validation_level,
                "elapsed_seconds": time.perf_counter() - started,
                "attempts": attempts,
            }
    if legacy_root is not None:
        legacy = legacy_root / record_id
        legacy_full = check_package(legacy, record_id)
        if legacy_full.valid:
            reuse_package(legacy, destination, replace=True)
            restored = check_package(destination, record_id)
            if restored.valid:
                return {
                    "record_id": record_id,
                    "status": "success",
                    "origin": "legacy_full_unvalidated",
                    "visual_count": restored.visual_count,
                    "collision_count": restored.collision_count,
                    "validation_level": restored.validation_level,
                    "elapsed_seconds": time.perf_counter() - started,
                    "attempts": attempts,
                }
        elif (legacy / "model.urdf").is_file():
            try:
                promoted = promote_legacy_visual_package(legacy, destination, record_id)
            except (ET.ParseError, OSError, ValueError) as error:
                attempts.append(
                    {
                        "phase": "legacy_visual_collision_promotion",
                        "error": f"{type(error).__name__}: {error}",
                        "package_check": "promotion raised an exception",
                    }
                )
            else:
                if promoted.valid:
                    return {
                        "record_id": record_id,
                        "status": "success",
                        "origin": "legacy_visual_collision_promotion",
                        "visual_count": promoted.visual_count,
                        "collision_count": promoted.collision_count,
                        "validation_level": promoted.validation_level,
                        "elapsed_seconds": time.perf_counter() - started,
                        "attempts": attempts,
                    }
    return {
        "record_id": record_id,
        "status": "failed",
        "origin": "compile",
        "reason": attempts[-1]["package_check"],
        "elapsed_seconds": time.perf_counter() - started,
        "attempts": attempts,
    }


def _record_ids(data_root: Path) -> list[str]:
    records_root = data_root / "records"
    if not records_root.is_dir():
        raise ValueError(f"records directory is missing: {records_root}")
    return sorted(
        path.name
        for path in records_root.iterdir()
        if path.is_dir() and (path / "record.json").is_file()
    )


def _summary(
    *, data_root: Path, records: Sequence[dict[str, Any]], expected: int, started: float
) -> dict[str, Any]:
    success = sum(row.get("status") == "success" for row in records)
    origins: dict[str, int] = {}
    for row in records:
        origin = str(row.get("origin") or "unknown")
        origins[origin] = origins.get(origin, 0) + 1
    return {
        "schema_version": "articraft_visual_collision_export_v1",
        "updated_at": datetime.now(UTC).isoformat(),
        "data_root": str(data_root),
        "output_root": str(data_root / "cache/record_materialization"),
        "expected": expected,
        "completed": len(records),
        "success": success,
        "failed": len(records) - success,
        "origins": dict(sorted(origins.items())),
        "elapsed_seconds": time.perf_counter() - started,
        "records": sorted(records, key=lambda row: str(row["record_id"])),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--legacy-root", type=Path, default=DEFAULT_LEGACY_ROOT)
    parser.add_argument(
        "--reuse-root",
        action="append",
        type=Path,
        dest="reuse_roots",
        help="Existing full package root to reuse; repeatable (defaults to known official runs).",
    )
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("--workers must be positive")
    if args.timeout_seconds <= 0:
        parser.error("--timeout-seconds must be positive")
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")
    for required in (HARNESS_PYTHON, COMPILE_SCRIPT, COMPAT_SCRIPT):
        if not required.is_file():
            parser.error(f"required compiler component is missing: {required}")

    data_root = args.data_root.resolve(strict=True)
    record_ids = _record_ids(data_root)
    if args.limit is not None:
        record_ids = record_ids[: args.limit]
    reuse_roots = tuple(
        path.resolve(strict=False)
        for path in (args.reuse_roots if args.reuse_roots is not None else DEFAULT_REUSE_ROOTS)
        if path.is_dir()
    )
    legacy_root = args.legacy_root.resolve(strict=False) if args.legacy_root.is_dir() else None
    state_root = data_root / "local/visual_collision_export"
    status_path = state_root / "status.json"
    started = time.perf_counter()
    results: list[dict[str, Any]] = []
    print(
        f"Exporting {len(record_ids)} records with {args.workers} workers; "
        f"output={data_root / 'cache/record_materialization'}",
        flush=True,
    )
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                export_one,
                record_id=record_id,
                data_root=data_root,
                reuse_roots=reuse_roots,
                legacy_root=legacy_root,
                state_root=state_root,
                timeout_seconds=args.timeout_seconds,
            ): record_id
            for record_id in record_ids
        }
        for future in as_completed(futures):
            record_id = futures[future]
            try:
                result = future.result()
            except Exception as error:
                result = {
                    "record_id": record_id,
                    "status": "failed",
                    "origin": "exception",
                    "reason": f"{type(error).__name__}: {error}",
                }
            results.append(result)
            if len(results) % 25 == 0 or result["status"] != "success":
                success = sum(row["status"] == "success" for row in results)
                print(
                    f"progress={len(results)}/{len(record_ids)} success={success} "
                    f"failed={len(results) - success}",
                    flush=True,
                )
            if len(results) % 100 == 0:
                atomic_json(
                    status_path,
                    _summary(
                        data_root=data_root,
                        records=results,
                        expected=len(record_ids),
                        started=started,
                    ),
                )

    summary = _summary(
        data_root=data_root,
        records=results,
        expected=len(record_ids),
        started=started,
    )
    atomic_json(status_path, summary)
    print(json.dumps({key: summary[key] for key in ("expected", "success", "failed", "origins", "elapsed_seconds")}, sort_keys=True))
    print(f"status={status_path}")
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
