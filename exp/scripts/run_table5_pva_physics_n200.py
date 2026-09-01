#!/usr/bin/env python3
"""Orchestrate preparation, physics injection, runtime, and aggregation for PV-A."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Sequence


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
MANIFEST_SCRIPT = SCRIPT_PATH.with_name("table5_pva_physics_n200_manifest.py")
RUNTIME_SCRIPT = SCRIPT_PATH.with_name("table5_pva_physics_n200_runtime.py")
AGGREGATE_SCRIPT = SCRIPT_PATH.with_name("table5_n200_aggregate.py")
DEFAULT_RUN_ROOT = EXP_ROOT / "runtime/table5_pva_n200_physics_v2_20260828"
DEFAULT_ROSTER = EXP_ROOT / "runtime/pva_table1234_full_release_20260826/roster/full_release_roster.jsonl"
DEFAULT_EVALUATION_ROOT = EXP_ROOT / "runtime/pva_table1234_full_release_20260826/evaluation"
SAMPLE_SIZE = 200
SIMULATORS = ("pybullet", "genesis", "mujoco")
DEFAULT_WORKERS = {"pybullet": 32, "genesis": 8, "mujoco": 32}
DEFAULT_GPUS = ("0", "1", "2", "3", "4", "5", "6", "7")
MODULE_ENV = {
    "pybullet": "TABLE5_PYBULLET_PYTHON",
    "genesis": "TABLE5_GENESIS_PYTHON",
    "mujoco": "TABLE5_MUJOCO_PYTHON",
}


class OrchestrationError(RuntimeError):
    pass


def _first_executable(environment_name: str, candidates: Sequence[Path]) -> str:
    configured = os.environ.get(environment_name)
    if configured:
        return configured
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return sys.executable


def _default_executables() -> dict[str, str]:
    return {
        "pybullet": _first_executable(MODULE_ENV["pybullet"], (EXP_ROOT / ".venv_low_medium/bin/python",)),
        "genesis": _first_executable(MODULE_ENV["genesis"], (Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python"),)),
        "mujoco": _first_executable(MODULE_ENV["mujoco"], (Path("/mnt/zsn/miniconda3/bin/python"),)),
    }


def _csv(value: str, *, allowed: set[str] | None = None) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values or len(values) != len(set(values)):
        raise OrchestrationError("comma-separated selection is empty or duplicated")
    if allowed is not None and set(values) - allowed:
        raise OrchestrationError(f"unsupported values: {sorted(set(values) - allowed)}")
    return values


def _worker_specs(values: Sequence[str] | None, aliases: dict[str, int | None]) -> list[str]:
    specs = list(values) if values else []
    for simulator, count in aliases.items():
        if count is not None:
            specs.append(f"{simulator}={count}")
    if not specs:
        specs.append(",".join(f"{simulator}={DEFAULT_WORKERS[simulator]}" for simulator in SIMULATORS))
    return specs


def _resolve_executable(value: str, *, validate: bool) -> str:
    located = shutil.which(value)
    candidate = Path(os.path.abspath(located or value)).expanduser()
    if validate and (not candidate.is_file() or not os.access(candidate, os.X_OK)):
        raise OrchestrationError(f"Python executable is unavailable: {value}")
    return str(candidate) if validate else value


def _normalize_executables(values: Sequence[str] | None, defaults: dict[str, str], *, validate: bool) -> list[str]:
    source = [",".join(f"{simulator}={defaults[simulator]}" for simulator in SIMULATORS)]
    source.extend(values or ())
    normalized: list[str] = []
    for value in source:
        assignments: list[str] = []
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            if "=" in item:
                simulator, executable = (part.strip() for part in item.split("=", 1))
                if simulator not in SIMULATORS or not executable:
                    raise OrchestrationError(f"invalid executable assignment: {item!r}")
                assignments.append(f"{simulator}={_resolve_executable(executable, validate=validate)}")
            else:
                assignments.append(_resolve_executable(item, validate=validate))
        if assignments:
            normalized.append(",".join(assignments))
    return normalized


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    print("+ " + " ".join(command), flush=True)
    if not dry_run:
        completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
        if completed.returncode != 0:
            raise OrchestrationError(f"phase command failed with exit code {completed.returncode}")


def _validate_existing_manifest(path: Path) -> None:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise OrchestrationError(f"cannot read existing manifest {path}: {error}") from error
    if not isinstance(manifest, dict) or manifest.get("schema_version") != "table5_pva_physics_n200_manifest_v2":
        raise OrchestrationError(f"existing manifest is not the physics PV-A N=200 manifest: {path}")
    if manifest.get("sample_size") != SAMPLE_SIZE or manifest.get("total_rows") != SAMPLE_SIZE:
        raise OrchestrationError(f"existing manifest has the wrong cohort size: {path}")
    injection = manifest.get("physics_injection")
    if not isinstance(injection, dict) or injection.get("policy_id") != "pva-physics-common-link-overlay-v1":
        raise OrchestrationError(f"existing manifest lacks the physics injection policy: {path}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("all", "prepare", "run", "aggregate"), default="all")
    parser.add_argument("--run-root", "--out", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--sample-size", type=int, default=SAMPLE_SIZE)
    parser.add_argument("--roster", type=Path, default=DEFAULT_ROSTER)
    parser.add_argument("--evaluation-root", type=Path, default=DEFAULT_EVALUATION_ROOT)
    parser.add_argument("--simulators", default=",".join(SIMULATORS))
    parser.add_argument("--workers", action="append")
    parser.add_argument("--executables", action="append")
    parser.add_argument("--gpus", action="append")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--pybullet-workers", type=int)
    parser.add_argument("--genesis-workers", type=int)
    parser.add_argument("--mujoco-workers", type=int)
    parser.add_argument("--pybullet-python")
    parser.add_argument("--genesis-python")
    parser.add_argument("--mujoco-python")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.sample_size != SAMPLE_SIZE:
        raise OrchestrationError("the physics PV-A cohort is fixed at N=200")
    if args.timeout <= 0:
        raise OrchestrationError("--timeout must be positive")
    simulators = _csv(args.simulators, allowed=set(SIMULATORS))
    run_root = args.run_root.resolve(strict=False)
    manifest_path = run_root / "manifest.json"
    runtime_root = run_root / "runtime"
    aggregate_root = run_root / "aggregate"
    if args.stage in {"all", "prepare"}:
        if manifest_path.is_file():
            _validate_existing_manifest(manifest_path)
            print(f"resume: using existing manifest {manifest_path}", flush=True)
        else:
            _run([sys.executable, str(MANIFEST_SCRIPT), "--out", str(manifest_path), "--roster", str(args.roster.resolve(strict=False)), "--evaluation-root", str(args.evaluation_root.resolve(strict=False))], dry_run=args.dry_run)
    if args.stage in {"all", "run"}:
        if not args.dry_run and not manifest_path.is_file():
            raise OrchestrationError(f"manifest is missing: {manifest_path}")
        worker_specs = _worker_specs(args.workers, {"pybullet": args.pybullet_workers, "genesis": args.genesis_workers, "mujoco": args.mujoco_workers})
        executable_values = list(args.executables or [])
        executable_values.extend(f"{simulator}={value}" for simulator, value in (("pybullet", args.pybullet_python), ("genesis", args.genesis_python), ("mujoco", args.mujoco_python)) if value is not None)
        executable_specs = _normalize_executables(executable_values or None, _default_executables(), validate=not args.dry_run)
        gpu_specs = list(args.gpus) if args.gpus else [",".join(DEFAULT_GPUS)]
        command = [sys.executable, str(RUNTIME_SCRIPT), "run", "--manifest", str(manifest_path), "--runtime-root", str(runtime_root), "--simulators", ",".join(simulators), "--timeout", str(args.timeout)]
        for spec in worker_specs:
            command.extend(("--workers", spec))
        for spec in executable_specs:
            command.extend(("--executables", spec))
        for spec in gpu_specs:
            command.extend(("--gpus", spec))
        _run(command, dry_run=args.dry_run)
    if args.stage in {"all", "aggregate"}:
        if not args.dry_run and not manifest_path.is_file():
            raise OrchestrationError(f"manifest is missing: {manifest_path}")
        _run([sys.executable, str(AGGREGATE_SCRIPT), "--manifest", str(manifest_path), "--run-root", str(run_root), "--out", str(aggregate_root)], dry_run=args.dry_run)
    print(json.dumps({"stage": args.stage, "run_root": str(run_root), "manifest": str(manifest_path), "runtime": str(runtime_root), "aggregate": str(aggregate_root), "simulators": simulators}, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except OrchestrationError as error:
        print(f"run_table5_pva_physics_n200: {error}", file=sys.stderr)
        raise SystemExit(2)
