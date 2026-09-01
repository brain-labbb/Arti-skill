#!/usr/bin/env python3
"""One-command orchestration for the six-dataset Table 5 N=200 evaluation."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any, Sequence


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
MANIFEST_SCRIPT = SCRIPT_PATH.with_name("table5_n200_manifest.py")
ARTICRAFT_PARENT_SCRIPT = SCRIPT_PATH.with_name("table5_articraft_github_parent.py")
RUNTIME_SCRIPT = SCRIPT_PATH.with_name("table5_n200_runtime.py")
AGGREGATE_SCRIPT = SCRIPT_PATH.with_name("table5_n200_aggregate.py")
DEFAULT_RUN_ROOT = EXP_ROOT / "runtime/table5_n200_articraft_github_20260827"
DEFAULT_ARTICRAFT_SOURCE_MANIFEST = (
    EXP_ROOT / "Articraft-10K-github/records_manifest.jsonl"
)
DEFAULT_ARTICRAFT_MATERIALIZED_ROOT = EXP_ROOT / "Articraft-10K/released_urdf"
SIMULATORS = ("pybullet", "genesis", "mujoco")
MODULES = {
    "pybullet": "pybullet",
    "genesis": "genesis",
    "mujoco": "mujoco",
}


class OrchestrationError(RuntimeError):
    """Raised before a phase starts when its inputs are unsafe or incomplete."""


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
        "pybullet": _first_executable(
            "TABLE5_PYBULLET_PYTHON",
            (EXP_ROOT / ".venv_low_medium/bin/python",),
        ),
        "genesis": _first_executable(
            "TABLE5_GENESIS_PYTHON",
            (Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python"),),
        ),
        "mujoco": _first_executable(
            "TABLE5_MUJOCO_PYTHON",
            (Path("/mnt/zsn/miniconda3/bin/python"),),
        ),
    }


def _csv(value: str, *, allowed: set[str] | None = None) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values or len(values) != len(set(values)):
        raise OrchestrationError("comma-separated selection is empty or duplicated")
    if allowed is not None and set(values) - allowed:
        raise OrchestrationError(f"unsupported values: {sorted(set(values) - allowed)}")
    return values


def _resolve_executable(value: str) -> str:
    located = shutil.which(value)
    # Preserve a virtualenv's Python entry point. Resolving its symlink to the
    # base interpreter changes sys.prefix and drops the environment packages.
    candidate = Path(os.path.abspath(Path(located or value).expanduser()))
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise OrchestrationError(f"Python executable is unavailable: {value}")
    return str(candidate)


def _check_environment(simulator: str, executable: str) -> dict[str, str]:
    distribution = "genesis-world" if simulator == "genesis" else simulator
    command = [
        executable,
        "-c",
        (
            "import importlib.metadata as m,importlib.util as u,json,sys;"
            f"assert u.find_spec({MODULES[simulator]!r}) is not None;"
            "print(json.dumps({"
            "'python':sys.executable,"
            f"'version':m.version({distribution!r})"
            "},sort_keys=True))"
        ),
    ]
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=60,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip()[-2000:]
        raise OrchestrationError(
            f"{simulator} environment check failed with {executable}: {detail}"
        )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    try:
        receipt = json.loads(lines[-1])
    except (IndexError, json.JSONDecodeError) as error:
        raise OrchestrationError(
            f"{simulator} environment check returned no JSON receipt"
        ) from error
    return {"python": str(receipt["python"]), "version": str(receipt["version"])}


def _gpu_inventory() -> list[dict[str, Any]]:
    command = [
        "nvidia-smi",
        "--query-gpu=index,uuid,name,memory.total,memory.free,utilization.gpu",
        "--format=csv,noheader,nounits",
    ]
    try:
        completed = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise OrchestrationError(f"cannot query Genesis GPUs: {error}") from error
    if completed.returncode != 0:
        raise OrchestrationError(
            "nvidia-smi failed: " + completed.stderr.strip()[-2000:]
        )
    rows: list[dict[str, Any]] = []
    for line in completed.stdout.splitlines():
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 6:
            raise OrchestrationError(f"malformed nvidia-smi row: {line!r}")
        index, gpu_uuid, name, total, free, utilization = parts
        rows.append(
            {
                "index": index,
                "uuid": gpu_uuid,
                "name": name,
                "memory_total_mib": int(total),
                "memory_free_mib": int(free),
                "utilization_percent": int(utilization),
            }
        )
    if not rows:
        raise OrchestrationError("nvidia-smi reported no GPUs")
    return rows


def _check_gpus(
    requested: Sequence[str],
    *,
    maximum_utilization: int,
    minimum_free_mib: int,
    allow_busy: bool,
) -> list[dict[str, Any]]:
    inventory = _gpu_inventory()
    selected: list[dict[str, Any]] = []
    for token in requested:
        matches = [row for row in inventory if token in {row["index"], row["uuid"]}]
        if len(matches) != 1:
            raise OrchestrationError(
                f"Genesis GPU token is unknown or ambiguous: {token}"
            )
        selected.append(matches[0])
    if len({row["uuid"] for row in selected}) != len(selected):
        raise OrchestrationError("Genesis GPU selection contains duplicates")
    busy = [
        row
        for row in selected
        if row["utilization_percent"] > maximum_utilization
        or row["memory_free_mib"] < minimum_free_mib
    ]
    if busy and not allow_busy:
        descriptions = ", ".join(
            f"GPU{row['index']} util={row['utilization_percent']}% "
            f"free={row['memory_free_mib']}MiB"
            for row in busy
        )
        raise OrchestrationError(
            "Genesis GPU gate rejected busy devices: "
            + descriptions
            + "; wait for capacity or pass --allow-busy-gpus explicitly"
        )
    return selected


def _run(command: Sequence[str], *, dry_run: bool) -> None:
    print("+ " + " ".join(command), flush=True)
    if dry_run:
        return
    completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
    if completed.returncode != 0:
        raise OrchestrationError(
            f"phase command failed with exit code {completed.returncode}"
        )


def _validate_existing_manifest(path: Path, sample_size: int) -> None:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise OrchestrationError(f"cannot resume manifest {path}: {error}") from error
    if (
        not isinstance(manifest, dict)
        or manifest.get("sample_size") != sample_size
        or manifest.get("dataset_count") != 6
        or manifest.get("total_rows") != 6 * sample_size
    ):
        raise OrchestrationError(
            f"existing manifest does not match six groups x {sample_size}: {path}"
        )


def _parser() -> argparse.ArgumentParser:
    defaults = _default_executables()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage",
        choices=("all", "prepare", "run", "aggregate"),
        default="all",
    )
    parser.add_argument("--run-root", "--out", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--sample-size", type=int, default=200)
    parser.add_argument(
        "--articraft-source-manifest",
        type=Path,
        default=DEFAULT_ARTICRAFT_SOURCE_MANIFEST,
    )
    parser.add_argument(
        "--articraft-materialized-root",
        type=Path,
        default=DEFAULT_ARTICRAFT_MATERIALIZED_ROOT,
    )
    parser.add_argument(
        "--datasets",
        help="comma-separated manifest dataset slugs; default: all six",
    )
    parser.add_argument(
        "--simulators",
        default=",".join(SIMULATORS),
        help="comma-separated pybullet,genesis,mujoco",
    )
    parser.add_argument("--pybullet-python", default=defaults["pybullet"])
    parser.add_argument("--genesis-python", default=defaults["genesis"])
    parser.add_argument("--mujoco-python", default=defaults["mujoco"])
    parser.add_argument("--pybullet-workers", type=int, default=8)
    parser.add_argument("--mujoco-workers", type=int, default=8)
    parser.add_argument("--genesis-workers", type=int, default=1)
    parser.add_argument("--gpus", default="0")
    parser.add_argument("--genesis-max-utilization", type=int, default=20)
    parser.add_argument("--genesis-min-free-mib", type=int, default=32768)
    parser.add_argument("--allow-busy-gpus", action="store_true")
    parser.add_argument("--skip-environment-check", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.sample_size <= 0:
        raise OrchestrationError("sample size must be positive")
    workers = {
        "pybullet": args.pybullet_workers,
        "genesis": args.genesis_workers,
        "mujoco": args.mujoco_workers,
    }
    if any(value <= 0 for value in workers.values()):
        raise OrchestrationError("worker counts must be positive")
    simulators = _csv(args.simulators, allowed=set(SIMULATORS))
    datasets = _csv(args.datasets) if args.datasets else None
    gpu_tokens = _csv(args.gpus)
    run_root = args.run_root.resolve(strict=False)
    articraft_parent_path = run_root / "articraft_github_parent.json"
    manifest_path = run_root / "manifest.json"
    runtime_root = run_root / "runtime"
    aggregate_root = run_root / "aggregate"

    if args.stage in {"all", "prepare"}:
        if manifest_path.exists():
            _validate_existing_manifest(manifest_path, args.sample_size)
            print(f"resume: using existing manifest {manifest_path}", flush=True)
        else:
            if articraft_parent_path.exists():
                print(
                    f"resume: using existing Articraft parent {articraft_parent_path}",
                    flush=True,
                )
            else:
                _run(
                    [
                        sys.executable,
                        str(ARTICRAFT_PARENT_SCRIPT),
                        "--out",
                        str(articraft_parent_path),
                        "--sample-size",
                        str(args.sample_size),
                        "--source",
                        str(args.articraft_source_manifest.resolve(strict=False)),
                        "--materialized-root",
                        str(args.articraft_materialized_root.resolve(strict=False)),
                    ],
                    dry_run=args.dry_run,
                )
            _run(
                [
                    sys.executable,
                    str(MANIFEST_SCRIPT),
                    "--out",
                    str(manifest_path),
                    "--sample-size",
                    str(args.sample_size),
                    "--articraft-parent",
                    str(articraft_parent_path),
                    "--articraft-category-records-root",
                    str(
                        args.articraft_source_manifest.resolve(strict=False).parent
                        / "records"
                    ),
                ],
                dry_run=args.dry_run,
            )

    if args.stage in {"all", "run"}:
        if not args.dry_run and not manifest_path.is_file():
            raise OrchestrationError(f"manifest is missing: {manifest_path}")
        executables = {
            "pybullet": _resolve_executable(args.pybullet_python),
            "genesis": _resolve_executable(args.genesis_python),
            "mujoco": _resolve_executable(args.mujoco_python),
        }
        environment_receipts = {}
        if not args.skip_environment_check and not args.dry_run:
            environment_receipts = {
                simulator: _check_environment(simulator, executables[simulator])
                for simulator in simulators
            }
            print(json.dumps({"environments": environment_receipts}, sort_keys=True))
        if "genesis" in simulators and not args.dry_run:
            gpu_receipt = _check_gpus(
                gpu_tokens,
                maximum_utilization=args.genesis_max_utilization,
                minimum_free_mib=args.genesis_min_free_mib,
                allow_busy=args.allow_busy_gpus,
            )
            print(json.dumps({"genesis_gpus": gpu_receipt}, sort_keys=True))
        command = [
            sys.executable,
            str(RUNTIME_SCRIPT),
            "run",
            "--manifest",
            str(manifest_path),
            "--runtime-root",
            str(runtime_root),
            "--simulators",
            ",".join(simulators),
            "--workers",
            ",".join(f"{name}={count}" for name, count in workers.items()),
        ]
        if datasets:
            command.extend(("--datasets", ",".join(datasets)))
        for simulator in SIMULATORS:
            command.extend(("--executables", f"{simulator}={executables[simulator]}"))
        if "genesis" in simulators:
            command.extend(("--gpus", ",".join(gpu_tokens)))
        _run(command, dry_run=args.dry_run)

    if args.stage in {"all", "aggregate"}:
        if not args.dry_run and not manifest_path.is_file():
            raise OrchestrationError(f"manifest is missing: {manifest_path}")
        _run(
            [
                sys.executable,
                str(AGGREGATE_SCRIPT),
                "--manifest",
                str(manifest_path),
                "--run-root",
                str(run_root),
                "--out",
                str(aggregate_root),
            ],
            dry_run=args.dry_run,
        )

    print(
        json.dumps(
            {
                "run_root": str(run_root),
                "manifest": str(manifest_path),
                "articraft_parent": str(articraft_parent_path),
                "runtime": str(runtime_root),
                "aggregate": str(aggregate_root),
                "stage": args.stage,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except OrchestrationError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
