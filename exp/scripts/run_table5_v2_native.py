#!/usr/bin/env python3
"""Run the native-fallback Table 5 v2 smoke and formal evaluations.

The smoke stage creates a deterministic subset from the frozen prepared
manifest and runs every simulator without contributing to the N=200 results.
The formal stages run one simulator at a time with a fixed worker count and
then aggregate the three receipt sets into Table 5a and Table 5b.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO_ROOT = SCRIPT_PATH.parents[2]
PREPARE_SCRIPT = SCRIPT_DIR / "table5_v2_prepare_r2.py"
RUNTIME_SCRIPT = SCRIPT_DIR / "table5_v2_runtime_r2.py"
GENESIS_RUNTIME_SCRIPT = RUNTIME_SCRIPT
AGGREGATE_SCRIPT = SCRIPT_DIR / "table5_v2_aggregate_r2.py"

SIMULATORS = ("genesis", "pybullet", "mujoco")
PREPARED_SCHEMA = "table5_v2_prepared_manifest_v1"
DEFAULT_PREPARED = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
)
DEFAULT_OUT_ROOT = REPO_ROOT / "exp/runtime/table5_v2_r2_formal_eight_datasets"
DEFAULT_EXECUTABLES = {
    "genesis": "/mnt/zsn/miniconda3/envs/genesis-main/bin/python",
    "pybullet": str(REPO_ROOT / "exp/.venv_low_medium/bin/python"),
    "mujoco": "/mnt/zsn/miniconda3/bin/python",
}


class RunnerError(RuntimeError):
    """Raised when an execution stage cannot safely start."""


def _canonical_sha256(value: Any, *, exclude: Sequence[str] = ()) -> str:
    excluded = set(exclude)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    payload = json.dumps(
        filtered(value),
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RunnerError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise RunnerError(f"JSON root is not an object: {path}")
    return value


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=True, allow_nan=False, sort_keys=True, indent=2)
        + "\n",
        encoding="utf-8",
    )


def _validate_prepared(path: Path, *, require_core200: bool) -> dict[str, Any]:
    manifest = _read_json(path)
    if manifest.get("schema_version") != PREPARED_SCHEMA:
        raise RunnerError(f"prepared manifest schema mismatch: {path}")
    declared_hash = manifest.get("manifest_sha256")
    if declared_hash != _canonical_sha256(manifest, exclude=("manifest_sha256",)):
        raise RunnerError(f"prepared manifest hash mismatch: {path}")
    protocol = manifest.get("protocol")
    if (
        not isinstance(protocol, Mapping)
        or protocol.get("schema_version") != "table5_v2_runtime_protocol_v2"
        or protocol.get("protocol_id") != "table5-v2-readiness-portability-v2"
        or protocol.get("v2_metrics", {}).get("metric_semantics_id")
        != "table5-v2-native-import-passive-stability-r2"
    ):
        raise RunnerError("prepared manifest does not use revision-2 metrics")
    datasets = manifest.get("datasets")
    if not isinstance(datasets, list) or len(datasets) != 8:
        raise RunnerError("prepared manifest must contain eight datasets")
    for dataset in datasets:
        rows = dataset.get("rows") if isinstance(dataset, Mapping) else None
        if not isinstance(rows, list) or not rows:
            raise RunnerError("prepared dataset has no rows")
        if require_core200 and len(rows) != 200:
            raise RunnerError(
                f"formal prepared dataset is not N=200: {dataset.get('dataset_slug')}"
            )
    return manifest


def _smoke_rows(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Select first row plus one opposite physics-readiness row, deterministically."""

    if not rows:
        raise RunnerError("cannot smoke an empty dataset")
    chosen: list[dict[str, Any]] = [deepcopy(dict(rows[0]))]
    first_status = rows[0].get("physics", {}).get("status")
    desired = "blocked" if first_status == "ready" else "ready"
    for row in rows[1:]:
        if row.get("physics", {}).get("status") == desired:
            chosen.append(deepcopy(dict(row)))
            break
    return chosen


def build_smoke_manifest(source_path: Path, output_path: Path) -> dict[str, Any]:
    source = _validate_prepared(source_path, require_core200=True)
    existing = output_path if output_path.is_file() else None
    if existing is not None:
        smoke = _read_json(existing)
        if smoke.get("smoke_source_manifest_sha256") == source["manifest_sha256"]:
            return smoke
        raise RunnerError(
            f"existing smoke manifest is bound to another source: {existing}"
        )

    smoke = deepcopy(source)
    smoke["smoke_source_manifest_sha256"] = source["manifest_sha256"]
    smoke["manifest_kind"] = "table5_v2_smoke_manifest_v1"
    smoke["datasets"] = []
    for dataset in source["datasets"]:
        rows = _smoke_rows(dataset["rows"])
        group = deepcopy(dataset)
        group["rows"] = rows
        group["preparation_summary"] = {
            "row_count": len(rows),
            "selection": "first row plus first opposite static physics status",
        }
        smoke["datasets"].append(group)
    smoke["sample_size"] = 2
    smoke["total_rows"] = sum(len(dataset["rows"]) for dataset in smoke["datasets"])
    smoke.pop("manifest_sha256", None)
    smoke["manifest_sha256"] = _canonical_sha256(smoke, exclude=("manifest_sha256",))
    _write_json(output_path, smoke)
    return smoke


def _csv(value: str) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values or len(values) != len(set(values)):
        raise RunnerError("comma-separated value is empty or duplicated")
    return values


def _run_command(command: Sequence[str]) -> None:
    print("+ " + " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
    if completed.returncode != 0:
        raise RunnerError(f"command failed with exit code {completed.returncode}")


def _runtime_command(
    *,
    prepared: Path,
    output: Path,
    simulator: str,
    workers: int,
    executable: str,
    gpus: Sequence[str],
    datasets: Sequence[str] | None,
) -> list[str]:
    runtime_script = RUNTIME_SCRIPT
    command = [
        sys.executable,
        str(runtime_script),
        "run",
        "--prepared",
        str(prepared),
        "--simulator",
        simulator,
        "--workers",
        str(workers),
        "--executable",
        executable,
        "--out",
        str(output),
    ]
    if simulator == "genesis":
        command.extend(["--gpus", ",".join(gpus)])
    if datasets:
        command.extend(["--datasets", ",".join(datasets)])
    return command


def run_simulators(
    *,
    prepared: Path,
    output_root: Path,
    workers: int,
    gpus: Sequence[str],
    executables: Mapping[str, str],
    require_core200: bool,
    simulators: Sequence[str] = SIMULATORS,
    datasets: Sequence[str] | None = None,
) -> None:
    manifest = _validate_prepared(prepared, require_core200=require_core200)
    if workers < 1:
        raise RunnerError("workers must be positive")
    if "genesis" in simulators and len(gpus) < workers:
        raise RunnerError(
            f"Genesis needs at least {workers} GPU tokens for {workers} workers; got {len(gpus)}"
        )
    unknown = set(simulators) - set(SIMULATORS)
    if not simulators or unknown:
        raise RunnerError(f"unsupported simulator selection: {sorted(unknown)}")
    available_datasets = {
        str(dataset["dataset_slug"]) for dataset in manifest["datasets"]
    }
    unknown_datasets = set(datasets or ()) - available_datasets
    if unknown_datasets:
        raise RunnerError(f"unsupported dataset selection: {sorted(unknown_datasets)}")
    for simulator in simulators:
        # Preserve virtualenv entry points.  Resolving the symlink can turn
        # PyBullet's environment Python into the system interpreter.
        executable = Path(os.path.abspath(os.path.expanduser(executables[simulator])))
        if not executable.is_file():
            raise RunnerError(f"simulator Python is unavailable: {executable}")
        _run_command(
            _runtime_command(
                prepared=prepared,
                output=output_root / simulator,
                simulator=simulator,
                workers=workers,
                executable=str(executable),
                gpus=gpus,
                datasets=datasets,
            )
        )


def write_smoke_report(root: Path, manifest: Mapping[str, Any]) -> None:
    report: dict[str, Any] = {
        "manifest": str((root / "prepared/manifest.json").resolve(strict=False)),
        "manifest_sha256": manifest["manifest_sha256"],
        "asset_count": manifest["total_rows"],
        "simulators": {},
    }
    for simulator in SIMULATORS:
        summary_path = root / simulator / "summary.json"
        report["simulators"][simulator] = (
            _read_json(summary_path) if summary_path.is_file() else {"missing": True}
        )
    _write_json(root / "smoke_report.json", report)
    print(json.dumps(report, ensure_ascii=True, sort_keys=True), flush=True)


def aggregate(*, prepared: Path, output_root: Path, final_root: Path) -> None:
    _validate_prepared(prepared, require_core200=True)
    _run_command(
        [
            sys.executable,
            str(AGGREGATE_SCRIPT),
            "--prepared",
            str(prepared),
            "--genesis",
            str(output_root / "genesis"),
            "--pybullet",
            str(output_root / "pybullet"),
            "--mujoco",
            str(output_root / "mujoco"),
            "--out",
            str(final_root),
        ]
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage", choices=("smoke", "run", "aggregate", "all"), default="smoke"
    )
    parser.add_argument("--prepared", type=Path, default=DEFAULT_PREPARED)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--gpus", default="0,1,2,3,4")
    parser.add_argument("--simulators", default=",".join(SIMULATORS))
    parser.add_argument(
        "--datasets",
        help="optional comma-separated dataset slugs for an incremental run",
    )
    parser.add_argument("--genesis-python", default=DEFAULT_EXECUTABLES["genesis"])
    parser.add_argument("--pybullet-python", default=DEFAULT_EXECUTABLES["pybullet"])
    parser.add_argument("--mujoco-python", default=DEFAULT_EXECUTABLES["mujoco"])
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    prepared = args.prepared.resolve(strict=False)
    output_root = args.out_root.resolve(strict=False)
    gpus = _csv(args.gpus)
    simulators = _csv(args.simulators)
    datasets = _csv(args.datasets) if args.datasets else None
    executables = {
        "genesis": args.genesis_python,
        "pybullet": args.pybullet_python,
        "mujoco": args.mujoco_python,
    }
    if args.stage == "smoke":
        smoke_root = output_root / "smoke"
        smoke_manifest = build_smoke_manifest(
            prepared, smoke_root / "prepared/manifest.json"
        )
        run_simulators(
            prepared=smoke_root / "prepared/manifest.json",
            output_root=smoke_root,
            workers=args.workers,
            gpus=gpus,
            executables=executables,
            require_core200=False,
            simulators=simulators,
            datasets=datasets,
        )
        write_smoke_report(smoke_root, smoke_manifest)
        return 0
    if args.stage in {"run", "all"}:
        run_simulators(
            prepared=prepared,
            output_root=output_root,
            workers=args.workers,
            gpus=gpus,
            executables=executables,
            require_core200=True,
            simulators=simulators,
            datasets=datasets,
        )
    if args.stage in {"aggregate", "all"}:
        aggregate(
            prepared=prepared,
            output_root=output_root,
            final_root=output_root / "final",
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RunnerError as error:
        print(f"run_table5_v2_native: {error}", file=sys.stderr)
        raise SystemExit(2)
