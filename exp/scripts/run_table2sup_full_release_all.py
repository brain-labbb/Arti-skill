#!/usr/bin/env python3
"""Run the full-release Table 2 supplementary evaluator for all eight cohorts.

This orchestration layer only calls ``run_table2sup_full_release.py``.  It
never changes the main Table 1/2/3 receipts and writes one combined receipt
after every cohort reaches a terminal state.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Iterable

try:
    from . import run_table2sup_full_release as runner
except ImportError:  # direct script execution
    import run_table2sup_full_release as runner  # type: ignore


SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
DEFAULT_ROSTER_ROOT = EXP_ROOT / "runtime" / "table123_full_release_20260825" / "rosters"
DEFAULT_OUTPUT_ROOT = EXP_ROOT / "runtime" / "table2sup_full_release_20260826"
DEFAULT_INFINIGEN_SOURCE = EXP_ROOT / "Infinigen-Sim"
DEFAULT_PARTS_ZIP = EXP_ROOT / "parts.zip"


def _selected(slugs: Iterable[str] | None) -> list[str]:
    if slugs is None:
        return list(runner.DATASETS)
    requested = [str(value).lower() for value in slugs]
    unknown = sorted(set(requested) - set(runner.DATASETS))
    if unknown:
        raise ValueError(f"unknown dataset slug(s): {', '.join(unknown)}")
    # Preserve the canonical order and reject duplicate work accidentally
    # requested on the command line.
    if len(requested) != len(set(requested)):
        raise ValueError("duplicate dataset slug")
    return [slug for slug in runner.DATASETS if slug in requested]


def _command(
    slug: str,
    *,
    roster_root: Path,
    output_root: Path,
    workers: int,
    timeout_seconds: float,
    resume: bool,
    parts_zip: Path,
    infinigen_source: Path,
) -> list[str]:
    roster = roster_root / slug / "full_release_manifest.json"
    output = output_root / slug
    command = [
        sys.executable,
        str(SCRIPT.with_name("run_table2sup_full_release.py")),
        "--dataset",
        slug,
        "--roster",
        str(roster),
        "--output",
        str(output),
        "--workers",
        str(workers),
        "--timeout-seconds",
        str(timeout_seconds),
    ]
    if resume:
        command.append("--resume")
    if slug == "infinite":
        command.extend(["--parts-zip", str(parts_zip)])
    if slug == "infinigen":
        command.extend(["--source-root", str(infinigen_source)])
    return command


def run_all(
    *,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    roster_root: Path = DEFAULT_ROSTER_ROOT,
    workers: int = 32,
    timeout_seconds: float = 120.0,
    resume: bool = False,
    parts_zip: Path = DEFAULT_PARTS_ZIP,
    infinigen_source: Path = DEFAULT_INFINIGEN_SOURCE,
    slugs: Iterable[str] | None = None,
    dry_run: bool = False,
) -> dict[str, object]:
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    output_root = Path(output_root).resolve()
    roster_root = Path(roster_root).resolve()
    parts_zip = Path(parts_zip).resolve()
    infinigen_source = Path(infinigen_source).resolve()
    selected = _selected(slugs)
    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Path] = {}
    completed: list[str] = []
    for slug in selected:
        roster = roster_root / slug / "full_release_manifest.json"
        if not roster.is_file():
            raise FileNotFoundError(f"missing roster: {roster}")
        output = output_root / slug
        command = _command(
            slug,
            roster_root=roster_root,
            output_root=output_root,
            workers=workers,
            timeout_seconds=timeout_seconds,
            resume=resume,
            parts_zip=parts_zip,
            infinigen_source=infinigen_source,
        )
        log_path = output_root / f"{slug}.runner.log"
        output.mkdir(parents=True, exist_ok=True)
        if dry_run:
            print(json.dumps({"dataset": slug, "command": command}, ensure_ascii=True))
            continue
        with log_path.open("a", encoding="utf-8") as log:
            log.write("$ " + " ".join(command) + "\n")
            log.flush()
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
        if process.returncode != 0:
            raise RuntimeError(f"{slug} runner failed with exit code {process.returncode}; see {log_path}")
        outputs[slug] = output
        completed.append(slug)
    receipt_path: Path | None = None
    if not dry_run and set(completed) == set(selected):
        receipt_path = runner.write_combined_receipt(outputs, output_root)
    return {
        "output_root": str(output_root),
        "datasets": completed,
        "receipt": str(receipt_path) if receipt_path else None,
        "dry_run": dry_run,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--roster-root", type=Path, default=DEFAULT_ROSTER_ROOT)
    parser.add_argument("--workers", type=int, default=32)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--parts-zip", type=Path, default=DEFAULT_PARTS_ZIP)
    parser.add_argument("--infinigen-source", type=Path, default=DEFAULT_INFINIGEN_SOURCE)
    parser.add_argument("--dataset", dest="datasets", action="append", choices=runner.DATASETS)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_all(
            output_root=args.output_root,
            roster_root=args.roster_root,
            workers=args.workers,
            timeout_seconds=args.timeout_seconds,
            resume=args.resume,
            parts_zip=args.parts_zip,
            infinigen_source=args.infinigen_source,
            slugs=args.datasets,
            dry_run=args.dry_run,
        )
    except Exception as error:  # concise CLI failure; partial outputs remain resumable
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
