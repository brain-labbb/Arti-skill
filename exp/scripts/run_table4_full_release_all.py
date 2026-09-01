#!/usr/bin/env python3
"""Run the generic Table 4 evaluator for all eight comparison releases.

Each dataset is written to its own directory under ``--output-root``.  The
datasets run sequentially so a large release cannot starve a smaller one and
each directory remains independently resumable.  At the end a self-contained
``full_release_receipt.json`` is emitted for the Markdown renderer.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

try:
    from . import run_table4_full_release as runner
except ImportError:
    import run_table4_full_release as runner  # type: ignore


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROSTER_ROOT = EXP_ROOT / "runtime/table123_full_release_20260825/rosters"
DEFAULT_OUTPUT_ROOT = EXP_ROOT / "runtime/table4_full_release_20260826"
DEFAULT_INFINIGEN_SOURCE = EXP_ROOT / "Infinigen-Sim"
DEFAULT_PARTS_ZIP = EXP_ROOT / "parts.zip"


def roster_path(root: Path, dataset: str) -> Path:
    path = root / dataset / "full_release_manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"roster missing for {dataset}: {path}")
    return path


def run_all(
    *,
    roster_root: Path = DEFAULT_ROSTER_ROOT,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    datasets: list[str] | None = None,
    workers: int = 1,
    timeout_seconds: float = 300.0,
    resume: bool = False,
    dry_run: bool = False,
    parts_zip: Path | None = None,
    infinigen_source: Path | None = None,
) -> dict[str, Path]:
    selected = datasets or list(runner.DATASETS)
    unknown = sorted(set(selected) - set(runner.DATASETS))
    if unknown:
        raise ValueError("unknown dataset(s): " + ", ".join(unknown))
    roster_root = Path(roster_root).resolve(strict=True)
    output_root = Path(output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Path] = {}
    for dataset in selected:
        roster = roster_path(roster_root, dataset)
        output = output_root / dataset
        if dry_run:
            print(json.dumps({"dataset": dataset, "roster": str(roster), "output": str(output)}, sort_keys=True))
            continue
        print(f"table4 starting {dataset}: N/J from {roster}", flush=True)
        outputs[dataset] = runner.run_dataset(
            roster,
            output,
            dataset=dataset,
            workers=workers,
            timeout_seconds=timeout_seconds,
            resume=resume,
            parts_zip=(parts_zip if dataset == "infinite" else None),
            source_root=(infinigen_source if dataset == "infinigen" else None),
        )
    if outputs:
        receipt = runner.write_combined_receipt(outputs, output_root)
        print(json.dumps({"receipt": str(receipt), "datasets": list(outputs)}, sort_keys=True))
    return outputs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", action="append", choices=runner.DATASETS)
    parser.add_argument("--roster-root", type=Path, default=DEFAULT_ROSTER_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--timeout-seconds", type=float, default=300.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--parts-zip", type=Path)
    parser.add_argument("--infinigen-source", type=Path, default=DEFAULT_INFINIGEN_SOURCE)
    args = parser.parse_args(argv)
    try:
        run_all(
            roster_root=args.roster_root,
            output_root=args.output_root,
            datasets=args.dataset,
            workers=args.workers,
            timeout_seconds=args.timeout_seconds,
            resume=args.resume,
            dry_run=args.dry_run,
            parts_zip=args.parts_zip or DEFAULT_PARTS_ZIP,
            infinigen_source=args.infinigen_source,
        )
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["run_all", "roster_path"]
