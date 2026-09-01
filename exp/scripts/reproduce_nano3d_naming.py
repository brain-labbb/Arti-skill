#!/usr/bin/env python3
"""Re-run Nano3D Naming into an independent exp-local directory and compare."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
HARNESS = EXP_ROOT / "scripts/run_nano3d_naming.py"
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_naming"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_naming_repro_v22"
FILES = [
    "summary.json",
    "asset_records.json",
    "cross_seed_records.json",
    "input_manifest.json",
    "judge_queue.jsonl",
    "report.md",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inside_exp(path: Path) -> bool:
    return path == EXP_ROOT or EXP_ROOT in path.parents


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    reference = args.reference.resolve()
    output = args.output.resolve()
    if not inside_exp(reference) or not inside_exp(output):
        raise RuntimeError(f"reference and output must remain under {EXP_ROOT}")
    if reference == output:
        raise RuntimeError("reproduction output must differ from reference")

    subprocess.run([sys.executable, str(HARNESS), "--output", str(output)], check=True)
    comparisons = []
    for name in FILES:
        expected = reference / name
        actual = output / name
        if not expected.exists() or not actual.exists():
            comparisons.append({"file": name, "match": False, "error": "missing file"})
            continue
        expected_hash = sha256(expected)
        actual_hash = sha256(actual)
        comparisons.append(
            {
                "file": name,
                "match": expected_hash == actual_hash,
                "reference_sha256": expected_hash,
                "reproduction_sha256": actual_hash,
            }
        )
    result = {
        "protocol": "nano3d_naming_reproduction_check_v2.2",
        "reference": str(reference),
        "reproduction": str(output),
        "all_files_match": all(row["match"] for row in comparisons),
        "comparisons": comparisons,
    }
    (output / "reproduction_check.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["all_files_match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
