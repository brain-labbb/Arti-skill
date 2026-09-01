#!/usr/bin/env python3
"""Replay and byte-verify the direct-root PartNet-Mobility reference."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference"
DEFAULT_REPLAY = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference_replay"
RUNNER = Path(__file__).with_name("run_partnet_mobility_official_reference.py")


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selected_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): sha256_file(path)
        for path in sorted((root / "selected_files").glob("*/*/*"))
        if path.is_file()
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    args = parser.parse_args()
    reference = contained(args.reference)
    replay = contained(args.replay, exists=False)
    if replay.exists():
        raise FileExistsError(f"refusing to overwrite replay: {replay}")
    command = [
        sys.executable,
        str(RUNNER),
        "--output",
        str(replay),
        "--selection",
        str(reference / "frozen_selection.json"),
        "--reference-protocol",
        str(reference / "reference_protocol_snapshot.json"),
        "--ontology-protocol",
        str(reference / "ontology_protocol_snapshot.json"),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"replay failed\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}")
    comparable = (
        "frozen_selection.json",
        "reference_protocol_snapshot.json",
        "ontology_protocol_snapshot.json",
        "manifest.jsonl",
        "structure_records.jsonl",
        "urdf_name_only_records.jsonl",
        "package_semantics_assisted_records.jsonl",
        "root_imputed_sensitivity_records.jsonl",
        "package_semantics_labels.jsonl",
        "archive_continuity_records.jsonl",
        "provenance.json",
        "summary.json",
        "report.md",
        "verification.json",
    )
    comparisons: dict[str, dict[str, Any]] = {}
    for relative in comparable:
        first = sha256_file(reference / relative)
        second = sha256_file(replay / relative)
        comparisons[relative] = {"reference_sha256": first, "replay_sha256": second, "identical": first == second}
    reference_selected = selected_hashes(reference)
    replay_selected = selected_hashes(replay)
    checks = {
        "replay_exit_zero": completed.returncode == 0,
        "all_comparable_files_byte_identical": all(row["identical"] for row in comparisons.values()),
        "selected_files_byte_identical": reference_selected == replay_selected,
        "reference_verification_passed": json.loads((reference / "verification.json").read_text(encoding="utf-8"))["passed"] is True,
        "replay_verification_passed": json.loads((replay / "verification.json").read_text(encoding="utf-8"))["passed"] is True,
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "file_comparisons": comparisons,
        "selected_file_count": len(reference_selected),
        "reference": str(reference),
        "replay": str(replay),
        "runner_stdout": completed.stdout,
        "runner_stderr": completed.stderr,
    }
    destination = reference / "determinism_verification.json"
    destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "output": str(destination)}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
